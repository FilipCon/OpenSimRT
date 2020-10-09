#include "IMUCalibrator.h"
#include "INIReader.h"
#include "InverseKinematics.h"
#include "MoticonReceiver.h"
#include "NGIMUInputDriver.h"
#include "OpenSimUtils.h"
#include "PositionTracker.h"
#include "Settings.h"
#include "Simulation.h"
#include "Visualization.h"

#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/State.h>
#include <Simulation/Model/Model.h>
#include <bits/stdint-uintn.h>
#include <chrono>
#include <cmath>
#include <exception>
#include <future>
#include <iostream>
#include <simbody/internal/Visualizer.h>
#include <thread>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

void run() {
    INIReader ini(INI_FILE);
    auto section = "LOWER_BODY_NGIMU";
    // auto section = "UPPER_LIMB_NGIMU";
    auto IMU_IP = ini.getVector(section, "IMU_IP", vector<string>());
    auto LISTEN_IP = ini.getString(section, "LISTEN_IP", "0.0.0.0");
    auto SEND_PORTS = ini.getVector(section, "IMU_SEND_PORTS", vector<int>());
    auto LISTEN_PORTS =
            ini.getVector(section, "IMU_LISTEN_PORTS", vector<int>());
    auto INSOLES_PORT = ini.getInteger(section, "INSOLE_LISTEN_PORT", 0);
    auto INSOLE_SIZE = ini.getInteger(section, "INSOLE_SIZE", 0);
    auto IMU_BODIES = ini.getVector(section, "IMU_BODIES", vector<string>());
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");

    // initial timetag (as decimal) of simulation
    const auto initTime =
            static_cast<double>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::high_resolution_clock::now()
                                    .time_since_epoch())
                            .count()) /
            1000000000;

    // setup model
    Model model(modelFile);

    // add marker to pelvis center to track model position from imus
    auto pelvisMarker =
            Marker("PelvisCenter", model.getBodySet().get("pelvis"), Vec3(0));
    model.addMarker(&pelvisMarker);
    model.finalizeConnections();

    State state = model.initSystem();
    model.realizePosition(state);
    auto height = model.getBodySet().get("pelvis").findStationLocationInGround(
            state, Vec3(0));

    // marker tasks
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> markerObservationOrder;
    InverseKinematics::createMarkerTasksFromMarkerNames(
            model, vector<string>{"PelvisCenter"}, markerTasks,
            markerObservationOrder);

    // imu tasks
    vector<InverseKinematics::IMUTask> imuTasks;
    vector<string> imuObservationOrder(IMU_BODIES);
    InverseKinematics::createIMUTasksFromObservationOrder(
            model, imuObservationOrder, imuTasks);

    // driver
    NGIMUInputDriver driver;
    driver.setupInput(imuObservationOrder,
                      vector<string>(LISTEN_PORTS.size(), LISTEN_IP),
                      LISTEN_PORTS);
    driver.setupTransmitters(IMU_IP, SEND_PORTS, LISTEN_IP, LISTEN_PORTS);
    thread listen(&NGIMUInputDriver::startListening, &driver);
    auto imuLogger = driver.initializeLogger();

    // insole driver
    MoticonReceiver mt(LISTEN_IP, INSOLES_PORT);
    mt.setInsoleSize(INSOLE_SIZE);
    auto mtLogger = mt.initializeLogger();

    // calibrator
    IMUCalibrator clb = IMUCalibrator(model, &driver, imuObservationOrder);
    clb.record(3); // record for 3 seconds
    clb.computeheadingRotation("pelvis", SimTK::CoordinateDirection(SimTK::ZAxis, 1));
    clb.calibrateIMUTasks(imuTasks);

    // initialize ik (lower constraint weight and accuracy -> faster tracking)
    InverseKinematics ik(model, markerTasks, imuTasks, SimTK::Infinity, 1e-5);
    auto qLogger = ik.initializeLogger();

    // // position tracking
    // PositionTracker pt = PositionTracker(model, imuObservationOrder);

    // visualizer
    BasicModelVisualizer visualizer(model);
    auto rightInsoleDecorator = new ForceDecorator(Green, 0.01, 3);
    visualizer.addDecorationGenerator(rightInsoleDecorator);
    auto leftInsoleDecorator = new ForceDecorator(Green, 0.01, 3);
    visualizer.addDecorationGenerator(leftInsoleDecorator);

    TimeSeriesTable avp;
    vector<string> columnNames{"px", "py", "pz"};
    avp.setColumnLabels(columnNames);

    try { // main loop
        while (true) {
            // get input from imus
            auto imuDataFrame = driver.getFrame();
            auto handle = std::async(std::launch::async,
                                     &MoticonReceiver::receiveData, &mt);

            // // estimate position from IMUs
            // auto pos = pt.computePosition(imuDataFrame) + height;

            Vec3 pos = Vec3(0, 0, 0) + height;

            // solve ik
            auto pose = ik.solve(
                    clb.transform(imuDataFrame, vector<SimTK::Vec3>{pos}));

            auto insoleDataFrame = handle.get();

            const auto& r_cop = insoleDataFrame.right.cop;
            const auto& r_force = insoleDataFrame.right.totalForce;
            const auto& l_cop = insoleDataFrame.left.cop;
            const auto& l_force = insoleDataFrame.left.totalForce;

            // visualize
            visualizer.update(pose.q);
            Vec3 calcn_r_point_in_ground;
            Vec3 calcn_l_point_in_ground;
            visualizer.expressPositionInGround("calcn_r_insole",
                                               Vec3(r_cop[0], 0, -r_cop[1]),
                                               calcn_r_point_in_ground);
            visualizer.expressPositionInGround("calcn_l_insole",
                                               Vec3(l_cop[0], 0, -l_cop[1]),
                                               calcn_l_point_in_ground);
            rightInsoleDecorator->update(calcn_r_point_in_ground,
                                         r_force * Vec3(0, 1, 0));
            leftInsoleDecorator->update(calcn_l_point_in_ground,
                                        l_force * Vec3(0, 1, 0));

            // record
            qLogger.appendRow(pose.t - initTime, ~pose.q);
            imuLogger.appendRow(imuDataFrame.first - initTime,
                                ~driver.asVector(imuDataFrame));
            mtLogger.appendRow(insoleDataFrame.timestamp - initTime,
                               ~insoleDataFrame.asVector());
            // avp.appendRow(pose.t - initTime, ~Vector(pos));
        }
    } catch (std::exception& e) {
        cout << e.what() << endl;
        driver.stopListening();
        listen.join();

        // store results
        STOFileAdapter::write(
                qLogger, subjectDir + "real_time/inverse_kinematics/q.sto");
        CSVFileAdapter::write(
                imuLogger, subjectDir + "experimental_data/ngimu_data.csv");
        CSVFileAdapter::write(mtLogger,
                              subjectDir + "experimental_data/moticon.csv");
        // CSVFileAdapter::write(avp,
        //                       subjectDir +
        //                       "experimental_data/estimates.csv");
    }
}

int main(int argc, char* argv[]) {
    try {
        run();
    } catch (exception& e) {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}
