#include "IMUCalibrator.h"
#include "INIReader.h"
#include "InverseKinematics.h"
#include "MoticonReceiver.h"
#include "NGIMUInputDriver.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "Simulation.h"
#include "Visualization.h"

#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/State.h>
#include <Simulation/Model/Model.h>
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
    auto IMU_BODIES = ini.getVector(section, "IMU_BODIES", vector<string>());
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");

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

    // initialize ik (lower constraint weight and accuracy -> faster tracking)
    InverseKinematics ik(model, markerTasks, imuTasks, SimTK::Infinity, 1e-5);
    auto qLogger = ik.initializeLogger();

    // manager
    NGIMUInputDriver driver;
    driver.setupInput(imuObservationOrder,
                      vector<string>(LISTEN_PORTS.size(), LISTEN_IP),
                      LISTEN_PORTS);
    driver.setupTransmitters(IMU_IP, SEND_PORTS, LISTEN_IP, LISTEN_PORTS);
    auto imuLogger = driver.initializeLogger();

    // insole driver
    MoticonReceiver mt(LISTEN_IP, INSOLES_PORT);
    auto mtLogger = mt.initializeLogger();

    // calibrator
    IMUCalibrator clb = IMUCalibrator(model, &driver, imuObservationOrder);
    PositionTracker pt = PositionTracker(model, imuObservationOrder);

    // start listening
    thread listen(&NGIMUInputDriver::startListening, &driver);

    clb.run(3.0); // record for 3 seconds

    // visualizer
    BasicModelVisualizer visualizer(model);
    auto vectorDecorator = new ForceDecorator(Green, 0.1, 3);
    visualizer.addDecorationGenerator(vectorDecorator);

    TimeSeriesTable avp;
    vector<string> columnNames{"px", "py", "pz"};
    avp.setColumnLabels(columnNames);

    try { // main loop
        while (true) {
            // get input from imus
            auto imuDataFrame = driver.getFrame();
            auto handle = std::async(std::launch::async,
                                     &MoticonReceiver::receiveData, &mt);

            // estimate position from IMUs
            auto pos = pt.computePosition(imuDataFrame) + height;

            // Vec3 pos = Vec3(0,0,0) + height;

            // solve ik
            auto pose = ik.solve(
                    clb.transform(imuDataFrame, vector<SimTK::Vec3>{pos}));

            auto insoleDataFrame = handle.get();

            const auto& cop = insoleDataFrame.right.cop;
            const auto& force = insoleDataFrame.right.totalForce;

            // visualize
            visualizer.update(pose.q);
            Vec3 pelvisJoint;
            vectorDecorator->update(Vec3(cop[1], 0, cop[0]),
                                    force * Vec3(0, 1, 0));

            // record
            imuLogger.appendRow(pose.t, ~driver.asVector(imuDataFrame));
            // mtLogger.appendRow(insoleDataFrame.timestamp,
            //                    ~insoleDataFrame.asVector());
            qLogger.appendRow(pose.t, ~pose.q);
            avp.appendRow(pose.t, ~Vector(pos));
        }
    } catch (std::exception& e) {
        cout << e.what() << endl;
        driver.stopListening();
        listen.join();

        // store results
        STOFileAdapter::write(
                qLogger, subjectDir + "real_time/inverse_kinematics/q.sto");
        STOFileAdapter::write(imuLogger,
                              subjectDir + "experimental_data/ngimu_data.sto");
        CSVFileAdapter::write(imuLogger,
                              subjectDir + "experimental_data/ngimu_data.csv");
        CSVFileAdapter::write(avp,
                              subjectDir + "experimental_data/estimates.csv");
        // CSVFileAdapter::write(mtLogger,
        //                       subjectDir + "experimental_data/moticon.csv");
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
