#include "ExternalForceBasedPhaseDetector.h"
#include "IMUCalibrator.h"
#include "INIReader.h"
#include "MoticonReceiver.h"
#include "NGIMUInputDriver.h"
#include "OpenSimUtils.h"
#include "PositionTracker.h"
#include "Settings.h"
#include "Simulation.h"
#include "SyncManager.h"
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
#include <iomanip>
#include <iostream>
#include <simbody/internal/Visualizer.h>
#include <thread>
#include <utility>
#include <vector>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

#define TIMESTAMP_NOW()                                                        \
    static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(  \
                                std::chrono::high_resolution_clock::now()      \
                                        .time_since_epoch())                   \
                                .count()) /                                    \
            1000000000

void run() {
    INIReader ini(INI_FILE);
    auto section = "LOWER_BODY_NGIMU";
    auto IMU_IP = ini.getVector(section, "IMU_IP", vector<string>());
    auto LISTEN_IP = ini.getString(section, "LISTEN_IP", "0.0.0.0");
    auto SEND_PORTS = ini.getVector(section, "IMU_SEND_PORTS", vector<int>());
    auto LISTEN_PORTS =
            ini.getVector(section, "IMU_LISTEN_PORTS", vector<int>());
    auto INSOLES_PORT = ini.getInteger(section, "INSOLE_LISTEN_PORT", 0);
    auto INSOLE_SIZE = ini.getInteger(section, "INSOLE_SIZE", 0);
    auto IMU_BODIES = ini.getVector(section, "IMU_BODIES", vector<string>());
    auto imuDirectionAxis = ini.getString(section, "IMU_DIRECTION_AXIS", "");
    auto imuBaseBody = ini.getString(section, "IMU_BASE_BODY", "");
    auto xGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_X", 0);
    auto yGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Y", 0);
    auto zGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Z", 0);

    auto memory = ini.getInteger(section, "MEMORY", 0);
    auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
    auto delay = ini.getInteger(section, "DELAY", 0);
    auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);

    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");

    // initial timetag (as decimal) of simulation
    const auto initTime = TIMESTAMP_NOW();

    // setup model
    Model model(modelFile);

    // add marker to pelvis center
    auto pelvisMarker =
            Marker("PelvisCenter", model.getBodySet().get("pelvis"), Vec3(0));
    model.addMarker(&pelvisMarker);

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
    clb.recordTime(3); // record for 3 seconds
    clb.setGroundOrientationSeq(xGroundRotDeg, yGroundRotDeg, zGroundRotDeg);
    clb.computeheadingRotation(imuBaseBody, imuDirectionAxis);
    clb.calibrateIMUTasks(imuTasks);

    // initialize ik (lower constraint weight and accuracy -> faster tracking)
    InverseKinematics ik(model, markerTasks, imuTasks, SimTK::Infinity, 1e-5);
    auto qLogger = ik.initializeLogger();

    // setup filters
    LowPassSmoothFilter::Parameters ikFilterParam;
    ikFilterParam.numSignals = model.getNumCoordinates();
    ikFilterParam.memory = memory;
    ikFilterParam.delay = delay;
    ikFilterParam.cutoffFrequency = cutoffFreq;
    ikFilterParam.splineOrder = splineOrder;
    ikFilterParam.calculateDerivatives = true;
    LowPassSmoothFilter ikFilter(ikFilterParam);

    ExternalForceBasedPhaseDetector::Parameters detectorParameters;
    ExternalForceBasedPhaseDetector detector(detectorParameters);
    detectorParameters.threshold = 100;
    detectorParameters.consecutive_values = 5;
    GRFMPrediction grfmPrediction(model, GRFMPrediction::Parameters(),
                                  &detector);

    // sensor data synchronization
    double samplingRate = 30;
    double threshold = 0.0001;
    SyncManager manager(samplingRate, threshold);

    // visualizer
    BasicModelVisualizer visualizer(model);
    auto rightGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.addDecorationGenerator(rightGRFDecorator);
    auto leftGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.addDecorationGenerator(leftGRFDecorator);

    try { // main loop
        while (true) {
            // get input from imus
            auto imuDataFrame = driver.getFrame();
            auto insoleDataFrame = mt.receiveData();

            // change frame representation
            const auto imuDataAsPairs = driver.asPairsOfVectors(imuDataFrame);
            const auto insoleDataAsPairs = make_pair(
                    insoleDataFrame.timestamp, insoleDataFrame.asVector());

            // synchronize data streams
            manager.appendPack(imuDataAsPairs, insoleDataAsPairs);
            auto pack = manager.getPack();

            if (pack.second.empty()) continue;

            // retrieve original representation of input data
            std::pair<double, std::vector<NGIMUData>> imuData;
            imuData.first = pack.first;
            imuData.second = driver.fromVector(pack.second[0]);

            MoticonReceivedBundle moticonData;
            moticonData.timestamp = pack.first;
            moticonData.fromVector(pack.second[1]);

            // solve ik
            auto pose = ik.solve(clb.transform(imuData, {height}));

            // filter
            auto ikFiltered = ikFilter.filter({pose.t, pose.q});
            auto q = ikFiltered.x;
            auto qDot = ikFiltered.xDot;
            auto qDDot = ikFiltered.xDDot;

            if (!ikFiltered.isValid) continue;

            // grfm prediction
            detector.updDetector({moticonData.timestamp,
                                  moticonData.right.totalForce,
                                  moticonData.left.totalForce});
            auto grfmOutput = grfmPrediction.solve({pose.t, q, qDot, qDDot});

            // visualize
            const auto& r_cop = moticonData.right.cop;
            const auto& l_cop = moticonData.left.cop;
            Vec3 calcn_r_point_in_ground;
            Vec3 calcn_l_point_in_ground;
            visualizer.expressPositionInGround("calcn_r_insole",
                                               Vec3(r_cop[0], 0, -r_cop[1]),
                                               calcn_r_point_in_ground);
            visualizer.expressPositionInGround("calcn_l_insole",
                                               Vec3(l_cop[0], 0, -l_cop[1]),
                                               calcn_l_point_in_ground);
            visualizer.update(pose.q);
            rightGRFDecorator->update(calcn_r_point_in_ground,
                                      grfmOutput[0].force);
            leftGRFDecorator->update(calcn_l_point_in_ground,
                                     grfmOutput[1].force);

            // record
            qLogger.appendRow(pose.t - initTime, ~pose.q);
            imuLogger.appendRow(imuData.first - initTime,
                                ~driver.asVector(imuData.second));
            mtLogger.appendRow(moticonData.timestamp - initTime,
                               ~moticonData.asVector());
        }
    } catch (std::exception& e) {
        cout << e.what() << endl;
        driver.stopListening();
        listen.join();

        // store results
        STOFileAdapter::write(
                qLogger, subjectDir + "real_time/inverse_kinematics/q.sto");
        CSVFileAdapter::write(imuLogger,
                              subjectDir + "experimental_data/ngimu_data.csv");
        CSVFileAdapter::write(mtLogger,
                              subjectDir + "experimental_data/moticon.csv");
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
