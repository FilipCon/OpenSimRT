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

    // marker tasks
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> markerObservationOrder;
    InverseKinematics::createMarkerTasksFromMarkerNames(
            model, vector<string>{}, markerTasks, markerObservationOrder);

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
    clb.computeHeadingRotation(imuBaseBody, imuDirectionAxis);
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
    detectorParameters.threshold = 100;
    detectorParameters.windowSize = 7;
    ExternalForceBasedPhaseDetector detector(detectorParameters);
    GRFMPrediction::Parameters grfmParameters;
    grfmParameters.method = "Newton-Euler";
    grfmParameters.pelvisBodyName = "pelvis";
    grfmParameters.rStationBodyName = "calcn_r";
    grfmParameters.lStationBodyName = "calcn_l";
    grfmParameters.rHeelStationLocation = SimTK::Vec3(0.014, -0.0168, -0.0055);
    grfmParameters.lHeelStationLocation = SimTK::Vec3(0.014, -0.0168, 0.0055);
    grfmParameters.rToeStationLocation = SimTK::Vec3(0.24, -0.0168, -0.00117);
    grfmParameters.lToeStationLocation = SimTK::Vec3(0.24, -0.0168, 0.00117);
    grfmParameters.directionWindowSize = 10;
    GRFMPrediction grfmPrediction(model, grfmParameters, &detector);
    auto grfRightLogger = ExternalWrench::initializeLogger();
    auto grfLeftLogger = ExternalWrench::initializeLogger();

    // sensor data synchronization
    double samplingRate = 20;
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
            auto pose = ik.solve(clb.transform(imuData, {}));

            // filter
            auto ikFiltered = ikFilter.filter({pose.t, pose.q});
            auto q = ikFiltered.x;
            auto qDot = ikFiltered.xDot;
            auto qDDot = ikFiltered.xDDot;

            if (!ikFiltered.isValid) continue;

            // // grfm prediction
            // detector.updDetector({moticonData.timestamp,
            //                       moticonData.right.totalForce,
            //                       moticonData.left.totalForce});
            // auto grfmOutput = grfmPrediction.solve({pose.t, q, qDot, qDDot});

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
                                      Vec3(0, moticonData.right.totalForce, 0));
            leftGRFDecorator->update(calcn_l_point_in_ground,
                                     Vec3(0, moticonData.left.totalForce, 0));

            // record
            qLogger.appendRow(pose.t, ~pose.q);
            imuLogger.appendRow(imuData.first,
                                ~driver.asVector(imuData.second));
            mtLogger.appendRow(moticonData.timestamp, ~moticonData.asVector());
            // grfRightLogger.appendRow(grfmOutput[0].t,
            //                          ~grfmOutput[0].asVector());
            // grfLeftLogger.appendRow(grfmOutput[1].t, ~grfmOutput[1].asVector());
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
        STOFileAdapter::write(
                grfRightLogger,
                subjectDir + "/real_time/grfm_prediction/wrench_right.sto");
        STOFileAdapter::write(
                grfLeftLogger,
                subjectDir + "/real_time/grfm_prediction/wrench_left.sto");
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
