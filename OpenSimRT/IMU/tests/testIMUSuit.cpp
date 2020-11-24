#include "AccelerationBasedPhaseDetector.h"
#include "IMUCalibrator.h"
#include "INIReader.h"
#include "InverseKinematics.h"
#include "NGIMUInputDriver.h"
#include "OpenSimUtils.h"
#include "PositionTracker.h"
#include "Settings.h"
#include "Simulation.h"
#include "SyncManager.h"
#include "Utils.h"
#include "Visualization.h"

#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/State.h>
#include <SimTKcommon/internal/negator.h>
#include <Simulation/Model/Model.h>
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
    auto IMU_IP = ini.getVector(section, "IMU_IP", vector<string>());
    auto LISTEN_IP = ini.getString(section, "LISTEN_IP", "0.0.0.0");
    auto SEND_PORTS = ini.getVector(section, "IMU_SEND_PORTS", vector<int>());
    auto LISTEN_PORTS =
            ini.getVector(section, "IMU_LISTEN_PORTS", vector<int>());
    auto IMU_BODIES = ini.getVector(section, "IMU_BODIES", vector<string>());
    auto imuDirectionAxis = ini.getString(section, "IMU_DIRECTION_AXIS", "");
    auto imuBaseBody = ini.getString(section, "IMU_BASE_BODY", "");
    auto xGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_X", 0);
    auto yGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Y", 0);
    auto zGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Z", 0);

    auto platform_offset = ini.getReal(section, "PLATFORM_OFFSET", 0.0);

    auto memory = ini.getInteger(section, "MEMORY", 0);
    auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
    auto delay = ini.getInteger(section, "DELAY", 0);
    auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);

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

    // add marker to pelvis center
    auto pelvisMarker =
            Marker("PelvisCenter", model.getBodySet().get("pelvis"), Vec3(0));
    model.addMarker(&pelvisMarker);

    State state = model.initSystem();
    model.realizePosition(state);

    model.realizePosition(state);
    auto height = model.getBodySet().get("pelvis").findStationLocationInGround(
            state, Vec3(0));

    // marker tasks
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> markerObservationOrder;
    InverseKinematics::createMarkerTasksFromMarkerNames(
            model, vector<string>{"PelvisCenter"},
            markerTasks, markerObservationOrder);

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

    // calibrator
    IMUCalibrator clb = IMUCalibrator(model, &driver, imuObservationOrder);
    clb.recordTime(3.0); // record for 3 seconds
    clb.setGroundOrientationSeq(xGroundRotDeg, yGroundRotDeg, zGroundRotDeg);
    clb.computeheadingRotation(imuBaseBody, imuDirectionAxis);
    clb.calibrateIMUTasks(imuTasks);

    double samplingRate = 40;
    double threshold = 0.0001;
    SyncManager manager(samplingRate, threshold);

    AccelerationBasedPhaseDetector::Parameters parameters;
    parameters.acc_threshold = 6;
    parameters.vel_threshold = 2;
    parameters.consecutive_values = 5;
    parameters.filterParameters.numSignals = 4 * SimTK::Vec3().size();
    parameters.filterParameters.memory = 20;
    parameters.filterParameters.delay = 5;
    parameters.filterParameters.cutoffFrequency = 1;
    parameters.filterParameters.splineOrder = 3;
    parameters.filterParameters.calculateDerivatives = true;
    AccelerationBasedPhaseDetector detector(model, parameters);
    GRFMPrediction grfmPrediction(model, GRFMPrediction::Parameters(),
                                  &detector);

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

            // synchronize data packets
            manager.appendPack(driver.asPairsOfVectors(imuDataFrame));
            auto pack = manager.getPack();

            if (pack.second.empty()) continue;

            std::pair<double, std::vector<NGIMUData>> imuData;
            imuData.first = pack.first;
            imuData.second = driver.fromVector(pack.second[0]);

            // solve ik
            auto pose = ik.solve(clb.transform(imuData, {height}));

            // filter
            auto ikFiltered = ikFilter.filter({pose.t, pose.q});
            auto q = ikFiltered.x;
            auto qDot = ikFiltered.xDot;
            auto qDDot = ikFiltered.xDDot;

            if (!ikFiltered.isValid) continue;

            // grfm prediction
            detector.updDetector({pose.t, q, qDot, qDDot});
            auto grfmOutput = grfmPrediction.solve({pose.t, q, qDot, qDDot});

            // visualize
            visualizer.update(pose.q);
            rightGRFDecorator->update(grfmOutput[0].point, grfmOutput[0].force);
            leftGRFDecorator->update(grfmOutput[1].point, grfmOutput[1].force);

            // record
            qLogger.appendRow(pose.t - initTime, ~pose.q);
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
