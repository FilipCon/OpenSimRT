#include "AccelerationBasedPhaseDetector.h"
#include "IMUCalibrator.h"
#include "INIReader.h"
#include "InverseDynamics.h"
#include "InverseKinematics.h"
#include "Measure.h"
#include "MoticonReceiverFromFile.h"
#include "NGIMUData.h"
#include "NGIMUInputDriver.h"
#include "NGIMUInputFromFileDriver.h"
#include "OpenSimUtils.h"
#include "PositionTracker.h"
#include "Settings.h"
#include "SyncManager.h"
#include "Utils.h"
#include "Visualization.h"

#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/CoordinateAxis.h>
#include <SimTKcommon/internal/NTraits.h>
#include <SimTKcommon/internal/State.h>
#include <SimTKcommon/internal/UnitVec.h>
#include <Simulation/Model/Model.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <iterator>
#include <optional>
#include <simbody/internal/Visualizer.h>
#include <stdexcept>
#include <thread>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

void run() {
    INIReader ini(INI_FILE);
    auto section = "LOWER_BODY_NGIMU_OFFLINE";
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

    auto platform_offset = ini.getReal(section, "PLATFORM_OFFSET", 0.0);

    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");

    auto ngimuDataFile =
            subjectDir + ini.getString(section, "NGIMU_DATA_CSV", "");
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

    auto grfRightLogger = ExternalWrench::initializeLogger();
    auto grfLeftLogger = ExternalWrench::initializeLogger();

    // ngimu input data driver from file
    NGIMUInputFromFileDriver imuDriver(ngimuDataFile, 60);
    imuDriver.startListening();

    // calibrator
    IMUCalibrator clb(model, &imuDriver, imuObservationOrder);
    clb.recordNumOfSamples(1);
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

    double samplingRate = 60;
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

    // visualizer
    BasicModelVisualizer visualizer(model);
    auto rightGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.addDecorationGenerator(rightGRFDecorator);
    auto leftGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.addDecorationGenerator(leftGRFDecorator);

    try { // main loop
        while (true) {
            // get input from sensors
            auto imuDataFrame = imuDriver.getFrameAsVector();

            if ((imuDriver.exc_ptr != nullptr)) throw std::runtime_error("End");

            // synchronize data streams
            manager.appendPack(imuDataFrame);
            auto pack = manager.getPack();

            if (pack.second.empty()) continue;

            auto imuData =
                    make_pair(pack.first, imuDriver.fromVector(pack.second[0]));

            // solve ik
            auto pose = ik.solve(clb.transform(imuData, {}));

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
            visualizer.update(q);
            rightGRFDecorator->update(grfmOutput[0].point, grfmOutput[0].force);
            leftGRFDecorator->update(grfmOutput[1].point, grfmOutput[1].force);

            // log output
            grfRightLogger.appendRow(grfmOutput[0].t,
                                     ~grfmOutput[0].asVector());
            grfLeftLogger.appendRow(grfmOutput[1].t, ~grfmOutput[1].asVector());
        }
    } catch (std::exception& e) { cout << e.what() << endl; }

    // store results
    STOFileAdapter::write(qLogger,
                          subjectDir + "real_time/inverse_kinematics/q.sto");
    STOFileAdapter::write(
            grfRightLogger,
            subjectDir + "/real_time/grfm_prediction/wrench_right.sto");
    STOFileAdapter::write(grfLeftLogger,
                          subjectDir +
                                  "/real_time/grfm_prediction/wrench_left.sto");
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
