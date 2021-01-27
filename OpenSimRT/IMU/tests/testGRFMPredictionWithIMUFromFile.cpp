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

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

void run() {
    INIReader ini(INI_FILE);
    auto section = "TEST_GRFM_PREDICTION_FROM_FILE";
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

    // acceleration-based detector parameters
    auto heelAccThreshold = ini.getReal(section, "HEEL_ACC_THRESHOLD", 0);
    auto toeAccThreshold = ini.getReal(section, "TOE_ACC_THRESHOLD", 0);
    auto consecutiveValues = ini.getReal(section, "CONSECUTIVE_VALUES", 0);
    auto windowSize = ini.getInteger(section, "WINDOW_SIZE", 0);
    auto rFootBodyName = ini.getString(section, "RIGHT_FOOT_BODY_NAME", "");
    auto lFootBodyName = ini.getString(section, "LEFT_FOOT_BODY_NAME", "");
    auto rHeelLocation =
            ini.getSimtkVec(section, "RIGHT_HEEL_LOCATION_IN_FOOT", Vec3(0));
    auto lHeelLocation =
            ini.getSimtkVec(section, "LEFT_HEEL_LOCATION_IN_FOOT", Vec3(0));
    auto rToeLocation =
            ini.getSimtkVec(section, "RIGHT_TOE_LOCATION_IN_FOOT", Vec3(0));
    auto lToeLocation =
            ini.getSimtkVec(section, "LEFT_TOE_LOCATION_IN_FOOT", Vec3(0));
    auto samplingFrequency = ini.getInteger(section, "SAMPLING_FREQUENCY", 0);
    auto accLPFilterFreq = ini.getInteger(section, "ACC_LP_FILTER_FREQ", 0);
    auto velLPFilterFreq = ini.getInteger(section, "VEL_LP_FILTER_FREQ", 0);
    auto posLPFilterFreq = ini.getInteger(section, "POS_LP_FILTER_FREQ", 0);
    auto accLPFilterOrder = ini.getInteger(section, "ACC_LP_FILTER_ORDER", 0);
    auto velLPFilterOrder = ini.getInteger(section, "VEL_LP_FILTER_ORDER", 0);
    auto posLPFilterOrder = ini.getInteger(section, "POS_LP_FILTER_ORDER", 0);
    auto posDiffOrder = ini.getInteger(section, "POS_DIFF_ORDER", 0);
    auto velDiffOrder = ini.getInteger(section, "VEL_DIFF_ORDER", 0);

    // grfm parameters
    auto grfmMethod = ini.getString(section, "METHOD", "");
    auto pelvisBodyName = ini.getString(section, "PELVIS_BODY_NAME", "");
    auto rHeelCoPLocation =
            ini.getSimtkVec(section, "RIGHT_HEEL_STATION_LOCATION", Vec3(0));
    auto lHeelCoPLocation =
            ini.getSimtkVec(section, "LEFT_HEEL_STATION_LOCATION", Vec3(0));
    auto rToeCoPLocation =
            ini.getSimtkVec(section, "RIGHT_TOE_STATION_LOCATION", Vec3(0));
    auto lToeCoPLocation =
            ini.getSimtkVec(section, "LEFT_TOE_STATION_LOCATION", Vec3(0));
    auto directionWindowSize =
            ini.getInteger(section, "DIRECTION_WINDOW_SIZE", 0);

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

    // ngimu input data driver from file
    NGIMUInputFromFileDriver imuDriver(ngimuDataFile, 60);
    imuDriver.startListening();

    // calibrator
    IMUCalibrator clb(model, &imuDriver, imuObservationOrder);
    clb.recordNumOfSamples(1);
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

    double samplingRate = 60;
    double threshold = 0.0001;
    SyncManager manager(samplingRate, threshold);

    AccelerationBasedPhaseDetector::Parameters parameters;
    parameters.heelAccThreshold = heelAccThreshold;
    parameters.toeAccThreshold = toeAccThreshold;
    parameters.windowSize = windowSize;
    parameters.consecutiveValues = consecutiveValues;
    parameters.rFootBodyName = rFootBodyName;
    parameters.lFootBodyName = lFootBodyName;
    parameters.rHeelLocationInFoot = rHeelLocation;
    parameters.lHeelLocationInFoot = lHeelLocation;
    parameters.rToeLocationInFoot = rToeLocation;
    parameters.lToeLocationInFoot = lToeLocation;
    parameters.samplingFrequency = samplingFrequency;
    parameters.accLPFilterFreq = accLPFilterFreq;
    parameters.velLPFilterFreq = velLPFilterFreq;
    parameters.posLPFilterFreq = posLPFilterFreq;
    parameters.accLPFilterOrder = accLPFilterOrder;
    parameters.velLPFilterOrder = velLPFilterOrder;
    parameters.posLPFilterOrder = posLPFilterOrder;
    parameters.posDiffOrder = posDiffOrder;
    parameters.velDiffOrder = velDiffOrder;
    AccelerationBasedPhaseDetector detector(model, parameters);
    GRFMPrediction::Parameters grfmParameters;
    grfmParameters.method = grfmMethod;
    grfmParameters.pelvisBodyName = pelvisBodyName;
    grfmParameters.rStationBodyName = rFootBodyName;
    grfmParameters.lStationBodyName = lFootBodyName;
    grfmParameters.rHeelStationLocation = rHeelCoPLocation;
    grfmParameters.lHeelStationLocation = lHeelCoPLocation;
    grfmParameters.rToeStationLocation = rToeCoPLocation;
    grfmParameters.lToeStationLocation = lToeCoPLocation;
    grfmParameters.directionWindowSize = directionWindowSize;
    GRFMPrediction grfmPrediction(model, grfmParameters, &detector);
    auto grfRightLogger = ExternalWrench::initializeLogger();
    auto grfLeftLogger = ExternalWrench::initializeLogger();

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
