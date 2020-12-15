#include "AccelerationBasedPhaseDetector.h"
#include "IMUCalibrator.h"
#include "INIReader.h"
#include "MoticonReceiver.h"
#include "NGIMUData.h"
#include "NGIMUInputDriver.h"
#include "NGIMUInputFromFileDriver.h"
#include "RealTimeAnalysis.h"
#include "Settings.h"
#include "SyncManager.h"

#include <SimTKcommon/Scalar.h>
#include <SimTKcommon/internal/Rotation.h>
#include <iostream>
#include <optional>
#include <thread>
#include <vector>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

void run(char const* name) {
    INIReader ini(INI_FILE);
    auto section = "LOWER_BODY_NGIMU_OFFLINE";
    auto IMU_BODIES = ini.getVector(section, "IMU_BODIES", vector<string>());
    auto imuDirectionAxis = ini.getString(section, "IMU_DIRECTION_AXIS", "");
    auto imuBaseBody = ini.getString(section, "IMU_BASE_BODY", "");
    auto xGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_X", 0);
    auto yGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Y", 0);
    auto zGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Z", 0);
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");

    auto grfRightApplyBody =
            ini.getString(section, "GRF_RIGHT_APPLY_TO_BODY", "");
    auto grfRightForceExpressed =
            ini.getString(section, "GRF_RIGHT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfRightPointExpressed =
            ini.getString(section, "GRF_RIGHT_POINT_EXPRESSED_IN_BODY", "");
    auto grfLeftApplyBody =
            ini.getString(section, "GRF_LEFT_APPLY_TO_BODY", "");
    auto grfLeftForceExpressed =
            ini.getString(section, "GRF_LEFT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfLeftPointExpressed =
            ini.getString(section, "GRF_LEFT_POINT_EXPRESSED_IN_BODY", "");

    auto memory = ini.getInteger(section, "MEMORY", 0);
    auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
    auto delay = ini.getInteger(section, "DELAY", 0);
    auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);
    auto calcDer = ini.getBoolean(section, "CALC_DER", true);

    auto ngimuDataFile =
            subjectDir + ini.getString(section, "NGIMU_DATA_CSV", "");

    auto momentArmLibraryPath =
            ini.getString(section, "MOMENT_ARM_LIBRARY", "");

    int convergenceTolerance = ini.getReal(section, "TOLERANCE", 0);
    bool solveMuscleOptimization = ini.getBoolean(section, "SOLVE_SO", true);
    bool useVisualizer = ini.getBoolean(section, "USE_VISUALIZER", false);

    // =============================================================================

    if (sizeof(Real) != sizeof(double)) {
        THROW_EXCEPTION(
                "Real and double of different size: memcpy may not work.");
    }

    // setup model
    Model model(modelFile);
    auto state = model.initSystem();
    const int dofs = state.getNU();
    const int joints = model.getJointSet().getSize();
    const int muscles = model.getMuscles().getSize();

    // load and verify moment arm function
    auto calcMomentArm = OpenSimUtils::getMomentArmFromDynamicLibrary(
            model, momentArmLibraryPath);

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

    // setup external forces parameters
    ExternalWrench::Parameters grfRightFootPar{
            grfRightApplyBody, grfRightForceExpressed, grfRightPointExpressed};
    ExternalWrench::Parameters grfLeftFootPar{
            grfLeftApplyBody, grfLeftForceExpressed, grfLeftPointExpressed};
    vector<ExternalWrench::Parameters> wrenchParameters;
    wrenchParameters.push_back(grfRightFootPar);
    wrenchParameters.push_back(grfLeftFootPar);

    // ngimu input data driver from file
    NGIMUInputFromFileDriver imuDriver(ngimuDataFile, 60);
    imuDriver.startListening();

    // calibrator
    IMUCalibrator clb(model, &imuDriver, imuObservationOrder);
    clb.recordNumOfSamples(1);
    clb.setGroundOrientationSeq(xGroundRotDeg, yGroundRotDeg, zGroundRotDeg);
    clb.computeHeadingRotation(imuBaseBody, imuDirectionAxis);
    clb.calibrateIMUTasks(imuTasks);

    // sensor data synchronization
    double samplingRate = 40;
    double threshold = 0.0001;
    SyncManager manager(samplingRate, threshold);

    // acquisition function
    auto dataAcquisitionFunction = [&]() -> MotionCaptureInput {
        MotionCaptureInput input;

        // get input from sensors
        auto imuDataFrame = imuDriver.getFrameAsVector();

        if (imuDriver.exc_ptr != nullptr) throw std::runtime_error("End");

        // synchronize data streams
        manager.appendPack(imuDataFrame);
        auto pack = manager.getPack();

        std::pair<double, std::vector<NGIMUData>> imuData;
        if (!pack.second.empty()) {
            imuData = make_pair(pack.first,
                                imuDriver.fromVector(pack.second.at(0)));
            // calibrate imu data
            input.IkFrame = clb.transform(imuData, {});
        } else {
            input.IkFrame.t = -Infinity;
            input.IkFrame.imuObservations = SimTK::Array_<SimTK::Rotation>(
                    imuObservationOrder.size(), Rotation());
        }
        return input;
    };

    // initialize filter parameters
    LowPassSmoothFilterTS::Parameters filterParameters;
    filterParameters.numSignals = dofs;
    filterParameters.memory = memory;
    filterParameters.delay = delay;
    filterParameters.cutoffFrequency = cutoffFreq;
    filterParameters.splineOrder = splineOrder;
    filterParameters.calculateDerivatives = calcDer;

    // gait phase detector
    AccelerationBasedPhaseDetector::Parameters detectorParameters;
    detectorParameters.accThreshold = 7;
    detectorParameters.velThreshold = 1.9;
    detectorParameters.windowSize = 7;
    detectorParameters.rFootBodyName = "calcn_r";
    detectorParameters.lFootBodyName = "calcn_l";
    detectorParameters.rHeelLocationInFoot =
            SimTK::Vec3(0.014, -0.0168, -0.0055);
    detectorParameters.rToeLocationInFoot =
            SimTK::Vec3(0.24, -0.0168, -0.00117);
    detectorParameters.lHeelLocationInFoot =
            SimTK::Vec3(0.014, -0.0168, 0.0055);
    detectorParameters.lToeLocationInFoot = SimTK::Vec3(0.24, -0.0168, 0.00117);
    detectorParameters.samplingFrequency = 60;
    detectorParameters.accLPFilterFreq = 5;
    detectorParameters.velLPFilterFreq = 5;
    detectorParameters.posLPFilterFreq = 5;
    detectorParameters.accLPFilterOrder = 1;
    detectorParameters.velLPFilterOrder = 1;
    detectorParameters.posLPFilterOrder = 1;
    detectorParameters.posDiffOrder = 2;
    detectorParameters.velDiffOrder = 2;
    auto phaseDetector =
            new AccelerationBasedPhaseDetector(model, detectorParameters);

    // grfm parameters
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

    // pipeline
    RealTimeAnalysis::Parameters pipelineParameters;
    pipelineParameters.useVisualizer = useVisualizer;
    pipelineParameters.solveMuscleOptimization = solveMuscleOptimization;
    pipelineParameters.ikMarkerTasks = markerTasks;
    pipelineParameters.ikIMUTasks = imuTasks;
    pipelineParameters.ikConstraintsWeight = 100;
    pipelineParameters.ikAccuracy = 0.01;
    pipelineParameters.filterParameters = filterParameters;
    pipelineParameters.muscleOptimizationParameters.convergenceTolerance =
            convergenceTolerance;
    pipelineParameters.wrenchParameters = wrenchParameters;
    pipelineParameters.dataAcquisitionFunction = dataAcquisitionFunction;
    pipelineParameters.momentArmFunction = calcMomentArm;
    pipelineParameters.reactionForceOnBodies =
            vector<string>{"tibia_r", "talus_l"};
    pipelineParameters.useGRFMPrediction = true;
    pipelineParameters.detectorUpdateMethod =
            RealTimeAnalysis::PhaseDetectorUpdateMethod::INTERNAL;
    pipelineParameters.phaseDetector = phaseDetector;
    pipelineParameters.grfmParameters = grfmParameters;
    pipelineParameters.internalPhaseDetectorUpdateFunction =
            [&](const double& t, const SimTK::Vector& q,
                const SimTK::Vector& qd, const SimTK::Vector& qdd) {
                phaseDetector->updDetector({t, q, qd, qdd});
            };
    RealTimeAnalysis pipeline(model, pipelineParameters);

    // run pipeline
    thread acquisitionThread(&RealTimeAnalysis::acquisition, &pipeline);
    thread processingThread(&RealTimeAnalysis::processing, &pipeline);

    try {
        while (true) {
            if (pipeline.exceptionPtr != nullptr)
                std::rethrow_exception(pipeline.exceptionPtr);

            // thread-safe fetch of RT results
            const auto& results = pipeline.getResults();

        } // while loop
    } catch (const exception& e) {
        cout << "Main: exception caught: " << e.what() << '\n';
        acquisitionThread.join();
        processingThread.join();

        cout << "Writing results in file..." << endl;
        // pipeline.exportResults(subjectDir + "real_time/pipeline");
    }
}

int main(int argc, char* argv[]) {
    try {
        run(argc > 1 ? argv[1] : nullptr);
    } catch (exception& e) {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}
