#include "AccelerationBasedPhaseDetector.h"
#include "IMUCalibrator.h"
#include "INIReader.h"
#include "InverseDynamics.h"
#include "MoticonReceiver.h"
#include "NGIMUInputDriver.h"
#include "RealTimeAnalysis.h"
#include "Settings.h"
#include "SyncManager.h"

#include <OpenSim/Common/CSVFileAdapter.h>
#include <iostream>
#include <optional>
#include <thread>

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

void run(char const* name) {
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

    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");

    auto imuOutputFile = ini.getString(section, "IMU_SAVE_FILE", "");

    auto momentArmLibraryPath =
            ini.getString(section, "MOMENT_ARM_LIBRARY", "");

    int convergenceTolerance = ini.getReal(section, "TOLERANCE", 0);

    // other setting
    bool useVisualizer = ini.getBoolean(section, "USE_VISUALIZER", false);
    bool solveMuscleOptimization = ini.getBoolean(section, "SOLVE_SO", true);

    // =============================================================================

    // initial timetag (as decimal) of simulation
    const auto initTime = TIMESTAMP_NOW();

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

    // driver
    NGIMUInputDriver imuDriver;
    imuDriver.setupInput(imuObservationOrder,
                         vector<string>(LISTEN_PORTS.size(), LISTEN_IP),
                         LISTEN_PORTS);
    imuDriver.setupTransmitters(IMU_IP, SEND_PORTS, LISTEN_IP, LISTEN_PORTS);
    thread listen(&NGIMUInputDriver::startListening, &imuDriver);
    auto imuLogger = imuDriver.initializeLogger();

    // calibrator
    IMUCalibrator clb = IMUCalibrator(model, &imuDriver, imuObservationOrder);
    clb.recordTime(3); // record for 3 seconds
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

        // get input from imus
        auto imuDataFrame = imuDriver.getFrame();

        // change frame representation
        const auto imuDataAsPairs = imuDriver.asPairsOfVectors(imuDataFrame);

        // synchronize data streams
        manager.appendPack(imuDataAsPairs);
        auto pack = manager.getPack();

        // retrieve original representation of input data
        std::pair<double, std::vector<NGIMUData>> imuData;
        if (!pack.second.empty()) {
            imuData = make_pair(pack.first,
                                imuDriver.fromVector(pack.second.at(0)));
            // calibrate imu data
            input.IkFrame = clb.transform(imuData, {});

            // record
            imuLogger.appendRow(imuData.first,
                                ~imuDriver.asVector(imuData.second));
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
    // detectorParameters.accThreshold = 7;
    detectorParameters.windowSize = 7;
    detectorParameters.rFootBodyName = "calcn_r";
    detectorParameters.lFootBodyName = "calcn_l";
    // detectorParameters.rLocationInFoot = SimTK::Vec3(0.24, -0.0168, -0.00117);
    // detectorParameters.lLocationInFoot = SimTK::Vec3(0.24, -0.0168, 0.00117);
    // detectorParameters.samplingFrequency = 60;
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

            const auto& results =
                    pipeline.getResults(); // thread-safe fetch of RT results

        } // while loop
    } catch (const exception& e) {
        cout << "Main: exception caught: " << e.what() << '\n';
        acquisitionThread.join();
        processingThread.join();

        cout << "Writing results in file..." << endl;
        CSVFileAdapter::write(imuLogger, subjectDir + imuOutputFile);
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
