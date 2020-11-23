#include "INIReader.h"
#include "InverseDynamics.h"
#include "RealTimeAnalysis.h"
#include "Settings.h"

#include <iostream>
#include <optional>
#include <thread>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

void run(char const* name) {
    INIReader ini(INI_FILE);
    auto section = "TEST_RT_PIPELINE_FROM_FILE";

    // subject data
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");

    // motion data
    auto trcFile = subjectDir + ini.getString(section, "TRC_FILE", "");
    auto ikTaskSetFile =
            subjectDir + ini.getString(section, "IK_TASK_SET_FILE", "");
    auto grfMotFile = subjectDir + ini.getString(section, "GRF_MOT_FILE", "");

    // grf body and identifier labels
    auto grfRightApplyBody =
            ini.getString(section, "GRF_RIGHT_APPLY_TO_BODY", "");
    auto grfRightForceExpressed =
            ini.getString(section, "GRF_RIGHT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfRightPointExpressed =
            ini.getString(section, "GRF_RIGHT_POINT_EXPRESSED_IN_BODY", "");
    auto grfRightPointIdentifier =
            ini.getString(section, "GRF_RIGHT_POINT_IDENTIFIER", "");
    auto grfRightForceIdentifier =
            ini.getString(section, "GRF_RIGHT_FORCE_IDENTIFIER", "");
    auto grfRightTorqueIdentifier =
            ini.getString(section, "GRF_RIGHT_TORQUE_IDENTIFIER", "");

    auto grfLeftApplyBody =
            ini.getString(section, "GRF_LEFT_APPLY_TO_BODY", "");
    auto grfLeftForceExpressed =
            ini.getString(section, "GRF_LEFT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfLeftPointExpressed =
            ini.getString(section, "GRF_LEFT_POINT_EXPRESSED_IN_BODY", "");
    auto grfLeftPointIdentifier =
            ini.getString(section, "GRF_LEFT_POINT_IDENTIFIER", "");
    auto grfLeftForceIdentifier =
            ini.getString(section, "GRF_LEFT_FORCE_IDENTIFIER", "");
    auto grfLeftTorqueIdentifier =
            ini.getString(section, "GRF_LEFT_TORQUE_IDENTIFIER", "");

    auto momentArmLibraryPath =
            ini.getString(section, "MOMENT_ARM_LIBRARY", "");

    auto numForcePlates = ini.getInteger(section, "NUM_OF_FORCEPLATES", 0);

    // Low pass filter parameters
    auto memory = ini.getInteger(section, "MEMORY", 0);
    auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
    auto delay = ini.getInteger(section, "DELAY", 0);
    auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);
    auto calcDer = ini.getBoolean(section, "CALC_DER", true);

    // so parameters
    int convergenceTolerance = ini.getReal(section, "TOLERANCE", 0);

    // other setting
    int lastLoopCount = ini.getInteger(section, "SIMULATION_LOOPS", 0);
    bool useVisualizer = ini.getBoolean(section, "USE_VISUALIZER", false);
    bool solveMuscleOptimization = ini.getBoolean(section, "SOLVE_SO", true);

    // =============================================================================

    if (sizeof(Real) != sizeof(double)) {
        THROW_EXCEPTION(
                "Real and double of different size: memcpy may not work.");
    }

    // prepare model
    Model model(modelFile);
    auto state = model.initSystem();
    const int dofs = state.getNU();
    const int joints = model.getJointSet().getSize();
    const int muscles = model.getMuscles().getSize();

    // load and verify moment arm function
    auto calcMomentArm = OpenSimUtils::getMomentArmFromDynamicLibrary(
            model, momentArmLibraryPath);

    // prepare marker tasks
    IKTaskSet ikTaskSet(ikTaskSetFile);
    MarkerData markerData(trcFile);
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> observationOrder;
    InverseKinematics::createMarkerTasksFromMarkerData(
            model, markerData, markerTasks, observationOrder);
    // InverseKinematics::createMarkerTasksFromIKTaskSet(
    //         model, ikTaskSet, markerTasks, observationOrder);

    // read external forces
    Storage grfMotion(grfMotFile);
    ExternalWrench::Parameters grfRightFootPar{
            grfRightApplyBody, grfRightForceExpressed, grfRightPointExpressed};
    auto grfRightLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
            grfRightPointIdentifier, grfRightForceIdentifier,
            grfRightTorqueIdentifier);
    ExternalWrench::Parameters grfLeftFootPar{
            grfLeftApplyBody, grfLeftForceExpressed, grfLeftPointExpressed};
    auto grfLeftLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
            grfLeftPointIdentifier, grfLeftForceIdentifier,
            grfLeftTorqueIdentifier);
    vector<ExternalWrench::Parameters> wrenchParameters;
    wrenchParameters.push_back(grfRightFootPar);
    wrenchParameters.push_back(grfLeftFootPar);

    // acquisition function (simulates acquisition from motion)
    auto dataAcquisitionFunction = [&]() -> MotionCaptureInput {
        static int i = 0;
        MotionCaptureInput input;

        // get frame data
        input.IkFrame = InverseKinematics::getFrameFromMarkerData(
                i, markerData, observationOrder, false);
        double t = input.IkFrame.t;

        // get grf force
        auto grfRightWrench = ExternalWrench::getWrenchFromStorage(
                t, grfRightLabels, grfMotion);
        auto grfLeftWrench = ExternalWrench::getWrenchFromStorage(
                t, grfLeftLabels, grfMotion);
        input.ExternalWrenches = {grfRightWrench, grfLeftWrench};

        // dummy delay to simulate real time
        this_thread::sleep_for(chrono::milliseconds(16));
        i++;
        return input;
    };

    // initialize filter parameters
    LowPassSmoothFilterTS::Parameters filterParameters;
    filterParameters.numSignals = dofs + 9 * numForcePlates;
    filterParameters.memory = memory;
    filterParameters.delay = delay;
    filterParameters.cutoffFrequency = cutoffFreq;
    filterParameters.splineOrder = splineOrder;
    filterParameters.calculateDerivatives = calcDer;

    // pipeline
    RealTimeAnalysis::Parameters pipelineParameters;
    pipelineParameters.useVisualizer = useVisualizer;
    pipelineParameters.solveMuscleOptimization = solveMuscleOptimization;
    pipelineParameters.ikMarkerTasks = markerTasks;
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
