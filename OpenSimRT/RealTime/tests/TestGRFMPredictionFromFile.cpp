#include "AccelerationBasedPhaseDetector.h"
#include "ContactForceBasedPhaseDetector.h"
#include "GRFMPrediction.h"
#include "GaitPhaseDetector.h"
#include "INIReader.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "SignalProcessing.h"
#include "Simulation.h"
#include "Utils.h"
#include "Visualization.h"

#include <OpenSim/Common/STOFileAdapter.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto section = "TEST_GRFM_PREDICTION_FROM_FILE";
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto resultsDir = DATA_DIR + ini.getString(section, "RESULTS_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto ikFile = subjectDir + ini.getString(section, "IK_FILE", "");
    auto grfMotFile = subjectDir + ini.getString(section, "GRF_MOT_FILE", "");

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

    auto platform_offset = ini.getReal(section, "PLATFORM_OFFSET", 0.0);

    auto memory = ini.getInteger(section, "MEMORY", 0);
    auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
    auto delay = ini.getInteger(section, "DELAY", 0);
    auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);

    // setup model
    Model model(modelFile);
    model.initSystem();

    // setup external forces
    Storage grfMotion(grfMotFile);

    // setup external forces parameters
    ExternalWrench::Parameters grfRightFootPar{
            grfRightApplyBody, grfRightForceExpressed, grfRightPointExpressed};
    auto grfRightLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
            grfRightPointIdentifier, grfRightForceIdentifier,
            grfRightTorqueIdentifier);
    auto grfRightLogger = ExternalWrench::initializeLogger();

    ExternalWrench::Parameters grfLeftFootPar{
            grfLeftApplyBody, grfLeftForceExpressed, grfLeftPointExpressed};
    auto grfLeftLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
            grfLeftPointIdentifier, grfLeftForceIdentifier,
            grfLeftTorqueIdentifier);
    auto grfLeftLogger = ExternalWrench::initializeLogger();

    vector<ExternalWrench::Parameters> wrenchParameters;
    wrenchParameters.push_back(grfRightFootPar);
    wrenchParameters.push_back(grfLeftFootPar);

    // get kinematics as a table with ordered coordinates
    auto qTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
            model, ikFile, 0.01);

    // setup filters
    LowPassSmoothFilter::Parameters ikFilterParam;
    ikFilterParam.numSignals = model.getNumCoordinates();
    ikFilterParam.memory = memory;
    ikFilterParam.delay = delay;
    ikFilterParam.cutoffFrequency = cutoffFreq;
    ikFilterParam.splineOrder = splineOrder;
    ikFilterParam.calculateDerivatives = true;
    LowPassSmoothFilter ikFilter(ikFilterParam);

    // // setup grfm prediction
    // ContactForceBasedPhaseDetector::Parameters detectorParameters;
    // detectorParameters.threshold = 100;
    // detectorParameters.windowSize = 2;
    // detectorParameters.plane_origin = Vec3(0.0, platform_offset, 0.0);
    // auto detector = ContactForceBasedPhaseDetector(model, detectorParameters);
    AccelerationBasedPhaseDetector::Parameters parameters;
    parameters.accThreshold = 4;
    parameters.velThreshold = 2.2;
    parameters.windowSize = 7;
    parameters.rFootBodyName = "calcn_r";
    parameters.lFootBodyName = "calcn_l";
    parameters.rHeelLocationInFoot = SimTK::Vec3(0.014, -0.0168, -0.0055);
    parameters.rToeLocationInFoot = SimTK::Vec3(0.24, -0.0168, -0.00117);
    parameters.lHeelLocationInFoot = SimTK::Vec3(0.014, -0.0168, 0.0055);
    parameters.lToeLocationInFoot = SimTK::Vec3(0.24, -0.0168, 0.00117);
    parameters.samplingFrequency = 60;
    parameters.accLPFilterFreq = 5;
    parameters.velLPFilterFreq = 5;
    parameters.posLPFilterFreq = 5;
    parameters.accLPFilterOrder = 1;
    parameters.velLPFilterOrder = 1;
    parameters.posLPFilterOrder = 1;
    parameters.posDiffOrder = 2;
    parameters.velDiffOrder = 2;
    AccelerationBasedPhaseDetector detector(model, parameters);
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
    GRFMPrediction grfm(model, grfmParameters, &detector);

    // initialize id and logger
    InverseDynamics id(model, wrenchParameters);
    auto tauLogger = id.initializeLogger();

    // visualizer
    BasicModelVisualizer visualizer(model);
    auto rightGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.addDecorationGenerator(rightGRFDecorator);
    auto leftGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.addDecorationGenerator(leftGRFDecorator);

    // loop through kinematic frames
    for (int i = 0; i < qTable.getNumRows(); i++) {
        // get raw pose from table
        double t = qTable.getIndependentColumn()[i];
        auto qRaw = qTable.getRowAtIndex(i).getAsVector();

        // filter
        auto ikFiltered = ikFilter.filter({t, qRaw});
        auto q = ikFiltered.x;
        auto qDot = ikFiltered.xDot;
        auto qDDot = ikFiltered.xDDot;

        if (!ikFiltered.isValid) { continue; }

        // // perform grfm prediction
        // detector.updDetector({ikFiltered.t, q, qDot, qDDot});
        // auto grfmOutput = grfm.solve({ikFiltered.t, q, qDot, qDDot});

        // // setup ID input
        // ExternalWrench::Input grfRightWrench = {
        //         grfmOutput[0].point, grfmOutput[0].force, grfmOutput[0].moment};
        // ExternalWrench::Input grfLeftWrench = {
        //         grfmOutput[1].point, grfmOutput[1].force, grfmOutput[1].moment};

        // // solve ID
        // auto idOutput = id.solve(
        //         {t, q, qDot, qDDot,
        //          vector<ExternalWrench::Input>{grfRightWrench, grfLeftWrench}});

                // get grf forces
        auto grfRightWrench = ExternalWrench::getWrenchFromStorage(
                t, grfRightLabels, grfMotion);
        auto grfLeftWrench = ExternalWrench::getWrenchFromStorage(
                t, grfLeftLabels, grfMotion);

        // visualization
        visualizer.update(q);
        // rightGRFDecorator->update(grfmOutput[0].point, grfmOutput[0].force);
        // leftGRFDecorator->update(grfmOutput[1].point, grfmOutput[1].force);

        rightGRFDecorator->update(grfRightWrench.point, grfRightWrench.force);
        leftGRFDecorator->update(grfLeftWrench.point, grfLeftWrench.force);

        // log data (use filter time to align with delay)
        // tauLogger.appendRow(ikFiltered.t, ~idOutput.tau);
        // grfRightLogger.appendRow(grfmOutput[0].t, ~grfmOutput[0].asVector());
        // grfLeftLogger.appendRow(grfmOutput[1].t, ~grfmOutput[1].asVector());

        std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }

    // // store results
    // STOFileAdapter::write(tauLogger, resultsDir + "tau.sto");
    // STOFileAdapter::write(grfRightLogger, resultsDir + "wrench_right.sto");
    // STOFileAdapter::write(grfLeftLogger, resultsDir + "wrench_left.sto");
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
