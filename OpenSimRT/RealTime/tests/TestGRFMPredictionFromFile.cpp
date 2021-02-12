#include "AccelerationBasedPhaseDetector.h"
#include "ContactForceBasedPhaseDetector.h"
#include "GRFMPrediction.h"
#include "INIReader.h"
#include "Measure.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "SignalProcessing.h"
#include "Simulation.h"
#include "Utils.h"
#include "Visualization.h"

#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <SimTKcommon/SmallMatrix.h>
#include <Simulation/SimbodyEngine/WeldJoint.h>
#include <iostream>

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

    // virtual contact surface as ground
    auto platform_offset = ini.getReal(section, "PLATFORM_OFFSET", 0.0);

    auto removeNLastRows =
            ini.getInteger(section, "REMOVE_N_LAST_TABLE_ROWS", 0);
    auto simulationLoops = ini.getInteger(section, "SIMULATION_LOOPS", 0);

    // rt filter
    auto memory = ini.getInteger(section, "MEMORY", 0);
    auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
    auto delay = ini.getInteger(section, "DELAY", 0);
    auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);

    // contact-force-based detector parameters
    auto threshold = ini.getReal(section, "THRESHOLD", 0);
    auto rHeelSphereLocation =
            ini.getSimtkVec(section, "RIGHT_HEEL_SPHERE_LOCATION", Vec3(0));
    auto lHeelSphereLocation =
            ini.getSimtkVec(section, "LEFT_HEEL_SPHERE_LOCATION", Vec3(0));
    auto rToeSphereLocation =
            ini.getSimtkVec(section, "RIGHT_TOE_SPHERE_LOCATION", Vec3(0));
    auto lToeSphereLocation =
            ini.getSimtkVec(section, "LEFT_TOE_SPHERE_LOCATION", Vec3(0));
    auto contactSphereRadius = ini.getReal(section, "SPHERE_RADIUS", 0);

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

    // setup model
    Model model(modelFile);
    model.initSystem();

    // setup external forces
    Storage grfMotion(grfMotFile);
    grfMotion.resampleLinear(0.01);

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

    // remove last rows
    for (int i = 0; i < removeNLastRows; ++i)
        qTable.removeRow(qTable.getIndependentColumn().back());

    // setup filters
    LowPassSmoothFilter::Parameters ikFilterParam;
    ikFilterParam.numSignals = model.getNumCoordinates();
    ikFilterParam.memory = memory;
    ikFilterParam.delay = delay;
    ikFilterParam.cutoffFrequency = cutoffFreq;
    ikFilterParam.splineOrder = splineOrder;
    ikFilterParam.calculateDerivatives = true;
    LowPassSmoothFilter ikFilter(ikFilterParam);

    LowPassSmoothFilter::Parameters grfFilterParam;
    grfFilterParam.numSignals = 9;
    grfFilterParam.memory = memory;
    grfFilterParam.delay = delay;
    grfFilterParam.cutoffFrequency = cutoffFreq;
    grfFilterParam.splineOrder = splineOrder;
    grfFilterParam.calculateDerivatives = false;
    LowPassSmoothFilter grfRightFilter(grfFilterParam),
            grfLeftFilter(grfFilterParam);

    // setup grfm prediction
    ContactForceBasedPhaseDetector::Parameters detectorParameters;
    detectorParameters.threshold = threshold;
    detectorParameters.windowSize = windowSize;
    detectorParameters.plane_origin = Vec3(0.0, platform_offset, 0.0);
    detectorParameters.rHeelSphereLocation = rHeelSphereLocation;
    detectorParameters.lHeelSphereLocation = lHeelSphereLocation;
    detectorParameters.rToeSphereLocation = rToeSphereLocation;
    detectorParameters.lToeSphereLocation = lToeSphereLocation;
    detectorParameters.sphereRadius = contactSphereRadius;
    detectorParameters.rFootBodyName = rFootBodyName;
    detectorParameters.lFootBodyName = lFootBodyName;
    auto detector = ContactForceBasedPhaseDetector(model, detectorParameters);
    // AccelerationBasedPhaseDetector::Parameters parameters;
    // parameters.heelAccThreshold = heelAccThreshold;
    // parameters.toeAccThreshold = toeAccThreshold;
    // parameters.windowSize = windowSize;
    // parameters.consecutiveValues = consecutiveValues;
    // parameters.rFootBodyName = rFootBodyName;
    // parameters.lFootBodyName = lFootBodyName;
    // parameters.rHeelLocationInFoot = rHeelLocation;
    // parameters.lHeelLocationInFoot = lHeelLocation;
    // parameters.rToeLocationInFoot = rToeLocation;
    // parameters.lToeLocationInFoot = lToeLocation;
    // parameters.samplingFrequency = samplingFrequency;
    // parameters.accLPFilterFreq = accLPFilterFreq;
    // parameters.velLPFilterFreq = velLPFilterFreq;
    // parameters.posLPFilterFreq = posLPFilterFreq;
    // parameters.accLPFilterOrder = accLPFilterOrder;
    // parameters.velLPFilterOrder = velLPFilterOrder;
    // parameters.posLPFilterOrder = posLPFilterOrder;
    // parameters.posDiffOrder = posDiffOrder;
    // parameters.velDiffOrder = velDiffOrder;
    // AccelerationBasedPhaseDetector detector(model, parameters);
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
    GRFMPrediction grfm(model, grfmParameters, &detector);

    // initialize id and logger
    InverseDynamics id(model, wrenchParameters);
    auto tauLogger = id.initializeLogger();

    // // // visualizer
    // BasicModelVisualizer visualizer(model);
    // auto rightGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    // visualizer.addDecorationGenerator(rightGRFDecorator);
    // auto leftGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    // visualizer.addDecorationGenerator(leftGRFDecorator);

    // mean delay
    vector<int> sumDelayMS;

    int loopCounter = 0;
    int i = 0;
    for (int k = 0; k < qTable.getNumRows() * simulationLoops; k++) {
        // get raw pose from table
        auto qRaw = qTable.getRowAtIndex(i).getAsVector();
        double t = qTable.getIndependentColumn()[i];

        // get gt grf forces
        auto grfRightWrenchGT = ExternalWrench::getWrenchFromStorage(
                t, grfRightLabels, grfMotion);
        auto grfLeftWrenchGT = ExternalWrench::getWrenchFromStorage(
                t, grfLeftLabels, grfMotion);

        t += loopCounter * (qTable.getIndependentColumn().back() + 0.01);

        // filter
        auto ikFiltered = ikFilter.filter({t, qRaw});
        auto q = ikFiltered.x;
        auto qDot = ikFiltered.xDot;
        auto qDDot = ikFiltered.xDDot;

        auto grfRightFiltered =
                grfRightFilter.filter({t, grfRightWrenchGT.toVector()});
        grfRightWrenchGT.fromVector(grfRightFiltered.x);
        auto grfLeftFiltered =
                grfLeftFilter.filter({t, grfLeftWrenchGT.toVector()});
        grfLeftWrenchGT.fromVector(grfLeftFiltered.x);

        // increment and reset loop and frame counters, respectively
        if (++i == qTable.getNumRows()) {
            i = 0;
            loopCounter++;
        }
        if (!ikFiltered.isValid || !grfRightFiltered.isValid ||
            !grfLeftFiltered.isValid) {
            continue;
        }

        chrono::high_resolution_clock::time_point t1;
        t1 = chrono::high_resolution_clock::now();

        // perform grfm prediction
        detector.updDetector({ikFiltered.t, q, qDot, qDDot});
        auto grfmOutput = grfm.solve({ikFiltered.t, q, qDot, qDDot});

        chrono::high_resolution_clock::time_point t2;
        t2 = chrono::high_resolution_clock::now();
        sumDelayMS.push_back(
                chrono::duration_cast<chrono::milliseconds>(t2 - t1).count());

        // project on plane
        grfmOutput[0].point = projectionOnPlane(
                grfmOutput[0].point, Vec3(0, -0.0075, 0), Vec3(0, 1, 0));
        grfmOutput[1].point = projectionOnPlane(
                grfmOutput[1].point, Vec3(0, -0.0075, 0), Vec3(0, 1, 0));

        // setup ID inputn
        ExternalWrench::Input grfRightWrench = {grfmOutput[0].point,
                                                grfRightWrenchGT.force,
                                                grfmOutput[0].moment};
        ExternalWrench::Input grfLeftWrench = {grfmOutput[1].point,
                                               grfLeftWrenchGT.force,
                                               grfmOutput[1].moment};

        // solve ID
        auto idOutput = id.solve(
                {ikFiltered.t, q, qDot, qDDot,
                 vector<ExternalWrench::Input>{grfRightWrench, grfLeftWrench}});

        // // visualization
        // visualizer.update(q);
        // rightGRFDecorator->update(grfmOutput[0].point, grfmOutput[0].force);
        // leftGRFDecorator->update(grfmOutput[1].point, grfmOutput[1].force);

        // log data (use filter time to align with delay)
        grfRightLogger.appendRow(grfmOutput[0].t, ~grfmOutput[0].asVector());
        grfLeftLogger.appendRow(grfmOutput[1].t, ~grfmOutput[1].asVector());
        tauLogger.appendRow(ikFiltered.t, ~idOutput.tau);
    }

    cout << "Mean delay: "
         << std::accumulate(sumDelayMS.begin(), sumDelayMS.end(), 0.0) /
                    sumDelayMS.size()
         << " ms" << endl;

    // // store results
    STOFileAdapter::write(grfRightLogger,
                          resultsDir + "wrench_right_with_gt_force.sto");
    STOFileAdapter::write(grfLeftLogger,
                          resultsDir + "wrench_left_with_gt_force.sto");
    STOFileAdapter::write(tauLogger, resultsDir + "tau_with_gt_force.sto");
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