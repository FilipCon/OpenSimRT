/**
 * @file testMarkerReconstruction.cpp
 * @author Filip Konstantinos (filip.k@ece.upatras.gr)
 * @brief
 * @version 0.1
 * @date 2019-12-18
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "INIReader.h"
#include "MarkerReconstruction.h"
#include "RealTimeAnalysis.h"
#include "Settings.h"
#include "Simulation.h"

#include <Common/Units.h>
#include <OpenSim/Common/TRCFileAdapter.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <iostream>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

InverseKinematics::Input
simulateMissingMarkers(const InverseKinematics::Input& inputFrame,
                       const vector<string>& markerList,
                       const vector<string>& missingMarkerList,
                       const double& duration, const double& t0) {
    if (t0 == 0.0) THROW_EXCEPTION("Invalid input. Select initSample != 0");

    auto outputFrame = inputFrame;
    for (int i = 0; i < missingMarkerList.size(); ++i) {
        auto itr = find(markerList.begin(), markerList.end(),
                        missingMarkerList[i]);
        if (itr == markerList.end()) {
            THROW_EXCEPTION("Invalid input. Marker name does not exist");
        }
        if (itr != markerList.end() && inputFrame.t >= t0 &&
            inputFrame.t - t0 <= duration) {
            int idx = distance(markerList.begin(), itr);
            outputFrame.markerObservations[idx] = Vec3(NaN, NaN, NaN);
        }
    }
    return outputFrame;
}

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto section = "TEST_IK_FROM_FILE";
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto trcFile = subjectDir + ini.getString(section, "TRC_FILE", "");
    auto ikTaskSetFile =
            subjectDir + ini.getString(section, "IK_TASK_SET_FILE", "");

    // simulate-missing-markers parameters
    vector<string> missingMarkers = {
            "R.ASIS",        "R.Thigh.Rear", "R.Toe.Tip", "R.Shank.Upper",
            "L.Shank.Upper", "L.Thigh.Rear", "L.Toe.Tip"};
    double duration = 2.4;
    double t0 = 0.02;

    Model model(modelFile);
    MarkerData markerData(trcFile);

    markerData.getDataRate();
    // prepare marker tasks
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> observationOrder;
    InverseKinematics::createMarkerTasksFromMarkerData(
            model, markerData, markerTasks, observationOrder);

    // acquisition function (simulates acquisition from motion)
    auto dataAcquisitionFunction = [&]() -> MotionCaptureInput {
        static int i = 0;
        MotionCaptureInput input;

        // get frame data
        input.IkFrame = InverseKinematics::getFrameFromMarkerData(
                i, markerData, observationOrder, false);
        double t = input.IkFrame.t;

        i++;
        return input;
    };

    // initialize marker reconstruction
    MarkerReconstruction mmr(model, markerTasks);
    mmr.init(dataAcquisitionFunction);
    auto mmrLogger = mmr.initializeLogger();
    // mmrLogger.addTableMetaData("DataRate", markerData.getDataRate());
    // mmrLogger.addTableMetaData("Units", Units::Millimeters);

    // loop through marker frames
    while (true) {
        auto dataFrame = dataAcquisitionFunction();
        if (dataFrame.IkFrame.t == markerData.getLastFrameTime()) break;

        auto frameOriginal = dataFrame.IkFrame;
        auto frameReconstructed = simulateMissingMarkers(
                frameOriginal, observationOrder, missingMarkers, duration, t0);

        // perform marker reconstruction
        mmr.solve(frameReconstructed.markerObservations);

        // write to logger
        mmrLogger.appendRow(frameReconstructed.t,
                            frameReconstructed.markerObservations);
    }

    // store results
    STOFileAdapter::write(mmrLogger.flatten(),
                          subjectDir + "real_time/marker_reconstruction/"
                                       "reconstructed_markers.sto");
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
