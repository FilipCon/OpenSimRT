#include "IMUCalibrator.h"
#include "INIReader.h"
#include "InverseKinematics.h"
#include "NGIMUInputFromFileDriver.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "Visualization.h"

#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/CoordinateAxis.h>
#include <SimTKcommon/internal/State.h>
#include <Simulation/Model/Model.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <iostream>
#include <mutex>
#include <simbody/internal/Visualizer.h>
#include <string>
#include <thread>
#include <vector>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

void run() {
    INIReader ini(INI_FILE);
    auto section = "UPPER_LIMB_NGIMU_OFFLINE";
    auto IMU_BODIES = ini.getVector(section, "IMU_BODIES", vector<string>());
    auto imuDirectionAxis = ini.getString(section, "IMU_DIRECTION_AXIS", "");
    auto imuBaseBody = ini.getString(section, "IMU_BASE_BODY", "");
    auto xGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_X", 0.0);
    auto yGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Y", 0.0);
    auto zGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Z", 0.0);
    auto markerNames = ini.getVector(section, "MARKER_NAMES", vector<string>());
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto ngimuDataFile =
            subjectDir + ini.getString(section, "NGIMU_DATA_CSV", "");
    auto markerDataFile =
            subjectDir + ini.getString(section, "MARKER_DATA_CSV", "");

    // setup model
    Model model(modelFile);
    auto chestMarker = Marker("chest", model.getBodySet().get(IMU_BODIES[0]),
                              Vec3(0.08, -0.15, 0));
    auto humerusMarker =
            Marker("humerus", model.getBodySet().get(IMU_BODIES[1]),
                   Vec3(0, -0.15, 0.04));
    auto radiusMarker = Marker("radius", model.getBodySet().get(IMU_BODIES[2]),
                               Vec3(0.02, -0.1, 0.055));
    model.addMarker(&chestMarker);
    model.addMarker(&humerusMarker);
    model.addMarker(&radiusMarker);
    model.finalizeConnections();

    // load recorded marker positions
    TimeSeriesTable_<Vec3> markerData(
            TimeSeriesTable(markerDataFile)
                    .pack<SimTK::Vec3>({"_x", "_y", "_z"}));
    // transform data in table
    double xDegrees = 0;
    double yDegrees = 0;
    double zDegrees = 90;
    auto xRad = SimTK::convertDegreesToRadians(xDegrees);
    auto yRad = SimTK::convertDegreesToRadians(yDegrees);
    auto zRad = SimTK::convertDegreesToRadians(zDegrees);
    for (size_t i = 0; i < markerData.getNumRows(); ++i) {
        auto row = markerData.updRowAtIndex(i);
        for (int j = 0; j < markerData.getNumColumns(); ++j) {
            row[j] = Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence,
                              xRad, SimTK::XAxis, yRad, SimTK::YAxis, zRad,
                              SimTK::ZAxis) * row[j];
        }
    }

    // marker tasks
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> markerObservationOrder;
    InverseKinematics::createMarkerTasksFromMarkerNames(
            model, markerNames, markerTasks, markerObservationOrder);

    // imu tasks
    vector<InverseKinematics::IMUTask> imuTasks;
    // vector<string> imuObservationOrder(IMU_BODIES);
    // InverseKinematics::createIMUTasksFromObservationOrder(
    //         model, imuObservationOrder, imuTasks);

    // // ngimu input data driver from file
    // NGIMUInputFromFileDriver driver(ngimuDataFile, 60);
    // driver.startListening();

    // // calibrator
    // IMUCalibrator clb(model, &driver, imuObservationOrder);
    // clb.recordNumOfSamples(1);
    // clb.setGroundOrientationSeq(xGroundRotDeg, yGroundRotDeg, zGroundRotDeg);
    // clb.computeheadingRotation(imuBaseBody, imuDirectionAxis);
    // clb.calibrateIMUTasks(imuTasks);

    // initialize ik (lower constraint weight and accuracy -> faster tracking)
    InverseKinematics ik(model, markerTasks, imuTasks, SimTK::Infinity, 1e-5);
    auto qLogger = ik.initializeLogger();

    // visualizer
    BasicModelVisualizer visualizer(model);

    // stream marker data
    InverseKinematics::Input input;
    std::mutex mu;
    std::condition_variable cond;
    bool flag = false;
    auto markerDataStream = [&](const int& duration) {
        static int i = 0;
        while (true) {
            {
                std::lock_guard<std::mutex> lock(mu);
                input = InverseKinematics::getFrameFromTimeSeriesTable(
                        i, markerData, markerObservationOrder, false);
                flag = true;
            }
            cond.notify_one();
            ++i;
            this_thread::sleep_for(chrono::milliseconds(duration));
        }
    };
    auto markerDataAcquisition = [&]() {
        std::unique_lock<std::mutex> lock(mu);
        cond.wait(lock, [&]() { return flag == true; });
        flag = false;
        return input;
    };

    thread t1(markerDataStream, 10);

    try { // main loop
        while (true) {
            // get input from imus
            // auto imuDataFrame = driver.getFrame();
            auto markerDataFrame = markerDataAcquisition();

            // solve ik
            // auto pose = ik.solve(clb.transform(
            //         imuDataFrame, markerDataFrame.markerObservations));
            auto pose = ik.solve(markerDataFrame);

            // visualize
            visualizer.update(pose.q);

            // record
            qLogger.appendRow(pose.t, ~pose.q);
        }
    } catch (std::exception& e) {
        cout << e.what() << endl;
        t1.join();
    }

    // store results
    STOFileAdapter::write(qLogger,
                          subjectDir + "real_time/inverse_kinematics/q.sto");
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
