#include "IMUCalibrator.h"
#include "INIReader.h"
#include "InverseKinematics.h"
#include "MoticonReceiverFromFile.h"
#include "NGIMUData.h"
#include "NGIMUInputDriver.h"
#include "NGIMUInputFromFileDriver.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "SyncManager.h"
#include "Utils.h"
#include "Visualization.h"

#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/CoordinateAxis.h>
#include <SimTKcommon/internal/NTraits.h>
#include <SimTKcommon/internal/State.h>
#include <Simulation/Model/Model.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <simbody/internal/Visualizer.h>
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
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto moticonDataFile =
            subjectDir + ini.getString(section, "MOTICON_DATA_CSV", "");
    auto ngimuDataFile =
            subjectDir + ini.getString(section, "NGIMU_DATA_CSV", "");

    // setup model
    Model model(modelFile);

    // add marker to pelvis center to track model position from imus
    auto pelvisMarker =
            Marker("PelvisCenter", model.getBodySet().get("pelvis"), Vec3(0));
    model.addMarker(&pelvisMarker);
    model.finalizeConnections();

    State state = model.initSystem();
    model.realizePosition(state);
    auto height = model.getBodySet().get("pelvis").findStationLocationInGround(
            state, Vec3(0));

    // marker tasks
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> markerObservationOrder;
    InverseKinematics::createMarkerTasksFromMarkerNames(
            model, vector<string>{"PelvisCenter"}, markerTasks,
            markerObservationOrder);

    // imu tasks
    vector<InverseKinematics::IMUTask> imuTasks;
    vector<string> imuObservationOrder(IMU_BODIES);
    InverseKinematics::createIMUTasksFromObservationOrder(
            model, imuObservationOrder, imuTasks);

    // ngimu input data driver from file
    NGIMUInputFromFileDriver driver(ngimuDataFile);

    // insole driver
    MoticonReceiverFromFile mt(moticonDataFile);

    // calibrator
    IMUCalibrator clb(model, &driver, imuObservationOrder);
    clb.recordNumOfSamples(1);
    clb.setGroundOrientationSeq(xGroundRotDeg, yGroundRotDeg, zGroundRotDeg);
    clb.computeheadingRotation(imuBaseBody, imuDirectionAxis);
    clb.calibrateIMUTasks(imuTasks);

    // initialize ik (lower constraint weight and accuracy -> faster tracking)
    InverseKinematics ik(model, markerTasks, imuTasks, SimTK::Infinity, 1e-5);
    auto qLogger = ik.initializeLogger();

    SyncManager manager(1 / 60 / 5.0);

    // visualizer
    BasicModelVisualizer visualizer(model);
    auto rightInsoleDecorator = new ForceDecorator(Green, 0.01, 3);
    visualizer.addDecorationGenerator(rightInsoleDecorator);
    auto leftInsoleDecorator = new ForceDecorator(Green, 0.01, 3);
    visualizer.addDecorationGenerator(leftInsoleDecorator);

    size_t i = 0;
    try { // main loop
        while (true) {
            // get input from sensors
            auto imuDataFrame = driver.getFrameAsVector();
            auto insoleDataFrame = mt.getFrameAsVector();

            // synchronize data streams
            manager.appendPack(imuDataFrame, insoleDataFrame);

            MoticonReceivedBundle moticonData;
            auto insolePack = manager.getPack(1);
            moticonData.timestamp = insolePack.first;
            moticonData.fromVector(insolePack.second);

            NGIMUInputDriver::IMUDataFrame imuData;
            auto imuPack = manager.getPack(0);
            imuData.first = imuPack.first;
            imuData.second = driver.fromVector(imuPack.second);

            ++i;

            // ignore nans
            if (!isVectorFinite(imuPack.second) ||
                !isVectorFinite(insolePack.second))
                continue;

            // solve ik
            auto pose = ik.solve(clb.transform(
                    imuData, vector<SimTK::Vec3>{Vec3(0, 0, 0) + height}));

            // insole aliases
            const auto& r_cop = moticonData.right.cop;
            const auto& r_force = moticonData.right.totalForce;
            const auto& l_cop = moticonData.left.cop;
            const auto& l_force = moticonData.left.totalForce;

            // visualize
            visualizer.update(pose.q);
            Vec3 calcn_r_point_in_ground;
            Vec3 calcn_l_point_in_ground;
            visualizer.expressPositionInGround("calcn_r_insole",
                                               Vec3(r_cop[0], 0, -r_cop[1]),
                                               calcn_r_point_in_ground);
            visualizer.expressPositionInGround("calcn_l_insole",
                                               Vec3(l_cop[0], 0, -l_cop[1]),
                                               calcn_l_point_in_ground);
            rightInsoleDecorator->update(calcn_r_point_in_ground,
                                         r_force * Vec3(0, 1, 0));
            leftInsoleDecorator->update(calcn_l_point_in_ground,
                                        l_force * Vec3(0, 1, 0));

            // record
            qLogger.appendRow(pose.t, ~pose.q);

            // // dummy delay to simulate real time
            std::this_thread::sleep_for(std::chrono::milliseconds(13));
        }
    } catch (std::exception& e) { cout << e.what() << endl; }

    // // store results
    // STOFileAdapter::write(qLogger,
    //                       subjectDir + "real_time/inverse_kinematics/q.sto");
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
