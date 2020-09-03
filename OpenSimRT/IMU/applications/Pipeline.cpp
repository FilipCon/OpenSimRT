#include "IMUCalibrator.h"
#include "INIReader.h"
#include "Manager.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "Simulation.h"
#include "Visualization.h"

#include <OpenSim/Common/STOFileAdapter.h>
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
    // auto section = "LOWER_BODY_NGIMU";
    auto section = "UPPER_LIMB_NGIMU";
    auto IMU_IP = ini.getVector(section, "IMU_IP", vector<string>());
    auto LISTEN_IP = ini.getString(section, "LISTEN_IP", "0.0.0.0");
    auto SEND_PORTS = ini.getVector(section, "SEND_PORTS", vector<int>());
    auto LISTEN_PORTS = ini.getVector(section, "LISTEN_PORTS", vector<int>());
    auto IMU_BODIES = ini.getVector(section, "IMU_BODIES", vector<string>());
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");

    // setup model
    Model model(modelFile);
    vector<InverseKinematics::IMUTask> imuTasks;
    vector<string> observationOrder(IMU_BODIES);
    InverseKinematics::createIMUTasksFromObservationOrder(
            model, observationOrder, imuTasks);

    // initialize ik (lower constraint weight and accuracy -> faster tracking)
    InverseKinematics ik(model, vector<InverseKinematics::MarkerTask>{},
                         imuTasks, SimTK::Infinity, 1e-5);
    auto qLogger = ik.initializeLogger();

    // manager
    NGIMUManager manager;
    manager.setupListeners(vector<string>(LISTEN_PORTS.size(), LISTEN_IP),
                           LISTEN_PORTS);
    manager.setupTransmitters(IMU_IP, SEND_PORTS, LISTEN_IP, LISTEN_PORTS);

    // calibrator
    IMUCalibrator clb = IMUCalibrator(model, &manager, observationOrder);
    PositionTracker pt = PositionTracker(model, observationOrder);

    // start listening
    thread listen(&NGIMUManager::startListeners, &manager);

    clb.run(3.0); // record for 3 seconds

    // visualizer
    BasicModelVisualizer visualizer(model);
    auto accVectorDecorator = new ForceDecorator(Red, 1, 3);
    visualizer.addDecorationGenerator(accVectorDecorator);

    try { // main loop
        while (true) {
            // get input from imus
            auto imuDataFrame = manager.getObservations();

            auto acc = pt.computeVelocity(imuDataFrame);

            // solve ik
            auto pose = ik.solve(clb.transform(imuDataFrame));

            // visualize
            visualizer.update(pose.q);
            // Vec3 pelvisJoint;
            // visualizer.expressPositionInGround("pelvis", Vec3(0),
            //                                    pelvisJoint);
            // accVectorDecorator->update(pelvisJoint, acc);

            // record
            qLogger.appendRow(pose.t, ~pose.q);
        }
    } catch (std::exception& e) {
        cout << e.what() << endl;
        manager.stopListeners();
        listen.join();

        // store results
        STOFileAdapter::write(
                qLogger, subjectDir + "real_time/inverse_kinematics/q.sto");
    }
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
