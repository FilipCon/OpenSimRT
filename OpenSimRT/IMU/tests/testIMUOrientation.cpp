#include "IMUCalibrator.h"
#include "INIReader.h"
#include "InverseKinematics.h"
#include "NGIMUInputDriver.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "Simulation.h"
#include "Visualization.h"

#include <Common/TimeSeriesTable.h>
#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/Rotation.h>
#include <SimTKcommon/internal/State.h>
#include <SimTKcommon/internal/UnitVec.h>
#include <Simulation/Model/Model.h>
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
    auto section = "TEST_NGIMU_ORIENTATION";
    auto IMU_IP = ini.getVector(section, "IMU_IP", vector<string>());
    auto LISTEN_IP = ini.getString(section, "LISTEN_IP", "0.0.0.0");
    auto SEND_PORTS = ini.getVector(section, "IMU_SEND_PORTS", vector<int>());
    auto LISTEN_PORTS =
            ini.getVector(section, "IMU_LISTEN_PORTS", vector<int>());
    auto IMU_BODIES = ini.getVector(section, "IMU_BODIES", vector<string>());
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");

    // setup model
    Model model(modelFile);
    model.finalizeConnections();

    // imu tasks
    vector<InverseKinematics::IMUTask> imuTasks;
    vector<string> imuObservationOrder(IMU_BODIES);
    InverseKinematics::createIMUTasksFromObservationOrder(
            model, imuObservationOrder, imuTasks);

    // initialize ik (lower constraint weight and accuracy -> faster tracking)
    InverseKinematics ik(model, vector<InverseKinematics::MarkerTask>{},
                         imuTasks, SimTK::Infinity, 1e-5);

    // driver
    NGIMUInputDriver driver;
    driver.setupInput(imuObservationOrder,
                      vector<string>(LISTEN_PORTS.size(), LISTEN_IP),
                      LISTEN_PORTS);
    driver.setupTransmitters(IMU_IP, SEND_PORTS, LISTEN_IP, LISTEN_PORTS);

    // calibrator
    IMUCalibrator clb =
            IMUCalibrator(model, &driver, imuObservationOrder);

    // start listening
    thread listen(&NGIMUInputDriver::startListening, &driver);

    clb.record(0.1);

    // visualizer
    BasicModelVisualizer visualizer(model);

    try { // main loop
        while (true) {
            // get input from imus
            auto imuDataFrame = driver.getFrame();


            auto transformedIMUData = clb.transform(imuDataFrame, vector<SimTK::Vec3>{});


            // solve ik
            auto pose = ik.solve(transformedIMUData);

            // visualize
            visualizer.update(pose.q);
        }
    } catch (std::exception& e) {
        cout << e.what() << endl;
        driver.stopListening();
        listen.join();
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
