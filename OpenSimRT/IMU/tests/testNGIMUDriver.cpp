#include "INIReader.h"
#include "NGIMUInputDriver.h"
#include "Settings.h"
#include "ip/UdpSocket.h"
#include "osc/OscPacketListener.h"
#include "osc/OscReceivedElements.h"

#include <chrono>
#include <iostream>
#include <thread>

using namespace osc;
using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

void run() {
    INIReader ini(INI_FILE);
    auto section = "TEST_NGIMU_DRIVER";

    auto IMU_IP = ini.getVector(section, "IMU_IP", vector<string>());
    auto LISTEN_IP = ini.getString(section, "LISTEN_IP", "0.0.0.0");
    auto SEND_PORTS = ini.getVector(section, "SEND_PORTS", vector<int>());
    auto LISTEN_PORTS = ini.getVector(section, "LISTEN_PORTS", vector<int>());
    auto IMU_NAMES = ini.getVector(section, "IMU_NAMES", vector<string>());

    NGIMUInputDriver driver;
    driver.setupInput(IMU_NAMES, vector<string>(LISTEN_PORTS.size(), LISTEN_IP),
                      LISTEN_PORTS);
    driver.setupTransmitters(IMU_IP, SEND_PORTS, LISTEN_IP, LISTEN_PORTS);

    auto timer = []() {
        int i = 0;
        while (true) {
            cout << i++ << " Passed ++++++++++++++++++++++++++++++++++++++" << endl;
            this_thread::sleep_for(1s);
        }
    };

    thread time (timer);
    thread listen(&NGIMUInputDriver::startListening, &driver);

    while (true) {
        auto input = driver.getFrame();
        // cout << input.first << endl;
    }
    listen.join();
}

int main(int argc, char* argv[]) {
    try {
        run();
    } catch (exception& e) {
        cout << e.what() << endl;
        // we do not want this test to fail since device connection is required
        // return -1;
    }
    return 0;
}
