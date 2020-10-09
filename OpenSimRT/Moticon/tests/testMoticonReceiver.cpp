#include "INIReader.h"
#include "MoticonReceiver.h"
#include "Settings.h"

#include <chrono>
#include <iomanip>
#include <iostream>

#define TIME_NOW                                                               \
    duration_cast<nanoseconds>(                                                \
            high_resolution_clock::now().time_since_epoch())                   \
            .count()

using namespace std;
using namespace chrono;
using namespace OpenSimRT;

void run() {
    INIReader ini(INI_FILE);
    auto section = "TEST_MOTICON";
    auto LISTEN_IP = ini.getString(section, "LISTEN_IP", "0.0.0.0");
    auto INSOLES_PORT = ini.getInteger(section, "INSOLE_LISTEN_PORT", 0);
    auto INSOLE_SIZE = ini.getInteger(section, "INSOLE_SIZE", 0);

    // insole driver
    MoticonReceiver mt(LISTEN_IP, INSOLES_PORT);
    mt.setInsoleSize(INSOLE_SIZE);
    auto mtLogger = mt.initializeLogger();

    while (true) {
        auto output = mt.receiveData();
        cout << std::setprecision(15) << output.timestamp << " " << TIME_NOW / 1000000000.0
             << endl;
    }
}

int main(int argc, char* argv[]) {
    try {
        run();
    } catch (exception& e) {
        cout << e.what() << endl;
        return -1;
    }
}
