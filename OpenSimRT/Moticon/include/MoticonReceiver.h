#include <SimTKcommon.h>

#pragma once
#include "Network.h"
#include "internal/MoticonExports.h"

#include <ostream>
#include <string>
#include <vector>

class Moticon_API MoticonReceiver {
 public:
    struct MoticonReceivedBundle {
        double timestamp;
        struct Measurement {
            SimTK::Vec3 acceleration;
            SimTK::Vec3 angularRate;
            SimTK::Vec2 cop;
            SimTK::Vec<16> pressure; // 16 sensors
            double totalForce;
        } left, right;
    };

    MoticonReceiver();
    MoticonReceivedBundle receiveData();

 private:
    UDPSocket socket;
    std::string getStream();
    std::vector<double> splitInputStream(std::string, std::string);
};

/**
 * @brief Overwrite ostream operator for MoticonReceivedBundle
 */
std::ostream& operator<<(std::ostream& os,
                         const MoticonReceiver::MoticonReceivedBundle& m) {
    os << "Time: " << m.timestamp << " "
       << "L.Acceleration: " << m.left.acceleration << " "
       << "L.AngularRate: " << m.left.angularRate << " "
       << "L.CoP: " << m.left.cop << " "
       << "L.Pressure: " << m.left.pressure << " "
       << "L.TotalForce: " << m.left.totalForce << " "
       << "R.Acceleration: " << m.right.acceleration << " "
       << "R.AngularRate: " << m.right.angularRate << " "
       << "R.CoP: " << m.right.cop << " "
       << "R.Pressure: " << m.right.pressure << " "
       << "R.TotalForce: " << m.right.totalForce;
    return os;
}