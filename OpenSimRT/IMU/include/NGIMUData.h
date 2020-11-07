#pragma once
#include "internal/IMUExports.h"

#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/Quaternion.h>
#include <Simbody.h>

#define PICOSECS_RESOLUTION_BIN 4294967295UL // pow(2, 32)
#define PICOSECS_RESOLUTION_DEC 1000000000   // pow(10, 9)

#define RFC_PROTOCOL                                                           \
    2208988800UL // RFC protocol counting 70 years since Jan 1 1900, 00:00 GMT
#define GMT_LOCAL_TIMEZONE 10800UL // Timezone in Greece is GMT+03:00

namespace OpenSimRT {

// NGIMU data container
struct IMU_API NGIMUData {
    struct Quaternion {
        SimTK::Quaternion q;
        double timeStamp;
    };
    struct Sensors {
        SimTK::Vec3 acceleration; // in g
        SimTK::Vec3 gyroscope;    // in circ/sec
        SimTK::Vec3 magnetometer; // in uT
        SimTK::Vec1 barometer;    // in hPa
        double timeStamp;
    };
    struct LinearAcceleration {
        SimTK::Vec3 acceleration;
        double timeStamp;
    };
    struct Altitude {
        SimTK::Vec1 measurement;
        double timeStamp;
    };

    Sensors sensors;
    Quaternion quaternion;
    LinearAcceleration linear;
    Altitude altitude;
    static constexpr int size() { return 18; }
    SimTK::Vector asVector() const;
    void fromVector(const SimTK::Vector&);

    std::vector<std::pair<double, SimTK::Vector>> getAsPairsOfVectors() const;
    void
    setFromPairsOfVectors(const std::vector<std::pair<double, SimTK::Vector>>&);
};

inline bool operator==(const NGIMUData& lhs, const NGIMUData& rhs) {
    bool cond = (lhs.sensors.timeStamp == rhs.sensors.timeStamp) &&
                (lhs.quaternion.timeStamp == rhs.quaternion.timeStamp) &&
                (lhs.altitude.timeStamp == rhs.altitude.timeStamp) &&
                (lhs.linear.timeStamp == rhs.linear.timeStamp);
    return (cond) ? true : false;
}

inline bool operator!=(const NGIMUData& lhs, const NGIMUData& rhs) {
    bool cond = (lhs.sensors.timeStamp == rhs.sensors.timeStamp) &&
                (lhs.quaternion.timeStamp == rhs.quaternion.timeStamp) &&
                (lhs.altitude.timeStamp == rhs.altitude.timeStamp) &&
                (lhs.linear.timeStamp == rhs.linear.timeStamp);
    return (cond) ? false : true;
}
} // namespace OpenSimRT
