#pragma once
#include "internal/IMUExports.h"

#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/Quaternion.h>
#include <Simbody.h>
#include <osc/OscTypes.h>

namespace OpenSimRT {

// NGIMU data container
struct IMU_API NGIMUData {
    struct Quaternion {
        SimTK::Quaternion q;
        osc::uint64 timeStamp;
    };
    struct Sensors {
        struct Acceleration {
            SimTK::Vec3 a;
        };
        struct Gyroscope {
            SimTK::Vec3 g;
        };
        struct Magnetometer {
            SimTK::Vec3 m;
        };
        Acceleration acceleration; // in g
        Gyroscope gyroscope;       // in circ/sec
        Magnetometer magnetometer; // in uT
        double barometer;          // in hPa
        osc::uint64 timeStamp;
    };
    struct LinearAcceleration {
        SimTK::Vec3 a;
        osc::uint64 timeStamp;
    };
    struct Altitude {
        double x;
        osc::uint64 timeStamp;
    };

    Sensors sensors;
    Quaternion quaternion;
    LinearAcceleration linear;
    Altitude altitude;
    static int size() { return 18; }
    SimTK::Vector asVector() const;
    void fromVector(const SimTK::Vector&);
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
