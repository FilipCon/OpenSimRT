#pragma once
#include "internal/IMUExports.h"

#include <Simbody.h>
#include <osc/OscTypes.h>

namespace OpenSimRT {

// NGIMU data container
struct IMU_API NGIMUData {
    struct Quaternion {
        double q1, q2, q3, q4;
        osc::uint64 timeStamp;
    };
    struct Sensors {
        struct Acceleration {
            double ax, ay, az;
        };
        struct Gyroscope {
            double gx, gy, gz;
        };
        struct Magnetometer {
            double mx, my, mz;
        };
        struct Barometer {
            double barometer;
        };
        Acceleration acceleration; // in g
        Gyroscope gyroscope;       // in circ/sec
        Magnetometer magnetometer; // in uT
        Barometer barometer;       // in hPa
        osc::uint64 timeStamp;
    };
    Quaternion quaternion;
    Sensors sensors;
    static int size() { return 14; }
    SimTK::Vector asVector() const;
};


} // namespace OpenSimRT
