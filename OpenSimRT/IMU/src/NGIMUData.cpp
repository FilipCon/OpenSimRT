#include "NGIMUData.h"
using namespace OpenSimRT;

// // operator overload
// std::ostream& operator<<(std::ostream& os, const NGIMUData::Quaternion& q) {
//     return os << q.q1 << " " << q.q2 << " " << q.q3 << " " << q.q4;
// }

SimTK::Vector NGIMUData::asVector() const {
    SimTK::Vector vec(14);
    vec[0] = this->quaternion.q[0];
    vec[1] = this->quaternion.q[1];
    vec[2] = this->quaternion.q[2];
    vec[3] = this->quaternion.q[3];
    vec[4] = this->sensors.acceleration.a[0];
    vec[5] = this->sensors.acceleration.a[1];
    vec[6] = this->sensors.acceleration.a[2];
    vec[7] = this->sensors.gyroscope.g[0];
    vec[8] = this->sensors.gyroscope.g[1];
    vec[9] = this->sensors.gyroscope.g[2];
    vec[10] = this->sensors.magnetometer.m[0];
    vec[11] = this->sensors.magnetometer.m[1];
    vec[12] = this->sensors.magnetometer.m[2];
    vec[13] = this->sensors.barometer;
    return vec;
}
