#include "NGIMUData.h"
using namespace OpenSimRT;

// // operator overload
// std::ostream& operator<<(std::ostream& os, const NGIMUData::Quaternion& q) {
//     return os << q.q1 << " " << q.q2 << " " << q.q3 << " " << q.q4;
// }

SimTK::Vector NGIMUData::asVector() const {
    SimTK::Vector vec(14);
    vec[0] = this->quaternion.q1;
    vec[1] = this->quaternion.q2;
    vec[2] = this->quaternion.q3;
    vec[3] = this->quaternion.q4;
    vec[4] = this->sensors.acceleration.ax;
    vec[5] = this->sensors.acceleration.ay;
    vec[6] = this->sensors.acceleration.az;
    vec[7] = this->sensors.gyroscope.gx;
    vec[8] = this->sensors.gyroscope.gy;
    vec[9] = this->sensors.gyroscope.gz;
    vec[10] = this->sensors.magnetometer.mx;
    vec[11] = this->sensors.magnetometer.my;
    vec[12] = this->sensors.magnetometer.mz;
    vec[13] = this->sensors.barometer.barometer;
    return vec;
}
