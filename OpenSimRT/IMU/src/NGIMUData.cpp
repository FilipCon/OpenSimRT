#include "NGIMUData.h"

#include "Utils.h"

#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/Quaternion.h>
#include <cstring>
#include <deque>
using namespace OpenSimRT;
using namespace SimTK;

Vector NGIMUData::asVector() const {
    double v[this->size()];
    int i = 0;
    vectorToDouble(Vector(this->quaternion.q), v + i);
    i += 4;
    vecToDouble(this->sensors.acceleration.a, v + i);
    i += 3;
    vecToDouble(this->sensors.gyroscope.g, v + i);
    i += 3;
    vecToDouble(this->sensors.magnetometer.m, v + i);
    i += 3;
    std::memcpy(v + i, &this->sensors.barometer, sizeof(double));
    i += 1;
    vecToDouble(this->linear.a, v + i);
    i += 3;
    std::memcpy(v + i, &this->altitude, sizeof(double));
    i += 1;
    return std::move(Vector_<double>(this->size(), v, true));
}

void NGIMUData::fromVector(const Vector& v) {
    int i = 0;
    this->quaternion.q = SimTK::Quaternion(v[0], v[1], v[2], v[3]);
    this->sensors.acceleration.a = Vec3(&v[4]);
    this->sensors.gyroscope.g = Vec3(&v[7]);
    this->sensors.magnetometer.m = Vec3(&v[10]);
    this->sensors.barometer = v[13];
    this->linear.a = Vec3(&v[14]);
    this->altitude.x = v[17];
}

