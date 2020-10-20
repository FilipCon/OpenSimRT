#include "MoticonData.h"
#include "Exception.h"
#include "Utils.h"

using namespace OpenSimRT;
using namespace SimTK;

Vector MoticonReceivedBundle::asVector() const {
    double v[this->size()];
    int i = 0;
    vecToDouble(left.acceleration, v + i);
    i += 3;
    vecToDouble(left.angularRate, v + i);
    i += 3;
    vecToDouble(left.cop, v + i);
    i += 2;
    vecToDouble(left.pressure, v + i);
    i += 16;
    std::memcpy(v + i, &left.totalForce, sizeof(double));
    i += 1;
    vecToDouble(right.acceleration, v + i);
    i += 3;
    vecToDouble(right.angularRate, v + i);
    i += 3;
    vecToDouble(right.cop, v + i);
    i += 2;
    vecToDouble(right.pressure, v + i);
    i += 16;
    std::memcpy(v + i, &right.totalForce, sizeof(double));
    i += 1;
    return std::move(SimTK::Vector_<double>(this->size(), v, true));
}

void MoticonReceivedBundle::fromVector(const Vector& v) {
    this->left.acceleration = Vec3(&v[0]);
    this->left.angularRate = Vec3(&v[3]);
    this->left.cop = Vec2(&v[6]);
    this->left.pressure = Vec<16>(&v[8]);
    this->left.totalForce = v[24];

    this->right.acceleration = Vec3(&v[25]);
    this->right.angularRate = Vec3(&v[28]);
    this->right.cop = Vec2(&v[31]);
    this->right.pressure = Vec<16>(&v[33]);
    this->right.totalForce = v[49];
}
