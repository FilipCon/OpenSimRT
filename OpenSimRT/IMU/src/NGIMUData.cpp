#include "NGIMUData.h"

#include "Utils.h"

#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/Quaternion.h>
#include <cstring>
#include <deque>
#include <tuple>
#include <utility>
#include <vector>
using namespace OpenSimRT;
using namespace SimTK;

Vector NGIMUData::asVector() const {
    double v[this->size()];
    int i = 0;
    vectorToDouble(Vector(this->quaternion.q), v + i);
    i += 4;
    vecToDouble(this->sensors.acceleration, v + i);
    i += 3;
    vecToDouble(this->sensors.gyroscope, v + i);
    i += 3;
    vecToDouble(this->sensors.magnetometer, v + i);
    i += 3;
    vecToDouble(this->sensors.barometer, v + i);
    i += 1;
    vecToDouble(this->linear.acceleration, v + i);
    i += 3;
    vecToDouble(this->altitude.measurement, v + i);
    i += 1;
    return std::move(Vector_<double>(this->size(), v, true));
}

void NGIMUData::fromVector(const Vector& v) {
    int i = 0;
    this->quaternion.q = SimTK::Quaternion(v[0], v[1], v[2], v[3]);
    this->sensors.acceleration = Vec3(&v[4]);
    this->sensors.gyroscope = Vec3(&v[7]);
    this->sensors.magnetometer = Vec3(&v[10]);
    this->sensors.barometer = Vec1(&v[13]);
    this->linear.acceleration = Vec3(&v[14]);
    this->altitude.measurement = Vec1(&v[17]);
}

std::vector<std::pair<double, SimTK::Vector>>
NGIMUData::getAsPairsOfVectors() const {
    std::vector<std::pair<double, SimTK::Vector>> res{
            std::make_pair(this->quaternion.timeStamp,
                           Vector(this->quaternion.q)),
            std::make_pair(this->sensors.timeStamp,
                           Vector(this->sensors.acceleration)),
            std::make_pair(this->sensors.timeStamp,
                           Vector(this->sensors.gyroscope)),
            std::make_pair(this->sensors.timeStamp,
                           Vector(this->sensors.magnetometer)),
            std::make_pair(this->sensors.timeStamp,
                           Vector(this->sensors.barometer)),
            std::make_pair(this->linear.timeStamp,
                           Vector(this->linear.acceleration)),
            std::make_pair(this->altitude.timeStamp,
                           Vector(this->altitude.measurement))};
    return res;
}

void NGIMUData::setFromPairsOfVectors(
        const std::vector<std::pair<double, SimTK::Vector>>& input) {
    const auto& v = input[0].second;
    this->quaternion.q = SimTK::Quaternion(v[0], v[1], v[2], v[3]);
    this->sensors.acceleration = Vec3(&input[1].second[0]);
    this->sensors.gyroscope = Vec3(&input[2].second[0]);
    this->sensors.magnetometer = Vec3(&input[3].second[0]);
    this->sensors.barometer = Vec1(&input[4].second[0]);
    this->linear.acceleration = Vec3(&input[5].second[0]);
    this->altitude.measurement = Vec1(&input[6].second[0]);

    this->quaternion.timeStamp = input[0].first;
    this->sensors.timeStamp = input[1].first;
    this->linear.timeStamp = input[5].first;
    this->altitude.timeStamp = input[6].first;
}
