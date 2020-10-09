#include "PositionTracker.h"

using namespace OpenSimRT;
using namespace OpenSim;
using namespace std;
using namespace SimTK;

PositionTracker::PositionTracker(
        const Model& otherModel,
        const std::vector<std::string>& observationOrder)
        : model(*otherModel.clone()) {
    double samplingFreq = 60;
    imuLabels = observationOrder;

    // numerical integrator
    accelerationIntegrator = new NumericalIntegrator(3);
    velocityIntegrator = new NumericalIntegrator(3);
    int filterOrder = 2;
    // construct filters
    accelerationLPFilter =
            new ButterworthFilter(filterOrder, (2 * 3) / samplingFreq,
                                  ButterworthFilter::FilterType::LowPass,
                                  IIRFilter::InitialValuePolicy::Zero);

    accelerationHPFilter =
            new ButterworthFilter(filterOrder, (2 * 0.001) / samplingFreq,
                                  ButterworthFilter::FilterType::HighPass,
                                  IIRFilter::InitialValuePolicy::Zero);

    velocityHPFilter =
            new ButterworthFilter(filterOrder, (2 * 0.1) / samplingFreq,
                                  ButterworthFilter::FilterType::HighPass,
                                  IIRFilter::InitialValuePolicy::Zero);

    positionHPFilter =
            new ButterworthFilter(filterOrder, (2 * 0.1) / samplingFreq,
                                  ButterworthFilter::FilterType::HighPass,
                                  IIRFilter::InitialValuePolicy::Zero);
}

Vec3 PositionTracker::computePosition(
        const std::pair<double, std::vector<NGIMUData>>& data) {
    // get index of pelvis in the imu label list
    const int pelvisIndex = std::distance(
            imuLabels.begin(),
            std::find(imuLabels.begin(), imuLabels.end(), "pelvis_imu"));

    const auto& t = data.first;
    // const auto& acc = data.second[pelvisIndex].sensors.acceleration;
    const auto& linAcc = data.second[pelvisIndex].linear.a;
    const auto& altitude = data.second[pelvisIndex].altitude.x;

    // LP filter acceleration
    const auto a_f = accelerationLPFilter->filter(
            accelerationHPFilter->filter(Vector(linAcc)));

    // double p = 20;
    // auto fc = (a_f.norm() > 1 / p) ? 1 / (a_f.norm() * p) : 2;

    // velocityHPFilter->setupFilter(2,  (2 * fc) / 60,
    //                               ButterworthFilter::FilterType::HighPass,
    //                               IIRFilter::InitialValuePolicy::Signal);

    // compute velocity and filter velocity
    const auto velVector =
            velocityHPFilter->filter(accelerationIntegrator->integrate(a_f, t));

    // return velocity based on acceleration threshold
    const double threshold = 0.1;
    double range = 0.1; // [0,1]
    const auto vel = (a_f.norm() < threshold)
                             ? Vec3(velVector[0], velVector[1], velVector[2]) *
                                       pow(range, threshold - a_f.norm())
                             : Vec3(velVector[0], velVector[1], velVector[2]);

    // auto posVector = positionHPFilter->filter(
    //         velocityIntegrator->integrate(Vector(vel), t));
    auto posVector = velocityIntegrator->integrate(Vector(vel), t);
    return Vec3(0, posVector[2], 0);
}

// SimTK::Vec3 PositionTracker::computePosition(const SimTK::Vec3& vel,
//                                              const double& t) {
// }
