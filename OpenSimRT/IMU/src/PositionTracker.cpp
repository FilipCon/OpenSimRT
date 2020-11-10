#include "PositionTracker.h"

#include <SimTKcommon/internal/BigMatrix.h>

using namespace OpenSimRT;
using namespace OpenSim;
using namespace std;
using namespace SimTK;

PositionTracker::PositionTracker(const double& rate, const double& th)
        : _threshold(th), _samplingFreq(rate) {
    int xDegrees = -90;
    int yDegrees = -90;
    int zDegrees = 0;
    auto xRad = SimTK::convertDegreesToRadians(xDegrees);
    auto yRad = SimTK::convertDegreesToRadians(yDegrees);
    auto zRad = SimTK::convertDegreesToRadians(zDegrees);

    R_GoGi = Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence, xRad,
                      SimTK::XAxis, yRad, SimTK::YAxis, zRad, SimTK::ZAxis);
    // numerical integrator
    accelerationIntegrator = new NumericalIntegrator(3);
    velocityIntegrator = new NumericalIntegrator(3);

    int filterOrder = 1; // TODO

    // construct filters
    accelerationLPFilter =
            new ButterworthFilter(3, filterOrder, (2 * 5.0) / _samplingFreq,
                                  ButterworthFilter::FilterType::LowPass,
                                  IIRFilter::InitialValuePolicy::Zero);

    accelerationHPFilter =
            new ButterworthFilter(3, filterOrder, (2 * 0.001) / _samplingFreq,
                                  ButterworthFilter::FilterType::HighPass,
                                  IIRFilter::InitialValuePolicy::Zero);

    velocityHPFilter =
            new ButterworthFilter(3, filterOrder, (2 * 0.1) / _samplingFreq,
                                  ButterworthFilter::FilterType::HighPass,
                                  IIRFilter::InitialValuePolicy::Zero);

    positionHPFilter =
            new ButterworthFilter(3, filterOrder, (2 * 0.1) / _samplingFreq,
                                  ButterworthFilter::FilterType::HighPass,
                                  IIRFilter::InitialValuePolicy::Zero);
}

Vec3 PositionTracker::computeVelocity(const double& t, const SimTK::Vec3& a) {
    const auto a_f = Vector(a);

    // compute velocity and filter velocity
    auto vel =  accelerationIntegrator->integrate(a_f, t);
    auto velV = velocityHPFilter->filter(vel);

    // return velocity based on acceleration threshold
    return (a_f.norm() < _threshold) ? Vec3(0) : Vec3(&velV[0]);
}

Vec3 PositionTracker::computePosition(const double& t, const SimTK::Vec3& v) {
    // auto posVector = positionHPFilter->filter(
    //         velocityIntegrator->integrate(Vector(v), t));
    auto posVector = velocityIntegrator->integrate(Vector(v), t);
    return Vec3(&posVector[0]);
}
