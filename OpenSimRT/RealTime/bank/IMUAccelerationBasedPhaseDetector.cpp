#include "IMUAccelerationBasedPhaseDetector.h"

#include "SignalProcessing.h"

#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/BigMatrix.h>

using namespace OpenSimRT;

IMUAccelerationBasedPhaseDetector::IMUAccelerationBasedPhaseDetector(
        const Parameters& otherParameters)
        : GaitPhaseDetector(), parameters(otherParameters) {
    const int& filterOrder = parameters.filterOrder;
    const double& samplingFreq = parameters.samplingFreq;
    const double& lpCutOffFreq = parameters.lpCutoffFreq;
    const double& hpCutOffFreq = parameters.hpCutoffFreq;

    // construct filter
    bwLPFilter = new ButterworthFilter(
            6, filterOrder, (2 * lpCutOffFreq) / parameters.samplingFreq,
            ButterworthFilter::FilterType::LowPass,
            IIRFilter::InitialValuePolicy::Zero);
    bwHPFilter = new ButterworthFilter(
            6, filterOrder, (2 * hpCutOffFreq) / parameters.samplingFreq,
            ButterworthFilter::FilterType::HighPass,
            IIRFilter::InitialValuePolicy::Zero);
}

void IMUAccelerationBasedPhaseDetector::updDetector(const Input& input) {
    // combine measurements as single Vector
    SimTK::Vector v(6, 0.0);
    v(0, 3) = SimTK::Vector(input.rightAcceleration);
    v(3, 3) = SimTK::Vector(input.leftAcceleration);

    // filter measurements
    // auto fAcc = bwHPFilter->filter(bwLPFilter->filter(v));
    auto rAcc = SimTK::Vec3(&v[0]);
    auto lAcc = SimTK::Vec3(&v[3]);

    // update detector internal state
    updDetectorState(input.time, parameters.threshold - rAcc.norm(),
                     parameters.threshold - lAcc.norm());
}
