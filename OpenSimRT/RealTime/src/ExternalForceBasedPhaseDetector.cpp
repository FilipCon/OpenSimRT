#include "ExternalForceBasedPhaseDetector.h"

#include "GRFMPrediction.h"

#include <algorithm>

using namespace OpenSimRT;

ExternalForceBasedPhaseDetector::ExternalForceBasedPhaseDetector(
        const Parameters& otherParameters)
        : GaitPhaseDetector(), parameters(otherParameters) {
    rSlidingWindow.init(
            SimTK::Array_<double>(parameters.consecutive_values, 0.0));
    lSlidingWindow.init(
            SimTK::Array_<double>(parameters.consecutive_values, 0.0));
}

void ExternalForceBasedPhaseDetector::updDetector(const Input& input) {
    // append to sliding windows
    rSlidingWindow.insert((input.rForce > parameters.threshold) ? 1 : 0);
    lSlidingWindow.insert((input.lForce > parameters.threshold) ? 1 : 0);

    // if n-last values are greater that the threshold, then return stance
    // (positive double), else return swing (negative double)
    double rPhase = (!rSlidingWindow.equal(0)) ? 1 : -1;
    double lPhase = (!lSlidingWindow.equal(0)) ? 1 : -1;

    // update detector internal state
    updDetectorState(input.t, rPhase, lPhase);
}
