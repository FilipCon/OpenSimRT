#include "ExternalForceBasedPhaseDetector.h"

#include "GRFMPrediction.h"

#include <algorithm>

using namespace OpenSimRT;

ExternalForceBasedPhaseDetector::ExternalForceBasedPhaseDetector(
        const Parameters& otherParameters)
        : GaitPhaseDetector(otherParameters.windowSize),
          parameters(otherParameters) {
    // rSlidingWindow.init(
    //         SimTK::Array_<double>(parameters.delay, 0.0));
    // lSlidingWindow.init(
    //         SimTK::Array_<double>(parameters.delay, 0.0));
}

void ExternalForceBasedPhaseDetector::updDetector(const Input& input) {
    // if n-last values are greater that the threshold, then return stance
    // (positive double), else return swing (negative double)
    double rPhase = (input.rForce > parameters.threshold) ? 1 : -1;
    double lPhase = (input.lForce > parameters.threshold) ? 1 : -1;

    // update detector internal state
    updDetectorState(input.t, rPhase, lPhase);
}
