#pragma once

#include "GRFMPrediction.h"
#include "GaitPhaseDetector.h"
#include "internal/RealTimeExports.h"

namespace OpenSimRT {

class RealTime_API ExternalForceBasedPhaseDetector : public GaitPhaseDetector {
 public:
    struct Input {
        double t;
        double rForce;
        double lForce;
    };

    struct Parameters {
        // stance/swing threshold
        double threshold;
        int windowSize;
    };

    ExternalForceBasedPhaseDetector(const Parameters& parameters);

    void updDetector(const Input& input);

 private:
    Parameters parameters;

    // SlidingWindow<double> rSlidingWindow;
    // SlidingWindow<double> lSlidingWindow;
};
} // namespace OpenSimRT
