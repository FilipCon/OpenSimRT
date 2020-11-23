#pragma once

#include "GRFMPrediction.h"
#include "GaitPhaseDetector.h"
#include "SignalProcessing.h"
#include "internal/RealTimeExports.h"

#include <SimTKcommon/internal/ReferencePtr.h>
#include <Simulation/Model/Model.h>
namespace OpenSimRT {

class RealTime_API IMUAccelerationBasedPhaseDetector
        : public GaitPhaseDetector {
 public:
    struct Input {
        double time;
        SimTK::Vec3 rightAcceleration;
        SimTK::Vec3 leftAcceleration;
    };

    struct Parameters {
        // stance/swing threshold
        double threshold;

        // filter parameters
        int filterOrder;
        double samplingFreq;
        double lpCutoffFreq;
        double hpCutoffFreq;
    };

    IMUAccelerationBasedPhaseDetector() = default;
    IMUAccelerationBasedPhaseDetector(const Parameters& parameters);

    void updDetector(const Input& input);

 private:
    SimTK::ReferencePtr<ButterworthFilter> bwLPFilter;
    SimTK::ReferencePtr<ButterworthFilter> bwHPFilter;

    Parameters parameters;
};
} // namespace OpenSimRT
