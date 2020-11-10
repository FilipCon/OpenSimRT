#pragma once

#include "GRFMPrediction.h"
#include "GaitPhaseDetector.h"
#include "internal/RealTimeExports.h"

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

    IMUAccelerationBasedPhaseDetector() = default;
    IMUAccelerationBasedPhaseDetector(
            const OpenSim::Model&,
            const GRFMPrediction::Parameters& parameters);

    void updDetector(const Input& input);

 private:
};
} // namespace OpenSimRT
