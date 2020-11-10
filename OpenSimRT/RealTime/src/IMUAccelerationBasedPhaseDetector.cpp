#include "IMUAccelerationBasedPhaseDetector.h"

using namespace OpenSimRT;

IMUAccelerationBasedPhaseDetector::IMUAccelerationBasedPhaseDetector(
        const OpenSim::Model& model, const GRFMPrediction::Parameters& parameters)
        : GaitPhaseDetector(model, parameters) {
}

void IMUAccelerationBasedPhaseDetector::updDetector(const Input& input) {
    updDetectorState(input.time,
                     parameters.threshold - input.rightAcceleration.norm(),
                     parameters.threshold - input.leftAcceleration.norm());
}
