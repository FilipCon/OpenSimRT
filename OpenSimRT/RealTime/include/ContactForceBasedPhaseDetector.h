#pragma once
#include "GaitPhaseDetector.h"

#include <OpenSim/Simulation/Model/HuntCrossleyForce.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSimRT {
// Gait phase detector implementation based on forces between contact surfaces
class RealTime_API ContactForceBasedPhaseDetector : public GaitPhaseDetector {
 public:
    struct Parameters {
        double threshold;
        int windowSize;
        SimTK::Vec3 plane_origin;
    };

    // constructor
    ContactForceBasedPhaseDetector(const OpenSim::Model&,
                                   const Parameters& parameters);

    void updDetector(const GRFMPrediction::Input& input);

 private:
    // contact force elements added to a copy of the original model
    SimTK::ReferencePtr<OpenSim::HuntCrossleyForce> rightContactForce;
    SimTK::ReferencePtr<OpenSim::HuntCrossleyForce> leftContactForce;

    OpenSim::Model model;
    SimTK::State state;
    Parameters parameters;
};
} // namespace OpenSimRT
