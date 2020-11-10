#pragma once
#include "GRFMPrediction.h"
#include "internal/RealTimeExports.h"

#include <SimTKcommon/internal/ReferencePtr.h>
#include <Simulation/Model/Model.h>
#include <utility>

namespace OpenSimRT {

// Interface class for event detection algorithms and gait-cycle related state.
class RealTime_API GaitPhaseDetector {
 public:
    using DetectEventFunction =
            std::function<bool(const SlidingWindow<GaitPhaseState::LegPhase>&)>;

    GaitPhaseDetector() = default;
    GaitPhaseDetector(const OpenSim::Model& otherModel,
                      const GRFMPrediction::Parameters& otherParameters);
    virtual ~GaitPhaseDetector() = default;

    bool isDetectorReady();
    GaitPhaseState::GaitPhase getPhase() { return gaitPhase; };
    const GaitPhaseState::LeadingLeg getLeadingLeg() { return leadingLeg; };
    const double getHeelStrikeTime() { return Ths; };
    const double getToeOffTime() { return Tto; };
    const double getDoubleSupportDuration() { return Tds; };
    const double getSingleSupportDuration() { return Tss; };

 protected:
    void updDetectorState(const double&, const double&, const double&);

    // update gait phase based on leg phase
    GaitPhaseState::GaitPhase updGaitPhase(const GaitPhaseState::LegPhase&,
                                           const GaitPhaseState::LegPhase&);

    // update leg phase
    GaitPhaseState::LegPhase updLegPhase(const double&);

    DetectEventFunction detectHS; // function to determine heel-strike
    DetectEventFunction detectTO; // function to determine toe-off

    GaitPhaseState::LeadingLeg leadingLeg;
    SlidingWindow<GaitPhaseState::LegPhase> phaseWindowR, phaseWindowL;

    // time constants. DS and SS time-period, and exact time of HS and TO
    // events
    double Ths, Tto, Tds, Tss;

    // current gait phase
    OpenSim::Model model;
    SimTK::State state;
    GRFMPrediction::Parameters parameters;
    GaitPhaseState::GaitPhase gaitPhase;
};

} // namespace OpenSimRT
