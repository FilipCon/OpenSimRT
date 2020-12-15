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
    template <typename T>
    using DetectEventFunction = std::function<bool(const SlidingWindow<T>&)>;

    struct TimeConstant {
        double right = -1;
        double left = -1;
    };

    GaitPhaseDetector(const int& windowSize);
    virtual ~GaitPhaseDetector() = default;

    bool isDetectorReady();
    GaitPhaseState::GaitPhase getPhase() { return gaitPhase; };
    const GaitPhaseState::LeadingLeg getLeadingLeg() { return leadingLeg; };
    const double getHeelStrikeTime() {
        return (Ths.right > Ths.left) ? Ths.right : Ths.left;
    };
    const double getToeOffTime() {
        return (Tto.right > Tto.left) ? Tto.right : Tto.left;
    };
    const double getDoubleSupportDuration() { return Tds; };
    const double getSingleSupportDuration() { return Tss; };

 protected:
    void updDetectorState(const double&, const double&, const double&);

    // update gait phase based on leg phase
    GaitPhaseState::GaitPhase updGaitPhase(const GaitPhaseState::LegPhase&,
                                           const GaitPhaseState::LegPhase&);

    // update leg phase
    GaitPhaseState::LegPhase updLegPhase(const double&);

    // function to determine heel-strike
    DetectEventFunction<GaitPhaseState::LegPhase> detectHS;
    // function to determine toe-off
    DetectEventFunction<GaitPhaseState::LegPhase> detectTO;

    GaitPhaseState::LeadingLeg leadingLeg;
    SlidingWindow<GaitPhaseState::LegPhase> phaseWindowR, phaseWindowL;

    // time constants. DS and SS time-period, and exact time of HS and TO
    // events
    TimeConstant Ths, Tto;
    double Tds, Tss;

    int _windowSize;
    // current gait phase
    GaitPhaseState::GaitPhase gaitPhase;
};

} // namespace OpenSimRT
