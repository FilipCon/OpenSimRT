#pragma once
#include "GRFMPrediction.h"
#include "internal/RealTimeExports.h"

#include <SimTKcommon/internal/ReferencePtr.h>
#include <Simulation/Model/Model.h>
#include <utility>

namespace OpenSimRT {

/**
 *  Interface class for event detection algorithms and gait-cycle related state.
 */
class RealTime_API GaitPhaseDetector {
 public:
    /**
     * Compute events separately for each leg.
     */
    struct TimeConstant {
        double right = -1;
        double left = -1;
    };

    /**
     * Type that describes the methods for detecting events in a Sliding Window.
     */
    template <typename T>
    using DetectEventFunction = std::function<bool(const SlidingWindow<T>&)>;

    GaitPhaseDetector(const int& windowSize);
    virtual ~GaitPhaseDetector() = default;

    /**
     * Determine if the detector is ready when all internal state is valid.
     */
    bool isDetectorReady();

    //
    // setters and getters
    //
    GaitPhaseState::GaitPhase getPhase() { return gaitPhase; };
    const GaitPhaseState::LeadingLeg getLeadingLeg() { return leadingLeg; };

    // always return the most recent event
    //
    const double getHeelStrikeTime() {
        return (Ths.right > Ths.left) ? Ths.right : Ths.left;
    };
    const double getToeOffTime() {
        return (Tto.right > Tto.left) ? Tto.right : Tto.left;
    };
    const double getDoubleSupportDuration() { return Tds; };
    const double getSingleSupportDuration() { return Tss; };

 protected:
    /**
     * Update the detector state by passing values that determine each leg
     * phase. Positive values correspond to the STANCE phase and negative values
     * to the SWING phase.
     */
    void updDetectorState(const double& t, const double& rValue,
                          const double& lValue);

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
    double Tds, Tss, Tstance, Tswing;

    int _windowSize;
    // current gait phase
    GaitPhaseState::GaitPhase gaitPhase;
};

} // namespace OpenSimRT
