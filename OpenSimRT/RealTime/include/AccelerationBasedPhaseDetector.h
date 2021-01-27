#pragma once

#include "GRFMPrediction.h"
#include "GaitPhaseDetector.h"
#include "SignalProcessing.h"
#include "internal/RealTimeExports.h"

#include <SimTKcommon/internal/ReferencePtr.h>
#include <SimTKcommon/internal/State.h>
#include <Simulation/Model/Model.h>
#include <simmath/Differentiator.h>
#include <string>
namespace OpenSimRT {

class RealTime_API AccelerationBasedPhaseDetector : public GaitPhaseDetector {
 public:
    struct Parameters {
        double heelAccThreshold; //
        double toeAccThreshold; //

        int windowSize;
        int consecutiveValues;

        std::string rFootBodyName;
        std::string lFootBodyName;
        SimTK::Vec3 rHeelLocationInFoot;
        SimTK::Vec3 lHeelLocationInFoot;
        SimTK::Vec3 rToeLocationInFoot;
        SimTK::Vec3 lToeLocationInFoot;

        double samplingFrequency;
        double accLPFilterFreq;
        double velLPFilterFreq;
        double posLPFilterFreq;
        int accLPFilterOrder;
        int velLPFilterOrder;
        int posLPFilterOrder;

        int posDiffOrder;
        int velDiffOrder;
    };

    AccelerationBasedPhaseDetector(const OpenSim::Model& otherModel,
                                   const Parameters& parameters);
    void updDetector(const GRFMPrediction::Input& input);

 private:
    OpenSim::Model model;
    SimTK::State state;
    Parameters parameters;

    SlidingWindow<SimTK::Vec2> rSlidingWindow;
    SlidingWindow<SimTK::Vec2> lSlidingWindow;

    SimTK::ReferencePtr<ButterworthFilter> posFilter;
    SimTK::ReferencePtr<ButterworthFilter> velFilter;
    SimTK::ReferencePtr<ButterworthFilter> accFilter;
    SimTK::ReferencePtr<NumericalDifferentiator> posDiff;
    SimTK::ReferencePtr<NumericalDifferentiator> velDiff;

    // station points
    SimTK::ReferencePtr<OpenSim::Station> heelStationR;
    SimTK::ReferencePtr<OpenSim::Station> heelStationL;
    SimTK::ReferencePtr<OpenSim::Station> toeStationR;
    SimTK::ReferencePtr<OpenSim::Station> toeStationL;
};
} // namespace OpenSimRT
