#pragma once

#include "GRFMPrediction.h"
#include "GaitPhaseDetector.h"
#include "SignalProcessing.h"
#include "internal/RealTimeExports.h"

#include <SimTKcommon/internal/State.h>
#include <Simulation/Model/Model.h>
namespace OpenSimRT {

class RealTime_API AccelerationBasedPhaseDetector : public GaitPhaseDetector {
 public:
    struct Parameters {
        // stance/swing threshold
        double acc_threshold;
        double vel_threshold;
        size_t consecutive_values;
        LowPassSmoothFilter::Parameters filterParameters;
    };

    AccelerationBasedPhaseDetector(const OpenSim::Model& otherModel,
                               const Parameters& parameters);

    void updDetector(const GRFMPrediction::Input& input);

 private:
    OpenSim::Model model;
    SimTK::State state;
    Parameters parameters;

    SlidingWindow<SimTK::Vec4> rSlidingWindow;
    SlidingWindow<SimTK::Vec4> lSlidingWindow;

    SimTK::ReferencePtr<LowPassSmoothFilter> filter;

    // station points
    SimTK::ReferencePtr<OpenSim::Station> heelStationR;
    SimTK::ReferencePtr<OpenSim::Station> heelStationL;
    SimTK::ReferencePtr<OpenSim::Station> toeStationR;
    SimTK::ReferencePtr<OpenSim::Station> toeStationL;
};
} // namespace OpenSimRT
