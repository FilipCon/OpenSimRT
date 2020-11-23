#include "AccelerationBasedPhaseDetector.h"

#include "GRFMPrediction.h"
#include "SignalProcessing.h"

#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <algorithm>

using namespace OpenSimRT;

AccelerationBasedPhaseDetector::AccelerationBasedPhaseDetector(
        const OpenSim::Model& otherModel, const Parameters& otherParameters)
        : GaitPhaseDetector(), model(*otherModel.clone()),
          parameters(otherParameters) {
    rSlidingWindow.init(SimTK::Array_<SimTK::Vec4>(
            parameters.consecutive_values, SimTK::Vec4(0.0)));
    lSlidingWindow.init(SimTK::Array_<SimTK::Vec4>(
            parameters.consecutive_values, SimTK::Vec4(0.0)));

    filter = new LowPassSmoothFilter(parameters.filterParameters);

    // add station points to the model for the CoP trajectory
    heelStationR = new OpenSim::Station(model.getBodySet().get("calcn_r"),
                                        SimTK::Vec3(0.014, -0.0168, -0.0055));
    heelStationL = new OpenSim::Station(model.getBodySet().get("calcn_l"),
                                        SimTK::Vec3(0.014, -0.0168, 0.0055));
    toeStationR = new OpenSim::Station(model.getBodySet().get("calcn_r"),
                                       SimTK::Vec3(0.24, -0.0168, -0.00117));
    toeStationL = new OpenSim::Station(model.getBodySet().get("calcn_l"),
                                       SimTK::Vec3(0.24, -0.0168, 0.00117));
    model.addModelComponent(heelStationR.get());
    model.addModelComponent(heelStationL.get());
    model.addModelComponent(toeStationR.get());
    model.addModelComponent(toeStationL.get());

    // initialize system
    state = model.initSystem();
}

void AccelerationBasedPhaseDetector::updDetector(
        const GRFMPrediction::Input& input) {
    // update detector simtk state
    updateState(input, model, state, SimTK::Stage::Acceleration);

    // get station position
    auto rHeelPos = heelStationR->getLocationInGround(state);
    auto lHeelPos = heelStationL->getLocationInGround(state);
    auto rToePos = toeStationR->getLocationInGround(state);
    auto lToePos = toeStationL->getLocationInGround(state);

    // prepare for filtering
    SimTK::Vector v(12);
    v(0, 3) = SimTK::Vector(rHeelPos);
    v(3, 3) = SimTK::Vector(lHeelPos);
    v(6, 3) = SimTK::Vector(rToePos);
    v(9, 3) = SimTK::Vector(lToePos);

    // filter position
    auto filtered = filter->filter({input.t, SimTK::Vector(v)});
    auto x = filtered.x;
    auto xDot = filtered.xDot;
    auto xDDot = filtered.xDDot;

    // get station velocities and accelerations
    auto rHeelVel = SimTK::Vec3(&xDot[0]);
    auto lHeelVel = SimTK::Vec3(&xDot[3]);
    auto rToeVel = SimTK::Vec3(&xDot[6]);
    auto lToeVel = SimTK::Vec3(&xDot[9]);
    auto rHeelAcc = SimTK::Vec3(&xDDot[0]);
    auto lHeelAcc = SimTK::Vec3(&xDDot[3]);
    auto rToeAcc = SimTK::Vec3(&xDDot[6]);
    auto lToeAcc = SimTK::Vec3(&xDDot[9]);

    // append to sliding windows
    rSlidingWindow.insert(SimTK::Vec4(
            (rToeAcc.norm() - parameters.acc_threshold > 0) ? 1 : 0,
            (rHeelAcc.norm() - parameters.acc_threshold > 0) ? 1 : 0,
            (rToeVel.norm() - parameters.vel_threshold > 0) ? 1 : 0,
            (rHeelVel.norm() - parameters.vel_threshold > 0) ? 1 : 0));
    lSlidingWindow.insert(SimTK::Vec4(
            (lToeAcc.norm() - parameters.acc_threshold > 0) ? 1 : 0,
            (lHeelAcc.norm() - parameters.acc_threshold > 0) ? 1 : 0,
            (lToeVel.norm() - parameters.vel_threshold > 0) ? 1 : 0,
            (lHeelVel.norm() - parameters.vel_threshold > 0) ? 1 : 0));

    // if the window has consecutive values equal to Vec4(0), then legPhase =
    // stance (posistive double). Else legPhase = swing (negative double)
    double rPhase = (rSlidingWindow.equal(SimTK::Vec4(0))) ? 1 : -1;
    double lPhase = (lSlidingWindow.equal(SimTK::Vec4(0))) ? 1 : -1;

    if (!filtered.isValid) rPhase = lPhase = SimTK::NaN;

    // std::cout << lToeAcc.norm() << " ";

    // update detector internal state
    updDetectorState(input.t, rPhase, lPhase);
}
