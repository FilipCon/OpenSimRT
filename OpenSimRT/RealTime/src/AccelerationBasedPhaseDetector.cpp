#include "AccelerationBasedPhaseDetector.h"

#include "GRFMPrediction.h"
#include "SignalProcessing.h"

#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <algorithm>

using namespace OpenSimRT;

AccelerationBasedPhaseDetector::AccelerationBasedPhaseDetector(
        const OpenSim::Model& otherModel, const Parameters& otherParameters)
        : GaitPhaseDetector(otherParameters.windowSize),
          model(*otherModel.clone()), parameters(otherParameters) {
    posFilter = new ButterworthFilter(12, parameters.posLPFilterOrder,
                                      (2 * parameters.posLPFilterFreq) /
                                              parameters.samplingFrequency,
                                      ButterworthFilter::FilterType::LowPass,
                                      IIRFilter::InitialValuePolicy::Zero);

    velFilter = new ButterworthFilter(12, parameters.velLPFilterOrder,
                                      (2 * parameters.velLPFilterFreq) /
                                              parameters.samplingFrequency,
                                      ButterworthFilter::FilterType::LowPass,
                                      IIRFilter::InitialValuePolicy::Zero);

    accFilter = new ButterworthFilter(12, parameters.accLPFilterOrder,
                                      (2 * parameters.accLPFilterFreq) /
                                              parameters.samplingFrequency,
                                      ButterworthFilter::FilterType::LowPass,
                                      IIRFilter::InitialValuePolicy::Zero);

    posDiff = new NumericalDifferentiator(12, parameters.posDiffOrder);
    velDiff = new NumericalDifferentiator(12, parameters.velDiffOrder);

    // add station points to the model for the CoP trajectory
    heelStationR = new OpenSim::Station(
            model.getBodySet().get(parameters.rFootBodyName),
            parameters.rHeelLocationInFoot);
    heelStationL = new OpenSim::Station(
            model.getBodySet().get(parameters.lFootBodyName),
            parameters.lHeelLocationInFoot);
    toeStationR = new OpenSim::Station(
            model.getBodySet().get(parameters.rFootBodyName),
            parameters.rToeLocationInFoot);
    toeStationL = new OpenSim::Station(
            model.getBodySet().get(parameters.lFootBodyName),
            parameters.lToeLocationInFoot);
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

    // filter and differentiation
    auto x = posFilter->filter(v);
    auto xDot = velFilter->filter(posDiff->diff(input.t, x));
    auto xDDot = accFilter->filter(velDiff->diff(input.t, xDot));

    // get station and accelerations
    auto rHeelAcc = SimTK::Vec3(&xDDot[0]);
    auto lHeelAcc = SimTK::Vec3(&xDDot[3]);
    auto rToeAcc = SimTK::Vec3(&xDDot[6]);
    auto lToeAcc = SimTK::Vec3(&xDDot[9]);

    // TODO not very accurate
    double rPhase = (rToeAcc.norm() < parameters.accThreshold) ? 1 : -1;
    double lPhase = (lToeAcc.norm() < parameters.accThreshold) ? 1 : -1;

    std::cout <<    rToeAcc.norm() << std::endl;
    // update detector internal state
    updDetectorState(input.t, rPhase, lPhase);
}
