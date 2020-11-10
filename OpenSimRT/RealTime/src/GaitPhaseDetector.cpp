#include "GaitPhaseDetector.h"

using namespace OpenSimRT;

GaitPhaseDetector::GaitPhaseDetector(
        const OpenSim::Model& otherModel,
        const GRFMPrediction::Parameters& otherParameters)
        : model(*otherModel.clone()), parameters(otherParameters), Ths(-1),
          Tto(-1), Tds(-1), Tss(-1) {
    // initialize and set size of legphase sliding window
    phaseWindowR.init({GaitPhaseState::LegPhase::INVALID,
                       GaitPhaseState::LegPhase::INVALID});
    phaseWindowL.init({GaitPhaseState::LegPhase::INVALID,
                       GaitPhaseState::LegPhase::INVALID});

    // init leading leg state
    leadingLeg = GaitPhaseState::LeadingLeg::INVALID;

    // define function for detecting HS - transition SWING -> STANCE
    detectHS = [](const SlidingWindow<GaitPhaseState::LegPhase>& w) {
        return (w.data[0] == GaitPhaseState::LegPhase::SWING &&
                w.data[1] == GaitPhaseState::LegPhase::STANCE)
                       ? true
                       : false;
    };

    // define function for detecting TO - transition STANCE -> SWING
    detectTO = [](const SlidingWindow<GaitPhaseState::LegPhase>& w) {
        return (w.data[0] == GaitPhaseState::LegPhase::STANCE &&
                w.data[1] == GaitPhaseState::LegPhase::SWING)
                       ? true
                       : false;
    };
};

GaitPhaseState::GaitPhase
GaitPhaseDetector::updGaitPhase(const GaitPhaseState::LegPhase& phaseR,
                                const GaitPhaseState::LegPhase& phaseL) {
    // determine current gait phase based on leg phase
    GaitPhaseState::GaitPhase phase;
    if (phaseR == GaitPhaseState::LegPhase::STANCE &&
        phaseL == GaitPhaseState::LegPhase::STANCE) {
        phase = GaitPhaseState::GaitPhase::DOUBLE_SUPPORT;

    } else if (phaseL == GaitPhaseState::LegPhase::SWING &&
               phaseR == GaitPhaseState::LegPhase::STANCE) {
        phase = GaitPhaseState::GaitPhase::LEFT_SWING;

    } else if (phaseR == GaitPhaseState::LegPhase::SWING &&
               phaseL == GaitPhaseState::LegPhase::STANCE) {
        phase = GaitPhaseState::GaitPhase::RIGHT_SWING;

    } else {
        phase = GaitPhaseState::GaitPhase::INVALID;
    }
    return phase;
}

bool GaitPhaseDetector::isDetectorReady() {
    // detector is ready when time constants, gait phase and leading leg
    // member variables are valid
    return (Ths >= 0 && Tto >= 0 && Tds >= 0 && Tss >= 0 &&
            leadingLeg != GaitPhaseState::LeadingLeg::INVALID &&
            gaitPhase != GaitPhaseState::GaitPhase::INVALID)
                   ? true
                   : false;
}

GaitPhaseState::LegPhase GaitPhaseDetector::updLegPhase(const double& x) {
    // f > threshold
    return [](const double& f) {
        if (f > 0)
            return GaitPhaseState::LegPhase::STANCE;
        else if (f <= 0)
            return GaitPhaseState::LegPhase::SWING;
        else
            return GaitPhaseState::LegPhase::INVALID;
    }(x);
}

void GaitPhaseDetector::updDetectorState(const double& t,
                                         const double& rightValue,
                                         const double& leftValue) {

    // update leg phase
    auto phaseR = updLegPhase(rightValue);
    auto phaseL = updLegPhase(leftValue);

    // push to sliding window
    phaseWindowR.insert(phaseR);
    phaseWindowL.insert(phaseL);

    // udpate time constants
    Tto = (detectTO(phaseWindowR) || detectTO(phaseWindowL)) ? t : Tto;
    Ths = (detectHS(phaseWindowR) || detectHS(phaseWindowL)) ? t : Ths;
    Tds = (Tto > Ths && Ths > 0) ? Tto - Ths : Tds;
    Tss = (Ths > Tto && Tto > 0) ? Ths - Tto : Tss;

    // udpate leading leg
    leadingLeg = [&]() {
        if (detectHS(phaseWindowR))
            return GaitPhaseState::LeadingLeg::RIGHT;
        else if (detectHS(phaseWindowL))
            return GaitPhaseState::LeadingLeg::LEFT;
        else
            return leadingLeg; // yield previous value
    }();

    // update gait phase
    gaitPhase = updGaitPhase(phaseR, phaseL);
}
