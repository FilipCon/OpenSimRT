#include "GaitPhaseDetector.h"
#include "GRFMPrediction.h"

using namespace OpenSimRT;
#define MEMORY 15

GaitPhaseDetector::GaitPhaseDetector() : Tds(-1), Tss(-1) {
    // initialize and set size of legphase sliding window
    phaseWindowR.init(SimTK::Array_<GaitPhaseState::LegPhase>(
            MEMORY, GaitPhaseState::LegPhase::INVALID));
    phaseWindowL.init(SimTK::Array_<GaitPhaseState::LegPhase>(
            MEMORY, GaitPhaseState::LegPhase::INVALID));

    // init leading leg state
    leadingLeg = GaitPhaseState::LeadingLeg::INVALID;

    // define function for detecting HS - transition SWING -> STANCE
    detectHS = [](const SlidingWindow<GaitPhaseState::LegPhase>& w) {
        return (w.nFirstEqual(GaitPhaseState::LegPhase::SWING,
                              w.data.size() - 1) &&
                w.data.back() == GaitPhaseState::LegPhase::STANCE)
                       ? true
                       : false;
    };

    // define function for detecting TO - transition STANCE -> SWING
    detectTO = [](const SlidingWindow<GaitPhaseState::LegPhase>& w) {
        return (w.nFirstEqual(GaitPhaseState::LegPhase::STANCE,
                              w.data.size() - 1) &&
                w.data.back() == GaitPhaseState::LegPhase::SWING)
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
    return (Ths.right >= 0 && Tto.right >= 0 && Tds >= 0 && Tss >= 0 &&
            Ths.left >= 0 && Tto.left >= 0 &&
            leadingLeg != GaitPhaseState::LeadingLeg::INVALID &&
            gaitPhase != GaitPhaseState::GaitPhase::INVALID)
                   ? true
                   : false;
}

GaitPhaseState::LegPhase GaitPhaseDetector::updLegPhase(const double& x) {
    GaitPhaseState::LegPhase phase;
    if (x > 0)
        phase = GaitPhaseState::LegPhase::STANCE;
    else if (x <= 0)
        phase = GaitPhaseState::LegPhase::SWING;
    else
        phase = GaitPhaseState::LegPhase::INVALID;
    return phase;
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

    // udpate leading leg
    leadingLeg = [&]() {
        if (detectHS(phaseWindowR))
            return GaitPhaseState::LeadingLeg::RIGHT;
        else if (detectHS(phaseWindowL))
            return GaitPhaseState::LeadingLeg::LEFT;
        else
            return leadingLeg; // yield previous value
    }();

    // udpate time constants
    Tto.right = (detectTO(phaseWindowR)) ? t : Tto.right;
    Tto.left = (detectTO(phaseWindowL)) ? t : Tto.left;
    Ths.right = (detectHS(phaseWindowR)) ? t : Ths.right;
    Ths.left = (detectHS(phaseWindowL)) ? t : Ths.left;
    if (Tto.left > Ths.right && leadingLeg == GaitPhaseState::LeadingLeg::RIGHT) Tds = Tto.left - Ths.right;
    if (Tto.right > Ths.left && leadingLeg == GaitPhaseState::LeadingLeg::LEFT) Tds = Tto.right - Ths.left;
    if (Ths.left > Tto.left && leadingLeg == GaitPhaseState::LeadingLeg::LEFT) Tss = Ths.left - Tto.left;
    if (Ths.right > Tto.right && leadingLeg == GaitPhaseState::LeadingLeg::RIGHT) Tss = Ths.right - Tto.right;

    // std::cout << Tto.left - Ths.right << " " << Tto.right - Ths.left <<
    // std::endl;

    // update gait phase
    gaitPhase = updGaitPhase(phaseR, phaseL);
}
