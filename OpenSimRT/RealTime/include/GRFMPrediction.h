#pragma once
#include "Exception.h"
#include "internal/RealTimeExports.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <SimTKcommon/Scalar.h>
#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/Rotation.h>
#include <stdexcept>

namespace OpenSimRT {

class RealTime_API GaitPhaseDetector;

/**
 * Helper function for updating OpenSim state.
 */
template <typename T>
void updateState(const T& input, const OpenSim::Model& model,
                 SimTK::State& state, const SimTK::Stage& stage) {
    const auto& coordinateSet = model.getCoordinatesInMultibodyTreeOrder();
    if (coordinateSet.size() != input.q.size())
        THROW_EXCEPTION("Wrong dimensions");
    for (size_t i = 0; i < coordinateSet.size(); ++i) {
        coordinateSet[i]->setValue(state, input.q[i]);
        coordinateSet[i]->setSpeedValue(state, input.qDot[i]);
    }
    model.getMultibodySystem().realize(state, stage);
}

/**
 * Basic Sliding Window implementation
 */
template <typename T> struct SlidingWindow {
    SimTK::Array_<T> data; // sliding window data
    std::size_t capacity;  // sliding window size

    // set initial values
    void init(SimTK::Array_<T>&& aData) {
        data = std::forward<SimTK::Array_<T>>(aData);
        capacity = data.size();
    }

    // insert element
    void insert(const T& x) {
        if (data.size() >= capacity) data.erase(data.begin());
        data.push_back(x);
    }

    // reserve space in memory
    void setSize(const std::size_t& size) {
        capacity = size;
        data.reserve(size);
    }

    // compute mean value
    T mean() {
        return std::move(1.0 * std::accumulate(data.begin(), data.end(), T()) /
                         int(data.size()));
    }

    // Determine if all elements in the window are equal to input value
    bool equal(const T& x) {
        for (const auto& e : data) {
            if (e != x) return false;
        }
        return true;
    }

    // Determine if the n first elements are equal to input value
    bool nFirstEqual(const T& x, const size_t& n) const {
        if (n > data.size()) throw std::runtime_error("Wrong input size");
        for (size_t i = 0; i < n; ++i) {
            if (data[i] != x) return false;
        }
        return true;
    }

    // Determine if the last n elements are equal to input value
    bool nLastEqual(const T& x, const size_t& n) const {
        if (n > data.size()) throw std::runtime_error("Wrong input size");
        for (size_t i = 0; i < n; ++i) {
            if (data[data.size() - i - 1] != x) return false;
        }
        return true;
    }
};
/*******************************************************************************/

// Possible gait related phases.
class RealTime_API GaitPhaseState {
 public:
    enum class LegPhase { INVALID, SWING, STANCE };

    enum class GaitPhase { INVALID, RIGHT_SWING, LEFT_SWING, DOUBLE_SUPPORT };

    enum class LeadingLeg { INVALID, RIGHT, LEFT };
};

/**
 *  Ground Reaction Force & Moment Prediction Module
 */
class RealTime_API GRFMPrediction {
    /**
     * Equations that describe the Smooth Transition Assumption.
     */
    typedef std::function<double(const double&)> TransitionFuction;

    /**
     * Function that describe the CoP trajectory during gait.
     */
    typedef std::function<SimTK::Vec3(const double&, const SimTK::Vec3&)> CoPTrajectory;

 public:
    struct Parameters {
        int directionWindowSize;
        std::string method;                             // Newton-Euler, ID
        std::string pelvisBodyName;                     // pelvis body name
        std::string rStationBodyName, lStationBodyName; // foot body Names
        SimTK::Vec3 rHeelStationLocation, lHeelStationLocation; // begin of cop
        SimTK::Vec3 rToeStationLocation, lToeStationLocation;   // end of cop
    };
    struct Input {
        double t;
        SimTK::Vector q;
        SimTK::Vector qDot;
        SimTK::Vector qDDot;
    };

    struct RealTime_API Output {
        double t;
        SimTK::Vec3 point;
        SimTK::Vec3 force;
        SimTK::Vec3 moment;
        SimTK::Vector asVector();
    };

    // constructor
    GRFMPrediction() = default;
    GRFMPrediction(const OpenSim::Model&, const Parameters&,
                   GaitPhaseDetector*);

    // compute the ground reaction forces, moments and cop
    std::vector<Output> solve(const Input& input);

 private:
    // transition functions based on the STA
    TransitionFuction reactionComponentTransition;
    // TransitionFuction anteriorForceTransition;

    // function that describes the CoP trajectory during gait
    CoPTrajectory copPosition;

    SimTK::Vec3 totalForceAtThs;
    SimTK::Vec3 totalMomentAtThs;
    double t, Tds, Tss; // current simulation time, double support and single
                        // support period

    // gait direction based on the average direction of the pelvis anterior axis
    SlidingWindow<SimTK::Vec3> gaitDirectionBuffer;

    OpenSim::Model model;
    SimTK::State state;
    Parameters parameters;

    // gait phase detection
    SimTK::ReferencePtr<GaitPhaseDetector> gaitPhaseDetector;
    GaitPhaseState::GaitPhase gaitphase; // gait phase during simulation

    // station points forming the cop trajectory
    SimTK::ReferencePtr<OpenSim::Station> heelStationR;
    SimTK::ReferencePtr<OpenSim::Station> heelStationL;
    SimTK::ReferencePtr<OpenSim::Station> toeStationR;
    SimTK::ReferencePtr<OpenSim::Station> toeStationL;

    /**
     * Compute the average heading direction during gait based on the anterior
     * axis of the pelvis local frame.
     */
    SimTK::Rotation computeGaitDirectionRotation(const std::string& bodyName);

    /**
     * Compute the total reaction components F_ext and M_ext based either on the
     * Newton-Euler method or the by solving ID.
     */
    void computeTotalReactionComponents(const Input& input,
                                        SimTK::Vec3& totalReactionForce,
                                        SimTK::Vec3& totalReactionMoment);

    /**
     * Separate the total reaction components into R/L foot reaction components.
     */
    void seperateReactionComponents(
            const double& time, const SimTK::Vec3& totalReactionComponent,
            const SimTK::Vec3& totalReactionAtThs,
            const TransitionFuction& anteriorComponentFunction,
            const TransitionFuction& verticalComponentFunction,
            const TransitionFuction& lateralComponentFunction,
            SimTK::Vec3& rightReactionComponent,
            SimTK::Vec3& leftReactionComponent);

    /**
     * Compute the CoP on each foot.
     */
    void computeReactionPoint(SimTK::Vec3& rightPoint, SimTK::Vec3& leftPoint);
};
} // namespace OpenSimRT
