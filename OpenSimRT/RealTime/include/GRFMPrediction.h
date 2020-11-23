#pragma once
#include "Exception.h"
#include "internal/RealTimeExports.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <SimTKcommon/Scalar.h>
#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/Rotation.h>
#include <SimTKcommon/internal/UnitVec.h>
#include <initializer_list>
#include <stdexcept>

namespace OpenSimRT {

class RealTime_API GaitPhaseDetector;

// helper function for updating opensim state
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
    state.updTime() = input.t;
    model.getMultibodySystem().realize(state, stage);
}

// basic sliding window implementation
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
        if (data.size() == capacity) data.erase(data.begin());
        data.push_back(x);
    }

    // reserve space in memory
    void setSize(const std::size_t& size) {
        capacity = size;
        data.reserve(size);
    }

    // compute mean value
    T mean() {
        return 1.0 * std::accumulate(data.begin(), data.end(), T()) /
               int(data.size());
    }

    bool equal(const T& x) {
        for (const auto& e : data) {
            if (e != x) return false;
        }
        return true;
    }

    bool nFirstEqual(const T& x, const size_t& n) const {
        if (n > data.size()) throw std::runtime_error("Wrong input size");
        for (size_t i = 0; i < n; ++i) {
            if (data[i] != x) return false;
        }
        return true;
    }
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

// Ground Reaction Force & Moment Prediction Module
class RealTime_API GRFMPrediction {
    typedef std::function<double(const double&)> TransitionFuction;
    typedef std::function<SimTK::Vec3(const double&, const SimTK::Vec3&)>
            CoPTrajectory;

 public:
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

    struct Parameters {
        double threshold = 0; // threshold
        SimTK::Vec3 plane_origin = SimTK::Vec3(0);
        SimTK::Vec3 plane_normal = SimTK::Vec3(0, 1, 0);
    } parameters;

    // constructor
    GRFMPrediction() = default;
    GRFMPrediction(const OpenSim::Model&, const Parameters&,
                   GaitPhaseDetector*);

    // compute the ground reaction forces, moments and cop
    std::vector<Output> solve(const Input& input);

 private:
    // transition functions based on the STA
    TransitionFuction anteriorForceTransition;
    TransitionFuction verticalForceTransition;
    TransitionFuction lateralForceTransition;
    TransitionFuction exponentialTransition;
    // TODO experiment with different functions
    // TODO implement for moments if required

    // function that describes the CoP trajectory during gait
    CoPTrajectory copPosition;

    double t, Tds, Tss; // current simulation time, double support and single
                        // support period

    // gait direction based on the average pelvis of the pelvis local frame
    SlidingWindow<SimTK::Vec3> gaitDirectionBuffer;

    OpenSim::Model model;
    SimTK::State state;

    // gait phase detection
    SimTK::ReferencePtr<GaitPhaseDetector> gaitPhaseDetector;
    GaitPhaseState::GaitPhase gaitphase; // gait phase during simulation

    // station points forming the cop trajectory
    SimTK::ReferencePtr<OpenSim::Station> heelStationR;
    SimTK::ReferencePtr<OpenSim::Station> heelStationL;
    SimTK::ReferencePtr<OpenSim::Station> toeStationR;
    SimTK::ReferencePtr<OpenSim::Station> toeStationL;

    SimTK::Rotation computeGaitDirectionRotation(const std::string& bodyName);
    void computeTotalReactionComponents(const Input& input,
                                        SimTK::Vec3& totalReactionForce,
                                        SimTK::Vec3& totalReactionMoment);

    // separate total reaction into R/L foot reaction components
    void seperateReactionComponents(
            const SimTK::Vec3& totalReactionComponent,
            const TransitionFuction& anteriorComponentFunction,
            const TransitionFuction& verticalComponentFunction,
            const TransitionFuction& lateralComponentFunction,
            SimTK::Vec3& rightReactionComponent,
            SimTK::Vec3& leftReactionComponent);

    // compute the CoP on each foot
    void computeReactionPoint(SimTK::Vec3& rightPoint, SimTK::Vec3& leftPoint);
};
} // namespace OpenSimRT
