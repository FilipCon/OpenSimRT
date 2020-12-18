/**
 * @file JointReaction.h
 *
 * \brief Utilities for calculating the joint reaction loads.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#ifndef JOINT_REACTION_H
#define JOINT_REACTION_H

#include "InverseDynamics.h"
#include "internal/RealTimeExports.h"

#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSimRT {
/**
 * \brief Calculates the joint reaction loads as applied on child bodies
 * expressed in ground.
 *
 * TODO: implement re-express in different frame of interest
 */
class RealTime_API JointReaction {
 public:
    struct Input {
        double t;
        SimTK::Vector q;
        SimTK::Vector qDot;
        SimTK::Vector fm;
        std::vector<ExternalWrench::Input> externalWrenches;
    };
    struct Output {
        double t;
        SimTK::Vector_<SimTK::SpatialVec> reactionWrench; // [m, f]^T
    };

    struct JointReactionKey {
        const OpenSim::Joint* joint;
        bool isAppliedOnChild = true; // /* TODO:  */
        const OpenSim::Frame* appliedOnBody;
        const OpenSim::Frame* expressedInFrame;
    };

    OpenSim::Model model;
    SimTK::State state;
    std::vector<ExternalWrench*> externalWrenches;

 public:
    JointReaction(
            const OpenSim::Model& model,
            const std::vector<ExternalWrench::Parameters>& wrenchParameters);
    Output solve(const Input& input);
    SimTK::Vector asForceMomentPoint(const Output& jrOutput);

    /**
     * Initialize joint reaction analysis log storage. Use this to create a
     * TimeSeriesTable that can be appended with the computed joint reaction
     * loads.
     */
    OpenSim::TimeSeriesTable initializeLogger();
};
} // namespace OpenSimRT
#endif