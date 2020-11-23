#ifndef REAL_TIME_ANALYSIS_H
#define REAL_TIME_ANALYSIS_H

#include "OpenSimUtils.h"
#include "SignalProcessing.h"
#include "Simulation.h"
#include "Visualization.h"
#include "internal/RealTimeExports.h"

#include <iostream>

namespace OpenSimRT {
/**
 * \brief TODO
 */
struct RealTime_API MotionCaptureInput {
    InverseKinematics::Input IkFrame;
    std::vector<ExternalWrench::Input> ExternalWrenches;
};

/**
 * \brief A higher order function that is an interface between the motion
 * capture system and the InverseKinematics input. Termination of the
 * acquisition should be handled by throwing an exception.
 */
typedef std::function<MotionCaptureInput()> DataAcquisitionFunction;

/**
 * \brief TODO
 */
class RealTime_API RealTimeAnalysis {
 public:
    struct UnfilteredData {
        double t;
        SimTK::Vector q;
        std::vector<ExternalWrench::Input> externalWrenches;
        SimTK::Vector toVector();
        int size();
    };

    struct FilteredData {
        double t;
        SimTK::Vector q;
        SimTK::Vector qd;
        SimTK::Vector qdd;
        std::vector<ExternalWrench::Input> externalWrenches;

        // vector x contains first nq coordinate values and the rest are wrench
        // values
        void fromVector(const double& time, const SimTK::Vector& x,
                        const SimTK::Vector& xd, const SimTK::Vector& xdd,
                        const int& nq);
    };

    struct Output {
        // filtered IK
        double t;
        SimTK::Vector q;
        SimTK::Vector qd;
        SimTK::Vector qdd;
        SimTK::Vector grfRightWrench;
        SimTK::Vector grfLeftWrench;

        // ID
        SimTK::Vector tau;

        // SO
        SimTK::Vector am;
        SimTK::Vector fm;
        SimTK::Vector residuals;

        // JRA
        SimTK::Vector reactionWrench;
    } output;

    struct Parameters {
        bool useVisualizer;
        bool solveMuscleOptimization;
        std::vector<InverseKinematics::MarkerTask> ikMarkerTasks;
        std::vector<InverseKinematics::IMUTask> ikIMUTasks;
        double ikConstraintsWeight;
        double ikAccuracy;
        std::vector<ExternalWrench::Parameters> wrenchParameters;
        MuscleOptimization::OptimizationParameters muscleOptimizationParameters;
        LowPassSmoothFilterTS::Parameters filterParameters;
        MomentArmFunctionT momentArmFunction;
        DataAcquisitionFunction dataAcquisitionFunction;
        std::vector<std::string> reactionForceOnBodies;
    } parameters;

    // modules
    SimTK::ReferencePtr<LowPassSmoothFilterTS> lowPassFilter;
    SimTK::ReferencePtr<InverseKinematics> inverseKinematics;
    SimTK::ReferencePtr<InverseDynamics> inverseDynamics;
    SimTK::ReferencePtr<MuscleOptimization> muscleOptimization;
    SimTK::ReferencePtr<JointReaction> jointReaction;

    // visualizer
    SimTK::ReferencePtr<BasicModelVisualizer> visualizer;
    std::vector<ForceDecorator*> GRFDecorators;
    std::vector<ForceDecorator*> reactionForceDecorators;

    // "hot potato" exception pointer to terminate threads
    inline static std::exception_ptr exceptionPtr = nullptr;

 public:
    RealTimeAnalysis(const OpenSim::Model& model, const Parameters& parameters);
    void run();
    void acquisition();
    void processing();

    Output getResults();

 private:
    OpenSim::Model model;
    double previousAcquisitionTime;
    double previousProcessingTime;

    std::mutex _mu;
    std::condition_variable _cond;
    bool _notifyParentThread = false;
};
} // namespace OpenSimRT
#endif
