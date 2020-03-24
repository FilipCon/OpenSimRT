#include "MuscleOptimization.h"

#include "Exception.h"
#include "OpenSimUtils.h"

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/Model/ForceSet.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

Matrix calculateMomentArm(const State& s, const Model& model,
                          const vector<int>& activeCoordinateIndices,
                          const vector<int>& activeMuscleIndices) {
    const auto& coordinates = model.getCoordinatesInMultibodyTreeOrder();
    const auto& muscles = model.getMuscles();
    Matrix R(activeCoordinateIndices.size(), activeMuscleIndices.size());
    for (int i = 0; i < activeCoordinateIndices.size(); i++) {
        for (int j = 0; j < activeMuscleIndices.size(); j++) {
            const Coordinate& coord = *coordinates[i];
            R[i][j] = muscles[activeMuscleIndices[j]].computeMomentArm(
                    s, const_cast<Coordinate&>(coord));
        }
    }
    return R;
}

/*******************************************************************************/

MuscleOptimization::MuscleOptimization(
        const Model& modelOther,
        const MuscleOptimization::OptimizationParameters&
                optimizationParameters,
        const MomentArmFunctionT& momentArmFunction)
        : model(*modelOther.clone()) {
    // configure optimizer
    target = new TorqueBasedTarget(&model,
                                   optimizationParameters.objectiveExponent,
                                   momentArmFunction);
    optimizer = new Optimizer(*target, OptimizerAlgorithm::InteriorPoint);
    optimizer->setConvergenceTolerance(
            optimizationParameters.convergenceTolerance);
    optimizer->setMaxIterations(optimizationParameters.maximumIterations);
    optimizer->setDiagnosticsLevel(0); // for debugging
    optimizer->useNumericalGradient(false);
    optimizer->useNumericalJacobian(false);
    optimizer->setLimitedMemoryHistory(500);
    optimizer->setAdvancedBoolOption("warm_start", true);
    optimizer->setAdvancedRealOption("expect_infeasible_problem", false);
    optimizer->setAdvancedRealOption("obj_scaling_factor", 1);
    optimizer->setAdvancedRealOption("nlp_scaling_max_gradient", 1);
    // optimizer->setAdvancedStrOption("hessian_approximation", "exact");
    parameterSeeds = Vector(target->getNumParameters(), 0.5);
}

MuscleOptimization::Output
MuscleOptimization::solve(const MuscleOptimization::Input& input) {
    PROFILE_FUNCTION();
    try {
        target->prepareForOptimization(input);
        optimizer->optimize(parameterSeeds);
    } catch (exception& e) {
        // optimization may find a feasible solution and fail
        cout << "Failed at time: " << input.t << endl << e.what() << endl;
        // exit(-1);
    }
    auto fm = target->extractMuscleForces(parameterSeeds);
    auto am = target->extractMuscleForces(
            parameterSeeds.elementwiseDivide(target->fMax));
    return MuscleOptimization::Output{input.t, am, fm, fm};
}

TimeSeriesTable MuscleOptimization::initializeMuscleLogger() {
    auto columnNames = OpenSimUtils::getMuscleNames(model);

    TimeSeriesTable m;
    m.setColumnLabels(columnNames);
    return m;
}

/*******************************************************************************/

TorqueBasedTarget::TorqueBasedTarget(Model* model, int objectiveExponent,
                                     const MomentArmFunctionT& momentArmFunction)
        : model(model), p(objectiveExponent), calcMomentArm(momentArmFunction) {

    // add coordinate actuators for each generalized coordinate in the model
    auto& cs = model->getCoordinateSet();
    W = Matrix(cs.getSize(),cs.getSize(), 0.0);
    for (int i = 0; i < cs.getSize(); i++) {
        const auto& coordName = cs.get(i).getName();
        string name = coordName + "_reserve";
        W(i,i) = (coordName.rfind("pelvis", 0) == 0) ? 1 : 0;
        if (!model->getForceSet().contains(name)) {
            CoordinateActuator* actuator = new CoordinateActuator();
            actuator->setCoordinate(&cs.get(i));
            actuator->setName(name);
            actuator->setMinControl(-SimTK::Infinity);
            actuator->setMaxControl(SimTK::Infinity);
            actuator->setOptimalForce(1);
            model->addForce(actuator);
        }
    }

    // number of inequalities (minus pelvis torques, which are
    // non-physiological)
    setNumEqualityConstraints(cs.getSize());
    setNumLinearEqualityConstraints(cs.getSize());

    // parameter bounds
    nma = 0;
    auto& as = model->getActuators();
    int na = as.getSize();
    fMax = Vector(na, 0.0);
    Vector lowerBounds(na, 0.0), upperBounds(na, 0.0);
    for (int i = 0; i < na; ++i) {
        auto muscle = dynamic_cast<const Muscle*>(&as[i]);
        auto pathAct = dynamic_cast<const PathActuator*>(&as[i]);
        auto coordAct = dynamic_cast<const CoordinateActuator*>(&as[i]);
        if (muscle) {
            nma++;
            fMax[i] = muscle->getMaxIsometricForce();
            lowerBounds[i] = 0.0;
            upperBounds[i] = Infinity;
        } else if (pathAct) {
            nma++;
            fMax[i] = pathAct->getOptimalForce();
            lowerBounds[i] = 0.0;
            upperBounds[i] = Infinity;
        } else if (coordAct) {
            fMax[i] = coordAct->getOptimalForce();
            lowerBounds[i] = coordAct->getMinControl() * fMax[i];
            upperBounds[i] = coordAct->getMaxControl() * fMax[i];
        } else {
            THROW_EXCEPTION("unsupported type of actuator");
        }
    }
    setNumParameters(na);
    setParameterLimits(lowerBounds, upperBounds);
}

void TorqueBasedTarget::prepareForOptimization(
        const MuscleOptimization::Input& input) {
    PROFILE_FUNCTION();
    tau = input.tau;
    R = calcMomentArm(input.q);
    // R(0, 0, 6, R.ncol()) = 0;
}
Vector TorqueBasedTarget::extractMuscleForces(const Vector& x) const {
    return x(0, nma);
}

Vector TorqueBasedTarget::extractResidualForces(const Vector& x) const {
    return x(nma, getNumParameters() - nma);
}

int TorqueBasedTarget::objectiveFunc(const Vector& x, bool newCoefficients,
                                     Real& rP) const {
    rP = 0.0;
    for (int i = 0; i < getNumParameters(); ++i) {
        rP += pow(x[i] / fMax[i], p);
    }
    return 0;
}

int TorqueBasedTarget::gradientFunc(const Vector& x, bool newCoefficients,
                                    Vector& gradient) const {
    for (int i = 0; i < getNumParameters(); ++i) {
        gradient[i] = p * pow(x[i] / fMax[i], p - 1);
    }
    return 0;
}

int TorqueBasedTarget::constraintFunc(const Vector& x, bool newCoefficients,
                                      Vector& constraints) const {
    constraints = R * extractMuscleForces(x) +
        extractResidualForces(x).elementwiseMultiply(W.diag()) - tau;
    return 0;
}

int TorqueBasedTarget::constraintJacobian(const Vector& x, bool newCoefficients,
                                          Matrix& jac) const {
    int n = getNumConstraints();
    jac(0, 0, n, nma) = R;
    jac(0, nma, n, n) = W;
    return 0;
}

/*******************************************************************************/
