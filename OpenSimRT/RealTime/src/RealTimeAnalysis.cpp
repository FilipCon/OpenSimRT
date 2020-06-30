#include "RealTimeAnalysis.h"

#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/Signal.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <thread>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

Vector RealTimeAnalysis::UnfilteredData::toVector() {
    if (size() == 0) { THROW_EXCEPTION("cannot convert from empty"); }

    Vector temp(size());
    int index = 0;
    // temp[index++] = t;
    for (int i = 0; i < q.size(); ++i) { temp[index++] = q[i]; }

    for (int i = 0; i < externalWrenches.size(); ++i) {
        auto tempVec = externalWrenches[i].toVector();
        for (int j = 0; j < tempVec.size(); ++j) { temp[index++] = tempVec[j]; }
    }

    return temp;
}

void RealTimeAnalysis::FilteredData::fromVector(const double& time,
                                                const SimTK::Vector& x,
                                                const SimTK::Vector& xd,
                                                const SimTK::Vector& xdd,
                                                const int& nq) {
    this->t = time;
    this->q = Vector(nq, 0.0);
    this->qd = Vector(nq, 0.0);
    this->qdd = Vector(nq, 0.0);
    this->externalWrenches = vector<ExternalWrench::Input>{};
    for (int i = 0; i < nq; ++i) {
        this->q[i] = x[i];
        this->qd[i] = xd[i];
        this->qdd[i] = xdd[i];
    }

    auto wrenchSize = ExternalWrench::Input::size();
    int wrenchCount = (x.size() - nq) / wrenchSize;

    Vector wrenchData(wrenchSize);
    for (int i = 0; i < wrenchCount; ++i) {
        for (int j = 0; j < wrenchSize; ++j) {
            wrenchData[j] = x[j + i * wrenchSize + nq];
        }
        ExternalWrench::Input wrench;
        wrench.fromVector(wrenchData);
        this->externalWrenches.push_back(wrench);
    }
}

int RealTimeAnalysis::UnfilteredData::size() {
    return (int) (q.size() +
                  externalWrenches.size() * ExternalWrench::Input::size());
}

RealTimeAnalysis::RealTimeAnalysis(
        const OpenSim::Model& otherModel,
        const RealTimeAnalysis::Parameters& parameters)
        : model(*otherModel.clone()), parameters(parameters),
          previousAcquisitionTime(-1.0), previousProcessingTime(-1.0) {
    lowPassFilter = new LowPassSmoothFilterTS(parameters.filterParameters);

    // markerReconstruction = new MarkerReconstruction(model,
    //         parameters.ikMarkerTasks);

    inverseKinematics = new InverseKinematics(
            model, parameters.ikMarkerTasks, parameters.ikIMUTasks,
            parameters.ikConstraintsWeight, parameters.ikAccuracy);

    inverseDynamics = new InverseDynamics(model, parameters.wrenchParameters);

    muscleOptimization = new MuscleOptimization(
            model, parameters.muscleOptimizationParameters,
            parameters.momentArmFunction, new TorqueBasedTargetLinearMuscle());
    // new TorqueBasedTargetNonLinearMuscle());

    jointReaction = new JointReaction(model, parameters.wrenchParameters);

    // visualization
    if (parameters.useVisualizer) {
        visualizer = new BasicModelVisualizer(model);
        for (int i = 0; i < parameters.wrenchParameters.size(); ++i) {
            GRFDecorators.push_back(new ForceDecorator(Green, 0.001, 3));
            visualizer->addDecorationGenerator(GRFDecorators.back());
        }
        for (int i = 0; i < parameters.reactionForceOnBodies.size(); ++i) {
            reactionForceDecorators.push_back(
                    new ForceDecorator(Red, 0.0005, 3));
            visualizer->addDecorationGenerator(reactionForceDecorators.back());
        }
    }
}

void RealTimeAnalysis::run() {
    thread acquisitionThread(&RealTimeAnalysis::acquisition, this);
    thread processingThread(&RealTimeAnalysis::processing, this);
    acquisitionThread.join();
    processingThread.join();
}

void RealTimeAnalysis::acquisition() {
    PROFILE_FUNCTION();
    try {
        // // use first valid frame as initial condition.
        // markerReconstruction->init(parameters.dataAcquisitionFunction);

        UnfilteredData unfilteredData;
        while (true) {
            // get data from Vicon or other method
            auto acquisitionData = parameters.dataAcquisitionFunction();
            if (previousAcquisitionTime >= acquisitionData.IkFrame.t) {
                continue;
            }

            // update time and marker observations
            previousAcquisitionTime = acquisitionData.IkFrame.t;

            // // reconstruct possible missing markers
            // markerReconstruction->solve(acquisitionData.ikFrame.markerObservations);

            // perform IK
            auto pose = inverseKinematics->solve(acquisitionData.IkFrame);

            // filter and push to buffer
            unfilteredData.t = pose.t;
            unfilteredData.q = pose.q;
            unfilteredData.externalWrenches = acquisitionData.ExternalWrenches;

            // update internal state of lowPassFilter
            lowPassFilter->updState({pose.t, unfilteredData.toVector()});
        }
    } catch (exception& e) {
        exceptionPtr = std::current_exception();
        cout << e.what() << endl;

        lowPassFilter->newDataReady = true;
        lowPassFilter->cond.notify_one();

        cout << "Acquisition terminated" << endl;
        // visualizer->shouldTerminate = true;
    }
}

void RealTimeAnalysis::processing() {
    PROFILE_FUNCTION();
    try {
        FilteredData filteredData;
        Vector_<SpatialVec> reactionWrenches;
        Vector am, fm, residuals, reactionWrench;
        while (true) {
            if (exceptionPtr != nullptr) {
                std::rethrow_exception(exceptionPtr);
            }

            // get data from buffer
            auto data = lowPassFilter->filter();
            filteredData.fromVector(data.t, data.x, data.xDot, data.xDDot,
                                    model.getNumCoordinates());

            // solve id
            auto id = inverseDynamics->solve({filteredData.t, filteredData.q,
                                              filteredData.qd, filteredData.qdd,
                                              filteredData.externalWrenches});

            // solve so and jr
            if (parameters.solveMuscleOptimization) {
                auto so = muscleOptimization->solve({filteredData.t,
                                                     filteredData.q,
                                                     filteredData.qd, id.tau});
                am = so.am;
                fm = so.fm;
                residuals = so.residuals;

                auto jr = jointReaction->solve({filteredData.t, filteredData.q,
                                                filteredData.qd, so.fm,
                                                filteredData.externalWrenches});
                reactionWrenches = jr.reactionWrench;
                reactionWrench = jointReaction->asForceMomentPoint(jr);
            }

            // thread-safe write to output
            unique_lock<mutex> locker(_mu);
            output.t = filteredData.t;
            output.q = filteredData.q;
            output.qd = filteredData.qd;
            output.qdd = filteredData.qdd;
            output.grfRightWrench = filteredData.externalWrenches[0].toVector();
            output.grfLeftWrench = filteredData.externalWrenches[1].toVector();
            output.tau = id.tau;
            output.am = am;
            output.fm = fm;
            output.residuals = residuals;
            output.reactionWrench = reactionWrench;

            // notify main thread to read output
            _notifyParentThread = true;
            locker.unlock();
            _cond.notify_one();

            // visualization
            if (parameters.useVisualizer) {
                visualizer->update(filteredData.q, am);
                for (int i = 0; i < filteredData.externalWrenches.size(); ++i) {
                    GRFDecorators[i]->update(
                            filteredData.externalWrenches[i].point,
                            filteredData.externalWrenches[i].force);
                }
                // if (parameters.solveMuscleOptimization) {
                //     for (int i = 0; i <
                //     parameters.reactionForceOnBodies.size();
                //          ++i) {
                //         visualizer->updateReactionForceDecorator(
                //         reactionWrenches,
                //                 parameters.reactionForceOnBodies[i],
                //                 reactionForceDecorators[i]);
                //     }
                // }
            }
        }
    } catch (const std::exception& e) {
        exceptionPtr = std::current_exception();
        cout << e.what() << endl;
        _notifyParentThread = true;
        _cond.notify_one();
        cout << "Processing terminated" << endl;
        // if (visualizer->shouldTerminate == true)
    }
}

RealTimeAnalysis::Output& RealTimeAnalysis::getResults() {
    PROFILE_FUNCTION();
    unique_lock<mutex> locker(_mu);
    _cond.wait(locker, [&]() { return _notifyParentThread; });
    _notifyParentThread = false;
    return output;
}
