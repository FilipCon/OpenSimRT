#include "IMUCalibrator.h"

#include "Exception.h"
#include "MahonyAHRS.h"
#include "RealTimeAnalysis.h"
#include "SignalProcessing.h"

#include <SimTKcommon/Constants.h>
#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/CoordinateAxis.h>
#include <SimTKcommon/internal/Quaternion.h>
#include <SimTKcommon/internal/Rotation.h>
#include <SimTKcommon/internal/Transform.h>
#include <SimTKcommon/internal/VectorMath.h>
#include <Simulation/Model/Model.h>
#include <Simulation/Model/PhysicalOffsetFrame.h>
#include <chrono>
#include <cmath>
#include <numeric>
#include <string>
#include <vector>

using namespace OpenSimRT;
using namespace OpenSim;
using namespace SimTK;
using namespace std;

Rotation quat2rot(const Quaternion& q) {
    Mat33 R;
    R[0][0] = 2.0 * q[0] * q[0] - 1 + 2.0 * q[1] * q[1];
    R[0][1] = 2.0 * (q[1] * q[2] + q[0] * q[3]);
    R[0][2] = 2.0 * (q[1] * q[3] - q[0] * q[2]);
    R[1][0] = 2.0 * (q[1] * q[2] - q[0] * q[3]);
    R[1][1] = 2.0 * q[0] * q[0] - 1 + 2.0 * q[2] * q[2];
    R[1][2] = 2.0 * (q[2] * q[3] + q[0] * q[1]);
    R[2][0] = 2.0 * (q[1] * q[3] + q[0] * q[2]);
    R[2][1] = 2.0 * (q[2] * q[3] - q[0] * q[1]);
    R[2][2] = 2.0 * q[0] * q[0] - 1 + 2.0 * q[3] * q[3];
    return Rotation(R);
}

IMUCalibrator::IMUCalibrator(const Model& model, NGIMUInputDriver* const driver,
                             const vector<string>& imuLabels)
        : m_driver(driver) {
    for (auto& imuName : imuLabels) {
        const PhysicalOffsetFrame* imuOffset = nullptr;
        if ((imuOffset = model.findComponent<PhysicalOffsetFrame>(imuName))) {
            imuModelOffsets.push_back(
                    imuOffset->getOffsetTransform().R().invert());
        } else {
            THROW_EXCEPTION("Calibrator: PhysicalOffsetFrame does not exists "
                            "in the model.");
        }
    }
}

InverseKinematics::Input
IMUCalibrator::transform(const NGIMUInputDriver::IMUDataFrame& imuData,
                         const vector<Vec3>& markerData) {
    PROFILE_FUNCTION();
    InverseKinematics::Input input;

    // time
    input.t = imuData.first;

    // imu data
    for (int i = 0; i < imuData.second.size(); ++i) {
        const auto& q = imuData.second[i].quaternion; // current input
        const auto& q0 = initIMUData[i].quaternion;   // initial input
        // transform from NGIMU earth frame to OpenSim reference system and
        // remove offset from initial input and IMU orientations in the model
        input.imuObservations.push_back(
                (Rotation(Quaternion(q0.q1, -q0.q2, -q0.q3, -q0.q4)) *
                 imuModelOffsets[i])
                        .invert() *
                Rotation(Quaternion(q.q1, -q.q2, -q.q3, -q.q4)) *
                imuModelOffsets[i]);
    }

    // marker data
    for (int i = 0; i < markerData.size(); ++i) {
        input.markerObservations.push_back(markerData[i]);
    }
    return input;
}

void IMUCalibrator::computeAvgStaticPose() {
    initIMUData = *(quatTable.end() - 1); //// TODO: compute actual average
}

void IMUCalibrator::run(const double& timeout) {
    cout << "Recording Static Pose..." << endl;
    const auto start = chrono::steady_clock::now();
    while (std::chrono::duration_cast<chrono::seconds>(
                   chrono::steady_clock::now() - start)
                   .count() < timeout) {
        // get frame measurements
        quatTable.push_back(m_driver->getFrame().second);
    }
    computeAvgStaticPose();
}

PositionTracker::PositionTracker(
        const Model& otherModel,
        const std::vector<std::string>& observationOrder)
        : model(*otherModel.clone()) {
    double samplingFreq = 60.0;

    State state = model.initSystem();
    model.realizePosition(state);

    imuLabels = observationOrder;

    const PhysicalOffsetFrame* imuOffset = nullptr;
    imuOffset = model.findComponent<PhysicalOffsetFrame>("pelvis_imu");
    if (!imuOffset)
        THROW_EXCEPTION("Tracker: PhysicalOffsetFrame does not exists "
                        "in the model.");
    modelOffsetRot = imuOffset->getOffsetTransform().R();

    // numerical integrator
    accelerationIntegrator = new NumericalIntegrator(3);
    velocityIntegrator = new NumericalIntegrator(3);
    int filterOrder = 2;
    // construct filters
    accelerationLPFilter =
            new ButterworthFilter(filterOrder, (2 * 3) / samplingFreq,
                                  ButterworthFilter::FilterType::LowPass,
                                  IIRFilter::InitialValuePolicy::Zero);

    accelerationHPFilter =
            new ButterworthFilter(filterOrder, (2 * 0.001) / samplingFreq,
                                  ButterworthFilter::FilterType::HighPass,
                                  IIRFilter::InitialValuePolicy::Zero);

    velocityHPFilter =
            new ButterworthFilter(filterOrder, (2 * 0.1) / samplingFreq,
                                  ButterworthFilter::FilterType::HighPass,
                                  IIRFilter::InitialValuePolicy::Zero);

    positionHPFilter =
            new ButterworthFilter(filterOrder, (2 * 0.1) / samplingFreq,
                                  ButterworthFilter::FilterType::HighPass,
                                  IIRFilter::InitialValuePolicy::Zero);
}

Vec3 PositionTracker::computePosition(
        const std::pair<double, std::vector<NGIMUData>>& data) {
    // get index of pelvis in the imu label list
    const int pelvisIndex = std::distance(
            imuLabels.begin(),
            std::find(imuLabels.begin(), imuLabels.end(), "pelvis_imu"));

    const auto& t = data.first;
    const auto& acc = data.second[pelvisIndex].sensors.acceleration;
    const auto& quat = data.second[pelvisIndex].quaternion;

    const auto R = quat2rot(Quaternion(quat.q1, quat.q2, quat.q3, quat.q4));

    // get the pelvis acceleration measurements relative to earth
    auto t_acc = (R * Vec3(acc.ax, acc.ay, acc.az) - Vec3(0, 0, 1)) *
                 abs(model.getGravity()[1]);

    // filter acceleration
    const auto a_f = accelerationLPFilter->filter(
            accelerationHPFilter->filter(Vector(t_acc)));
    // const auto a_f = Vector(t_acc);

    // double p = 20;
    // auto fc = (a_f.norm() > 1 / p) ? 1 / (a_f.norm() * p) : 2;

    // velocityHPFilter->setupFilter(2,  (2 * fc) / 60,
    //                               ButterworthFilter::FilterType::HighPass,
    //                               IIRFilter::InitialValuePolicy::Signal);

    // compute velocity and filter velocity
    const auto velVector =
            velocityHPFilter->filter(accelerationIntegrator->integrate(a_f, t));

    // return velocity based on acceleration threshold
    const double threshold = 0.1;
    double range = 0.1; // [0,1]
    const auto vel = (a_f.norm() < threshold)
                             ? Vec3(velVector[0], velVector[1], velVector[2]) *
                                       pow(range, threshold - a_f.norm())
                             : Vec3(velVector[0], velVector[1], velVector[2]);

    // auto posVector = positionHPFilter->filter(
    //         velocityIntegrator->integrate(Vector(vel), t));
    auto posVector = velocityIntegrator->integrate(Vector(vel), t);

    return Vec3(0, posVector[2], 0);
}

// SimTK::Vec3 PositionTracker::computePosition(const SimTK::Vec3& vel,
//                                              const double& t) {
// }
