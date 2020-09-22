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

/******************************************************************************/
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
    InverseKinematics::Input input;

    // time
    input.t = imuData.first;

    // imu data
    for (int i = 0; i < imuData.second.size(); ++i) {
        const auto& q = imuData.second[i].quaternion.q; // current input
        const auto& q0 = initIMUData[i].quaternion.q;   // initial input
        // transform from NGIMU earth frame to OpenSim reference system and
        // remove offset from initial input and IMU orientations in the model
        input.imuObservations.push_back(
                (Rotation(Quaternion(q0[0], -q0[1], -q0[2], -q0[3])) *
                 imuModelOffsets[i])
                        .invert() *
                Rotation(Quaternion(q[0], -q[1], -q[2], -q[3])) *
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
    double samplingFreq = 60;
    imuLabels = observationOrder;

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
    // const auto& acc = data.second[pelvisIndex].sensors.acceleration;
    const auto& linAcc = data.second[pelvisIndex].linear.a;
    const auto& altitude = data.second[pelvisIndex].altitude.x;

    // LP filter acceleration
    const auto a_f = accelerationLPFilter->filter(
            accelerationHPFilter->filter(Vector(linAcc)));

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
