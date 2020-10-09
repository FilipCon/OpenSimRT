#include "IMUCalibrator.h"

#include "Exception.h"
#include "InverseKinematics.h"
#include "MahonyAHRS.h"
#include "NGIMUData.h"
#include "RealTimeAnalysis.h"
#include "SignalProcessing.h"

#include <Common/Exception.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <SimTKcommon/Constants.h>
#include <SimTKcommon/Scalar.h>
#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/CoordinateAxis.h>
#include <SimTKcommon/internal/Quaternion.h>
#include <SimTKcommon/internal/Rotation.h>
#include <SimTKcommon/internal/Transform.h>
#include <SimTKcommon/internal/UnitVec.h>
#include <SimTKcommon/internal/VectorMath.h>
#include <Simulation/Model/Model.h>
#include <Simulation/Model/PhysicalFrame.h>
#include <Simulation/Model/PhysicalOffsetFrame.h>
#include <Simulation/SimbodyEngine/Coordinate.h>
#include <Simulation/SimbodyEngine/Joint.h>
#include <chrono>
#include <cmath>
#include <iterator>
#include <numeric>
#include <string>
#include <vector>

using namespace OpenSimRT;
using namespace OpenSim;
using namespace SimTK;
using namespace std;

// // hamilton quaternion product
// template <typename A, typename B>
// static inline SimTK::Quaternion quaternProd(const A& a, const B& b) {
//     SimTK::Quaternion c;
//     c[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
//     c[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
//     c[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
//     c[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
//     return c;
// }

// SimTK::Quaternion quatConjugate(const SimTK::Quaternion& q) {
//     return SimTK::Quaternion(q[0], -q[1], -q[2], -q[3]);
// }

// // compute the projection of a point or vector on a arbitrary plane
// static inline Vec3 projectionOnPlane(const Vec3& point, const Vec3&
// planeOrigin,
//                                      const Vec3& planeNormal) {
//     return point - dot(point - planeOrigin, planeNormal) * planeNormal;
// }

// /**
//    Decompose the rotation on to 2 parts.
//    1. Twist - rotation around the "direction" vector
//    2. Swing - rotation around axis that is perpendicular to "direction"
//    vector The rotation can be composed back by rotation = swing * twist

//    has singularity in case of swing_rotation close to 180 degrees rotation.
//    if the input quaternion is of non-unit length, the outputs are non-unit as
//    well otherwise, outputs are both unit
// */
// static inline Quaternion quaternDecomposition(const Quaternion& q,
//                                               const Vec3& direction) {
//     UnitVec3 ra(q[1], q[2], q[3]);                  // rotation axis
//     auto p = SimTK::dot(ra, direction) * direction; //  project v1 on to v2
//     auto twist = Quaternion(q[0], p[0], p[1], p[2]).normalize();
//     // auto swing = Quaternion(quaternProd(q, twist));
//     return twist;
// }

/******************************************************************************/
IMUCalibrator::IMUCalibrator(const Model& otherModel,
                             InputDriver<NGIMUData>* const driver,
                             const vector<string>& observationOrder)
    : m_driver(driver), model(*otherModel.clone()), R_heading(Rotation()) {
    // copy observation order list
    imuBodiesObservationOrder = std::vector<std::string>(
            observationOrder.begin(), observationOrder.end());

    // set rotation from imu ground to opensim ground
    R_GoGi = Rotation(-SimTK::Pi / 2, SimTK::YAxis) *
             Rotation(-SimTK::Pi / 2, SimTK::XAxis);

    // initialize system
    state = model.initSystem();
    model.realizePosition(state);

    // get default model pose body orientation in ground
    for (const auto& label : imuBodiesObservationOrder) {
        const PhysicalFrame* frame = nullptr;
        if ((frame = model.findComponent<PhysicalFrame>(label))) {
            imuBodiesInGround[label] = frame->getTransformInGround(state).R();
        }
    }
}

void IMUCalibrator::computeheadingRotation(
        const std::string& baseImuName,
        const SimTK::CoordinateDirection baseHeadingDirection) {

    // compute heading vector
    auto headingRotationVec =
            computeHeadingCorrection(baseImuName, baseHeadingDirection);

    // set heading rotation
    R_heading =
            Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence,
                     headingRotationVec[0], SimTK::XAxis, headingRotationVec[1],
                     SimTK::YAxis, headingRotationVec[2], SimTK::ZAxis);
}

SimTK::Vec3 IMUCalibrator::computeHeadingCorrection(
        const std::string& baseImuName,
        const SimTK::CoordinateDirection baseHeadingDirection) {
    auto baseBodyIndex = std::distance(
            imuBodiesObservationOrder.begin(),
            std::find(imuBodiesObservationOrder.begin(),
                      imuBodiesObservationOrder.end(), baseImuName));

    const auto& q0 = initIMUData[baseBodyIndex].quaternion.q;
    const auto base_R = R_GoGi * ~Rotation(q0);

    UnitVec3 baseSegmentXheading = base_R(baseHeadingDirection.getAxis());
    if (baseHeadingDirection.getDirection() < 0)
        baseSegmentXheading = baseSegmentXheading.negate();
    bool baseFrameFound = false;

    const Frame* baseFrame = nullptr;
    for (int j = 0; j < model.getNumJoints() && !baseFrameFound; ++j) {
        auto& joint = model.getJointSet().get(j);

        // look for body whose parent is ground
        if (joint.getParentFrame().findBaseFrame() == model.getGround()) {
            baseFrame = &(joint.getChildFrame().findBaseFrame());
            baseFrameFound = true;
            break;
        }
    }

    OPENSIM_THROW_IF(!baseFrameFound, OpenSim::Exception,
                     "No base segment was found");

    Vec3 baseFrameX = UnitVec3(1, 0, 0);
    const SimTK::Transform& baseXForm = baseFrame->getTransformInGround(state);
    Vec3 baseFrameXInGround = baseXForm.xformFrameVecToBase(baseFrameX);

    auto angularDifference = acos(~baseSegmentXheading * baseFrameXInGround);

    // compute sign
    auto xproduct = baseFrameXInGround % baseSegmentXheading;
    if (xproduct.get(1) > 0) { angularDifference *= -1; }

    return Vec3(0, angularDifference, 0);
}

void IMUCalibrator::calibrateIMUTasks(
        vector<InverseKinematics::IMUTask>& imuTasks) {
    for (int i = 0; i < initIMUData.size(); ++i) {
        const auto& q0 = initIMUData[i].quaternion.q;
        const auto R0 = R_heading * R_GoGi * ~Rotation(q0);

        const auto& bodyName = imuTasks[i].body;
        const auto R_FB = ~imuBodiesInGround[bodyName] * R0;

        imuTasks[i].orientation = std::move(R_FB);
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
        const auto R = R_heading * R_GoGi * ~Rotation(q);
        input.imuObservations.push_back(R);
    }

    // marker data
    for (int i = 0; i < markerData.size(); ++i) {
        input.markerObservations.push_back(markerData[i]);
    }
    return input;
}

void IMUCalibrator::record(const double& timeout) {
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

void IMUCalibrator::computeAvgStaticPose() {
    initIMUData = *(quatTable.end() - 1); //// TODO: compute actual average
}
