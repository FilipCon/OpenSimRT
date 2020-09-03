#include "IMUCalibrator.h"

#include "Exception.h"
#include "Manager.h"

#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/CoordinateAxis.h>
#include <SimTKcommon/internal/Quaternion.h>
#include <SimTKcommon/internal/Rotation.h>
#include <SimTKcommon/internal/Transform.h>
#include <Simulation/Model/Model.h>
#include <chrono>
#include <numeric>
#include <string>
#include <vector>

using namespace OpenSimRT;
using namespace OpenSim;
using namespace SimTK;
using namespace std;

IMUCalibrator::IMUCalibrator(const Model& model, NGIMUManager* const manager,
                             const vector<string>& imuLabels)
        : m_manager(manager) {
    for (auto& imuName : imuLabels) {
        const PhysicalOffsetFrame* imuOffset = nullptr;
        if ((imuOffset = model.findComponent<PhysicalOffsetFrame>(imuName))) {
            imuModelOffsets.push_back(imuOffset->getOffsetTransform().R().invert());
        } else {
            THROW_EXCEPTION("Calibrator: PhysicalOffsetFrame does not exists "
                            "in the model.");
        }
    }
}

InverseKinematics::Input
IMUCalibrator::transform(const std::pair<double,std::vector<IMUData>>& data) {

    InverseKinematics::Input input;
    for (int i = 0; i < data.second.size(); ++i) {
        const auto& q = data.second[i].quaternion;           // current input
        const auto& q0 = initIMUData[i].quaternion; // initial input
        // transform from NGIMU earth frame to OpenSim reference system and
        // remove offset from initial input and IMU orientations in the model
        input.imuObservations.push_back(
                (SimTK::Rotation(
                         SimTK::Quaternion(q0.q1, -q0.q2, -q0.q3, -q0.q4)) *
                 imuModelOffsets[i])
                        .invert() *
                SimTK::Rotation(SimTK::Quaternion(q.q1, -q.q2, -q.q3, -q.q4)) *
                imuModelOffsets[i]);
    }
    input.t = data.first; //// TODO:
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
        quatTable.push_back(m_manager->getObservations().second);
    }
    computeAvgStaticPose();
}

PositionTracker::PositionTracker(
        const Model& otherModel,
        const std::vector<std::string>& observationOrder)
        : model(*otherModel.clone()) {
    imuLabels = observationOrder;
}

Vec3 PositionTracker::computeVelocity(const std::pair<double, std::vector<IMUData>>& data) {
    auto it = std::find(imuLabels.begin(), imuLabels.end(), "pelvis_imu");
    int index = std::distance(imuLabels.begin(), it);
    auto g = model.getGravity();
    auto acc = data.second[index].sensors.acceleration;

    return  Vec3(acc.ax, acc.ay, acc.az);
}
