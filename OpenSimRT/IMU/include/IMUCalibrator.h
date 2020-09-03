#pragma once

#include "InverseKinematics.h"
#include "Manager.h"
#include "internal/IMUExports.h"

#include <SimTKcommon/internal/Rotation.h>
#include <Simulation/Model/Model.h>
#include <iostream>
#include <string>
#include <vector>

namespace OpenSimRT {

class IMU_API IMUCalibrator {
 public:
    IMUCalibrator(const OpenSim::Model& model, NGIMUManager* const manager,
                  const std::vector<std::string>& observationOrder);
    InverseKinematics::Input transform(const std::pair<double, std::vector<IMUData>>& quat);

    void run(const double& timeout);

 private:
    void computeAvgStaticPose();

    SimTK::ReferencePtr<NGIMUManager> m_manager;
    std::vector<std::vector<IMUData>> quatTable;
    std::vector<SimTK::Rotation> imuModelOffsets;
    std::vector<IMUData> initIMUData;
};

class IMU_API PositionTracker {
 public:
    PositionTracker(const OpenSim::Model& model,
                    const std::vector<std::string>& observationOrder);
    SimTK::Vec3 computeVelocity(const std::pair<double, std::vector<IMUData>>& data);

 private:
    OpenSim::Model model;
    std::vector<std::string> imuLabels;
};
} // namespace OpenSimRT
