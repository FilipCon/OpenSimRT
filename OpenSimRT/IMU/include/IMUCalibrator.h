#pragma once

#include "InverseKinematics.h"
#include "NGIMUInputDriver.h"
#include "MahonyAHRS.h"
#include "internal/IMUExports.h"
#include "SignalProcessing.h"
#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/Rotation.h>
#include <Simulation/Model/Model.h>
#include <iostream>
#include <string>
#include <vector>

namespace OpenSimRT {

class IMU_API IMUCalibrator {
 public:
    IMUCalibrator(const OpenSim::Model& model, NGIMUInputDriver* const manager,
                  const std::vector<std::string>& observationOrder);
    InverseKinematics::Input
    transform(const std::pair<double, std::vector<NGIMUData>>& quat,
              const std::vector<SimTK::Vec3>& markerData);

    void run(const double& timeout);

 private:
    void computeAvgStaticPose();

    SimTK::ReferencePtr<NGIMUInputDriver> m_driver;
    std::vector<std::vector<NGIMUData>> quatTable;
    std::vector<SimTK::Rotation> imuModelOffsets;
    std::vector<NGIMUData> initIMUData;
};



class IMU_API PositionTracker {
 public:
    PositionTracker(const OpenSim::Model& model,
                    const std::vector<std::string>& observationOrder);
    SimTK::Vec3
    computePosition(const std::pair<double, std::vector<NGIMUData>>& data);
    // SimTK::Vec3 computePosition(const SimTK::Vec3& vel, const double& t);

 private:
    OpenSim::Model model;
    std::vector<std::string> imuLabels;
    // SimTK::ReferencePtr<MahonyAHRS> ahrs;
    SimTK::ReferencePtr<NumericalIntegrator> accelerationIntegrator;
    SimTK::ReferencePtr<NumericalIntegrator> velocityIntegrator;

    SimTK::ReferencePtr<ButterworthFilter> accelerationHPFilter;
    SimTK::ReferencePtr<ButterworthFilter> accelerationLPFilter;
    SimTK::ReferencePtr<ButterworthFilter> velocityHPFilter;
    SimTK::ReferencePtr<ButterworthFilter> positionHPFilter;

    SimTK::Rotation modelOffsetRot;
    };
} // namespace OpenSimRT
