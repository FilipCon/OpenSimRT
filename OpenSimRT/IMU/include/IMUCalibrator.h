#pragma once

#include "InverseKinematics.h"
#include "NGIMUData.h"
#include "NGIMUInputDriver.h"
#include "internal/IMUExports.h"

#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/CoordinateAxis.h>
#include <SimTKcommon/internal/Quaternion.h>
#include <SimTKcommon/internal/ReferencePtr.h>
#include <SimTKcommon/internal/Rotation.h>
#include <SimTKcommon/internal/State.h>
#include <Simulation/Model/Model.h>
#include <Simulation/Model/PhysicalFrame.h>
#include <iostream>
#include <string>
#include <vector>

namespace OpenSimRT {

class IMU_API IMUCalibrator {
 public:
    IMUCalibrator(const OpenSim::Model& otherModel,
                  InputDriver<NGIMUData>* const driver,
                  const std::vector<std::string>& observationOrder);

    SimTK::Rotation setGroundOrientationSeq(const double& xDegrees, const double& yDegrees,
                                 const double& zDegrees);
    void recordTime(const double& timeout);

    void recordNumOfSamples(const size_t& numSamples);

    SimTK::Rotation computeHeadingRotation(const std::string& baseImuName,
                                const std::string& imuDirectionAxis);
    void calibrateIMUTasks(std::vector<InverseKinematics::IMUTask>& imuTasks);

    InverseKinematics::Input
    transform(const std::pair<double, std::vector<NGIMUData>>& imuData,
              const SimTK::Array_<SimTK::Vec3>& markerData);

 private:
    void computeAvgStaticPose();

    OpenSim::Model model;
    SimTK::State state;
    SimTK::ReferencePtr<InputDriver<NGIMUData>> m_driver;
    std::vector<NGIMUData> initIMUData;
    std::vector<std::vector<NGIMUData>> quatTable;
    std::map<std::string, SimTK::Rotation> imuBodiesInGround;
    std::vector<std::string> imuBodiesObservationOrder;
    SimTK::Rotation R_GoGi;
    SimTK::Rotation R_heading;
};
} // namespace OpenSimRT
