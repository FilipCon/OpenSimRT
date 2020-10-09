#pragma once
#include "internal/IMUExports.h"
#include <Simulation/Model/Model.h>
#include "MahonyAHRS.h"
#include "SignalProcessing.h"
#include <vector>
#include "NGIMUData.h"

namespace OpenSimRT {
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
    SimTK::ReferencePtr<NumericalIntegrator> accelerationIntegrator;
    SimTK::ReferencePtr<NumericalIntegrator> velocityIntegrator;

    SimTK::ReferencePtr<ButterworthFilter> accelerationHPFilter;
    SimTK::ReferencePtr<ButterworthFilter> accelerationLPFilter;
    SimTK::ReferencePtr<ButterworthFilter> velocityHPFilter;
    SimTK::ReferencePtr<ButterworthFilter> positionHPFilter;
};
} // namespace OpenSimRT
