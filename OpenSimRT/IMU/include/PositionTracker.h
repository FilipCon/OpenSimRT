#pragma once
#include "MahonyAHRS.h"
#include "NGIMUData.h"
#include "SignalProcessing.h"
#include "internal/IMUExports.h"

#include <SimTKcommon/internal/Rotation.h>
#include <Simulation/Model/Model.h>
#include <vector>

namespace OpenSimRT {
class IMU_API PositionTracker {
 public:
    PositionTracker(const double&, const double&);
    SimTK::Vec3 computeVelocity(const double&, const SimTK::Vec3&);
    SimTK::Vec3 computePosition(const double&, const SimTK::Vec3&);

 private:
    SimTK::ReferencePtr<NumericalIntegrator> accelerationIntegrator;
    SimTK::ReferencePtr<NumericalIntegrator> velocityIntegrator;

    SimTK::ReferencePtr<ButterworthFilter> accelerationHPFilter;
    SimTK::ReferencePtr<ButterworthFilter> accelerationLPFilter;
    SimTK::ReferencePtr<ButterworthFilter> velocityHPFilter;
    SimTK::ReferencePtr<ButterworthFilter> positionHPFilter;

    double _threshold;
    double _samplingFreq;

    SimTK::Rotation R_GoGi;
};
} // namespace OpenSimRT
