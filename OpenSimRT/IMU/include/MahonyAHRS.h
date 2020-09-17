//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

// Adapted for C++ by Filip.K

#ifndef MahonyAHRS_h
#    define MahonyAHRS_h

#    include "internal/IMUExports.h"

#    include <SimTKcommon/SmallMatrix.h>
#    include <SimTKcommon/internal/Quaternion.h>
#    include <Simbody.h>
//----------------------------------------------------------------------------------------------------
// Variable declaration

class IMU_API MahonyAHRS {
 public:

    MahonyAHRS() = default;
    MahonyAHRS(const double& sampleRate, const double& Kp, const double& Ki);

    void update(const SimTK::Vec3& gyr, const SimTK::Vec3& acc, const SimTK::Vec3& mag);
    void updateIMU(const SimTK::Vec3& gyr, const SimTK::Vec3& acc);
    /* // setters */
    /* void setSampleFreq(const double& sampleRate) { sampleFreq = sampleRate; }
     */
    /* void setProportionalGain(const double& value) { twoKp = 2 * value; } */
    /* void setIntegralGain(const double& value) { twoKi = 2 * value; } */

    // getters
    SimTK::Quaternion getQuaternion() { return q; }

 private:
    double sampleFreq;
    double Kp;           // proportional gain (Kp)
    double Ki;           // integral gain (Ki)
    SimTK::Vec3 integralFB; // integral error terms scaled by Ki
    SimTK::Quaternion q;
};
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
