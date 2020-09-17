//============================================================================================
// MahonyAHRS.c
//============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//============================================================================================

// Header files

#include "MahonyAHRS.h"

#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/Quaternion.h>
#include <SimTKcommon/internal/Rotation.h>
#include <SimTKcommon/internal/SmallMatrixMixed.h>
#include <math.h>

SimTK::Vec4 quaternProd(const SimTK::Vec4& a, const SimTK::Vec4& b) {
    SimTK::Vec4 c;
    c[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    c[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    c[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    c[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    return c;
}

SimTK::Vec4 quatConjugate(const SimTK::Vec4& q) {
    return SimTK::Vec4(q[0], -q[1], -q[2], -q[3]);
}

//--------------------------------------------------------------------------------------------
MahonyAHRS::MahonyAHRS(const double& sampleRate, const double& Kp,
                       const double& Ki)
        : sampleFreq(sampleRate), Kp(Kp), Ki(Ki), q(SimTK::Quaternion()),
          integralFB(SimTK::Vec3(0)) {}

void MahonyAHRS::update(const SimTK::Vec3& gyr, const SimTK::Vec3& acc,
                        const SimTK::Vec3& mag) {
    if (mag.norm() == 0 || acc.norm() == 0) return;
    // create local copies
    auto g = gyr;
    auto m = mag.normalize();

    // Reference direction of Earth's magnetic field
    auto h = quaternProd(q, quaternProd(SimTK::Vec4(0, m[0], m[1], m[2]),
                                        quatConjugate(q.asVec4())));
    const auto b = SimTK::Vec4(0, SimTK::Vec2(h[1], h[2]).norm(), 0, h[3]);

    // Estimated direction of gravity and magnetic field
    const auto v =
            SimTK::Vec3(2.0 * (q[1] * q[3] - q[0] * q[2]),
                        2.0 * (q[0] * q[1] - q[2] * q[3]),
                        q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

    const auto w =
            SimTK::Vec3(2.0 * b[1] * (0.5 - q[2] * q[2] - q[3] * q[3]) +
                                2.0 * b(3) * (q[1] * q[3] - q[0] * q[2]),
                        2.0 * b[1] * (q[1] * q[2] - q[0] * q[3]) +
                                2.0 * b(3) * (q[0] * q[1] + q[2] * q[3]),
                        2.0 * b[1] * (q[0] * q[2] + q[1] * q[3]) +
                                2.0 * b(3) * (0.5 - q[1] * q[1] * q[2] * q[2]));

    // error is sum of cross product between estimated direction and measured
    // direction of fields
    const auto e = SimTK::cross(acc.normalize(), v) + SimTK::cross(m, w);
    if (Ki > 0.0) { // integral error scaled by Ki
        integralFB += e * (1.0 / sampleFreq);

    } else { // prevent integral windup
        integralFB = SimTK::Vec3(0);
    }

    // Apply proportional feedback
    g += Kp * e + Ki * integralFB;

    // hamilton product (q, [0, g])
    auto qDot = 0.5 * SimTK::Quaternion(quaternProd(
                              q.asVec4(), SimTK::Vec4(0, g[0], g[1], g[2])));

    q += qDot * (1 / sampleFreq);

    // Normalise quaternion
    q.normalize();
}

// IMU algorithm update
void MahonyAHRS::updateIMU(const SimTK::Vec3& gyr, const SimTK::Vec3& acc) {
    // Compute feedback only if accelerometer measurement valid (avoids NaN in
    // accelerometer normalisation)
    if (acc.norm() == 0) return;

    auto g = gyr; // create local copy

    // Estimated direction of gravity and  magnetic flux
    const auto v =
            SimTK::Vec3(2.0 * (q[1] * q[3] - q[0] * q[2]),
                        2.0 * (q[0] * q[1] + q[2] * q[3]),
                        q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

    // Error is sum of cross product between estimated and measured
    // direction of gravity
    const auto e = SimTK::cross(acc.normalize(), v);

    // Compute and apply integral feedback if enabled
    if (Ki > 0.0) { // integral error scaled by Ki
        integralFB += e * (1.0 / sampleFreq);

    } else { // prevent integral windup
        integralFB = SimTK::Vec3(0);
    }

    // Apply proportional feedback
    g += Kp * e + Ki * integralFB;

    // hamilton product (q, [0, g])
    auto qDot = 0.5 * SimTK::Quaternion(quaternProd(
                              q.asVec4(), SimTK::Vec4(0, g[0], g[1], g[2])));

    q += qDot * (1 / sampleFreq);

    // Normalise quaternion
    q.normalize();
}
