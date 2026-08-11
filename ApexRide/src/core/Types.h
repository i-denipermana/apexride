#pragma once
//
// Shared value types and the project's coordinate conventions.
//
// ---------------------------------------------------------------------------
// BODY FRAME (right-handed, fixed to the motorcycle chassis)
//
//   +X  forward  (direction of travel)
//   +Y  left
//   +Z  up
//
// A level, stationary accelerometer therefore reads (0, 0, +9.81 m/s^2).
// This matches the frame assumed by the reference Mahony/Madgwick filters.
//
// Gyroscope sign follows the right-hand rule about each axis.
//
// ---------------------------------------------------------------------------
// VEHICLE ANGLES (see Orientation.h)
//
//   roll   positive = leaning RIGHT
//   pitch  positive = nose UP
//   yaw    positive = turning LEFT (counter-clockwise seen from above)
//
// ---------------------------------------------------------------------------
// The ICM-20948 breakout will almost certainly not be mounted in this
// orientation. ImuManager::Config::axisMap remaps the raw sensor axes into the
// body frame before anything else touches the data.
//

#include <stdint.h>

#include "MathUtils.h"

namespace apex {

/// One calibrated IMU sample, already expressed in the body frame.
struct ImuReading {
    uint64_t timestampUs = 0;

    Vec3 accel;  ///< specific force, m/s^2
    Vec3 gyro;   ///< angular rate, rad/s
    Vec3 mag;    ///< magnetic field, microtesla

    bool magValid = false;
};

enum class GnssFix : uint8_t {
    None = 0,
    Fix2D = 2,
    Fix3D = 3,
};

/// One GNSS solution.
struct GnssReading {
    uint64_t timestampUs = 0;  ///< monotonic device time the fix was received

    uint32_t unixTime = 0;  ///< seconds since epoch, 0 when unknown

    double latitude  = 0.0;  ///< degrees
    double longitude = 0.0;  ///< degrees

    float speedMps   = 0.0f;
    float courseDeg  = 0.0f;  ///< 0..360, compass (0 = north, increasing clockwise)
    float altitudeM  = 0.0f;
    float hdop       = 99.9f;

    uint8_t satellites = 0;
    GnssFix fix        = GnssFix::None;

    bool hasFix() const { return fix != GnssFix::None; }
};

/// Output of the fusion filter, in vehicle angle convention.
struct FusedState {
    uint64_t timestampUs = 0;

    float rollRad  = 0.0f;  ///< positive = lean right
    float pitchRad = 0.0f;  ///< positive = nose up
    float yawRad   = 0.0f;  ///< positive = turning left, relative to calibration

    /// Gravity-compensated longitudinal acceleration, m/s^2.
    /// Positive = accelerating, negative = braking.
    float longitudinalAccelMps2 = 0.0f;

    bool valid = false;

    float rollDeg() const { return rollRad * kRadToDeg; }
    float pitchDeg() const { return pitchRad * kRadToDeg; }
    float yawDeg() const { return yawRad * kRadToDeg; }
};

}  // namespace apex
