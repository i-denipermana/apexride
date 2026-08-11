#pragma once
//
// Attitude estimation for a leaning vehicle.
//
// ---------------------------------------------------------------------------
// Why not atan2(accelY, accelZ)?
//
// In a steady, coordinated corner the motorcycle leans until the resultant of
// gravity and centripetal acceleration points straight down through the tyres.
// The accelerometer then measures
//
//     (a_long, 0, g / cos(lean))
//
// The lateral component is ZERO no matter how hard you are cornering, so an
// accelerometer-only estimate reports upright at 45 degrees of lean. This is
// not noise that can be filtered out — it is the physics of the measurement.
//
// ---------------------------------------------------------------------------
// What this class does instead
//
// 1. A Mahony complementary filter integrates the gyroscope for attitude and
//    uses the accelerometer only as a slow gravity reference to bound drift.
//
// 2. That reference is still wrong mid-corner, which would pull the estimate
//    back toward upright over a long turn. So when GNSS speed is available the
//    filter first removes the kinematic (transport-rate) acceleration:
//
//        gravity_body = accel_measured - omega x velocity_body
//
//    With velocity_body = (v, 0, 0) that is exactly
//
//        accel - (0, r*v, -q*v)
//
//    which recovers the true gravity direction (0, g*sin(roll), g*cos(roll)).
//    This is the standard technique for attitude estimation on a vehicle whose
//    forward speed is known, and it is what makes the lean angle trustworthy
//    through a sustained corner rather than merely at turn-in.
//
// Set Config::useKinematicCorrection = false to fall back to plain Mahony.
//
// ---------------------------------------------------------------------------
// Mounting calibration
//
// The device will not sit perfectly upright on the bike. captureMountingOffset()
// records the current attitude as "zero" and every later reading is expressed
// relative to it. The offset is stored as a quaternion rather than a pair of
// scalar angles, so a device rotated in yaw or mounted on a slanted bracket is
// corrected properly rather than approximately.
//

#include "../core/Types.h"

namespace moto {

class Orientation {
public:
    struct Config {
        /// Proportional gain: how hard the accelerometer pulls the estimate.
        /// Low, because on a motorcycle the accelerometer is the untrustworthy
        /// sensor. Raise it only if gyro drift becomes visible.
        float kp = 0.6f;

        /// Integral gain, which slowly learns residual gyro bias.
        float ki = 0.02f;

        bool useKinematicCorrection = true;

        /// Below this speed the correction is skipped: omega x v is negligible
        /// and GNSS speed/course are noisy when nearly stationary.
        float minSpeedForCorrectionMps = 3.0f;

        /// Accelerometer samples whose magnitude is this far from 1 g are
        /// dominated by bumps and impacts; they are not usable as a gravity
        /// reference, so the filter coasts on the gyro for that sample.
        float maxGravityDeviationMps2 = 3.0f;
    };

    /// Speed hint supplied by GNSS, used for the kinematic correction.
    struct KinematicHint {
        bool  valid    = false;
        float speedMps = 0.0f;
    };

    void begin(const Config& config);

    /// Sets the attitude directly from a stationary accelerometer reading, so
    /// the filter starts converged instead of spending seconds settling.
    void seedFromAccel(const ImuReading& reading);

    /// Advances the filter by `dt` seconds.
    void update(const ImuReading& reading, float dt, const KinematicHint& hint);

    /// Latest attitude in motorcycle convention (roll = lean right positive,
    /// pitch = nose up positive), with the mounting offset removed.
    const FusedState& state() const { return state_; }

    /// Records the current attitude as the upright reference. The motorcycle
    /// must be stationary, upright and on level ground.
    void captureMountingOffset();

    void              setMountingOffset(const Quaternion& offset);
    const Quaternion& mountingOffset() const { return mountingOffset_; }
    void              clearMountingOffset();

    /// Raw attitude before the mounting offset is applied. Exposed so the
    /// calibration routine can report what it measured.
    const Quaternion& rawAttitude() const { return attitude_; }

    void reset();

private:
    void publish(uint64_t timestampUs, const Vec3& accel);

    Config config_{};

    Quaternion attitude_       = Quaternion::identity();
    Quaternion mountingOffset_ = Quaternion::identity();

    Vec3 integralFeedback_;

    FusedState state_{};
    bool       seeded_ = false;
};

}  // namespace moto
