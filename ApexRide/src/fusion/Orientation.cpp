#include "Orientation.h"

#include <math.h>

namespace apex {
namespace {

/// Builds a quaternion from roll/pitch with yaw zero, using the intrinsic
/// Z-Y-X convention that Quaternion::eulerX/Y/Z decode.
Quaternion fromRollPitch(float rollRad, float pitchRad) {
    const float cx = cosf(rollRad * 0.5f);
    const float sx = sinf(rollRad * 0.5f);
    const float cy = cosf(pitchRad * 0.5f);
    const float sy = sinf(pitchRad * 0.5f);

    Quaternion q(cy * cx, cy * sx, sy * cx, -sy * sx);
    q.normalize();
    return q;
}

}  // namespace

void Orientation::begin(const Config& config) {
    config_ = config;
    reset();
}

void Orientation::reset() {
    attitude_         = Quaternion::identity();
    integralFeedback_ = Vec3();
    state_            = FusedState();
    seeded_           = false;
}

void Orientation::clearMountingOffset() {
    mountingOffset_ = Quaternion::identity();
}

void Orientation::setMountingOffset(const Quaternion& offset) {
    mountingOffset_ = offset;
    mountingOffset_.normalize();
}

void Orientation::captureMountingOffset() {
    mountingOffset_ = attitude_;
    mountingOffset_.normalize();
}

void Orientation::seedFromAccel(const ImuReading& reading) {
    Vec3 a = reading.accel;
    if (!a.normalize()) {
        return;
    }

    // Body-frame gravity direction for roll phi and Euler-Y theta is
    //   (-sin(theta), sin(phi)cos(theta), cos(phi)cos(theta))
    const float roll    = atan2f(a.y, a.z);
    const float eulerY  = atan2f(-a.x, sqrtf(a.y * a.y + a.z * a.z));

    attitude_         = fromRollPitch(roll, eulerY);
    integralFeedback_ = Vec3();
    seeded_           = true;

    publish(reading.timestampUs, reading.accel);
}

void Orientation::update(const ImuReading& reading, float dt, const KinematicHint& hint) {
    if (dt <= 0.0f || dt > 0.5f) {
        // A gap this large means samples were dropped; integrating across it
        // would inject a large attitude error. Skip instead.
        return;
    }

    if (!seeded_) {
        seedFromAccel(reading);
        return;
    }

    Vec3 rate = reading.gyro;

    Vec3 gravityRef = reading.accel;

    // Remove the vehicle's own acceleration, leaving gravity:
    //   gravity_body = f_measured - (dv/dt + omega x v_body)
    // with v_body = (v, 0, 0), so omega x v = (0, r*v, -q*v).
    //
    // The two terms are gated independently because they matter at different
    // speeds: omega x v only exists once the bike is moving properly, while
    // dv/dt is largest pulling away from a standstill.
    bool transportCorrected = false;
    bool accelCorrected     = false;

    if (config_.useKinematicCorrection && hint.valid) {
        if (hint.speedMps >= config_.minSpeedForCorrectionMps) {
            gravityRef.y -= reading.gyro.z * hint.speedMps;
            gravityRef.z += reading.gyro.y * hint.speedMps;
            transportCorrected = true;
        }

        // Acts along body X; without it, hard braking looks like a steep
        // nose-down attitude.
        if (hint.accelValid && hint.speedMps >= config_.minSpeedForAccelCorrectionMps) {
            gravityRef.x -= hint.accelMps2;
            accelCorrected = true;
        }
    }

    const float magnitude = gravityRef.norm();
    const bool  gravityUsable =
        fabsf(magnitude - kGravityMps2) <= config_.maxGravityDeviationMps2 && gravityRef.normalize();

    if (gravityUsable) {
        // Direction the filter currently believes "up" points, in body frame.
        const Vec3 estimatedUp{
            2.0f * (attitude_.x * attitude_.z - attitude_.w * attitude_.y),
            2.0f * (attitude_.w * attitude_.x + attitude_.y * attitude_.z),
            attitude_.w * attitude_.w - attitude_.x * attitude_.x - attitude_.y * attitude_.y +
                attitude_.z * attitude_.z};

        const Vec3 error = cross(gravityRef, estimatedUp);

        // If the bike is moving but the correction could not be applied, the
        // accelerometer is measuring manoeuvre forces as much as gravity. Back
        // off rather than steer the estimate with it, and stop accumulating
        // that error into the integral term.
        const bool corrected = transportCorrected || accelCorrected;
        const bool distrust  = !corrected && hint.movingWithoutHint;
        const float kp       = distrust ? config_.kpUncorrected : config_.kp;

        if (config_.ki > 0.0f && !distrust) {
            integralFeedback_ = integralFeedback_ + error * (config_.ki * dt);
        }

        rate = rate + error * kp + integralFeedback_;
    }

    // Quaternion derivative: qDot = 0.5 * q (x) (0, wx, wy, wz)
    const Quaternion omega(0.0f, rate.x, rate.y, rate.z);
    const Quaternion qDot = attitude_ * omega;

    attitude_.w += 0.5f * qDot.w * dt;
    attitude_.x += 0.5f * qDot.x * dt;
    attitude_.y += 0.5f * qDot.y * dt;
    attitude_.z += 0.5f * qDot.z * dt;
    attitude_.normalize();

    publish(reading.timestampUs, reading.accel);
}

void Orientation::publish(uint64_t timestampUs, const Vec3& accel) {
    // attitude_ rotates the DEVICE frame into the world. The mounting offset is
    // the device attitude recorded while the bike was level, so
    //   q_bike->world = q_device->world (x) conj(q_mount)
    //
    // The order matters: the reversed product only happens to agree when the
    // offset is a pure rotation about one axis, and the filter's free-running
    // yaw guarantees that it is not.
    const Quaternion corrected = attitude_ * mountingOffset_.conjugate();

    state_.timestampUs = timestampUs;
    state_.rollRad     = corrected.eulerX();

    // eulerY is positive nose-DOWN in an X-forward / Y-left / Z-up frame.
    // The reported pitch is negated so that positive means nose up.
    state_.pitchRad = -corrected.eulerY();
    state_.yawRad   = corrected.eulerZ();
    state_.valid    = true;

    // Strip the gravity component that pitch projects onto the body X axis:
    //   accelX = a_long * cos(pitch) + g * sin(-pitch)
    const float cosPitch = cosf(state_.pitchRad);
    if (fabsf(cosPitch) > 0.1f) {
        state_.longitudinalAccelMps2 =
            (accel.x - kGravityMps2 * sinf(state_.pitchRad)) / cosPitch;
    } else {
        state_.longitudinalAccelMps2 = 0.0f;
    }
}

}  // namespace apex
