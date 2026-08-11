#include "MockImuSensor.h"

#include <math.h>

namespace apex {
namespace {

/// Expresses a body-frame vector in the (deliberately misaligned) sensor frame.
///
/// Two separate errors are modelled, because they need two different fixes:
///   - a roll tilt of the bracket, which mounting calibration must remove
///   - a 90 degree rotation about the vertical axis, which is a wiring/mounting
///     fact that ImuManager's axis map must remove
Vec3 toSensorFrame(const Vec3& body, float tiltRad) {
    const Vec3 tilted = rotateXNeg(body, tiltRad);
    return Vec3(tilted.y, -tilted.x, tilted.z);
}

}  // namespace

MockImuSensor::MockImuSensor(const Clock& clock, RideSimulator& simulator)
    : clock_(clock), simulator_(simulator) {}

bool MockImuSensor::begin() {
    intervalUs_   = static_cast<uint64_t>(1e6f / config_.sampleRateHz);
    nextSampleUs_ = clock_.micros();
    return true;
}

bool MockImuSensor::read(RawImuSample& out) {
    const uint64_t now = clock_.micros();
    if (now < nextSampleUs_) {
        return false;
    }

    // Advance the shared simulation to the moment this sample was taken.
    simulator_.advanceTo(static_cast<uint32_t>(nextSampleUs_ / 1000u));

    out.timestampUs = nextSampleUs_;

    const float tilt = config_.applyMountingError ? config_.mountingTiltDeg * kDegToRad : 0.0f;

    out.accel    = toSensorFrame(simulator_.accelerometer(), tilt);
    out.gyro     = toSensorFrame(simulator_.gyroscope(), tilt);
    out.mag      = Vec3();
    out.magValid = false;

    // Advance the schedule. If the caller stalled long enough to miss whole
    // samples, drop them rather than bursting — that is what a real driver
    // reading a FIFO-less sensor would do.
    nextSampleUs_ += intervalUs_;
    if (nextSampleUs_ + intervalUs_ < now) {
        nextSampleUs_ = now;
    }

    return true;
}

}  // namespace apex
