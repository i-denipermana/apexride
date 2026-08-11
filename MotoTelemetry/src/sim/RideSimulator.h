#pragma once
//
// Physically consistent motorcycle ride simulator.
//
// Both MockImuSensor and MockGnssSensor sample this one object, so the
// simulated accelerometer, gyroscope, position and speed all describe the same
// motorcycle. That is what makes an offline test of the fusion filter and the
// ride pipeline meaningful.
//
// The model is a coordinated-turn bicycle model:
//
//   - the rider leans to balance gravity against centripetal acceleration
//   - yaw rate follows from lean and speed:  psi_dot = -g * tan(lean) / v
//   - the accelerometer therefore reads (a_long, 0, g/cos(lean)) in a steady
//     corner, reproducing the exact measurement geometry that defeats
//     accelerometer-only lean estimation
//   - suspension dive is modelled as a pitch proportional to longitudinal
//     acceleration
//
// It also keeps the ground truth around, so tests can assert that the fusion
// filter recovers the real lean angle.
//

#include <stddef.h>
#include <stdint.h>

#include "../core/MathUtils.h"
#include "../core/Types.h"

namespace moto {

/// One scripted phase of the ride.
struct RideSegment {
    uint32_t durationMs;
    float    targetSpeedMps;
    float    targetLeanDeg;  ///< positive = lean right
    const char* label;
};

class RideSimulator {
public:
    struct Config {
        double startLatitude  = -6.2088;   ///< Jakarta, matching the handoff example
        double startLongitude = 106.8456;
        float  startAltitudeM = 12.0f;
        uint32_t startUnixTime = 1770000000u;  ///< 2026-02-02T02:40:00Z

        float maxAccelMps2 = 3.0f;
        float maxBrakeMps2 = 6.0f;
        float maxRollRateDps = 55.0f;

        /// Nose-down pitch per unit of braking, radians per m/s^2.
        float suspensionPitchGain = 0.006f;
        float maxPitchRateDps     = 40.0f;

        /// Below this speed the bike cannot be leaned, so lean is forced to zero.
        float minLeanSpeedMps = 4.0f;

        float accelNoiseMps2 = 0.35f;  ///< engine and road vibration
        float gyroNoiseDps   = 0.9f;
        float gyroBiasDps    = 0.8f;   ///< constant offset the filter must tolerate

        float gnssPositionNoiseM = 1.5f;
        float gnssSpeedNoiseMps  = 0.15f;

        uint32_t randomSeed = 0x5EED1234u;
    };

    void begin(const Config& config, const RideSegment* segments, size_t segmentCount);

    /// Advances the simulation to `targetMs` since begin(). Safe to call at any
    /// rate; internally it steps at a fixed small interval.
    void advanceTo(uint32_t targetMs);

    bool finished() const { return finished_; }

    uint32_t elapsedMs() const { return elapsedMs_; }

    uint32_t totalDurationMs() const { return totalDurationMs_; }

    const char* currentSegmentLabel() const;

    // --- Ground truth ------------------------------------------------------
    float trueLeanRad() const { return leanRad_; }
    float truePitchRad() const { return pitchRad_; }
    float trueSpeedMps() const { return speedMps_; }
    float trueHeadingDeg() const;
    double trueLatitude() const { return latitude_; }
    double trueLongitude() const { return longitude_; }
    float trueDistanceM() const { return distanceM_; }
    uint32_t unixTime() const;

    // --- Simulated sensor output -------------------------------------------

    /// Specific force in the body frame, including vibration noise.
    Vec3 accelerometer() const { return accel_; }

    /// Angular rate in the body frame, including noise and a constant bias.
    Vec3 gyroscope() const { return gyro_; }

    float altitudeM() const { return altitudeM_; }

    /// Deterministic pseudo-random noise, so test runs are reproducible.
    float noise(float amplitude);

private:
    void step(float dt);

    Config             config_{};
    const RideSegment* segments_     = nullptr;
    size_t             segmentCount_ = 0;
    size_t             segmentIndex_ = 0;
    uint32_t           segmentElapsedMs_ = 0;
    uint32_t           totalDurationMs_  = 0;

    uint32_t elapsedMs_ = 0;
    uint32_t pendingMicros_ = 0;
    bool     finished_ = false;

    float speedMps_   = 0.0f;
    float leanRad_    = 0.0f;
    float pitchRad_   = 0.0f;
    float headingRad_ = 0.0f;  ///< compass, radians, 0 = north, clockwise positive
    float distanceM_  = 0.0f;
    float altitudeM_  = 0.0f;

    float longitudinalAccelMps2_ = 0.0f;
    float leanRateRadS_          = 0.0f;
    float pitchRateRadS_         = 0.0f;
    float yawRateRadS_           = 0.0f;  ///< math convention: positive = left turn

    double latitude_  = 0.0;
    double longitude_ = 0.0;

    Vec3 accel_;
    Vec3 gyro_;
    Vec3 gyroBias_;

    uint32_t rngState_ = 1u;
};

/// A representative ~2 minute ride: warm-up, acceleration, corners in both
/// directions, hard braking and a stop.
const RideSegment* defaultRideScript(size_t& segmentCountOut);

}  // namespace moto
