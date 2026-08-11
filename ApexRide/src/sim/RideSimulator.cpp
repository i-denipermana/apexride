#include "RideSimulator.h"

#include <math.h>

namespace apex {
namespace {

/// Fixed physics step. Small enough that the caller's polling rate never
/// changes the trajectory.
constexpr uint32_t kStepMicros = 2000;  // 500 Hz
constexpr float    kStepSeconds = kStepMicros * 1e-6f;

constexpr float kMetresPerDegreeLat = 110540.0f;
constexpr float kMetresPerDegreeLon = 111320.0f;

constexpr float kKmhToMps = 1.0f / 3.6f;

// targetLeanDeg follows the body-frame convention: positive = leaning RIGHT.
const RideSegment kDefaultScript[] = {
    {  8000,  0.0f * kKmhToMps,   0.0f, "idle"          },
    { 14000, 60.0f * kKmhToMps,   0.0f, "accelerate"    },
    {  9000, 60.0f * kKmhToMps,  30.0f, "right corner"  },
    {  6000, 65.0f * kKmhToMps,   0.0f, "straight"      },
    {  9000, 60.0f * kKmhToMps, -35.0f, "left corner"   },
    {  8000, 20.0f * kKmhToMps,   0.0f, "hard braking"  },
    { 13000, 90.0f * kKmhToMps,   0.0f, "accelerate"    },
    {  7000, 85.0f * kKmhToMps,  40.0f, "fast right"    },
    { 12000, 85.0f * kKmhToMps,   0.0f, "straight"      },
    { 14000,  0.0f * kKmhToMps,   0.0f, "decelerate"    },
    {  8000,  0.0f * kKmhToMps,   0.0f, "stopped"       },
};

}  // namespace

void RideSimulator::begin(const Config& config, const RideSegment* segments,
                          size_t segmentCount) {
    config_       = config;
    segments_     = segments;
    segmentCount_ = segmentCount;

    segmentIndex_     = 0;
    segmentElapsedMs_ = 0;
    elapsedMs_        = 0;
    pendingMicros_    = 0;
    finished_         = false;

    totalDurationMs_ = 0;
    for (size_t i = 0; i < segmentCount; ++i) {
        totalDurationMs_ += segments[i].durationMs;
    }

    speedMps_   = 0.0f;
    leanRad_    = 0.0f;
    pitchRad_   = 0.0f;
    headingRad_ = 0.0f;
    distanceM_  = 0.0f;
    altitudeM_  = config.startAltitudeM;

    longitudinalAccelMps2_ = 0.0f;
    leanRateRadS_          = 0.0f;
    pitchRateRadS_         = 0.0f;
    yawRateRadS_           = 0.0f;

    latitude_  = config.startLatitude;
    longitude_ = config.startLongitude;

    rngState_ = config.randomSeed != 0 ? config.randomSeed : 1u;

    // A fixed gyroscope bias the fusion filter's integral term has to learn.
    gyroBias_ = Vec3(config_.gyroBiasDps * kDegToRad * 0.6f,
                     config_.gyroBiasDps * kDegToRad * -0.4f,
                     config_.gyroBiasDps * kDegToRad * 0.9f);

    step(0.0f);  // populate the initial sensor outputs
}

float RideSimulator::noise(float amplitude) {
    // xorshift32 — deterministic and cheap.
    rngState_ ^= rngState_ << 13;
    rngState_ ^= rngState_ >> 17;
    rngState_ ^= rngState_ << 5;

    // Two samples averaged gives a roughly triangular distribution, which
    // looks more like real sensor noise than a flat one.
    const float a = static_cast<float>(rngState_ & 0xFFFFu) / 65535.0f - 0.5f;
    const float b = static_cast<float>((rngState_ >> 16) & 0xFFFFu) / 65535.0f - 0.5f;
    return (a + b) * amplitude;
}

const char* RideSimulator::currentSegmentLabel() const {
    if (segments_ == nullptr || segmentIndex_ >= segmentCount_) {
        return "done";
    }
    return segments_[segmentIndex_].label;
}

float RideSimulator::trueHeadingDeg() const {
    float deg = headingRad_ * kRadToDeg;
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

uint32_t RideSimulator::unixTime() const {
    return config_.startUnixTime + elapsedMs_ / 1000u;
}

void RideSimulator::advanceTo(uint32_t targetMs) {
    if (targetMs <= elapsedMs_) {
        return;
    }

    uint64_t remainingMicros =
        static_cast<uint64_t>(targetMs - elapsedMs_) * 1000ull + pendingMicros_;

    while (remainingMicros >= kStepMicros) {
        step(kStepSeconds);
        remainingMicros -= kStepMicros;

        if (segments_ != nullptr && segmentIndex_ < segmentCount_) {
            segmentElapsedMs_ += kStepMicros / 1000;
            if (segmentElapsedMs_ >= segments_[segmentIndex_].durationMs) {
                segmentElapsedMs_ = 0;
                ++segmentIndex_;
                if (segmentIndex_ >= segmentCount_) {
                    finished_ = true;
                }
            }
        }
    }

    pendingMicros_ = static_cast<uint32_t>(remainingMicros);
    elapsedMs_     = targetMs;
}

void RideSimulator::step(float dt) {
    float targetSpeed = 0.0f;
    float targetLean  = 0.0f;

    if (segments_ != nullptr && segmentIndex_ < segmentCount_) {
        targetSpeed = segments_[segmentIndex_].targetSpeedMps;
        targetLean  = segments_[segmentIndex_].targetLeanDeg * kDegToRad;
    }

    if (dt > 0.0f) {
        // --- Speed -------------------------------------------------------
        const float speedLimit =
            (targetSpeed > speedMps_ ? config_.maxAccelMps2 : config_.maxBrakeMps2) * dt;
        const float newSpeed   = moveToward(speedMps_, targetSpeed, speedLimit);
        longitudinalAccelMps2_ = (newSpeed - speedMps_) / dt;
        speedMps_              = newSpeed;
        if (speedMps_ < 0.0f) speedMps_ = 0.0f;

        // --- Lean --------------------------------------------------------
        // A motorcycle cannot hold lean at walking pace.
        if (speedMps_ < config_.minLeanSpeedMps) {
            targetLean = 0.0f;
        }
        const float leanLimit = config_.maxRollRateDps * kDegToRad * dt;
        const float newLean   = moveToward(leanRad_, targetLean, leanLimit);
        leanRateRadS_         = (newLean - leanRad_) / dt;
        leanRad_              = newLean;

        // --- Pitch (suspension dive) --------------------------------------
        const float targetPitch = longitudinalAccelMps2_ * config_.suspensionPitchGain;
        const float pitchLimit  = config_.maxPitchRateDps * kDegToRad * dt;
        const float newPitch    = moveToward(pitchRad_, targetPitch, pitchLimit);
        pitchRateRadS_          = (newPitch - pitchRad_) / dt;
        pitchRad_               = newPitch;

        // --- Yaw from the coordinated-turn condition -----------------------
        // Leaning right (positive) turns right, which is a negative yaw rate
        // in the math convention (positive = counter-clockwise / left).
        if (speedMps_ > 0.5f) {
            yawRateRadS_ = -kGravityMps2 * tanf(leanRad_) / speedMps_;
        } else {
            yawRateRadS_ = 0.0f;
        }

        // Compass heading increases clockwise, so it moves opposite to yaw.
        headingRad_ -= yawRateRadS_ * dt;
        headingRad_ = wrapPi(headingRad_);

        // --- Position ------------------------------------------------------
        const float stepM = speedMps_ * dt;
        distanceM_ += stepM;

        const float dNorth = stepM * cosf(headingRad_);
        const float dEast  = stepM * sinf(headingRad_);

        latitude_ += dNorth / kMetresPerDegreeLat;
        const float cosLat = cosf(static_cast<float>(latitude_) * kDegToRad);
        if (fabsf(cosLat) > 1e-4f) {
            longitude_ += dEast / (kMetresPerDegreeLon * cosLat);
        }
    }

    // --- Synthesise the sensor readings ------------------------------------
    //
    // Specific force in the heading-aligned frame (x forward, y left, z up):
    // gravity plus the centripetal reaction. Leaning right pulls the bike
    // toward -Y, hence the negative lateral term.
    const Vec3 forceHeading{longitudinalAccelMps2_,
                            -kGravityMps2 * tanf(leanRad_),
                            kGravityMps2};

    // Rotate world -> body: first pitch, then roll.
    Vec3 force = rotateXNeg(rotateYNeg(forceHeading, -pitchRad_), leanRad_);

    force.x += noise(config_.accelNoiseMps2);
    force.y += noise(config_.accelNoiseMps2);
    force.z += noise(config_.accelNoiseMps2 * 1.4f);  // vertical hits hardest
    accel_ = force;

    // Body angular rates for the Z-Y-X Euler set. eulerY is positive nose-down,
    // so the simulator's nose-up pitch enters with a negated sign.
    const float eulerY     = -pitchRad_;
    const float eulerYRate = -pitchRateRadS_;

    const float sinRoll  = sinf(leanRad_);
    const float cosRoll  = cosf(leanRad_);
    const float sinPitch = sinf(eulerY);
    const float cosPitch = cosf(eulerY);

    gyro_ = Vec3(leanRateRadS_ - sinPitch * yawRateRadS_,
                 cosRoll * eulerYRate + sinRoll * cosPitch * yawRateRadS_,
                 -sinRoll * eulerYRate + cosRoll * cosPitch * yawRateRadS_);

    gyro_ = gyro_ + gyroBias_;
    gyro_.x += noise(config_.gyroNoiseDps * kDegToRad);
    gyro_.y += noise(config_.gyroNoiseDps * kDegToRad);
    gyro_.z += noise(config_.gyroNoiseDps * kDegToRad);
}

const RideSegment* defaultRideScript(size_t& segmentCountOut) {
    segmentCountOut = sizeof(kDefaultScript) / sizeof(kDefaultScript[0]);
    return kDefaultScript;
}

}  // namespace apex
