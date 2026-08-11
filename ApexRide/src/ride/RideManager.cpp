#include "RideManager.h"

#include <math.h>

#include "../core/Log.h"

namespace apex {

const char* toString(RideState state) {
    switch (state) {
        case RideState::Sleep:     return "SLEEP";
        case RideState::Awake:     return "AWAKE";
        case RideState::Recording: return "RECORDING";
        case RideState::Waiting:   return "WAITING";
    }
    return "?";
}

void RideManager::begin(const Config& config) {
    config_         = config;
    state_          = RideState::Sleep;
    stateSinceMs_   = 0;
    movingValid_    = false;
    stoppedValid_   = false;
    moving_         = false;
    manualOverride_ = false;
}

void RideManager::transition(RideState next, uint32_t nowMs) {
    if (next == state_) {
        return;
    }
    APEX_LOGI("Ride state %s -> %s", toString(state_), toString(next));
    state_        = next;
    stateSinceMs_ = nowMs;
}

RideManager::Action RideManager::requestManualStart(uint32_t nowMs) {
    if (state_ == RideState::Recording) {
        return Action::None;
    }

    const bool resuming = state_ == RideState::Waiting;
    manualOverride_     = true;
    transition(RideState::Recording, nowMs);
    return resuming ? Action::ResumeRide : Action::StartRide;
}

RideManager::Action RideManager::requestManualStop(uint32_t nowMs) {
    if (state_ != RideState::Recording && state_ != RideState::Waiting) {
        return Action::None;
    }

    manualOverride_ = false;
    transition(RideState::Awake, nowMs);
    return Action::EndRide;
}

RideManager::Action RideManager::update(uint32_t nowMs, bool speedValid, float speedMps,
                                        const ImuReading& imu) {
    // --- Motion test -------------------------------------------------------
    //
    // GNSS speed is authoritative when available. The IMU fallback keeps the
    // device responsive indoors and during the cold-start window before the
    // first fix.
    const float gyroDps       = imu.gyro.norm() * kRadToDeg;
    const float accelDeviate  = fabsf(imu.accel.norm() - kGravityMps2);
    const bool  imuSaysMoving = gyroDps > config_.motionGyroDps ||
                               accelDeviate > config_.motionAccelMps2;

    const bool aboveStart = speedValid ? speedMps >= config_.startSpeedMps : imuSaysMoving;
    const bool belowStop  = speedValid ? speedMps < config_.stopSpeedMps : !imuSaysMoving;

    moving_ = aboveStart;

    if (aboveStart) {
        if (!movingValid_) {
            movingSinceMs_ = nowMs;
            movingValid_   = true;
        }
    } else {
        movingValid_ = false;
    }

    if (belowStop) {
        if (!stoppedValid_) {
            stoppedSinceMs_ = nowMs;
            stoppedValid_   = true;
        }
    } else {
        stoppedValid_ = false;
    }

    const uint32_t movingForMs  = movingValid_ ? nowMs - movingSinceMs_ : 0;
    const uint32_t stoppedForMs = stoppedValid_ ? nowMs - stoppedSinceMs_ : 0;

    switch (state_) {
        case RideState::Sleep:
            if (imuSaysMoving || aboveStart) {
                transition(RideState::Awake, nowMs);
            }
            return Action::None;

        case RideState::Awake:
            if (config_.autoStart && aboveStart && movingForMs >= config_.startHoldMs) {
                transition(RideState::Recording, nowMs);
                return Action::StartRide;
            }
            if (!imuSaysMoving && !aboveStart && nowMs - stateSinceMs_ >= config_.sleepTimeoutMs) {
                transition(RideState::Sleep, nowMs);
            }
            return Action::None;

        case RideState::Recording: {
            // A manual start keeps recording through stops until manually
            // stopped, so a deliberate session is never cut short.
            const uint32_t requiredStopMs =
                speedValid ? config_.waitingEnterMs : config_.waitingEnterNoGnssMs;
            if (!manualOverride_ && belowStop && stoppedForMs >= requiredStopMs) {
                transition(RideState::Waiting, nowMs);
                return Action::PauseRide;
            }
            return Action::None;
        }

        case RideState::Waiting:
            if (aboveStart) {
                transition(RideState::Recording, nowMs);
                return Action::ResumeRide;
            }
            if (nowMs - stateSinceMs_ >= config_.waitingTimeoutMs) {
                transition(RideState::Awake, nowMs);
                return Action::EndRide;
            }
            return Action::None;
    }

    return Action::None;
}

}  // namespace apex
