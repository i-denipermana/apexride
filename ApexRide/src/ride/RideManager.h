#pragma once
//
// Decides when a ride starts and stops.
//
// The device is battery powered and not wired to the ignition, so there is no
// authoritative "engine on" signal. Movement is all we have:
//
//     Sleep  --motion detected-->        Awake
//     Awake  --speed above threshold-->  Recording
//     Recording --stopped for a while--> Waiting        (ride stays open)
//     Waiting --moving again-->          Recording      (same ride continues)
//     Waiting --inactivity timeout-->    close ride, Awake
//     Awake  --no motion for a while-->  Sleep
//
// Waiting exists so that a traffic light does not end the ride and a fuel stop
// does. Its thresholds are first guesses and will need road testing; per the
// handoff this stays deliberately simple until basic recording is proven.
//
// Manual start/stop always overrides the automatic logic.
//

#include <stdint.h>

#include "../core/Types.h"

namespace apex {

enum class RideState : uint8_t {
    Sleep,
    Awake,
    Recording,
    Waiting,
};

const char* toString(RideState state);

class RideManager {
public:
    struct Config {
        bool autoStart = true;

        /// Speed that starts a ride, and the lower speed that counts as stopped.
        /// The gap between them stops a ride flapping at walking pace.
        float startSpeedMps = 2.5f;
        float stopSpeedMps  = 1.0f;

        /// Speed must stay above startSpeedMps this long before recording.
        uint32_t startHoldMs = 1500;

        /// Stopped this long while recording -> Waiting.
        uint32_t waitingEnterMs = 8000;

        /// The same, but for when GNSS is unavailable and only the IMU can be
        /// consulted. Much longer, because an accelerometer physically cannot
        /// distinguish a steady cruise from standing still — under a bridge or
        /// in a tunnel the IMU alone will claim the bike has stopped. Ending a
        /// ride on that evidence would split it in two, so the device waits
        /// long enough that only a genuine parked bike trips it.
        uint32_t waitingEnterNoGnssMs = 45000;

        /// Stopped this long in Waiting -> the ride is closed.
        uint32_t waitingTimeoutMs = 120000;

        /// No motion at all this long while Awake -> Sleep.
        uint32_t sleepTimeoutMs = 300000;

        /// Fallback motion detection for when GNSS has no fix, e.g. in a garage.
        float motionGyroDps       = 12.0f;
        float motionAccelMps2     = 1.2f;
    };

    /// What the caller should do as a result of this update.
    enum class Action : uint8_t {
        None,
        StartRide,
        PauseRide,   ///< entered Waiting; keep the file open
        ResumeRide,
        EndRide,
    };

    void begin(const Config& config);

    /// `speedValid` is false when GNSS cannot be trusted, in which case only
    /// the IMU motion test is used.
    Action update(uint32_t nowMs, bool speedValid, float speedMps, const ImuReading& imu);

    RideState state() const { return state_; }

    /// Forces a ride to start or stop, bypassing the thresholds.
    Action requestManualStart(uint32_t nowMs);
    Action requestManualStop(uint32_t nowMs);

    bool isMoving() const { return moving_; }

private:
    void transition(RideState next, uint32_t nowMs);

    Config    config_{};
    RideState state_ = RideState::Sleep;

    uint32_t stateSinceMs_    = 0;
    uint32_t movingSinceMs_   = 0;
    uint32_t stoppedSinceMs_  = 0;
    bool     moving_          = false;
    bool     movingValid_     = false;
    bool     stoppedValid_    = false;
    bool     manualOverride_  = false;
};

}  // namespace apex
