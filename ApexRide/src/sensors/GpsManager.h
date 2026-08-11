#pragma once
//
// Owns the GNSS receiver and tracks fix state.
//
// Its other job is supplying the speed hint that Orientation needs for the
// kinematic lean correction, including deciding when that hint has gone stale.
// A stale hint is worse than none: correcting with a speed from ten seconds ago
// would inject an error rather than remove one.
//

#include "../core/Clock.h"
#include "../core/Types.h"
#include "IGnssSensor.h"

namespace apex {

class GpsManager {
public:
    struct Config {
        /// A fix older than this stops being used for the kinematic correction.
        uint32_t maxHintAgeMs = 1500;

        /// HDOP above which a fix is treated as too poor to trust for speed.
        float maxUsableHdop = 5.0f;

        /// Low-pass coefficient applied to the differentiated speed. Low,
        /// because differentiating a noisy 5 Hz signal is noisy.
        float accelFilterAlpha = 0.35f;

        /// Differentiated values beyond this are noise, not a motorcycle.
        float maxPlausibleAccelMps2 = 12.0f;

        /// How long after a lost fix the bike is still assumed to be moving.
        /// Covers a tunnel or an urban canyon without pretending a bike parked
        /// in a garage an hour ago is still under way.
        uint32_t movementMemoryMs = 30000;

        /// Last known speed above which the bike counts as moving.
        float movingSpeedMps = 2.0f;
    };

    GpsManager(IGnssSensor& sensor, const Clock& clock);

    bool begin(const Config& config);

    /// Services the receiver. Returns true when `out` holds a new solution.
    bool poll(GnssReading& out);

    bool               hasFix() const { return lastReading_.hasFix(); }
    const GnssReading& lastReading() const { return lastReading_; }

    /// True once any fix has ever been acquired this power cycle.
    bool everHadFix() const { return everHadFix_; }

    /// Wall-clock time from the most recent fix, or 0 if never fixed.
    uint32_t lastUnixTime() const { return lastUnixTime_; }

    /// Speed to feed the fusion filter, or a hint marked invalid when the fix
    /// is missing, stale or low quality.
    bool speedHint(float& speedMpsOut) const;

    /// Longitudinal acceleration, differentiated from successive GNSS speeds
    /// and low-pass filtered.
    ///
    /// The fusion filter needs this to complete the kinematic correction: the
    /// accelerometer's X axis reads gravity plus dv/dt, so without it a 6 m/s^2
    /// stop looks to the filter like a 38 degree nose-down attitude. A
    /// differentiated 5 Hz speed is noisy, but it is an order of magnitude
    /// better than leaving the term out.
    bool accelerationHint(float& accelMps2Out) const;

    /// True when the bike is probably still moving even though there is no
    /// usable fix right now — the last good fix was recent and showed it under
    /// way. The fusion filter uses this to distrust the accelerometer during a
    /// GNSS dropout instead of treating manoeuvre forces as gravity.
    bool likelyMoving() const;

    uint32_t fixCount() const { return fixCount_; }

private:
    void updateAcceleration(const GnssReading& reading);

    IGnssSensor& sensor_;
    const Clock& clock_;
    Config       config_{};

    GnssReading lastReading_{};
    uint64_t    lastFixUs_    = 0;
    uint32_t    lastUnixTime_ = 0;
    uint32_t    fixCount_     = 0;
    bool        everHadFix_   = false;

    // Speed differentiation state.
    float    filteredAccelMps2_ = 0.0f;
    float    previousSpeedMps_  = 0.0f;
    uint64_t previousSpeedUs_   = 0;
    bool     hasPreviousSpeed_  = false;
    bool     accelValid_        = false;

    float    lastGoodSpeedMps_  = 0.0f;
};

}  // namespace apex
