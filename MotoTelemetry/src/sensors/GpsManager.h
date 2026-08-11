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

namespace moto {

class GpsManager {
public:
    struct Config {
        /// A fix older than this stops being used for the kinematic correction.
        uint32_t maxHintAgeMs = 1500;

        /// HDOP above which a fix is treated as too poor to trust for speed.
        float maxUsableHdop = 5.0f;
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

    uint32_t fixCount() const { return fixCount_; }

private:
    IGnssSensor& sensor_;
    const Clock& clock_;
    Config       config_{};

    GnssReading lastReading_{};
    uint64_t    lastFixUs_    = 0;
    uint32_t    lastUnixTime_ = 0;
    uint32_t    fixCount_     = 0;
    bool        everHadFix_   = false;
};

}  // namespace moto
