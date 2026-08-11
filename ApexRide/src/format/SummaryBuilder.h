#pragma once
//
// Accumulates a RideSummary from a stream of telemetry records.
//
// Used in two places, which is exactly why it is factored out:
//   - TelemetryRecorder feeds it live while a ride is being recorded, so the
//     .met file is always close to current and the phone never has to scan.
//   - RideStorage feeds it records replayed from a .bin when a .met file is
//     missing or corrupt, rebuilding the summary after an unclean shutdown.
//
// Both paths therefore produce identical numbers by construction.
//

#include "TelemetryFormat.h"

namespace apex {

class SummaryBuilder {
public:
    void reset(uint32_t rideId, uint32_t startUnixTime, uint32_t startMillis,
               uint16_t calibrationVersion);

    void addImu(const ImuRecord& record);
    void addGnss(const GnssRecord& record);
    void addEvent(const EventRecord& record);

    /// Ride start time is unknown until the first GNSS fix arrives; backfill it.
    void setStartUnixTime(uint32_t unixTime);

    void setFileSize(uint32_t bytes) { summary_.fileSizeBytes = bytes; }
    void setDataCrc(uint32_t crc) { summary_.dataCrc = crc; }
    void setFlag(uint16_t flag) { summary_.flags |= flag; }
    void clearFlag(uint16_t flag) { summary_.flags &= static_cast<uint16_t>(~flag); }

    /// Returns the summary with its CRC filled in.
    RideSummary build() const;

    const RideSummary& raw() const { return summary_; }

private:
    void noteTimestamp(uint32_t timestampMs);

    RideSummary summary_{};

    uint32_t startMillis_ = 0;

    bool    hasPreviousFix_ = false;
    double  previousLat_    = 0.0;
    double  previousLon_    = 0.0;

    // Longitudinal acceleration is low-pass filtered before peak detection so
    // that engine and road vibration do not register as a hard brake.
    bool  hasAccelFilter_  = false;
    float filteredAccelG_  = 0.0f;
};

/// Fills in RideSummary::summaryCrc for `summary` (also used when re-writing
/// an existing summary, e.g. after marking it synced).
void finalizeSummaryCrc(RideSummary& summary);

/// True if magic, version and CRC all check out.
bool validateSummary(const RideSummary& summary);

}  // namespace apex
