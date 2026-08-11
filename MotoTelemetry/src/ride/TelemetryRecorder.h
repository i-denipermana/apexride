#pragma once
//
// Encodes telemetry into the binary format and gets it onto flash safely.
//
// ---------------------------------------------------------------------------
// Write strategy
//
// Samples are encoded into a RAM block buffer and handed to LittleFS only in
// whole blocks. Writing 50 records per second individually would multiply
// flash wear: LittleFS rewrites a whole 4 KB block for any change inside it,
// so ~24 bytes of telemetry would cost 4 KB of erase-write.
//
// Buffering also means an unclean shutdown can lose at most one block, which
// the recovery scan in RideStorage handles as a truncated tail.
//
// The summary is kept current as samples arrive and rewritten to its own file
// periodically, so the phone can list rides without reading any sample data.
//

#include "../core/BlockBuffer.h"
#include "../core/Clock.h"
#include "../core/Crc32.h"
#include "../core/Types.h"
#include "../format/SummaryBuilder.h"
#include "../storage/RideStorage.h"

namespace moto {

class TelemetryRecorder {
public:
    struct Config {
        /// Rate at which fused samples are written. Deliberately lower than the
        /// fusion rate: see the storage budget note in README.md.
        float imuLogRateHz = 50.0f;

        /// Flushed to flash once this much is buffered. Match the filesystem
        /// block size.
        size_t blockSizeBytes = 4096;

        /// Upper bound on how long data may sit unflushed, so a crash during a
        /// quiet moment does not lose much.
        uint32_t maxFlushIntervalMs = 4000;

        /// How often the .met file is rewritten while recording.
        uint32_t summaryIntervalMs = 10000;
    };

    TelemetryRecorder(RideStorage& storage, const Clock& clock);
    ~TelemetryRecorder();

    /// `buffer` must remain valid for the recorder's lifetime; it is supplied
    /// externally so it can be placed in PSRAM.
    bool begin(const Config& config, uint8_t* buffer, size_t bufferSize);

    /// Opens a new ride. `startUnixTime` may be 0 if GNSS has no fix yet — it
    /// is backfilled into the summary once a fix arrives.
    bool startRide(uint32_t startUnixTime, uint16_t calibrationVersion);

    /// Flushes, writes the final summary and closes the file.
    bool endRide();

    bool     isRecording() const { return file_ != nullptr; }
    uint32_t currentRideId() const { return rideId_; }

    /// Offers a fused sample. Decimated internally to imuLogRateHz, so it is
    /// safe (and expected) to call at the full fusion rate.
    void recordImu(const FusedState& fused, const ImuReading& reading);

    void recordGnss(const GnssReading& reading);
    void recordEvent(EventCode code, int32_t value);

    /// Writes buffered data to flash. Called automatically; expose it so the
    /// caller can force a flush before a risky operation.
    bool flush();

    /// Must be called regularly: drives the time-based flush and summary writes.
    void update();

    const RideSummary& summary() const { return builder_.raw(); }

    uint32_t bytesWritten() const { return bytesWritten_; }
    uint32_t droppedRecords() const { return droppedRecords_; }

private:
    bool appendRecord(RecordType type, const void* payload, uint8_t length);
    bool writeSummaryFile(bool closed);
    void closeFile();

    RideStorage& storage_;
    const Clock& clock_;
    Config       config_{};

    BlockBuffer buffer_;
    IRideFile*  file_ = nullptr;

    SummaryBuilder builder_;
    Crc32          dataCrc_;

    uint32_t rideId_             = 0;
    uint32_t startMillis_        = 0;
    uint32_t bytesWritten_       = 0;
    uint32_t droppedRecords_     = 0;
    uint32_t lastFlushMs_        = 0;
    uint32_t lastSummaryMs_      = 0;
    uint64_t nextImuLogUs_       = 0;
    uint64_t imuLogIntervalUs_   = 20000;
    uint16_t calibrationVersion_ = 0;
    bool     storageFullLogged_  = false;
    bool     writeFailed_        = false;
};

}  // namespace moto
