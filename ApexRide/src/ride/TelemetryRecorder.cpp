#include "TelemetryRecorder.h"

#include <string.h>

#include "../core/Log.h"

namespace apex {

TelemetryRecorder::TelemetryRecorder(RideStorage& storage, const Clock& clock)
    : storage_(storage), clock_(clock) {}

TelemetryRecorder::~TelemetryRecorder() {
    closeFile();
}

bool TelemetryRecorder::begin(const Config& config, uint8_t* buffer, size_t bufferSize) {
    config_ = config;

    if (buffer == nullptr || bufferSize < config.blockSizeBytes) {
        APEX_LOGE("Recorder buffer too small: %u < %u", static_cast<unsigned>(bufferSize),
                static_cast<unsigned>(config.blockSizeBytes));
        return false;
    }

    buffer_.attach(buffer, bufferSize);

    imuLogIntervalUs_ = config.imuLogRateHz > 0.0f
                            ? static_cast<uint64_t>(1e6f / config.imuLogRateHz)
                            : 0;
    return true;
}

bool TelemetryRecorder::startRide(uint32_t startUnixTime, uint16_t calibrationVersion) {
    if (isRecording()) {
        APEX_LOGW("startRide called while ride %u is open", static_cast<unsigned>(rideId_));
        return false;
    }

    if (!storage_.canStartRide()) {
        // Try to make room by dropping already-synced rides. Unsynced rides are
        // never touched, so this can legitimately fail.
        if (!storage_.reclaimSpace(storage_.minFreeBytesToStart())) {
            APEX_LOGE("Not enough space to start a ride");
            return false;
        }
    }

    const uint32_t rideId = storage_.nextRideId();

    file_ = storage_.createRideFile(rideId);
    if (file_ == nullptr) {
        APEX_LOGE("Could not create data file for ride %u", static_cast<unsigned>(rideId));
        return false;
    }

    // Bring the new ride into the index straight away so status reporting and
    // the unsynced count include the ride currently being recorded.
    storage_.refresh();

    rideId_              = rideId;
    startMillis_         = clock_.millis();
    calibrationVersion_  = calibrationVersion;
    bytesWritten_        = 0;
    droppedRecords_      = 0;
    storageFullLogged_   = false;
    writeFailed_         = false;
    lastFlushMs_         = startMillis_;
    lastSummaryMs_       = startMillis_;
    nextImuLogUs_        = clock_.micros();

    buffer_.clear();
    dataCrc_.reset();

    FileHeader header{};
    header.magic              = kFileMagic;
    header.formatVersion      = kFormatVersion;
    header.headerSize         = sizeof(FileHeader);
    header.rideId             = rideId;
    header.startUnixTime      = startUnixTime;
    header.startMillis        = startMillis_;
    header.firmwareVersion    = kFirmwareVersion;
    header.calibrationVersion = calibrationVersion;
    header.imuLogRateHz       = static_cast<uint16_t>(config_.imuLogRateHz);
    header.gnssLogRateHz      = 0;  // GNSS is logged as it arrives, not at a fixed rate
    header.headerCrc          = crc32(&header, sizeof(FileHeader) - sizeof(uint32_t));

    if (!buffer_.append(&header, sizeof(header))) {
        APEX_LOGE("Buffer cannot hold the file header");
        closeFile();
        return false;
    }
    dataCrc_.update(&header, sizeof(header));

    builder_.reset(rideId, startUnixTime, startMillis_, calibrationVersion);

    recordEvent(EventCode::RideStart, static_cast<int32_t>(rideId));

    // Write the header and an initial summary immediately, so a ride that is
    // interrupted seconds after starting still appears in the catalogue.
    flush();
    writeSummaryFile(false);

    APEX_LOGI("Ride %u started (unix %u)", static_cast<unsigned>(rideId),
            static_cast<unsigned>(startUnixTime));
    return true;
}

bool TelemetryRecorder::appendRecord(RecordType type, const void* payload, uint8_t length) {
    if (!isRecording()) {
        return false;
    }

    const size_t needed = sizeof(RecordHeader) + length;

    if (buffer_.remaining() < needed && !flush()) {
        ++droppedRecords_;
        return false;
    }

    RecordHeader header;
    header.type   = static_cast<uint8_t>(type);
    header.length = length;

    if (!buffer_.append(&header, sizeof(header)) || !buffer_.append(payload, length)) {
        // Should be unreachable after the flush above, but losing a sample is
        // strictly better than corrupting the stream with a partial record.
        ++droppedRecords_;
        return false;
    }

    dataCrc_.update(&header, sizeof(header));
    dataCrc_.update(payload, length);
    return true;
}

void TelemetryRecorder::recordImu(const FusedState& fused, const ImuReading& reading) {
    if (!isRecording() || imuLogIntervalUs_ == 0) {
        return;
    }

    if (reading.timestampUs < nextImuLogUs_) {
        return;  // decimating the fusion rate down to the log rate
    }

    nextImuLogUs_ += imuLogIntervalUs_;
    if (nextImuLogUs_ + imuLogIntervalUs_ < reading.timestampUs) {
        // Fell behind badly; resynchronise instead of trying to catch up.
        nextImuLogUs_ = reading.timestampUs + imuLogIntervalUs_;
    }

    ImuRecord record;
    record.timestampMs = static_cast<uint32_t>(reading.timestampUs / 1000u);
    record.roll        = toCentiDegrees(fused.rollDeg());
    record.pitch       = toCentiDegrees(fused.pitchDeg());
    record.yaw         = toCentiDegrees(fused.yawDeg());
    record.accelX      = toMilliG(reading.accel.x);
    record.accelY      = toMilliG(reading.accel.y);
    record.accelZ      = toMilliG(reading.accel.z);
    record.gyroX       = toDeciDps(reading.gyro.x);
    record.gyroY       = toDeciDps(reading.gyro.y);
    record.gyroZ       = toDeciDps(reading.gyro.z);

    if (appendRecord(RecordType::Imu, &record, sizeof(record))) {
        builder_.addImu(record);
    }
}

void TelemetryRecorder::recordGnss(const GnssReading& reading) {
    if (!isRecording()) {
        return;
    }

    GnssRecord record;
    record.timestampMs = static_cast<uint32_t>(reading.timestampUs / 1000u);
    record.unixTime    = reading.unixTime;
    record.latitude    = toDegreesE7(reading.latitude);
    record.longitude   = toDegreesE7(reading.longitude);
    record.speed       = toCmPerSec(reading.speedMps);
    record.heading     = toCentiDegreesUnsigned(reading.courseDeg);
    record.altitude    = toDecimetres(reading.altitudeM);
    record.hdop        = toHdopByte(reading.hdop);
    record.satellites  = reading.satellites;
    record.fixType     = static_cast<uint8_t>(reading.fix);
    record.reserved    = 0;

    if (appendRecord(RecordType::Gnss, &record, sizeof(record))) {
        builder_.addGnss(record);
    }
}

void TelemetryRecorder::recordEvent(EventCode code, int32_t value) {
    if (!isRecording()) {
        return;
    }

    EventRecord record;
    record.timestampMs = clock_.millis();
    record.code        = static_cast<uint8_t>(code);
    record.reserved    = 0;
    record.value       = value;

    if (appendRecord(RecordType::Event, &record, sizeof(record))) {
        builder_.addEvent(record);
    }
}

bool TelemetryRecorder::flush() {
    if (!isRecording() || buffer_.empty()) {
        return true;
    }

    const size_t pending = buffer_.size();
    const size_t written = file_->write(buffer_.data(), pending);

    if (written != pending) {
        // Almost always a full filesystem. The running CRC has already absorbed
        // these bytes, so it no longer describes what is on disk; endRide()
        // recovers by rescanning the file instead of trusting it.
        writeFailed_ = true;
        if (!storageFullLogged_) {
            APEX_LOGE("Ride %u: wrote %u of %u bytes — storage is full",
                    static_cast<unsigned>(rideId_), static_cast<unsigned>(written),
                    static_cast<unsigned>(pending));
            storageFullLogged_ = true;
        }
        buffer_.clear();
        return false;
    }

    file_->flush();
    buffer_.clear();

    bytesWritten_ += static_cast<uint32_t>(written);
    lastFlushMs_ = clock_.millis();
    return true;
}

void TelemetryRecorder::update() {
    if (!isRecording()) {
        return;
    }

    const uint32_t nowMs = clock_.millis();

    if (buffer_.size() >= config_.blockSizeBytes ||
        (!buffer_.empty() && nowMs - lastFlushMs_ >= config_.maxFlushIntervalMs)) {
        flush();
    }

    if (nowMs - lastSummaryMs_ >= config_.summaryIntervalMs) {
        writeSummaryFile(false);
        lastSummaryMs_ = nowMs;
    }
}

bool TelemetryRecorder::writeSummaryFile(bool closed) {
    builder_.setFileSize(bytesWritten_);
    builder_.setDataCrc(dataCrc_.value());

    if (closed) {
        builder_.setFlag(kRideFlagClosed);
    } else {
        builder_.clearFlag(kRideFlagClosed);
    }

    return storage_.writeSummary(builder_.build());
}

void TelemetryRecorder::closeFile() {
    if (file_ != nullptr) {
        file_->close();
        delete file_;
        file_ = nullptr;
    }
}

bool TelemetryRecorder::endRide() {
    if (!isRecording()) {
        return false;
    }

    recordEvent(EventCode::RideEnd, static_cast<int32_t>(builder_.raw().durationMs / 1000u));

    const bool flushed = flush();
    const uint32_t endedRideId = rideId_;

    closeFile();

    // The summary is written after the data file is closed so that its recorded
    // size and CRC describe exactly what is on disk.
    bool summaryWritten;
    if (writeFailed_) {
        // Some records never reached the disk, so the accumulated summary and
        // CRC describe a file that does not exist. Rebuild both by reading back
        // what actually got written — otherwise the phone would reject the ride
        // as corrupt and it could never be synced or reclaimed.
        APEX_LOGW("Ride %u had failed writes; rebuilding its summary from disk",
                static_cast<unsigned>(endedRideId));

        RideSummary rebuilt;
        if (storage_.rebuildSummary(endedRideId, rebuilt)) {
            rebuilt.flags |= kRideFlagTruncated;
            summaryWritten = storage_.writeSummary(rebuilt);
        } else {
            summaryWritten = writeSummaryFile(true);
        }
    } else {
        summaryWritten = writeSummaryFile(true);
    }

    const RideSummary& s = builder_.raw();
    APEX_LOGI("Ride %u ended: %u s, %u.%02u km, %u IMU / %u GNSS samples, %u KB",
            static_cast<unsigned>(rideId_), static_cast<unsigned>(s.durationMs / 1000u),
            static_cast<unsigned>(s.distanceCm / 100000u),
            static_cast<unsigned>((s.distanceCm / 1000u) % 100u),
            static_cast<unsigned>(s.imuSampleCount), static_cast<unsigned>(s.gnssSampleCount),
            static_cast<unsigned>(bytesWritten_ / 1024u));

    if (droppedRecords_ > 0) {
        APEX_LOGW("Ride %u dropped %u records", static_cast<unsigned>(rideId_),
                static_cast<unsigned>(droppedRecords_));
    }

    rideId_ = 0;
    storage_.refresh();

    return flushed && summaryWritten;
}

}  // namespace apex
