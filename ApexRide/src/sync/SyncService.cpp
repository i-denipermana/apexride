#include "SyncService.h"

#include "../core/Log.h"

namespace apex {

SyncService::SyncService(RideStorage& storage, const Clock& clock)
    : storage_(storage), clock_(clock) {}

SyncService::~SyncService() {
    closeTransfer();
}

bool SyncService::begin(const Config& config) {
    config_ = config;
    APEX_LOGI("Sync service ready as '%s'", config_.deviceName);
    return true;
}

void SyncService::closeTransfer() {
    if (openFile_ != nullptr) {
        openFile_->close();
        delete openFile_;
        openFile_ = nullptr;
    }
    openRideId_ = 0;
    openOffset_ = 0;
}

void SyncService::update() {
    const uint32_t nowMs = clock_.millis();

    if (openFile_ != nullptr && nowMs - lastActivityMs_ >= config_.transferIdleTimeoutMs) {
        APEX_LOGI("Sync transfer of ride %u timed out; releasing the file",
                  static_cast<unsigned>(openRideId_));
        closeTransfer();
    }

    if (session_.active && nowMs - session_.lastActivityMs >= config_.sessionIdleTimeoutMs) {
        APEX_LOGI("Sync session idle for %u s; closing so the radio can be shut down",
                  static_cast<unsigned>(config_.sessionIdleTimeoutMs / 1000));
        endSession();
    }
}

void SyncService::touchSession() {
    if (session_.active) {
        session_.lastActivityMs = clock_.millis();
    }
}

SyncService::Result SyncService::beginSession(bool force) {
    if (session_.active) {
        touchSession();
        return Result::Ok;  // idempotent: reconnecting mid-session is normal
    }

    if (config_.refuseSyncWhileRecording && !force && deviceStatus().recording) {
        APEX_LOGW("Sync refused: a ride is being recorded");
        return Result::Conflict;
    }

    session_                   = Session();
    session_.active            = true;
    session_.startedMs         = clock_.millis();
    session_.lastActivityMs    = session_.startedMs;

    APEX_LOGI("Sync session opened: %u ride(s), %llu KB outstanding",
              static_cast<unsigned>(pendingRideCount()),
              static_cast<unsigned long long>(pendingBytes() / 1024));
    return Result::Ok;
}

void SyncService::endSession() {
    if (!session_.active) {
        return;
    }

    closeTransfer();

    APEX_LOGI("Sync session closed: %u ride(s) acknowledged, %llu KB served, %u still pending",
              static_cast<unsigned>(session_.ridesAcknowledged),
              static_cast<unsigned long long>(session_.bytesServed / 1024),
              static_cast<unsigned>(pendingRideCount()));

    session_.active = false;
}

size_t SyncService::pendingRideCount() const {
    size_t count = 0;
    for (size_t i = 0; i < storage_.rideCount(); ++i) {
        const RideStorage::RideEntry& entry = storage_.rideAt(i);
        // A ride with no valid summary has no CRC to verify against, so the
        // phone cannot acknowledge it; offering it would loop forever.
        if (entry.summaryValid && entry.summary.isClosed() && !entry.summary.isSynced()) {
            ++count;
        }
    }
    return count;
}

bool SyncService::pendingRideAt(size_t index, uint32_t& rideIdOut) const {
    // Newest first: rides_ is held in ascending id order, so walk it backwards.
    size_t seen = 0;
    for (size_t i = storage_.rideCount(); i > 0; --i) {
        const RideStorage::RideEntry& entry = storage_.rideAt(i - 1);
        if (!entry.summaryValid || !entry.summary.isClosed() || entry.summary.isSynced()) {
            continue;
        }
        if (seen == index) {
            rideIdOut = entry.rideId;
            return true;
        }
        ++seen;
    }
    return false;
}

uint64_t SyncService::pendingBytes() const {
    uint64_t total = 0;
    for (size_t i = 0; i < storage_.rideCount(); ++i) {
        const RideStorage::RideEntry& entry = storage_.rideAt(i);
        if (entry.summaryValid && entry.summary.isClosed() && !entry.summary.isSynced()) {
            total += entry.summary.fileSizeBytes;
        }
    }
    return total;
}

DeviceStatus SyncService::deviceStatus() const {
    DeviceStatus status;
    if (statusSource_ != nullptr) {
        statusSource_->fillStatus(status);
    }
    return status;
}

bool SyncService::isRideInFlight(uint32_t rideId, const RideSummary& summary) const {
    const DeviceStatus status = deviceStatus();
    if (status.recording && status.activeRideId == rideId) {
        return true;
    }
    return !summary.isClosed();
}

SyncService::Result SyncService::readRideChunk(uint32_t rideId, uint32_t offset, uint8_t* out,
                                               size_t maxLength, size_t& readLength) {
    readLength = 0;

    if (out == nullptr || maxLength == 0) {
        return Result::BadRequest;
    }

    const RideStorage::RideEntry* entry = storage_.findRide(rideId);
    if (entry == nullptr) {
        return Result::NotFound;
    }

    if (entry->summaryValid && isRideInFlight(rideId, entry->summary)) {
        // Serving a growing file would hand the phone bytes that do not match
        // the CRC the ride will eventually have.
        return Result::Conflict;
    }

    if (maxLength > config_.maxChunkBytes) {
        maxLength = config_.maxChunkBytes;
    }

    // Reuse the cached handle when the phone is reading sequentially, which is
    // the normal case; seek only when it is not.
    if (openFile_ != nullptr && openRideId_ != rideId) {
        closeTransfer();
    }

    if (openFile_ == nullptr) {
        openFile_ = storage_.openRideFile(rideId, FileMode::Read);
        if (openFile_ == nullptr) {
            return Result::IoError;
        }
        openRideId_ = rideId;
        openOffset_ = 0;
    }

    const uint32_t fileSize = static_cast<uint32_t>(openFile_->size());
    if (offset > fileSize) {
        return Result::BadRequest;
    }
    if (offset == fileSize) {
        return Result::Ok;  // clean end of file, zero bytes read
    }

    if (offset != openOffset_) {
        if (!openFile_->seek(offset)) {
            closeTransfer();
            return Result::IoError;
        }
        openOffset_ = offset;
    }

    const uint32_t remaining = fileSize - offset;
    if (maxLength > remaining) {
        maxLength = remaining;
    }

    readLength = openFile_->read(out, maxLength);
    openOffset_ += static_cast<uint32_t>(readLength);
    lastActivityMs_ = clock_.millis();
    ++chunksServed_;

    session_.bytesServed += readLength;
    touchSession();

    return Result::Ok;
}

SyncService::Result SyncService::acknowledge(uint32_t rideId, uint32_t checksumFromPhone) {
    const RideStorage::RideEntry* entry = storage_.findRide(rideId);
    if (entry == nullptr) {
        return Result::NotFound;
    }

    if (!entry->summaryValid) {
        APEX_LOGW("Sync ack for ride %u refused: no valid summary to check against",
                  static_cast<unsigned>(rideId));
        return Result::Conflict;
    }

    if (isRideInFlight(rideId, entry->summary)) {
        return Result::Conflict;
    }

    if (entry->summary.dataCrc != checksumFromPhone) {
        // The phone received something other than what is on flash. Leave the
        // ride unsynced so it stays undeletable and can be retried.
        APEX_LOGW("Sync ack for ride %u REJECTED: phone reported 0x%08x, device has 0x%08x",
                  static_cast<unsigned>(rideId), static_cast<unsigned>(checksumFromPhone),
                  static_cast<unsigned>(entry->summary.dataCrc));
        return Result::Conflict;
    }

    if (entry->summary.isSynced()) {
        return Result::Ok;  // idempotent: a repeated ack is not an error
    }

    // Release the handle first: on LittleFS, rewriting metadata while a read
    // handle is open to the same ride is asking for trouble.
    if (openRideId_ == rideId) {
        closeTransfer();
    }

    if (!storage_.markSynced(rideId)) {
        return Result::IoError;
    }

    session_.ridesAcknowledged++;
    touchSession();

    APEX_LOGI("Ride %u verified by the phone and marked synced (%u still pending)",
              static_cast<unsigned>(rideId), static_cast<unsigned>(pendingRideCount()));
    return Result::Ok;
}

SyncService::Result SyncService::deleteRide(uint32_t rideId) {
    const RideStorage::RideEntry* entry = storage_.findRide(rideId);
    if (entry == nullptr) {
        return Result::NotFound;
    }

    if (!entry->summaryValid || !entry->summary.isSynced()) {
        // Matches the retention policy: unsynced data is never destroyed, not
        // even on request.
        return Result::Conflict;
    }

    if (openRideId_ == rideId) {
        closeTransfer();
    }

    return storage_.deleteRide(rideId) ? Result::Ok : Result::IoError;
}

}  // namespace apex
