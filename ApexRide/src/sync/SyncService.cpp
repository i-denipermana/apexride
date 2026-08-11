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
    if (openFile_ == nullptr) {
        return;
    }

    if (clock_.millis() - lastActivityMs_ >= config_.transferIdleTimeoutMs) {
        APEX_LOGI("Sync transfer of ride %u timed out; releasing the file",
                  static_cast<unsigned>(openRideId_));
        closeTransfer();
    }
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

    APEX_LOGI("Ride %u verified by the phone and marked synced",
              static_cast<unsigned>(rideId));
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
