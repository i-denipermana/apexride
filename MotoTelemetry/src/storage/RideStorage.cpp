#include "RideStorage.h"

#include <stdio.h>
#include <string.h>

#include "../core/Crc32.h"
#include "../core/Log.h"

namespace moto {
namespace {

constexpr size_t kRideFileNameLength = 11;  // "R000001.bin"

bool parseRideFileName(const char* name, uint32_t& idOut) {
    if (name == nullptr || name[0] != 'R') return false;
    if (strlen(name) != kRideFileNameLength) return false;
    if (strcmp(name + 7, ".bin") != 0) return false;

    uint32_t id = 0;
    for (int i = 1; i < 7; ++i) {
        if (name[i] < '0' || name[i] > '9') return false;
        id = id * 10u + static_cast<uint32_t>(name[i] - '0');
    }

    idOut = id;
    return true;
}

/// Buffered sequential reader that also CRCs everything it pulls off disk.
/// Recovery walks whole ride files, so unbuffered per-record reads would make
/// a boot-time repair take an unpleasantly long time.
class RecordStreamReader {
public:
    explicit RecordStreamReader(IRideFile& file) : file_(file) {}

    /// Reads exactly `length` bytes, or returns false having consumed nothing
    /// more than what was available.
    bool readExact(void* out, size_t length) {
        uint8_t* dest = static_cast<uint8_t*>(out);
        while (length > 0) {
            if (head_ == tail_ && !refill()) {
                return false;
            }
            const size_t available = tail_ - head_;
            const size_t chunk     = length < available ? length : available;
            memcpy(dest, buffer_ + head_, chunk);
            head_ += chunk;
            dest += chunk;
            length -= chunk;
        }
        return true;
    }

    /// Drains whatever is left so the CRC covers the entire file even when
    /// parsing stopped early at a truncated record.
    void drain() {
        head_ = tail_;
        while (refill()) {
            head_ = tail_;
        }
    }

    uint32_t crc() const { return crc_.value(); }

private:
    bool refill() {
        const size_t got = file_.read(buffer_, sizeof(buffer_));
        if (got == 0) {
            return false;
        }
        crc_.update(buffer_, got);
        head_ = 0;
        tail_ = got;
        return true;
    }

    IRideFile& file_;
    Crc32      crc_;
    uint8_t    buffer_[512];
    size_t     head_ = 0;
    size_t     tail_ = 0;
};

/// Collects ride ids while walking the rides directory.
class RideFileVisitor : public IDirectoryVisitor {
public:
    RideFileVisitor(uint32_t* ids, uint32_t* sizes, size_t capacity)
        : ids_(ids), sizes_(sizes), capacity_(capacity) {}

    void onEntry(const char* name, size_t sizeBytes) override {
        uint32_t id = 0;
        if (!parseRideFileName(name, id)) {
            return;
        }
        if (count_ >= capacity_) {
            overflowed_ = true;
            return;
        }
        ids_[count_]   = id;
        sizes_[count_] = static_cast<uint32_t>(sizeBytes);
        ++count_;
    }

    size_t count() const { return count_; }
    bool   overflowed() const { return overflowed_; }

private:
    uint32_t* ids_;
    uint32_t* sizes_;
    size_t    capacity_;
    size_t    count_      = 0;
    bool      overflowed_ = false;
};

}  // namespace

void RideStorage::dataPath(uint32_t rideId, char* out, size_t outSize) const {
    snprintf(out, outSize, "%s/R%06u.bin", config_.basePath, static_cast<unsigned>(rideId));
}

void RideStorage::summaryPath(uint32_t rideId, char* out, size_t outSize) const {
    snprintf(out, outSize, "%s/R%06u.met", config_.basePath, static_cast<unsigned>(rideId));
}

bool RideStorage::begin(IRideStore& store, const Config& config) {
    store_  = &store;
    config_ = config;

    if (!store_->begin()) {
        MT_LOGE("Storage failed to mount");
        return false;
    }

    if (!store_->ensureDirectory(config_.basePath)) {
        MT_LOGE("Could not create %s", config_.basePath);
        return false;
    }

    if (!refresh()) {
        return false;
    }

    MT_LOGI("Storage: %u rides (%u unsynced), %llu of %llu KB free",
            static_cast<unsigned>(rideCount_), static_cast<unsigned>(unsyncedCount()),
            static_cast<unsigned long long>(freeBytes() / 1024),
            static_cast<unsigned long long>(totalBytes() / 1024));
    return true;
}

void RideStorage::insertSorted(const RideEntry& entry) {
    if (rideCount_ >= kMaxRides) {
        return;
    }

    size_t position = rideCount_;
    while (position > 0 && rides_[position - 1].rideId > entry.rideId) {
        rides_[position] = rides_[position - 1];
        --position;
    }
    rides_[position] = entry;
    ++rideCount_;
}

bool RideStorage::refresh() {
    if (store_ == nullptr) {
        return false;
    }

    uint32_t ids[kMaxRides];
    uint32_t sizes[kMaxRides];

    RideFileVisitor visitor(ids, sizes, kMaxRides);
    if (!store_->list(config_.basePath, visitor)) {
        MT_LOGE("Could not list %s", config_.basePath);
        return false;
    }

    if (visitor.overflowed()) {
        MT_LOGW("More than %u rides on disk; only the first %u are indexed",
                static_cast<unsigned>(kMaxRides), static_cast<unsigned>(kMaxRides));
    }

    rideCount_  = 0;
    nextRideId_ = 1;

    for (size_t i = 0; i < visitor.count(); ++i) {
        RideEntry entry;
        entry.rideId       = ids[i];
        entry.dataBytes    = sizes[i];
        entry.summaryValid = readSummary(ids[i], entry.summary);
        insertSorted(entry);

        if (ids[i] >= nextRideId_) {
            nextRideId_ = ids[i] + 1;
        }
    }

    return true;
}

const RideStorage::RideEntry* RideStorage::findRide(uint32_t rideId) const {
    for (size_t i = 0; i < rideCount_; ++i) {
        if (rides_[i].rideId == rideId) {
            return &rides_[i];
        }
    }
    return nullptr;
}

uint32_t RideStorage::unsyncedCount() const {
    uint32_t count = 0;
    for (size_t i = 0; i < rideCount_; ++i) {
        // A ride with no readable summary is treated as unsynced, which is the
        // safe direction to be wrong in.
        if (!rides_[i].summaryValid || !rides_[i].summary.isSynced()) {
            ++count;
        }
    }
    return count;
}

bool RideStorage::canStartRide() const {
    return store_ != nullptr && freeBytes() >= config_.minFreeBytesToStart;
}

bool RideStorage::mustStopForSpace() const {
    return store_ == nullptr || freeBytes() < config_.minFreeBytesToContinue;
}

IRideFile* RideStorage::createRideFile(uint32_t rideId) {
    if (store_ == nullptr) {
        return nullptr;
    }

    char path[64];
    dataPath(rideId, path, sizeof(path));

    if (store_->exists(path)) {
        MT_LOGE("Refusing to overwrite existing ride %s", path);
        return nullptr;
    }

    return store_->open(path, FileMode::Write);
}

IRideFile* RideStorage::openRideFile(uint32_t rideId, FileMode mode) {
    if (store_ == nullptr) {
        return nullptr;
    }

    char path[64];
    dataPath(rideId, path, sizeof(path));
    return store_->open(path, mode);
}

bool RideStorage::writeSummary(const RideSummary& summary) {
    if (store_ == nullptr) {
        return false;
    }

    RideSummary copy = summary;
    finalizeSummaryCrc(copy);

    char path[64];
    summaryPath(copy.rideId, path, sizeof(path));

    IRideFile* file = store_->open(path, FileMode::Write);
    if (file == nullptr) {
        MT_LOGE("Could not write %s", path);
        return false;
    }

    const size_t written = file->write(&copy, sizeof(copy));
    const bool   flushed = file->flush();
    file->close();
    delete file;

    if (written != sizeof(copy) || !flushed) {
        MT_LOGE("Short write on %s", path);
        return false;
    }

    // Keep the in-memory index consistent without a full directory rescan.
    for (size_t i = 0; i < rideCount_; ++i) {
        if (rides_[i].rideId == copy.rideId) {
            rides_[i].summary      = copy;
            rides_[i].summaryValid = true;
            break;
        }
    }

    return true;
}

bool RideStorage::readSummary(uint32_t rideId, RideSummary& out) const {
    if (store_ == nullptr) {
        return false;
    }

    char path[64];
    summaryPath(rideId, path, sizeof(path));

    if (!store_->exists(path)) {
        return false;
    }

    IRideFile* file = store_->open(path, FileMode::Read);
    if (file == nullptr) {
        return false;
    }

    RideSummary summary{};
    const size_t got = file->read(&summary, sizeof(summary));
    file->close();
    delete file;

    if (got != sizeof(summary) || !validateSummary(summary)) {
        return false;
    }
    if (summary.rideId != rideId) {
        MT_LOGW("Summary %s claims ride %u", path, static_cast<unsigned>(summary.rideId));
        return false;
    }

    out = summary;
    return true;
}

bool RideStorage::markSynced(uint32_t rideId) {
    const RideEntry* entry = findRide(rideId);
    if (entry == nullptr) {
        MT_LOGW("markSynced: ride %u not found", static_cast<unsigned>(rideId));
        return false;
    }

    RideSummary summary;
    if (entry->summaryValid) {
        summary = entry->summary;
    } else if (!rebuildSummary(rideId, summary)) {
        MT_LOGE("markSynced: no usable summary for ride %u", static_cast<unsigned>(rideId));
        return false;
    }

    summary.flags |= kRideFlagSynced;
    if (!writeSummary(summary)) {
        return false;
    }

    MT_LOGI("Ride %u marked synced", static_cast<unsigned>(rideId));
    return true;
}

bool RideStorage::deleteRide(uint32_t rideId, bool force) {
    if (store_ == nullptr) {
        return false;
    }

    const RideEntry* entry = findRide(rideId);
    if (entry == nullptr) {
        return false;
    }

    const bool synced = entry->summaryValid && entry->summary.isSynced();
    if (!synced && !force) {
        MT_LOGW("Refusing to delete unsynced ride %u", static_cast<unsigned>(rideId));
        return false;
    }

    char dataFile[64];
    char summaryFile[64];
    dataPath(rideId, dataFile, sizeof(dataFile));
    summaryPath(rideId, summaryFile, sizeof(summaryFile));

    const bool dataRemoved = store_->remove(dataFile);
    if (store_->exists(summaryFile)) {
        store_->remove(summaryFile);
    }

    if (!dataRemoved) {
        MT_LOGE("Could not delete %s", dataFile);
        return false;
    }

    MT_LOGI("Deleted ride %u", static_cast<unsigned>(rideId));
    return refresh();
}

bool RideStorage::reclaimSpace(uint64_t neededBytes) {
    if (store_ == nullptr) {
        return false;
    }

    while (freeBytes() < neededBytes) {
        // Rides are held in ascending id order, which is also chronological,
        // so the first synced entry is the oldest eligible one.
        uint32_t victim      = 0;
        bool     foundVictim = false;
        for (size_t i = 0; i < rideCount_; ++i) {
            if (rides_[i].summaryValid && rides_[i].summary.isSynced()) {
                victim      = rides_[i].rideId;
                foundVictim = true;
                break;
            }
        }

        if (!foundVictim) {
            MT_LOGW("Storage low (%llu KB free) and every ride is unsynced — keeping them all",
                    static_cast<unsigned long long>(freeBytes() / 1024));
            return false;
        }

        if (!deleteRide(victim)) {
            return false;
        }
    }

    return true;
}

bool RideStorage::rebuildSummary(uint32_t rideId, RideSummary& out) {
    if (store_ == nullptr) {
        return false;
    }

    char path[64];
    dataPath(rideId, path, sizeof(path));

    IRideFile* file = store_->open(path, FileMode::Read);
    if (file == nullptr) {
        return false;
    }

    const size_t fileSize = file->size();

    RecordStreamReader reader(*file);

    FileHeader header{};
    if (!reader.readExact(&header, sizeof(header))) {
        MT_LOGE("Ride %u: file too short to hold a header", static_cast<unsigned>(rideId));
        file->close();
        delete file;
        return false;
    }

    const uint32_t expectedHeaderCrc = crc32(&header, sizeof(FileHeader) - sizeof(uint32_t));
    if (header.magic != kFileMagic || header.formatVersion != kFormatVersion ||
        header.headerCrc != expectedHeaderCrc) {
        MT_LOGE("Ride %u: bad file header", static_cast<unsigned>(rideId));
        file->close();
        delete file;
        return false;
    }

    SummaryBuilder builder;
    builder.reset(header.rideId, header.startUnixTime, header.startMillis,
                  header.calibrationVersion);

    bool truncated = false;

    for (;;) {
        RecordHeader recordHeader{};
        if (!reader.readExact(&recordHeader, sizeof(recordHeader))) {
            break;  // clean end of file
        }

        uint8_t payload[64];
        if (recordHeader.length > sizeof(payload)) {
            MT_LOGW("Ride %u: record type %u claims %u bytes; stopping scan",
                    static_cast<unsigned>(rideId), static_cast<unsigned>(recordHeader.type),
                    static_cast<unsigned>(recordHeader.length));
            truncated = true;
            break;
        }

        if (!reader.readExact(payload, recordHeader.length)) {
            // Power was lost partway through writing a record.
            truncated = true;
            break;
        }

        switch (static_cast<RecordType>(recordHeader.type)) {
            case RecordType::Imu:
                if (recordHeader.length == sizeof(ImuRecord)) {
                    ImuRecord record;
                    memcpy(&record, payload, sizeof(record));
                    builder.addImu(record);
                }
                break;
            case RecordType::Gnss:
                if (recordHeader.length == sizeof(GnssRecord)) {
                    GnssRecord record;
                    memcpy(&record, payload, sizeof(record));
                    builder.addGnss(record);
                }
                break;
            case RecordType::Event:
                if (recordHeader.length == sizeof(EventRecord)) {
                    EventRecord record;
                    memcpy(&record, payload, sizeof(record));
                    builder.addEvent(record);
                }
                break;
            default:
                // Unknown record type from a newer firmware: the length prefix
                // means it can simply be skipped.
                break;
        }
    }

    reader.drain();

    builder.setFileSize(static_cast<uint32_t>(fileSize));
    builder.setDataCrc(reader.crc());
    builder.setFlag(kRideFlagClosed | kRideFlagRecovered);
    if (truncated) {
        builder.setFlag(kRideFlagTruncated);
    }

    file->close();
    delete file;

    out = builder.build();
    return true;
}

uint32_t RideStorage::recoverIncompleteRides() {
    uint32_t repaired = 0;

    for (size_t i = 0; i < rideCount_; ++i) {
        if (rides_[i].summaryValid) {
            continue;
        }

        const uint32_t rideId = rides_[i].rideId;
        MT_LOGW("Ride %u has no valid summary; rebuilding from data",
                static_cast<unsigned>(rideId));

        RideSummary rebuilt;
        if (!rebuildSummary(rideId, rebuilt)) {
            MT_LOGE("Ride %u could not be recovered", static_cast<unsigned>(rideId));
            continue;
        }

        if (!writeSummary(rebuilt)) {
            continue;
        }

        MT_LOGI("Ride %u recovered: %u IMU / %u GNSS samples, %u s",
                static_cast<unsigned>(rideId), static_cast<unsigned>(rebuilt.imuSampleCount),
                static_cast<unsigned>(rebuilt.gnssSampleCount),
                static_cast<unsigned>(rebuilt.durationMs / 1000));
        ++repaired;
    }

    return repaired;
}

}  // namespace moto
