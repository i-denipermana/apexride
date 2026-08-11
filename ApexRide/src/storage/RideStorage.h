#pragma once
//
// Ride catalogue: allocation, summaries, recovery and retention.
//
// ---------------------------------------------------------------------------
// Retention rules (from the handoff, enforced here)
//
//   - an unsynced ride is NEVER deleted automatically
//   - only synced rides are eligible, oldest first
//   - when nothing is eligible, reclaiming fails and the caller must warn
//     rather than quietly dropping data
//
// ---------------------------------------------------------------------------
// Crash recovery
//
// Ride IDs are derived by scanning the directory rather than kept in a counter
// file, so a lost counter can never cause an existing ride to be overwritten.
//
// A ride whose .met file is missing or fails CRC (battery died mid-ride) is
// rebuilt by replaying its .bin through the same SummaryBuilder used during
// recording, and flagged kRideFlagRecovered.
//

#include <stddef.h>
#include <stdint.h>

#include "../format/SummaryBuilder.h"
#include "IRideStore.h"

namespace apex {

class RideStorage {
public:
    /// Upper bound on rides tracked in RAM. At ~10 MB of usable flash and a
    /// few MB per ride this is far more than the filesystem can hold.
    static constexpr size_t kMaxRides = 64;

    struct Config {
        const char* basePath = "/rides";

        /// Recording refuses to start unless this much space is free.
        uint64_t minFreeBytesToStart = 512u * 1024u;

        /// An in-progress ride stops when free space falls this low, leaving
        /// room to close the file and write its summary.
        uint64_t minFreeBytesToContinue = 96u * 1024u;
    };

    struct RideEntry {
        uint32_t    rideId       = 0;
        RideSummary summary      = {};
        bool        summaryValid = false;
        uint32_t    dataBytes    = 0;
    };

    bool begin(IRideStore& store, const Config& config);

    /// Re-reads the directory. Called by begin() and after any mutation.
    bool refresh();

    size_t           rideCount() const { return rideCount_; }
    const RideEntry& rideAt(size_t index) const { return rides_[index]; }
    const RideEntry* findRide(uint32_t rideId) const;

    uint32_t unsyncedCount() const;
    uint32_t nextRideId() const { return nextRideId_; }

    /// Creates the .bin for a new ride and returns it open for append.
    /// Ownership passes to the caller. Returns nullptr if space is short.
    IRideFile* createRideFile(uint32_t rideId);

    IRideFile* openRideFile(uint32_t rideId, FileMode mode);

    bool writeSummary(const RideSummary& summary);
    bool readSummary(uint32_t rideId, RideSummary& out) const;

    /// Marks a ride synced. Call only after the phone has verified the
    /// checksum and acknowledged — this is what makes the ride deletable.
    bool markSynced(uint32_t rideId);

    /// Deletes both files for a ride. Refuses unsynced rides unless `force`.
    bool deleteRide(uint32_t rideId, bool force = false);

    /// Frees space by deleting the oldest synced rides until `neededBytes` are
    /// available. Returns false when no more rides are eligible.
    bool reclaimSpace(uint64_t neededBytes);

    /// Rebuilds any missing or corrupt summary by replaying the ride data.
    /// Returns the number of rides repaired.
    uint32_t recoverIncompleteRides();

    /// Replays a .bin and recomputes its summary from scratch.
    bool rebuildSummary(uint32_t rideId, RideSummary& out);

    uint64_t freeBytes() const { return store_ != nullptr ? store_->freeBytes() : 0; }
    uint64_t totalBytes() const { return store_ != nullptr ? store_->totalBytes() : 0; }

    bool canStartRide() const;

    uint64_t minFreeBytesToStart() const { return config_.minFreeBytesToStart; }

    /// True when an in-progress ride must stop to protect the filesystem.
    bool mustStopForSpace() const;

    void dataPath(uint32_t rideId, char* out, size_t outSize) const;
    void summaryPath(uint32_t rideId, char* out, size_t outSize) const;

private:
    void insertSorted(const RideEntry& entry);

    IRideStore* store_ = nullptr;
    Config      config_{};

    RideEntry rides_[kMaxRides];
    size_t    rideCount_  = 0;
    uint32_t  nextRideId_ = 1;
};

}  // namespace apex
