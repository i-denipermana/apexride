#pragma once
//
// IRideStore backed by a real directory on the development machine.
//
// Shared by the test suite and by tools/syncserver, so both exercise the same
// filesystem behaviour the firmware sees.
//
// This is what lets the entire recording pipeline — encoding, buffering,
// flushing, summaries, recovery and retention — be tested before any hardware
// exists. It also emulates a fixed-capacity filesystem so the retention policy
// can be driven into its "storage full" corner on demand.
//

#include <stdint.h>

#include <string>

#include "../ApexRide/src/storage/IRideStore.h"

namespace apex {

class HostRideStore : public IRideStore {
public:
    HostRideStore(std::string rootPath, uint64_t capacityBytes);

    /// Deletes and recreates the root directory.
    bool reset();

    bool begin() override;
    bool ensureDirectory(const char* path) override;

    IRideFile* open(const char* path, FileMode mode) override;
    bool       exists(const char* path) override;
    bool       remove(const char* path) override;
    bool       list(const char* directory, IDirectoryVisitor& visitor) override;

    uint64_t totalBytes() const override { return capacityBytes_; }
    uint64_t usedBytes() const override;

    /// Shrinks the emulated capacity, to exercise the retention path.
    void setCapacityBytes(uint64_t bytes) { capacityBytes_ = bytes; }

    const std::string& rootPath() const { return rootPath_; }

    /// Truncates a file to `newSize`, emulating a power cut mid-write.
    bool truncate(const char* path, uint64_t newSize);

    /// Marks the cached usage figure stale. Called by open files after a write.
    void invalidateUsage() { usageDirty_ = true; }

private:
    std::string resolve(const char* path) const;

    std::string rootPath_;
    uint64_t    capacityBytes_;

    // usedBytes() is consulted on every firmware update() call. Walking the
    // directory each time turned a 110-second simulated ride into a minute of
    // wall time, so the figure is cached and invalidated on mutation.
    mutable uint64_t cachedUsage_ = 0;
    mutable bool     usageDirty_  = true;
};

}  // namespace apex
