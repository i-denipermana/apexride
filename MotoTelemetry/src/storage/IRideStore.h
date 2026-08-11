#pragma once
//
// Filesystem abstraction.
//
// On the device this is LittleFS on internal flash. Host tests substitute a
// plain-directory implementation, which is what lets the whole recording
// pipeline be verified without hardware.
//
// Deliberately narrow: open/read/write/flush/close, delete, list, and space
// accounting. Nothing above this layer knows what a LittleFS is.
//

#include <stddef.h>
#include <stdint.h>

namespace moto {

enum class FileMode {
    Read,
    Write,   ///< create or truncate
    Append,  ///< create if absent, otherwise append
};

class IRideFile {
public:
    virtual ~IRideFile() = default;

    virtual size_t write(const void* data, size_t length) = 0;
    virtual size_t read(void* buffer, size_t length)      = 0;

    /// Commits buffered data. Called after every block write, so a power cut
    /// costs at most one unflushed block.
    virtual bool flush() = 0;

    virtual bool   seek(size_t offset) = 0;
    virtual size_t size() const        = 0;
    virtual void   close()             = 0;
};

/// Receives one filename (not a full path) per entry during a listing.
class IDirectoryVisitor {
public:
    virtual ~IDirectoryVisitor() = default;
    virtual void onEntry(const char* name, size_t sizeBytes) = 0;
};

class IRideStore {
public:
    virtual ~IRideStore() = default;

    virtual bool begin() = 0;

    /// Creates `path` and any parent directory. No-op if it already exists.
    virtual bool ensureDirectory(const char* path) = 0;

    /// Returns nullptr on failure. The caller owns the returned file and must
    /// delete it (TelemetryRecorder and RideStorage use unique_ptr).
    virtual IRideFile* open(const char* path, FileMode mode) = 0;

    virtual bool exists(const char* path)                    = 0;
    virtual bool remove(const char* path)                    = 0;
    virtual bool list(const char* directory, IDirectoryVisitor& visitor) = 0;

    virtual uint64_t totalBytes() const = 0;
    virtual uint64_t usedBytes() const  = 0;

    uint64_t freeBytes() const {
        const uint64_t total = totalBytes();
        const uint64_t used  = usedBytes();
        return used >= total ? 0 : total - used;
    }
};

}  // namespace moto
