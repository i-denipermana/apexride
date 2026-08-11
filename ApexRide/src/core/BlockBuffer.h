#pragma once
//
// Fixed-capacity staging buffer for encoded telemetry records.
//
// Telemetry is appended here at the log rate and handed to the filesystem in
// whole blocks (see TelemetryRecorder). Writing ~1 kB/s straight to LittleFS
// would cause heavy write amplification; batching into filesystem-block-sized
// chunks avoids that.
//
// The backing storage is supplied by the caller so it can live in PSRAM.
//

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace apex {

class BlockBuffer {
public:
    BlockBuffer() = default;

    BlockBuffer(uint8_t* storage, size_t capacity) : storage_(storage), capacity_(capacity) {}

    void attach(uint8_t* storage, size_t capacity) {
        storage_  = storage;
        capacity_ = capacity;
        size_     = 0;
    }

    /// Appends `length` bytes. Returns false without writing anything if the
    /// buffer cannot hold them — the caller is expected to flush first.
    bool append(const void* data, size_t length) {
        if (storage_ == nullptr || size_ + length > capacity_) {
            return false;
        }
        memcpy(storage_ + size_, data, length);
        size_ += length;
        return true;
    }

    const uint8_t* data() const { return storage_; }
    size_t         size() const { return size_; }
    size_t         capacity() const { return capacity_; }
    size_t         remaining() const { return capacity_ - size_; }
    bool           empty() const { return size_ == 0; }

    void clear() { size_ = 0; }

private:
    uint8_t* storage_  = nullptr;
    size_t   capacity_ = 0;
    size_t   size_     = 0;
};

}  // namespace apex
