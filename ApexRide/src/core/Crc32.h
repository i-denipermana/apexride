#pragma once
//
// CRC-32 (IEEE 802.3, reflected, polynomial 0xEDB88320) — the same variant used
// by zlib/PNG, so a phone app can verify a downloaded ride with any stock CRC32
// implementation.
//
// Computed bitwise to avoid a 1 KB lookup table. At our logging rates (a few
// kB/s) the cost is negligible.
//

#include <stddef.h>
#include <stdint.h>

namespace apex {

/// Incremental CRC-32, so a ride file can be checksummed as it is written
/// rather than re-read at ride close.
class Crc32 {
public:
    void reset() { state_ = 0xFFFFFFFFu; }

    void update(const void* data, size_t length);

    uint32_t value() const { return state_ ^ 0xFFFFFFFFu; }

private:
    uint32_t state_ = 0xFFFFFFFFu;
};

/// One-shot convenience wrapper.
uint32_t crc32(const void* data, size_t length);

}  // namespace apex
