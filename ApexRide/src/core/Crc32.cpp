#include "Crc32.h"

namespace apex {

void Crc32::update(const void* data, size_t length) {
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    uint32_t       state = state_;

    for (size_t i = 0; i < length; ++i) {
        state ^= bytes[i];
        for (int bit = 0; bit < 8; ++bit) {
            const uint32_t mask = -(state & 1u);
            state = (state >> 1) ^ (0xEDB88320u & mask);
        }
    }

    state_ = state;
}

uint32_t crc32(const void* data, size_t length) {
    Crc32 c;
    c.update(data, length);
    return c.value();
}

}  // namespace apex
