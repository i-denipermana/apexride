#pragma once
//
// Monotonic time source.
//
// Every subsystem shares one Clock instance, which is what makes the IMU,
// GNSS and event streams share a single timebase inside a ride file.
//
// The indirection also lets host tests drive a VirtualClock and simulate a
// 2-minute ride in a few milliseconds of wall time.
//

#include <stdint.h>

namespace moto {

class Clock {
public:
    virtual ~Clock() = default;

    /// Microseconds since boot. 64-bit, so it does not wrap in practice.
    virtual uint64_t micros() const = 0;

    /// Milliseconds since boot.
    uint32_t millis() const { return static_cast<uint32_t>(micros() / 1000u); }
};

/// Real hardware/host clock. Backed by esp_timer_get_time() on the ESP32 and
/// std::chrono::steady_clock elsewhere.
class SystemClock : public Clock {
public:
    uint64_t micros() const override;
};

/// Manually advanced clock for tests and offline replay.
class VirtualClock : public Clock {
public:
    uint64_t micros() const override { return micros_; }

    void advanceMicros(uint64_t delta) { micros_ += delta; }
    void advanceMillis(uint32_t delta) { micros_ += static_cast<uint64_t>(delta) * 1000u; }
    void reset() { micros_ = 0; }

private:
    uint64_t micros_ = 0;
};

}  // namespace moto
