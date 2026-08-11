#include "Clock.h"

#if defined(ARDUINO)
#include <esp_timer.h>
#else
#include <chrono>
#endif

namespace moto {

uint64_t SystemClock::micros() const {
#if defined(ARDUINO)
    // esp_timer_get_time() is a 64-bit monotonic microsecond counter, unlike
    // Arduino's micros() which wraps every ~71 minutes.
    return static_cast<uint64_t>(esp_timer_get_time());
#else
    using namespace std::chrono;
    static const steady_clock::time_point origin = steady_clock::now();
    return static_cast<uint64_t>(
        duration_cast<microseconds>(steady_clock::now() - origin).count());
#endif
}

}  // namespace moto
