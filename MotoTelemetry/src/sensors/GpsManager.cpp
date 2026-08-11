#include "GpsManager.h"

#include "../core/Log.h"

namespace moto {

GpsManager::GpsManager(IGnssSensor& sensor, const Clock& clock) : sensor_(sensor), clock_(clock) {}

bool GpsManager::begin(const Config& config) {
    config_ = config;

    if (!sensor_.begin()) {
        MT_LOGE("GNSS %s failed to start", sensor_.name());
        return false;
    }

    MT_LOGI("GNSS %s ready at %.0f Hz", sensor_.name(),
            static_cast<double>(sensor_.updateRateHz()));
    return true;
}

bool GpsManager::poll(GnssReading& out) {
    GnssReading reading;
    if (!sensor_.read(reading)) {
        return false;
    }

    const bool hadFix = lastReading_.hasFix();

    if (reading.hasFix()) {
        lastFixUs_ = reading.timestampUs;
        fixCount_++;
        if (reading.unixTime != 0) {
            lastUnixTime_ = reading.unixTime;
        }
        if (!everHadFix_) {
            everHadFix_ = true;
            MT_LOGI("GNSS first fix: %u satellites, HDOP %.1f",
                    static_cast<unsigned>(reading.satellites), static_cast<double>(reading.hdop));
        } else if (!hadFix) {
            MT_LOGI("GNSS fix reacquired");
        }
    } else if (hadFix) {
        MT_LOGW("GNSS fix lost");
    }

    lastReading_ = reading;
    out          = reading;
    return true;
}

bool GpsManager::speedHint(float& speedMpsOut) const {
    if (!lastReading_.hasFix()) {
        return false;
    }
    if (lastReading_.hdop > config_.maxUsableHdop) {
        return false;
    }

    const uint64_t now = clock_.micros();
    if (now < lastFixUs_) {
        return false;
    }
    if ((now - lastFixUs_) / 1000ull > config_.maxHintAgeMs) {
        return false;
    }

    speedMpsOut = lastReading_.speedMps;
    return true;
}

}  // namespace moto
