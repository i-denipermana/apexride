#include "GpsManager.h"

#include <math.h>

#include "../core/Log.h"

namespace apex {

GpsManager::GpsManager(IGnssSensor& sensor, const Clock& clock) : sensor_(sensor), clock_(clock) {}

bool GpsManager::begin(const Config& config) {
    config_ = config;

    if (!sensor_.begin()) {
        APEX_LOGE("GNSS %s failed to start", sensor_.name());
        return false;
    }

    APEX_LOGI("GNSS %s ready at %.0f Hz", sensor_.name(),
            static_cast<double>(sensor_.updateRateHz()));
    return true;
}

bool GpsManager::poll(GnssReading& out) {
    GnssReading reading;
    if (!sensor_.read(reading)) {
        return false;
    }

    const bool hadFix = lastReading_.hasFix();
    ++solutionCount_;
    rawSpeedMps_ = reading.speedMps;

    if (reading.hasFix()) {
        const float deadbanded = reading.speedMps < config_.speedDeadbandMps
                                     ? 0.0f : reading.speedMps;
        if (deadbanded == 0.0f) {
            // Reset at rest so the EMA cannot take several seconds to decay.
            smoothedSpeedMps_ = 0.0f;
            filteredSpeedMps_ = 0.0f;
            speedFilterValid_ = true;
        } else if (!speedFilterValid_) {
            smoothedSpeedMps_ = deadbanded;
            speedFilterValid_ = true;
        } else {
            smoothedSpeedMps_ += config_.speedFilterAlpha *
                                 (deadbanded - smoothedSpeedMps_);
        }
        // Apply the stationary deadband after smoothing too. Otherwise the EMA
        // itself emits small non-zero speeds while climbing toward a spike.
        filteredSpeedMps_ = smoothedSpeedMps_ < config_.speedDeadbandMps
                                ? 0.0f : smoothedSpeedMps_;
        lastFixUs_        = reading.timestampUs;
        lastGoodSpeedMps_ = filteredSpeedMps_;
        fixCount_++;
        if (reading.unixTime != 0) {
            lastUnixTime_ = reading.unixTime;
        }
        if (!everHadFix_) {
            everHadFix_ = true;
            APEX_LOGI("GNSS first fix: %u satellites, HDOP %.1f",
                    static_cast<unsigned>(reading.satellites), static_cast<double>(reading.hdop));
        } else if (!hadFix) {
            APEX_LOGI("GNSS fix reacquired");
        }
    } else if (hadFix) {
        APEX_LOGW("GNSS fix lost");
        speedFilterValid_ = false;
    }

    updateAcceleration(reading);

    lastReading_ = reading;
    out          = reading;
    return true;
}

void GpsManager::updateAcceleration(const GnssReading& reading) {
    if (!reading.hasFix() || reading.hdop > config_.maxUsableHdop) {
        // Differentiating across a gap in the fix would turn the reacquisition
        // step into a phantom acceleration spike.
        hasPreviousSpeed_ = false;
        accelValid_       = false;
        return;
    }

    if (hasPreviousSpeed_ && reading.timestampUs > previousSpeedUs_) {
        const float dt = static_cast<float>(reading.timestampUs - previousSpeedUs_) * 1e-6f;

        if (dt >= 0.02f && dt <= 2.0f) {
            const float raw = (filteredSpeedMps_ - previousSpeedMps_) / dt;

            if (fabsf(raw) <= config_.maxPlausibleAccelMps2) {
                if (!accelValid_) {
                    filteredAccelMps2_ = raw;
                    accelValid_        = true;
                } else {
                    filteredAccelMps2_ += config_.accelFilterAlpha * (raw - filteredAccelMps2_);
                }
            }
        }
    }

    previousSpeedMps_ = filteredSpeedMps_;
    previousSpeedUs_  = reading.timestampUs;
    hasPreviousSpeed_ = true;
}

bool GpsManager::likelyMoving() const {
    if (!everHadFix_) {
        return false;
    }

    const uint64_t now = clock_.micros();
    if (now < lastFixUs_) {
        return false;
    }
    if ((now - lastFixUs_) / 1000ull > config_.movementMemoryMs) {
        return false;
    }

    return lastGoodSpeedMps_ >= config_.movingSpeedMps;
}

bool GpsManager::accelerationHint(float& accelMps2Out) const {
    float unusedSpeed = 0.0f;
    if (!accelValid_ || !speedHint(unusedSpeed)) {
        return false;
    }

    accelMps2Out = filteredAccelMps2_;
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

    speedMpsOut = filteredSpeedMps_;
    return true;
}

}  // namespace apex
