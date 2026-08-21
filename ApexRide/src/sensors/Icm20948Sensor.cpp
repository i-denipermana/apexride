#include "Icm20948Sensor.h"

#include <Arduino.h>
#include <driver/i2c.h>
#include <esp_intr_alloc.h>
#include <esp_timer.h>

#include "../core/Log.h"

namespace apex {
namespace {

constexpr uint8_t kRegisterBankSelect = 0x7F;

// User bank 0.
constexpr uint8_t kWhoAmI       = 0x00;
constexpr uint8_t kPowerMgmt1   = 0x06;
constexpr uint8_t kPowerMgmt2   = 0x07;
constexpr uint8_t kAccelXoutHigh = 0x2D;

constexpr uint8_t kExpectedWhoAmI = 0xEA;

// The device is reset during begin(), so these power-on ranges are known.
constexpr float kAccelCountsPerG   = 16384.0f;  // +/- 2 g
constexpr float kGyroCountsPerDps  = 131.0f;    // +/- 250 dps

constexpr i2c_port_t kI2cPort = I2C_NUM_0;
constexpr TickType_t kTransactionTimeout = pdMS_TO_TICKS(20);

uint64_t monotonicMicros() {
    return static_cast<uint64_t>(esp_timer_get_time());
}

void clearPhysicalBus(int sdaPin, int sclPin) {
    // Release SDA and clock a slave out of a half-finished byte left behind by
    // an ESP32 reset. Finish with an explicit STOP before installing the
    // peripheral driver.
    pinMode(sdaPin, OUTPUT_OPEN_DRAIN);
    pinMode(sclPin, OUTPUT_OPEN_DRAIN);
    digitalWrite(sdaPin, HIGH);
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(10);

    for (uint8_t pulse = 0; pulse < 9 && digitalRead(sdaPin) == LOW; ++pulse) {
        digitalWrite(sclPin, LOW);
        delayMicroseconds(10);
        digitalWrite(sclPin, HIGH);
        delayMicroseconds(10);
    }

    digitalWrite(sdaPin, LOW);
    delayMicroseconds(10);
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sdaPin, HIGH);
    delayMicroseconds(10);
}

}  // namespace

bool Icm20948Sensor::begin() {
    if (config_.sampleRateHz <= 0.0f || config_.transactionRetries == 0) {
        APEX_LOGE("ICM-20948 has invalid configuration");
        return false;
    }

    if (!startBus()) {
        APEX_LOGE("ICM-20948 I2C bus start failed");
        return false;
    }
    delay(250);

    constexpr uint8_t kStartupAttempts = 3;
    const auto selectBankAtStartup = [&](uint8_t bank) {
        for (uint8_t attempt = 0; attempt < kStartupAttempts; ++attempt) {
            if (selectBank(bank)) return true;
            delay(20);
        }
        return false;
    };
    const auto writeAtStartup = [&](uint8_t reg, uint8_t value) {
        for (uint8_t attempt = 0; attempt < kStartupAttempts; ++attempt) {
            if (writeRegister(reg, value)) return true;
            delay(20);
        }
        return false;
    };

    if (!selectBankAtStartup(0)) {
        APEX_LOGE("ICM-20948 bank 0 select failed");
        return false;
    }

    uint8_t identity = 0;
    bool identityRead = false;
    for (uint8_t attempt = 0; attempt < kStartupAttempts && !identityRead; ++attempt) {
        identityRead = readBytes(kWhoAmI, &identity, 1) && identity == kExpectedWhoAmI;
        if (!identityRead) delay(20);
    }
    if (!identityRead) {
        APEX_LOGE("ICM-20948 WHO_AM_I was 0x%02x (expected 0xea)", identity);
        return false;
    }

    // Reset the sensor so the conversion factors below always match its
    // configured ranges, even when only the ESP32 was rebooted.
    if (!writeAtStartup(kPowerMgmt1, 0x80)) {
        APEX_LOGE("ICM-20948 reset failed");
        return false;
    }
    delay(100);

    if (!selectBankAtStartup(0) || !writeAtStartup(kPowerMgmt1, 0x01) ||
        !writeAtStartup(kPowerMgmt2, 0x00)) {
        APEX_LOGE("ICM-20948 wake failed");
        return false;
    }
    delay(250);

    // The burst starts at ACCEL_XOUT_H and continues through GYRO_ZOUT_L.
    // Keep bank 0 selected for every later sample.
    if (!selectBankAtStartup(0)) {
        APEX_LOGE("ICM-20948 final bank select failed");
        return false;
    }

    intervalUs_   = static_cast<uint64_t>(1000000.0f / config_.sampleRateHz);
    nextSampleUs_ = monotonicMicros();
    readErrors_   = 0;

    APEX_LOGI("ICM-20948 confirmed at 0x%02x on SDA %d / SCL %d (%u Hz I2C)",
              config_.address, config_.sdaPin, config_.sclPin,
              static_cast<unsigned>(config_.i2cFrequency));
    return true;
}

bool Icm20948Sensor::startBus() {
    // Arduino-ESP32 3.3.x uses ESP-IDF's newer I2C master implementation. On
    // ESP32-S3 that implementation can remain in ESP_ERR_INVALID_STATE after a
    // NACK (IDFGH-13084). The legacy controller remains supported by the SDK
    // and is deterministic for this single-device telemetry bus.
    clearPhysicalBus(config_.sdaPin, config_.sclPin);

    i2c_config_t busConfig{};
    busConfig.mode             = I2C_MODE_MASTER;
    busConfig.sda_io_num       = static_cast<gpio_num_t>(config_.sdaPin);
    busConfig.scl_io_num       = static_cast<gpio_num_t>(config_.sclPin);
    busConfig.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    busConfig.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    busConfig.master.clk_speed = config_.i2cFrequency;
    busConfig.clk_flags        = 0;

    if (i2c_param_config(kI2cPort, &busConfig) != ESP_OK) {
        return false;
    }
    // Telemetry flushes to LittleFS while the ride is active. The SDK requires
    // an IRAM-safe I2C interrupt when flash writes can disable the cache. All
    // buffers passed by this driver are stack/static internal-RAM objects.
    return i2c_driver_install(kI2cPort, I2C_MODE_MASTER, 0, 0,
                              ESP_INTR_FLAG_IRAM) == ESP_OK;
}

bool Icm20948Sensor::recoverBus() {
    i2c_driver_delete(kI2cPort);
    delay(2);

    const bool restarted = startBus();
    delay(2);

    if (restarted) {
        ++busRecoveries_;
        if (busRecoveries_ == 1 || busRecoveries_ % 100 == 0) {
            APEX_LOGW("ICM-20948 I2C bus recovered (%u total)",
                      static_cast<unsigned>(busRecoveries_));
        }
    }
    return restarted;
}

bool Icm20948Sensor::read(RawImuSample& out) {
    const uint64_t now = monotonicMicros();
    if (now < nextSampleUs_) {
        return false;
    }

    uint8_t data[12];
    if (!readBytes(kAccelXoutHigh, data, sizeof(data))) {
        ++readErrors_;
        // A failed transaction can consume several retry delays. Schedule from
        // the end of that work so TelemetrySystem's poll loop cannot hammer the
        // bus repeatedly when a jumper is loose.
        nextSampleUs_ = monotonicMicros() + intervalUs_;

        if (readErrors_ == 1 || readErrors_ % 100 == 0) {
            APEX_LOGW("ICM-20948 burst read failed (%u total)",
                      static_cast<unsigned>(readErrors_));
        }
        return false;
    }

    const int16_t ax = decodeInt16(data[0], data[1]);
    const int16_t ay = decodeInt16(data[2], data[3]);
    const int16_t az = decodeInt16(data[4], data[5]);
    const int16_t gx = decodeInt16(data[6], data[7]);
    const int16_t gy = decodeInt16(data[8], data[9]);
    const int16_t gz = decodeInt16(data[10], data[11]);

    const float accelScale = kGravityMps2 / kAccelCountsPerG;
    const float gyroScale  = kDegToRad / kGyroCountsPerDps;

    out.timestampUs = monotonicMicros();
    out.accel       = Vec3(ax * accelScale, ay * accelScale, az * accelScale);
    out.gyro        = Vec3(gx * gyroScale, gy * gyroScale, gz * gyroScale);
    out.mag         = Vec3();
    out.magValid    = false;

    nextSampleUs_ += intervalUs_;
    if (nextSampleUs_ + intervalUs_ < out.timestampUs) {
        // Drop missed samples instead of producing a burst of duplicates.
        nextSampleUs_ = out.timestampUs + intervalUs_;
    }

    return true;
}

bool Icm20948Sensor::writeRegister(uint8_t reg, uint8_t value) {
    const uint8_t payload[] = {reg, value};
    for (uint8_t attempt = 0; attempt < config_.transactionRetries; ++attempt) {
        if (i2c_master_write_to_device(kI2cPort, config_.address, payload,
                                       sizeof(payload), kTransactionTimeout) == ESP_OK) {
            return true;
        }
        delay(2);
    }
    recoverBus();
    return false;
}

bool Icm20948Sensor::readBytes(uint8_t reg, uint8_t* buffer, uint8_t length) {
    for (uint8_t attempt = 0; attempt < config_.transactionRetries; ++attempt) {
        // This breakout was validated with a STOP after selecting the register
        // and a short settling delay before the read transaction.
        if (i2c_master_write_to_device(kI2cPort, config_.address, &reg, 1,
                                       kTransactionTimeout) == ESP_OK) {
            delayMicroseconds(config_.registerSettleUs);
            if (i2c_master_read_from_device(kI2cPort, config_.address, buffer,
                                            length, kTransactionTimeout) == ESP_OK) {
                return true;
            }
        }
        delayMicroseconds(250);
    }
    recoverBus();
    return false;
}

bool Icm20948Sensor::selectBank(uint8_t bank) {
    if (bank > 3) {
        return false;
    }
    return writeRegister(kRegisterBankSelect, static_cast<uint8_t>(bank << 4));
}

int16_t Icm20948Sensor::decodeInt16(uint8_t high, uint8_t low) {
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

}  // namespace apex
