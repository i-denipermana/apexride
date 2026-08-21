#pragma once
//
// Minimal I2C driver for the ApexRide ICM-20948 breakout.
//
// The driver deliberately covers only the accelerometer and gyroscope path
// needed by V1. The magnetometer can be added independently later without
// changing IImuSensor or the fusion pipeline.
//

#include "IImuSensor.h"

namespace apex {

class Icm20948Sensor : public IImuSensor {
public:
    struct Config {
        int      sdaPin       = 8;
        int      sclPin       = 9;
        uint32_t i2cFrequency = 50000;
        uint8_t  address      = 0x68;

        float sampleRateHz = 200.0f;

        /// Retained for compatibility with boards that need a short pause
        /// between configuration writes. Burst reads use a repeated START.
        uint16_t registerSettleUs = 500;
        uint8_t  transactionRetries = 5;
    };

    Icm20948Sensor() = default;

    void setConfig(const Config& config) { config_ = config; }

    bool begin() override;
    bool read(RawImuSample& out) override;

    float sampleRateHz() const override { return config_.sampleRateHz; }
    const char* name() const override { return "ICM-20948"; }

    uint32_t readErrorCount() const override { return readErrors_; }
    uint32_t busRecoveryCount() const { return busRecoveries_; }

private:
    bool startBus();
    bool recoverBus();
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readBytes(uint8_t reg, uint8_t* buffer, uint8_t length);
    bool selectBank(uint8_t bank);

    static int16_t decodeInt16(uint8_t high, uint8_t low);

    Config   config_{};

    uint64_t nextSampleUs_ = 0;
    uint64_t intervalUs_   = 5000;
    uint32_t readErrors_   = 0;
    uint32_t busRecoveries_ = 0;
};

}  // namespace apex
