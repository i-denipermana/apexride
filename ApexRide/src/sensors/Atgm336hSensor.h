#pragma once
//
// ATGM336H UART/NMEA receiver for ApexRide.
//
// The module normally emits RMC and GGA sentences. RMC supplies UTC,
// position, speed and course; GGA supplies fix quality, satellites, HDOP and
// altitude. The parser accepts any talker prefix (GP, GN, BD, ...), validates
// every checksum, and never blocks the telemetry loop.
//

#include <Arduino.h>

#include "../core/Clock.h"
#include "IGnssSensor.h"

namespace apex {

class Atgm336hSensor : public IGnssSensor {
public:
    struct Config {
        int      rxPin = 18;
        int      txPin = 17;
        uint32_t baud  = 9600;

        // The factory NMEA output is normally 1 Hz. Increase this only after
        // sending and validating the module-specific rate command.
        float updateRateHz = 1.0f;

        uint32_t noDataWarningMs = 3000;
        uint32_t ggaMaxAgeMs     = 2500;
    };

    Atgm336hSensor(HardwareSerial& serial, const Clock& clock);

    void setConfig(const Config& config) { config_ = config; }

    bool begin() override;
    bool read(GnssReading& out) override;

    float updateRateHz() const override { return config_.updateRateHz; }
    uint32_t packetCount() const override { return validSentenceCount_; }
    uint32_t parseErrorCount() const override { return checksumErrorCount_ + overflowCount_; }
    const char* name() const override { return "ATGM336H"; }

    uint32_t byteCount() const { return byteCount_; }
    uint32_t validSentenceCount() const { return validSentenceCount_; }
    uint32_t checksumErrorCount() const { return checksumErrorCount_; }

private:
    static constexpr size_t kSentenceCapacity = 128;
    static constexpr size_t kMaxFields        = 20;

    bool consume(char character, GnssReading& out);
    bool parseSentence(char* sentence, GnssReading& out);
    bool parseRmc(char** fields, size_t count, GnssReading& out);
    void parseGga(char** fields, size_t count);

    static bool validateChecksum(const char* sentence);
    static size_t splitFields(char* sentence, char** fields, size_t capacity);
    static bool sentenceIs(const char* type, const char* suffix);
    static bool parseCoordinate(const char* value, const char* hemisphere,
                                double& degreesOut);
    static bool parseFloat(const char* value, float& out);
    static bool parseUnsigned(const char* value, uint32_t& out);
    static uint32_t parseUnixTime(const char* utc, const char* date);

    HardwareSerial& serial_;
    const Clock&    clock_;
    Config          config_{};

    char   sentence_[kSentenceCapacity]{};
    size_t sentenceLength_ = 0;
    bool   collecting_     = false;

    uint64_t startUs_          = 0;
    uint64_t lastGgaUs_        = 0;
    float    lastAltitudeM_    = 0.0f;
    float    lastHdop_         = 99.9f;
    uint8_t  lastSatellites_   = 0;
    uint8_t  lastFixQuality_   = 0;

    uint32_t byteCount_          = 0;
    uint32_t validSentenceCount_ = 0;
    uint32_t checksumErrorCount_ = 0;
    uint32_t overflowCount_      = 0;
    bool     streamLogged_       = false;
    bool     noDataWarned_       = false;
};

}  // namespace apex
