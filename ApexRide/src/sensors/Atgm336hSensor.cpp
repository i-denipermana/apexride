#include "Atgm336hSensor.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "../core/Log.h"

namespace apex {
namespace {

int hexValue(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// Days since 1970-01-01. This is Howard Hinnant's civil-calendar transform,
// kept local so GNSS time never depends on the ESP32's timezone configuration.
int64_t daysFromCivil(int year, unsigned month, unsigned day) {
    year -= month <= 2;
    const int      era = (year >= 0 ? year : year - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(year - era * 400);
    const unsigned doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return static_cast<int64_t>(era) * 146097 + static_cast<int64_t>(doe) - 719468;
}

}  // namespace

Atgm336hSensor::Atgm336hSensor(HardwareSerial& serial, const Clock& clock)
    : serial_(serial), clock_(clock) {}

bool Atgm336hSensor::begin() {
    if (config_.baud == 0 || config_.updateRateHz <= 0.0f || config_.rxPin < 0) {
        APEX_LOGE("ATGM336H has invalid UART configuration");
        return false;
    }

    serial_.setRxBufferSize(2048);
    serial_.begin(config_.baud, SERIAL_8N1, config_.rxPin, config_.txPin);

    sentenceLength_ = 0;
    collecting_     = false;
    startUs_        = clock_.micros();
    lastGgaUs_      = 0;
    byteCount_      = 0;
    validSentenceCount_ = 0;
    checksumErrorCount_ = 0;
    overflowCount_      = 0;
    streamLogged_       = false;
    noDataWarned_       = false;

    while (serial_.available()) {
        serial_.read();
    }

    APEX_LOGI("ATGM336H UART listening on RX %d / TX %d at %u baud",
              config_.rxPin, config_.txPin,
              static_cast<unsigned>(config_.baud));
    return true;
}

bool Atgm336hSensor::read(GnssReading& out) {
    while (serial_.available() > 0) {
        const int value = serial_.read();
        if (value < 0) break;

        ++byteCount_;
        if (consume(static_cast<char>(value), out)) {
            return true;
        }
    }

    if (!noDataWarned_) {
        const uint64_t elapsedMs = (clock_.micros() - startUs_) / 1000ull;
        if (elapsedMs >= config_.noDataWarningMs) {
            if (byteCount_ == 0) {
                APEX_LOGW("ATGM336H UART has no data — check TX->GPIO%d, power and ground",
                          config_.rxPin);
            } else if (validSentenceCount_ == 0) {
                APEX_LOGW("ATGM336H received %u bytes but no valid NMEA checksums",
                          static_cast<unsigned>(byteCount_));
            }
            noDataWarned_ = true;
        }
    }

    return false;
}

bool Atgm336hSensor::consume(char character, GnssReading& out) {
    if (character == '$') {
        collecting_     = true;
        sentenceLength_ = 0;
        sentence_[sentenceLength_++] = character;
        return false;
    }

    if (!collecting_) return false;

    if (character == '\r') return false;

    if (character == '\n') {
        collecting_ = false;
        sentence_[sentenceLength_] = '\0';

        if (!validateChecksum(sentence_)) {
            ++checksumErrorCount_;
            return false;
        }

        ++validSentenceCount_;
        if (!streamLogged_) {
            streamLogged_ = true;
            APEX_LOGI("ATGM336H NMEA stream detected");
        }
        return parseSentence(sentence_, out);
    }

    if (sentenceLength_ + 1 >= sizeof(sentence_)) {
        collecting_     = false;
        sentenceLength_ = 0;
        ++overflowCount_;
        if (overflowCount_ == 1) {
            APEX_LOGW("ATGM336H NMEA sentence exceeded %u bytes",
                      static_cast<unsigned>(sizeof(sentence_) - 1));
        }
        return false;
    }

    sentence_[sentenceLength_++] = character;
    return false;
}

bool Atgm336hSensor::parseSentence(char* sentence, GnssReading& out) {
    char*  fields[kMaxFields]{};
    size_t count = splitFields(sentence, fields, kMaxFields);
    if (count == 0 || fields[0][0] != '$') return false;

    const char* type = fields[0] + 1;
    if (sentenceIs(type, "GGA")) {
        parseGga(fields, count);
        return false;
    }
    if (sentenceIs(type, "RMC")) {
        return parseRmc(fields, count, out);
    }
    return false;
}

bool Atgm336hSensor::parseRmc(char** fields, size_t count, GnssReading& out) {
    if (count < 10) return false;

    GnssReading reading;
    reading.timestampUs = clock_.micros();

    const bool active = fields[2][0] == 'A';
    double latitude = 0.0;
    double longitude = 0.0;
    const bool positionValid = parseCoordinate(fields[3], fields[4], latitude) &&
                               parseCoordinate(fields[5], fields[6], longitude);

    float speedKnots = 0.0f;
    float courseDeg  = 0.0f;
    parseFloat(fields[7], speedKnots);
    parseFloat(fields[8], courseDeg);

    reading.unixTime  = parseUnixTime(fields[1], fields[9]);
    reading.latitude  = positionValid ? latitude : 0.0;
    reading.longitude = positionValid ? longitude : 0.0;
    reading.speedMps  = speedKnots > 0.0f ? speedKnots * 0.514444f : 0.0f;
    reading.courseDeg = fmodf(courseDeg + 360.0f, 360.0f);

    const uint64_t ggaAgeMs = lastGgaUs_ == 0 || reading.timestampUs < lastGgaUs_
                                  ? UINT64_MAX
                                  : (reading.timestampUs - lastGgaUs_) / 1000ull;
    if (ggaAgeMs <= config_.ggaMaxAgeMs) {
        reading.altitudeM  = lastAltitudeM_;
        reading.hdop       = lastHdop_;
        reading.satellites = lastSatellites_;
    }

    if (active && positionValid) {
        reading.fix = (ggaAgeMs <= config_.ggaMaxAgeMs && lastFixQuality_ > 0)
                          ? GnssFix::Fix3D
                          : GnssFix::Fix2D;
    }

    out = reading;
    return true;
}

void Atgm336hSensor::parseGga(char** fields, size_t count) {
    if (count < 10) return;

    uint32_t fixQuality = 0;
    uint32_t satellites = 0;
    float    hdop       = 99.9f;
    float    altitude   = 0.0f;

    parseUnsigned(fields[6], fixQuality);
    parseUnsigned(fields[7], satellites);
    parseFloat(fields[8], hdop);
    parseFloat(fields[9], altitude);

    lastFixQuality_ = static_cast<uint8_t>(fixQuality > 255 ? 255 : fixQuality);
    lastSatellites_ = static_cast<uint8_t>(satellites > 255 ? 255 : satellites);
    lastHdop_       = hdop > 0.0f ? hdop : 99.9f;
    lastAltitudeM_  = altitude;
    lastGgaUs_      = clock_.micros();
}

bool Atgm336hSensor::validateChecksum(const char* sentence) {
    if (sentence == nullptr || sentence[0] != '$') return false;

    const char* star = strchr(sentence, '*');
    if (star == nullptr || star[1] == '\0' || star[2] == '\0') return false;

    uint8_t calculated = 0;
    for (const char* p = sentence + 1; p < star; ++p) {
        calculated ^= static_cast<uint8_t>(*p);
    }

    const int high = hexValue(star[1]);
    const int low  = hexValue(star[2]);
    return high >= 0 && low >= 0 && calculated == static_cast<uint8_t>((high << 4) | low);
}

size_t Atgm336hSensor::splitFields(char* sentence, char** fields, size_t capacity) {
    if (sentence == nullptr || capacity == 0) return 0;

    size_t count = 0;
    fields[count++] = sentence;
    for (char* p = sentence; *p != '\0'; ++p) {
        if (*p == ',' || *p == '*') {
            *p = '\0';
            if (count < capacity) fields[count++] = p + 1;
        }
    }
    return count;
}

bool Atgm336hSensor::sentenceIs(const char* type, const char* suffix) {
    if (type == nullptr || suffix == nullptr) return false;
    const size_t typeLength = strlen(type);
    const size_t suffixLength = strlen(suffix);
    return typeLength >= suffixLength &&
           strcmp(type + typeLength - suffixLength, suffix) == 0;
}

bool Atgm336hSensor::parseCoordinate(const char* value, const char* hemisphere,
                                     double& degreesOut) {
    if (value == nullptr || value[0] == '\0' || hemisphere == nullptr ||
        hemisphere[0] == '\0') {
        return false;
    }

    char* end = nullptr;
    const double raw = strtod(value, &end);
    if (end == value || *end != '\0' || raw < 0.0) return false;

    const double degrees = floor(raw / 100.0);
    const double minutes = raw - degrees * 100.0;
    if (minutes < 0.0 || minutes >= 60.0) return false;

    double result = degrees + minutes / 60.0;
    if (hemisphere[0] == 'S' || hemisphere[0] == 'W') {
        result = -result;
    } else if (hemisphere[0] != 'N' && hemisphere[0] != 'E') {
        return false;
    }

    degreesOut = result;
    return true;
}

bool Atgm336hSensor::parseFloat(const char* value, float& out) {
    if (value == nullptr || value[0] == '\0') return false;
    char* end = nullptr;
    const float parsed = strtof(value, &end);
    if (end == value || *end != '\0' || !isfinite(parsed)) return false;
    out = parsed;
    return true;
}

bool Atgm336hSensor::parseUnsigned(const char* value, uint32_t& out) {
    if (value == nullptr || value[0] == '\0') return false;
    char* end = nullptr;
    const unsigned long parsed = strtoul(value, &end, 10);
    if (end == value || *end != '\0') return false;
    out = static_cast<uint32_t>(parsed);
    return true;
}

uint32_t Atgm336hSensor::parseUnixTime(const char* utc, const char* date) {
    if (utc == nullptr || date == nullptr || strlen(utc) < 6 || strlen(date) != 6) {
        return 0;
    }

    for (size_t i = 0; i < 6; ++i) {
        if (utc[i] < '0' || utc[i] > '9' || date[i] < '0' || date[i] > '9') return 0;
    }

    const int hour   = (utc[0] - '0') * 10 + (utc[1] - '0');
    const int minute = (utc[2] - '0') * 10 + (utc[3] - '0');
    const int second = (utc[4] - '0') * 10 + (utc[5] - '0');
    const int day    = (date[0] - '0') * 10 + (date[1] - '0');
    const int month  = (date[2] - '0') * 10 + (date[3] - '0');
    const int year2  = (date[4] - '0') * 10 + (date[5] - '0');
    const int year   = year2 >= 80 ? 1900 + year2 : 2000 + year2;

    if (hour > 23 || minute > 59 || second > 60 || day < 1 || day > 31 ||
        month < 1 || month > 12) {
        return 0;
    }

    const int64_t days = daysFromCivil(year, static_cast<unsigned>(month),
                                       static_cast<unsigned>(day));
    const int64_t seconds = days * 86400 + hour * 3600 + minute * 60 + second;
    return seconds > 0 && seconds <= UINT32_MAX ? static_cast<uint32_t>(seconds) : 0;
}

}  // namespace apex
