#include "SummaryBuilder.h"

#include <math.h>
#include <string.h>

#include "../core/Crc32.h"
#include "../core/MathUtils.h"
#include "../core/Types.h"

namespace apex {
namespace {

constexpr float kMetresPerDegreeLat = 110540.0f;
constexpr float kMetresPerDegreeLon = 111320.0f;

/// Low-pass coefficient for the longitudinal acceleration peak detector.
/// At a 50 Hz log rate this gives roughly a 3 Hz corner.
constexpr float kAccelFilterAlpha = 0.25f;

/// Ignore GNSS jitter while stationary: fixes closer together than this do not
/// contribute to trip distance.
constexpr float kMinDistanceStepM = 0.75f;

/// Position noise makes a parked bike wander. Distance only accumulates while
/// the receiver also reports real movement.
constexpr uint16_t kMinDistanceSpeedCmS = 60;

/// Equirectangular approximation. Accurate to well under a metre over the
/// sub-100 m steps between consecutive fixes, and avoids double-precision
/// trigonometry on a CPU with only a single-precision FPU.
float approximateDistanceM(double lat1, double lon1, double lat2, double lon2) {
    const float meanLatRad = static_cast<float>((lat1 + lat2) * 0.5) * kDegToRad;
    const float dNorth     = static_cast<float>(lat2 - lat1) * kMetresPerDegreeLat;
    const float dEast = static_cast<float>(lon2 - lon1) * kMetresPerDegreeLon * cosf(meanLatRad);
    return sqrtf(dNorth * dNorth + dEast * dEast);
}

uint16_t maxU16(uint16_t a, uint16_t b) {
    return a > b ? a : b;
}

}  // namespace

void SummaryBuilder::reset(uint32_t rideId, uint32_t startUnixTime, uint32_t startMillis,
                           uint16_t calibrationVersion) {
    memset(&summary_, 0, sizeof(summary_));

    summary_.magic              = kSummaryMagic;
    summary_.version            = kFormatVersion;
    summary_.rideId             = rideId;
    summary_.startUnixTime      = startUnixTime;
    summary_.firmwareVersion    = kFirmwareVersion;
    summary_.calibrationVersion = calibrationVersion;

    startMillis_    = startMillis;
    hasPreviousFix_ = false;
    previousLat_    = 0.0;
    previousLon_    = 0.0;
    hasAccelFilter_ = false;
    filteredAccelG_ = 0.0f;
}

void SummaryBuilder::setStartUnixTime(uint32_t unixTime) {
    if (summary_.startUnixTime == 0) {
        summary_.startUnixTime = unixTime;
    }
}

void SummaryBuilder::noteTimestamp(uint32_t timestampMs) {
    // Unsigned subtraction handles a millis() wrap correctly for any ride
    // shorter than 49 days.
    const uint32_t elapsed = timestampMs - startMillis_;
    if (elapsed > summary_.durationMs) {
        summary_.durationMs = elapsed;
    }
}

void SummaryBuilder::addImu(const ImuRecord& record) {
    noteTimestamp(record.timestampMs);
    summary_.imuSampleCount++;

    // Lean extremes, tracked as magnitudes per side.
    if (record.roll < 0) {
        summary_.maxLeanLeft = maxU16(summary_.maxLeanLeft, static_cast<uint16_t>(-record.roll));
    } else {
        summary_.maxLeanRight = maxU16(summary_.maxLeanRight, static_cast<uint16_t>(record.roll));
    }

    if (record.pitch > summary_.maxPitchUp) {
        summary_.maxPitchUp = record.pitch;
    }
    if (record.pitch < 0 && -record.pitch > summary_.maxPitchDown) {
        summary_.maxPitchDown = static_cast<int16_t>(-record.pitch);
    }

    // Remove the gravity component that pitch puts on the body X axis, so a
    // wheelie or a steep hill is not mistaken for acceleration. In g units,
    // with pitch positive nose-up:
    //   accelX_measured = a_long * cos(pitch) - sin(pitch)
    const float pitchRad      = fromCentiDegrees(record.pitch) * kDegToRad;
    const float cosPitch      = cosf(pitchRad);
    float       longitudinalG = fromMilliG(record.accelX);
    if (fabsf(cosPitch) > 0.1f) {
        longitudinalG = (longitudinalG - sinf(pitchRad)) / cosPitch;
    }

    if (!hasAccelFilter_) {
        filteredAccelG_ = longitudinalG;
        hasAccelFilter_ = true;
    } else {
        filteredAccelG_ += kAccelFilterAlpha * (longitudinalG - filteredAccelG_);
    }

    const float milliG = filteredAccelG_ * 1000.0f;
    if (milliG > 0.0f) {
        summary_.maxAcceleration =
            maxU16(summary_.maxAcceleration, static_cast<uint16_t>(milliG > 65535.0f ? 65535.0f : milliG));
    } else {
        const float magnitude = -milliG;
        summary_.maxBraking = maxU16(
            summary_.maxBraking, static_cast<uint16_t>(magnitude > 65535.0f ? 65535.0f : magnitude));
    }
}

void SummaryBuilder::addGnss(const GnssRecord& record) {
    noteTimestamp(record.timestampMs);
    summary_.gnssSampleCount++;

    if (record.unixTime != 0) {
        setStartUnixTime(record.unixTime);
        summary_.endUnixTime = record.unixTime;
    }

    if (record.fixType == static_cast<uint8_t>(GnssFix::None)) {
        // A lost fix breaks the distance chain; do not interpolate across it.
        hasPreviousFix_ = false;
        return;
    }

    summary_.maxSpeed = maxU16(summary_.maxSpeed, record.speed);

    const double lat = fromDegreesE7(record.latitude);
    const double lon = fromDegreesE7(record.longitude);

    if (record.speed < kMinDistanceSpeedCmS) {
        // Keep the anchor current so the resumption of movement is measured
        // from where the bike actually is.
        previousLat_    = lat;
        previousLon_    = lon;
        hasPreviousFix_ = true;
        return;
    }

    if (hasPreviousFix_) {
        const float step = approximateDistanceM(previousLat_, previousLon_, lat, lon);
        if (step >= kMinDistanceStepM) {
            summary_.distanceCm += static_cast<uint32_t>(step * 100.0f);
            previousLat_ = lat;
            previousLon_ = lon;
        }
        // Below the threshold the anchor point is deliberately left in place so
        // slow movement still accumulates instead of being discarded.
    } else {
        previousLat_    = lat;
        previousLon_    = lon;
        hasPreviousFix_ = true;
    }
}

void SummaryBuilder::addEvent(const EventRecord& record) {
    noteTimestamp(record.timestampMs);
    summary_.eventCount++;
}

RideSummary SummaryBuilder::build() const {
    RideSummary out = summary_;
    finalizeSummaryCrc(out);
    return out;
}

void finalizeSummaryCrc(RideSummary& summary) {
    summary.summaryCrc = crc32(&summary, sizeof(RideSummary) - sizeof(uint32_t));
}

bool validateSummary(const RideSummary& summary) {
    if (summary.magic != kSummaryMagic) return false;
    if (summary.version != kFormatVersion) return false;
    return summary.summaryCrc == crc32(&summary, sizeof(RideSummary) - sizeof(uint32_t));
}

}  // namespace apex
