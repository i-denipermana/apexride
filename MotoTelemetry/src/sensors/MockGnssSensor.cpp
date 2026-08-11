#include "MockGnssSensor.h"

namespace moto {
namespace {

constexpr double kMetresPerDegreeLat = 110540.0;

}  // namespace

MockGnssSensor::MockGnssSensor(const Clock& clock, RideSimulator& simulator)
    : clock_(clock), simulator_(simulator) {}

bool MockGnssSensor::begin() {
    intervalUs_   = static_cast<uint64_t>(1e6f / config_.updateRateHz);
    startUs_      = clock_.micros();
    nextSampleUs_ = startUs_ + intervalUs_;
    return true;
}

bool MockGnssSensor::read(GnssReading& out) {
    const uint64_t now = clock_.micros();
    if (now < nextSampleUs_) {
        return false;
    }

    const uint64_t sampleUs = nextSampleUs_;
    nextSampleUs_ += intervalUs_;
    if (nextSampleUs_ + intervalUs_ < now) {
        nextSampleUs_ = now;
    }

    simulator_.advanceTo(static_cast<uint32_t>(sampleUs / 1000u));

    const uint32_t elapsedMs = static_cast<uint32_t>((sampleUs - startUs_) / 1000u);

    out              = GnssReading();
    out.timestampUs  = sampleUs;

    const bool inDropout = config_.dropoutEndMs > config_.dropoutStartMs &&
                           elapsedMs >= config_.dropoutStartMs &&
                           elapsedMs < config_.dropoutEndMs;

    if (elapsedMs < config_.timeToFirstFixMs || inDropout) {
        out.fix        = GnssFix::None;
        out.satellites = inDropout ? 3 : 0;
        out.hdop       = 99.9f;
        return true;  // a real receiver still emits sentences with no fix
    }

    // Position noise, converted from metres to degrees at this latitude.
    const double latNoiseDeg =
        simulator_.noise(config_.hdopWithFix * 2.0f) / kMetresPerDegreeLat;
    const double lonNoiseDeg =
        simulator_.noise(config_.hdopWithFix * 2.0f) / kMetresPerDegreeLat;

    out.fix        = GnssFix::Fix3D;
    out.satellites = config_.satellitesWithFix;
    out.hdop       = config_.hdopWithFix;
    out.unixTime   = simulator_.unixTime();
    out.latitude   = simulator_.trueLatitude() + latNoiseDeg;
    out.longitude  = simulator_.trueLongitude() + lonNoiseDeg;
    out.altitudeM  = simulator_.altitudeM() + simulator_.noise(2.0f);
    out.courseDeg  = simulator_.trueHeadingDeg();

    out.speedMps = simulator_.trueSpeedMps() + simulator_.noise(0.3f);
    if (out.speedMps < 0.0f) {
        out.speedMps = 0.0f;
    }

    return true;
}

}  // namespace moto
