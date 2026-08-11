#pragma once
//
// GNSS stand-in driven by the same RideSimulator as MockImuSensor.
//
// Emulates the parts of a real receiver that the pipeline has to survive:
//   - a cold-start delay before the first fix
//   - position and speed noise
//   - a scripted fix dropout (tunnel / underpass), so the recovery path and
//     the distance calculation are actually exercised
//

#include "../core/Clock.h"
#include "../sim/RideSimulator.h"
#include "IGnssSensor.h"

namespace moto {

class MockGnssSensor : public IGnssSensor {
public:
    struct Config {
        float updateRateHz = 5.0f;

        /// Time from power-on to first fix.
        uint32_t timeToFirstFixMs = 6000;

        /// Simulated dropout window, relative to begin(). Set both to zero to
        /// disable.
        uint32_t dropoutStartMs = 62000;
        uint32_t dropoutEndMs   = 70000;

        uint8_t satellitesWithFix = 11;
        float   hdopWithFix       = 0.9f;
    };

    MockGnssSensor(const Clock& clock, RideSimulator& simulator);

    void setConfig(const Config& config) { config_ = config; }

    bool begin() override;
    bool read(GnssReading& out) override;

    float updateRateHz() const override { return config_.updateRateHz; }

    const char* name() const override { return "MockGNSS"; }

private:
    const Clock&   clock_;
    RideSimulator& simulator_;
    Config         config_{};

    uint64_t startUs_      = 0;
    uint64_t nextSampleUs_ = 0;
    uint64_t intervalUs_   = 200000;
};

}  // namespace moto
