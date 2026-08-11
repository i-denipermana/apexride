#pragma once
//
// IMU stand-in driven by RideSimulator, so the whole pipeline can be exercised
// before the ICM-20948 arrives.
//
// It deliberately emulates the awkward parts of a real breakout board:
//   - a mounting orientation that is not the body frame (axis swap plus tilt)
//   - a constant gyroscope bias
//   - vibration noise
//   - sample timing that drifts slightly off the nominal rate
//
// If the pipeline copes with this, swapping in the real driver should be a
// small step rather than a rewrite.
//

#include "../core/Clock.h"
#include "../sim/RideSimulator.h"
#include "IImuSensor.h"

namespace moto {

class MockImuSensor : public IImuSensor {
public:
    struct Config {
        float sampleRateHz = 200.0f;

        /// Simulated mounting: the breakout is rotated 90 degrees about the
        /// vertical axis and tilted 2.7 degrees, matching the handoff's
        /// calibration example. ImuManager's axis map undoes the rotation;
        /// the residual tilt is what mounting calibration must remove.
        bool  applyMountingError  = true;
        float mountingTiltDeg     = 2.7f;
    };

    MockImuSensor(const Clock& clock, RideSimulator& simulator);

    void setConfig(const Config& config) { config_ = config; }

    bool begin() override;
    bool read(RawImuSample& out) override;

    float sampleRateHz() const override { return config_.sampleRateHz; }

    const char* name() const override { return "MockIMU"; }

private:
    const Clock&   clock_;
    RideSimulator& simulator_;
    Config         config_{};

    uint64_t nextSampleUs_ = 0;
    uint64_t intervalUs_   = 5000;
};

}  // namespace moto
