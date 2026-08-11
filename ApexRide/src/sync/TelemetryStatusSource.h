#pragma once
//
// Adapts the running telemetry system into the status the sync layer needs.
//
// This is the one place ride recording and ride transfer meet. It exists as an
// adapter rather than by making TelemetrySystem implement IDeviceStatusSource
// so the dependency points one way only: sync knows about ride, ride knows
// nothing about sync.
//
// It is also what makes "refuse to sync while recording" real. Without it the
// sync layer has no idea a ride is in progress, and would happily serve bulk
// data while the recorder is trying to write to the same filesystem.
//

#include "../ride/TelemetrySystem.h"
#include "SyncService.h"

namespace apex {

class TelemetryStatusSource : public IDeviceStatusSource {
public:
    explicit TelemetryStatusSource(const TelemetrySystem& system) : system_(system) {}

    void fillStatus(DeviceStatus& out) const override;

private:
    const TelemetrySystem& system_;
};

}  // namespace apex
