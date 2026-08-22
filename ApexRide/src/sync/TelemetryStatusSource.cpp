#include "TelemetryStatusSource.h"

namespace apex {

void TelemetryStatusSource::fillStatus(DeviceStatus& out) const {
    const TelemetrySystem::Status status = system_.status();

    // A ride is in progress whenever the recorder holds an open file — not
    // merely when the state machine says RECORDING. The two differ during
    // WAITING, where the bike has stopped but the ride file is deliberately
    // still open in case the rider sets off again.
    out.recording    = status.activeRideId != 0;
    out.activeRideId = status.activeRideId;

    out.gnssFix    = status.gnssFix;
    out.satellites = status.satellites;
    out.latitude   = status.latitude;
    out.longitude  = status.longitude;

    out.rideState        = toString(status.state);
    out.calibrationState = status.calibrationState;
    out.leanDeg          = status.fused.rollDeg();
    out.pitchDeg         = status.fused.pitchDeg();
    out.speedKph         = status.speedMps * 3.6f;
    out.rawSpeedKph      = status.rawSpeedMps * 3.6f;
    out.imuSamples       = status.imuSamples;
    out.imuErrors        = status.imuErrors;
    out.droppedSamples   = status.droppedSamples;
    out.gnssErrors       = status.gnssErrors;

    // No divider on the V1 hardware, so there is nothing honest to report.
    // See the bill-of-materials note in README.md.
    out.batteryAvailable = false;
    out.batteryPercent   = 0;
}

}  // namespace apex
