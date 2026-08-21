#include "TelemetrySystem.h"

#include <math.h>

#include "../core/Log.h"

namespace apex {

TelemetrySystem::TelemetrySystem(IImuSensor& imuSensor, IGnssSensor& gnssSensor, IRideStore& store,
                                 const Clock& clock)
    : clock_(clock),
      store_(store),
      imu_(imuSensor),
      gnss_(gnssSensor, clock),
      recorder_(storage_, clock) {}

bool TelemetrySystem::begin(const Config& config, uint8_t* recorderBuffer,
                            size_t recorderBufferSize) {
    config_ = config;

    fusionIntervalUs_ = config.fusionRateHz > 0.0f
                            ? static_cast<uint64_t>(1e6f / config.fusionRateHz)
                            : 10000;

    if (!storage_.begin(store_, config.storage)) {
        return false;
    }

    // Repair anything left behind by an unclean shutdown before touching the
    // filesystem further, so a rebuilt ride cannot be mistaken for free space.
    const uint32_t repaired = storage_.recoverIncompleteRides();
    if (repaired > 0) {
        APEX_LOGW("Recovered %u ride(s) after an unclean shutdown",
                static_cast<unsigned>(repaired));
    }

    if (!imu_.begin(config.imu)) {
        return false;
    }
    if (!gnss_.begin(config.gnss)) {
        return false;
    }
    if (!recorder_.begin(config.recorder, recorderBuffer, recorderBufferSize)) {
        return false;
    }

    orientation_.begin(config.orientation);
    rideManager_.begin(config.ride);

    gyroBiasRequested_ = config.autoCalibrateGyroBias;
    lastFusionUs_      = clock_.micros();

    return true;
}

void TelemetrySystem::calibrateGyroBias() {
    gyroBiasRequested_ = true;
    imu_.startGyroBiasCapture();
}

bool TelemetrySystem::calibrateMounting() {
    if (!imu_.isStationary()) {
        APEX_LOGW("Mounting calibration refused: the bike is moving");
        return false;
    }

    // Capture the filter's current estimate rather than re-seeding from a
    // single accelerometer sample: one sample carries a couple of degrees of
    // vibration noise, which would be baked into the offset permanently. The
    // filter has been averaging that noise away for as long as it has run.
    orientation_.captureMountingOffset();

    const Quaternion& offset = orientation_.mountingOffset();
    APEX_LOGI("Mounting offset captured: roll %.2f deg, pitch %.2f deg",
            static_cast<double>(offset.eulerX() * kRadToDeg),
            static_cast<double>(-offset.eulerY() * kRadToDeg));

    if (recorder_.isRecording()) {
        recorder_.recordEvent(EventCode::CalibrationApplied,
                              static_cast<int32_t>(imu_.calibration().version));
    }

    return true;
}

void TelemetrySystem::beginRide() {
    if (recorder_.isRecording()) {
        return;
    }

    if (!storage_.canStartRide() && !storage_.reclaimSpace(storage_.minFreeBytesToStart())) {
        if (!spaceWarningSent_) {
            APEX_LOGE("Cannot start a ride: storage is full of unsynced rides");
            spaceWarningSent_ = true;
        }
        return;
    }

    spaceWarningSent_ = false;
    recorder_.startRide(gnss_.lastUnixTime(), imu_.calibration().version);
}

void TelemetrySystem::finishRide() {
    if (recorder_.isRecording()) {
        recorder_.endRide();
    }
}

void TelemetrySystem::applyAction(RideManager::Action action) {
    switch (action) {
        case RideManager::Action::StartRide:
            beginRide();
            break;
        case RideManager::Action::EndRide:
            finishRide();
            break;
        case RideManager::Action::PauseRide:
            // The ride file stays open through a stop; only a marker is written.
            recorder_.recordEvent(EventCode::RecordingPaused, 0);
            recorder_.flush();
            break;
        case RideManager::Action::ResumeRide:
            if (!recorder_.isRecording()) {
                beginRide();
            } else {
                recorder_.recordEvent(EventCode::RecordingResumed, 0);
            }
            break;
        case RideManager::Action::None:
            break;
    }
}

bool TelemetrySystem::startRideManually() {
    applyAction(rideManager_.requestManualStart(clock_.millis()));
    return recorder_.isRecording();
}

bool TelemetrySystem::stopRideManually() {
    const bool wasRecording = recorder_.isRecording();
    applyAction(rideManager_.requestManualStop(clock_.millis()));
    return wasRecording;
}

void TelemetrySystem::update() {
    if (gyroBiasRequested_ && !imu_.calibratingGyroBias()) {
        imu_.startGyroBiasCapture();
    }

    // --- GNSS --------------------------------------------------------------
    GnssReading gnssReading;
    if (gnss_.poll(gnssReading)) {
        if (recorder_.isRecording()) {
            recorder_.recordGnss(gnssReading);
        }
    }

    // --- IMU and fusion ----------------------------------------------------
    ImuReading imuReading;
    while (imu_.poll(imuReading)) {
        ++imuSampleCount_;

        ImuReading levelReference;
        if (imu_.consumeLevelReference(levelReference)) {
            // Seed from the stable averaged accelerometer batch, then make that
            // attitude the zero reference. This removes bench/mounting offset
            // without baking one noisy sample into calibration.
            orientation_.seedFromAccel(levelReference);
            orientation_.captureMountingOffset();
            ++mountingCalibrationVersion_;
            gyroBiasRequested_ = false;
            APEX_LOGI("Startup lean/pitch zero applied from accepted calibration batch");
        }

        // Integrate over the measured interval rather than the nominal one, so
        // jitter in the sampling loop does not become attitude error.
        float dt = 0.0f;
        if (haveLastImu_ && imuReading.timestampUs > lastImuUs_) {
            dt = static_cast<float>(imuReading.timestampUs - lastImuUs_) * 1e-6f;
        }
        lastImuUs_   = imuReading.timestampUs;
        haveLastImu_ = true;

        Orientation::KinematicHint hint;
        hint.valid             = gnss_.speedHint(hint.speedMps);
        hint.accelValid        = hint.valid && gnss_.accelerationHint(hint.accelMps2);
        hint.movingWithoutHint = !hint.valid && gnss_.likelyMoving();

        if (dt > 0.0f) {
            orientation_.update(imuReading, dt, hint);
        }

        // Publish a fused sample to the recorder at the configured fusion rate.
        if (imuReading.timestampUs >= lastFusionUs_) {
            lastFusionUs_ += fusionIntervalUs_;
            if (lastFusionUs_ + fusionIntervalUs_ < imuReading.timestampUs) {
                lastFusionUs_ = imuReading.timestampUs + fusionIntervalUs_;
            }

            if (recorder_.isRecording()) {
                recorder_.recordImu(orientation_.state(), imuReading);
            }
        }
    }

    // --- Ride state machine -------------------------------------------------
    const uint32_t nowMs = clock_.millis();

    float      speedMps   = 0.0f;
    const bool speedValid = gnss_.speedHint(speedMps);

    applyAction(rideManager_.update(nowMs, speedValid, speedMps, imu_.lastReading()));

    // --- Recorder housekeeping ---------------------------------------------
    recorder_.update();

    if (recorder_.isRecording() && storage_.mustStopForSpace()) {
        APEX_LOGE("Storage exhausted — closing ride %u to protect the filesystem",
                static_cast<unsigned>(recorder_.currentRideId()));
        recorder_.recordEvent(EventCode::StorageFull, 0);
        finishRide();
        rideManager_.requestManualStop(nowMs);
    }
}

TelemetrySystem::Status TelemetrySystem::status() const {
    Status status;
    status.state         = rideManager_.state();
    status.fused         = orientation_.state();
    status.gnssFix       = gnss_.hasFix();
    status.satellites    = gnss_.lastReading().satellites;
    float usableSpeed = 0.0f;
    status.speedMps      = gnss_.speedHint(usableSpeed) ? usableSpeed : 0.0f;
    status.rawSpeedMps   = gnss_.hasFix() ? gnss_.rawSpeedMps() : 0.0f;
    status.rawAccel      = imu_.lastReading().accel;
    status.rawGyro       = imu_.lastRawGyro();
    status.calibratedGyro = imu_.lastReading().gyro;
    status.accelLeanDeg  = atan2f(status.rawAccel.y, status.rawAccel.z) * kRadToDeg;
    status.accelPitchDeg = -atan2f(-status.rawAccel.x,
        sqrtf(status.rawAccel.y * status.rawAccel.y +
              status.rawAccel.z * status.rawAccel.z)) * kRadToDeg;
    status.activeRideId  = recorder_.currentRideId();
    status.imuSamples    = imuSampleCount_;
    status.imuErrors     = imu_.readErrorCount();
    status.droppedSamples = imu_.droppedSampleCount();
    status.gnssFixes     = gnss_.fixCount();
    status.gnssSolutions = gnss_.solutionCount();
    status.gnssPackets   = gnss_.packetCount();
    status.gnssErrors    = gnss_.parseErrorCount();
    status.accelCorrections = orientation_.accelCorrectionAcceptedCount();
    status.accelRejections = orientation_.accelCorrectionRejectedCount();
    status.timingRejections = orientation_.timingRejectedCount();
    status.calibrationRejections = imu_.calibrationRejectionCount();
    status.calibrationState = imu_.calibrationStateName();
    status.rideBytes     = recorder_.isRecording() ? recorder_.bytesWritten() : 0;
    status.unsyncedRides = storage_.unsyncedCount();
    status.freeBytes     = storage_.freeBytes();
    return status;
}

}  // namespace apex
