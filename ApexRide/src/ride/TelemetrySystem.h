#pragma once
//
// Composition root: wires the sensors, fusion, ride detection, recording and
// storage into one object with a single update() call.
//
// Both the Arduino sketch and the host test drive this same class, so what the
// tests verify is what runs on the device.
//

#include "../fusion/Orientation.h"
#include "../sensors/GpsManager.h"
#include "../sensors/ImuManager.h"
#include "../storage/RideStorage.h"
#include "RideManager.h"
#include "TelemetryRecorder.h"

namespace apex {

class TelemetrySystem {
public:
    struct Config {
        ImuManager::Config        imu;
        GpsManager::Config        gnss;
        Orientation::Config       orientation;
        RideManager::Config       ride;
        TelemetryRecorder::Config recorder;
        RideStorage::Config       storage;

        /// Fusion output rate. The IMU is sampled faster; every sample is
        /// integrated, but only this often is a fused state published.
        float fusionRateHz = 100.0f;

        /// Capture gyro bias automatically once the bike has been still for a
        /// moment after boot.
        bool autoCalibrateGyroBias = true;
    };

    struct Status {
        RideState state          = RideState::Sleep;
        FusedState fused;
        bool       gnssFix       = false;
        uint8_t    satellites    = 0;
        float      speedMps      = 0.0f;
        uint32_t   activeRideId  = 0;
        uint32_t   imuSamples    = 0;
        uint32_t   gnssFixes     = 0;
        uint32_t   rideBytes     = 0;
        uint32_t   unsyncedRides = 0;
        uint64_t   freeBytes     = 0;
    };

    TelemetrySystem(IImuSensor& imuSensor, IGnssSensor& gnssSensor, IRideStore& store,
                    const Clock& clock);

    /// `recorderBuffer` must outlive the system; on the ESP32 it is allocated
    /// in PSRAM.
    bool begin(const Config& config, uint8_t* recorderBuffer, size_t recorderBufferSize);

    /// Call as often as possible from the main loop.
    void update();

    /// Records the current attitude as upright. The bike must be stationary,
    /// upright and on level ground. Returns false if it is moving.
    bool calibrateMounting();

    /// Measures the gyroscope zero-rate offset. The bike must not move.
    void calibrateGyroBias();

    bool startRideManually();
    bool stopRideManually();

    Status status() const;

    RideStorage&       storage() { return storage_; }
    ImuManager&        imu() { return imu_; }
    GpsManager&        gnss() { return gnss_; }
    Orientation&       orientation() { return orientation_; }
    TelemetryRecorder& recorder() { return recorder_; }
    RideManager&       rideManager() { return rideManager_; }

private:
    void applyAction(RideManager::Action action);
    void beginRide();
    void finishRide();

    const Clock& clock_;
    IRideStore&  store_;
    Config       config_{};

    ImuManager        imu_;
    GpsManager        gnss_;
    Orientation       orientation_;
    RideStorage       storage_;
    TelemetryRecorder recorder_;
    RideManager       rideManager_;

    uint64_t lastFusionUs_    = 0;
    uint64_t fusionIntervalUs_ = 10000;
    uint64_t lastImuUs_       = 0;
    bool     haveLastImu_     = false;

    bool     gyroBiasRequested_ = false;
    bool     spaceWarningSent_  = false;
    uint32_t imuSampleCount_    = 0;
};

}  // namespace apex
