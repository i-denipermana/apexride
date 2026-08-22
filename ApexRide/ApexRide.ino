//
// ApexRide V1 — offline-first motorcycle telemetry logger.
//
// The recording pipeline runs with real ICM-20948 and ATGM336H sensors. Flipping
// APEX_USE_MOCK_IMU / APEX_USE_MOCK_GNSS in config.h restores the deterministic
// simulator for host-free bench demonstrations.
//
// Serial commands (115200 baud, one letter then Enter):
//   s  start a ride manually        c  capture the mounting offset
//   x  stop the ride                g  re-measure gyro bias
//   l  list stored rides            k  clear stored calibration
//   i  storage and device info      y  mark every ride synced (sync stand-in)
//   d  dump the newest ride as hex  f  FORMAT the filesystem (destroys rides)
//   p  print the sync API responses  w  toggle the Wi-Fi dashboard
//   h  this help
//

#include "config.h"
#include "src/core/Clock.h"
#include "src/core/Log.h"
#include "src/ride/TelemetrySystem.h"
#include "src/storage/CalibrationStore.h"
#include "src/storage/LittleFsRideStore.h"
#include "src/sync/SyncProtocol.h"
#include "src/sync/SyncService.h"
#include "src/sync/TelemetryStatusSource.h"
#include "src/sync/WifiSyncServer.h"

#if APEX_USE_MOCK_IMU || APEX_USE_MOCK_GNSS
#include "src/sim/RideSimulator.h"
#endif

#if APEX_USE_MOCK_IMU
#include "src/sensors/MockImuSensor.h"
#else
#include "src/sensors/Icm20948Sensor.h"
#endif

#if APEX_USE_MOCK_GNSS
#include "src/sensors/MockGnssSensor.h"
#else
#include "src/sensors/Atgm336hSensor.h"
#endif

using namespace apex;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

namespace {

SystemClock       g_clock;
LittleFsRideStore g_store;
CalibrationStore  g_calibration(APEX_NVS_NAMESPACE);

#if APEX_USE_MOCK_IMU || APEX_USE_MOCK_GNSS
RideSimulator g_simulator;
#endif
#if APEX_USE_MOCK_IMU
MockImuSensor  g_imuSensor(g_clock, g_simulator);
#else
Icm20948Sensor g_imuSensor;
#endif

#if APEX_USE_MOCK_GNSS
MockGnssSensor g_gnssSensor(g_clock, g_simulator);
#else
HardwareSerial  g_gnssUart(APEX_GNSS_UART_NUM);
Atgm336hSensor g_gnssSensor(g_gnssUart, g_clock);
#endif

TelemetrySystem g_system(g_imuSensor, g_gnssSensor, g_store, g_clock);

TelemetryStatusSource g_statusSource(g_system);
SyncService           g_sync(g_system.storage(), g_clock);
SyncProtocol          g_syncProtocol(g_sync);
WifiSyncServer        g_wifi(g_syncProtocol, g_sync, g_system);

/// Buffer for sync JSON responses. Large enough for a full ride manifest, so
/// it comes from PSRAM alongside the telemetry buffer.
char*  g_syncBuffer     = nullptr;
size_t g_syncBufferSize = 0;
uint8_t* g_wifiTransferBuffer = nullptr;
bool   g_systemReady    = false;

/// Telemetry staging buffer. Placed in PSRAM at boot; the static array is the
/// fallback for a board without working PSRAM.
uint8_t  g_fallbackBuffer[APEX_RECORD_BUFFER_BYTES];
uint8_t* g_recordBuffer = g_fallbackBuffer;
uint8_t  g_fallbackWifiTransferBuffer[APEX_WIFI_TRANSFER_BUFFER_BYTES];

uint32_t g_lastStatusMs   = 0;
uint32_t g_lastHealthMs   = 0;
uint16_t g_savedCalVersion = 0;
uint16_t g_savedMountVersion = 0;
uint32_t g_healthImuSamples = 0;
uint32_t g_healthGnssSolutions = 0;

// ---------------------------------------------------------------------------
// Logging bridge
// ---------------------------------------------------------------------------

void serialLogSink(LogLevel level, const char* line) {
    static const char* const kPrefix[] = {"ERR ", "WARN", "INFO", "DBG "};
    Serial.print('[');
    Serial.print(kPrefix[static_cast<int>(level)]);
    Serial.print("] ");
    Serial.println(line);
}

// ---------------------------------------------------------------------------
// Configuration assembly
// ---------------------------------------------------------------------------

TelemetrySystem::Config buildConfig() {
    TelemetrySystem::Config config;

#if APEX_USE_MOCK_IMU
    // The mock emulates a breakout rotated 90 degrees about the vertical axis,
    // so the demo exercises the axis map rather than bypassing it:
    //   body X = -sensor Y,  body Y = +sensor X,  body Z = +sensor Z
    config.imu.axisMap.sourceIndex[0] = 1;
    config.imu.axisMap.sign[0]        = -1;
    config.imu.axisMap.sourceIndex[1] = 0;
    config.imu.axisMap.sign[1]        = 1;
    config.imu.axisMap.sourceIndex[2] = 2;
    config.imu.axisMap.sign[2]        = 1;
#else
    // Bench validation found that physical lean uses sensor accel X + gyro Y.
    // This rotation preserves the firmware convention: left lean is negative,
    // right lean is positive, and a level board reads body Z = +1 g.
    config.imu.axisMap.sourceIndex[0] = 1;  // body X = -sensor Y
    config.imu.axisMap.sign[0]        = -1;
    config.imu.axisMap.sourceIndex[1] = 0;  // body Y = +sensor X
    config.imu.axisMap.sign[1]        = 1;
    config.imu.axisMap.sourceIndex[2] = 2;  // body Z = +sensor Z
    config.imu.axisMap.sign[2]        = 1;
#endif

    config.orientation.kp = APEX_MAHONY_KP;
    config.orientation.ki = APEX_MAHONY_KI;
    config.orientation.minDtSeconds = APEX_FUSION_MIN_DT_SECONDS;
    config.orientation.maxDtSeconds = APEX_FUSION_MAX_DT_SECONDS;

    config.imu.biasSampleCount = APEX_CALIBRATION_SAMPLES;
    config.imu.stationaryHoldMs = APEX_CALIBRATION_STILL_HOLD_MS;
    config.imu.calibrationMaxGyroDps = APEX_CALIBRATION_MAX_GYRO_DPS;
    config.imu.calibrationMaxAccelMps2 = APEX_CALIBRATION_MAX_ACCEL_ERROR_MPS2;
    config.imu.calibrationMaxGyroStdDps = APEX_CALIBRATION_MAX_GYRO_STD_DPS;
    config.imu.calibrationMaxAccelStdMps2 = APEX_CALIBRATION_MAX_ACCEL_STD_MPS2;
    config.imu.calibrationMaxLevelAngleDeg = APEX_CALIBRATION_MAX_LEVEL_ANGLE_DEG;

    config.gnss.speedDeadbandMps = APEX_GNSS_SPEED_DEADBAND_MPS;
    config.gnss.speedFilterAlpha = APEX_GNSS_SPEED_FILTER_ALPHA;

    // Kinematic correction is valid only when IMU and GNSS describe the same
    // motion. During real-IMU bring-up the GNSS stream is still a script, so
    // feeding its fictitious speed into live hand motion would corrupt lean.
    config.orientation.useKinematicCorrection =
        APEX_USE_KINEMATIC_CORRECTION &&
        (APEX_USE_MOCK_IMU == APEX_USE_MOCK_GNSS);
    config.orientation.minSpeedForCorrectionMps = APEX_MIN_SPEED_FOR_CORRECTION_MPS;

    config.ride.autoStart             = APEX_RIDE_AUTO_START != 0;
    config.ride.startSpeedMps         = APEX_RIDE_START_SPEED_MPS;
    config.ride.stopSpeedMps          = APEX_RIDE_STOP_SPEED_MPS;
    config.ride.startHoldMs           = APEX_RIDE_START_HOLD_MS;
    config.ride.waitingEnterMs        = APEX_RIDE_WAITING_ENTER_MS;
    config.ride.waitingEnterNoGnssMs  = APEX_RIDE_WAITING_ENTER_NO_GNSS_MS;
    config.ride.waitingTimeoutMs      = APEX_RIDE_WAITING_TIMEOUT_MS;
    config.ride.sleepTimeoutMs        = APEX_RIDE_SLEEP_TIMEOUT_MS;

#if !APEX_USE_MOCK_IMU && APEX_USE_MOCK_GNSS
    // The scripted GNSS speed must not start fake rides while a real IMU is on
    // the bench. Manual recording with the 's' command remains available for
    // explicit flash/I2C stress tests.
    config.ride.startSpeedMps = 1000.0f;
#endif

    config.recorder.imuLogRateHz       = APEX_IMU_LOG_RATE_HZ;
    config.recorder.blockSizeBytes     = APEX_RECORD_BLOCK_BYTES;
    config.recorder.maxFlushIntervalMs = APEX_MAX_FLUSH_INTERVAL_MS;
    config.recorder.summaryIntervalMs  = APEX_SUMMARY_INTERVAL_MS;

    config.storage.basePath               = APEX_RIDES_DIRECTORY;
    config.storage.minFreeBytesToStart    = APEX_MIN_FREE_BYTES_TO_START;
    config.storage.minFreeBytesToContinue = APEX_MIN_FREE_BYTES_TO_CONTINUE;

    config.fusionRateHz = APEX_FUSION_RATE_HZ;

    return config;
}

// ---------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------

void printStorageBudget() {
    const uint64_t total = g_system.storage().totalBytes();
    const float    bytesPerSecond =
        APEX_IMU_LOG_RATE_HZ * (sizeof(RecordHeader) + sizeof(ImuRecord)) +
        APEX_GNSS_RATE_HZ * (sizeof(RecordHeader) + sizeof(GnssRecord));

    Serial.printf("Storage budget: %.0f B/s -> %.1f MB/hour; %llu KB holds ~%.1f hours\n",
                  bytesPerSecond, bytesPerSecond * 3600.0f / (1024.0f * 1024.0f),
                  static_cast<unsigned long long>(total / 1024),
                  static_cast<float>(total) / bytesPerSecond / 3600.0f);
}

void printStatus() {
    const TelemetrySystem::Status status = g_system.status();

    Serial.printf(
        "%-9s ride %-4u | lean %+6.1f pitch %+6.1f | %5.1f km/h | GNSS %s %2u sat | "
        "%6u smp %5u KB | free %5llu KB | unsynced %u\n",
        toString(status.state), static_cast<unsigned>(status.activeRideId),
        static_cast<double>(status.fused.rollDeg()),
        static_cast<double>(status.fused.pitchDeg()),
        static_cast<double>(status.speedMps * 3.6f), status.gnssFix ? "fix" : "---",
        static_cast<unsigned>(status.satellites), static_cast<unsigned>(status.imuSamples),
        static_cast<unsigned>(status.rideBytes / 1024),
        static_cast<unsigned long long>(status.freeBytes / 1024),
        static_cast<unsigned>(status.unsyncedRides));
}

void printHealth(uint32_t nowMs) {
    const TelemetrySystem::Status status = g_system.status();
    const uint32_t elapsedMs = g_lastHealthMs == 0 ? APEX_HEALTH_INTERVAL_MS
                                                   : nowMs - g_lastHealthMs;
    const float seconds = elapsedMs > 0 ? elapsedMs / 1000.0f : 1.0f;
    const float imuRate = (status.imuSamples - g_healthImuSamples) / seconds;
    const float gnssRate = (status.gnssSolutions - g_healthGnssSolutions) / seconds;

    Serial.printf(
        "HEALTH IMU %.1f Hz err/drop %u/%u | GNSS %.2f Hz sol/pkt/err %u/%u/%u | "
        "CAL %s reject %u | FUSE accel %u/%u dtReject %u\n",
        static_cast<double>(imuRate), static_cast<unsigned>(status.imuErrors),
        static_cast<unsigned>(status.droppedSamples), static_cast<double>(gnssRate),
        static_cast<unsigned>(status.gnssSolutions), static_cast<unsigned>(status.gnssPackets),
        static_cast<unsigned>(status.gnssErrors), status.calibrationState,
        static_cast<unsigned>(status.calibrationRejections),
        static_cast<unsigned>(status.accelCorrections),
        static_cast<unsigned>(status.accelRejections),
        static_cast<unsigned>(status.timingRejections));
    Serial.printf(
        "RAW A[%+.2f %+.2f %+.2f] G[%+.2f %+.2f %+.2f] dps -> "
        "Gcal[%+.2f %+.2f %+.2f] | accel angle %+.1f/%+.1f | speed %.1f -> %.1f km/h\n",
        static_cast<double>(status.rawAccel.x), static_cast<double>(status.rawAccel.y),
        static_cast<double>(status.rawAccel.z),
        static_cast<double>(status.rawGyro.x * kRadToDeg),
        static_cast<double>(status.rawGyro.y * kRadToDeg),
        static_cast<double>(status.rawGyro.z * kRadToDeg),
        static_cast<double>(status.calibratedGyro.x * kRadToDeg),
        static_cast<double>(status.calibratedGyro.y * kRadToDeg),
        static_cast<double>(status.calibratedGyro.z * kRadToDeg),
        static_cast<double>(status.accelLeanDeg), static_cast<double>(status.accelPitchDeg),
        static_cast<double>(status.rawSpeedMps * 3.6f),
        static_cast<double>(status.speedMps * 3.6f));

    g_healthImuSamples = status.imuSamples;
    g_healthGnssSolutions = status.gnssSolutions;
    g_lastHealthMs = nowMs;
}

void listRides() {
    RideStorage& storage = g_system.storage();
    storage.refresh();

    Serial.printf("\n%u ride(s), %llu KB free of %llu KB\n",
                  static_cast<unsigned>(storage.rideCount()),
                  static_cast<unsigned long long>(storage.freeBytes() / 1024),
                  static_cast<unsigned long long>(storage.totalBytes() / 1024));
    Serial.println("  id      dur    dist     max     lean L/R      size  state");

    for (size_t i = 0; i < storage.rideCount(); ++i) {
        const RideStorage::RideEntry& entry = storage.rideAt(i);

        if (!entry.summaryValid) {
            Serial.printf("  R%06u  (summary missing or corrupt, %u KB of data)\n",
                          static_cast<unsigned>(entry.rideId),
                          static_cast<unsigned>(entry.dataBytes / 1024));
            continue;
        }

        const RideSummary& s = entry.summary;
        Serial.printf("  R%06u  %4us  %6.2fkm  %3ukm/h  %5.1f/%5.1f  %5uKB  %s%s%s\n",
                      static_cast<unsigned>(s.rideId), static_cast<unsigned>(s.durationMs / 1000),
                      s.distanceCm / 100000.0, static_cast<unsigned>(s.maxSpeed * 36 / 1000),
                      s.maxLeanLeft / 100.0, s.maxLeanRight / 100.0,
                      static_cast<unsigned>(s.fileSizeBytes / 1024),
                      s.isSynced() ? "synced" : "UNSYNCED",
                      (s.flags & kRideFlagRecovered) ? " recovered" : "",
                      (s.flags & kRideFlagTruncated) ? " truncated" : "");
    }
    Serial.println();
}

void dumpNewestRide() {
    RideStorage& storage = g_system.storage();
    storage.refresh();

    if (storage.rideCount() == 0) {
        Serial.println("No rides stored.");
        return;
    }

    const uint32_t rideId = storage.rideAt(storage.rideCount() - 1).rideId;

    IRideFile* file = storage.openRideFile(rideId, FileMode::Read);
    if (file == nullptr) {
        Serial.println("Could not open the ride file.");
        return;
    }

    Serial.printf("\nR%06u first 256 bytes:\n", static_cast<unsigned>(rideId));

    uint8_t      chunk[16];
    size_t       offset = 0;
    while (offset < 256) {
        const size_t got = file->read(chunk, sizeof(chunk));
        if (got == 0) break;

        Serial.printf("%04x  ", static_cast<unsigned>(offset));
        for (size_t i = 0; i < got; ++i) {
            Serial.printf("%02x ", chunk[i]);
        }
        Serial.println();
        offset += got;
    }

    file->close();
    delete file;
    Serial.println();
}

/// Development shortcut for exercising retention without a phone app. Browser
/// downloads deliberately remain unsynced because a web page cannot prove the
/// file reached durable phone storage.
void markAllSynced() {
    RideStorage& storage = g_system.storage();
    storage.refresh();

    uint32_t marked = 0;
    while (true) {
        uint32_t target = 0;
        for (size_t i = 0; i < storage.rideCount(); ++i) {
            const RideStorage::RideEntry& entry = storage.rideAt(i);
            if (entry.summaryValid && !entry.summary.isSynced()) {
                target = entry.rideId;
                break;
            }
        }
        if (target == 0) break;
        if (!storage.markSynced(target)) break;
        ++marked;
    }

    Serial.printf("Marked %u ride(s) synced.\n", static_cast<unsigned>(marked));
}

void printHelp() {
    Serial.println(
        "\nCommands: s=start x=stop c=calibrate-mounting g=gyro-bias l=list i=info\n"
        "          d=dump-newest y=mark-all-synced p=sync-api w=toggle-wifi\n"
        "          k=clear-calibration\n"
        "          f=format h=help\n");
}

/// Runs the sync API locally and prints what it returns.
///
/// Serial probe for the same protocol now served by WifiSyncServer.
void printSyncApi() {
    if (g_syncBuffer == nullptr) {
        Serial.println("Sync buffer unavailable.");
        return;
    }

    struct Probe {
        const char* method;
        const char* path;
    };
    const Probe probes[] = {
        {"GET", "/status"},
        {"POST", "/sync/begin"},
        {"GET", "/sync/pending"},
        {"GET", "/rides"},
        {"POST", "/sync/end"},
    };

    Serial.println();
    for (const Probe& probe : probes) {
        const SyncResponse response =
            g_syncProtocol.route(probe.method, probe.path, nullptr, g_syncBuffer,
                                 g_syncBufferSize);

        Serial.printf("%-4s %-14s -> %u  ", probe.method, probe.path, response.status);
        if (response.isRideData) {
            Serial.printf("<%u bytes of ride data>\n",
                          static_cast<unsigned>(response.dataLength));
        } else {
            Serial.println(response.body);
        }
    }
    Serial.println();
}

void printInfo() {
    Serial.printf("\nApexRide firmware 0x%04x, format v%u\n", kFirmwareVersion,
                  kFormatVersion);
    Serial.printf("Sensors: %s + %s\n", g_imuSensor.name(), g_gnssSensor.name());
    Serial.printf("Ride recording: %s (s=start, x=stop)\n",
                  APEX_RIDE_AUTO_START ? "AUTOMATIC" : "MANUAL ONLY");
    Serial.printf("Wi-Fi: %s", g_wifi.running() ? "ON" : "off");
    if (g_wifi.running()) {
        Serial.printf(" — %s at http://%s (%u client%s)", APEX_WIFI_SSID,
                      g_wifi.address().toString().c_str(),
                      static_cast<unsigned>(g_wifi.clientCount()),
                      g_wifi.clientCount() == 1 ? "" : "s");
    }
    Serial.println();
    Serial.printf("Heap: %u free, PSRAM: %u free\n", static_cast<unsigned>(ESP.getFreeHeap()),
                  static_cast<unsigned>(ESP.getFreePsram()));
    Serial.printf("Record sizes: IMU %u B, GNSS %u B, summary %u B\n",
                  static_cast<unsigned>(sizeof(RecordHeader) + sizeof(ImuRecord)),
                  static_cast<unsigned>(sizeof(RecordHeader) + sizeof(GnssRecord)),
                  static_cast<unsigned>(sizeof(RideSummary)));
    printStorageBudget();
    Serial.println();
}

/// Persists whatever calibration has been captured, if it changed.
void saveCalibrationIfChanged() {
    const ImuCalibration& calibration = g_system.imu().calibration();
    if (calibration.gyroBiasValid && calibration.version != g_savedCalVersion) {
        if (g_calibration.saveImuCalibration(calibration)) {
            g_savedCalVersion = calibration.version;
        }
    }

    const uint16_t mountVersion = g_system.mountingCalibrationVersion();
    if (mountVersion != g_savedMountVersion) {
        if (g_calibration.saveMountingOffset(g_system.orientation().mountingOffset())) {
            g_savedMountVersion = mountVersion;
        }
    }
}

void handleSerialCommand(char command) {
    switch (command) {
        case 's':
            Serial.println(g_system.startRideManually() ? "Ride started." : "Could not start.");
            break;
        case 'x':
            Serial.println(g_system.stopRideManually() ? "Ride stopped." : "No ride was running.");
            break;
        case 'c':
            if (g_system.calibrateMounting()) {
                g_calibration.saveMountingOffset(g_system.orientation().mountingOffset());
                Serial.println("Mounting offset captured and saved.");
            } else {
                Serial.println("Hold the bike upright and still, then try again.");
            }
            break;
        case 'g':
            g_system.calibrateGyroBias();
            Serial.println("Measuring gyro bias — keep the bike still.");
            break;
        case 'l': listRides(); break;
        case 'i': printInfo(); break;
        case 'd': dumpNewestRide(); break;
        case 'y': markAllSynced(); break;
        case 'p': printSyncApi(); break;
        case 'w':
            if (g_wifi.running()) {
                g_wifi.stop();
                Serial.println("Wi-Fi dashboard stopped.");
            } else {
                Serial.println(g_wifi.start() ? "Wi-Fi dashboard started." :
                                                "Could not start Wi-Fi dashboard.");
            }
            break;
        case 'k':
            g_calibration.clear();
            g_system.orientation().clearMountingOffset();
            Serial.println("Calibration cleared. Reboot to start fresh.");
            break;
        case 'f':
            Serial.println("Formatting — every stored ride will be lost.");
            g_system.stopRideManually();
            g_store.format();
            Serial.println("Done. Reboot the device.");
            break;
        case 'h':
        case '?': printHelp(); break;
        default: break;
    }
}

void pollSerialCommands() {
    while (Serial.available() > 0) {
        const char command = static_cast<char>(Serial.read());
        if (command != '\n' && command != '\r' && command != ' ') {
            handleSerialCommand(command);
        }
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(APEX_SERIAL_BAUD);
#if APEX_WAIT_FOR_SERIAL
    while (!Serial) {
        delay(10);
    }
#else
    delay(500);  // give the USB CDC port a moment to enumerate
#endif

    setLogSink(serialLogSink);
    setLogLevel(LogLevel::Info);

    Serial.println("\n\n=== ApexRide V1 ===");

    // Prefer PSRAM for the telemetry buffer, keeping internal SRAM for stacks.
    if (psramFound()) {
        uint8_t* psramBuffer = static_cast<uint8_t*>(ps_malloc(APEX_RECORD_BUFFER_BYTES));
        if (psramBuffer != nullptr) {
            g_recordBuffer = psramBuffer;
            APEX_LOGI("Telemetry buffer allocated in PSRAM (%u bytes)", APEX_RECORD_BUFFER_BYTES);
        }
    } else {
        APEX_LOGW("No PSRAM detected — using internal SRAM for the telemetry buffer");
    }

    // Production safety: a mount failure must never erase unsynced rides.
    if (!g_store.begin()) {
        APEX_LOGE("Filesystem unavailable; NOT formatting. Recording is disabled");
        return;
    }

    g_calibration.begin();

#if APEX_USE_MOCK_IMU || APEX_USE_MOCK_GNSS
    size_t             segmentCount = 0;
    const RideSegment* script       = defaultRideScript(segmentCount);
    g_simulator.begin(RideSimulator::Config(), script, segmentCount);
    APEX_LOGW("Simulation enabled for %s%s%s — scripted %u second ride starts at boot",
              APEX_USE_MOCK_IMU ? "IMU" : "",
              (APEX_USE_MOCK_IMU && APEX_USE_MOCK_GNSS) ? " + " : "",
              APEX_USE_MOCK_GNSS ? "GNSS" : "",
              static_cast<unsigned>(g_simulator.totalDurationMs() / 1000));
#endif

#if !APEX_USE_MOCK_IMU && APEX_USE_MOCK_GNSS
    APEX_LOGW("Mixed bench mode: GNSS correction and automatic rides disabled");
#endif

#if APEX_USE_MOCK_IMU
    MockImuSensor::Config imuConfig;
    imuConfig.sampleRateHz = APEX_IMU_SAMPLE_RATE_HZ;
    g_imuSensor.setConfig(imuConfig);
#else
    Icm20948Sensor::Config imuConfig;
    imuConfig.sdaPin       = APEX_I2C_SDA_PIN;
    imuConfig.sclPin       = APEX_I2C_SCL_PIN;
    imuConfig.i2cFrequency = APEX_I2C_FREQUENCY;
    imuConfig.address      = APEX_IMU_I2C_ADDRESS;
    imuConfig.sampleRateHz = APEX_IMU_SAMPLE_RATE_HZ;
    g_imuSensor.setConfig(imuConfig);
#endif

#if APEX_USE_MOCK_GNSS
    MockGnssSensor::Config gnssConfig;
    gnssConfig.updateRateHz = APEX_GNSS_RATE_HZ;
    g_gnssSensor.setConfig(gnssConfig);
#else
    Atgm336hSensor::Config gnssConfig;
    gnssConfig.rxPin        = APEX_GNSS_RX_PIN;
    gnssConfig.txPin        = APEX_GNSS_TX_PIN;
    gnssConfig.baud         = APEX_GNSS_BAUD;
    gnssConfig.updateRateHz = APEX_GNSS_RATE_HZ;
    g_gnssSensor.setConfig(gnssConfig);
#endif

    if (!g_system.begin(buildConfig(), g_recordBuffer, APEX_RECORD_BUFFER_BYTES)) {
        APEX_LOGE("Telemetry system failed to start");
        return;
    }
    g_systemReady = true;

    // Restore calibration so the device is usable immediately after a reboot.
    ImuCalibration storedCalibration;
    if (g_calibration.loadImuCalibration(storedCalibration)) {
        g_system.imu().setCalibration(storedCalibration);
        g_savedCalVersion = storedCalibration.version;
    }

    Quaternion storedMounting;
    if (g_calibration.loadMountingOffset(storedMounting)) {
        g_system.orientation().setMountingOffset(storedMounting);
    } else {
        APEX_LOGW("No mounting calibration stored — park upright and level, then press 'c'");
    }

    // Sync service and local Wi-Fi transport. Protocol and storage safety stay
    // independent of networking; WifiSyncServer only adapts HTTP requests.
    g_syncBufferSize = SyncProtocol::manifestBufferSize(RideStorage::kMaxRides);
    g_syncBuffer     = static_cast<char*>(psramFound() ? ps_malloc(g_syncBufferSize)
                                                       : malloc(g_syncBufferSize));
    if (g_syncBuffer == nullptr) {
        APEX_LOGE("Could not allocate the %u byte sync buffer",
                  static_cast<unsigned>(g_syncBufferSize));
        g_syncBufferSize = 0;
    }

    SyncService::Config syncConfig;
    syncConfig.deviceName = APEX_DEVICE_NAME;
    syncConfig.maxChunkBytes = APEX_WIFI_TRANSFER_BUFFER_BYTES;
    g_sync.begin(syncConfig);
    g_sync.setStatusSource(&g_statusSource);

    g_wifiTransferBuffer = g_fallbackWifiTransferBuffer;
    if (psramFound()) {
        uint8_t* psramTransfer = static_cast<uint8_t*>(ps_malloc(APEX_WIFI_TRANSFER_BUFFER_BYTES));
        if (psramTransfer != nullptr) g_wifiTransferBuffer = psramTransfer;
    }

#if APEX_WIFI_ENABLED
    if (g_syncBuffer != nullptr) {
        WifiSyncServer::Config wifiConfig;
        wifiConfig.ssid = APEX_WIFI_SSID;
        wifiConfig.password = APEX_WIFI_PASSWORD;
        wifiConfig.channel = APEX_WIFI_CHANNEL;
        wifiConfig.maxClients = APEX_WIFI_MAX_CLIENTS;
        if (!g_wifi.begin(wifiConfig, g_syncBuffer, g_syncBufferSize,
                          g_wifiTransferBuffer, APEX_WIFI_TRANSFER_BUFFER_BYTES)) {
            APEX_LOGE("Wi-Fi dashboard unavailable; telemetry recording will continue");
        }
    }
#endif

    printInfo();
    printHelp();
    listRides();
}

void loop() {
    if (!g_systemReady) {
        delay(100);
        return;
    }

    g_system.update();
    g_sync.update();
    g_wifi.update();
    pollSerialCommands();
    saveCalibrationIfChanged();

    const uint32_t nowMs = millis();
    if (nowMs - g_lastStatusMs >= APEX_STATUS_INTERVAL_MS) {
        g_lastStatusMs = nowMs;
        printStatus();
    }
    if (nowMs - g_lastHealthMs >= APEX_HEALTH_INTERVAL_MS) {
        printHealth(nowMs);
    }
}
