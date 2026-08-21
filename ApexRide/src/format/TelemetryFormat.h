#pragma once
//
// ApexRide on-disk ride format, version 1.
//
// A ride is stored as two files:
//
//   /rides/R000001.bin   append-only record stream
//   /rides/R000001.met   fixed-size RideSummary, rewritten periodically
//
// The summary lives in its own file so the phone can list rides without
// scanning gigabytes of samples, and so the append-only data file is never
// seeked back into while recording. If a .met file is missing or fails its
// CRC (battery died mid-ride), RideStorage rebuilds it by replaying the .bin.
//
// .bin layout:
//
//   [FileHeader (32 bytes)]
//   [RecordHeader][payload] [RecordHeader][payload] ...
//
// Every record is length-prefixed, so a reader that meets an unknown record
// type can skip it. That is the forward-compatibility hook for V2.
//
// All integers are little-endian (ESP32, ARM and x86 all agree).
// All physical quantities are fixed-point; no floats and no text are stored.
//

#include <stddef.h>
#include <stdint.h>

namespace apex {

/// Little-endian, so these read as ASCII in a hex dump of the first four bytes.
constexpr uint32_t kFileMagic    = 0x31445241u;  ///< 'A','R','D','1' — ApexRide Data
constexpr uint32_t kSummaryMagic = 0x31535241u;  ///< 'A','R','S','1' — ApexRide Summary

constexpr uint16_t kFormatVersion = 1;

/// Bumped whenever the meaning of a record changes.
constexpr uint16_t kFirmwareVersion = 0x0103;  ///< 1.3: calibration and data-quality hardening

enum class RecordType : uint8_t {
    Imu   = 0x01,
    Gnss  = 0x02,
    Event = 0x03,
};

enum class EventCode : uint8_t {
    RideStart          = 1,
    RideEnd            = 2,
    GnssFixAcquired    = 3,
    GnssFixLost        = 4,
    CalibrationApplied = 5,
    StorageLow         = 6,
    RecordingPaused    = 7,
    RecordingResumed   = 8,
    BufferOverrun      = 9,
    StorageFull        = 10,
};

#pragma pack(push, 1)

struct FileHeader {
    uint32_t magic;
    uint16_t formatVersion;
    uint16_t headerSize;
    uint32_t rideId;
    uint32_t startUnixTime;  ///< 0 until GNSS provides a fix
    uint32_t startMillis;    ///< device monotonic ms at ride start
    uint16_t firmwareVersion;
    uint16_t calibrationVersion;
    uint16_t imuLogRateHz;
    uint16_t gnssLogRateHz;
    uint32_t headerCrc;  ///< CRC32 of the preceding 28 bytes
};

struct RecordHeader {
    uint8_t type;    ///< RecordType
    uint8_t length;  ///< payload bytes following this header
};

/// Fused attitude plus raw motion. Written at TelemetryRecorder::Config::imuLogRateHz.
struct ImuRecord {
    uint32_t timestampMs;  ///< device monotonic ms, same timebase as every other record

    int16_t roll;   ///< degrees x100, positive = lean right
    int16_t pitch;  ///< degrees x100, positive = nose up
    int16_t yaw;    ///< degrees x100, positive = turning left, relative

    int16_t accelX;  ///< milli-g
    int16_t accelY;
    int16_t accelZ;

    int16_t gyroX;  ///< deci-degrees/second (dps x10)
    int16_t gyroY;
    int16_t gyroZ;
};

/// Written once per GNSS solution — never duplicated into ImuRecord.
struct GnssRecord {
    uint32_t timestampMs;
    uint32_t unixTime;

    int32_t latitude;   ///< degrees x1e7
    int32_t longitude;  ///< degrees x1e7

    uint16_t speed;      ///< cm/s
    uint16_t heading;    ///< degrees x100, 0..35999 compass
    int16_t  altitude;   ///< decimetres

    uint8_t hdop;        ///< HDOP x10, saturating at 25.5
    uint8_t satellites;
    uint8_t fixType;     ///< GnssFix
    uint8_t reserved;
};

struct EventRecord {
    uint32_t timestampMs;
    uint8_t  code;  ///< EventCode
    uint8_t  reserved;
    int32_t  value;  ///< code-specific payload
};

/// Ride flags stored in RideSummary::flags.
enum RideFlags : uint16_t {
    kRideFlagSynced    = 1u << 0,  ///< phone confirmed receipt; safe to delete
    kRideFlagClosed    = 1u << 1,  ///< ride ended cleanly
    kRideFlagRecovered = 1u << 2,  ///< summary rebuilt by scanning the .bin
    kRideFlagTruncated = 1u << 3,  ///< trailing partial record found during scan
};

struct RideSummary {
    uint32_t magic;
    uint16_t version;
    uint16_t flags;  ///< RideFlags

    uint32_t rideId;
    uint32_t startUnixTime;
    uint32_t endUnixTime;
    uint32_t durationMs;

    uint32_t distanceCm;
    uint32_t imuSampleCount;
    uint32_t gnssSampleCount;
    uint32_t eventCount;
    uint32_t fileSizeBytes;

    uint16_t maxSpeed;          ///< cm/s
    uint16_t maxLeanLeft;       ///< degrees x100, magnitude
    uint16_t maxLeanRight;      ///< degrees x100, magnitude
    int16_t  maxPitchUp;        ///< degrees x100
    int16_t  maxPitchDown;      ///< degrees x100, magnitude
    uint16_t maxAcceleration;   ///< milli-g, magnitude
    uint16_t maxBraking;        ///< milli-g, magnitude
    uint16_t firmwareVersion;
    uint16_t calibrationVersion;
    uint16_t reserved;

    uint32_t dataCrc;     ///< CRC32 over the whole .bin file
    uint32_t summaryCrc;  ///< CRC32 of the preceding 68 bytes

    bool isSynced() const { return (flags & kRideFlagSynced) != 0; }
    bool isClosed() const { return (flags & kRideFlagClosed) != 0; }
};

#pragma pack(pop)

static_assert(sizeof(FileHeader) == 32, "FileHeader layout changed");
static_assert(sizeof(RecordHeader) == 2, "RecordHeader layout changed");
static_assert(sizeof(ImuRecord) == 22, "ImuRecord layout changed");
static_assert(sizeof(GnssRecord) == 26, "GnssRecord layout changed");
static_assert(sizeof(EventRecord) == 10, "EventRecord layout changed");
static_assert(sizeof(RideSummary) == 72, "RideSummary layout changed");

/// Bytes on disk per logged record, including framing.
constexpr size_t kImuRecordBytes   = sizeof(RecordHeader) + sizeof(ImuRecord);
constexpr size_t kGnssRecordBytes  = sizeof(RecordHeader) + sizeof(GnssRecord);
constexpr size_t kEventRecordBytes = sizeof(RecordHeader) + sizeof(EventRecord);

// --- Fixed-point conversion helpers -----------------------------------------
//
// Saturating, so a sensor glitch produces a clipped value rather than a
// wrapped one that would look like a violent flick in the opposite direction.

int16_t  toCentiDegrees(float degrees);
int16_t  toMilliG(float mps2);
int16_t  toDeciDps(float radPerSec);
int32_t  toDegreesE7(double degrees);
uint16_t toCmPerSec(float mps);
uint16_t toCentiDegreesUnsigned(float degrees);
int16_t  toDecimetres(float metres);
uint8_t  toHdopByte(float hdop);

inline float fromCentiDegrees(int16_t v) { return v * 0.01f; }
inline float fromMilliG(int16_t v) { return v * 0.001f; }
inline float fromDeciDps(int16_t v) { return v * 0.1f; }
inline double fromDegreesE7(int32_t v) { return v * 1e-7; }

}  // namespace apex
