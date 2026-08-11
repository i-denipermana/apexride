#pragma once
//
// Device side of ride synchronisation, independent of how the bytes travel.
//
// ---------------------------------------------------------------------------
// The rule this exists to enforce
//
// A ride is marked synced ONLY after the phone has downloaded it, computed a
// CRC-32 over what it received, sent that CRC back, and had the device confirm
// it matches. Until then the ride stays unsynced and is therefore undeletable
// by the retention policy.
//
// A download that starts, or even completes, proves nothing — the phone may
// have written a corrupt file or crashed before saving it. So acknowledge()
// takes the phone's own checksum and is the only path that sets the flag.
//
// ---------------------------------------------------------------------------
// Resume
//
// Chunk reads are addressed by absolute offset and carry no session state the
// phone depends on, so a transfer that dies at 80% resumes at 80% rather than
// starting again. That matters over a link the rider may walk out of range of.
//
// A file handle is cached across sequential chunks purely for speed; dropping
// it is always safe.
//

#include <stddef.h>
#include <stdint.h>

#include "../core/Clock.h"
#include "../storage/RideStorage.h"

namespace apex {

/// Live device state that the storage layer does not know about.
struct DeviceStatus {
    bool     recording    = false;
    uint32_t activeRideId = 0;
    bool     gnssFix      = false;
    uint8_t  satellites   = 0;

    /// V1 hardware has no way to measure cell voltage — the TP4056 and MT3608
    /// do not expose it and the DevKitC-1 has no divider. Reported as
    /// unavailable rather than as a fabricated percentage.
    bool    batteryAvailable = false;
    uint8_t batteryPercent   = 0;
};

class IDeviceStatusSource {
public:
    virtual ~IDeviceStatusSource() = default;
    virtual void fillStatus(DeviceStatus& out) const = 0;
};

class SyncService {
public:
    enum class Result {
        Ok,
        NotFound,      ///< no such ride
        Conflict,      ///< ride is still being recorded, or CRC did not match
        BadRequest,    ///< offset past the end of the file, bad argument
        IoError,
    };

    struct Config {
        const char* deviceName = "ApexRide-01";

        /// Cached transfer handles are closed after this long without a read,
        /// so a phone that walks away does not pin a file open.
        uint32_t transferIdleTimeoutMs = 30000;

        /// Largest single chunk served. Bounds the transport's buffer.
        size_t maxChunkBytes = 8192;
    };

    SyncService(RideStorage& storage, const Clock& clock);
    ~SyncService();

    bool begin(const Config& config);

    void setStatusSource(const IDeviceStatusSource* source) { statusSource_ = source; }

    /// Releases an idle transfer handle. Call from the main loop.
    void update();

    // --- Queries ------------------------------------------------------------

    DeviceStatus deviceStatus() const;

    const Config& config() const { return config_; }

    RideStorage& storage() { return storage_; }
    const RideStorage& storage() const { return storage_; }

    // --- Transfer -----------------------------------------------------------

    /// Reads up to `maxLength` bytes of a ride's data file from `offset`.
    /// `readLength` receives the count actually read; 0 at end of file.
    Result readRideChunk(uint32_t rideId, uint32_t offset, uint8_t* out, size_t maxLength,
                         size_t& readLength);

    /// Verifies the phone's checksum against the device's own and, only on a
    /// match, marks the ride synced. This is the single place the flag is set.
    Result acknowledge(uint32_t rideId, uint32_t checksumFromPhone);

    /// Deletes a ride. Refuses unless it has been acknowledged.
    Result deleteRide(uint32_t rideId);

    /// Drops any cached transfer handle.
    void closeTransfer();

    uint32_t transfersServed() const { return chunksServed_; }

private:
    /// True when the ride is the one currently being written, or its summary
    /// says it was never closed. Such a file is still growing, so its size and
    /// CRC are not yet meaningful.
    bool isRideInFlight(uint32_t rideId, const RideSummary& summary) const;

    RideStorage& storage_;
    const Clock& clock_;
    Config       config_{};

    const IDeviceStatusSource* statusSource_ = nullptr;

    IRideFile* openFile_       = nullptr;
    uint32_t   openRideId_     = 0;
    uint32_t   openOffset_     = 0;
    uint32_t   lastActivityMs_ = 0;
    uint32_t   chunksServed_   = 0;
};

}  // namespace apex
