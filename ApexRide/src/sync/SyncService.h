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

    /// A sync session bounds the window in which the device keeps its radio up.
    ///
    /// Without one, auto-sync would either leave Wi-Fi on permanently — which a
    /// 1500 mAh cell will not tolerate — or tear it down between rides and pay
    /// the reconnect cost every time.
    struct Session {
        bool     active            = false;
        uint32_t startedMs         = 0;
        uint32_t lastActivityMs    = 0;
        uint32_t ridesAcknowledged = 0;
        uint64_t bytesServed       = 0;
    };

    struct Config {
        const char* deviceName = "ApexRide-01";

        /// A session with no traffic for this long is closed, so a phone that
        /// goes out of range does not pin the radio on.
        uint32_t sessionIdleTimeoutMs = 60000;

        /// Refuse to start a session while a ride is being recorded. Serving
        /// bulk data competes with the recorder for flash and CPU, and dropping
        /// samples to make a sync faster is the wrong trade.
        bool refuseSyncWhileRecording = true;

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

    /// Releases idle transfer handles and closes idle sessions. Call from the
    /// main loop.
    void update();

    // --- Session ------------------------------------------------------------

    /// Opens a sync session. Refused while recording unless `force`.
    Result beginSession(bool force = false);

    /// Closes the session and releases any transfer handle. Safe to call when
    /// no session is open.
    void endSession();

    const Session& session() const { return session_; }

    /// Rides the phone still needs, newest first — a rider who just parked
    /// wants the ride they just did, and nothing is at risk of deletion in the
    /// meantime because unsynced rides are never reclaimed.
    size_t pendingRideCount() const;
    bool   pendingRideAt(size_t index, uint32_t& rideIdOut) const;
    uint64_t pendingBytes() const;

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

    void touchSession();

    Session session_{};

    IRideFile* openFile_       = nullptr;
    uint32_t   openRideId_     = 0;
    uint32_t   openOffset_     = 0;
    uint32_t   lastActivityMs_ = 0;
    uint32_t   chunksServed_   = 0;
};

}  // namespace apex
