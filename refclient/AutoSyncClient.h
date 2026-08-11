#pragma once
//
// Reference implementation of the phone's auto-sync loop.
//
// This is the algorithm the Flutter app has to implement. Writing it once, in
// testable C++, means the awkward parts — resume, retry, what to do when a
// checksum fails — are settled and covered by tests rather than rediscovered
// in Dart against hardware.
//
// ---------------------------------------------------------------------------
// The loop
//
//   POST /sync/begin            refused while recording
//   GET  /sync/pending          newest first
//   for each ride:
//       resume from whatever is already held locally
//       GET /rides/{id}/data?offset=&length=   until complete
//       checksum what was received
//       POST /rides/{id}/ack?crc=              device verifies and marks synced
//       on rejection: discard locally and start that ride again
//   POST /sync/end              device may now drop the radio
//
// A rejected checksum means the local copy is wrong, so it is thrown away
// rather than resumed — resuming would preserve the corruption. A dropped
// connection is different: the bytes already held are good, so it resumes.
//
// Rides that fail every attempt are skipped for the rest of the session, or
// pending would return them forever and the loop would never finish.
//

#include <stddef.h>
#include <stdint.h>

#include <vector>

namespace apex {

/// How requests reach the device. Returns an HTTP-style status code, or a
/// negative value if the request could not be made at all.
class ISyncTransport {
public:
    virtual ~ISyncTransport() = default;

    virtual int request(const char* method, const char* path, const char* query,
                        std::vector<uint8_t>& responseBody) = 0;
};

/// Where downloaded rides are kept. On a phone this is the filesystem; the
/// tests use memory.
class ISyncSink {
public:
    virtual ~ISyncSink() = default;

    /// Bytes already held for this ride, which is where a transfer resumes.
    virtual uint32_t bytesHeld(uint32_t rideId) = 0;

    virtual bool append(uint32_t rideId, const uint8_t* data, size_t length) = 0;

    /// Throws away everything held for a ride, after a failed verification.
    virtual bool discard(uint32_t rideId) = 0;

    /// CRC-32 of what is held — computed by the phone, over its own copy.
    /// This is the value the device checks, so it must not come from the device.
    virtual uint32_t checksum(uint32_t rideId) = 0;
};

class AutoSyncClient {
public:
    struct Config {
        uint32_t chunkBytes = 8192;

        /// Attempts per ride before it is left for the next session.
        int attemptsPerRide = 3;

        /// Safety net: stop after this many rides in one session.
        uint32_t maxRidesPerSession = 64;
    };

    struct Report {
        uint32_t ridesSynced      = 0;
        uint32_t ridesFailed      = 0;
        uint64_t bytesTransferred = 0;
        uint32_t retries          = 0;
        bool     sessionRefused   = false;
        bool     transportFailed  = false;
    };

    AutoSyncClient(ISyncTransport& transport, ISyncSink& sink)
        : transport_(transport), sink_(sink) {}

    /// Runs one complete auto-sync pass. Safe to call again; anything left
    /// outstanding is picked up next time.
    Report run(const Config& config);

    // Separate overload rather than a default argument: a default argument
    // referring to a nested type's own member initializers is not valid inside
    // the enclosing class definition, but a member function body is.
    Report run() { return run(Config()); }

private:
    struct PendingRide {
        uint32_t rideId = 0;
        uint32_t bytes  = 0;
        uint32_t crc32  = 0;
    };

    bool fetchPending(std::vector<PendingRide>& out);

    /// Downloads and verifies one ride. Returns true once the device has
    /// acknowledged it.
    bool syncRide(const PendingRide& ride, const Config& config, Report& report);

    ISyncTransport& transport_;
    ISyncSink&      sink_;
};

// --- Minimal JSON field extraction ------------------------------------------
//
// Purpose-built for the handful of shapes this protocol emits. A real app uses
// its platform's JSON parser; this exists so the reference client has no
// dependencies.

bool jsonUInt(const char* json, const char* key, uint32_t& valueOut, const char** afterOut = nullptr);
bool jsonHexString(const char* json, const char* key, uint32_t& valueOut,
                   const char** afterOut = nullptr);

}  // namespace apex
