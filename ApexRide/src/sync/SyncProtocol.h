#pragma once
//
// The wire protocol: request routing and JSON encoding.
//
// Kept separate from SyncService, and free of any networking code, so the
// whole request/response surface the phone app talks to can be driven from
// host tests. The ESP32 Wi-Fi server is then a thin adapter that forwards a
// method, path and query string into route() and writes back what it returns.
//
// ---------------------------------------------------------------------------
// Endpoints
//
//   POST /sync/begin                      open a session; keeps the radio up
//   GET  /sync/pending                    what the phone still needs, newest first
//   POST /sync/end                        close the session; radio may drop
//   GET  /status                          device, storage and GNSS state
//   GET  /rides                           manifest of every stored ride
//   GET  /rides/R000001                   one ride's summary
//   GET  /rides/R000001/data              ride bytes; ?offset= &length=
//   POST /rides/R000001/ack?crc=<hex>     verify checksum, then mark synced
//   POST /rides/R000001/delete            delete, only if already synced
//
// Ride ids are the same R%06u strings used for filenames, so a path can be
// read straight off a directory listing.
//
// Status codes follow HTTP conventions:
//   200 ok · 400 bad request · 404 unknown ride · 409 still recording, or the
//   phone's checksum did not match · 500 filesystem error
//

#include <stddef.h>
#include <stdint.h>

#include "SyncService.h"

namespace apex {

struct SyncResponse {
    uint16_t    status      = 200;
    const char* contentType = "application/json";

    /// JSON body, written into the caller's buffer.
    const char* body       = "";
    size_t      bodyLength = 0;

    /// When set, the body is not `body` but raw ride bytes that the transport
    /// must pull with SyncService::readRideChunk. Keeps whole rides out of RAM.
    bool     isRideData = false;
    uint32_t rideId     = 0;
    uint32_t dataOffset = 0;
    uint32_t dataLength = 0;
};

class SyncProtocol {
public:
    explicit SyncProtocol(SyncService& service) : service_(service) {}

    /// Handles one request. `buffer` receives any JSON body and must stay
    /// valid for as long as the response is used.
    ///
    /// `query` may be null. Neither path nor query is modified.
    SyncResponse route(const char* method, const char* path, const char* query, char* buffer,
                       size_t bufferSize);

    /// Bytes needed for a ride manifest covering `rideCount` rides. The ESP32
    /// allocates this from PSRAM.
    static size_t manifestBufferSize(size_t rideCount);

private:
    SyncResponse writeStatus(char* buffer, size_t bufferSize);
    SyncResponse writePending(char* buffer, size_t bufferSize);
    SyncResponse writeSessionState(char* buffer, size_t bufferSize);
    SyncResponse writeRideList(char* buffer, size_t bufferSize);
    SyncResponse writeRideSummary(uint32_t rideId, char* buffer, size_t bufferSize);

    SyncService& service_;
};

// --- Helpers, exposed for testing -------------------------------------------

/// Parses "R000021" into 21. Returns false for anything else.
bool parseRideId(const char* text, uint32_t& rideIdOut);

/// Reads an unsigned decimal parameter out of a query string.
bool queryUInt(const char* query, const char* key, uint32_t& valueOut);

/// Reads a hexadecimal parameter, with or without a 0x prefix.
bool queryHex(const char* query, const char* key, uint32_t& valueOut);

}  // namespace apex
