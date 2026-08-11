//
// syncserver — serves the ApexRide sync API over HTTP from a directory of ride
// files, on your development machine.
//
//   make -C tools
//   tools/build/syncserver --dir tests/build/testdata/ride --port 8080
//   curl localhost:8080/rides
//
// Why this exists: the phone app needs a device to talk to, and the device does
// not exist yet. This runs the real SyncProtocol and SyncService over real
// sockets, so the app can be written and debugged against the actual API today.
// When the ESP32 arrives, its Wi-Fi handler is the same adapter over a
// different socket layer — everything above route() is already proven.
//
// Development tool: plain HTTP, no authentication, binds to localhost by
// default. Not a model for anything exposed to a network.
//

#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <string>
#include <vector>

#include "../ApexRide/src/core/Clock.h"
#include "../ApexRide/src/core/Log.h"
#include "../ApexRide/src/sync/SyncProtocol.h"
#include "../ApexRide/src/sync/SyncService.h"
#include "../hostfs/HostRideStore.h"

using namespace apex;

namespace {

volatile sig_atomic_t g_running = 1;

void onInterrupt(int) {
    g_running = 0;
}

void logToStdout(LogLevel level, const char* line) {
    static const char* kPrefix[] = {"E", "W", "I", "D"};
    printf("[%s] %s\n", kPrefix[static_cast<int>(level)], line);
}

bool sendAll(int socketFd, const void* data, size_t length) {
    const char* cursor = static_cast<const char*>(data);
    while (length > 0) {
        const ssize_t sent = send(socketFd, cursor, length, 0);
        if (sent <= 0) return false;
        cursor += sent;
        length -= static_cast<size_t>(sent);
    }
    return true;
}

const char* reasonPhrase(uint16_t status) {
    switch (status) {
        case 200: return "OK";
        case 400: return "Bad Request";
        case 404: return "Not Found";
        case 409: return "Conflict";
        case 500: return "Internal Server Error";
        default:  return "Unknown";
    }
}

bool sendHeaders(int socketFd, uint16_t status, const char* contentType, size_t contentLength) {
    char header[512];
    const int written = snprintf(header, sizeof(header),
                                 "HTTP/1.1 %u %s\r\n"
                                 "Content-Type: %s\r\n"
                                 "Content-Length: %zu\r\n"
                                 // Lets a browser-hosted Flutter build call this
                                 // during development.
                                 "Access-Control-Allow-Origin: *\r\n"
                                 "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
                                 "Connection: close\r\n"
                                 "\r\n",
                                 status, reasonPhrase(status), contentType, contentLength);
    return written > 0 && sendAll(socketFd, header, static_cast<size_t>(written));
}

/// Reads request bytes until the end of the headers.
bool readRequest(int socketFd, std::string& out) {
    char buffer[2048];
    for (;;) {
        const ssize_t got = recv(socketFd, buffer, sizeof(buffer), 0);
        if (got <= 0) return !out.empty();

        out.append(buffer, static_cast<size_t>(got));
        if (out.find("\r\n\r\n") != std::string::npos) return true;
        if (out.size() > 16384) return false;  // header flood
    }
}

void handleConnection(int socketFd, SyncProtocol& protocol, SyncService& service,
                      std::vector<char>& jsonBuffer) {
    std::string request;
    if (!readRequest(socketFd, request)) {
        return;
    }

    // "GET /rides/R000001/data?offset=0 HTTP/1.1"
    const size_t lineEnd = request.find("\r\n");
    std::string  line    = request.substr(0, lineEnd);

    const size_t firstSpace  = line.find(' ');
    const size_t secondSpace = line.find(' ', firstSpace + 1);
    if (firstSpace == std::string::npos || secondSpace == std::string::npos) {
        sendHeaders(socketFd, 400, "application/json", 0);
        return;
    }

    const std::string method = line.substr(0, firstSpace);
    std::string       target = line.substr(firstSpace + 1, secondSpace - firstSpace - 1);

    std::string query;
    const size_t questionMark = target.find('?');
    if (questionMark != std::string::npos) {
        query  = target.substr(questionMark + 1);
        target = target.substr(0, questionMark);
    }

    if (method == "OPTIONS") {  // CORS preflight
        sendHeaders(socketFd, 200, "text/plain", 0);
        return;
    }

    const SyncResponse response = protocol.route(method.c_str(), target.c_str(),
                                                 query.empty() ? nullptr : query.c_str(),
                                                 jsonBuffer.data(), jsonBuffer.size());

    printf("  %-4s %-40s -> %u\n", method.c_str(), target.c_str(), response.status);

    if (!response.isRideData) {
        if (sendHeaders(socketFd, response.status, response.contentType, response.bodyLength)) {
            sendAll(socketFd, response.body, response.bodyLength);
        }
        return;
    }

    // Ride bytes are streamed straight off the filesystem rather than buffered,
    // exactly as the ESP32 must do — a ride is far larger than its RAM.
    if (!sendHeaders(socketFd, response.status, response.contentType, response.dataLength)) {
        return;
    }

    uint8_t  chunk[4096];
    uint32_t offset    = response.dataOffset;
    uint32_t remaining = response.dataLength;

    while (remaining > 0) {
        const size_t want = remaining < sizeof(chunk) ? remaining : sizeof(chunk);
        size_t       got  = 0;

        if (service.readRideChunk(response.rideId, offset, chunk, want, got) !=
                SyncService::Result::Ok ||
            got == 0) {
            break;
        }

        if (!sendAll(socketFd, chunk, got)) break;

        offset += static_cast<uint32_t>(got);
        remaining -= static_cast<uint32_t>(got);
    }
}

void printUsage() {
    fprintf(stderr,
            "usage: syncserver [--dir <path>] [--port <n>] [--capacity-mb <n>]\n"
            "\n"
            "  --dir          directory containing a rides/ subdirectory\n"
            "                 (default: tests/build/testdata/ride)\n"
            "  --port         TCP port to listen on (default 8080)\n"
            "  --capacity-mb  emulated flash size, for the storage figures\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::string root       = "tests/build/testdata/ride";
    int         port       = 8080;
    uint64_t    capacityMb = 13;

    for (int i = 1; i < argc; ++i) {
        const std::string flag = argv[i];
        if (flag == "--dir" && i + 1 < argc) {
            root = argv[++i];
        } else if (flag == "--port" && i + 1 < argc) {
            port = atoi(argv[++i]);
        } else if (flag == "--capacity-mb" && i + 1 < argc) {
            capacityMb = static_cast<uint64_t>(atoll(argv[++i]));
        } else {
            printUsage();
            return 2;
        }
    }

    // A client that disconnects mid-download must not kill the process.
    signal(SIGPIPE, SIG_IGN);
    signal(SIGINT, onInterrupt);

    setLogSink(logToStdout);
    setLogLevel(LogLevel::Info);

    HostRideStore store(root, capacityMb * 1024u * 1024u);
    if (!store.begin()) {
        fprintf(stderr, "syncserver: cannot open %s\n", root.c_str());
        return 1;
    }

    SystemClock         clock;
    RideStorage         storage;
    RideStorage::Config storageConfig;
    if (!storage.begin(store, storageConfig)) {
        fprintf(stderr, "syncserver: no rides directory under %s\n", root.c_str());
        return 1;
    }

    SyncService service(storage, clock);
    service.begin(SyncService::Config());
    SyncProtocol protocol(service);

    std::vector<char> jsonBuffer(SyncProtocol::manifestBufferSize(RideStorage::kMaxRides));

    const int listenFd = socket(AF_INET, SOCK_STREAM, 0);
    if (listenFd < 0) {
        perror("socket");
        return 1;
    }

    int reuse = 1;
    setsockopt(listenFd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address{};
    address.sin_family      = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port        = htons(static_cast<uint16_t>(port));

    if (bind(listenFd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        perror("bind");
        close(listenFd);
        return 1;
    }

    if (listen(listenFd, 8) < 0) {
        perror("listen");
        close(listenFd);
        return 1;
    }

    printf("\nApexRide sync server\n");
    printf("  serving  %s  (%u ride(s), %u unsynced)\n", root.c_str(),
           static_cast<unsigned>(storage.rideCount()),
           static_cast<unsigned>(storage.unsyncedCount()));
    printf("  listening on http://localhost:%d\n\n", port);
    printf("  curl localhost:%d/status\n", port);
    printf("  curl localhost:%d/rides\n", port);
    printf("  curl localhost:%d/rides/R000001/data?offset=0'&'length=4096 --output chunk.bin\n\n",
           port);

    while (g_running) {
        const int clientFd = accept(listenFd, nullptr, nullptr);
        if (clientFd < 0) {
            if (g_running) perror("accept");
            continue;
        }

        // The device serves one phone at a time, so a single-threaded loop is
        // an accurate model as well as a simple one.
        handleConnection(clientFd, protocol, service, jsonBuffer);
        close(clientFd);

        service.update();
    }

    printf("\nshutting down\n");
    close(listenFd);
    return 0;
}
