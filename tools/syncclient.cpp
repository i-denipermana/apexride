//
// syncclient — runs the auto-sync loop against a device over HTTP.
//
//   tools/build/syncserver --dir tests/build/testdata/ride &
//   tools/build/syncclient --host localhost --port 8080 --out ~/rides
//
// Same AutoSyncClient the tests drive in-process, here over real sockets and
// writing real files. This is what the Flutter app has to do on connect.
//

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <string>
#include <vector>

#include "../ApexRide/src/core/Crc32.h"
#include "../refclient/AutoSyncClient.h"

using namespace apex;

namespace {

/// One request per connection. Crude, but it matches how a constrained device
/// server behaves and keeps the client obviously correct.
class HttpTransport : public ISyncTransport {
public:
    HttpTransport(std::string host, int port) : host_(std::move(host)), port_(port) {}

    int request(const char* method, const char* path, const char* query,
                std::vector<uint8_t>& responseBody) override {
        responseBody.clear();

        const int socketFd = connectToDevice();
        if (socketFd < 0) return -1;

        char header[512];
        const int headerLength = snprintf(header, sizeof(header),
                                          "%s %s%s%s HTTP/1.1\r\n"
                                          "Host: %s\r\n"
                                          "Connection: close\r\n"
                                          "Content-Length: 0\r\n"
                                          "\r\n",
                                          method, path, query != nullptr ? "?" : "",
                                          query != nullptr ? query : "", host_.c_str());

        if (send(socketFd, header, static_cast<size_t>(headerLength), 0) != headerLength) {
            close(socketFd);
            return -1;
        }

        std::string raw;
        char        buffer[8192];
        for (;;) {
            const ssize_t got = recv(socketFd, buffer, sizeof(buffer), 0);
            if (got <= 0) break;
            raw.append(buffer, static_cast<size_t>(got));
        }
        close(socketFd);

        const size_t headerEnd = raw.find("\r\n\r\n");
        if (raw.size() < 12 || headerEnd == std::string::npos) return -1;

        const int status = atoi(raw.c_str() + 9);

        const char* bodyStart = raw.data() + headerEnd + 4;
        const size_t bodyLength = raw.size() - headerEnd - 4;
        responseBody.assign(bodyStart, bodyStart + bodyLength);

        return status;
    }

private:
    int connectToDevice() {
        const int socketFd = socket(AF_INET, SOCK_STREAM, 0);
        if (socketFd < 0) return -1;

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port   = htons(static_cast<uint16_t>(port_));

        if (inet_pton(AF_INET, host_.c_str(), &address.sin_addr) != 1) {
            const hostent* entry = gethostbyname(host_.c_str());
            if (entry == nullptr) {
                close(socketFd);
                return -1;
            }
            memcpy(&address.sin_addr, entry->h_addr, static_cast<size_t>(entry->h_length));
        }

        if (connect(socketFd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
            close(socketFd);
            return -1;
        }
        return socketFd;
    }

    std::string host_;
    int         port_;
};

/// Writes rides into a directory, resuming from whatever is already there.
class FileSink : public ISyncSink {
public:
    explicit FileSink(std::string directory) : directory_(std::move(directory)) {
        mkdir(directory_.c_str(), 0755);
    }

    uint32_t bytesHeld(uint32_t rideId) override {
        struct stat info {};
        if (stat(pathFor(rideId).c_str(), &info) != 0) return 0;
        return static_cast<uint32_t>(info.st_size);
    }

    bool append(uint32_t rideId, const uint8_t* data, size_t length) override {
        FILE* handle = fopen(pathFor(rideId).c_str(), "ab");
        if (handle == nullptr) return false;
        const size_t written = fwrite(data, 1, length, handle);
        fclose(handle);
        return written == length;
    }

    bool discard(uint32_t rideId) override {
        printf("  discarding local copy of R%06u and starting again\n", rideId);
        return ::remove(pathFor(rideId).c_str()) == 0;
    }

    uint32_t checksum(uint32_t rideId) override {
        FILE* handle = fopen(pathFor(rideId).c_str(), "rb");
        if (handle == nullptr) return 0;

        Crc32   crc;
        uint8_t buffer[8192];
        for (;;) {
            const size_t got = fread(buffer, 1, sizeof(buffer), handle);
            if (got == 0) break;
            crc.update(buffer, got);
        }
        fclose(handle);
        return crc.value();
    }

    std::string pathFor(uint32_t rideId) const {
        char name[32];
        snprintf(name, sizeof(name), "/R%06u.bin", rideId);
        return directory_ + name;
    }

private:
    std::string directory_;
};

}  // namespace

int main(int argc, char** argv) {
    std::string host      = "localhost";
    int         port      = 8080;
    std::string outputDir = "./synced-rides";

    for (int i = 1; i < argc; ++i) {
        const std::string flag = argv[i];
        if (flag == "--host" && i + 1 < argc) {
            host = argv[++i];
        } else if (flag == "--port" && i + 1 < argc) {
            port = atoi(argv[++i]);
        } else if (flag == "--out" && i + 1 < argc) {
            outputDir = argv[++i];
        } else {
            fprintf(stderr, "usage: syncclient [--host h] [--port n] [--out dir]\n");
            return 2;
        }
    }

    signal(SIGPIPE, SIG_IGN);

    printf("\nauto-sync: %s:%d -> %s\n\n", host.c_str(), port, outputDir.c_str());

    HttpTransport transport(host, port);
    FileSink      sink(outputDir);

    AutoSyncClient               client(transport, sink);
    const AutoSyncClient::Report report = client.run();

    if (report.transportFailed && report.ridesSynced == 0) {
        printf("could not reach the device at %s:%d\n\n", host.c_str(), port);
        return 1;
    }

    if (report.sessionRefused) {
        printf("device is recording a ride; sync deferred\n\n");
        return 0;
    }

    // Printed in KB, not rounded MB: the difference between a full transfer
    // and a resumed one is exactly what you want to see here.
    printf("  synced   %u ride(s), %llu KB transferred\n", report.ridesSynced,
           static_cast<unsigned long long>(report.bytesTransferred / 1024));
    if (report.retries > 0) printf("  retries  %u\n", report.retries);
    if (report.ridesFailed > 0) {
        printf("  FAILED   %u ride(s) — left on the device for the next attempt\n",
               report.ridesFailed);
    }
    printf("\n");

    return report.ridesFailed == 0 ? 0 : 1;
}
