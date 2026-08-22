#pragma once
//
// Local Wi-Fi transport and browser dashboard for ApexRide.
//
// The ESP32 runs as an access point, so viewing and downloading rides needs no
// internet connection and no phone app. HTTP remains a thin adapter over
// SyncProtocol: storage safety, CRC acknowledgement and recording conflicts
// stay in the transport-independent service where host tests cover them.
//

#include <DNSServer.h>
#include <WebServer.h>
#include <WiFi.h>

#include "SyncProtocol.h"

namespace apex {

class TelemetrySystem;

class WifiSyncServer {
public:
    struct Config {
        const char* ssid = "ApexRide-01";
        const char* password = "apexride01";
        uint8_t channel = 6;
        uint8_t maxClients = 2;
    };

    WifiSyncServer(SyncProtocol& protocol, SyncService& service, TelemetrySystem& system);

    /// `jsonBuffer` and `transferBuffer` must remain valid while the server is
    /// running. Both are allocated from PSRAM by the sketch.
    bool begin(const Config& config, char* jsonBuffer, size_t jsonBufferSize,
               uint8_t* transferBuffer, size_t transferBufferSize);

    bool start();
    void stop();
    void update();

    bool running() const { return running_; }
    uint8_t clientCount() const;
    IPAddress address() const { return running_ ? WiFi.softAPIP() : IPAddress(); }

private:
    void registerHandlers();
    void sendDashboard();
    void redirectDashboard();
    void handleApiRequest();
    void handleRideControl(bool start);
    void sendBody(uint16_t status, const char* contentType, const char* body, size_t length);
    void sendError(uint16_t status, const char* message);
    String buildQuery() const;

    SyncProtocol& protocol_;
    SyncService&  service_;
    TelemetrySystem& system_;
    WebServer     server_{80};
    DNSServer     dns_;

    Config config_{};
    char* jsonBuffer_ = nullptr;
    size_t jsonBufferSize_ = 0;
    uint8_t* transferBuffer_ = nullptr;
    size_t transferBufferSize_ = 0;

    bool handlersRegistered_ = false;
    bool running_ = false;
    uint8_t lastClientCount_ = 0;
};

}  // namespace apex
