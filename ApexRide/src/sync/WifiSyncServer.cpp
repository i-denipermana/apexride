#include "WifiSyncServer.h"

#include "../core/Log.h"
#include "../ride/TelemetrySystem.h"

namespace apex {
namespace {

const char kDashboardHtml[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0b1018">
<title>ApexRide</title>
<style>
:root{color-scheme:dark;--bg:#080c12;--panel:#111925;--line:#263244;--muted:#8794a8;--text:#f4f7fb;--cyan:#2be0c3;--blue:#5da9ff;--warn:#ffb454;--bad:#ff647c}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 20% 0,#16263a 0,transparent 42%),var(--bg);color:var(--text);font:15px/1.45 system-ui,-apple-system,Segoe UI,sans-serif}
main{width:min(980px,100%);margin:auto;padding:20px 16px 48px}header{display:flex;align-items:center;justify-content:space-between;margin-bottom:18px}.brand{font-size:24px;font-weight:800;letter-spacing:.02em}.brand span{color:var(--cyan)}
.connection{display:flex;gap:8px;align-items:center;color:var(--muted);font-size:13px}.dot{width:9px;height:9px;border-radius:50%;background:var(--bad);box-shadow:0 0 14px currentColor}.dot.online{background:var(--cyan)}
.grid{display:grid;grid-template-columns:repeat(4,1fr);gap:12px}.card{background:linear-gradient(145deg,#141e2c,#0e151f);border:1px solid var(--line);border-radius:16px;padding:16px;min-height:128px;box-shadow:0 14px 35px #0005}.label{text-transform:uppercase;letter-spacing:.12em;color:var(--muted);font-size:11px}.value{font-size:35px;font-weight:750;margin-top:8px;font-variant-numeric:tabular-nums}.unit{font-size:14px;color:var(--muted);margin-left:3px}.sub{color:var(--muted);font-size:13px;margin-top:5px}.mapLink{display:inline-block;color:var(--blue);font-size:12px;margin-top:6px;text-decoration:none}.mapLink:hover{text-decoration:underline}
.leanCard{grid-column:span 2}.gauge{height:8px;background:#253142;border-radius:99px;margin-top:18px;position:relative}.gauge:before{content:"";position:absolute;left:50%;top:-4px;width:2px;height:16px;background:#66758a}.needle{position:absolute;left:50%;top:-5px;width:4px;height:18px;background:var(--cyan);border-radius:4px;box-shadow:0 0 12px var(--cyan);transform:translateX(-50%)}
.section{margin-top:22px}.sectionHead{display:flex;align-items:center;justify-content:space-between;margin-bottom:10px}.section h2{font-size:17px;margin:0}.pill{border:1px solid var(--line);border-radius:999px;padding:5px 10px;color:var(--muted);font-size:12px}.health{display:flex;gap:8px;flex-wrap:wrap}.health .pill strong{color:var(--text)}
.rides{display:grid;gap:8px}.ride{display:grid;grid-template-columns:1.1fr 1fr 1fr auto;align-items:center;gap:10px;background:var(--panel);border:1px solid var(--line);border-radius:13px;padding:12px 14px}.ride strong{font-variant-numeric:tabular-nums}.rideMeta{color:var(--muted);font-size:12px}.rideButtons{display:flex;gap:7px}.empty{padding:24px;text-align:center;color:var(--muted);border:1px dashed var(--line);border-radius:13px}
button{border:0;border-radius:9px;padding:9px 12px;background:#203149;color:var(--text);font-weight:650;cursor:pointer}button:hover{background:#2b4363}button:disabled{opacity:.45;cursor:not-allowed}.danger{background:#512536}.danger:hover{background:#6b2c42}.rideActions{display:flex;gap:8px;margin-top:12px}.rideActions button{flex:1}.rideActions .stop{background:#462637}.notice{display:none;margin-top:10px;color:var(--warn);font-size:13px}
@media(max-width:700px){.grid{grid-template-columns:repeat(2,1fr)}.leanCard{grid-column:span 2}.ride{grid-template-columns:1fr auto}.ride .optional{display:none}.value{font-size:30px}}
</style>
</head>
<body><main>
<header><div class="brand">Apex<span>Ride</span></div><div class="connection"><span id="dot" class="dot"></span><span id="connection">Connecting</span></div></header>
<div class="grid">
 <section class="card leanCard"><div class="label">Lean angle</div><div class="value"><span id="lean">--</span><span class="unit">deg</span></div><div class="gauge"><span id="needle" class="needle"></span></div><div id="leanSide" class="sub">Waiting for telemetry</div></section>
 <section class="card"><div class="label">Speed</div><div class="value"><span id="speed">--</span><span class="unit">km/h</span></div><div id="rawSpeed" class="sub">Raw --</div></section>
 <section class="card"><div class="label">GNSS</div><div id="gnss" class="value">--</div><div id="satellites" class="sub">No fix</div><div id="coordinates" class="sub">Coordinates unavailable</div><a id="mapLink" class="mapLink" target="_blank" rel="noopener" hidden>Open in Maps</a></section>
 <section class="card"><div class="label">Pitch</div><div class="value"><span id="pitch">--</span><span class="unit">deg</span></div><div class="sub">Positive nose up</div></section>
 <section class="card"><div class="label">Ride state</div><div id="state" class="value" style="font-size:25px">--</div><div id="recording" class="sub">Not recording</div><div class="rideActions"><button id="startRide" onclick="setRide('start')">Start Ride</button><button id="stopRide" class="stop" onclick="setRide('stop')" disabled>Stop Ride</button></div></section>
 <section class="card"><div class="label">Storage free</div><div class="value"><span id="free">--</span><span class="unit">MB</span></div><div id="ridesCount" class="sub">-- rides</div></section>
 <section class="card"><div class="label">Calibration</div><div id="calibration" class="value" style="font-size:25px">--</div><div class="sub">Startup IMU quality gate</div></section>
</div>
<section class="section"><div class="sectionHead"><h2>Sensor health</h2><span id="updated" class="pill">Never updated</span></div><div class="health"><span class="pill">IMU <strong id="imuRate">--</strong> Hz</span><span class="pill">I²C errors <strong id="imuErrors">--</strong></span><span class="pill">Dropped <strong id="dropped">--</strong></span><span class="pill">GNSS errors <strong id="gnssErrors">--</strong></span></div></section>
<section class="section"><div class="sectionHead"><h2>Stored rides</h2><button onclick="loadRides()">Refresh</button></div><div id="rides" class="rides"><div class="empty">Loading ride catalogue…</div></div><div id="notice" class="notice"></div></section>
</main>
<script>
const $=id=>document.getElementById(id);let lastSamples=0,lastAt=0,currentRecording=false;
const n=(v,d=1)=>Number(v||0).toFixed(d);const mb=v=>(Number(v||0)/1048576).toFixed(1);const rideName=id=>'R'+String(id).padStart(6,'0');
async function api(path,options){const r=await fetch(path,{cache:'no-store',...options});if(!r.ok)throw new Error((await r.text())||('HTTP '+r.status));return r}
async function refresh(){try{const s=await (await api('/status')).json(),t=Date.now(),x=s.telemetry,h=s.health,g=s.gnss;
 $('lean').textContent=n(x.leanDeg);$('pitch').textContent=n(x.pitchDeg);$('speed').textContent=n(x.speedKph);$('rawSpeed').textContent='Raw '+n(x.rawSpeedKph)+' km/h';
 const lean=Math.max(-60,Math.min(60,Number(x.leanDeg)));$('needle').style.left=(50+lean/1.2)+'%';$('leanSide').textContent=Math.abs(lean)<1?'Upright':(lean<0?'Left':'Right')+' '+Math.abs(lean).toFixed(1)+'°';
 $('gnss').textContent=g.fix?'FIX':'NO FIX';$('satellites').textContent=g.satellites+' satellites';const map=$('mapLink');if(g.fix){const lat=Number(g.latitude),lon=Number(g.longitude);$('coordinates').textContent=lat.toFixed(6)+', '+lon.toFixed(6);map.href='https://maps.google.com/?q='+encodeURIComponent(lat+','+lon);map.hidden=false}else{$('coordinates').textContent='Coordinates unavailable';map.hidden=true}$('state').textContent=x.state;$('calibration').textContent=h.calibration;
 $('free').textContent=mb(s.storage.freeBytes);$('ridesCount').textContent=s.rides+' rides · '+s.unsynced+' unsynced';currentRecording=s.recording;$('recording').textContent=s.recording?'Recording '+rideName(s.activeRide):'Not recording';$('startRide').disabled=s.recording;$('stopRide').disabled=!s.recording;
 $('imuErrors').textContent=h.imuErrors;$('dropped').textContent=h.droppedSamples;$('gnssErrors').textContent=g.errors;
 if(lastAt){$('imuRate').textContent=((h.imuSamples-lastSamples)/((t-lastAt)/1000)).toFixed(1)}lastSamples=h.imuSamples;lastAt=t;
 $('dot').classList.add('online');$('connection').textContent='Live';$('updated').textContent=new Date().toLocaleTimeString();
 }catch(e){$('dot').classList.remove('online');$('connection').textContent='Disconnected'}}
function duration(ms){let s=Math.round(Number(ms||0)/1000);if(s>86400)return 'legacy';return Math.floor(s/60)+'m '+s%60+'s'}
async function loadRides(){try{const d=await (await api('/rides')).json(),box=$('rides');box.innerHTML='';if(!d.rides.length){box.innerHTML='<div class="empty">No rides recorded yet</div>';return}
 d.rides.slice().reverse().forEach(r=>{const row=document.createElement('div');row.className='ride';const valid=r.summaryValid!==false;row.innerHTML='<div><strong>'+rideName(r.id)+'</strong><div class="rideMeta">'+(valid?duration(r.durationMs):'Summary unavailable')+'</div></div><div class="optional"><strong>'+(valid?(r.distanceCm/100000).toFixed(2)+' km':'--')+'</strong><div class="rideMeta">Distance</div></div><div class="optional"><strong>'+((r.bytes||0)/1024).toFixed(0)+' KB</strong><div class="rideMeta">'+(r.synced?'Synced':'Unsynced')+'</div></div><div class="rideButtons"><button class="download" '+(!valid?'disabled':'')+'>Download</button><button class="danger delete">Delete</button></div>';row.querySelector('.download').onclick=()=>downloadRide(r.id,r.bytes||0);row.querySelector('.delete').onclick=()=>deleteRide(r.id,!!r.synced);box.appendChild(row)})
 }catch(e){$('rides').innerHTML='<div class="empty">Could not load rides</div>'}}
async function setRide(action){const note=$('notice');$('startRide').disabled=true;$('stopRide').disabled=true;note.style.display='block';note.textContent=action==='start'?'Starting ride…':'Stopping ride…';try{await api('/ride/'+action,{method:'POST'});await refresh();await loadRides();note.textContent=action==='start'?'Ride recording started.':'Ride recording stopped and saved.'}catch(e){note.textContent='Ride control failed: '+e.message;await refresh()}}
async function deleteRide(id,synced){const name=rideName(id),note=$('notice');if(currentRecording){note.style.display='block';note.textContent='Stop the current ride before deleting stored rides.';return}const warning=synced?'Delete '+name+' permanently?':'WARNING: '+name+' is UNSYNCED. Deleting it permanently destroys the only verified-on-device copy. Continue?';if(!confirm(warning))return;if(!synced&&!confirm('Final confirmation: permanently delete unsynced '+name+'?'))return;note.style.display='block';note.textContent='Deleting '+name+'…';try{await api('/rides/'+name+'/delete'+(synced?'':'?force=1'),{method:'POST'});await refresh();await loadRides();note.textContent=name+' permanently deleted.'}catch(e){note.textContent='Delete failed: '+e.message}}
async function downloadRide(id,total){const note=$('notice');if(currentRecording){note.style.display='block';note.textContent='Stop the current ride before downloading.';return}note.style.display='block';note.textContent='Preparing '+rideName(id)+'…';
 try{await api('/sync/begin',{method:'POST'});const chunks=[];let offset=0;while(offset<total){const r=await api('/rides/'+rideName(id)+'/data?offset='+offset+'&length=8192');const b=await r.arrayBuffer();if(!b.byteLength)break;chunks.push(b);offset+=b.byteLength;note.textContent='Downloading '+Math.round(offset/Math.max(total,1)*100)+'%';}
 const url=URL.createObjectURL(new Blob(chunks,{type:'application/octet-stream'})),a=document.createElement('a');a.href=url;a.download=rideName(id)+'.bin';a.click();setTimeout(()=>URL.revokeObjectURL(url),5000);note.textContent=rideName(id)+' downloaded. It remains unsynced until a phone app verifies its CRC.';
 }catch(e){note.textContent='Download failed: '+e.message}finally{try{await api('/sync/end',{method:'POST'})}catch(e){}}}
refresh();loadRides();setInterval(refresh,500);setInterval(loadRides,10000);
</script></body></html>)HTML";

}  // namespace

WifiSyncServer::WifiSyncServer(SyncProtocol& protocol, SyncService& service,
                               TelemetrySystem& system)
    : protocol_(protocol), service_(service), system_(system) {}

bool WifiSyncServer::begin(const Config& config, char* jsonBuffer, size_t jsonBufferSize,
                           uint8_t* transferBuffer, size_t transferBufferSize) {
    if (config.ssid == nullptr || config.password == nullptr || strlen(config.ssid) == 0 ||
        strlen(config.password) < 8 || strlen(config.password) > 63 || jsonBuffer == nullptr ||
        jsonBufferSize < 256 || transferBuffer == nullptr || transferBufferSize == 0) {
        APEX_LOGE("Wi-Fi server configuration is invalid");
        return false;
    }

    config_ = config;
    jsonBuffer_ = jsonBuffer;
    jsonBufferSize_ = jsonBufferSize;
    transferBuffer_ = transferBuffer;
    transferBufferSize_ = transferBufferSize;
    registerHandlers();
    return start();
}

void WifiSyncServer::registerHandlers() {
    if (handlersRegistered_) return;

    server_.enableCORS(true);
    server_.on("/", HTTP_GET, [this]() { sendDashboard(); });
    server_.on("/index.html", HTTP_GET, [this]() { sendDashboard(); });
    server_.on("/generate_204", HTTP_ANY, [this]() { redirectDashboard(); });
    server_.on("/hotspot-detect.html", HTTP_ANY, [this]() { redirectDashboard(); });
    server_.on("/ncsi.txt", HTTP_ANY, [this]() { redirectDashboard(); });
    server_.on("/connecttest.txt", HTTP_ANY, [this]() { redirectDashboard(); });
    server_.onNotFound([this]() { handleApiRequest(); });
    handlersRegistered_ = true;
}

bool WifiSyncServer::start() {
    if (running_) return true;
    if (!handlersRegistered_ || jsonBuffer_ == nullptr || transferBuffer_ == nullptr) {
        APEX_LOGE("Wi-Fi dashboard was not initialized");
        return false;
    }

    WiFi.mode(WIFI_AP);
    // Favor the most broadly compatible 2.4 GHz AP settings. In particular,
    // some laptop adapters can see an ESP32-S3 beacon but time out during
    // association when power saving or wider channel modes are negotiated.
    WiFi.setSleep(false);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    const IPAddress address(192, 168, 4, 1);
    const IPAddress subnet(255, 255, 255, 0);
    if (!WiFi.softAPConfig(address, address, subnet) ||
        !WiFi.softAP(config_.ssid, config_.password, config_.channel, false,
                     config_.maxClients)) {
        APEX_LOGE("Could not start Wi-Fi access point '%s'", config_.ssid);
        WiFi.mode(WIFI_OFF);
        return false;
    }
    WiFi.softAPbandwidth(WIFI_BW_HT20);

    dns_.start(53, "*", address);
    server_.begin();
    running_ = true;
    lastClientCount_ = 0;
    APEX_LOGI("Wi-Fi dashboard: connect to '%s' on channel %u, then open "
              "http://192.168.4.1", config_.ssid,
              static_cast<unsigned>(config_.channel));
    return true;
}

void WifiSyncServer::stop() {
    if (!running_) return;
    server_.stop();
    dns_.stop();
    service_.endSession();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    running_ = false;
    lastClientCount_ = 0;
    APEX_LOGI("Wi-Fi dashboard stopped");
}

void WifiSyncServer::update() {
    if (!running_) return;
    dns_.processNextRequest();
    server_.handleClient();

    const uint8_t clients = clientCount();
    if (clients != lastClientCount_) {
        APEX_LOGI("Wi-Fi dashboard client count: %u", static_cast<unsigned>(clients));
        lastClientCount_ = clients;
    }
}

uint8_t WifiSyncServer::clientCount() const {
    return running_ ? static_cast<uint8_t>(WiFi.softAPgetStationNum()) : 0;
}

void WifiSyncServer::sendDashboard() {
    server_.sendHeader("Cache-Control", "no-store");
    server_.send_P(200, "text/html; charset=utf-8", kDashboardHtml);
}

void WifiSyncServer::redirectDashboard() {
    server_.sendHeader("Location", "http://192.168.4.1/", true);
    server_.send(302, "text/plain", "ApexRide dashboard");
}

String WifiSyncServer::buildQuery() const {
    String query;
    for (int i = 0; i < server_.args(); ++i) {
        if (i > 0) query += '&';
        query += server_.argName(i);
        query += '=';
        query += server_.arg(i);
    }
    return query;
}

void WifiSyncServer::handleApiRequest() {
    const HTTPMethod httpMethod = server_.method();
    const char* method = httpMethod == HTTP_GET ? "GET" :
                         httpMethod == HTTP_POST ? "POST" : nullptr;
    if (method == nullptr) {
        sendError(405, "method not allowed");
        return;
    }

    const String uri = server_.uri();
    if (httpMethod == HTTP_POST && uri == "/ride/start") {
        handleRideControl(true);
        return;
    }
    if (httpMethod == HTTP_POST && uri == "/ride/stop") {
        handleRideControl(false);
        return;
    }

    const DeviceStatus live = service_.deviceStatus();
    const bool rideMutation = httpMethod == HTTP_POST && uri.startsWith("/rides/");
    const bool rideDownload = httpMethod == HTTP_GET && uri.startsWith("/rides/") &&
                              uri.endsWith("/data");
    if (live.recording && (rideMutation || rideDownload)) {
        sendError(409, "stop the current ride before transferring ride data");
        return;
    }

    const String query = buildQuery();
    const SyncResponse response = protocol_.route(method, uri.c_str(),
                                                   query.length() ? query.c_str() : nullptr,
                                                   jsonBuffer_, jsonBufferSize_);
    server_.sendHeader("Cache-Control", "no-store");

    if (!response.isRideData) {
        sendBody(response.status, response.contentType, response.body, response.bodyLength);
        return;
    }

    if (response.dataLength > transferBufferSize_) {
        sendError(500, "transfer buffer too small");
        return;
    }

    size_t readLength = 0;
    const SyncService::Result result = service_.readRideChunk(
        response.rideId, response.dataOffset, transferBuffer_, response.dataLength, readLength);
    if (result != SyncService::Result::Ok) {
        sendError(result == SyncService::Result::NotFound ? 404 :
                  result == SyncService::Result::Conflict ? 409 : 500,
                  "could not read ride data");
        return;
    }

    sendBody(200, response.contentType, reinterpret_cast<const char*>(transferBuffer_), readLength);
}

void WifiSyncServer::handleRideControl(bool start) {
    const bool success = start ? system_.startRideManually() : system_.stopRideManually();
    if (!success) {
        sendError(409, start ? "could not start ride" : "no ride is recording");
        return;
    }

    const DeviceStatus status = service_.deviceStatus();
    const int written = snprintf(jsonBuffer_, jsonBufferSize_,
                                 "{\"ok\":true,\"recording\":%s,\"activeRide\":%u}",
                                 status.recording ? "true" : "false",
                                 static_cast<unsigned>(status.activeRideId));
    sendBody(200, "application/json", jsonBuffer_,
             written > 0 ? static_cast<size_t>(written) : 0);
}

void WifiSyncServer::sendBody(uint16_t status, const char* contentType, const char* body,
                              size_t length) {
    server_.setContentLength(length);
    server_.send(status, contentType, "");
    if (length > 0) server_.sendContent(body, length);
}

void WifiSyncServer::sendError(uint16_t status, const char* message) {
    const int written = snprintf(jsonBuffer_, jsonBufferSize_, "{\"error\":\"%s\"}", message);
    const size_t length = written > 0 ? static_cast<size_t>(written) : 0;
    sendBody(status, "application/json", jsonBuffer_, length);
}

}  // namespace apex
