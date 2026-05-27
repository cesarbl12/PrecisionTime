/*
 *  RECEPTOR SILBATO ARBITRO - v3.1 WebServer + LoRa (Modo AP)
 *  Dependencias (Library Manager):
 *    - LoRa       by Sandeep Mistry
 *    - WebSockets by Markus Sattler
 */

#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// ==================== PINES LORA ====================
#define LORA_SCK   12
#define LORA_MISO  13
#define LORA_MOSI  14
#define LORA_CS    15
#define LORA_RST   16
#define LORA_DIO0  17
#define LORA_FREQ  915E6

// ==================== SERVIDORES ====================
WebServer        httpServer(80);
WebSocketsServer wsServer(81);

// ==================== ESTADO ARBITROS ====================
struct RefereeState {
  bool          online;
  String        status;
  int           battery;
  unsigned long lastSeen;
};

RefereeState referees[3] = {
  {false, "idle", -1, 0},
  {false, "idle", -1, 0},
  {false, "idle", -1, 0}
};

// ==================== PRESENCIA / PING ====================
#define PRESENCE_TIMEOUT_MS  (2UL * 60 * 1000)
#define PING_INTERVAL_MS     (30UL * 60 * 1000)
unsigned long lastPingTime = 0;

// ==================== PROTOTIPOS ====================
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
void sendFullState(uint8_t clientNum);
void wsBroadcast(String json);
void handleWsCommand(String msg);
void sendLoRaCmd(String payload);
void checkLoRa();
void checkPresenceTimeout();
void checkPingInterval();
void handleSerialCommand(String input);
void printHelp();
int  refereeIndex(String id);
String refereeLoRaID(int idx);

// ==================== HELPERS ====================
int refereeIndex(String id) {
  if (id == "CREW_CHIEF") return 0;
  if (id == "JUDGE_1")    return 1;
  if (id == "JUDGE_2")    return 2;
  return -1;
}

String refereeLoRaID(int idx) {
  if (idx == 0) return "CREW_CHIEF";
  if (idx == 1) return "JUDGE_1";
  if (idx == 2) return "JUDGE_2";
  return "";
}

// ==================== HTML (dividido en 3 partes) ====================
// Parte 1: todo hasta antes del <script>
const char HTML_P1[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
  <title>Referee Panel</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { background-color: #0d0d0d; font-family: 'Segoe UI', sans-serif;
      display: flex; flex-direction: column; align-items: center;
      justify-content: center; min-height: 100vh; gap: 32px; padding: 24px; }
    h1 { color: #fff; font-size: 2rem; letter-spacing: 5px; text-transform: uppercase; opacity: .85; }
    .cards-container { display: flex; gap: 28px; flex-wrap: wrap; justify-content: center; }
    .card { background-color: #1a1a1a; border: 3px solid #2e2e2e; border-radius: 20px;
      width: 240px; padding: 40px 28px 28px; display: flex; flex-direction: column;
      align-items: center; gap: 18px; transition: all 0.4s ease; position: relative; }
    .card.status-standby { border-color: #f0a500; box-shadow: 0 0 30px rgba(240,165,0,.4); }
    .card.status-running  { border-color: #00ff00; box-shadow: 0 0 30px rgba(0,255,0,.4); }
    .card.status-stop     { border-color: #ff0000; box-shadow: 0 0 30px rgba(255,0,0,.4); }
    .card.status-whistle  { border-color: #007bff; box-shadow: 0 0 30px rgba(0,123,255,.4); }
    .presence-dot { position: absolute; top: 14px; right: 14px; width: 13px; height: 13px;
      border-radius: 50%; background: #333; border: 2px solid #555;
      transition: background .4s, box-shadow .4s; }
    .presence-dot.online  { background: #00e676; box-shadow: 0 0 8px rgba(0,230,118,.8);
      animation: pulse-green 2s infinite; }
    .presence-dot.offline { background: #555; box-shadow: none; }
    @keyframes pulse-green {
      0%,100% { box-shadow: 0 0 6px rgba(0,230,118,.7); }
      50%      { box-shadow: 0 0 14px rgba(0,230,118,1); } }
    .card svg { width: 110px; height: 110px; opacity: .9; }
    .role { color: #fff; font-size: 1rem; font-weight: 600; letter-spacing: 2px; text-transform: uppercase; }
    .battery-container { width: 100%; display: flex; flex-direction: column; align-items: center; gap: 6px; }
    .battery-label { color: #888; font-size: .72rem; letter-spacing: 1px; text-transform: uppercase; }
    .battery-bar-wrap { width: 100%; height: 10px; background: #2e2e2e; border-radius: 6px; overflow: hidden; }
    .battery-bar { height: 100%; border-radius: 6px; background: #00e676; transition: width .6s ease, background .4s; }
    .battery-bar.medium { background: #f0a500; }
    .battery-bar.low    { background: #ff3d3d; }
    .battery-pct { color: #aaa; font-size: .8rem; font-weight: 600; }
    .battery-unknown { color: #555; font-size: .75rem; font-style: italic; }
    .btn-calibrate { background: transparent; border: 1px solid #444; color: #888;
      padding: 12px 28px; border-radius: 10px; font-size: .85rem; font-weight: 600;
      letter-spacing: 1.5px; text-transform: uppercase; cursor: pointer; transition: all .2s; width: 100%; }
    .btn-calibrate:hover { border-color: #f0a500; color: #f0a500; }
    .modal-overlay { display: none; position: fixed; inset: 0; background: rgba(0,0,0,.9);
      z-index: 100; align-items: center; justify-content: center; }
    .modal-overlay.active { display: flex; }
    .modal { background: #1a1a1a; border: 1px solid #f0a500; border-radius: 16px;
      padding: 40px 36px; max-width: 380px; width: 90%; display: flex; flex-direction: column;
      align-items: center; gap: 20px; box-shadow: 0 0 40px rgba(240,165,0,.25); text-align: center; }
    .modal h2 { color: #fff; font-size: 1.1rem; letter-spacing: 2px; text-transform: uppercase; }
    .modal p  { color: #aaa; font-size: .9rem; line-height: 1.6; }
    .modal-actions { display: flex; gap: 14px; width: 100%; }
    .btn-confirm { flex:1; background:#f0a500; padding:12px; border-radius:10px; font-weight:700;
      text-transform:uppercase; cursor:pointer; border:none; color:#000; }
    .btn-cancel  { flex:1; background:transparent; border:1px solid #444; color:#aaa;
      padding:12px; border-radius:10px; text-transform:uppercase; cursor:pointer; }
    .modal-warning { display:flex; align-items:flex-start; gap:10px;
      background:rgba(240,165,0,.08); border:1px solid rgba(240,165,0,.35);
      border-radius:10px; padding:12px 14px; width:100%; text-align:left; }
    .modal-warning p { color:#c8922a; font-size:.78rem; line-height:1.5; margin:0; }
    .modal.validation { border-color: #00e676; box-shadow: 0 0 40px rgba(0,230,118,.2); }
    .modal.validation h2 { color: #00e676; }
    .val-row { display: flex; align-items: center; justify-content: space-between;
      width: 100%; padding: 8px 0; border-bottom: 1px solid #2a2a2a; }
    .val-row:last-child { border-bottom: none; }
    .val-name { color: #ccc; font-size: .9rem; }
    .val-badge { font-size: .75rem; font-weight: 700; padding: 4px 12px;
      border-radius: 20px; text-transform: uppercase; letter-spacing: 1px; }
    .val-badge.ok   { background: rgba(0,230,118,.15); color: #00e676; border: 1px solid #00e676; }
    .val-badge.fail { background: rgba(255,61,61,.15);  color: #ff3d3d; border: 1px solid #ff3d3d; }
    .ping-countdown { color: #555; font-size: .78rem; }
    .event-banner { display: none; width: 100%; max-width: 760px; padding: 14px 24px;
      border-radius: 14px; font-size: 1rem; font-weight: 700; letter-spacing: 2px;
      text-transform: uppercase; text-align: center; transition: all .3s; }
    .event-banner.show    { display: block; }
    .event-banner.standby { background: rgba(240,165,0,.15); color: #f0a500; border: 1px solid #f0a500; }
    .event-banner.running { background: rgba(0,255,0,.12);   color: #00ff00; border: 1px solid #00ff00; }
    .event-banner.stop    { background: rgba(255,0,0,.12);   color: #ff4444; border: 1px solid #ff4444; }
    .event-banner.whistle { background: rgba(0,123,255,.12); color: #4da6ff; border: 1px solid #4da6ff; }
  </style>
</head>
<body>
  <h1>Referee Panel</h1>
  <div class="event-banner" id="eventBanner"></div>
  <div class="cards-container">
    <div class="card" id="card-0">
      <div class="presence-dot offline" id="dot-0"></div>
      <svg viewBox="0 0 64 64" fill="none" xmlns="http://www.w3.org/2000/svg">
        <circle cx="32" cy="18" r="10" fill="white" opacity="0.85"/>
        <path d="M12 54c0-11 8-18 20-18s20 7 20 18" stroke="white" stroke-width="3" stroke-linecap="round" opacity="0.85"/>
      </svg>
      <span class="role">Crew Chief</span>
      <div class="battery-container" id="bat-0"><span class="battery-unknown">No data</span></div>
      <button class="btn-calibrate" onclick="openModal('Crew Chief', 0)">Calibrate</button>
    </div>
    <div class="card" id="card-1">
      <div class="presence-dot offline" id="dot-1"></div>
      <svg viewBox="0 0 64 64" fill="none" xmlns="http://www.w3.org/2000/svg">
        <circle cx="32" cy="18" r="10" fill="white" opacity="0.85"/>
        <path d="M12 54c0-11 8-18 20-18s20 7 20 18" stroke="white" stroke-width="3" stroke-linecap="round" opacity="0.85"/>
      </svg>
      <span class="role">Judge 1</span>
      <div class="battery-container" id="bat-1"><span class="battery-unknown">No data</span></div>
      <button class="btn-calibrate" onclick="openModal('Judge 1', 1)">Calibrate</button>
    </div>
    <div class="card" id="card-2">
      <div class="presence-dot offline" id="dot-2"></div>
      <svg viewBox="0 0 64 64" fill="none" xmlns="http://www.w3.org/2000/svg">
        <circle cx="32" cy="18" r="10" fill="white" opacity="0.85"/>
        <path d="M12 54c0-11 8-18 20-18s20 7 20 18" stroke="white" stroke-width="3" stroke-linecap="round" opacity="0.85"/>
      </svg>
      <span class="role">Judge 2</span>
      <div class="battery-container" id="bat-2"><span class="battery-unknown">No data</span></div>
      <button class="btn-calibrate" onclick="openModal('Judge 2', 2)">Calibrate</button>
    </div>
  </div>
  <div class="ping-countdown" id="pingCountdown">Next validation check: --:--</div>
  <div class="modal-overlay" id="modalOverlay">
    <div class="modal">
      <h2>Calibration Mode</h2>
      <p>You are about to enter calibration mode for <strong style="color:#fff"><span id="modalReferee"></span></strong>.</p>
      <div class="modal-warning">
        <span>&#9888;&#65039;</span>
        <p>This will interrupt any active session. Make sure the device is ready and in a quiet environment.</p>
      </div>
      <div class="modal-actions">
        <button class="btn-confirm" onclick="confirmCalibration()">Confirm</button>
        <button class="btn-cancel"  onclick="closeModal()">Cancel</button>
      </div>
    </div>
  </div>
  <div class="modal-overlay" id="valOverlay">
    <div class="modal validation">
      <h2>Validation Check</h2>
      <p>30-minute presence check results:</p>
      <div id="valRows" style="width:100%"></div>
      <div class="modal-actions">
        <button class="btn-confirm" style="background:#00e676;" onclick="closeValModal()">OK</button>
      </div>
    </div>
  </div>
)HTML";

// Parte 2: el bloque <script> (comillas dobles para evitar conflictos con PROGMEM)
const char HTML_P2[] PROGMEM = R"HTML(
<script>
var ws;
var NAMES = ["Crew Chief", "Judge 1", "Judge 2"];
var PING_INTERVAL = 30 * 60 * 1000;
var nextPingIn = PING_INTERVAL;
var bannerTimer = null;
var BANNER_MSGS = {
  standby: "Standby - Waiting for pair...",
  running: "Running - Session Active",
  stop:    "Stop - Button Pressed",
  whistle: "Whistle Detected",
  idle:    ""
};

function showBanner(status, refName) {
  var banner = document.getElementById("eventBanner");
  if (status === "idle") { banner.className = "event-banner"; banner.textContent = ""; return; }
  banner.className = "event-banner show " + status;
  banner.textContent = refName.toUpperCase() + " - " + (BANNER_MSGS[status] || status.toUpperCase());
  clearTimeout(bannerTimer);
  if (status !== "standby" && status !== "running")
    bannerTimer = setTimeout(function() { banner.className = "event-banner"; banner.textContent = ""; }, 4000);
}

function connectWS() {
  ws = new WebSocket("ws://" + location.hostname + ":81");
  ws.onopen  = function() { console.log("WS connected"); };
  ws.onclose = function() { console.log("WS closed, retrying..."); setTimeout(connectWS, 2000); };
  ws.onerror = function(e) { console.log("WS error", e); };
  ws.onmessage = function(event) {
    try {
      var msg = JSON.parse(event.data);
      if (msg.type === "status")      applyStatus(msg.id, msg.status);
      if (msg.type === "battery")     applyBattery(msg.id, msg.pct);
      if (msg.type === "presence")    applyPresence(msg.id, msg.online);
      if (msg.type === "ping_result") handlePingResult(msg.id, msg.online, msg.pct);
      if (msg.type === "ping_show")   showValModal(msg.results);
    } catch(e) { console.error("WS parse error", e); }
  };
}
connectWS();

function applyStatus(id, status) {
  var card = document.getElementById("card-" + id);
  if (!card) return;
  card.className = "card" + (status !== "idle" ? " status-" + status : "");
  showBanner(status, NAMES[id] || ("Referee " + id));
}
function applyPresence(id, online) {
  var dot = document.getElementById("dot-" + id);
  if (!dot) return;
  dot.className = "presence-dot " + (online ? "online" : "offline");
}
function applyBattery(id, pct) {
  var container = document.getElementById("bat-" + id);
  if (!container) return;
  
  var currentPct = parseInt(pct); 
  
  // Si no hay datos o es inválido, muestra "No data"
  if (currentPct < 0 || isNaN(currentPct)) { 
    container.innerHTML = '<span class="battery-unknown">No data</span>'; 
    return; 
  }
  
  var barClass = "battery-bar";
  if (currentPct <= 20) barClass += " low";
  else if (currentPct <= 50) barClass += " medium";
  
  // Renderiza solo la etiqueta "Battery" y la barra visual
  container.innerHTML =
    "<span class=\"battery-label\">Battery</span>" +
    "<div class=\"battery-bar-wrap\"><div class=\"" + barClass + "\" style=\"width:" + currentPct + "%\"></div></div>";
}
function handlePingResult(id, online, pct) {
  applyPresence(id, online);
  if (pct >= 0) applyBattery(id, pct);
}
function showValModal(results) {
  var html = "";
  for (var i = 0; i < 3; i++) {
    var r = results[i] || { online: false, pct: -1 };
    var badge = r.online
      ? "<span class=\"val-badge ok\">Online</span>"
      : "<span class=\"val-badge fail\">Offline</span>";
    
    // Eliminamos la variable 'bat' que agregaba el text "- X%"
    html += "<div class=\"val-row\"><span class=\"val-name\">" + NAMES[i] + "</span>" + badge + "</div>";
  }
  document.getElementById("valRows").innerHTML = html;
  document.getElementById("valOverlay").classList.add("active");
}
function closeValModal() { document.getElementById("valOverlay").classList.remove("active"); }

setInterval(function() {
  nextPingIn -= 1000;
  if (nextPingIn < 0) nextPingIn = PING_INTERVAL;
  var m = Math.floor(nextPingIn / 60000);
  var s = Math.floor((nextPingIn % 60000) / 1000);
  document.getElementById("pingCountdown").textContent =
    "Next validation check: " + String(m).padStart(2,"0") + ":" + String(s).padStart(2,"0");
}, 1000);

var activeReferee = "", activeRefereeId = -1;
function openModal(name, id) {
  activeReferee = name; activeRefereeId = id;
  document.getElementById("modalReferee").textContent = name;
  document.getElementById("modalOverlay").classList.add("active");
}
function closeModal() { document.getElementById("modalOverlay").classList.remove("active"); }
function confirmCalibration() {
  closeModal();
  if (ws && ws.readyState === WebSocket.OPEN)
    ws.send(JSON.stringify({ cmd: "calibrate", id: activeRefereeId }));
}
</script>
</body>
</html>
)HTML";

// ==================== SERVIR HTML ====================
void handleRoot() {
  String page = "";
  page += FPSTR(HTML_P1);
  page += FPSTR(HTML_P2);
  httpServer.send(200, "text/html", page);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n=== RECEPTOR v3.1 WebServer + LoRa (Modo AP) ==="));

  // WiFi - Modo Punto de Acceso
  WiFi.softAP("Referee_Panel", "arbitro123");
  Serial.println(F("AP creado -> Conéctate a: Referee_Panel"));
  Serial.println(F("Contraseña: arbitro123"));
  Serial.printf("Dashboard: http://%s\n", WiFi.softAPIP().toString().c_str());

  // LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("[ERROR] LoRa no encontrado."));
    while (1) delay(1000);
  }
  Serial.println(F("LoRa OK"));

  // HTTP
  httpServer.on("/", handleRoot);
  httpServer.begin();
  Serial.println(F("HTTP OK (puerto 80)"));

  // WebSocket
  wsServer.begin();
  wsServer.onEvent(onWsEvent);
  Serial.println(F("WebSocket OK (puerto 81)"));

  lastPingTime = millis();
}

// ==================== LOOP ====================
void loop() {
  httpServer.handleClient();
  wsServer.loop();
  checkLoRa();
  checkPresenceTimeout();
  checkPingInterval();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) handleSerialCommand(cmd);
  }
  delay(2);
}

// ==================== WEBSOCKET EVENTOS ====================
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("[WS] Cliente #%d conectado\n", num);
    sendFullState(num);
  } else if (type == WStype_TEXT) {
    String msg = String((char*)payload);
    handleWsCommand(msg);
  }
}

void sendFullState(uint8_t clientNum) {
  for (int i = 0; i < 3; i++) {
    wsServer.sendTXT(clientNum,
      "{\"type\":\"presence\",\"id\":" + String(i) +
      ",\"online\":" + (referees[i].online ? "true" : "false") + "}");
    wsServer.sendTXT(clientNum,
      "{\"type\":\"status\",\"id\":" + String(i) +
      ",\"status\":\"" + referees[i].status + "\"}");
    wsServer.sendTXT(clientNum,
      "{\"type\":\"battery\",\"id\":" + String(i) +
      ",\"pct\":" + String(referees[i].battery) + "}");
  }
}

void wsBroadcast(String json) {
  wsServer.broadcastTXT(json);
}

// ==================== COMANDOS DESDE BROWSER ====================
void handleWsCommand(String msg) {
  if (msg.indexOf("\"calibrate\"") != -1) {
    int idPos = msg.indexOf("\"id\":");
    if (idPos == -1) return;
    int idx = msg.charAt(idPos + 5) - '0';
    if (idx < 0 || idx > 2) return;
    sendLoRaCmd(refereeLoRaID(idx) + ":c");
    Serial.printf("[WS->LoRa] Calibrar %s\n", refereeLoRaID(idx).c_str());
  }
}

// ==================== ENVIO LORA ====================
void sendLoRaCmd(String payload) {
  String pkt = "CMD:" + payload;
  LoRa.beginPacket();
  LoRa.print(pkt);
  LoRa.endPacket();
  Serial.printf("[TX] %s\n", pkt.c_str());
}

// ==================== RECEPCION LORA ====================
void checkLoRa() {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;

  String pkt = "";
  while (LoRa.available()) pkt += (char)LoRa.read();
  pkt.trim();

  Serial.printf("[RX] '%s' | RSSI:%d SNR:%.1f\n",
    pkt.c_str(), LoRa.packetRssi(), LoRa.packetSnr());

  // Esperamos formatos:
  //  - ID:comando
  //  - ID:comando:BAT   (ej. CREW_CHIEF:start:78)
  int sep = pkt.indexOf(':');
  if (sep == -1) return;

  String id      = pkt.substring(0, sep);
  String tail    = pkt.substring(sep + 1); // puede ser "comando" o "comando:BAT"
  int    idx     = refereeIndex(id);
  if (idx == -1) return;

  // Asumimos presencia al recibir cualquier paquete válido
  referees[idx].online   = true;
  referees[idx].lastSeen = millis();
  wsBroadcast("{\"type\":\"presence\",\"id\":" + String(idx) + ",\"online\":true}");

  // Si hay un segundo ':' en tail, extraer battery
  int sep2 = tail.indexOf(':');
  String comando;
  int batPct = -1;
  if (sep2 == -1) {
    comando = tail;
  } else {
    comando = tail.substring(0, sep2);
    String batStr = tail.substring(sep2 + 1);
    batStr.trim();
    // quitar '%' si viene
    if (batStr.endsWith("%")) batStr = batStr.substring(0, batStr.length() - 1);
    // convertir a int (toInt devuelve 0 si no hay número; usar comprobación)
    int parsed = batStr.toInt();
    if (batStr.length() > 0 && (parsed != 0 || batStr == "0")) {
      batPct = parsed;
      // guardar y notificar via websocket
      referees[idx].battery = batPct;
      wsBroadcast("{\"type\":\"battery\",\"id\":" + String(idx) + ",\"pct\":" + String(batPct) + "}");
      Serial.printf("[BAT] %s -> %d%%\n", id.c_str(), batPct);
    } else {
      // no válido -> dejar -1 (sin cambio)
      Serial.printf("[BAT] parse error from '%s'\n", batStr.c_str());
    }
  }

  comando.trim();

  String status = "idle";
  if      (comando == "silbatazo") status = "whistle";
  else if (comando == "start")     status = "running";
  else if (comando == "stop")      status = "stop";

  if (status != "idle") {
    referees[idx].status = status;
    wsBroadcast("{\"type\":\"status\",\"id\":" + String(idx) +
                ",\"status\":\"" + status + "\"}");
    Serial.printf("[STATUS] %s -> %s\n", id.c_str(), status.c_str());
  }
}

// ==================== PRESENCIA / TIMEOUT ====================
void checkPresenceTimeout() {
  unsigned long now = millis();
  for (int i = 0; i < 3; i++) {
    if (referees[i].online && (now - referees[i].lastSeen > PRESENCE_TIMEOUT_MS)) {
      referees[i].online = false;
      referees[i].status = "idle";
      wsBroadcast("{\"type\":\"presence\",\"id\":" + String(i) + ",\"online\":false}");
      wsBroadcast("{\"type\":\"status\",\"id\":" + String(i) + ",\"status\":\"idle\"}");
      Serial.printf("[TIMEOUT] %s offline\n", refereeLoRaID(i).c_str());
    }
  }
}

// ==================== PING CADA 30 MIN ====================
void checkPingInterval() {
  if (millis() - lastPingTime < PING_INTERVAL_MS) return;
  lastPingTime = millis();
  Serial.println(F("[PING] Enviando ping broadcast..."));
  sendLoRaCmd("s");

  String json = "{\"type\":\"ping_show\",\"results\":[";
  for (int i = 0; i < 3; i++) {
    json += "{\"online\":" + String(referees[i].online ? "true" : "false") +
            ",\"pct\":"    + String(referees[i].battery) + "}";
    if (i < 2) json += ",";
  }
  json += "]}";
  wsBroadcast(json);
}

// ==================== COMANDOS SERIAL ====================
void handleSerialCommand(String input) {
  input.toLowerCase();
  if (input == "c" || input == "s" || input == "+" || input == "-") {
    sendLoRaCmd(input); return;
  }
  if (input == "h" || input == "?") { printHelp(); return; }

  String destID = "";
  char   cmd    = 0;
  if      (input.startsWith("cc") || input.startsWith("cs") ||
           input.startsWith("c+") || input.startsWith("c-"))
    { destID = "CREW_CHIEF"; cmd = input.charAt(1); }
  else if (input.startsWith("j1") && input.length() >= 3)
    { destID = "JUDGE_1"; cmd = input.charAt(2); }
  else if (input.startsWith("j2") && input.length() >= 3)
    { destID = "JUDGE_2"; cmd = input.charAt(2); }

  if (destID != "" && (cmd == 'c' || cmd == 's' || cmd == '+' || cmd == '-')) {
    sendLoRaCmd(destID + ":" + cmd); return;
  }
  Serial.printf("[!] Comando no reconocido: '%s'\n", input.c_str());
}

void printHelp() {
  Serial.println(F("--- COMANDOS ---"));
  Serial.println(F("  c/s/+/-          -> broadcast"));
  Serial.println(F("  cc/cs/c+/c-      -> CREW_CHIEF"));
  Serial.println(F("  j1c/j1s/j1+/j1-  -> JUDGE_1"));
  Serial.println(F("  j2c/j2s/j2+/j2-  -> JUDGE_2"));
  Serial.println(F("  h                -> ayuda"));
}