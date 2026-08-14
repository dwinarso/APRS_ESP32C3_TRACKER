#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <TinyGPS++.h>
#include <ctype.h>

// ========================= PIN DEFINITIONS (ESP32-C3 Super Mini) =========================
#define AFSK_PIN       10   // Audio Out (Sinyal AFSK)
#define PTT_PIN        5    // PTT Control Out
#define STATUS_LED     8    // Status LED (Onboard LED C3 Super Mini)

// --- PIN GPS SUDAH DIPINDAHKAN AGAR AMAN SAAT BOOTING ---
#define GPS_RX_PIN     6    // Dihubungkan ke TX GPS
#define GPS_TX_PIN     7    // Dihubungkan ke RX GPS

#define BUSY_IN_PIN    3    // Audio Squelch In (Membutuhkan Pin ADC1)

// ========================= APRS / AX.25 ============================
#define AX25_FLAG      0x7E
#define AX25_CTRL      0x03
#define AX25_PID       0xF0

#define DT_STATUS      '>'
#define DT_POS         '!'

#define PKT_STATUS        1
#define PKT_FIXPOS        2
#define PKT_FIXPOS_STATUS 3

// ========================= MODEM ============================
static bool nrziLevel = 0;
static uint8_t bitStuffCount = 0;
static uint16_t crc = 0xFFFF;

const float baud_adj = 0.985f;
const unsigned int tc1200 = (unsigned int)(0.5f * baud_adj * 1000000.0f / 1200.0f);
const unsigned int tc2200 = (unsigned int)(0.5f * baud_adj * 1000000.0f / 2200.0f);

// ========================= CONFIG ============================
char mycall[7] = "YC2BGO";
uint8_t myssid = 7;
char mystatus[100] = "Tracker Over ESP32C3 - YC2BGO";

bool smartBeaconing = false;
unsigned long tx_interval = 60000UL; // Default 60 detik (60000 ms)

char destCall[7] = "APRS";
uint8_t destSSID = 0;

char digi1[7] = "WIDE1";
uint8_t digissid1 = 1;
char digi2[7] = "WIDE2";
uint8_t digissid2 = 2;

char lat[9]  = "0000.00N";
char lon[10] = "00000.00E";

char dummy_lat[9]  = "0659.00S";
char dummy_lon[10] = "10649.00E";

char sym_ovl = '/';
char sym_tab = 'k';

// ========================= BUSY DETECT =========================
uint8_t busyMode = 1;
uint16_t busyAdcThreshold = 140;
unsigned long busySampleWindowMs = 25;
unsigned long busyHangMs = 220;
unsigned long busyClearHoldMs = 300;
unsigned long busyRetryMs = 5000;
unsigned long busyMaxWaitMs = 30000UL;
uint16_t busyCenterCalSamples = 400;
unsigned long busyRecalMs = 15000UL;

// ========================= RUNTIME ============================
bool gps_valid = false;
bool gps_locked = false;
float last_speed = 0.0f;

unsigned long last_tx_time = 0;
unsigned long pending_tx_since = 0;
bool pending_auto_tx = false;
bool pending_dummy_tx = false;
bool pending_manual_tx = false;

String gps_connection_status = "Not Connected";
String channel_busy_status = "UNKNOWN";

unsigned long last_gps_data_time = 0;
unsigned long last_status_print = 0;
unsigned long last_busy_retry_time = 0;
unsigned long lastBusyDetectedMs = 0;
unsigned long lastBusyCalMs = 0;

uint16_t busyAdcCenter = 2048;
uint16_t lastBusyPeak = 0;

// ========================= OBJECTS ============================
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
WebServer server(80);
Preferences prefs;

// ========================= UTIL ============================
void safeCopyUpper(char *dst, size_t dstSize, const String &src, size_t maxLen) {
  String s = src;
  s.trim();
  s.toUpperCase();
  if (s.length() > maxLen) s = s.substring(0, maxLen);
  s.toCharArray(dst, dstSize);
  dst[dstSize - 1] = '\0';
}

bool isValidBaseCall(const String &in) {
  if (in.length() < 1 || in.length() > 6) return false;
  for (size_t i = 0; i < in.length(); i++) {
    char c = in.charAt(i);
    if (!(isalnum((unsigned char)c))) return false;
  }
  return true;
}

bool isValidSSID(int v) {
  return v >= 0 && v <= 15;
}

bool isValidSymbolChar(char c) {
  return c >= 33 && c <= 126;
}

unsigned long getEffectiveInterval() {
  unsigned long effective_interval = tx_interval;
  if (smartBeaconing && gps_valid && gps.speed.isValid()) {
    last_speed = gps.speed.kmph();
    if (last_speed > 70.0f) effective_interval = tx_interval / 4;
    else if (last_speed > 30.0f) effective_interval = tx_interval / 2;
    else if (last_speed < 5.0f) effective_interval = tx_interval;
    
    // Batas minimum Smart Beaconing diturunkan jadi 15 detik agar sinkron dengan opsi 30 detik
    if (effective_interval < 15000UL) effective_interval = 15000UL;
  }
  return effective_interval;
}

// ========================= AFSK TONES ============================
void tone1200() {
  digitalWrite(AFSK_PIN, HIGH);
  delayMicroseconds(tc1200);
  digitalWrite(AFSK_PIN, LOW);
  delayMicroseconds(tc1200);
}

void tone2200() {
  digitalWrite(AFSK_PIN, HIGH);
  delayMicroseconds(tc2200);
  digitalWrite(AFSK_PIN, LOW);
  delayMicroseconds(tc2200);
  digitalWrite(AFSK_PIN, HIGH);
  delayMicroseconds(tc2200);
  digitalWrite(AFSK_PIN, LOW);
  delayMicroseconds(tc2200);
}

void sendTone(bool level) {
  if (level) tone1200();
  else tone2200();
}

// ========================= CRC ============================
void crcReset() { crc = 0xFFFF; }

void crcUpdate(bool bitIn) {
  unsigned short xor_in = crc ^ bitIn;
  crc >>= 1;
  if (xor_in & 0x01) crc ^= 0x8408;
}

void sendRawBit(bool bitVal, bool updateCrc, bool enableBitStuff) {
  if (updateCrc) crcUpdate(bitVal);
  if (bitVal) {
    sendTone(nrziLevel);
    if (enableBitStuff) {
      bitStuffCount++;
      if (bitStuffCount == 5) {
        nrziLevel ^= 1;
        sendTone(nrziLevel);
        bitStuffCount = 0;
      }
    }
  } else {
    nrziLevel ^= 1;
    sendTone(nrziLevel);
    bitStuffCount = 0;
  }
}

void sendByteNRZI(uint8_t b, bool updateCrc, bool enableBitStuff) {
  for (int i = 0; i < 8; i++) {
    bool bitVal = b & 0x01;
    sendRawBit(bitVal, updateCrc, enableBitStuff);
    b >>= 1;
  }
}

void sendFlag(uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    sendByteNRZI(AX25_FLAG, false, false);
  }
}

void sendCRC() {
  uint8_t crc_lo = crc ^ 0xFF;
  uint8_t crc_hi = (crc >> 8) ^ 0xFF;
  sendByteNRZI(crc_lo, false, true);
  sendByteNRZI(crc_hi, false, true);
}

// ========================= AX.25 ADDRESS ============================
void sendAX25Address(const char *call, uint8_t ssid, bool last) {
  char padded[7] = {' ', ' ', ' ', ' ', ' ', ' ', '\0'};
  size_t len = strlen(call);
  if (len > 6) len = 6;
  for (size_t i = 0; i < len; i++) padded[i] = toupper((unsigned char)call[i]);
  for (int i = 0; i < 6; i++) sendByteNRZI(((uint8_t)padded[i]) << 1, true, true);
  uint8_t ssidByte = 0x60 | ((ssid & 0x0F) << 1);
  if (last) ssidByte |= 0x01;
  sendByteNRZI(ssidByte, true, true);
}

void sendHeader() {
  sendAX25Address(destCall, destSSID, false);
  sendAX25Address(mycall, myssid, false);
  sendAX25Address(digi1, digissid1, false);
  sendAX25Address(digi2, digissid2, true);
  sendByteNRZI(AX25_CTRL, true, true);
  sendByteNRZI(AX25_PID, true, true);
}

// ========================= PAYLOAD ============================
void sendStringAX25(const char *s) {
  while (*s) sendByteNRZI((uint8_t)*s++, true, true);
}

void sendPayload(uint8_t type, bool useDummy = false) {
  const char *useLat = useDummy ? dummy_lat : lat;
  const char *useLon = useDummy ? dummy_lon : lon;

  if (type == PKT_FIXPOS) {
    sendByteNRZI(DT_POS, true, true);
    sendStringAX25(useLat);
    sendByteNRZI(sym_ovl, true, true);
    sendStringAX25(useLon);
    sendByteNRZI(sym_tab, true, true);
  } else if (type == PKT_STATUS) {
    sendByteNRZI(DT_STATUS, true, true);
    sendStringAX25(mystatus);
  } else if (type == PKT_FIXPOS_STATUS) {
    sendByteNRZI(DT_POS, true, true);
    sendStringAX25(useLat);
    sendByteNRZI(sym_ovl, true, true);
    sendStringAX25(useLon);
    sendByteNRZI(sym_tab, true, true);
    sendByteNRZI(' ', true, true);
    sendStringAX25(mystatus);
  }
}

// ========================= DEBUG ============================
void printDebug(uint8_t type, bool useDummy = false) {
  const char *useLat = useDummy ? dummy_lat : lat;
  const char *useLon = useDummy ? dummy_lon : lon;

  Serial.print("Data Packet : ");
  Serial.print(mycall); Serial.print("-"); Serial.print(myssid); Serial.print(">");
  Serial.print(destCall); Serial.print(","); Serial.print(digi1); Serial.print("-");
  Serial.print(digissid1); Serial.print(","); Serial.print(digi2); Serial.print("-");
  Serial.print(digissid2); Serial.print(":");

  if (type == PKT_FIXPOS) {
    Serial.print("!"); Serial.print(useLat); Serial.print(sym_ovl); Serial.print(useLon); Serial.print(sym_tab);
  } else if (type == PKT_STATUS) {
    Serial.print(">"); Serial.print(mystatus);
  } else if (type == PKT_FIXPOS_STATUS) {
    Serial.print("!"); Serial.print(useLat); Serial.print(sym_ovl); Serial.print(useLon); Serial.print(sym_tab);
    Serial.print(" "); Serial.print(mystatus);
  }
  Serial.println();
}

// ========================= BUSY DETECT ============================
uint16_t readBusyPeak(unsigned long windowMs) {
  unsigned long start = millis();
  uint16_t peak = 0;
  while (millis() - start < windowMs) {
    int v = analogRead(BUSY_IN_PIN);
    int dev = abs(v - (int)busyAdcCenter);
    if ((uint16_t)dev > peak) peak = (uint16_t)dev;
    delayMicroseconds(200);
  }
  return peak;
}

void calibrateBusyCenter(uint16_t samples = 400) {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += analogRead(BUSY_IN_PIN);
    delayMicroseconds(200);
  }
  busyAdcCenter = (uint16_t)(sum / samples);
}

bool isBusyAudioRaw() {
  lastBusyPeak = readBusyPeak(busySampleWindowMs);
  if (lastBusyPeak >= busyAdcThreshold) {
    lastBusyDetectedMs = millis();
    return true;
  }
  return false;
}

bool isChannelBusyRaw() {
  if (busyMode == 0) return false;
  if (isBusyAudioRaw()) return true;
  if (millis() - lastBusyDetectedMs < busyHangMs) return true;
  return false;
}

bool isChannelClearStable(unsigned long clearHoldMs) {
  unsigned long start = millis();
  while (millis() - start < clearHoldMs) {
    if (isChannelBusyRaw()) return false;
    delay(5);
  }
  return true;
}

bool canTransmitNow() {
  if (busyMode == 0) return true;
  if (isChannelBusyRaw()) return false;
  return isChannelClearStable(busyClearHoldMs);
}

void updateBusyStatusText() {
  if (busyMode == 0) { channel_busy_status = "DISABLED"; return; }
  bool busy = isChannelBusyRaw();
  if (busy) channel_busy_status = "BUSY";
  else channel_busy_status = "CLEAR";
}

// ========================= TX ============================
void beginTX() {
  digitalWrite(STATUS_LED, LOW); // LED ON
  digitalWrite(PTT_PIN, HIGH);
  delay(120);
}

void endTX() {
  delay(50);
  digitalWrite(PTT_PIN, LOW);
  digitalWrite(STATUS_LED, HIGH); // LED OFF
}

void sendPacket(uint8_t packetType, bool useDummy = false) {
  Serial.println("\n[>>>>>>>> MENGIRIM DATA APRS (TX) >>>>>>>>]");
  printDebug(packetType, useDummy);
  
  bitStuffCount = 0;
  nrziLevel = 0;
  
  beginTX();
  sendFlag(80);
  crcReset();
  sendHeader();
  sendPayload(packetType, useDummy);
  sendCRC();
  sendFlag(3);
  endTX();
  
  Serial.println("[<<<<<<<< SELESAI MENGIRIM (TX Selesai) <<<<<<<<]\n");
}

// ========================= GPS ============================
void updateCoordinates() {
  double lat_val = gps.location.lat();
  double lon_val = gps.location.lng();
  char lat_dir = (lat_val < 0) ? 'S' : 'N';
  char lon_dir = (lon_val < 0) ? 'W' : 'E';
  lat_val = fabs(lat_val);
  lon_val = fabs(lon_val);
  int lat_deg = (int)lat_val;
  int lon_deg = (int)lon_val;
  double lat_min = (lat_val - lat_deg) * 60.0;
  double lon_min = (lon_val - lon_deg) * 60.0;
  snprintf(lat, sizeof(lat), "%02d%05.2f%c", lat_deg, lat_min, lat_dir);
  snprintf(lon, sizeof(lon), "%03d%05.2f%c", lon_deg, lon_min, lon_dir);
}

// ========================= PREFERENCES ============================
void savePrefs() {
  prefs.begin("aprs_config", false);
  prefs.putString("callsign", mycall);
  prefs.putUChar("ssid", myssid);
  prefs.putString("status", mystatus);
  prefs.putBool("smartbeac", smartBeaconing);
  prefs.putULong("interval", tx_interval);
  prefs.putString("digi1", digi1);
  prefs.putUChar("digissid1", digissid1);
  prefs.putString("digi2", digi2);
  prefs.putUChar("digissid2", digissid2);
  prefs.putChar("symtab", sym_tab);
  prefs.putChar("symovl", sym_ovl);

  prefs.putUChar("busymode", busyMode);
  prefs.putUShort("busythr", busyAdcThreshold);
  prefs.putULong("busywin", busySampleWindowMs);
  prefs.putULong("busyhang", busyHangMs);
  prefs.putULong("busyhold", busyClearHoldMs);
  prefs.putULong("busyretry", busyRetryMs);
  prefs.putULong("busymaxw", busyMaxWaitMs);
  prefs.putUShort("busycents", busyCenterCalSamples);
  prefs.putULong("busyrecal", busyRecalMs);
  prefs.end();
}

void loadPrefs() {
  prefs.begin("aprs_config", true);
  String call = prefs.getString("callsign", "YC2BGO");
  safeCopyUpper(mycall, sizeof(mycall), call, 6);
  myssid = prefs.getUChar("ssid", 7);
  String stat = prefs.getString("status", "Tracker Over ESP32C3 - YC2BGO");
  stat.toCharArray(mystatus, sizeof(mystatus));
  mystatus[sizeof(mystatus) - 1] = '\0';
  smartBeaconing = prefs.getBool("smartbeac", false);
  tx_interval = prefs.getULong("interval", 60000UL);
  String d1 = prefs.getString("digi1", "WIDE1");
  String d2 = prefs.getString("digi2", "WIDE2");
  safeCopyUpper(digi1, sizeof(digi1), d1, 6);
  safeCopyUpper(digi2, sizeof(digi2), d2, 6);
  digissid1 = prefs.getUChar("digissid1", 1);
  digissid2 = prefs.getUChar("digissid2", 2);
  sym_tab = prefs.getChar("symtab", 'k');
  sym_ovl = prefs.getChar("symovl", '/');
  busyMode = prefs.getUChar("busymode", 1);
  if (busyMode > 1) busyMode = 1;
  busyAdcThreshold = prefs.getUShort("busythr", 140);
  busySampleWindowMs = prefs.getULong("busywin", 25);
  busyHangMs = prefs.getULong("busyhang", 220);
  busyClearHoldMs = prefs.getULong("busyhold", 300);
  busyRetryMs = prefs.getULong("busyretry", 5000);
  busyMaxWaitMs = prefs.getULong("busymaxw", 30000UL);
  busyCenterCalSamples = prefs.getUShort("busycents", 400);
  busyRecalMs = prefs.getULong("busyrecal", 15000UL);
  prefs.end();
}

// ========================= WEB ============================
String getGPSStatus() {
  if (gps_locked) return "Locked - Ready to Transmit";
  return "Not Locked - Not Ready";
}

String getRootHtml() {
  String gpsColor = gps_locked ? "green" : "red";
  String connColor = (gps_connection_status == "Locked") ? "green" :
                     (gps_connection_status == "Searching") ? "orange" : "red";
  String busyColor = (channel_busy_status == "BUSY") ? "red" :
                     (channel_busy_status == "CLEAR") ? "green" : "gray";

  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>APRS Tracker ESP32-C3</title>
<style>
body{font-family:Arial,sans-serif;background:#eef2f7;margin:0;padding:20px;color:#222}
.card{max-width:920px;margin:auto;background:#fff;border-radius:16px;padding:22px;box-shadow:0 8px 24px rgba(0,0,0,.08)}
h1{margin-top:0;color:#0b63ce}
h2{margin-top:24px;color:#0f172a;font-size:18px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:14px}
@media(max-width:700px){.grid{grid-template-columns:1fr}}
label{display:block;font-weight:bold;margin:10px 0 6px}
input,select{width:100%;padding:11px;border:1px solid #d0d7e2;border-radius:10px;box-sizing:border-box}
button{padding:12px 16px;border:0;border-radius:10px;color:#fff;cursor:pointer;margin-top:12px;font-weight:bold}
.save{background:#16a34a}.tx{background:#2563eb}.dummy{background:#d97706}
.status{margin-top:18px;padding:16px;border-radius:12px;background:#f6f8fb}
small{color:#667;display:block;margin-top:4px}
</style>
<script>
function postAction(url){
  fetch(url,{method:'POST'})
    .then(r=>r.text().then(t=>({ok:r.ok,text:t})))
    .then(x=>alert(x.text))
    .catch(e=>alert('Error: '+e));
}
setInterval(()=>{
  const ae=document.activeElement;
  if(!ae || (ae.tagName!=='INPUT' && ae.tagName!=='SELECT' && ae.tagName!=='TEXTAREA')){
    location.reload();
  }
},5000);
</script>
</head>
<body>
<div class="card">
<h1>APRS Tracker ESP32-C3</h1>

<form action="/save" method="POST">
<h2>APRS Settings</h2>
<div class="grid">
<div>
<label>Callsign Base</label>
<input name="callsign" maxlength="6" value=")rawliteral" + String(mycall) + R"rawliteral(">
</div>
<div>
<label>SSID</label>
<input name="ssid" type="number" min="0" max="15" value=")rawliteral" + String(myssid) + R"rawliteral(">
</div>

<div>
<label>Status / Comment</label>
<input name="comment" maxlength="99" value=")rawliteral" + String(mystatus) + R"rawliteral(">
</div>

<div>
<label>Interval Transmit</label>
<select name="interval">
<option value="30" )rawliteral" + String(tx_interval == 30000UL ? "selected" : "") + R"rawliteral(>30 Detik</option>
<option value="60" )rawliteral" + String(tx_interval == 60000UL ? "selected" : "") + R"rawliteral(>1 Menit</option>
<option value="120" )rawliteral" + String(tx_interval == 120000UL ? "selected" : "") + R"rawliteral(>2 Menit</option>
<option value="300" )rawliteral" + String(tx_interval == 300000UL ? "selected" : "") + R"rawliteral(>5 Menit</option>
<option value="600" )rawliteral" + String(tx_interval == 600000UL ? "selected" : "") + R"rawliteral(>10 Menit</option>
<option value="900" )rawliteral" + String(tx_interval == 900000UL ? "selected" : "") + R"rawliteral(>15 Menit</option>
</select>
</div>

<div>
<label>Digi 1</label>
<input name="digi1" maxlength="6" value=")rawliteral" + String(digi1) + R"rawliteral(">
</div>
<div>
<label>Digi1 SSID</label>
<input name="digissid1" type="number" min="0" max="15" value=")rawliteral" + String(digissid1) + R"rawliteral(">
</div>

<div>
<label>Digi 2</label>
<input name="digi2" maxlength="6" value=")rawliteral" + String(digi2) + R"rawliteral(">
</div>
<div>
<label>Digi2 SSID</label>
<input name="digissid2" type="number" min="0" max="15" value=")rawliteral" + String(digissid2) + R"rawliteral(">
</div>

<div>
<label>Symbol Table</label>
<input name="symtab" maxlength="1" value=")rawliteral" + String(sym_tab) + R"rawliteral(">
</div>
<div>
<label>Symbol Overlay / Table ID</label>
<input name="symovl" maxlength="1" value=")rawliteral" + String(sym_ovl) + R"rawliteral(">
</div>
</div>

<label style="margin-top:12px">
<input type="checkbox" name="smartbeaconing" )rawliteral" + String(smartBeaconing ? "checked" : "") + R"rawliteral(>
 Enable Smart Beaconing
</label>

<h2>Channel Busy dari Speaker Radio</h2>
<div class="grid">
<div>
<label>Busy Mode</label>
<select name="busymode">
<option value="0" )rawliteral" + String(busyMode == 0 ? "selected" : "") + R"rawliteral(>OFF</option>
<option value="1" )rawliteral" + String(busyMode == 1 ? "selected" : "") + R"rawliteral(>Speaker Audio Detect</option>
</select>
</div>

<div>
<label>ADC Threshold</label>
<input name="busythr" type="number" min="10" max="2000" value=")rawliteral" + String(busyAdcThreshold) + R"rawliteral(">
<small>Naikkan jika terlalu sensitif, turunkan jika tidak mendeteksi burst APRS</small>
</div>

<div>
<label>Sample Window (ms)</label>
<input name="busywin" type="number" min="5" max="200" value=")rawliteral" + String(busySampleWindowMs) + R"rawliteral(">
</div>

<div>
<label>Busy Hang (ms)</label>
<input name="busyhang" type="number" min="20" max="3000" value=")rawliteral" + String(busyHangMs) + R"rawliteral(">
</div>

<div>
<label>Clear Hold (ms)</label>
<input name="busyhold" type="number" min="50" max="5000" value=")rawliteral" + String(busyClearHoldMs) + R"rawliteral(">
</div>

<div>
<label>Retry Delay (ms)</label>
<input name="busyretry" type="number" min="500" max="60000" value=")rawliteral" + String(busyRetryMs) + R"rawliteral(">
</div>

<div>
<label>Max Wait Before Skip TX (ms)</label>
<input name="busymaxw" type="number" min="1000" max="300000" value=")rawliteral" + String(busyMaxWaitMs) + R"rawliteral(">
</div>

<div>
<label>Center Calibration Samples</label>
<input name="busycents" type="number" min="50" max="3000" value=")rawliteral" + String(busyCenterCalSamples) + R"rawliteral(">
</div>

<div>
<label>Recalibration Interval (ms)</label>
<input name="busyrecal" type="number" min="1000" max="120000" value=")rawliteral" + String(busyRecalMs) + R"rawliteral(">
</div>
</div>

<button class="save" type="submit">Save Settings</button>
</form>

<button class="tx" onclick="postAction('/transmit')">Manual Transmit</button>
<button class="dummy" onclick="postAction('/dummy_transmit')">TX Dummy Data</button>

<div class="status">
<p><b>Latitude:</b> )rawliteral" + String(lat) + R"rawliteral(</p>
<p><b>Longitude:</b> )rawliteral" + String(lon) + R"rawliteral(</p>
<p><b>GPS Connection:</b> <span style="color:)rawliteral" + connColor + R"rawliteral(;">)rawliteral" + gps_connection_status + R"rawliteral(</span></p>
<p><b>GPS Status:</b> <span style="color:)rawliteral" + gpsColor + R"rawliteral(;">)rawliteral" + getGPSStatus() + R"rawliteral(</span></p>
<p><b>Channel Busy:</b> <span style="color:)rawliteral" + busyColor + R"rawliteral(;">)rawliteral" + channel_busy_status + R"rawliteral(</span></p>
<p><b>ADC Center:</b> )rawliteral" + String(busyAdcCenter) + R"rawliteral(</p>
<p><b>Last Peak:</b> )rawliteral" + String(lastBusyPeak) + R"rawliteral(</p>
<p><b>Path:</b> )rawliteral" + String(digi1) + "-" + String(digissid1) + "," + String(digi2) + "-" + String(digissid2) + R"rawliteral(</p>
</div>
</div>
</body>
</html>
)rawliteral";
  return html;
}

void handleRoot() {
  updateBusyStatusText();
  server.send(200, "text/html", getRootHtml());
}

void handleSave() {
  if (server.hasArg("callsign")) {
    String s = server.arg("callsign");
    s.trim();
    s.toUpperCase();
    if (isValidBaseCall(s)) safeCopyUpper(mycall, sizeof(mycall), s, 6);
  }
  if (server.hasArg("ssid")) {
    int v = server.arg("ssid").toInt();
    if (isValidSSID(v)) myssid = v;
  }
  if (server.hasArg("comment")) {
    String s = server.arg("comment");
    if (s.length() > 99) s = s.substring(0, 99);
    s.toCharArray(mystatus, sizeof(mystatus));
    mystatus[sizeof(mystatus) - 1] = '\0';
  }
  
  // LOGIKA INTERVAL DIPERBARUI (Satuan Detik)
  if (server.hasArg("interval")) {
    int val = server.arg("interval").toInt();
    // Jika value dropdown antara 10 detik dan 3600 detik (1 jam), simpan
    if (val >= 10 && val <= 3600) {
        tx_interval = (unsigned long)val * 1000UL; // Konversi detik ke ms
    }
  }

  if (server.hasArg("digi1")) {
    String s = server.arg("digi1");
    s.trim();
    s.toUpperCase();
    if (isValidBaseCall(s)) safeCopyUpper(digi1, sizeof(digi1), s, 6);
  }
  if (server.hasArg("digissid1")) {
    int v = server.arg("digissid1").toInt();
    if (isValidSSID(v)) digissid1 = v;
  }
  if (server.hasArg("digi2")) {
    String s = server.arg("digi2");
    s.trim();
    s.toUpperCase();
    if (isValidBaseCall(s)) safeCopyUpper(digi2, sizeof(digi2), s, 6);
  }
  if (server.hasArg("digissid2")) {
    int v = server.arg("digissid2").toInt();
    if (isValidSSID(v)) digissid2 = v;
  }
  if (server.hasArg("symtab")) {
    String s = server.arg("symtab");
    if (s.length() >= 1 && isValidSymbolChar(s.charAt(0))) sym_tab = s.charAt(0);
  }
  if (server.hasArg("symovl")) {
    String s = server.arg("symovl");
    if (s.length() >= 1 && isValidSymbolChar(s.charAt(0))) sym_ovl = s.charAt(0);
  }
  smartBeaconing = server.hasArg("smartbeaconing");
  if (server.hasArg("busymode")) {
    int v = server.arg("busymode").toInt();
    if (v >= 0 && v <= 1) busyMode = v;
  }
  if (server.hasArg("busythr")) {
    int v = server.arg("busythr").toInt();
    if (v >= 10 && v <= 2000) busyAdcThreshold = v;
  }
  if (server.hasArg("busywin")) {
    unsigned long v = strtoul(server.arg("busywin").c_str(), nullptr, 10);
    if (v >= 5 && v <= 200) busySampleWindowMs = v;
  }
  if (server.hasArg("busyhang")) {
    unsigned long v = strtoul(server.arg("busyhang").c_str(), nullptr, 10);
    if (v >= 20 && v <= 3000) busyHangMs = v;
  }
  if (server.hasArg("busyhold")) {
    unsigned long v = strtoul(server.arg("busyhold").c_str(), nullptr, 10);
    if (v >= 50 && v <= 5000) busyClearHoldMs = v;
  }
  if (server.hasArg("busyretry")) {
    unsigned long v = strtoul(server.arg("busyretry").c_str(), nullptr, 10);
    if (v >= 500 && v <= 60000) busyRetryMs = v;
  }
  if (server.hasArg("busymaxw")) {
    unsigned long v = strtoul(server.arg("busymaxw").c_str(), nullptr, 10);
    if (v >= 1000 && v <= 300000) busyMaxWaitMs = v;
  }
  if (server.hasArg("busycents")) {
    int v = server.arg("busycents").toInt();
    if (v >= 50 && v <= 3000) busyCenterCalSamples = v;
  }
  if (server.hasArg("busyrecal")) {
    unsigned long v = strtoul(server.arg("busyrecal").c_str(), nullptr, 10);
    if (v >= 1000 && v <= 120000) busyRecalMs = v;
  }

  savePrefs();
  calibrateBusyCenter(busyCenterCalSamples);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleTransmit() {
  if (!gps_valid) {
    server.send(400, "text/plain", "No valid GPS data");
    return;
  }
  if (canTransmitNow()) {
    sendPacket(PKT_FIXPOS_STATUS, false);
    last_tx_time = millis();
    pending_manual_tx = false;
    server.send(200, "text/plain", "Packet transmitted");
  } else {
    pending_manual_tx = true;
    pending_tx_since = millis();
    last_busy_retry_time = 0;
    server.send(200, "text/plain", "Channel busy from speaker audio, manual TX queued");
  }
}

void handleDummyTransmit() {
  if (canTransmitNow()) {
    sendPacket(PKT_FIXPOS_STATUS, true);
    last_tx_time = millis();
    pending_dummy_tx = false;
    server.send(200, "text/plain", "Dummy packet transmitted");
  } else {
    pending_dummy_tx = true;
    pending_tx_since = millis();
    last_busy_retry_time = 0;
    server.send(200, "text/plain", "Channel busy from speaker audio, dummy TX queued");
  }
}

// ========================= SETUP ============================
void setup() {
  pinMode(AFSK_PIN, OUTPUT);
  pinMode(PTT_PIN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(BUSY_IN_PIN, INPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(BUSY_IN_PIN, ADC_11db);

  digitalWrite(AFSK_PIN, LOW);
  digitalWrite(PTT_PIN, LOW);
  digitalWrite(STATUS_LED, HIGH);

  Serial.begin(115200);
  delay(1000); 

  Serial.println("\n=============================================");
  Serial.println("    APRS TRACKER ESP32-C3 SUPER MINI SIAP    ");
  Serial.println("=============================================");

  // Inisialisasi GPS sekarang di PIN 7 (RX) dan PIN 6 (TX)
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  loadPrefs();
  calibrateBusyCenter(busyCenterCalSamples);
  lastBusyCalMs = millis();

  WiFi.softAP("APRS-Tracker", "12345678");
  Serial.print("WiFi AP Aktif! Hubungkan HP ke SSID: APRS-Tracker\n");
  Serial.print("Buka browser ke IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("=============================================\n");

  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/transmit", HTTP_POST, handleTransmit);
  server.on("/dummy_transmit", HTTP_POST, handleDummyTransmit);
  server.begin();
}

// ========================= LOOP ============================
void loop() {
  server.handleClient();
  unsigned long now = millis();

  bool data_received = false;
  while (gpsSerial.available() > 0) {
    data_received = true;
    gps.encode(gpsSerial.read());
  }

  if (data_received) last_gps_data_time = now;

  if (now - last_gps_data_time > 3000) {
    gps_connection_status = "Not Connected";
  } else if (!gps_locked) {
    gps_connection_status = "Searching";
  } else {
    gps_connection_status = "Locked";
  }

  gps_locked = gps.location.isValid() && gps.satellites.isValid() && gps.satellites.value() >= 3;
  gps_valid = gps_locked;

  if (gps.location.isUpdated() && gps.location.isValid()) {
    updateCoordinates();
  }

  if (busyMode == 1 && (now - lastBusyCalMs >= busyRecalMs)) {
    bool noPending = !(pending_manual_tx || pending_dummy_tx || pending_auto_tx);
    if (noPending && !isChannelBusyRaw()) {
      calibrateBusyCenter(busyCenterCalSamples);
      lastBusyCalMs = now;
    }
  }

  updateBusyStatusText();

  unsigned long effective_interval = getEffectiveInterval();

  if (gps_valid && !pending_auto_tx && (now - last_tx_time >= effective_interval)) {
    pending_auto_tx = true;
    pending_tx_since = now;
    last_busy_retry_time = 0;
  }

  bool havePending = pending_manual_tx || pending_dummy_tx || pending_auto_tx;

  if (havePending) {
    if (canTransmitNow()) {
      if (pending_dummy_tx) {
        sendPacket(PKT_FIXPOS_STATUS, true);
        pending_dummy_tx = false;
      } else if (pending_manual_tx) {
        sendPacket(PKT_FIXPOS_STATUS, false);
        pending_manual_tx = false;
      } else if (pending_auto_tx && gps_valid) {
        sendPacket(PKT_FIXPOS_STATUS, false);
        pending_auto_tx = false;
      }

      last_tx_time = millis();
      pending_tx_since = 0;
      last_busy_retry_time = 0;
    } else {
      if (now - pending_tx_since >= busyMaxWaitMs) {
        Serial.println(">>> TX BATAL: Frekuensi radio terlalu lama sibuk! <<<");
        pending_manual_tx = false;
        pending_dummy_tx = false;
        pending_auto_tx = false;
        pending_tx_since = 0;
        last_busy_retry_time = 0;
        last_tx_time = now;
      } else if (now - last_busy_retry_time >= busyRetryMs) {
        last_busy_retry_time = now;
      }
    }
  }

  // ===================== TAMPILAN MONITOR SERIAL =====================
  if (now - last_status_print >= 5000) {
    Serial.println("\n--- [ STATUS MONITOR APRS ] ---");
    
    Serial.printf("WiFi AP    : Aktif | Klien Terhubung: %d\n", WiFi.softAPgetStationNum());
    
    Serial.printf("GPS Status : %s\n", gps_connection_status.c_str());
    if (gps_valid) {
      Serial.printf("Satelit    : %d Satelit\n", gps.satellites.value());
      Serial.printf("Koordinat  : %s / %s\n", lat, lon);
      Serial.printf("Kecepatan  : %.1f km/h\n", gps.speed.kmph());
    } else {
      Serial.println("Koordinat  : Menunggu Lock Satelit...");
    }

    Serial.printf("Freq Radio : %s (Level Suara: %d, Threshold: %d)\n", 
                  channel_busy_status.c_str(), lastBusyPeak, busyAdcThreshold);

    long next_tx = (effective_interval - (now - last_tx_time)) / 1000;
    if (next_tx < 0 || !gps_valid) next_tx = 0;
    
    if (havePending) {
       Serial.println("Kirim APRS : MENUNGGU FREKUENSI KOSONG...");
    } else if (gps_valid) {
       Serial.printf("Kirim APRS : %ld Detik lagi\n", next_tx);
    } else {
       Serial.println("Kirim APRS : Ditunda (Menunggu GPS)");
    }
    
    Serial.println("-------------------------------");
    last_status_print = now;
  }
}
