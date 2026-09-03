// Toshiba Suzumi/Shorai/Seiya AC <-> MQTT/Home Assistant bridge.
// ESP32-C3, raw Arduino C++, no ESPHome — same pattern as power-switch1,
// temp-humidity1, door-sensor1.
//
// Protocol reverse-engineered by pedobry:
//   https://github.com/pedobry/esphome_toshiba_suzumi (GPL-3.0)
// Ported into toshiba_protocol.h / toshiba_link.h without the ESPHome
// runtime; this file adds WiFi/MQTT/HA-discovery on top, matching the
// established project pattern (espMqttClient w/ backoff, retained HA
// discovery configs, LWT availability, periodic WiFi signal, reset reason
// on connect, MQTT fail-count diagnostic, static IP, manufacturer P@cho).

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <espMqttClient.h>
#include <Adafruit_NeoPixel.h>
#include <WebServer.h>
#include <Preferences.h>
#include "config.h"
#include "toshiba_protocol.h"
#include "toshiba_link.h"

using namespace toshiba;

// ---------------- Topics ----------------
static const String BASE = String(DEVICE_ID) + "/";
static const String T_AVAIL = BASE + "status";
static const String T_MODE_STATE = BASE + "mode/state";
static const String T_MODE_CMD = BASE + "mode/set";
static const String T_POWER_CMD = BASE + "power/set";
static const String T_TEMP_STATE = BASE + "temperature/state";
static const String T_TEMP_CMD = BASE + "temperature/set";
static const String T_CURTEMP_STATE = BASE + "current_temperature/state";
static const String T_FAN_STATE = BASE + "fan_mode/state";
static const String T_FAN_CMD = BASE + "fan_mode/set";
static const String T_SWING_STATE = BASE + "swing_mode/state";
static const String T_SWING_CMD = BASE + "swing_mode/set";
static const String T_PRESET_STATE = BASE + "preset_mode/state";
static const String T_PRESET_CMD = BASE + "preset_mode/set";
static const String T_OUTDOOR_TEMP = BASE + "outdoor_temperature/state";
static const String T_PWRLEVEL_STATE = BASE + "power_level/state";
static const String T_PWRLEVEL_CMD = BASE + "power_level/set";
static const String T_WIFI_RSSI = BASE + "wifi_signal/state";
static const String T_RESET_REASON = BASE + "reset_reason/state";
static const String T_MQTT_FAILS = BASE + "mqtt_fail_count/state";
static const String T_OTA_RESTART_CMD = BASE + "ota_restart/set";

// ---------------- Globals ----------------
HardwareSerial &acSerial = Serial1;
ToshibaLink acLink(acSerial);
espMqttClient mqttClient;
// NVS-backed persistence of AC state, so a power loss to the AC (which
// also power-cycles this board, since it draws from the AC's own supply)
// doesn't leave the AC in an unknown state or require manually re-setting
// everything from HA. Namespace kept short/specific to avoid any collision
// with other NVS users on this device.
Preferences prefs;
#if WEB_SERVER_ENABLED
WebServer webServer(80);
#endif

// dbg mirrors every debug Serial.print/println call in this file out to a
// raw TCP stream too, so logs are reachable once the board is sealed inside
// the AC with no USB access. A single Print::write(uint8_t) override is
// enough — Print's own print()/println() overloads (int, HEX-formatted,
// String, etc.) all funnel through write() internally, so this covers
// every call site below without needing per-type overloads. Connect with a
// plain `telnet <device-ip> <NETLOG_PORT>` (or `nc <device-ip>
// <NETLOG_PORT>`) — see the README for why this stream is intentionally
// unauthenticated and what that does and doesn't expose.
#if NETLOG_ENABLED
WiFiServer netLogServer(NETLOG_PORT);
WiFiClient netLogClient;
#endif

class DebugPrint : public Print {
 public:
  size_t write(uint8_t c) override {
#if NETLOG_ENABLED
    if (netLogClient && netLogClient.connected()) netLogClient.write(c);
#endif
    return Serial.write(c);
  }
  size_t write(const uint8_t *buffer, size_t size) override {
#if NETLOG_ENABLED
    if (netLogClient && netLogClient.connected()) netLogClient.write(buffer, size);
#endif
    return Serial.write(buffer, size);
  }
};
DebugPrint dbg;

#if NETLOG_ENABLED
void netLogUpdate() {
  if (netLogServer.hasClient()) {
    if (netLogClient && netLogClient.connected()) netLogClient.stop();
    netLogClient = netLogServer.accept();
    dbg.println("[netlog] client connected, mirroring " DEVICE_ID " serial output");
  }
}
#endif

#if USE_STATUS_LED
Adafruit_NeoPixel statusLed(1, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);
uint32_t ledLastToggleMs = 0;
bool ledBlinkOn = false;

void ledSet(uint8_t r, uint8_t g, uint8_t b) {
  statusLed.setPixelColor(0, statusLed.Color(r, g, b));
  statusLed.show();
}

// Blocking on purpose — only called once at the very start of setup(),
// before WiFi/MQTT/UART are touched, so there's nothing to hold up yet.
void ledBootBlink() {
  for (int i = 0; i < 3; i++) {
    ledSet(30, 30, 30);
    delay(120);
    ledSet(0, 0, 0);
    delay(120);
  }
}

// Non-blocking, called every loop() iteration:
//   no WiFi yet      -> blinking blue
//   WiFi, no MQTT yet -> solid blue
//   WiFi + MQTT       -> solid green
void ledUpdate(bool wifiConnected, bool mqttConnected) {
  uint32_t now = millis();
  if (!wifiConnected) {
    if (now - ledLastToggleMs > 400) {
      ledLastToggleMs = now;
      ledBlinkOn = !ledBlinkOn;
      ledSet(0, 0, ledBlinkOn ? 30 : 0);
    }
    return;
  }
  if (!mqttConnected) {
    ledSet(0, 0, 30);
    return;
  }
  ledSet(0, 30, 0);
}
#endif

uint32_t mqttFailCount = 0;
uint32_t lastReconnectAttempt = 0;
uint32_t reconnectBackoffMs = 1000;
static const uint32_t RECONNECT_BACKOFF_MAX_MS = 60000;

uint32_t lastWifiPublish = 0;
uint32_t lastPoll = 0;
static const uint32_t POLL_INTERVAL_MS = 30000;   // room/outdoor temp watchdog poll
static const uint32_t WIFI_PUBLISH_INTERVAL_MS = 60000;

// Locally tracked AC state. It is updated only from validated AC responses;
// command handlers enqueue a readback instead of optimistically publishing.
bool acOn = false;
SpecialMode currentSpecialMode = SpecialMode::STANDARD;
// Last non-off mode the AC actually reported, used to resume the correct
// mode on a power-on (see handlePower()). Deliberately updated only from
// real AC readbacks, not from commands we send — that way it stays correct
// even if the mode was last changed via the unit's own physical remote,
// not just via this firmware. Defaults to COOL until the very first
// readback arrives (which happens within seconds of boot regardless, via
// the initial Cmd::MODE request in setup()).
Mode lastActiveMode = Mode::COOL;
String acModeState = "unknown";
String acTargetTempState = "unknown";
String acFanState = "unknown";
String acSwingState = "unknown";
String acPresetState = "unknown";
String acPowerLevelState = "unknown";

// ---------------- Helpers: protocol value <-> HA string ----------------
const char *mode_to_str(Mode m) {
  switch (m) {
    case Mode::HEAT_COOL: return "heat_cool";
    case Mode::COOL: return "cool";
    case Mode::HEAT: return "heat";
    case Mode::DRY: return "dry";
    case Mode::FAN_ONLY: return "fan_only";
  }
  return "off";
}
bool str_to_mode(const String &s, Mode &out) {
  if (s == "heat_cool") { out = Mode::HEAT_COOL; return true; }
  if (s == "cool") { out = Mode::COOL; return true; }
  if (s == "heat") { out = Mode::HEAT; return true; }
  if (s == "dry") { out = Mode::DRY; return true; }
  if (s == "fan_only") { out = Mode::FAN_ONLY; return true; }
  return false;
}

const char *fan_to_str(Fan f) {
  switch (f) {
    case Fan::AUTO: return "auto";
    case Fan::QUIET: return "quiet";
    case Fan::SPEED_LOW: return "low";
    case Fan::LEVEL_2: return "low_medium";
    case Fan::MEDIUM: return "medium";
    case Fan::LEVEL_4: return "medium_high";
    case Fan::SPEED_HIGH: return "high";
  }
  return "auto";
}
bool str_to_fan(const String &s, Fan &out) {
  if (s == "auto") { out = Fan::AUTO; return true; }
  if (s == "quiet") { out = Fan::QUIET; return true; }
  if (s == "low") { out = Fan::SPEED_LOW; return true; }
  if (s == "low_medium") { out = Fan::LEVEL_2; return true; }
  if (s == "medium") { out = Fan::MEDIUM; return true; }
  if (s == "medium_high") { out = Fan::LEVEL_4; return true; }
  if (s == "high") { out = Fan::SPEED_HIGH; return true; }
  return false;
}

const char *swing_to_str(Swing s) {
  if (s == Swing::VERTICAL || s == Swing::BOTH) return "vertical";
  return "off";
}
Swing str_to_swing(const String &s) { return s == "vertical" ? Swing::VERTICAL : Swing::OFF; }

const char *special_to_preset(SpecialMode m) {
  switch (m) {
    case SpecialMode::HI_POWER: return "hi_power";
    case SpecialMode::ECO: return "eco";
    case SpecialMode::EIGHT_DEG: return "8_degrees";
    case SpecialMode::FLOOR: return "floor";
    default: return "none";
  }
}
bool preset_to_special(const String &s, SpecialMode &out) {
  if (s == "hi_power") { out = SpecialMode::HI_POWER; return true; }
  if (s == "eco") { out = SpecialMode::ECO; return true; }
  if (s == "8_degrees") { out = SpecialMode::EIGHT_DEG; return true; }
  if (s == "floor") { out = SpecialMode::FLOOR; return true; }
  if (s == "none") { out = SpecialMode::STANDARD; return true; }
  return false;
}

const char *pwrlevel_to_str(PowerLevel p) {
  switch (p) {
    case PowerLevel::PCT_50: return "50%";
    case PowerLevel::PCT_75: return "75%";
    case PowerLevel::PCT_100: return "100%";
  }
  return "100%";
}
bool str_to_pwrlevel(const String &s, PowerLevel &out) {
  if (s == "50%") { out = PowerLevel::PCT_50; return true; }
  if (s == "75%") { out = PowerLevel::PCT_75; return true; }
  if (s == "100%") { out = PowerLevel::PCT_100; return true; }
  return false;
}

bool parseTargetTemp(const String &s, uint8_t &out) {
  // Home Assistant's MQTT climate integration sends temperature commands
  // as a float string (e.g. "27.0") even with temp_step=1 — that's normal
  // HA behavior, not malformed input. The AC protocol only stores whole
  // degrees, so accept and discard an optional fractional part rather
  // than rejecting it outright.
  int dot = s.indexOf('.');
  String intPart = (dot == -1) ? s : s.substring(0, dot);

  if (intPart.length() == 0 || intPart.length() > 2) return false;
  uint8_t value = 0;
  for (size_t i = 0; i < intPart.length(); i++) {
    char c = intPart[i];
    if (c < '0' || c > '9') return false;
    value = value * 10 + (c - '0');
  }
  // Toshiba supports either FrostGuard's 5-13 C range or normal 17-30 C.
  if ((value >= EIGHT_DEG_MIN_TEMP && value <= EIGHT_DEG_MAX_TEMP) ||
      (value >= MIN_TEMP_STANDARD && value <= MAX_TEMP)) {
    out = value;
    return true;
  }
  return false;
}

const char *reset_reason_str() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power_on";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_watchdog";
    case ESP_RST_TASK_WDT: return "task_watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_DEEPSLEEP: return "deep_sleep";
    default: return "unknown";
  }
}

// ---------------- MQTT publish helpers ----------------
void publishRetained(const String &topic, const String &payload) {
#if DEBUG_MODE
  dbg.print("[mqtt>] ");
  dbg.print(topic);
  dbg.print(" = ");
  dbg.println(payload);
#endif
  uint16_t packetId = mqttClient.publish(topic.c_str(), 0, true, payload.c_str());
  if (packetId == 0) {
    dbg.print("[mqtt] publish FAILED (likely low memory), topic=");
    dbg.print(topic);
    dbg.print(" payload_len=");
    dbg.print(payload.length());
    dbg.print(" free_heap=");
    dbg.println(ESP.getFreeHeap());
  }
}

// ---------------- HA discovery ----------------
// Removes entities left behind by earlier versions of this firmware.
// Simply no longer *publishing* a discovery config doesn't delete
// anything in Home Assistant — the old retained config is still sitting
// on the broker, so the entity stays registered forever showing
// "Unknown" once nothing publishes its state anymore. The MQTT-discovery
// way to actually remove an entity is an empty retained payload on its
// own config topic. Safe to run on every connect: once the broker's
// retained topic is already empty, this is a no-op.
void cleanupRemovedDiscoveryEntities() {
  const char *removedTopics[] = {
      "homeassistant/select/" DEVICE_ID "/air_outlet/config",
      "homeassistant/sensor/" DEVICE_ID "/compressor_load/config",
      "homeassistant/sensor/" DEVICE_ID "/compressor_current/config",
      "homeassistant/sensor/" DEVICE_ID "/cdu_td_temp/config",
      "homeassistant/sensor/" DEVICE_ID "/cdu_ts_temp/config",
      "homeassistant/sensor/" DEVICE_ID "/cdu_te_temp/config",
      "homeassistant/sensor/" DEVICE_ID "/fcu_tc_temp/config",
      "homeassistant/sensor/" DEVICE_ID "/fcu_tcj_temp/config",
      "homeassistant/sensor/" DEVICE_ID "/fcu_fan_rpm/config",
      "homeassistant/sensor/" DEVICE_ID "/energy_daily/config",
      "homeassistant/sensor/" DEVICE_ID "/power_draw/config",
  };
  for (const char *topic : removedTopics) {
    mqttClient.publish(topic, 0, true, "");
  }
#if DEBUG_MODE
  dbg.print("[mqtt] cleaned up ");
  dbg.print((int) (sizeof(removedTopics) / sizeof(removedTopics[0])));
  dbg.println(" stale discovery entities");
#endif
}

void publishDiscovery() {
  String deviceBlock =
      String("\"device\":{\"identifiers\":[\"") + DEVICE_ID + "\"],\"manufacturer\":\"P@cho\"," +
      "\"model\":\"Toshiba AC Bridge\",\"name\":\"" + DEVICE_NAME + "\"}";
  String avail = String("\"availability_topic\":\"") + T_AVAIL + "\"";

  // Climate entity
  {
    String cfg = "{";
    cfg += "\"name\":null,\"unique_id\":\"" + String(DEVICE_ID) + "_climate\",";
    cfg += avail + ",";
    cfg += "\"mode_command_topic\":\"" + T_MODE_CMD + "\",\"mode_state_topic\":\"" + T_MODE_STATE + "\",";
    cfg += "\"modes\":[\"off\",\"cool\",\"heat\",\"heat_cool\",\"dry\",\"fan_only\"],";
    // power_command_topic decouples HA's power toggle from mode selection
    // entirely — without it, HA's generic turn_on() fallback just picks the
    // first non-off entry in "modes" above, which is how a power-button
    // press was turning the AC on in Auto instead of resuming the last
    // mode it was actually in. handlePower() below decides what "ON" means.
    cfg += "\"power_command_topic\":\"" + T_POWER_CMD + "\",";
    cfg += "\"temperature_command_topic\":\"" + T_TEMP_CMD + "\",\"temperature_state_topic\":\"" + T_TEMP_STATE + "\",";
    cfg += "\"current_temperature_topic\":\"" + T_CURTEMP_STATE + "\",";
    cfg += "\"fan_mode_command_topic\":\"" + T_FAN_CMD + "\",\"fan_mode_state_topic\":\"" + T_FAN_STATE + "\",";
    cfg += "\"fan_modes\":[\"auto\",\"quiet\",\"low\",\"low_medium\",\"medium\",\"medium_high\",\"high\"],";
    cfg += "\"swing_mode_command_topic\":\"" + T_SWING_CMD + "\",\"swing_mode_state_topic\":\"" + T_SWING_STATE + "\",";
    cfg += "\"swing_modes\":[\"off\",\"vertical\"],";
    cfg += "\"preset_mode_command_topic\":\"" + T_PRESET_CMD + "\",\"preset_mode_state_topic\":\"" + T_PRESET_STATE + "\",";
    cfg += "\"preset_modes\":[\"eco\",\"hi_power\",\"8_degrees\",\"floor\"],";
    cfg += "\"min_temp\":5,\"max_temp\":30,\"temp_step\":1,";
    cfg += deviceBlock;
    cfg += "}";
    publishRetained("homeassistant/climate/" + String(DEVICE_ID) + "/config", cfg);
  }
  // Outdoor temp sensor
  {
    String cfg = "{\"name\":\"Outdoor Temperature\",\"unique_id\":\"" + String(DEVICE_ID) + "_outdoor_temp\",";
    cfg += avail + ",\"state_topic\":\"" + T_OUTDOOR_TEMP + "\",\"device_class\":\"temperature\",";
    cfg += "\"unit_of_measurement\":\"°C\",\"state_class\":\"measurement\"," + deviceBlock + "}";
    publishRetained("homeassistant/sensor/" + String(DEVICE_ID) + "/outdoor_temp/config", cfg);
  }
  // Power level select
  {
    String cfg = "{\"name\":\"Power Level\",\"unique_id\":\"" + String(DEVICE_ID) + "_power_level\",";
    cfg += avail + ",\"state_topic\":\"" + T_PWRLEVEL_STATE + "\",\"command_topic\":\"" + T_PWRLEVEL_CMD + "\",";
    cfg += "\"options\":[\"50%\",\"75%\",\"100%\"],\"entity_category\":\"config\"," + deviceBlock + "}";
    publishRetained("homeassistant/select/" + String(DEVICE_ID) + "/power_level/config", cfg);
  }
  // Diagnostics
  {
    String cfg = "{\"name\":\"WiFi Signal\",\"unique_id\":\"" + String(DEVICE_ID) + "_wifi_signal\",";
    cfg += avail + ",\"state_topic\":\"" + T_WIFI_RSSI + "\",\"device_class\":\"signal_strength\",";
    cfg += "\"unit_of_measurement\":\"dBm\",\"entity_category\":\"diagnostic\",\"state_class\":\"measurement\"," + deviceBlock + "}";
    publishRetained("homeassistant/sensor/" + String(DEVICE_ID) + "/wifi_signal/config", cfg);
  }
  {
    String cfg = "{\"name\":\"Reset Reason\",\"unique_id\":\"" + String(DEVICE_ID) + "_reset_reason\",";
    cfg += avail + ",\"state_topic\":\"" + T_RESET_REASON + "\",\"entity_category\":\"diagnostic\"," + deviceBlock + "}";
    publishRetained("homeassistant/sensor/" + String(DEVICE_ID) + "/reset_reason/config", cfg);
  }
  {
    String cfg = "{\"name\":\"MQTT Fail Count\",\"unique_id\":\"" + String(DEVICE_ID) + "_mqtt_fails\",";
    cfg += avail + ",\"state_topic\":\"" + T_MQTT_FAILS + "\",\"entity_category\":\"diagnostic\",\"state_class\":\"total_increasing\"," + deviceBlock + "}";
    publishRetained("homeassistant/sensor/" + String(DEVICE_ID) + "/mqtt_fails/config", cfg);
  }
  {
    String cfg = "{\"name\":\"OTA Restart\",\"unique_id\":\"" + String(DEVICE_ID) + "_ota_restart\",";
    cfg += avail + ",\"command_topic\":\"" + T_OTA_RESTART_CMD + "\",\"entity_category\":\"config\"," + deviceBlock + "}";
    publishRetained("homeassistant/button/" + String(DEVICE_ID) + "/ota_restart/config", cfg);
  }
}

// ---------------- AC state -> MQTT ----------------
void onAcState(const DecodedState &st) {
#if DEBUG_MODE
  // Logged here (raw decoded value) separately from publishRetained's own
  // debug line, since some fields (e.g. mode while the unit is off) get
  // decoded but deliberately not published — this line fires regardless.
  if (st.has_power_state) { dbg.print("[ac] power_state="); dbg.println(st.power_state == PowerState::ON ? "ON" : "OFF"); }
  if (st.has_mode) { dbg.print("[ac] mode="); dbg.println(mode_to_str(st.mode)); }
  if (st.has_target_temp) { dbg.print("[ac] target_temp_raw="); dbg.println(st.target_temp); }
  if (st.has_room_temp) { dbg.print("[ac] room_temp="); dbg.println(st.room_temp); }
  if (st.has_outdoor_temp) { dbg.print("[ac] outdoor_temp="); dbg.println(st.outdoor_temp); }
  if (st.has_fan) { dbg.print("[ac] fan="); dbg.println(fan_to_str(st.fan)); }
  if (st.has_swing) { dbg.print("[ac] swing="); dbg.println(swing_to_str(st.swing)); }
  if (st.has_power_level) { dbg.print("[ac] power_level="); dbg.println(pwrlevel_to_str(st.power_level)); }
  if (st.has_special_mode) { dbg.print("[ac] special_mode="); dbg.println((int) st.special_mode); }
#endif
  if (st.has_power_state) {
    bool wasOn = acOn;
    acOn = (st.power_state == PowerState::ON);
    prefs.putBool("on", acOn);
    prefs.putBool("valid", true);
    if (!acOn) {
      acModeState = "off";
      publishRetained(T_MODE_STATE, acModeState);
    } else if (!wasOn) {
      // Power just turned on without necessarily including a fresh MODE
      // frame in the same burst — likely via the AC's own physical remote,
      // since our own commands always set MODE alongside POWER_STATE. Left
      // alone, mode_state would just keep showing "off" forever, since the
      // only other place that publishes it is triggered by an actual MODE
      // frame arriving. Publish our best-known mode immediately so HA
      // isn't stuck stale, then request a fresh readback to confirm/
      // correct it — self-corrects within ~100-200ms if this guess turns
      // out to be wrong (e.g. very first boot before any mode is known).
      acModeState = mode_to_str(lastActiveMode);
      publishRetained(T_MODE_STATE, acModeState);
      acLink.request(Cmd::MODE);
    }
  }
  if (st.has_mode && acOn) {
    acModeState = mode_to_str(st.mode);
    publishRetained(T_MODE_STATE, acModeState);
    lastActiveMode = st.mode;
    prefs.putUChar("mode", (uint8_t) st.mode);
    prefs.putUChar("lastmode", (uint8_t) st.mode);
  }
  if (st.has_target_temp) {
    uint8_t t = st.target_temp;
    if (currentSpecialMode == SpecialMode::EIGHT_DEG) t -= SPECIAL_TEMP_OFFSET;
    acTargetTempState = String(t);
    publishRetained(T_TEMP_STATE, acTargetTempState);
    prefs.putUChar("temp", t);
  }
  if (st.has_room_temp) {
    publishRetained(T_CURTEMP_STATE, String(st.room_temp));
  }
  if (st.has_outdoor_temp) {
    publishRetained(T_OUTDOOR_TEMP, String(st.outdoor_temp));
  }
  if (st.has_fan) {
    acFanState = fan_to_str(st.fan);
    publishRetained(T_FAN_STATE, acFanState);
    prefs.putUChar("fan", (uint8_t) st.fan);
  }
  if (st.has_swing) {
    acSwingState = swing_to_str(st.swing);
    publishRetained(T_SWING_STATE, acSwingState);
    prefs.putUChar("swing", (uint8_t) st.swing);
  }
  if (st.has_power_level) {
    acPowerLevelState = pwrlevel_to_str(st.power_level);
    publishRetained(T_PWRLEVEL_STATE, acPowerLevelState);
    prefs.putUChar("pwrlvl", (uint8_t) st.power_level);
  }
  if (st.has_special_mode) {
    currentSpecialMode = st.special_mode;
    acPresetState = special_to_preset(st.special_mode);
    publishRetained(T_PRESET_STATE, acPresetState);
    prefs.putUChar("special", (uint8_t) st.special_mode);
  }
}

// ---------------- MQTT command handling ----------------
void handleMode(const String &val) {
  if (val == "off") {
    acLink.set(Cmd::POWER_STATE, (uint8_t) PowerState::OFF);
    acLink.request(Cmd::POWER_STATE);
    return;
  }
  Mode m;
  if (!str_to_mode(val, m)) return;
  if (!acOn) {
    acLink.set(Cmd::POWER_STATE, (uint8_t) PowerState::ON);
  }
  acLink.set(Cmd::MODE, (uint8_t) m);
  acLink.request(Cmd::POWER_STATE);
  acLink.request(Cmd::MODE);
}

// Handles HA's dedicated power toggle (power_command_topic), decoupled from
// mode selection — see the comment on that discovery field for why this
// exists. "ON" resumes lastActiveMode rather than leaving the AC's own
// power-on default (observed to be Auto) or HA's mode-list-order fallback
// to decide. "OFF" is identical to handleMode("off").
void handlePower(const String &val) {
  if (val == "OFF") {
    acLink.set(Cmd::POWER_STATE, (uint8_t) PowerState::OFF);
    acLink.request(Cmd::POWER_STATE);
    return;
  }
  if (val != "ON") return;
  acLink.set(Cmd::POWER_STATE, (uint8_t) PowerState::ON);
  acLink.set(Cmd::MODE, (uint8_t) lastActiveMode);
  acLink.request(Cmd::POWER_STATE);
  acLink.request(Cmd::MODE);
}

void handleTargetTemp(const String &val) {
  uint8_t t;
  if (!parseTargetTemp(val, t)) {
    dbg.println("[mqtt] rejected invalid target temperature");
    return;
  }
  // Mirror the frost-guard auto-switch behaviour from upstream: crossing the
  // MIN_TEMP_STANDARD boundary flips special mode between STANDARD/EIGHT_DEG.
  SpecialMode desiredSpecialMode = currentSpecialMode;
  bool changed = false;
  if (t >= MIN_TEMP_STANDARD && desiredSpecialMode == SpecialMode::EIGHT_DEG) {
    desiredSpecialMode = SpecialMode::STANDARD;
    changed = true;
  } else if (t < MIN_TEMP_STANDARD && desiredSpecialMode != SpecialMode::EIGHT_DEG) {
    desiredSpecialMode = SpecialMode::EIGHT_DEG;
    changed = true;
  }
  if (changed) {
    acLink.set(Cmd::SPECIAL_MODE, (uint8_t) desiredSpecialMode);
  }
  uint8_t wireTemp = t;
  if (desiredSpecialMode == SpecialMode::EIGHT_DEG) wireTemp += SPECIAL_TEMP_OFFSET;
  acLink.set(Cmd::TARGET_TEMP, wireTemp);
  // Read special mode before target temp so the FrostGuard offset is known.
  acLink.request(Cmd::SPECIAL_MODE);
  acLink.request(Cmd::TARGET_TEMP);
}

void handleFan(const String &val) {
  Fan f;
  if (!str_to_fan(val, f)) return;
  acLink.set(Cmd::FAN, (uint8_t) f);
  acLink.request(Cmd::FAN);
}

void handleSwing(const String &val) {
  Swing s = str_to_swing(val);
  acLink.set(Cmd::SWING, (uint8_t) s);
  acLink.request(Cmd::SWING);
}

void handlePreset(const String &val) {
  SpecialMode sm;
  if (!preset_to_special(val, sm)) return;
  acLink.set(Cmd::SPECIAL_MODE, (uint8_t) sm);
  acLink.request(Cmd::SPECIAL_MODE);
  acLink.request(Cmd::TARGET_TEMP);
}

void handlePowerLevel(const String &val) {
  PowerLevel p;
  if (!str_to_pwrlevel(val, p)) return;
  acLink.set(Cmd::POWER_SEL, (uint8_t) p);
  acLink.request(Cmd::POWER_SEL);
}

String mqttRxTopic;
String mqttRxPayload;
size_t mqttRxTotal = 0;
size_t mqttRxNextIndex = 0;
static const size_t MAX_MQTT_COMMAND_PAYLOAD = 64;

void dispatchMqttCommand(const String &topic, const String &payload) {
  if (topic == T_MODE_CMD) handleMode(payload);
  else if (topic == T_POWER_CMD) handlePower(payload);
  else if (topic == T_TEMP_CMD) handleTargetTemp(payload);
  else if (topic == T_FAN_CMD) handleFan(payload);
  else if (topic == T_SWING_CMD) handleSwing(payload);
  else if (topic == T_PRESET_CMD) handlePreset(payload);
  else if (topic == T_PWRLEVEL_CMD) handlePowerLevel(payload);
  else if (topic == T_OTA_RESTART_CMD) ESP.restart();
}

void onMqttMessage(const espMqttClientTypes::MessageProperties &props, const char *topic, const uint8_t *payload,
                    size_t len, size_t index, size_t total) {
  // espMqttClient may split a publish across callbacks. Assemble it before
  // interpreting it so a partial value can never change AC state.
  if (index == 0) {
    if (total > MAX_MQTT_COMMAND_PAYLOAD) {
      dbg.println("[mqtt] discarded oversized command payload");
      return;
    }
    mqttRxTopic = topic;
    mqttRxPayload = "";
    mqttRxPayload.reserve(total);
    mqttRxTotal = total;
    mqttRxNextIndex = 0;
  }
  if (mqttRxTopic != topic || total != mqttRxTotal || index != mqttRxNextIndex || index > total || len > total - index) {
    dbg.println("[mqtt] discarded malformed fragmented message");
    mqttRxTopic = "";
    mqttRxPayload = "";
    mqttRxTotal = mqttRxNextIndex = 0;
    return;
  }
  for (size_t i = 0; i < len; i++) mqttRxPayload += (char) payload[i];
  mqttRxNextIndex += len;
  if (mqttRxNextIndex != mqttRxTotal) return;

  String t = mqttRxTopic;
  String v = mqttRxPayload;
  mqttRxTopic = "";
  mqttRxPayload = "";
  mqttRxTotal = mqttRxNextIndex = 0;

#if DEBUG_MODE
  dbg.print("[mqtt<] ");
  dbg.print(t);
  dbg.print(" = ");
  dbg.println(v);
#endif

  dispatchMqttCommand(t, v);
}

#if DEBUG_MODE && DEBUG_SIMULATE_MQTT
struct SimulatedMqttCommand {
  const char *topic_suffix;
  const char *payload;
};

static const SimulatedMqttCommand SIMULATED_MQTT_COMMANDS[] = {
    {"mode/set", "cool"},
    {"temperature/set", "24"},
    // HA's MQTT climate sends floats even at temp_step=1 — this is the
    // format that actually arrives from Home Assistant in practice, and
    // the one the old validator used to wrongly reject.
    {"temperature/set", "24.0"},
    {"fan_mode/set", "medium"},
    {"swing_mode/set", "vertical"},
    {"power_level/set", "75%"},
    {"preset_mode/set", "eco"},
    // Deliberately invalid: confirms that input validation rejects 14-16 C.
    {"temperature/set", "16"},
    {"temperature/set", "16.0"},
    {"mode/set", "off"},
    // Power toggle: should resume "cool" (the last mode set above), not
    // whatever HA's mode-list ordering would otherwise default to.
    {"power/set", "ON"},
    {"power/set", "OFF"},
};

void simulateMqttCommands() {
  static size_t commandIndex = 0;
  static uint32_t lastCommandMs = 0;
  static const uint32_t START_DELAY_MS = 5000;
  static const uint32_t COMMAND_INTERVAL_MS = 3000;
  static const uint32_t CYCLE_PAUSE_MS = 15000;

  uint32_t now = millis();
  if (now < START_DELAY_MS || now - lastCommandMs < COMMAND_INTERVAL_MS) return;

  const SimulatedMqttCommand &command = SIMULATED_MQTT_COMMANDS[commandIndex];
  String topic = BASE + command.topic_suffix;
  dbg.print("[mqtt SIM] ");
  dbg.print(topic);
  dbg.print(" = ");
  dbg.println(command.payload);
  dispatchMqttCommand(topic, command.payload);

  lastCommandMs = now;
  commandIndex++;
  if (commandIndex == sizeof(SIMULATED_MQTT_COMMANDS) / sizeof(SIMULATED_MQTT_COMMANDS[0])) {
    commandIndex = 0;
    // Leave a visible pause between test cycles.
    lastCommandMs += CYCLE_PAUSE_MS - COMMAND_INTERVAL_MS;
  }
}
#endif

#if WEB_SERVER_ENABLED
String htmlEscape(const String &value) {
  String escaped;
  escaped.reserve(value.length());
  for (size_t i = 0; i < value.length(); i++) {
    switch (value[i]) {
      case '&': escaped += "&amp;"; break;
      case '<': escaped += "&lt;"; break;
      case '>': escaped += "&gt;"; break;
      case '"': escaped += "&quot;"; break;
      default: escaped += value[i]; break;
    }
  }
  return escaped;
}

void handleWebRoot() {
  String page;
  page.reserve(5000);
  page += F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>");
  page += F("<title>Toshiba AC</title><style>body{font-family:system-ui,sans-serif;max-width:720px;margin:2rem auto;padding:0 1rem;background:#f5f7fa;color:#18212f}h1{margin-bottom:.2rem}.muted{color:#5e6b7a}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:1rem}.card{background:#fff;padding:1rem;border-radius:.7rem;box-shadow:0 1px 4px #0002}label{display:block;font-weight:600;margin:.4rem 0}select,input,button{font:inherit;padding:.55rem;width:100%;box-sizing:border-box}button{background:#1267b3;color:white;border:0;border-radius:.35rem;margin-top:.7rem;cursor:pointer}.danger{background:#b42318}dl{margin:0}dt{font-size:.8rem;color:#5e6b7a}dd{margin:0 0 .6rem;font-weight:600}</style></head><body>");
  page += F("<h1>"); page += htmlEscape(String(DEVICE_NAME)); page += F("</h1><p class='muted'>Local controls. Changes are confirmed only after the AC replies.</p>");
  page += F("<div class='grid'><section class='card'><h2>Status</h2><dl><dt>IP address</dt><dd>");
  page += htmlEscape(WiFi.localIP().toString()); page += F("</dd><dt>Wi-Fi RSSI</dt><dd>");
  page += String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0); page += F(" dBm</dd><dt>MQTT</dt><dd>");
  page += mqttClient.connected() ? "connected" : "disconnected"; page += F("</dd><dt>AC power / mode</dt><dd>");
  page += htmlEscape(acModeState); page += F("</dd><dt>Target / fan / swing</dt><dd>");
  page += htmlEscape(acTargetTempState); page += F(" °C / "); page += htmlEscape(acFanState); page += F(" / "); page += htmlEscape(acSwingState);
  page += F("</dd><dt>Preset / power limit</dt><dd>"); page += htmlEscape(acPresetState); page += F(" / "); page += htmlEscape(acPowerLevelState);
  page += F("</dd></dl><form method='post' action='/restart'><button class='danger'>Restart device</button></form></section>");
  page += F("<section class='card'><h2>Mode</h2><form method='post' action='/command'><label>Operating mode</label><select name='mode'><option value='off'>Off</option><option value='cool'>Cool</option><option value='heat'>Heat</option><option value='heat_cool'>Auto</option><option value='dry'>Dry</option><option value='fan_only'>Fan only</option></select><button>Apply mode</button></form></section>");
  page += F("<section class='card'><h2>Temperature</h2><form method='post' action='/command'><label>Setpoint (5-13 or 17-30 °C)</label><input name='temperature' type='number' min='5' max='30' step='1' required><button>Apply temperature</button></form></section>");
  page += F("<section class='card'><h2>Fan and swing</h2><form method='post' action='/command'><label>Fan</label><select name='fan'><option>auto</option><option>quiet</option><option>low</option><option value='low_medium'>low medium</option><option>medium</option><option value='medium_high'>medium high</option><option>high</option></select><button>Apply fan</button></form><form method='post' action='/command'><label>Swing</label><select name='swing'><option>off</option><option>vertical</option></select><button>Apply swing</button></form></section>");
  page += F("<section class='card'><h2>Preset and limit</h2><form method='post' action='/command'><label>Preset</label><select name='preset'><option>none</option><option>eco</option><option value='hi_power'>hi power</option><option value='8_degrees'>8 degrees</option><option>floor</option></select><button>Apply preset</button></form><form method='post' action='/command'><label>Power limit</label><select name='power_level'><option>50%</option><option>75%</option><option selected>100%</option></select><button>Apply limit</button></form></section></div></body></html>");
  webServer.send(200, "text/html; charset=utf-8", page);
}

void handleWebCommand() {
  const char *fieldNames[] = {"mode", "temperature", "fan", "swing", "preset", "power_level"};
  const String topics[] = {T_MODE_CMD, T_TEMP_CMD, T_FAN_CMD, T_SWING_CMD, T_PRESET_CMD, T_PWRLEVEL_CMD};
  for (size_t i = 0; i < sizeof(fieldNames) / sizeof(fieldNames[0]); i++) {
    if (!webServer.hasArg(fieldNames[i])) continue;
    String value = webServer.arg(fieldNames[i]);
    dbg.print("[web] ");
    dbg.print(topics[i]);
    dbg.print(" = ");
    dbg.println(value);
    dispatchMqttCommand(topics[i], value);
    webServer.sendHeader("Location", "/");
    webServer.send(303);
    return;
  }
  webServer.send(400, "text/plain", "Missing command value");
}

void handleWebRestart() {
  webServer.send(200, "text/plain", "Restarting...");
  delay(100);
  ESP.restart();
}

void setupWebServer() {
  webServer.on("/", HTTP_GET, handleWebRoot);
  webServer.on("/command", HTTP_POST, handleWebCommand);
  webServer.on("/restart", HTTP_POST, handleWebRestart);
  webServer.onNotFound([]() { webServer.send(404, "text/plain", "Not found"); });
  webServer.begin();
  dbg.println("[web] server started on port 80");
}
#endif

void onMqttConnect(bool sessionPresent) {
  dbg.println("[mqtt] connected");
  reconnectBackoffMs = 1000;
  mqttClient.subscribe(T_MODE_CMD.c_str(), 1);
  mqttClient.subscribe(T_POWER_CMD.c_str(), 1);
  mqttClient.subscribe(T_TEMP_CMD.c_str(), 1);
  mqttClient.subscribe(T_FAN_CMD.c_str(), 1);
  mqttClient.subscribe(T_SWING_CMD.c_str(), 1);
  mqttClient.subscribe(T_PRESET_CMD.c_str(), 1);
  mqttClient.subscribe(T_PWRLEVEL_CMD.c_str(), 1);
  mqttClient.subscribe(T_OTA_RESTART_CMD.c_str(), 1);

  publishRetained(T_AVAIL, "online");
  publishRetained(T_RESET_REASON, reset_reason_str());
  dbg.print("[mqtt] free_heap before discovery burst=");
  dbg.println(ESP.getFreeHeap());
  cleanupRemovedDiscoveryEntities();
  publishDiscovery();
  dbg.println("[mqtt] subscribed + discovery published");
}

void onMqttDisconnect(espMqttClientTypes::DisconnectReason reason) {
  mqttFailCount++;
  dbg.print("[mqtt] disconnected, reason=");
  dbg.println((int) reason);
}

const char *wifi_status_str(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS: return "idle";
    case WL_NO_SSID_AVAIL: return "no_ssid_avail";
    case WL_SCAN_COMPLETED: return "scan_completed";
    case WL_CONNECTED: return "connected";
    case WL_CONNECT_FAILED: return "connect_failed";
    case WL_CONNECTION_LOST: return "connection_lost";
    case WL_DISCONNECTED: return "disconnected";
    default: return "unknown";
  }
}

// WiFi.mode()/WiFi.config() only need to run once, before the first
// begin(). Calling them again while a connection attempt is still in
// flight makes the ESP-IDF WiFi stack log "sta is connecting, cannot set
// config" and does nothing useful — that's what was happening every 5s
// here before this got fixed. On a retry we just WiFi.disconnect() to
// cleanly abort whatever attempt is stuck, then begin() again.
bool wifiConfigured = false;

void connectWifi() {
  dbg.print("[wifi] connecting to ");
  dbg.print(WIFI_SSID);
  dbg.print(", prior status=");
  dbg.println(wifi_status_str(WiFi.status()));

  if (!wifiConfigured) {
    WiFi.mode(WIFI_STA);
#if USE_STATIC_IP
    IPAddress ip(STATIC_IP), gw(STATIC_GATEWAY), sn(STATIC_SUBNET), dns(STATIC_DNS);
    WiFi.config(ip, gw, sn, dns);
#endif
    wifiConfigured = true;
  } else {
    WiFi.disconnect();
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

#if DEBUG_MODE
void acHexDump(const char *tag, const std::vector<uint8_t> &bytes) {
  dbg.print("[ac ");
  dbg.print(tag);
  dbg.print("] ");
  for (uint8_t b : bytes) {
    if (b < 0x10) dbg.print('0');
    dbg.print(b, HEX);
    dbg.print(' ');
  }
  dbg.println();
}
#endif

void setup() {
  // Running at full 240MHz rather than the throttled-down clock this
  // project used earlier — restored per request.
  setCpuFrequencyMhz(240);

#if USE_STATUS_LED
  statusLed.begin();
  statusLed.setBrightness(40);  // WS2812 at full brightness is startlingly bright at close range
  ledBootBlink();
#endif

#if USE_EXTERNAL_ANTENNA
  pinMode(ANT_SW_PIN, OUTPUT);
  digitalWrite(ANT_SW_PIN, ANT_SW_EXTERNAL_LEVEL);
#endif

  Serial.begin(115200);
  delay(300);  // give the USB CDC a moment to enumerate before we print
  dbg.println();
  dbg.println("[boot] toshiba_ac_bridge starting, device_id=" DEVICE_ID);

  acSerial.begin(9600, SERIAL_8E1, AC_UART_RX_PIN, AC_UART_TX_PIN);
  dbg.println("[boot] AC UART started");

#if DEBUG_MODE
  acLink.set_debug_callback(acHexDump);
#endif

  connectWifi();

#if NETLOG_ENABLED
  netLogServer.begin();
  netLogServer.setNoDelay(true);
  dbg.print("[boot] netlog listening on port ");
  dbg.println(NETLOG_PORT);
#endif

#if WEB_SERVER_ENABLED
  setupWebServer();
#endif

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USER, MQTT_PASSWORD);
  mqttClient.setClientId(DEVICE_ID);
  mqttClient.setWill(T_AVAIL.c_str(), 0, true, "offline");
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);

  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.setHostname(DEVICE_ID);
  ArduinoOTA.begin();

  acLink.start_handshake();

  // Restore AC state saved from before a power loss. "valid" only becomes
  // true once a real AC readback has actually been saved (see onAcState()),
  // so this correctly does nothing on a genuinely first-ever boot with
  // empty NVS, rather than commanding the AC into some arbitrary default.
  // These SET commands are queued right after the handshake and before the
  // normal initial request-based pull below — the requests that follow
  // will read back and confirm whatever state actually resulted, which is
  // also what gets published to MQTT and re-saved, consistent with this
  // firmware's "never optimistically publish, always trust readback"
  // design elsewhere.
  prefs.begin("toshiba_ac", false);
  bool haveSavedState = prefs.getBool("valid", false);
  if (haveSavedState) {
    bool savedOn = prefs.getBool("on", false);
    Mode savedMode = (Mode) prefs.getUChar("mode", (uint8_t) Mode::COOL);
    uint8_t savedTemp = prefs.getUChar("temp", 24);
    Fan savedFan = (Fan) prefs.getUChar("fan", (uint8_t) Fan::AUTO);
    Swing savedSwing = (Swing) prefs.getUChar("swing", (uint8_t) Swing::OFF);
    PowerLevel savedPowerLevel = (PowerLevel) prefs.getUChar("pwrlvl", (uint8_t) PowerLevel::PCT_100);
    SpecialMode savedSpecial = (SpecialMode) prefs.getUChar("special", (uint8_t) SpecialMode::STANDARD);
    lastActiveMode = (Mode) prefs.getUChar("lastmode", (uint8_t) Mode::COOL);
    currentSpecialMode = savedSpecial;

    dbg.print("[boot] restoring saved AC state: on=");
    dbg.print(savedOn ? "yes" : "no");
    dbg.print(" mode=");
    dbg.print(mode_to_str(savedMode));
    dbg.print(" temp=");
    dbg.println(savedTemp);

    acLink.set(Cmd::POWER_STATE, (uint8_t)(savedOn ? PowerState::ON : PowerState::OFF));
    if (savedOn) {
      acLink.set(Cmd::MODE, (uint8_t) savedMode);
      uint8_t wireTemp = savedTemp;
      if (savedSpecial == SpecialMode::EIGHT_DEG) wireTemp += SPECIAL_TEMP_OFFSET;
      acLink.set(Cmd::TARGET_TEMP, wireTemp);
      acLink.set(Cmd::FAN, (uint8_t) savedFan);
      acLink.set(Cmd::SWING, (uint8_t) savedSwing);
      acLink.set(Cmd::POWER_SEL, (uint8_t) savedPowerLevel);
      acLink.set(Cmd::SPECIAL_MODE, (uint8_t) savedSpecial);
    }
  } else {
    dbg.println("[boot] no saved AC state yet (first boot)");
  }

  // Initial data pull
  acLink.request(Cmd::POWER_STATE);
  acLink.request(Cmd::MODE);
  // Read special mode before target temp so the FrostGuard offset is known.
  acLink.request(Cmd::SPECIAL_MODE);
  acLink.request(Cmd::TARGET_TEMP);
  acLink.request(Cmd::FAN);
  acLink.request(Cmd::POWER_SEL);
  acLink.request(Cmd::SWING);
  acLink.request(Cmd::ROOM_TEMP);
  acLink.request(Cmd::OUTDOOR_TEMP);
  // Wifi-module status LED on the AC's own display, off by default like a
  // dumb bridge should be.
  acLink.set(Cmd::WIFI_LED_1, 0x00);
  acLink.set(Cmd::WIFI_LED_2, 0x80);

  dbg.println("[boot] setup complete, entering loop");
}

void loop() {
  ArduinoOTA.handle();
  mqttClient.loop();
#if NETLOG_ENABLED
  netLogUpdate();
#endif
#if WEB_SERVER_ENABLED
  webServer.handleClient();
#endif
  acLink.loop(onAcState);

#if DEBUG_MODE && DEBUG_SIMULATE_MQTT
  simulateMqttCommands();
#endif

  uint32_t now = millis();

  static bool wasWifiConnected = false;
  bool isWifiConnected = (WiFi.status() == WL_CONNECTED);
  if (isWifiConnected && !wasWifiConnected) {
    dbg.print("[wifi] connected, ip=");
    dbg.println(WiFi.localIP().toString());
  } else if (!isWifiConnected && wasWifiConnected) {
    dbg.println("[wifi] link lost");
  }
  wasWifiConnected = isWifiConnected;

#if USE_STATUS_LED
  ledUpdate(isWifiConnected, mqttClient.connected());
#endif

#if DEBUG_MODE
  static uint32_t lastHeartbeatMs = 0;
  static const uint32_t HEARTBEAT_INTERVAL_MS = 10000;
  if (now - lastHeartbeatMs > HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = now;
    dbg.print("[hb] uptime_s=");
    dbg.print(now / 1000);
    dbg.print(" heap=");
    dbg.print(ESP.getFreeHeap());
    dbg.print(" rssi=");
    dbg.print(isWifiConnected ? WiFi.RSSI() : 0);
    dbg.print(" wifi_status=");
    dbg.print(wifi_status_str(WiFi.status()));
    dbg.print(" mqtt=");
    dbg.print(mqttClient.connected() ? "up" : "down");
    dbg.print(" ac_on=");
    dbg.println(acOn ? "yes" : "no");
  }
#endif

  if (!isWifiConnected) {
    static uint32_t lastWifiRetry = 0;
    // 15s, not 5s: WiFi.begin() needs real time to either succeed or
    // definitively fail before we abort and retry. Retrying too fast was
    // what caused the "sta is connecting, cannot set config" spam.
    static const uint32_t WIFI_RETRY_INTERVAL_MS = 15000;
    if (now - lastWifiRetry > WIFI_RETRY_INTERVAL_MS) {
      lastWifiRetry = now;
      connectWifi();
    }
    return;
  }

  if (!mqttClient.connected() && now - lastReconnectAttempt > reconnectBackoffMs) {
    lastReconnectAttempt = now;
    dbg.print("[mqtt] connecting to ");
    dbg.print(MQTT_HOST);
    dbg.print(":");
    dbg.println(MQTT_PORT);
    mqttClient.connect();
    reconnectBackoffMs = min(reconnectBackoffMs * 2, RECONNECT_BACKOFF_MAX_MS);
  }

  if (now - lastPoll > POLL_INTERVAL_MS) {
    lastPoll = now;
    acLink.request(Cmd::ROOM_TEMP);
    acLink.request(Cmd::OUTDOOR_TEMP);
  }

  if (mqttClient.connected() && now - lastWifiPublish > WIFI_PUBLISH_INTERVAL_MS) {
    lastWifiPublish = now;
    publishRetained(T_WIFI_RSSI, String(WiFi.RSSI()));
    publishRetained(T_MQTT_FAILS, String(mqttFailCount));
  }
}
