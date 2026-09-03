#pragma once
// Toshiba Suzumi/Shorai/Seiya AC UART protocol.
//
// Reverse-engineered by pedobry (https://github.com/pedobry/esphome_toshiba_suzumi,
// GPL-3.0). This header ports just the protocol constants/framing out of that
// ESPHome component into plain values usable without the ESPHome runtime.
//
// Wire format: 9600 baud, 8E1 (even parity), half-duplex-ish request/response.

#include <Arduino.h>
#include <vector>

namespace toshiba {

// ---- Command byte codes (register addresses) ----
enum class Cmd : uint8_t {
  POWER_STATE = 128,
  POWER_SEL = 135,
  FAN = 160,
  SWING = 163,
  MODE = 176,
  TARGET_TEMP = 179,
  ROOM_TEMP = 187,
  OUTDOOR_TEMP = 190,
  WIFI_LED_1 = 222,
  WIFI_LED_2 = 223,
  SPECIAL_MODE = 247,
};

// ---- Value enums, as sent/received on the wire ----
enum class Mode : uint8_t { HEAT_COOL = 65, COOL = 66, HEAT = 67, DRY = 68, FAN_ONLY = 69 };

enum class Fan : uint8_t {
  QUIET = 49,
  SPEED_LOW = 50,
  LEVEL_2 = 51,
  MEDIUM = 52,
  LEVEL_4 = 53,
  SPEED_HIGH = 54,
  AUTO = 65,
};

enum class Swing : uint8_t {
  OFF = 49,
  BOTH = 67,
  VERTICAL = 65,
  HORIZONTAL = 66,
  FIX_1 = 80,
  FIX_2 = 81,
  FIX_3 = 82,
  FIX_4 = 83,
  FIX_5 = 84,
};

enum class PowerState : uint8_t { ON = 48, OFF = 49 };

enum class PowerLevel : uint8_t { PCT_50 = 50, PCT_75 = 75, PCT_100 = 100 };

enum class SpecialMode : uint8_t {
  STANDARD = 0,
  HI_POWER = 1,
  SILENT_1 = 2,
  ECO = 3,
  EIGHT_DEG = 4,
  SLEEP = 5,
  FLOOR = 6,
  COMFORT = 7,
  SILENT_2 = 10,
  FIREPLACE_1 = 32,
  FIREPLACE_2 = 48,
};

static const uint8_t MAX_TEMP = 30;
static const uint8_t MIN_TEMP_STANDARD = 17;
static const uint8_t SPECIAL_TEMP_OFFSET = 16;  // applied to target temp while in EIGHT_DEG mode
static const uint8_t EIGHT_DEG_MIN_TEMP = 5;
static const uint8_t EIGHT_DEG_MAX_TEMP = 13;
static const uint8_t EIGHT_DEG_DEFAULT_TEMP = 8;
static const uint8_t NORMAL_DEFAULT_TEMP = 20;

static const uint32_t COMMAND_DELAY_MS = 100;
static const uint32_t RECEIVE_TIMEOUT_MS = 200;

// Fixed handshake sequence sent once at startup to make the unit start talking.
// Bytes are exactly what the original wifi module sends; do not change these.
static const std::vector<uint8_t> HANDSHAKE[6] = {
    {2, 255, 255, 0, 0, 0, 0, 2},
    {2, 255, 255, 1, 0, 0, 1, 2, 254},
    {2, 0, 0, 0, 0, 0, 2, 2, 2, 250},
    {2, 0, 1, 129, 1, 0, 2, 0, 0, 123},
    {2, 0, 1, 2, 0, 0, 2, 0, 0, 254},
    {2, 0, 2, 0, 0, 0, 0, 254},
};
static const std::vector<uint8_t> AFTER_HANDSHAKE[2] = {
    {2, 0, 2, 1, 0, 0, 2, 0, 0, 251},
    {2, 0, 2, 2, 0, 0, 2, 0, 0, 250},
};

// Checksum = 256 - (sum of bytes[1..length-1] mod 256). Byte 0 (header 0x02) excluded.
inline uint8_t checksum(const std::vector<uint8_t> &data, size_t length) {
  uint8_t sum = 0;
  for (size_t i = 1; i < length; i++) sum += data[i];
  return (uint8_t) (256 - sum);
}

// Build a "set value" command: 02 00 03 10 00 00 07 01 30 01 00 02 <cmd> <val> <checksum>
inline std::vector<uint8_t> build_set(Cmd cmd, uint8_t value) {
  std::vector<uint8_t> p = {2, 0, 3, 16, 0, 0, 7, 1, 48, 1, 0, 2};
  p.push_back((uint8_t) cmd);
  p.push_back(value);
  p.push_back(checksum(p, p.size()));
  return p;
}

// Build a "request value" command: 02 00 03 10 00 00 06 01 30 01 00 01 <cmd> <checksum>
inline std::vector<uint8_t> build_request(Cmd cmd) {
  std::vector<uint8_t> p = {2, 0, 3, 16, 0, 0, 6, 1, 48, 1, 0, 1};
  p.push_back((uint8_t) cmd);
  p.push_back(checksum(p, p.size()));
  return p;
}

}  // namespace toshiba
