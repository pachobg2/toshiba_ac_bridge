#pragma once
// UART link handling for the Toshiba AC protocol: framing, checksum validation,
// command queue with the same timing as the original component (100ms between
// commands, 200ms rx timeout to drop unparseable/unknown-format messages).

#include <Arduino.h>
#include <functional>
#include <vector>
#include "toshiba_protocol.h"

namespace toshiba {

struct DecodedState {
  bool has_power_state = false;
  PowerState power_state;

  bool has_mode = false;
  Mode mode;

  bool has_target_temp = false;
  uint8_t target_temp;  // already de-shifted for EIGHT_DEG if applicable

  bool has_room_temp = false;
  int8_t room_temp;

  bool has_outdoor_temp = false;
  int8_t outdoor_temp;

  bool has_fan = false;
  Fan fan;

  bool has_swing = false;
  Swing swing;

  bool has_power_level = false;
  PowerLevel power_level;

  bool has_special_mode = false;
  SpecialMode special_mode;
};

// A queue item is either bytes to send, or an in-queue delay marker (mirrors
// upstream's ToshibaCommandType::DELAY sentinel) so ordering relative to
// handshake/init-data stays exactly as the original component sequences it.
struct QueueItem {
  bool is_delay;
  uint32_t delay_ms;
  std::vector<uint8_t> payload;
};

class ToshibaLink {
 public:
  // Tag is "TX", "RX" (validated + parsed), or "RX-BAD" (checksum failed,
  // frame discarded). Set before start_handshake() if you want boot-time
  // handshake bytes included.
  using DebugFn = std::function<void(const char *tag, const std::vector<uint8_t> &bytes)>;
  void set_debug_callback(DebugFn cb) { debug_cb_ = cb; }

  // uartSerial must already be begun with 9600 8E1 by the caller.
  explicit ToshibaLink(HardwareSerial &uart) : uart_(uart) {}

  // Called once from setup(), after uart_.begin(...). Any request()/set()
  // calls made right after this in setup() are queued behind it, same as
  // upstream's start_handshake() + getInitData() both running in setup().
  void start_handshake() {
    for (auto &h : HANDSHAKE) queue_.push_back({false, 0, h});
    queue_.push_back({true, 2000, {}});
    for (auto &h : AFTER_HANDSHAKE) queue_.push_back({false, 0, h});
  }

  void request(Cmd cmd) { queue_.push_back({false, 0, build_request(cmd)}); }
  void set(Cmd cmd, uint8_t value) { queue_.push_back({false, 0, build_set(cmd, value)}); }

  // Call every loop(). Feeds rx bytes, drains tx queue on timing, and invokes
  // on_state whenever a fully validated response updates the DecodedState.
  void loop(const std::function<void(const DecodedState &)> &on_state) {
    while (uart_.available()) {
      uint8_t c = (uint8_t) uart_.read();

      // A UART can pick up noise during power transitions. Ignore bytes until
      // a frame header, and never allow an unknown frame to consume the heap.
      if (rx_.empty() && c != 0x02) continue;
      rx_.push_back(c);
      if (rx_.size() > MAX_RX_MESSAGE_LEN) {
        if (debug_cb_) debug_cb_("RX-BAD", rx_);
        rx_.clear();
        rx_complete_ = false;
        continue;
      }
      if (!validate_()) {
        if (debug_cb_) debug_cb_("RX-BAD", rx_);
        rx_.clear();
      } else {
        last_rx_char_ms_ = millis();
        if (rx_complete_) {
          if (debug_cb_) debug_cb_("RX", rx_);
          DecodedState st;
          parse_(rx_, st);
          rx_.clear();
          rx_complete_ = false;
          on_state(st);
        }
      }
    }
    pump_queue_();
  }

 private:
  HardwareSerial &uart_;
  std::vector<uint8_t> rx_;
  bool rx_complete_ = false;
  uint32_t last_rx_char_ms_ = 0;
  uint32_t last_cmd_ms_ = 0;
  std::vector<QueueItem> queue_;
  uint32_t delay_started_ms_ = 0;
  bool delay_in_progress_ = false;
  DebugFn debug_cb_ = nullptr;
  static constexpr size_t MAX_RX_MESSAGE_LEN = 80;

  void pump_queue_() {
    uint32_t now = millis();

    // If we've been mid-message for too long without new bytes, give up on it
    // (unknown-format handshake replies end this way, same as upstream).
    if (now - last_rx_char_ms_ > RECEIVE_TIMEOUT_MS) {
      rx_.clear();
      rx_complete_ = false;
    }

    if (now - last_cmd_ms_ <= COMMAND_DELAY_MS) return;
    if (!rx_.empty()) return;  // don't step on an in-flight response
    if (queue_.empty()) return;

    auto &front = queue_.front();
    if (front.is_delay) {
      if (!delay_in_progress_) {
        delay_in_progress_ = true;
        delay_started_ms_ = now;
      }
      if (now - delay_started_ms_ < front.delay_ms) return;
      delay_in_progress_ = false;
      queue_.erase(queue_.begin());
      return;
    }

    send_(front.payload);
    queue_.erase(queue_.begin());
  }

  void send_(const std::vector<uint8_t> &payload) {
    last_cmd_ms_ = millis();
    if (debug_cb_) debug_cb_("TX", payload);
    uart_.write(payload.data(), payload.size());
  }

  // Mirrors validate_message_() from the original component.
  bool validate_() {
    size_t at = rx_.size() - 1;
    uint8_t new_byte = rx_[at];

    if (at == 0) return new_byte == 0x02;
    if (at < 2) return true;
    if (rx_[2] != 0x03) return true;  // non-standard handshake reply; can't validate, just timeout it
    if (at <= 5) return true;

    uint8_t length = 6 + rx_[6] + 1;  // prefix + data + checksum
    if (at < length) return true;

    uint8_t rx_checksum = new_byte;
    uint8_t calc = checksum(rx_, at);
    if (rx_checksum != calc) return false;

    rx_complete_ = true;
    return true;
  }

  void parse_(const std::vector<uint8_t> &d, DecodedState &st) {
    uint8_t length = d.size();
    Cmd sensor;
    uint8_t value;

    switch (length) {
      case 15:
        sensor = (Cmd) d[12];
        value = d[13];
        break;
      case 16:
        return;  // ack frame, nothing to parse
      case 17:
        sensor = (Cmd) d[14];
        value = d[15];
        break;
      default:
        // ODU/IDU extended status (22/24 bytes) and the daily energy
        // histogram (69/70 bytes) were decoded here in an earlier version
        // of this project. Removed after confirming, via a live capture,
        // that this specific unit's WiFi module firmware never sends
        // those frames at all (E4/E5/D8 never appeared, even with the
        // compressor actively running). Re-add if testing against a unit
        // that does support them — the exact offset math is preserved in
        // git history / an earlier chat export if needed again.
        return;
    }

    switch (sensor) {
      case Cmd::TARGET_TEMP:
        st.has_target_temp = true;
        st.target_temp = value;  // caller applies EIGHT_DEG offset using its own special_mode state
        break;
      case Cmd::FAN:
        st.has_fan = true;
        st.fan = (Fan) value;
        break;
      case Cmd::SWING:
        st.has_swing = true;
        st.swing = (Swing) value;
        break;
      case Cmd::MODE:
        st.has_mode = true;
        st.mode = (Mode) value;
        break;
      case Cmd::ROOM_TEMP:
        if (value != 127) {
          st.has_room_temp = true;
          st.room_temp = (int8_t) value;
        }
        break;
      case Cmd::OUTDOOR_TEMP:
        if (value != 127) {
          st.has_outdoor_temp = true;
          st.outdoor_temp = (int8_t) value;
        }
        break;
      case Cmd::POWER_SEL:
        st.has_power_level = true;
        st.power_level = (PowerLevel) value;
        break;
      case Cmd::POWER_STATE:
        st.has_power_state = true;
        st.power_state = (PowerState) value;
        break;
      case Cmd::SPECIAL_MODE:
        st.has_special_mode = true;
        st.special_mode = (SpecialMode) value;
        break;
      default:
        break;  // unknown/unhandled register, ignore
    }
  }
};

}  // namespace toshiba
