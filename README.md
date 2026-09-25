# toshiba_ac_bridge

Arduino IDE sketch that replaces the [pedobry/esphome_toshiba_suzumi](https://github.com/pedobry/esphome_toshiba_suzumi)
ESPHome component — same UART protocol to the Toshiba Suzumi/Shorai/Seiya
internal wifi-module connector, but running as a standalone ESP32 sketch
that talks MQTT + Home Assistant discovery directly, no ESPHome runtime.

Follows the same project pattern as the other ESP32 devices: `espMqttClient`
with exponential-backoff reconnect, retained HA MQTT discovery configs, LWT
availability, periodic WiFi signal + MQTT fail-count diagnostics, reset
reason on connect, static IP, manufacturer `P@cho`.

## Attribution

The wire protocol itself — handshake bytes, checksum, command framing,
register addresses (`toshiba_protocol.h`, `toshiba_link.h`) — is ported
directly from pedobry's reverse-engineering work, licensed GPL-3.0. Nothing
about the protocol was re-derived independently; only the MQTT/HA glue code
around it is new. If you redistribute this, keep that attribution and the
GPL-3.0 terms in mind for the protocol portion.

## What's included vs. the original ESPHome component

Ported:
- Handshake + checksum + request/set command framing
- Power state, mode, target temp (incl. the 8°/FrostGuard offset logic),
  fan speed, room temp, outdoor temp, swing (vertical only), power level
  select, special-mode presets (Standard/Eco/Hi-Power/8°/Floor)

Deliberately left out of this first pass (upstream has all of this in
`toshiba_climate.cpp` if you want to port more later):
- Energy/power sensors, ODU/IDU extended diagnostics — tried and removed;
  see "Tried and confirmed unsupported" below for why
- Self-clean status sensor
- Time sync to the unit
- Horizontal swing / fixed vertical air-direction positions
- The `scan()` unknown-register sweep utility

All of that is straightforward to add later — the parsing switch in
`toshiba_link.h::parse_()` and `onAcState()` in the `.ino` are the two spots
to extend.

## Power toggle resumes the last mode, not Auto

Home Assistant's climate card has a dedicated power toggle, separate from
the mode dropdown. Without a `power_command_topic`, HA's generic
`turn_on()` fallback just picks the first non-off entry in whatever order
`"modes":[...]` lists — which meant pressing that power button turned the
AC on in Auto every time, regardless of what mode it was last actually
running in.

Fixed by adding `power_command_topic` (`power/set`, plain `ON`/`OFF`
payloads) to the climate discovery config. This fully decouples HA's power
button from mode selection — it becomes a simple toggle the firmware
interprets itself, rather than something that implicitly picks a mode via
HA's list-order fallback. `handlePower()` resumes `lastActiveMode`, which
is updated only from real AC readbacks (`onAcState()`), not from commands
this firmware sends — so it stays correct even if the mode was last
changed via the AC's own physical remote, not just via this firmware. It
defaults to `cool` until the first readback arrives, which happens within
seconds of boot via the initial `Cmd::MODE` request in `setup()`.

This only affects the dedicated power toggle. Selecting a mode directly
from the dropdown (`mode/set`) behaves exactly as before — turns the AC on
in that specific mode, same as always.

## Tried and confirmed unsupported on this unit

Two features were built, tested against a real **Toshiba RAS-B10J2FVG-E1 /
RAS-10J2AVSG-E** (a genuine Bi-Flow console), and then removed after
confirming via live capture that this unit's WiFi module firmware doesn't
support them. Recorded here in case you're testing against different
hardware and want to re-add either:

**Bi-Flow outlet selector.** This model has a real physical feature —
Toshiba's own literature: *"select the favourable air flow outlet between
the two available positions at the top and bottom front of the unit."*
Confirmed via a live capture: cycling the AC's own physical remote through
its Bi-Flow toggle several times produced **zero** `SWING` (`0xA3`)
state-change frames on the UART bus. Whatever controls this on the unit's
own control board, it isn't reported to (or reachable via) the WiFi
module's UART protocol. (An earlier version of this project also
conflated this feature with the vertical-louver-angle `FIX_1`/`FIX_5`
values, which is a different, real, but unrelated control — that
conflation was a mistake on my part, unrelated to this hardware
limitation.)

**ODU/IDU diagnostics + energy/power.** Ported faithfully from upstream:
compressor load/current, discharge/suction/evaporator temps, indoor
heat-exchanger temps, indoor fan RPM (`ODU_STATUS`/`IDU_STATUS`,
unsolicited), plus daily energy total and an estimated realtime wattage
derived from it (`ENERGY_DAILY`, actively polled every 60s). Confirmed via
a ~16-minute live capture, compressor actively running the whole time:
`ENERGY_DAILY` requests (`D8`) went out roughly every 60s as designed and
were never once answered; no `ODU_STATUS`/`IDU_STATUS` (`E4`/`E5`) frame
ever appeared despite the AC being exercised through nearly every other
control during that window. This unit's firmware simply doesn't implement
these three registers over this protocol.

If you're working with different hardware and want to try either of
these again: the exact byte-offset parsing logic (verified correct via
synthetic frame tests, independent of whether any real unit ever sends
the frames) is preserved in this project's chat history rather than left
as dead code here.

## Wiring: AC UART

> [!WARNING]
> Disconnect the indoor unit from **mains power** before connecting or
> disconnecting this header. Do **not** connect the outermost pink-wire / pin-5
> contact on the Wi-Fi-module connector to anything. Verify the connector from
> your unit's documentation rather than relying only on wire colour; an
> incorrect connection can damage the AC control board.

The AC's internal wifi-module UART header runs at 5V logic, 9600 8E1. Use a
bidirectional level shifter between it and the ESP32 (a Pololu-style
shifter or TXS0108E) — there are reports upstream (issue #9) of connecting
RX/TX directly causing WiFi to drop out, consistent with a brownout from
backfeeding the AC's 5V logic into the 3V3 rail.

```
AC UART TX --[level shift]--> ESP32 RX  (AC_UART_RX_PIN)
AC UART RX <--[level shift]-- ESP32 TX  (AC_UART_TX_PIN)
AC GND ---------------------- ESP32 GND / shifter GND
```

Power the shifter's low-voltage side from ESP32 **3V3** and its high-voltage
side from the AC connector's **5V** supply. The AC 5V rail must never touch
the ESP32 3V3 rail. If the board is powered by USB, do not also tie that USB
5V rail to the AC's 5V output; use one deliberate power scheme instead.

The Pololu bidirectional shifter is adequate at 9600 baud. If serial traffic
is intermittent, replace it with two direction-controlled translation channels
(one AC-to-ESP, one ESP-to-AC); that avoids auto-direction contention during
power-up. Always test the connection with the AC powered down first, then
inspect the UART debug log after power is restored.

`config.h.example` defaults `AC_UART_TX_PIN`/`AC_UART_RX_PIN` to GPIO4/GPIO5.
On the ESP32-C5-Zero those are clear of GPIO27 (onboard WS2812 LED),
GPIO26 (antenna switch, see below), the internal SPI flash bus (GPIO6–11),
USB D-/D+ (GPIO13/14), and the other boot-strapping pins
(GPIO2/3/7/25/28) — no change needed switching boards, but always
cross-check against your board's own pinout diagram before wiring.

## Antenna switch (ESP32-C5-Zero, external antenna)

The C5-Zero has an onboard RF switch chip that picks between the built-in
PCB antenna and a U.FL/IPEX connector for an external antenna. Pulled
straight from the board's schematic: **GPIO26 drives that switch's control
line directly** — it's not part of the chip's internal multi-antenna PHY
diversity engine, just a plain GPIO feeding a switch IC, so a normal
`pinMode()`/`digitalWrite()` before `WiFi.begin()` is enough. No special
API needed.

That's already wired up in `config.h.example`:

```cpp
#define USE_EXTERNAL_ANTENNA 1   // 0 = use the onboard PCB antenna instead
#define ANT_SW_PIN 26
#define ANT_SW_EXTERNAL_LEVEL HIGH
```

One thing I couldn't pin down from the schematic: which logic level
(HIGH/LOW) actually selects the external port vs. the onboard antenna —
that depends on the specific switch IC's pinout, which didn't come through
clearly in the extracted schematic text. It's a safe thing to get wrong
(worst case you're just using the wrong antenna, nothing breaks), so:

1. Connect your external antenna to the U.FL connector.
2. Flash with `ANT_SW_EXTERNAL_LEVEL` at `HIGH` (the default).
3. Check the `wifi_signal/state` MQTT topic this firmware already publishes
   (or the WiFi Signal diagnostic sensor in Home Assistant) for the RSSI.
4. If it looks worse than you'd expect with the external antenna attached,
   flip the define to `LOW`, reflash, and compare again.

## Setup

1. **Install the ESP32 board package.** File → Preferences → "Additional
   boards manager URLs" →
   `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`,
   then Tools → Board → Boards Manager → install `esp32` by Espressif
   Systems, **version 3.3.5 or newer** (ESP32-C5 support landed there;
   earlier versions won't show a C5 board option at all).
2. **Install the MQTT library.** Tools → Manage Libraries → search
   `espMqttClient` (by Bert Melis) → Install. `ArduinoOTA`, `WiFi`, and
   `Preferences` (used for saving AC state across power loss) ship with
   the esp32 core already, nothing extra needed for those.
3. **Install AsyncTCP too, even though this firmware doesn't use it
   directly.** Arduino IDE compiles every `.cpp` file inside a library's
   `src/` folder, not just the parts your sketch references — and
   `espMqttClient` ships an async transport alongside the sync one this
   project uses, which unconditionally `#include <AsyncTCP.h>`. Without it
   installed you'll hit `fatal error: AsyncTCP.h: No such file or
   directory` at compile time even though nothing here calls into the
   async client. Tools → Manage Libraries → search `AsyncTCP` → install
   the one by **ESP32Async** → Install.
4. **Install the status LED library.** Tools → Manage Libraries → search
   `Adafruit NeoPixel` → Install. Only needed if `USE_STATUS_LED` is left
   at `1` in `config.h` (see below) — set it to `0` and skip this if your
   board has no onboard WS2812.
5. **Open the sketch.** File → Open → `toshiba_ac_bridge.ino`. The `.h`
   files appear as extra tabs automatically.
6. **Add your config.** Copy `config.h.example` to `config.h` in the same
   folder (or add a new tab named `config.h` via the ⋮ menu and paste the
   contents in) and fill in your real values.
7. **Select the board.** Tools → Board → search "C5" → pick your board
   (e.g. `Waveshare ESP32-C5-Zero` if listed, otherwise the generic
   `ESP32C5 Dev Module`).
8. **Upload.** Normal Arduino IDE upload button, or Sketch → Export
   Compiled Binary if you want a `.bin` for OTA elsewhere.

## Network security

`config.h` contains Wi-Fi, MQTT and OTA secrets and is intentionally ignored
by Git. Keep it local, use a unique OTA password, and rotate any credentials
that are accidentally shared. MQTT and Arduino OTA are not encrypted in this
sketch; place the device and broker on a trusted IoT network, or add TLS before
using them across an untrusted network.

## Local web interface

With `WEB_SERVER_ENABLED` set to `1` (the default), browse to the device's
static IP address after it joins Wi-Fi. The page provides local controls
for mode, temperature, fan, swing, preset, and power limit, plus AC state
and connection diagnostics. It uses the same validation and readback queue
as MQTT, so a submitted value is not presented as confirmed until the AC
responds.

**No authentication** — anyone who can reach the device's IP on your
network can view and change AC settings through this page. Same trust
model as the netlog stream described below: fine on a trusted local/IoT
network, not something to expose beyond it. The web page works even when
no MQTT broker is available.

## Remote access once the device is sealed inside the AC

Once the enclosure's closed and USB isn't reachable, two things still work
over the network:

### OTA updates

`ArduinoOTA` is already wired up (`setPassword`/`setHostname`/`begin()` in
`setup()`, `handle()` in `loop()`) — nothing new to add. To flash over the
air:

1. In Arduino IDE, Tools → Port. The device should appear under **"Network
   ports"** as `<DEVICE_ID> at <ip-address>` once it's on Wi-Fi, found via
   mDNS. Select it instead of a USB `COMx`/`/dev/tty*` port.
2. Upload as normal (Sketch → Upload, or the toolbar button). You'll be
   prompted for `OTA_PASSWORD` from `config.h`.

If it doesn't show up under Network ports, mDNS discovery is failing —
common on Windows without Bonjour installed, or on networks with client
isolation between Wi-Fi devices. Workaround: use `espota.py` directly
(ships with the ESP32 core, under
`<arduino-esp32-install>/tools/espota.py`) with the device's static IP
instead of relying on discovery:
```
python espota.py -i <device-ip> -p 3232 -a <OTA_PASSWORD> -f build/toshiba_ac_bridge.ino.bin
```

### Remote serial log

`NETLOG_ENABLED` (default `1`) mirrors every debug line this firmware
prints — boot, WiFi/MQTT milestones, `[ac TX]`/`[ac RX]` hex dumps,
`[mqtt>]`/`[mqtt<]`, the heartbeat, everything — out to a raw TCP socket on
`NETLOG_PORT` (default `23`), in addition to the physical USB serial port.
Connect with:
```
telnet <device-ip> <NETLOG_PORT>
```
or, if `telnet` isn't installed on your machine:
```
nc <device-ip> <NETLOG_PORT>
```
Only one client is held at a time — connecting again drops whatever was
previously connected and takes over, so you don't end up with a stale
half-open session silently eating log lines. Disconnecting and
reconnecting is always safe.

**This stream has no authentication.** Anyone who can reach `NETLOG_PORT`
on your LAN can read it. Worth knowing what that does and doesn't expose:
this firmware never prints WiFi/MQTT/web passwords to `dbg` anywhere, so
the practical exposure is operational telemetry (AC state, MQTT topic
traffic, WiFi signal) rather than credentials — but it's still more than
nothing, so keep this on a trusted IoT network like everything else here,
and set `NETLOG_ENABLED` to `0` if you'd rather not have it listening at
all once you're done actively debugging.

## Debug mode

`DEBUG_MODE` in `config.h` (default `1`) adds, on top of the always-on
boot/WiFi/MQTT milestone lines:

- `[ac TX]` / `[ac RX]` / `[ac RX-BAD]` — hex dump of every byte sent to
  and received from the AC UART, including checksum-rejected frames.
  Handy once this is actually wired to a unit; useless noise on a bare
  board since nothing ever replies.
- `[ac] <field>=<value>` — every decoded AC state field as it's parsed,
  even for ones that don't get published (e.g. a mode reading while the
  unit is off).
- `[mqtt>] <topic> = <payload>` — every outgoing publish, state updates
  and HA discovery configs alike.
- `[mqtt<] <topic> = <payload>` — every incoming command before it's
  handled.
- `[hb] uptime_s=... heap=... rssi=... mqtt=... ac_on=...` — a heartbeat
  line every 10s, useful for catching a slow memory leak or WiFi
  degradation over a long soak test.

Set it to `0` for quieter logs once things are working — no functional
behavior changes either way, purely log volume. Leaving it on doesn't
block anything (all the extra logging is non-blocking `Serial.print`
calls on the same code paths that already run).

### Bare-board MQTT simulation

Set `DEBUG_SIMULATE_MQTT` to `1` in `config.h` while `DEBUG_MODE` is enabled
to test without an AC, Wi-Fi, or MQTT broker. After five seconds the firmware
repeats a small sequence of simulated MQTT commands (mode, temperature, fan,
swing, power level, preset, an intentionally invalid temperature, then off).
The serial log labels these with `[mqtt SIM]`; the normal `[ac TX]` lines show
the frames that would be sent to the AC. No state is fabricated—the readback
requests simply receive no reply on a bare board, which is intentional.

## Status LED

The onboard WS2812 (GPIO27 on the C5-Zero — same pin confirmed from the
schematic as mentioned above) shows connection state, driven from `loop()`
so it never blocks WiFi/MQTT/UART handling:

| Pattern | Meaning |
|---|---|
| 3 quick white blinks | Boot, right at the start of `setup()` |
| Blinking blue | No WiFi yet (or WiFi dropped) |
| Solid blue | WiFi connected, MQTT not connected yet |
| Solid green | WiFi + MQTT both connected |

Controlled by `USE_STATUS_LED` / `STATUS_LED_PIN` in `config.h`. Set
`USE_STATUS_LED` to `0` if your board doesn't have one wired to that pin —
on a bare ESP32-C3 dev board, for instance, there usually isn't one at
GPIO27, so leaving it enabled without adjusting the pin will just drive an
unconnected GPIO and do nothing useful (harmless, but pointless).

## CPU frequency

`setup()` calls `setCpuFrequencyMhz(240)` — full speed. An earlier version
of this project throttled this down to 80MHz on the reasoning that a
9600-baud UART poll loop doesn't need much CPU; reverted back to 240MHz
per request. Either is fine for this workload — 80MHz was never a
stability requirement, WiFi's actual floor is lower than that — so this is
purely a preference, not something that needs fixing if you'd rather have
the headroom back down for lower power draw.

## Persistent state (survives power loss)

The AC and this board share the same power source, so a mains outage
power-cycles both. To avoid coming back up in an unknown state — or
requiring you to manually re-set everything from Home Assistant — every
AC state field is saved to NVS flash (via the `Preferences` library, the
modern ESP32-Arduino equivalent of "EEPROM"; genuinely non-volatile,
wear-leveled, survives power loss) the moment it's confirmed by a real AC
readback: on/off, mode, target temperature, fan, swing, preset, power
level.

On boot, if a previously-saved state exists, it's re-applied to the AC
*before* the normal initial state pull — the AC gets actively commanded
back to where it was, not just passively read. The subsequent readback
requests (already part of the normal boot sequence) confirm whatever state
actually resulted and publish that to MQTT, consistent with this
firmware's existing "never optimistically publish, always trust a real
readback" design.

On a genuinely first-ever boot (empty NVS), none of this fires — the
`"valid"` flag only gets set once a real field has actually been saved at
least once, so there's no risk of commanding the AC into an arbitrary
default before we've ever seen its actual state.

One tradeoff worth knowing: this can't distinguish "rebooted because of a
power outage" from "rebooted because of an OTA update" or a manual reset
— both look identical to the firmware, and both trigger the same restore.
In practice this is almost always the right behavior (you don't want the
AC changing state just because you flashed new firmware), but it does mean
if the AC's mode was changed via its own physical remote in the brief
window while this board was rebooting, that change would get overwritten
by the restored value once this board comes back online.

## MQTT topics (base `toshiba-ac1/`, set `DEVICE_ID` in config.h)

| Topic | Direction | Values |
|---|---|---|
| `mode/state`, `mode/set` | both | off, cool, heat, heat_cool, dry, fan_only |
| `power/set` | in | ON, OFF — HA's dedicated power toggle; resumes last mode on ON (see above) |
| `temperature/state`, `temperature/set` | both | 5–13 (8° preset) or 17–30 °C (normal); 14–16 are rejected |
| `current_temperature/state` | out | room temp °C |
| `outdoor_temperature/state` | out | outdoor temp °C |
| `fan_mode/state`, `fan_mode/set` | both | auto, quiet, low, low_medium, medium, medium_high, high |
| `swing_mode/state`, `swing_mode/set` | both | off, vertical |
| `preset_mode/state`, `preset_mode/set` | both | none, eco, hi_power, 8_degrees, floor |
| `power_level/state`, `power_level/set` | both | 50%, 75%, 100% |
| `wifi_signal/state` | out | RSSI dBm |
| `reset_reason/state` | out | reset cause string |
| `mqtt_fail_count/state` | out | disconnect counter |
| `uptime/state` | out | seconds since boot (zeroes on any reset/power loss) |
| `ota_restart/set` | in | any payload restarts the device |
| `status` | out | online/offline (LWT) |

All of this is published via retained HA discovery configs on
`homeassistant/<component>/toshiba-ac1/.../config`, so it should show up in
Home Assistant automatically once MQTT discovery is enabled.

## Version History

Unlike the rest of this fleet, this firmware doesn't define a
`FW_VERSION`/`FIRMWARE_VERSION` constant yet, so there's nothing to log a
number against — just the one release so far. Worth adding a version
constant (and an `sw_version` field in the HA discovery device block) if
you want this tracked going forward like the other projects.

| Version | Date | Changes |
|---|---|---|
| — | 2026-09-03 | Initial release. |
| — | 2026-09-25 | Added an `Uptime` diagnostic sensor (seconds since boot, `device_class: duration`) using 64-bit `esp_timer_get_time()` -- zeroes on any reboot or power loss, never wraps at ~49.7 days like `millis()`. This project has no `FIRMWARE_VERSION` constant, so no version bump. |
