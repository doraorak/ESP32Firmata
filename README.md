# ESP32 Firmata firmware (Bonjour / BLE)

Firmata 2.x firmware for the **original ESP32** (ESP32-WROOM-32 / -WROVER),
written to be byte-for-byte compatible with the
[**SwiftFirmataClient**](https://github.com/doraorak/SwiftFirmataClient) Swift package.

It is self-contained — it implements the Firmata protocol directly, so the only
dependencies are the ESP32 Arduino core libraries (`WiFi`, `ESPmDNS`, `BLE*`,
`Wire`). No Firmata library needs to be installed.

One sketch runs **both transports at once** — Wi-Fi/TCP + Bonjour **and** BLE — so
a client can connect either way with no reflashing. Each can be turned off with a
`#define` at the top of the sketch:

| `#define` (default) | Transport | Matches client |
|---------------------|-----------|----------------|
| `ENABLE_WIFI 1` | Wi-Fi / TCP + Bonjour (mDNS) | `BonjourTransport` |
| `ENABLE_BLE  1` | BLE Nordic UART Service (NUS) | `BLETransport` |

Only one client controls the board at a time. The policy is **latest-wins**: a new
connection on either transport evicts the current holder (a single, standard
Firmata master). Set one `#define` to `0` to force a single-transport build.

## What it implements

* Protocol-version, firmware, capability, analog-mapping and pin-state queries
* `setPinMode`, single-pin and whole-port digital writes
* Digital-input reporting (sent on change, only for ports the host enabled)
* Analog (ADC) reporting at the sampling interval, PWM analog writes, extended analog
* Sampling-interval, string and system-reset messages
* I2C config / write / read-once / continuous-read / stop (standard Firmata framing)
* **Scheduler** (SysEx `0x7B`) — store tasks and run them autonomously, even
  after the client disconnects: create / add / schedule / delay / query / reset,
  with Encoder7Bit packing and "trailing delay loops the task" (up to 8 tasks,
  512 bytes each, RAM-only). A queued task keeps running across reconnects; only
  `SYSTEM_RESET` (or power loss) clears tasks.

The pin map matches a typical ESP32 dev board:

* **Full digital** (INPUT / PULLUP / OUTPUT / PWM): GPIO 0, 2, 4, 5, 12–19, 21–23, 25–27, 32, 33
* **Input-only**: GPIO 34, 35, 36, 39
* **Analog (A0–A5)**: GPIO 32, 33, 34, 35, 36, 39 (ADC1 only — ADC2 can't be read while Wi-Fi is on)
* **I2C**: SDA = 21, SCL = 22

Flash-connected and non-bonded GPIOs (1, 3, 6–11, 20, 24, 28–31, 37, 38) are
reported with no capability so a client won't offer them.

## ⚠️ On-device logic (non-standard extension)

> This branch (`nonstandard-scheduler-logic`) **steps outside the Firmata
> standard.** It only works with the matching `SwiftFirmataClient`
> `nonstandard-scheduler-logic` branch. The standard line is on `main` /
> `standard`. Full details and wire format: [`NONSTANDARD.md`](NONSTANDARD.md).

On top of the Scheduler, a stored task can **make its own decisions** instead of
just replaying a fixed sequence — so it can act as a thermostat, night-light, etc.
entirely on the board with nobody connected:

* **16 global Int32 registers** (`R0`–`R15`), shared across tasks, reset by `SYSTEM_RESET`.
* New `SCHEDULER_DATA` sub-commands `0x10`–`0x14`:
  * `SET` — load a constant into a register
  * `READ_DIGITAL` / `READ_ANALOG` — read a pin / channel into a register
  * `IF` — compare two operands (register or constant) with `== != < > <= >=`;
    skip the guarded block when false
  * `SKIP` — unconditional forward skip (implements `else`)
* **Forward-only** branching (no backward jumps), so a task can branch but can
  never hang the board; looping still happens via the Scheduler's trailing delay.

From the client this is `setRegister`, `readDigital`/`readAnalog`, and
`ifTrue(_:_:_:then:elseDo:)` on the task recorder.

## Setup

1. **Arduino IDE** → install the **esp32 by Espressif** boards package
   (core 3.x recommended; 2.x also supported).
2. Open `ESP32Firmata.ino`.
3. Edit the **USER CONFIGURATION** block at the top:
   * Set `WIFI_SSID` and `WIFI_PASS` for your network (used by the Bonjour/Wi-Fi transport).
   * Optionally set `ENABLE_WIFI` or `ENABLE_BLE` to `0` to drop a transport.
4. **Tools** menu:
   * **Board**: *ESP32 Dev Module* (`esp32:esp32:esp32`).
   * **Partition Scheme**: the dual build (Wi-Fi **+** BLE) is large — pick
     *Minimal SPIFFS (1.9MB APP)* or *Huge APP*. (A Wi-Fi-only build fits the
     default scheme.)
5. Upload, then open Serial Monitor at **115200** to see the IP / advertising status.

## Connecting

* **Bonjour** — the board advertises `_firmata._tcp` on port **3030** with TXT
  records `ip` and `port`, so a client connects straight to the IP (skipping
  flaky mDNS A-record resolution). On macOS a client is prompted once for
  **Local Network** permission.
* **BLE** — the device advertises the Nordic UART Service UUID (so a client's
  service-filtered scan finds it) and the name `Firmata-ESP32` (usable as an
  optional name filter).

> Both transports run simultaneously, so there's **no reflashing to switch** —
> connect over Bonjour or BLE. Whichever a client connects with becomes the
> board's single master; connecting the other way evicts it (latest-wins).

## Protocol notes

* Firmware reports name **`FirmataESP32`** v2.8, protocol v2.8 in the firmware-report message.
* Verified against `SwiftFirmataClient`'s parser test vectors: firmware report,
  capability, analog-mapping, pin-state, digital/analog messages, extended
  analog and I2C reply all use the exact wire format the client expects.
* The client's I2C read/register encoding was corrected to standard Firmata
  framing (`FirmataClient.swift` → `sendI2CRequest`) so I2C interoperates with
  this firmware and other standard Firmata hosts.

## Verified

* Dual build (Wi-Fi + BLE): `arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs` — OK, ~88% flash.
* Single-transport builds also compile (Wi-Fi-only ~75% on the default partition; BLE-only ~58%).
* Latest-wins eviction, the Scheduler, and digital/analog I/O verified live against the board.
