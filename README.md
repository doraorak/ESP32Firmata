# ESP32 Firmata firmware (Bonjour + BLE) — standard

Firmata 2.x firmware for the **original ESP32** (ESP32-WROOM-32 / -WROVER),
byte-for-byte compatible with the
[**SwiftFirmataClient**](https://github.com/doraorak/SwiftFirmataClient) package.

Self-contained — it implements the Firmata protocol directly, so the only
dependencies are the ESP32 Arduino core libraries (`WiFi`, `ESPmDNS`, `BLE*`,
`Wire`). One sketch runs **both transports at once** — Wi-Fi/TCP + Bonjour **and**
BLE — so a client can connect either way with no reflashing.

> **This is the `no-extension` branch: standard-compliant Firmata, Scheduler
> included but with no on-device logic ops.** For the version where stored tasks
> can make their own decisions (registers + `if`/`else`), use
> [`main`](https://github.com/doraorak/ESP32Firmata/tree/main).

| `#define` (default) | Transport | Matches client |
|---|---|---|
| `ENABLE_WIFI 1` | Wi-Fi / TCP + Bonjour (mDNS) | `BonjourTransport` |
| `ENABLE_BLE  1` | BLE Nordic UART Service (NUS) | `BLETransport` |

Only one client controls the board at a time — **latest-wins**: a new connection
on either transport evicts the current holder. Scheduler tasks are global and
survive eviction/disconnect.

## What it implements

* Protocol-version, firmware, capability, analog-mapping and pin-state queries
* `setPinMode`, single-pin and whole-port digital writes
* Digital-input reporting (on change), analog (ADC) reporting at the sampling
  interval, PWM analog writes (`analogWrite`), extended analog
* Sampling-interval, string and system-reset messages
* I2C config / write / read-once / continuous-read / stop (standard framing)
* **Scheduler** (SysEx `0x7B`) — store tasks and run them autonomously, even after
  the client disconnects: create / add / schedule / delay / query / reset, with
  Encoder7Bit packing and "a trailing DELAY_TASK loops the task" (up to 8 tasks,
  512 bytes each, RAM-only). Fully standard Firmata Scheduler wire format.

### Pin map (typical ESP32 dev board)
* **Full digital** (INPUT / PULLUP / OUTPUT / PWM): GPIO 0, 2, 4, 5, 12–19, 21–23, 25–27, 32, 33
* **Input-only**: GPIO 34, 35, 36, 39
* **Analog (A0–A5)**: GPIO 32, 33, 34, 35, 36, 39 (ADC1 only)
* **I2C**: SDA = 21, SCL = 22

## Setup

1. **Arduino IDE** → install **esp32 by Espressif** (core **3.x** recommended;
   PWM uses the core's `analogWrite`).
2. Open `ESP32Firmata.ino`; set `WIFI_SSID` / `WIFI_PASS` in the USER
   CONFIGURATION block (and optionally set `ENABLE_WIFI`/`ENABLE_BLE` to `0`).
3. **Board**: *ESP32 Dev Module*. **Partition Scheme**: *Minimal SPIFFS (1.9 MB
   APP)* or *Huge APP* (the dual Wi-Fi+BLE build is large).
4. Upload; open Serial Monitor @ **115200** for IP / advertising status.

## Connecting

* **Bonjour** — advertises `_firmata._tcp` on port **3030** with TXT records
  `ip` / `port`.
* **BLE** — advertises the Nordic UART Service UUID and the name `Firmata-ESP32`.

## Verified
* Dual build (Wi-Fi + BLE) compiles (`min_spiffs`); single-transport builds also
  compile. Latest-wins eviction, the Scheduler, and digital/analog I/O verified
  live against the board.
