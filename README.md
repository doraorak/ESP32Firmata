# ESP32Firmata

Firmata firmware for the ESP32, in a single Arduino sketch. Speaks Firmata 2.x
over Wi-Fi (Bonjour/TCP), BLE (Nordic UART), and USB serial — with an on-device
task extension: registers, branches, arithmetic, HTTP + JSON inspection,
strings, and tasks that spawn tasks.

This is the C++ twin of [ESP32FirmataSwift](https://github.com/doraorak/ESP32FirmataSwift):
**byte-identical wire protocol**, same features, same version numbers. Use this
one if you want the plain Arduino toolchain; use the Swift one if you want
Embedded Swift on ESP-IDF.

## The project suite

| Repo | Role |
|---|---|
| [SwiftFirmataClient](https://github.com/doraorak/SwiftFirmataClient) | macOS/iOS client package (start here for API + COOKBOOK) |
| [ESP32FirmataSwift](https://github.com/doraorak/ESP32FirmataSwift) | Embedded Swift firmware — its README holds the **wire-format spec** |
| [ESP32Firmata](https://github.com/doraorak/ESP32Firmata) | This repo — the C++/Arduino firmware |

## Flash it

**Arduino IDE**: open `ESP32Firmata.ino`, board **ESP32 Dev Module**, Partition
Scheme **Huge APP (3 MB)**, Upload.

**CLI** (needs [arduino-cli](https://arduino.github.io/arduino-cli/)):

```bash
./flash.sh                      # auto-detects the port, builds + uploads
./monitor.sh                    # serial log — shows the board's IP / Bonjour name
```

No toolchain? Each [release](https://github.com/doraorak/ESP32Firmata/releases)
ships a merged 4 MB image — flash it at **0x0** with esptool or
[ESP Web Tools](https://esp.huhn.me), then provision Wi-Fi from the client.

## Setup (building from source)

1. Arduino IDE → install the **esp32 by Espressif** core (3.x; PWM uses the
   core's `analogWrite`).
2. Install the **NimBLE-Arduino** library. The sketch uses NimBLE instead of
   Bluedroid deliberately: Bluedroid leaves ~40 KB of free heap, too little for
   the Arduino core's TLS buffers, so HTTPS handshakes fail with
   `SSL - Memory allocation failed`. NimBLE frees enough heap (and ~480 KB flash)
   to run **Wi-Fi + BLE + HTTPS together**.

Everything else is the stock core (`WiFi`, `ESPmDNS`, `Wire`, `HTTPClient`).

## Wi-Fi credentials

- **Compile-time**: `WIFI_SSID` / `WIFI_PASS` near the top of the sketch.
- **Provisioned**: leave the placeholders and send credentials from the client
  (`provisionWiFi` — encrypted X25519 + AES-GCM, over BLE, TCP, or serial).
  Stored in NVS only after a successful join; a wrong password rolls back.

## Connecting

| Transport | Details |
|---|---|
| Wi-Fi | `_firmata._tcp` on port **3030**, instance `esp32-firmata` (TXT carries `ip`/`port`) |
| BLE | Nordic UART Service, name `Firmata-ESP32` |
| USB serial | 115200. Boots as the log console; the **first byte** a host sends claims the Firmata session and silences logging. No disconnect event — the claim lasts until eviction or reboot. |

One master at a time, latest wins; the evicted client gets an `EVICTED` string
notice. Scheduler tasks keep running across disconnects.

## What it implements

Standard Firmata 2.x (digital/analog I/O, PWM/servo, extended analog, reports,
capability/analog-mapping/pin-state queries, sampling interval, full I²C, the
Scheduler) plus the task extension — registers `R0–R15` / floats `F0–F7`,
forward-only `if/else`, int+float arithmetic, HTTP(S) with JSON/string
inspection and a 12-slot snapshot pool, I²C reads into registers, `sendString`
telemetry, and nested tasks (a task may create/schedule/delete other tasks).

The byte-level spec for all ext ops (`0x10–0x30`) lives in the
[ESP32FirmataSwift README](https://github.com/doraorak/ESP32FirmataSwift#wire-format-ext-ops)
— this firmware implements it byte-for-byte. Drive it from Swift with
[SwiftFirmataClient](https://github.com/doraorak/SwiftFirmataClient) and its
[COOKBOOK](https://github.com/doraorak/SwiftFirmataClient/blob/main/COOKBOOK.md).

## Pin map (typical ESP32 dev board)

- Full digital (input/pullup/output/PWM): GPIO 0, 2, 4, 5, 12–19, 21–23, 25–27, 32, 33
- Input-only: GPIO 34, 35, 36, 39 · Analog A0–A5 → GPIO 32, 33, 34, 35, 36, 39 · I²C: SDA 21 / SCL 22

## License

MIT — see [LICENSE](LICENSE).
