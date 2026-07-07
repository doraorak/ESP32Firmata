# ESP32Firmata

Firmata firmware for the ESP32, in a single Arduino sketch. Speaks Firmata 2.x over
Wi-Fi (Bonjour/TCP), BLE (Nordic UART), and USB serial, with an on-device task
extension: registers, branches, loops, arithmetic, HTTP + JSON inspection, strings,
and tasks that spawn tasks.

This is the C++ twin of [ESP32FirmataSwift](https://github.com/doraorak/ESP32FirmataSwift):
**byte-identical wire protocol**, same features, same version numbers. Use this one
for the plain Arduino toolchain; use the Swift one for Embedded Swift on ESP-IDF.

## The suite

| Repo | Role |
|---|---|
| [SwiftFirmataClient](https://github.com/doraorak/SwiftFirmataClient) | macOS/iOS client package (start here for the API + COOKBOOK) |
| [ESP32FirmataSwift](https://github.com/doraorak/ESP32FirmataSwift) | Embedded Swift firmware — its README holds the **wire-format spec** |
| [ESP32Firmata](https://github.com/doraorak/ESP32Firmata) | This repo — the C++/Arduino firmware |

## Flash it

**Arduino IDE**: open `ESP32Firmata.ino`, board **ESP32 Dev Module**, Partition
Scheme **Huge APP (3 MB)**, Upload.

**CLI** ([arduino-cli](https://arduino.github.io/arduino-cli/)):

```bash
./flash.sh                      # auto-detects the port, builds + uploads
./monitor.sh                    # serial log — shows the board's IP / Bonjour name
```

No toolchain? Each [release](https://github.com/doraorak/ESP32Firmata/releases) ships
a merged 4 MB image — flash it at **0x0** with esptool or
[ESP Web Tools](https://esp.huhn.me), then provision Wi-Fi from the client.

## Setup (building from source)

1. Install the **esp32 by Espressif** core (3.x) in the Arduino IDE.
2. Install the **NimBLE-Arduino** library. The sketch uses NimBLE, not Bluedroid,
   deliberately: Bluedroid leaves too little free heap for the Arduino core's TLS
   buffers, so HTTPS handshakes fail. NimBLE frees enough heap to run **Wi-Fi + BLE +
   HTTPS together**.

Everything else is the stock core (`WiFi`, `ESPmDNS`, `Wire`, `HTTPClient`).

## Wi-Fi credentials

- **Compile-time**: `WIFI_SSID` / `WIFI_PASS` near the top of the sketch.
- **Provisioned**: leave the placeholders and send credentials from the client
  (`provisionWiFi` — encrypted X25519 + AES-GCM, over BLE, TCP, or serial). Stored in
  NVS only after a successful join; a wrong password rolls back.

## Connecting

| Transport | Details |
|---|---|
| Wi-Fi | `_firmata._tcp` on port **3030**, instance `esp32-firmata` (TXT carries `ip`/`port`) |
| BLE | Nordic UART Service, name `Firmata-ESP32` |
| USB serial | 115200. Boots as the log console; the **first byte** a host sends claims the Firmata session and silences logging. |

One master at a time, latest wins; the evicted client gets an `EVICTED` string
notice. Scheduler tasks keep running across disconnects.

## What it implements

Standard Firmata 2.x (digital/analog I/O, PWM/servo, extended analog, reports,
capability/analog-mapping/pin-state queries, sampling interval, full I²C, the
scheduler) plus the task extension: 32 registers + 16 floats (`R0–R15`/`F0–F7` public,
`R16–R31`/`F8–F15` internal), forward-only `if/else`, native counted loops, int+float
arithmetic, HTTP(S) with JSON/string inspection and a 12-slot snapshot pool, I²C reads
into registers, `sendString` telemetry, and nested tasks.

The byte-level spec for all ext ops (`0x10–0x35`) lives in the
[ESP32FirmataSwift README](https://github.com/doraorak/ESP32FirmataSwift#wire-format-ext-ops)
— this firmware implements it byte-for-byte. Drive it from Swift with
[SwiftFirmataClient](https://github.com/doraorak/SwiftFirmataClient) and its
[COOKBOOK](https://github.com/doraorak/SwiftFirmataClient/blob/main/COOKBOOK.md).

## Modules

Compile-time plugins behind one reserved SysEx, **`MODULE_DATA` (`0x0D`)**, each in
the firmware's own language — native C++ here. Wire:

- `F0 0D 00 F7` — **query**; reply `F0 0D 7F <n> [<id> <maj> <min> <nameLen> <name…>]* F7`.
- `F0 0D <id> <payload…> F7` — talk to module `<id>` (`0x01–0x7E`); the payload is that
  module's own protocol, both directions.
- Task ext op `0x33 <id> <payload…>` — a scheduled task drives a module.

Each module is a class deriving from an abstract `ModuleHandler` (its `id()`/version/
`name()`, a `handle()` for its wire ops, and a `tick()`); a `ModuleHandler* modules[]`
array is the registry that discovery, dispatch, and the per-loop tick iterate. Adding
a module is one class plus one array entry.

| ID | Module | Purpose |
|----|--------|---------|
| `0x01` | `ir` | Infrared NEC/RC6 transmit + NEC receive over RMT |

The IR module transmits any protocol via one raw op (`0x03 <kHz> <mark/space µs pairs>`)
with NEC/RC6 encoded host-side (see [SwiftFirmataIR](https://github.com/doraorak/SwiftFirmataIR));
it also carries on-device encoders (`0x05 <protocol> <reg>`) to replay a code held in a
register. Drive the LED at 5 V, keep the receiver on 3.3 V.

## Pin map (typical ESP32 dev board)

- Full digital (input/pullup/output/PWM): GPIO 0, 2, 4, 5, 12–19, 21–23, 25–27, 32, 33
- Input-only: GPIO 34, 35, 36, 39 · Analog A0–A5 → GPIO 32, 33, 34, 35, 36, 39 · I²C: SDA 21 / SCL 22

## License

MIT — see [LICENSE](LICENSE).
