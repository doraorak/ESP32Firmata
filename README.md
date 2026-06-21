# ESP32 Firmata firmware (Bonjour + BLE) — with on-device logic extension

Firmata 2.x firmware for the **original ESP32** (ESP32-WROOM-32 / -WROVER),
byte-for-byte compatible with the
[**SwiftFirmataClient**](https://github.com/doraorak/SwiftFirmataClient) package.

Self-contained — it implements the Firmata protocol directly, so the only
dependencies are the ESP32 Arduino core libraries (`WiFi`, `ESPmDNS`, `BLE*`,
`Wire`). One sketch runs **both transports at once** — Wi-Fi/TCP + Bonjour **and**
BLE — so a client can connect either way with no reflashing.

> **Branches:** `main` (this one) includes the **Scheduler logic extension**
> (registers + `if`/`else`) **and internet actions** (HTTP(S) over Wi-Fi + JSON/
> string response inspection), documented below. The
> [`no-extension`](https://github.com/doraorak/ESP32Firmata/tree/no-extension)
> branch is the standard-compliant firmware (Scheduler only, no logic ops).

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
  the client disconnects (create / add / schedule / delay / query / reset,
  Encoder7Bit packing, "trailing delay loops the task"; up to 8 tasks, 512 B each)
* **On-device logic extension** (this branch) — 16 registers + `if`/`else`
* **Internet actions** (this branch) — a task (or live host) makes HTTP(S) requests
  over Wi-Fi and inspects the JSON/text response (number/string/contains ops) to
  branch on it; see *Custom protocol* below

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
3. **Board**: *ESP32 Dev Module*. **Partition Scheme**: *Huge APP (3 MB)*
   recommended — the dual Wi-Fi+BLE build fills ~98% of *Minimal SPIFFS*.
4. Upload; open Serial Monitor @ **115200** for IP / advertising status.

## Connecting

* **Bonjour** — advertises `_firmata._tcp` on port **3030** with TXT records
  `ip` / `port`.
* **BLE** — advertises the Nordic UART Service UUID and the name `Firmata-ESP32`.

---

## Custom protocol — Scheduler logic extension

On top of the Firmata Scheduler, a stored task can **make its own decisions**
(thermostat, night-light, …) entirely on the board with nobody connected:

* **16 global Int32 registers** `R0`–`R15`, shared across tasks, reset by `SYSTEM_RESET`.
* **Forward-only** branching (no backward jumps) — a task can branch but can never
  hang the board; looping still happens via the Scheduler's trailing delay.

It rides under the reference scheduler's reserved `EXTENDED_SCHEDULER_COMMAND`
(`0x7F`), so a standard Firmata scheduler **ignores it gracefully** (no crash;
conditionals become no-ops). This is why it lives on `main` while the strictly
standard build stays on `no-extension`.

### High-level API (SwiftFirmataClient `FirmataTaskRecorder`)

```swift
let rec = board.makeTask(id: 1)
rec.setRegister(0, to: 25)                 // R0 = 25
rec.readAnalog(channel: 0, into: 1)        // R1 = analogRead(A0)
rec.ifTrue(.register(1), .greaterThan, .register(0)) {   // if R1 > R0
    $0.digitalWrite(pin: 2, value: 0)      //   then: LED off
} elseDo: {
    $0.digitalWrite(pin: 2, value: 1)      //   else: LED on
}
rec.delay(ms: 1000)                        // loop every 1 s
board.schedule(rec, afterMs: 0)
```

| High-level | Effect |
|---|---|
| `setRegister(_ d, to: const)` | `R[d] = const` |
| `readDigital(pin:, into: d)` | `R[d] = digitalRead(pin)` |
| `readAnalog(channel:, into: d)` | `R[d] = analogRead(channel)` |
| `ifTrue(_:_:_:then:elseDo:)` | compare two operands; run `then`, else `elseDo` |
| `httpGet(_ url, statusInto: s)` | HTTP(S) GET over Wi-Fi; `R[s]`=status, body retained |
| `httpPost(_ url, body:, statusInto: s)` | as above, POST with a JSON body |
| `jsonNumber(_ path, scaledBy:)` | `R` = JSON number at path × 10ⁿ (truncated); returns operand |
| `jsonStringEquals(_ path, _ value)` / `jsonStringContains(_ path, _ sub)` | compare JSON string at path → `R` (0/1) |
| `bodyContains(_ text)` | substring over the whole body → `R` (0/1) |

So a task can fetch JSON from the internet and act on it with no host connected —
e.g. `httpGet` a quote endpoint, `jsonNumber("changePercent", scaledBy: 2)`, then
`ifTrue` on that register to drive a pin. The client can also call `httpGet`/
`httpPost` **live** and inspect the `HTTPResponse` (status + body) on the host with
`HTTPResponse.json()` / `.decode(_:)`.

> **HTTPS heap note.** `https://` is validated against the IDF cert bundle and
> works **out of the box in the Wi-Fi-only build** (`ENABLE_BLE 0`). In the **dual
> Wi-Fi + BLE build**, Bluedroid leaves only ~40 KB heap — too little for the
> Arduino core's TLS buffers (its prebuilt mbedtls has `ASYMMETRIC_CONTENT_LEN`
> off, so it needs ~32 KB), so an `https://` request fails with `SSL - Memory
> allocation failed` while `http://` still works. For HTTPS **with** BLE, use the
> [ESP32FirmataSwift](https://github.com/doraorak/ESP32FirmataSwift) firmware (its
> ESP-IDF build uses asymmetric TLS buffers and fits). The JSON/string inspection
> ops work in **all** builds (they operate on whatever body was retained).

### Byte commands (wire format)

All ops are SysEx embedded in a task's data and replayed by the Scheduler, under
`SCHEDULER_DATA` (`0x7B`) → `EXTENDED_SCHEDULER_COMMAND` (`0x7F`). `<const>` is an
Int32 as 5 Encoder7Bit bytes; `<skip>` is a 14-bit count, little-endian 7-bit
(`skipLo skipHi`). `<len>` fields are 14-bit LE; `<path>`/`<str>`/`<url>`/`<body>`
are raw 7-bit ASCII.

```
SET            F0 7B 7F 10 <reg> <const:5>                          F7  // R[reg] = const
READ_DIGITAL   F0 7B 7F 11 <reg> <pin>                              F7  // R[reg] = digitalRead(pin)
READ_ANALOG    F0 7B 7F 12 <reg> <channel>                          F7  // R[reg] = analogRead(channel)
IF             F0 7B 7F 13 <op> <operandA> <operandB> <skip:2>      F7  // if !(A op B): pos += skip
SKIP           F0 7B 7F 14 <skip:2>                                 F7  // pos += skip (else)
HTTP           F0 7B 7F 15 <method> <statusReg> <urlLen:2> <url…> <bodyLen:2> <body…> F7
JSON_NUM       F0 7B 7F 16 <dst> <found> <scale> <pathLen:2> <path…>     F7
JSON_STR_EQ    F0 7B 7F 17 <dst> <pathLen:2> <path…> <strLen:2> <str…>   F7
BODY_CONTAINS  F0 7B 7F 18 <dst> <strLen:2> <str…>                       F7
JSON_STR_CONT  F0 7B 7F 19 <dst> <pathLen:2> <path…> <strLen:2> <str…>   F7
HTTP_REPLY     F0 7B 0B <status:2> <body 14-bit pairs…>                  F7  // device -> host
```

* `<reg>`: register index, low nibble (`0`–`15`).
* `<op>`: `0 ==`, `1 !=`, `2 <`, `3 >`, `4 <=`, `5 >=`.
* `<operand>`: a type byte then data — `00 <reg>` (register) or `01 <const:5>` (literal).
* `<pin>`: GPIO number. `<channel>`: analog channel index (A0 = 0…), **not a pin**.
* `HTTP` (`0x15`): `<method>` `0`=GET `1`=POST. Makes the HTTP(S) request over Wi-Fi,
  sets `R[statusReg]` = HTTP status (`0` on failure), and retains the body for the
  inspection ops. POST sends `Content-Type: application/json`.
* `JSON_NUM` (`0x16`): `R[dst]` = number at `<path>` × 10^`<scale>` (truncated),
  `R[found]` = `1`/`0`. `<path>` is dotted/indexed, e.g. `result[0].changePercent`.
  `JSON_STR_EQ`/`JSON_STR_CONT` (`0x17`/`0x19`): `R[dst]` = `1`/`0` from comparing the
  JSON string at `<path>`. `BODY_CONTAINS` (`0x18`): `R[dst]` = `1`/`0` substring over
  the whole body. `HTTP_REPLY` carries status (`lo hi`) + body (14-bit pairs, up to
  ~4 KB) back to a connected host.

`if`/`else` is laid out so the byte counts line up:
`[IF skip=thenLen] [then bytes] [SKIP skip=elseLen] [else bytes]` — a false `IF`
skips the whole then-block (landing on `else`); a true one runs `then`, whose
trailing `SKIP` jumps over `else`.

The base Scheduler control messages (`CREATE_TASK` `0x00`, `ADD_TO_TASK` `0x02`,
`SCHEDULE_TASK` `0x04`, `DELAY_TASK` `0x03`, `QUERY` `0x06`/`0x05`, `RESET` `0x07`)
are unchanged from standard Firmata.

## Verified
* Dual build (Wi-Fi + BLE) compiles (`min_spiffs`, ~98% flash; use *Huge APP* for
  headroom); single-transport builds also compile.
* Latest-wins eviction, Scheduler, digital/analog I/O and the logic extension
  verified live against the board.
* Internet actions verified live: `http://` GET returns status + body in the dual
  build; `https://` (cert-validated) GET returns 200 + body in the Wi-Fi-only build.
  The JSON/string inspection ops are a direct port of the (hardware-verified)
  ESP32FirmataSwift implementation.
