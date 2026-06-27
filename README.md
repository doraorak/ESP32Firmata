# ESP32 Firmata firmware (Bonjour + BLE) — with on-device logic extension

Firmata 2.x firmware for the **original ESP32** (ESP32-WROOM-32 / -WROVER),
byte-for-byte compatible with the
[**SwiftFirmataClient**](https://github.com/doraorak/SwiftFirmataClient) package.

## The project suite

Part of a three-repo Firmata-for-ESP32 suite — grab whichever piece you need:

- **[ESP32Firmata](https://github.com/doraorak/ESP32Firmata)** — the C++/Arduino ESP32 firmware *(this repo)*.
- **[ESP32FirmataSwift](https://github.com/doraorak/ESP32FirmataSwift)** — the Embedded-Swift firmware port (same wire protocol).
- **[SwiftFirmataClient](https://github.com/doraorak/SwiftFirmataClient)** — the macOS/iOS Swift client package.

## Flash it

**Arduino IDE:** open `ESP32Firmata.ino`, choose **ESP32 Dev Module** + **Partition Scheme → Huge APP (3 MB)**, pick your port, hit **Upload**.

**One command** (same thing, from this folder in Terminal — needs [`arduino-cli`](https://arduino.github.io/arduino-cli/)):

```bash
./flash.sh      # auto-detects the port, builds + uploads
./monitor.sh    # serial log — shows the board's Wi-Fi IP / Bonjour name
```

Pass a port explicitly with `./flash.sh /dev/cu.XXXX` if needed.

Prefer not to build? A **prebuilt `.bin`** is attached to each
[release](https://github.com/doraorak/ESP32Firmata/releases) — flash it with `esptool` or
[ESP Web Tools](https://esp.huhn.me), then hand it your Wi-Fi over BLE (see below — no
rebuild needed).

## Wi-Fi credentials — two ways

**A · Compile-time (works out of the box).** Set `WIFI_SSID` / `WIFI_PASS` at the top of
`ESP32Firmata.ino` before flashing; the board joins your network on boot. Simplest when you
build it yourself.

**B · Provision over BLE (no rebuild — ideal for the prebuilt `.bin`).** Leave the
placeholders, flash, then send credentials from the client over BLE:

```swift
let client = FirmataClient(transport: BLETransport())   // Wi-Fi is down, so use BLE
await client.connect()
let status = try await client.provisionWiFi(ssid: "MyNetwork", password: "hunter2")
print(status.connected, status.ip ?? "—")               // e.g. true 192.168.1.50
```

The handshake is **encrypted** — an ephemeral X25519 ECDH → HKDF-SHA256 → AES-256-GCM, so
a passive sniffer never sees the password (no BLE pairing required). Provisioned creds are
saved on the device (NVS) and **override** the compile-time defaults on every boot; clear
them with `client.forgetWiFi()`. If the new credentials don't actually connect, the device
**rolls back** to its previous network and doesn't store them — a wrong password can't
strand the board. (No pairing means it's not hardened against an active
real-time MITM during the handshake — fine for the typical "set it up on your own bench"
case.)

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
2. **Library Manager** → install **NimBLE-Arduino** (≥ 2.x) — the BLE transport
   uses NimBLE (lighter than Bluedroid, so `https://` fits alongside Wi-Fi + BLE).
   Not needed if you build with `ENABLE_BLE 0`.
3. Open `ESP32Firmata.ino`; set `WIFI_SSID` / `WIFI_PASS` in the USER
   CONFIGURATION block (and optionally set `ENABLE_WIFI`/`ENABLE_BLE` to `0`).
4. **Board**: *ESP32 Dev Module*. **Partition Scheme**: *Huge APP (3 MB)*
   recommended (the build is large; NimBLE keeps it well under the limit).
5. Upload; open Serial Monitor @ **115200** for IP / advertising status.

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

> **HTTPS** is validated against the IDF cert bundle and works in the **dual
> Wi-Fi + BLE build**. This needs the BLE stack to be **NimBLE**, not Bluedroid —
> Bluedroid leaves only ~40 KB free heap, too little for the Arduino core's TLS
> buffers (its prebuilt mbedtls has `ASYMMETRIC_CONTENT_LEN` off → ~32 KB), so a
> handshake fails with `SSL - Memory allocation failed`. NimBLE frees enough heap
> (and ~480 KB of flash). This sketch uses NimBLE — see *Setup* for the one-time
> library install.

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
ARITH          F0 7B 7F 1A <subop> <dst> <operandA> <operandB>          F7  // R[dst] = A op B (int)
SET_FLOAT      F0 7B 7F 1B <fdst> <const:5>                             F7  // F[fdst] = float
ARITH_F        F0 7B 7F 1C <subop> <fdst> <operandA> <operandB>         F7  // F[fdst] = A op B (float)
JSON_FLOAT     F0 7B 7F 1D <fdst> <found> <pathLen:2> <path…>           F7  // F[fdst] = json float
JSON_TYPE      F0 7B 7F 1E <dst> <pathLen:2> <path…>                    F7  // R[dst] = type at path
JSON_SIZE      F0 7B 7F 1F <dst> <pathLen:2> <path…>                    F7  // R[dst] = span byte length
STR_LEN        F0 7B 7F 20 <dst> <pathLen:2> <path…>                    F7  // R[dst] = string content length
HEAP           F0 7B 7F 21 <freeReg> <largestReg>                       F7  // R = free heap / largest block
REQUEST_COUNT  F0 7B 7F 22 <dst>                                        F7  // R[dst] = request generation
SNAPSHOT       F0 7B 7F 23 <slot> <pathLen:2> <path…>                   F7  // copy value -> snapshot slot
SELECT         F0 7B 7F 24 <sel> <expGenReg>                            F7  // 0=live(gen-checked), k=snap k-1
FREE           F0 7B 7F 25 <slot>                                       F7  // free a snapshot slot
LAST_STATUS    F0 7B 7F 26 <dst>                                        F7  // R[dst] = last inspection status
CMP            F0 7B 7F 27 <op> <dst> <operandA> <operandB>             F7  // R[dst] = (A op B) ? 1 : 0
HTTP_REPLY     F0 7B 0B <status:2> <body 14-bit pairs…>                  F7  // device -> host
```

* `<reg>`: int register index, low nibble (`0`–`15`). `<fdst>`: float register (`0`–`7`).
* `<op>`: `0 ==`, `1 !=`, `2 <`, `3 >`, `4 <=`, `5 >=`. `<subop>`: `0 +`, `1 −`, `2 ×`, `3 ÷`, `4 %`.
* `<operand>`: a type byte then data — `00 <reg>` (int register), `01 <const:5>` (int
  literal), `02 <freg>` (float register), or `03 <const:5>` (float literal, IEEE754 bits).
  `IF`/`ARITH`/`ARITH_F`/`CMP` accept any type; if either side is float the device promotes.
* `<pin>`: GPIO number. `<channel>`: analog channel index (A0 = 0…), **not a pin**.
* `HTTP` (`0x15`): `<method>` `0`=GET `1`=POST. Makes the HTTP(S) request over Wi-Fi,
  sets `R[statusReg]` = HTTP status (`0` on failure), retains the body for the
  inspection ops, and bumps the request generation (`REQUEST_COUNT`). POST sends
  `Content-Type: application/json`.
* `JSON_NUM` (`0x16`): `R[dst]` = number at `<path>` × 10^`<scale>` (truncated),
  `R[found]` = `1`/`0`. `<path>` is dotted/indexed, e.g. `result[0].changePercent`.
  `JSON_STR_EQ`/`JSON_STR_CONT` (`0x17`/`0x19`): `R[dst]` = `1`/`0` from comparing the
  JSON string at `<path>`. `BODY_CONTAINS` (`0x18`): `R[dst]` = `1`/`0` substring over
  the whole body. `JSON_FLOAT` (`0x1D`) reads a JSON number (quoted/fractional/exponent)
  into `F[fdst]`. `JSON_TYPE` (`0x1E`) → `0` none, `1` obj, `2` arr, `3` str, `4` num,
  `5` bool, `6` null. `JSON_SIZE` (`0x1F`) → span byte length; `STR_LEN` (`0x20`) → string
  content length; `HEAP` (`0x21`) → free heap + largest block.
* Handles: `REQUEST_COUNT` (`0x22`) reads the generation (++ per request). `SNAPSHOT`
  (`0x23`) copies a value into one of 5 grow-only slots that outlive the next request
  (`<pathLen>` 0 = whole body). `SELECT` (`0x24`) picks the inspection source: `0` = live
  (marked **stale** if `requestCount != R[expGenReg]`), `k` = snapshot slot `k-1`. `FREE`
  (`0x25`) releases a slot. `LAST_STATUS` (`0x26`) → the last inspection op's status
  (`0` ok, `1` notFound, `2` stale, `3` typeMismatch, `4` tooBig, `5` allocFailed). `CMP`
  (`0x27`) materialises `(A op B) ? 1 : 0` into a register (same operands/promotion as `IF`).
  `HTTP_REPLY` carries status (`lo hi`) + body (14-bit pairs, up to ~4 KB) back to a host.
* `JSON_GET_STRING` (`0x2C`): copy the **content** (unquoted) of the JSON string at `<path>`
  from the live body into a snapshot slot — backs `board.json.getString` → a `StringHandle`.
* `STR_SET_SLOT` (`0x2D`): set a snapshot slot to a **literal** string from the payload —
  backs the standalone `StringHandle("…", on: board)` (a `board.string` value, no HTTP body).
* Raw-string ops on a selected string (`board.string`): `STR_BODY_LEN` (`0x28`) → byte
  length; `STR_EQUALS` (`0x29`) → `== <str>` ? 1 : 0; `STR_INDEXOF` (`0x2A`) → index of
  `<str>`, or `-1`; `STR_TO_NUM` (`0x2B`) → leading integer into `R[dst]`, `R[found]`=`1`/`0`.
  (`contains` reuses `BODY_CONTAINS` `0x18`.)

`if`/`else` is laid out so the byte counts line up:
`[IF skip=thenLen] [then bytes] [SKIP skip=elseLen] [else bytes]` — a false `IF`
skips the whole then-block (landing on `else`); a true one runs `then`, whose
trailing `SKIP` jumps over `else`.

The base Scheduler control messages (`CREATE_TASK` `0x00`, `ADD_TO_TASK` `0x02`,
`SCHEDULE_TASK` `0x04`, `DELAY_TASK` `0x03`, `QUERY` `0x06`/`0x05`, `RESET` `0x07`)
are unchanged from standard Firmata.

## Verified
* Dual build (Wi-Fi + BLE, NimBLE) compiles (`huge_app`, ~46% flash) and boots:
  Wi-Fi + Bonjour up and BLE advertising at the same time.
* Latest-wins eviction, Scheduler, digital/analog I/O and the logic extension
  verified live against the board.
* Internet actions verified live in the **dual** build: `http://` and
  **`https://` (cert-validated)** GET both return 200 + body — e.g. example.com
  (559 B) and api JSON (83 B). The JSON/string inspection ops mirror the
  (hardware-verified) ESP32FirmataSwift implementation.
* Full logic-extension parity with ESP32FirmataSwift — ops `0x10`–`0x27`, including
  integer + float operands, float-promoting `IF`/`CMP`, `ARITH`/`ARITH_F`,
  `JSON_FLOAT`/`JSON_TYPE`/`JSON_SIZE`/`STR_LEN`, `HEAP`, and the snapshot/select/
  staleness model (`SNAPSHOT`/`SELECT`/`FREE`/`REQUEST_COUNT`/`LAST_STATUS`).
  Hardware-verified on the board: `CMP` (true/false/float-promoted), `ARITH`, and
  `isValid()` (fresh + stale) all pass end-to-end via pin-state read-back.
