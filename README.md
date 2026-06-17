# ESP32 Firmata firmware (Bonjour + BLE) — with on-device logic extension

Firmata 2.x firmware for the **original ESP32** (ESP32-WROOM-32 / -WROVER),
byte-for-byte compatible with the
[**SwiftFirmataClient**](https://github.com/doraorak/SwiftFirmataClient) package.

Self-contained — it implements the Firmata protocol directly, so the only
dependencies are the ESP32 Arduino core libraries (`WiFi`, `ESPmDNS`, `BLE*`,
`Wire`). One sketch runs **both transports at once** — Wi-Fi/TCP + Bonjour **and**
BLE — so a client can connect either way with no reflashing.

> **Branches:** `main` (this one) includes the **Scheduler logic extension**
> (registers + `if`/`else`, documented below). The
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
* **On-device logic extension** (this branch) — see *Custom protocol* below

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

### Byte commands (wire format)

All ops are SysEx embedded in a task's data and replayed by the Scheduler, under
`SCHEDULER_DATA` (`0x7B`) → `EXTENDED_SCHEDULER_COMMAND` (`0x7F`). `<const>` is an
Int32 as 5 Encoder7Bit bytes; `<skip>` is a 14-bit count, little-endian 7-bit
(`skipLo skipHi`).

```
SET           F0 7B 7F 10 <reg> <const:5>                     F7   // R[reg] = const
READ_DIGITAL  F0 7B 7F 11 <reg> <pin>                         F7   // R[reg] = digitalRead(pin)
READ_ANALOG   F0 7B 7F 12 <reg> <channel>                     F7   // R[reg] = analogRead(channel)
IF            F0 7B 7F 13 <op> <operandA> <operandB> <skip:2> F7   // if !(A op B): pos += skip
SKIP          F0 7B 7F 14 <skip:2>                            F7   // pos += skip (else)
```

* `<reg>`: register index, low nibble (`0`–`15`).
* `<op>`: `0 ==`, `1 !=`, `2 <`, `3 >`, `4 <=`, `5 >=`.
* `<operand>`: a type byte then data — `00 <reg>` (register) or `01 <const:5>` (literal).
* `<pin>`: GPIO number. `<channel>`: analog channel index (A0 = 0…), **not a pin**.

`if`/`else` is laid out so the byte counts line up:
`[IF skip=thenLen] [then bytes] [SKIP skip=elseLen] [else bytes]` — a false `IF`
skips the whole then-block (landing on `else`); a true one runs `then`, whose
trailing `SKIP` jumps over `else`.

The base Scheduler control messages (`CREATE_TASK` `0x00`, `ADD_TO_TASK` `0x02`,
`SCHEDULE_TASK` `0x04`, `DELAY_TASK` `0x03`, `QUERY` `0x06`/`0x05`, `RESET` `0x07`)
are unchanged from standard Firmata.

## Verified
* Dual build (Wi-Fi + BLE) compiles (`min_spiffs`, ~88% flash); single-transport
  builds also compile.
* Latest-wins eviction, Scheduler, digital/analog I/O and the logic extension
  verified live against the board.
