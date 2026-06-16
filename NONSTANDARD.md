# ⚠️ Non-standard branch — scheduler logic extension

This branch (`nonstandard-scheduler-logic`) **deliberately steps outside the Firmata
standard.** Everything on `main` / the `standard` branch is standard-compliant
Firmata (interoperable with StandardFirmata, ConfigurableFirmata, and any Firmata
host). This branch is **not**.

## What it adds

A small on-device "logic" layer on top of the Firmata Scheduler so that a stored
task can make decisions by itself — registers, reads into registers, and
`if`/`else` branching — instead of being a flat replay of recorded messages.

New sub-commands under `SCHEDULER_DATA` (`0x7B`), in the `0x10+` range so they
don't collide with the standard scheduler sub-commands (`0x00`–`0x0A`):

| Sub-cmd | Meaning |
|---|---|
| `0x10 SET`          | `R[d] = constant` |
| `0x11 READ_DIGITAL` | `R[d] = digitalRead(pin)` |
| `0x12 READ_ANALOG`  | `R[d] = analogRead(channel)` |
| `0x13 IF`           | compare two operands (reg/const) with `== != < > <= >=`; if **false**, skip the next *N* bytes of task data |
| `0x14 SKIP`         | unconditional forward skip of *N* bytes (used to implement `else`) |

- **16 global Int32 registers** (`R0`–`R15`), shared across tasks, reset by `SYSTEM_RESET`.
- **Forward-only skips** (no backward jumps), so a task can branch but can never
  loop without the scheduler's normal trailing-delay reschedule — it cannot hang
  the board.

## Wire format

All messages are SysEx, embedded in a task's data and replayed by the scheduler
executor. `<const>` is an Int32 packed as 5 Encoder7Bit bytes (same packing the
standard scheduler uses for time values). `<skip>` is a 14-bit byte count, sent
little-endian 7-bit: `skipLo skipHi`.

```
SET           F0 7B 10 <reg> <const:5>                        F7   // R[reg] = const
READ_DIGITAL  F0 7B 11 <reg> <pin>                            F7   // R[reg] = digitalRead(pin)
READ_ANALOG   F0 7B 12 <reg> <channel>                        F7   // R[reg] = analogRead(channel)
IF            F0 7B 13 <op> <operandA> <operandB> <skip:2>    F7   // if !(A op B): pos += skip
SKIP          F0 7B 14 <skip:2>                               F7   // pos += skip (else)
```

- `<reg>`: register index, low nibble used (`0`–`15`).
- `<op>`: `0 ==`, `1 !=`, `2 <`, `3 >`, `4 <=`, `5 >=`.
- `<operand>`: a **type byte** then its data — `00 <reg>` (register) or
  `01 <const:5>` (literal).
- `<pin>`: GPIO number. `<channel>`: analog channel index (A0 = 0…), **not a pin**.

The host (`FirmataTaskRecorder`) lays out `if`/`else` so the byte counts line up:
`[IF skip=thenLen] [then bytes] [SKIP skip=elseLen] [else bytes]`. When the
condition is false the `IF` skips the whole then-block (landing on the else
block); when true, the then-block ends with the `SKIP` that jumps over the else.

## Why it's a separate branch

This only works with the matching `SwiftFirmataClient` `nonstandard-scheduler-logic`
branch and this firmware. No other Firmata host understands these bytes. Keep it
out of the standard line unless you knowingly want the extension.
