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

- **16 global Int32 registers**, shared across tasks, reset by `SYSTEM_RESET`.
- **Forward-only skips** (no backward jumps), so a task can branch but can never
  loop without the scheduler's normal trailing-delay reschedule — it cannot hang
  the board.

## Why it's a separate branch

This only works with the matching `SwiftFirmataClient` `nonstandard-scheduler-logic`
branch and this firmware. No other Firmata host understands these bytes. Keep it
out of the standard line unless you knowingly want the extension.
