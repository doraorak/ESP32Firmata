# Scheduler logic extension (this branch)

This branch (`nonstandard-scheduler-logic`) adds an **on-device logic layer** on top
of the Firmata Scheduler so a stored task can make decisions by itself — registers,
reads into registers, and `if`/`else` — instead of being a flat replay of recorded
messages. The standard-compliant firmware is on `main` / `standard`.

## How standard is it?

The **Scheduler control protocol is untouched** — tasks are created, filled, and
scheduled with the ordinary `CREATE_TASK` / `ADD_TO_TASK` / `SCHEDULE_TASK`
messages. The logic lives entirely inside the **task data payload** (the bytes the
scheduler replays), exactly where the standard already puts `DELAY_TASK`.

The logic ops ride under **`EXTENDED_SCHEDULER_COMMAND` (`0x7F`)** — the sub-command
the reference ConfigurableFirmata scheduler reserves for *"extended schedulers"* and
ignores in its base form. So:

- A **standard** scheduler that replays one of these tasks **ignores** the `0x7F`
  messages — it won't crash; the conditionals/registers are simply no-ops (every
  branch runs). Graceful degradation.
- This firmware still accepts ordinary (non-extended) tasks unchanged.

So it's a **conformant scheduler extension** using the documented extension hook —
not a protocol violation. It's "non-standard" only in that the specific ops are
ours; another host won't *act on* them.

## What it adds

`SCHEDULER_DATA` (`0x7B`) → `EXTENDED_SCHEDULER_COMMAND` (`0x7F`) → one of:

| Ext sub-cmd | Meaning |
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

All messages are SysEx embedded in a task's data and replayed by the scheduler.
`<const>` is an Int32 packed as 5 Encoder7Bit bytes (same packing the standard
scheduler uses for time values). `<skip>` is a 14-bit byte count, little-endian
7-bit: `skipLo skipHi`.

```
SET           F0 7B 7F 10 <reg> <const:5>                     F7   // R[reg] = const
READ_DIGITAL  F0 7B 7F 11 <reg> <pin>                         F7   // R[reg] = digitalRead(pin)
READ_ANALOG   F0 7B 7F 12 <reg> <channel>                     F7   // R[reg] = analogRead(channel)
IF            F0 7B 7F 13 <op> <operandA> <operandB> <skip:2> F7   // if !(A op B): pos += skip
SKIP          F0 7B 7F 14 <skip:2>                            F7   // pos += skip (else)
```

- `<reg>`: register index, low nibble used (`0`–`15`).
- `<op>`: `0 ==`, `1 !=`, `2 <`, `3 >`, `4 <=`, `5 >=`.
- `<operand>`: a **type byte** then its data — `00 <reg>` (register) or
  `01 <const:5>` (literal).
- `<pin>`: GPIO number. `<channel>`: analog channel index (A0 = 0…), **not a pin**.

The host (`FirmataTaskRecorder`) lays out `if`/`else` so the byte counts line up:
`[IF skip=thenLen] [then bytes] [SKIP skip=elseLen] [else bytes]`. When the
condition is false the `IF` skips the whole then-block (landing on the else block);
when true, the then-block ends with the `SKIP` that jumps over the else.

## Why it's a separate branch

Other Firmata hosts won't *implement* these ops (they ignore them), so keep it out
of the standard line unless you knowingly want the extension. Pairs with the
matching `SwiftFirmataClient` `nonstandard-scheduler-logic` branch.
