// IR module (id 0x01) + the generic module subsystem, split out of the main
// sketch. Arduino concatenates every .ino into one translation unit (the main
// sketch first, then the others alphabetically), so this file freely uses the
// globals and helpers defined in ESP32Firmata.ino (scheduler, frameBuf,
// sendFrame, the protocol constants, …).

/* ==== Module subsystem ====================================================
   Compile-time firmware plugins behind one reserved SysEx (MODULE_DATA 0x0D)
   and one task ext op (MODULE_OP 0x33) — byte-identical to ESP32FirmataSwift.
   Modules put results in the scheduler registers, which plugs them into
   ifTrue/tasks for free.

   A module is a class deriving from `ModuleHandler`: it owns its state, handles
   the wire payloads addressed to its `id()`, and is `tick()`ed every loop. Add a
   module by giving `modules[]` one more instance — discovery (MODULE_QUERY) and
   dispatch read `id()/major()/minor()/name()` off it automatically. (This mirrors
   the Swift firmware's `ModuleHandler` protocol; the two stay wire-identical.) */

struct ModuleHandler {
  virtual uint8_t id()    const = 0;
  virtual uint8_t major() const = 0;
  virtual uint8_t minor() const = 0;
  virtual const char *name() const = 0;
  virtual void handle(const uint8_t *payload, int length) = 0;
  virtual void tick() = 0;
  virtual ~ModuleHandler() {}
};

/* ==== IR module (NEC over RMT) ============================================
   ops: 0x00 <pin> = configure TX (carrier set per send); 0x02 <pin> <dstReg> =
   start RX — each decoded NEC frame lands in R[dstReg] and is pushed to the host
   as event 0x03; 0x03 <carrierKHz> <dur pairs> = raw mark/space send (any
   protocol encoded host-side); 0x04 = repeat/hold (legacy A/B toggle, re-sent by
   txTick); 0x05 <protocol> <srcReg> = encode+send a numeric code from a register. */
struct IRModuleHandler : ModuleHandler {
  uint8_t id()    const override { return 0x01; }
  uint8_t major() const override { return 1; }
  uint8_t minor() const override { return 0; }
  const char *name() const override { return "ir"; }

  int      txPin = -1;
  int      rxPin = -1;
  int      dstReg = 0;
  rmt_data_t rxSyms[96];
  size_t   rxLen = 0;
  bool     rxArmed = false;

  // Staged frame(s) for transmit: frame A, then an optional frame B (the RC6
  // toggle variant). Durations are alternating mark/space (µs); marks HIGH.
  rmt_data_t sym[80];        // frame A is sym[0..symA); frame B is sym[symA..symTotal)
  int        symA = 0;       // symbols in frame A
  int        symTotal = 0;
  int        txSendIdx = 0;  // send # in a repeat run (even=A, odd=B: RC6 toggle)
  int        txRepeatLeft = 0;
  uint32_t   txGapMs = 0;
  uint32_t   txNextMs = 0;

  // Stage 14-bit LE duration pairs from payload[start...] into sym symbols; return the count.
  int stageSyms(const uint8_t *payload, int length, int start) {
    int count = 0, index = start;
    while (index + 1 < length && count < 80) {
      uint32_t markDuration = (uint32_t)(payload[index] & 0x7F) | ((uint32_t)(payload[index + 1] & 0x7F) << 7); index += 2;
      uint32_t spaceDuration = 1;
      if (index + 1 < length) { spaceDuration = (uint32_t)(payload[index] & 0x7F) | ((uint32_t)(payload[index + 1] & 0x7F) << 7); index += 2; }
      sym[count].level0 = 1; sym[count].duration0 = markDuration ? markDuration : 1;
      sym[count].level1 = 0; sym[count].duration1 = spaceDuration ? spaceDuration : 1;
      count++;
    }
    return count;
  }

  // Send frame A (even index) or frame B (odd index, when a distinct B exists — the RC6 toggle
  // variant). NEC/raw have no B → always A. Named txFrame, not sendFrame, to avoid shadowing the
  // global sendFrame(buf, len).
  void txFrame(int sendIndex) {
    if (symA <= 0) return;
    bool useFrameB = (symTotal > symA) && (sendIndex & 1);
    int offset = useFrameB ? symA : 0;
    int count  = useFrameB ? symTotal - symA : symA;
    rmtWrite(txPin, sym + offset, count, 200);
  }

  // op 0x03: single raw send. payload = [0x03, carrierKHz, dur pairs...].
  void sendRaw(const uint8_t *payload, int length) {
    if (txPin < 0 || length < 4) return;
    uint32_t carrierKHz = payload[1] & 0x7F;
    rmtSetCarrier(txPin, carrierKHz > 0, false, carrierKHz * 1000, 0.33f);
    txRepeatLeft = 0;                   // cancel any pending repeat
    symTotal = stageSyms(payload, length, 2);
    symA = symTotal;                    // one frame, no B
    txFrame(0);
  }

  // op 0x04: repeat/hold. payload = [0x04, carrierKHz, repeat, gapLo, gapHi, nA_lo, nA_hi, A durs, B durs].
  // nA = durations in frame A (even); the rest are B (RC6 toggle variant). No B → repeats send A.
  void sendRepeat(const uint8_t *payload, int length) {
    if (txPin < 0 || length < 8) return;
    uint32_t carrierKHz = payload[1] & 0x7F;
    rmtSetCarrier(txPin, carrierKHz > 0, false, carrierKHz * 1000, 0.33f);
    int repeats = payload[2] & 0x7F;
    txGapMs = (uint32_t)(payload[3] & 0x7F) | ((uint32_t)(payload[4] & 0x7F) << 7);
    int frameADurations = (int)(payload[5] & 0x7F) | ((int)(payload[6] & 0x7F) << 7);   // durations in A (even)
    symTotal = stageSyms(payload, length, 7);
    int frameASymbols = frameADurations / 2;
    symA = (frameASymbols > 0 && frameASymbols < symTotal) ? frameASymbols : symTotal;
    txFrame(0);
    txSendIdx = 1;
    txRepeatLeft = repeats > 1 ? repeats - 1 : 0;
    txNextMs = millis() + txGapMs;
  }

  void txTick() {
    if (txRepeatLeft > 0 && txPin >= 0 && (int32_t)(millis() - txNextMs) >= 0) {
      txFrame(txSendIdx);
      txSendIdx++;
      txRepeatLeft--;
      txNextMs = millis() + txGapMs;
    }
  }

  // On-device NEC/RC6 encoders (op 0x05): build the timing waveform for a numeric code held in a
  // register (a runtime value the host can't pre-encode). Pack into sym; return the symbol count.
  // Mirrors SwiftFirmataIR's host-side necTiming/rc6Timing.
  int packSyms(const int *durations, int count) {
    int nSyms = 0, k = 0;
    while (k < count && nSyms < 80) {
      uint32_t mark  = durations[k++];
      uint32_t space = (k < count) ? durations[k++] : 1;
      sym[nSyms].level0 = 1; sym[nSyms].duration0 = mark  ? mark  : 1;
      sym[nSyms].level1 = 0; sym[nSyms].duration1 = space ? space : 1;
      nSyms++;
    }
    return nSyms;
  }

  int encodeNEC(uint32_t code) {
    int d[80], n = 0;
    d[n++] = 9000; d[n++] = 4500;                                   // 9 ms / 4.5 ms header
    for (int bit = 31; bit >= 0; bit--) { d[n++] = 562; d[n++] = ((code >> bit) & 1) ? 1687 : 562; }
    d[n++] = 562;                                                   // trailing mark
    return packSyms(d, n);
  }

  int encodeRC6(uint32_t data, int bits) {
    const int t = 444;
    int d[96], n = 0;
    auto emit = [&](bool mark, int dur) {                           // merge consecutive same-level runs
      bool expectMark = (n % 2 == 0);
      if (mark == expectMark) { if (n < 96) d[n++] = dur; }
      else if (n > 0) d[n - 1] += dur;
    };
    emit(true, 6*t); emit(false, 2*t);                             // leader
    emit(true, t);   emit(false, t);                               // start bit (always 1)
    int i = 1; uint32_t mask = bits > 0 ? (1u << (bits - 1)) : 0;
    while (mask) {
      int bw = (i == 4) ? 2*t : t;                                 // 4th bit = double-width toggle
      if (data & mask) { emit(true, bw); emit(false, bw); }        // 1
      else             { emit(false, bw); emit(true, bw); }        // 0
      i++; mask >>= 1;
    }
    return packSyms(d, n);
  }

  void handle(const uint8_t *payload, int length) override {
    if (length < 1) return;
    switch (payload[0]) {
      case 0x00:
        // Configure the TX pin. The carrier is set per send by the raw op (0x03).
        if (length >= 2 && rmtInit(payload[1] & 0x7F, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 1000000))
          txPin = payload[1] & 0x7F;
        break;
      case 0x03:
        sendRaw(payload, length);   // raw mark/space replay (NEC, RC6, any protocol encoded host-side)
        break;
      case 0x04:
        sendRepeat(payload, length);   // repeat/hold — alternates A/B (RC6 toggle), re-sends via txTick
        break;
      case 0x05:
        // Encode+send a numeric code from a register: <protocol> <srcReg>. 0=NEC(38k), 1=RC6(36k).
        if (length >= 3 && txPin >= 0) {
          uint8_t  proto = payload[1] & 0x7F;
          uint32_t code  = (uint32_t)scheduler.regs[payload[2] & 0x1F];
          uint32_t khz   = proto == 1 ? 36 : 38;
          rmtSetCarrier(txPin, khz > 0, false, khz * 1000, 0.33f);
          txRepeatLeft = 0;
          symTotal = proto == 1 ? encodeRC6(code, 20) : encodeNEC(code);
          symA = symTotal;
          txFrame(0);
        }
        break;
      case 0x02:
        if (length >= 3 && rmtInit(payload[1] & 0x7F, RMT_RX_MODE, RMT_MEM_NUM_BLOCKS_2, 1000000)) {
          rxPin = payload[1] & 0x7F;
          dstReg = payload[2] & 0x1F;
          rmtSetRxMaxThreshold(rxPin, 12000);
          // Glitch filter runs on the 80 MHz source clock (8-bit, max ~255 cycles ~= 3.2 us),
          // NOT the 1 MHz resolution clock. Larger values make rmt_receive() reject the config
          // and reception silently never starts. 2 us is safely under the limit.
          rmtSetRxMinThreshold(rxPin, 2);
          rxLen = sizeof(rxSyms) / sizeof(rxSyms[0]);
          rxArmed = rmtReadAsync(rxPin, rxSyms, &rxLen);
        }
        break;
      default: break;
    }
  }

  static bool near(int32_t value, int32_t target) {
    return value > target - target / 4 && value < target + target / 4;
  }

  void tick() override {
    txTick();
    if (rxPin < 0 || !rxArmed || !rmtReceiveCompleted(rxPin)) return;
    static int32_t durations[192];
    int count = 0;
    for (size_t i = 0; i < rxLen && count + 1 < 192; i++) {
      durations[count++] = (int32_t)rxSyms[i].duration0;
      durations[count++] = (int32_t)rxSyms[i].duration1;
    }
    rxLen = sizeof(rxSyms) / sizeof(rxSyms[0]);
    rxArmed = rmtReadAsync(rxPin, rxSyms, &rxLen);
    if (count < 66) return;
    // Find the 9 ms / 4.5 ms header, then read 32 mark/space bit pairs.
    int index = 0;
    while (index + 1 < count && !(near(durations[index], 9000) && near(durations[index + 1], 4500))) index++;
    if (index + 66 > count) return;
    index += 2;
    uint32_t code = 0;
    for (int bitIndex = 0; bitIndex < 32; bitIndex++) {
      if (!near(durations[index], 562)) return;
      if (near(durations[index + 1], 1687)) code = (code << 1) | 1;
      else if (near(durations[index + 1], 562)) code = code << 1;
      else return;
      index += 2;
    }
    scheduler.regs[dstReg] = (int32_t)code;
    uint8_t out[10];
    int outIndex = 0;
    out[outIndex++] = START_SYSEX; out[outIndex++] = MODULE_DATA; out[outIndex++] = id(); out[outIndex++] = 0x03;
    uint32_t remaining = code;
    for (int limb = 0; limb < 5; limb++) { out[outIndex++] = remaining & 0x7F; remaining >>= 7; }
    out[outIndex++] = END_SYSEX;
    sendFrame(out, outIndex);
  }
};

/* ==== Registry + dispatch ================================================
   Every compiled-in module, in one place. */
static IRModuleHandler irModule;
static ModuleHandler *const modules[] = { &irModule };
static const uint8_t MODULE_COUNT = sizeof(modules) / sizeof(modules[0]);

static void moduleDispatch(uint8_t id, const uint8_t *payload, int length) {
  for (uint8_t i = 0; i < MODULE_COUNT; i++)
    if (modules[i]->id() == id) { modules[i]->handle(payload, length); return; }
}

static void moduleTick() {
  for (uint8_t i = 0; i < MODULE_COUNT; i++) modules[i]->tick();
}

static void handleModuleData(const uint8_t *data, int length) {
  if (length < 1) return;
  if (data[0] == MODULE_QUERY) {
    int index = 0;
    frameBuf[index++] = START_SYSEX;
    frameBuf[index++] = MODULE_DATA;
    frameBuf[index++] = MODULE_LIST_REPLY;
    frameBuf[index++] = MODULE_COUNT;
    for (uint8_t moduleIndex = 0; moduleIndex < MODULE_COUNT; moduleIndex++) {
      frameBuf[index++] = modules[moduleIndex]->id();
      frameBuf[index++] = modules[moduleIndex]->major();
      frameBuf[index++] = modules[moduleIndex]->minor();
      const char *moduleName = modules[moduleIndex]->name();
      uint8_t nameLength = (uint8_t)strlen(moduleName);
      frameBuf[index++] = nameLength;
      for (uint8_t charIndex = 0; charIndex < nameLength; charIndex++)
        frameBuf[index++] = moduleName[charIndex] & 0x7F;
    }
    frameBuf[index++] = END_SYSEX;
    sendFrame(frameBuf, index);
    return;
  }
  moduleDispatch(data[0], data + 1, length - 1);
}
