// IR module (id 0x01), split out of the main sketch. Arduino concatenates every
// .ino into one translation unit (main sketch first, then the others
// alphabetically), so this file freely uses the globals and helpers defined in
// ESP32Firmata.ino (scheduler, frameBuf, sendFrame, ModuleHandler, ...). The
// module registry lives in ZRegistry.ino (sorts last, sees every instance).

/* ==== IR module (NEC over RMT) ============================================
   ops: 0x00 <pin> = configure TX (carrier set per send); 0x02 <pin> <dstReg> =
   start RX — each decoded NEC frame lands in R[dstReg] and is pushed to the host
   as event 0x03; 0x03 <carrierKHz> <dur pairs> = raw mark/space send (any
   protocol encoded host-side); 0x04 = repeat/hold (legacy A/B toggle, re-sent by
   txTick); 0x05 <protocol> <srcReg> = encode+send a numeric code from a register;
   0x06 <pin> <enable> = raw capture — every received burst (any protocol) is pushed
   as event 0x07 <totalLo> <totalHi> <durations as 14-bit LE pairs> for protocol
   sniffing (AC remotes etc.); NEC decode keeps running alongside. */
struct IRModuleHandler : ModuleHandler {
  uint8_t id()    const override { return 0x01; }
  uint8_t major() const override { return 1; }
  uint8_t minor() const override { return 2; }
  const char *name() const override { return "ir"; }

  int      txPin = -1;
  int      rxPin = -1;
  int      dstReg = 0;
  uint8_t  rxProtocol = 0;                    // 0 NEC, 1 RC6, 2 Coolix
  bool     rawReport = false;                 // op 0x06: mirror captures to the host
  rmt_data_t rxSyms[128];                     // full 2-block capacity: 256 durations
  size_t   rxLen = 0;
  bool     rxArmed = false;

  // Staged frame(s) for transmit: frame A, then an optional frame B (the RC6
  // toggle variant). Durations are alternating mark/space (µs); marks HIGH.
  rmt_data_t sym[112];       // frame A is sym[0..symA); frame B is sym[symA..symTotal)
                             // (sized for a doubled Coolix message: 100 symbols)
  int        symA = 0;       // symbols in frame A
  int        symTotal = 0;
  int        txSendIdx = 0;  // send # in a repeat run (even=A, odd=B: RC6 toggle)
  int        txRepeatLeft = 0;
  uint32_t   txGapMs = 0;
  uint32_t   txNextMs = 0;

  // Stage 14-bit LE duration pairs from payload[start...] into sym symbols; return the count.
  int stageSyms(const uint8_t *payload, int length, int start) {
    int count = 0, index = start;
    while (index + 1 < length && count < 112) {
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
    while (k < count && nSyms < 112) {
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

  // Coolix (Midea AC family), 38 kHz: 4692/4416 us header, 552 us marks, space 1656 = 1 /
  // 552 = 0. The 24-bit code goes out as byte+complement pairs (48 wire bits), and the
  // whole message is sent twice — real Coolix remotes always double it.
  int encodeCoolix(uint32_t code) {
    int d[224], n = 0;
    for (int sec = 0; sec < 2; sec++) {
      d[n++] = 4692; d[n++] = 4416;
      for (int byteIndex = 2; byteIndex >= 0; byteIndex--) {
        uint32_t byte = (code >> (byteIndex * 8)) & 0xFF;
        for (int half = 0; half < 2; half++) {
          uint32_t value = half ? (~byte & 0xFF) : byte;
          for (int bit = 7; bit >= 0; bit--) {
            d[n++] = 552;
            d[n++] = ((value >> bit) & 1) ? 1656 : 552;
          }
        }
      }
      d[n++] = 552;                                       // footer mark
      if (sec == 0) d[n++] = 5244;                        // gap before the repeat
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
        // Encode+send a numeric code from a register: <protocol> <srcReg>.
        // 0=NEC(38k), 1=RC6(36k), 2=Coolix(38k).
        if (length >= 3 && txPin >= 0) {
          uint8_t  proto = payload[1] & 0x7F;
          uint32_t code  = (uint32_t)scheduler.regs[payload[2] & 0x1F];
          uint32_t khz   = proto == 1 ? 36 : 38;
          rmtSetCarrier(txPin, khz > 0, false, khz * 1000, 0.33f);
          txRepeatLeft = 0;
          symTotal = proto == 1 ? encodeRC6(code, 20)
                   : proto == 2 ? encodeCoolix(code)
                   : encodeNEC(code);
          symA = symTotal;
          txFrame(0);
        }
        break;
      case 0x02:
        if (length >= 3 && rmtInit(payload[1] & 0x7F, RMT_RX_MODE, RMT_MEM_NUM_BLOCKS_2, 1000000)) {
          rxPin = payload[1] & 0x7F;
          dstReg = payload[2] & 0x1F;
          rxProtocol = length >= 4 ? (payload[3] & 0x7F) : 0;
          rmtSetRxMaxThreshold(rxPin, 12000);
          // Glitch filter runs on the 80 MHz source clock (8-bit, max ~255 cycles ~= 3.2 us),
          // NOT the 1 MHz resolution clock. Larger values make rmt_receive() reject the config
          // and reception silently never starts. 2 us is safely under the limit.
          rmtSetRxMinThreshold(rxPin, 2);
          rxLen = sizeof(rxSyms) / sizeof(rxSyms[0]);
          rxArmed = rmtReadAsync(rxPin, rxSyms, &rxLen);
        }
        break;
      case 0x06:
        // Raw capture on/off. Enabling (re)arms the receiver on <pin>; disabling only
        // stops the reports (the receiver stays armed for NEC decode).
        if (length >= 3) {
          if ((payload[2] & 0x7F) != 0) {
            if (rmtInit(payload[1] & 0x7F, RMT_RX_MODE, RMT_MEM_NUM_BLOCKS_2, 1000000)) {
              rxPin = payload[1] & 0x7F;
              rmtSetRxMaxThreshold(rxPin, 12000);
              rmtSetRxMinThreshold(rxPin, 2);
              rxLen = sizeof(rxSyms) / sizeof(rxSyms[0]);
              rxArmed = rmtReadAsync(rxPin, rxSyms, &rxLen);
              rawReport = true;
            }
          } else {
            rawReport = false;
          }
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
    static int32_t durations[256];
    int count = 0;
    for (size_t i = 0; i < rxLen && count + 1 < 256; i++) {
      durations[count++] = (int32_t)rxSyms[i].duration0;
      durations[count++] = (int32_t)rxSyms[i].duration1;
    }
    rxLen = sizeof(rxSyms) / sizeof(rxSyms[0]);
    rxArmed = rmtReadAsync(rxPin, rxSyms, &rxLen);
    if (rawReport && count >= 6) {              // skip sub-3-symbol noise blips
      static uint8_t rawOut[6 + 256 * 2 + 1];
      int outIndex = 0;
      rawOut[outIndex++] = START_SYSEX;
      rawOut[outIndex++] = MODULE_DATA;
      rawOut[outIndex++] = id();
      rawOut[outIndex++] = 0x07;
      rawOut[outIndex++] = count & 0x7F;
      rawOut[outIndex++] = (count >> 7) & 0x7F;
      for (int i = 0; i < count; i++) {
        int32_t d = durations[i];
        if (d < 0) d = 0;
        if (d > 16383) d = 16383;
        rawOut[outIndex++] = d & 0x7F;
        rawOut[outIndex++] = (d >> 7) & 0x7F;
      }
      rawOut[outIndex++] = END_SYSEX;
      sendFrame(rawOut, outIndex);
    }
    // All protocols decode the same raw capture the sniffer sees (op 0x02's protocol
    // byte picks the decoder; a burst that doesn't parse is simply ignored).
    if (rxProtocol == 1)      decodeRC6Capture(durations, count);
    else if (rxProtocol == 2) decodeCoolixCapture(durations, count);
    else                      decodeNECCapture(durations, count);
  }

  // Deliver a decoded frame: destination register + event 0x03 to the host.
  void emitReceived(uint32_t code) {
    scheduler.regs[dstReg] = (int32_t)code;
    uint8_t out[10];
    int outIndex = 0;
    out[outIndex++] = START_SYSEX; out[outIndex++] = MODULE_DATA; out[outIndex++] = id(); out[outIndex++] = 0x03;
    uint32_t remaining = code;
    for (int limb = 0; limb < 5; limb++) { out[outIndex++] = remaining & 0x7F; remaining >>= 7; }
    out[outIndex++] = END_SYSEX;
    sendFrame(out, outIndex);
  }

  // NEC: 9 ms / 4.5 ms header, then 32 bits of 562 us mark + 562/1687 us space.
  void decodeNECCapture(const int32_t *durations, int count) {
    if (count < 66) return;
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
    emitReceived(code);
  }

  // Coolix: ~4.5/4.4 ms header, 552 us marks, space 1656 = 1 / 552 = 0; 48 wire bits =
  // three byte+complement pairs, folded to the 24-bit code after the complements check.
  // Real remotes double the message; the first section in the capture is enough.
  void decodeCoolixCapture(const int32_t *durations, int count) {
    if (count < 98) return;
    int index = 0;
    while (index + 1 < count && !(near(durations[index], 4550) && near(durations[index + 1], 4400))) index++;
    if (index + 98 > count) return;
    index += 2;
    uint64_t bits = 0;
    // Wide windows, split at 1 ms: real receivers stretch marks (up to ~750 us seen) and
    // shrink zero-spaces (down to ~370 us) — the two space clusters stay far apart.
    for (int bitIndex = 0; bitIndex < 48; bitIndex++) {
      int32_t mark = durations[index], space = durations[index + 1];
      if (mark < 350 || mark > 950) return;
      if (space > 1000 && space < 2400) bits = (bits << 1) | 1;
      else if (space > 250 && space <= 1000) bits = bits << 1;
      else return;
      index += 2;
    }
    uint32_t code = 0;
    for (int byteIndex = 0; byteIndex < 3; byteIndex++) {
      uint32_t byte       = (uint32_t)(bits >> (40 - byteIndex * 16)) & 0xFF;
      uint32_t complement = (uint32_t)(bits >> (32 - byteIndex * 16)) & 0xFF;
      if (complement != (~byte & 0xFF)) return;
      code = (code << 8) | byte;
    }
    emitReceived(code);
  }

  // RC6 mode 0: 6t/2t leader (t = 444 us), a `1` start bit, then Manchester bits with the
  // 4th one double-width (the toggle). Durations are converted to half-bit units and the
  // unit stream is walked bit by bit; 16-32 decoded bits (incl. mode+toggle) = a frame —
  // so a TV key arrives as e.g. 0x0000C or 0x1000C depending on the toggle.
  void decodeRC6Capture(const int32_t *durations, int count) {
    const int32_t t = 444;
    // Flatten durations into a level-per-unit stream (true = mark). A duration that
    // doesn't round to 1..8 units ends the usable stream (idle gap / glitch).
    bool unitLevel[200];
    int unitCount = 0;
    for (int durIndex = 0; durIndex < count && unitCount < 200; durIndex++) {
      int units = (int)((durations[durIndex] + t / 2) / t);
      if (units < 1 || units > 8) break;
      bool isMark = (durIndex % 2) == 0;
      for (int k = 0; k < units && unitCount < 200; k++) unitLevel[unitCount++] = isMark;
    }
    // Leader: 6 mark units + 2 space units; start bit: mark, space (= 1).
    if (unitCount < 12) return;
    int p = 0;
    for (int k = 0; k < 6; k++) { if (!unitLevel[p]) return; p++; }
    for (int k = 0; k < 2; k++) { if (unitLevel[p]) return; p++; }
    if (!(unitLevel[p] && !unitLevel[p + 1])) return;
    p += 2;
    // Manchester bits: first half mark = 1, space = 0; the 4th bit is double-width.
    uint32_t code = 0;
    int bitsRead = 0;
    while (bitsRead < 32) {
      int width = (bitsRead == 3) ? 2 : 1;
      if (p + 2 * width > unitCount) break;
      bool halvesValid = true;
      for (int k = 1; k < width; k++) {
        if (unitLevel[p + k] != unitLevel[p]) halvesValid = false;
        if (unitLevel[p + width + k] != unitLevel[p + width]) halvesValid = false;
      }
      if (!halvesValid || unitLevel[p] == unitLevel[p + width]) break;
      code = (code << 1) | (unitLevel[p] ? 1 : 0);
      p += 2 * width;
      bitsRead++;
    }
    if (bitsRead < 16) return;
    emitReceived(code);
  }
};

/* The module's single instance (registered in ZRegistry.ino). */
static IRModuleHandler irModule;
