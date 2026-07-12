// Sonar module (id 0x02): HC-SR04 ultrasonic distance over trigger/echo pins.
// Mirrors ESP32FirmataSwift's SonarModuleHandler byte-for-byte.

/* ==== Sonar module ========================================================
   Distance lands in a scheduler register (cm), so tasks branch on it directly.
   Ops:
     0x00 <trig> <echo>                  configure the pin pair
     0x01 <dstReg>                       ping once now -> R[dstReg] = cm (-1 = no echo)
     0x02 <dstReg> <perLo> <perHi>       auto-ping every period ms -> R[dstReg]; period 0 stops
   A ping blocks up to ~25 ms (4 m round trip) — fine at handle/tick cadence. */
struct SonarModuleHandler : ModuleHandler {
  uint8_t id()    const override { return 0x02; }
  uint8_t major() const override { return 1; }
  uint8_t minor() const override { return 1; }   // 1.1: op 0x03 one-shot ping -> host reply
  const char *name() const override { return "sonar"; }

  int      trigPin = -1;
  int      echoPin = -1;
  int      autoReg = 0;
  uint32_t autoPeriodMs = 0;        // 0 = auto-ping off
  uint32_t nextPingMs = 0;

  // One blocking ping -> centimetres (us / 58), or -1 when nothing echoed in range.
  int32_t pingCm() {
    if (trigPin < 0 || echoPin < 0) return -1;
    digitalWrite(trigPin, LOW);  delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    unsigned long us = pulseIn((uint8_t)echoPin, HIGH, 25000);   // ~4.3 m ceiling
    return us > 0 ? (int32_t)(us / 58) : -1;
  }

  void handle(const uint8_t *payload, int length) override {
    if (length < 1) return;
    switch (payload[0]) {
      case 0x00:                    // configure: trig, echo
        if (length >= 3) {
          trigPin = payload[1] & 0x7F;
          echoPin = payload[2] & 0x7F;
          pinMode(trigPin, OUTPUT);
          pinMode(echoPin, INPUT);
        }
        break;
      case 0x01:                    // ping once -> R[dst]
        if (length >= 2) scheduler.regs[payload[1] & 0x1F] = pingCm();
        break;
      case 0x02:                    // auto-ping every period ms -> R[dst]; 0 stops
        if (length >= 4) {
          autoReg = payload[1] & 0x1F;
          autoPeriodMs = (uint32_t)(payload[2] & 0x7F) | ((uint32_t)(payload[3] & 0x7F) << 7);
          nextPingMs = millis();
        }
        break;
      case 0x03: {                  // one-shot: ping now, reply cm to the host (no register)
        uint8_t out[10]; int n = 0;
        out[n++] = START_SYSEX; out[n++] = MODULE_DATA; out[n++] = id(); out[n++] = 0x03;
        uint32_t v = (uint32_t)pingCm();          // cm (-1 = no echo) as 5x7-bit limbs
        for (int k = 0; k < 5; k++) { out[n++] = v & 0x7F; v >>= 7; }
        out[n++] = END_SYSEX;
        sendFrame(out, n);
        break;
      }
      default: break;
    }
  }

  void tick() override {
    if (autoPeriodMs == 0 || trigPin < 0) return;
    uint32_t now = millis();
    if ((int32_t)(now - nextPingMs) < 0) return;
    scheduler.regs[autoReg] = pingCm();
    nextPingMs = now + autoPeriodMs;
  }
};

/* The module's single instance (registered in ZRegistry.ino). */
static SonarModuleHandler sonarModule;
