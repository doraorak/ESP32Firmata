// DHT module (id 0x03): DHT11/DHT22 temperature + humidity on one data pin.
// Mirrors ESP32FirmataSwift's DHTModuleHandler byte-for-byte.

/* DHT read primitive: DHT11/22 bit-bang (type 0=DHT11, 1=DHT22). 40 bits, bit value decided by
   the high-pulse width (~26-28 us = 0, ~70 us = 1). Returns 0 ok, -1 fail.
   Timing-critical: the line release and every sample use DIRECT GPIO registers
   (Arduino pinMode/digitalRead can burn >100 us in peripheral bookkeeping, which
   desynchronises the 26-70 us bit stream), and the whole ~4 ms window runs with
   interrupts masked. Direct-register path covers GPIO 0-31 (use those pins). */
#include "soc/gpio_struct.h"
static inline int dhtLvl(uint32_t bit) { return (GPIO.in & bit) ? 1 : 0; }
static int dhtWaitBit(uint32_t bit, int level, uint32_t timeout_us) {
  uint32_t t0 = micros();
  while (dhtLvl(bit) != level) {
    if ((uint32_t)(micros() - t0) > timeout_us) return -1;
  }
  return (int)(uint32_t)(micros() - t0);
}
static int dhtRead(uint8_t pinIn, int type, float *temp_c, float *hum_pct) {
  if (pinIn >= 32) return -1;                          // direct-reg path: GPIO 0-31
  uint8_t pin = pinIn;
  uint32_t bit = 1UL << pin;
  uint8_t data[5] = {0, 0, 0, 0, 0};

  // Configure pull-up once; drive the start signal via direct output-enable.
  pinMode(pin, INPUT_PULLUP);
  GPIO.out_w1tc = bit;                                 // latched LOW when output enables
  GPIO.enable_w1ts = bit;                              // drive low (start signal)
  delay(20);                                           // >=18 ms covers DHT11 and DHT22

  static portMUX_TYPE dhtMux = portMUX_INITIALIZER_UNLOCKED;
  bool fail = false;
  taskENTER_CRITICAL(&dhtMux);
  GPIO.enable_w1tc = bit;                              // release: pull-up snaps high (~ns)
  // Sensor response: low within 20-40 us, then ~80 us low / ~80 us high, then 40 bits.
  if (dhtWaitBit(bit, 0, 90)  < 0 ||
      dhtWaitBit(bit, 1, 120) < 0 ||
      dhtWaitBit(bit, 0, 120) < 0) {
    fail = true;
  } else {
    for (int i = 0; i < 40; i++) {
      if (dhtWaitBit(bit, 1, 80) < 0) { fail = true; break; }        // ~50 us low preamble
      int high = dhtWaitBit(bit, 0, 110);                            // bit = high-pulse width
      if (high < 0) { fail = true; break; }
      data[i / 8] <<= 1;
      if (high > 45) data[i / 8] |= 1;                               // ~26 us = 0, ~70 us = 1
    }
  }
  taskEXIT_CRITICAL(&dhtMux);
  if (fail) return -1;
  if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) return -1;

  if (type == 0) {                                     // DHT11: integral bytes
    *hum_pct = (float)data[0] + (float)data[1] * 0.1f;
    *temp_c  = (float)data[2] + (float)(data[3] & 0x7F) * 0.1f;
  } else {                                             // DHT22: 10ths, sign bit on temp
    *hum_pct = ((float)(((uint16_t)data[0] << 8) | data[1])) * 0.1f;
    float t  = ((float)(((uint16_t)(data[2] & 0x7F) << 8) | data[3])) * 0.1f;
    *temp_c  = (data[2] & 0x80) ? -t : t;
  }
  return 0;
}

/* ==== DHT module ==========================================================
   Results land in float registers (deg C / %RH) plus an ok-flag int register.
   Ops:
     0x00 <pin> <type> <tempF> <humF> <statusReg>   configure + start auto-reads
     0x01                                           force a read on the next tick
   The sensor allows one read per ~2 s; the module auto-reads on that cadence.
   A failed read sets R[status] = 0 and keeps the previous values. */
struct DHTModuleHandler : ModuleHandler {
  uint8_t id()    const override { return 0x03; }
  uint8_t major() const override { return 1; }
  uint8_t minor() const override { return 0; }
  const char *name() const override { return "dht"; }

  int      pin = -1;
  int      sensorType = 0;
  int      tempFReg = 0;
  int      humFReg = 0;
  int      statusReg = 0;
  uint32_t nextReadMs = 0;
  static const uint32_t READ_PERIOD_MS = 2000;   // datasheet minimum interval

  void handle(const uint8_t *payload, int length) override {
    if (length < 1) return;
    switch (payload[0]) {
      case 0x00:                    // configure: pin, type, tempFReg, humFReg, statusReg
        if (length >= 6) {
          pin        = payload[1] & 0x7F;
          sensorType = payload[2] & 0x7F;
          tempFReg   = payload[3] & (NUM_FLOAT_REGS - 1);
          humFReg    = payload[4] & (NUM_FLOAT_REGS - 1);
          statusReg  = payload[5] & 0x1F;
          nextReadMs = millis();    // first read on the next tick
        }
        break;
      case 0x01:                    // read now
        nextReadMs = millis();
        break;
      default: break;
    }
  }

  void tick() override {
    if (pin < 0) return;
    uint32_t now = millis();
    if ((int32_t)(now - nextReadMs) < 0) return;
    nextReadMs = now + READ_PERIOD_MS;
    float t = 0, h = 0;
    if (dhtRead((uint8_t)pin, sensorType, &t, &h) == 0) {
      scheduler.fregs[tempFReg] = t;
      scheduler.fregs[humFReg]  = h;
      scheduler.regs[statusReg] = 1;
    } else {
      scheduler.regs[statusReg] = 0;   // keep last good values in the float regs
    }
  }
};

/* The module's single instance (registered in ZRegistry.ino). */
static DHTModuleHandler dhtModule;
