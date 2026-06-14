/*
 * ESP32Firmata.ino  —  Firmata 2.x firmware for the original ESP32
 * ---------------------------------------------------------------------------
 * Self-contained (no external Firmata library) firmware that speaks the
 * Firmata protocol over one of two transports, selected at compile time:
 *
 *   USE_BLE 0 -> Wi-Fi / TCP + Bonjour (mDNS)   ← matches BonjourTransport.swift
 *   USE_BLE 1 -> BLE Nordic UART Service (NUS)   ← matches BLETransport.swift
 *
 * It is byte-for-byte compatible with the SwiftFirmataClient package and the
 * "swiftata" macOS app.  Tested wire formats (firmware report, capability,
 * analog-mapping, pin-state, digital/analog I/O, extended-analog, I2C) follow
 * the protocol exactly as parsed by FirmataParser.swift.
 *
 * ── Bonjour side ──
 *   Advertises  _firmata._tcp  on port 3030 with TXT records:
 *       ip   = <device IP>     (lets the client skip flaky mDNS A-record
 *       port = 3030             resolution — see BonjourTransport.swift)
 *
 * ── BLE side ──
 *   Service  6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX char  6E400002-...  WRITE / WRITE_NR   (host  -> device)
 *   TX char  6E400003-...  NOTIFY             (device -> host)
 *   The service UUID is placed in the advertisement so the client's
 *   service-filtered scan (scanForPeripherals(withServices:)) finds it.
 *
 * Target:  original ESP32 (ESP32-WROOM-32 / -WROVER) — "ESP32 Dev Module".
 * Core:    ESP32 Arduino core 3.x (recommended) or 2.x (fallback supported).
 *
 *   For the BLE build choose a partition scheme with a large app partition
 *   (Tools ▸ Partition Scheme ▸ "Minimal SPIFFS (1.9MB APP)" or "Huge APP").
 * ---------------------------------------------------------------------------
 */

// ===========================================================================
//  USER CONFIGURATION
// ===========================================================================

#define USE_BLE            0          // 0 = Wi-Fi/Bonjour, 1 = BLE

// --- Wi-Fi / Bonjour settings (used when USE_BLE == 0) ---------------------
#define WIFI_SSID          "YOUR_WIFI_SSID"
#define WIFI_PASS          "YOUR_WIFI_PASSWORD"
#define MDNS_HOSTNAME      "esp32-firmata"   // also the Bonjour instance name
#define FIRMATA_TCP_PORT   3030              // must match BonjourTransport default

// --- BLE settings (used when USE_BLE == 1) --------------------------------
#define BLE_DEVICE_NAME    "Firmata-ESP32"   // optional name filter in the app

// --- Firmware identity (shown in the app header) --------------------------
#define FIRMWARE_NAME      "Swiftata"
#define FIRMWARE_MAJOR     2
#define FIRMWARE_MINOR     8
#define PROTOCOL_MAJOR     2
#define PROTOCOL_MINOR     8

// ===========================================================================
//  Core-version helper (3.x vs 2.x differ for PWM / LEDC APIs)
// ===========================================================================
#if defined(ESP_ARDUINO_VERSION) && defined(ESP_ARDUINO_VERSION_VAL)
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    #define FIRMATA_CORE3 1
  #endif
#endif

// ===========================================================================
//  Transport-specific includes
// ===========================================================================
#if USE_BLE
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>
#else
  #include <WiFi.h>
  #include <ESPmDNS.h>
#endif
#include <Wire.h>

// ===========================================================================
//  Firmata protocol constants
// ===========================================================================
static const uint8_t ANALOG_MESSAGE        = 0xE0;
static const uint8_t DIGITAL_MESSAGE       = 0x90;
static const uint8_t REPORT_ANALOG         = 0xC0;
static const uint8_t REPORT_DIGITAL        = 0xD0;
static const uint8_t SET_PIN_MODE          = 0xF4;
static const uint8_t SET_DIGITAL_PIN_VALUE = 0xF5;
static const uint8_t REPORT_VERSION        = 0xF9;
static const uint8_t SYSTEM_RESET          = 0xFF;
static const uint8_t START_SYSEX           = 0xF0;
static const uint8_t END_SYSEX             = 0xF7;

static const uint8_t ANALOG_MAPPING_QUERY    = 0x69;
static const uint8_t ANALOG_MAPPING_RESPONSE = 0x6A;
static const uint8_t CAPABILITY_QUERY        = 0x6B;
static const uint8_t CAPABILITY_RESPONSE     = 0x6C;
static const uint8_t PIN_STATE_QUERY         = 0x6D;
static const uint8_t PIN_STATE_RESPONSE      = 0x6E;
static const uint8_t EXTENDED_ANALOG         = 0x6F;
static const uint8_t STRING_DATA             = 0x71;
static const uint8_t I2C_REQUEST             = 0x76;
static const uint8_t I2C_REPLY               = 0x77;
static const uint8_t I2C_CONFIG              = 0x78;
static const uint8_t REPORT_FIRMWARE         = 0x79;
static const uint8_t SAMPLING_INTERVAL       = 0x7A;

// Pin modes (Firmata §2)
static const uint8_t PIN_MODE_INPUT  = 0x00;
static const uint8_t PIN_MODE_OUTPUT = 0x01;
static const uint8_t PIN_MODE_ANALOG = 0x02;
static const uint8_t PIN_MODE_PWM    = 0x03;
static const uint8_t PIN_MODE_I2C    = 0x06;
static const uint8_t PIN_MODE_PULLUP = 0x0B;

// ===========================================================================
//  ESP32 pin model
// ===========================================================================
static const uint8_t TOTAL_PINS = 40;                 // GPIO 0..39
static const uint8_t NUM_PORTS  = (TOTAL_PINS + 7) / 8; // 5 ports

// ADC1 channels broken out on a typical ESP32 dev board -> analog channels A0..A5.
// (ADC2 pins are avoided: ADC2 cannot be read while Wi-Fi is active.)
static const int8_t  ANALOG_PINS[] = { 32, 33, 34, 35, 36, 39 };
static const uint8_t NUM_ANALOG    = sizeof(ANALOG_PINS) / sizeof(ANALOG_PINS[0]);

// Default I2C pins on the original ESP32.
static const uint8_t I2C_SDA_PIN = 21;
static const uint8_t I2C_SCL_PIN = 22;

// Full digital pins: support INPUT / INPUT_PULLUP / OUTPUT / PWM.
static bool isFullDigital(uint8_t pin) {
  switch (pin) {
    case 0:  case 2:  case 4:  case 5:
    case 12: case 13: case 14: case 15:
    case 16: case 17: case 18: case 19:
    case 21: case 22: case 23:
    case 25: case 26: case 27:
    case 32: case 33:
      return true;
    default:
      return false;
  }
}

// Input-only pins: INPUT (no output, no PWM, no internal pull-up).
static bool isInputOnly(uint8_t pin) {
  return pin == 34 || pin == 35 || pin == 36 || pin == 39;
}

static bool isUsable(uint8_t pin) {
  return isFullDigital(pin) || isInputOnly(pin);
}

// Analog channel for a pin, or -1 if the pin is not analog-capable.
static int analogChannelOfPin(uint8_t pin) {
  for (uint8_t i = 0; i < NUM_ANALOG; i++)
    if (ANALOG_PINS[i] == (int8_t)pin) return i;
  return -1;
}

static int pinOfAnalogChannel(uint8_t ch) {
  return (ch < NUM_ANALOG) ? ANALOG_PINS[ch] : -1;
}

// ===========================================================================
//  Runtime pin / reporting state
// ===========================================================================
static uint8_t  pinModes[TOTAL_PINS];   // current Firmata mode per pin
static int      pinValues[TOTAL_PINS];  // last written output / PWM value
static uint16_t analogReportMask = 0;   // bit c = report analog channel c
static bool     reportPort[NUM_PORTS];  // digital input reporting per port
static uint8_t  previousPort[NUM_PORTS];// last reported digital port mask

static uint16_t samplingInterval = 19;  // ms (Firmata default)
static const uint16_t MIN_SAMPLING = 10;
static unsigned long  lastSampleMs = 0;

// I2C
static uint16_t i2cReadDelayUs = 0;
struct ContinuousRead { uint16_t address; int reg; uint16_t count; bool active; };
static const uint8_t MAX_CONT_READS = 8;
static ContinuousRead contReads[MAX_CONT_READS];

// ===========================================================================
//  Input parser state (host -> device)
// ===========================================================================
static bool    parsingSysex = false;
static int     waitForData = 0;
static uint8_t executeMultiByteCommand = 0;
static uint8_t multiByteChannel = 0;
static uint8_t storedInputData[2];

static const int SYSEX_MAX = 256;
static uint8_t   sysexBuffer[SYSEX_MAX];
static int       sysexBytesRead = 0;

// Scratch buffer used to build outgoing frames.
static uint8_t   frameBuf[600];

// ===========================================================================
//  Forward declarations
// ===========================================================================
static void transportInit();
static void transportPoll();
static void sendFrame(const uint8_t *buf, size_t len);
static bool transportConnected();
static void onNewConnection();

static void processByte(uint8_t b);
static void processSysex(const uint8_t *buf, int len);
static void systemResetState();

// ===========================================================================
//  PWM helper (works on both core 2.x and 3.x)
// ===========================================================================
static void pwmWrite(uint8_t pin, int value) {
  if (value < 0)   value = 0;
  if (value > 255) value = 255;
#if FIRMATA_CORE3
  static bool attached[TOTAL_PINS] = { false };
  if (!attached[pin]) { ledcAttach(pin, 5000, 8); attached[pin] = true; }
  ledcWrite(pin, value);
#else
  static int  chForPin[TOTAL_PINS];
  static bool init = false;
  static int  nextCh = 0;
  if (!init) { for (int i = 0; i < TOTAL_PINS; i++) chForPin[i] = -1; init = true; }
  if (chForPin[pin] < 0 && nextCh < 16) {
    chForPin[pin] = nextCh++;
    ledcSetup(chForPin[pin], 5000, 8);
    ledcAttachPin(pin, chForPin[pin]);
  }
  if (chForPin[pin] >= 0) ledcWrite(chForPin[pin], value);
#endif
}

// ===========================================================================
//  Outgoing Firmata messages
// ===========================================================================
static void sendProtocolVersion() {
  uint8_t b[3] = { REPORT_VERSION, PROTOCOL_MAJOR, PROTOCOL_MINOR };
  sendFrame(b, 3);
}

static void sendFirmwareReport() {
  int n = 0;
  frameBuf[n++] = START_SYSEX;
  frameBuf[n++] = REPORT_FIRMWARE;
  frameBuf[n++] = FIRMWARE_MAJOR;
  frameBuf[n++] = FIRMWARE_MINOR;
  const char *name = FIRMWARE_NAME;
  for (const char *p = name; *p; ++p) {
    frameBuf[n++] = (uint8_t)(*p) & 0x7F;
    frameBuf[n++] = ((uint8_t)(*p) >> 7) & 0x7F;
  }
  frameBuf[n++] = END_SYSEX;
  sendFrame(frameBuf, n);
}

static void sendCapabilityResponse() {
  int n = 0;
  frameBuf[n++] = START_SYSEX;
  frameBuf[n++] = CAPABILITY_RESPONSE;
  for (uint8_t pin = 0; pin < TOTAL_PINS; pin++) {
    if (isFullDigital(pin)) {
      frameBuf[n++] = PIN_MODE_INPUT;  frameBuf[n++] = 1;
      frameBuf[n++] = PIN_MODE_PULLUP; frameBuf[n++] = 1;
      frameBuf[n++] = PIN_MODE_OUTPUT; frameBuf[n++] = 1;
      frameBuf[n++] = PIN_MODE_PWM;    frameBuf[n++] = 8;
      if (pin == I2C_SDA_PIN || pin == I2C_SCL_PIN) {
        frameBuf[n++] = PIN_MODE_I2C;  frameBuf[n++] = 1;
      }
      if (analogChannelOfPin(pin) >= 0) {
        frameBuf[n++] = PIN_MODE_ANALOG; frameBuf[n++] = 12;
      }
    } else if (isInputOnly(pin)) {
      frameBuf[n++] = PIN_MODE_INPUT; frameBuf[n++] = 1;
      if (analogChannelOfPin(pin) >= 0) {
        frameBuf[n++] = PIN_MODE_ANALOG; frameBuf[n++] = 12;
      }
    }
    frameBuf[n++] = 0x7F;  // end of this pin's capabilities
  }
  frameBuf[n++] = END_SYSEX;
  sendFrame(frameBuf, n);
}

static void sendAnalogMappingResponse() {
  int n = 0;
  frameBuf[n++] = START_SYSEX;
  frameBuf[n++] = ANALOG_MAPPING_RESPONSE;
  for (uint8_t pin = 0; pin < TOTAL_PINS; pin++) {
    int ch = analogChannelOfPin(pin);
    frameBuf[n++] = (ch >= 0) ? (uint8_t)ch : 0x7F;
  }
  frameBuf[n++] = END_SYSEX;
  sendFrame(frameBuf, n);
}

static void sendPinStateResponse(uint8_t pin) {
  if (pin >= TOTAL_PINS) return;
  int n = 0;
  frameBuf[n++] = START_SYSEX;
  frameBuf[n++] = PIN_STATE_RESPONSE;
  frameBuf[n++] = pin;
  frameBuf[n++] = pinModes[pin];
  // value as little-endian 7-bit groups (always at least one byte)
  uint32_t v = (uint32_t)pinValues[pin];
  frameBuf[n++] = v & 0x7F;
  v >>= 7;
  while (v) { frameBuf[n++] = v & 0x7F; v >>= 7; }
  frameBuf[n++] = END_SYSEX;
  sendFrame(frameBuf, n);
}

static void sendAnalogReport(uint8_t channel, int value) {
  uint8_t b[3];
  b[0] = ANALOG_MESSAGE | (channel & 0x0F);
  b[1] = value & 0x7F;
  b[2] = (value >> 7) & 0x7F;
  sendFrame(b, 3);
}

static void sendDigitalPort(uint8_t port, uint8_t mask) {
  uint8_t b[3];
  b[0] = DIGITAL_MESSAGE | (port & 0x0F);
  b[1] = mask & 0x7F;
  b[2] = (mask >> 7) & 0x01;
  sendFrame(b, 3);
}

static void sendI2CReply(uint16_t address, int reg, const uint8_t *data, int count) {
  int n = 0;
  frameBuf[n++] = START_SYSEX;
  frameBuf[n++] = I2C_REPLY;
  frameBuf[n++] = address & 0x7F;
  frameBuf[n++] = (address >> 7) & 0x7F;
  uint16_t r = (reg < 0) ? 0 : (uint16_t)reg;
  frameBuf[n++] = r & 0x7F;
  frameBuf[n++] = (r >> 7) & 0x7F;
  for (int i = 0; i < count; i++) {
    frameBuf[n++] = data[i] & 0x7F;
    frameBuf[n++] = (data[i] >> 7) & 0x7F;
  }
  frameBuf[n++] = END_SYSEX;
  sendFrame(frameBuf, n);
}

// ===========================================================================
//  Pin I/O handlers
// ===========================================================================
static void handleSetPinMode(uint8_t pin, uint8_t mode) {
  if (pin >= TOTAL_PINS) return;
  switch (mode) {
    case PIN_MODE_INPUT:
      if (isUsable(pin)) { pinMode(pin, INPUT); pinModes[pin] = mode; }
      break;
    case PIN_MODE_PULLUP:
      if (isFullDigital(pin)) { pinMode(pin, INPUT_PULLUP); pinModes[pin] = mode; pinValues[pin] = 1; }
      break;
    case PIN_MODE_OUTPUT:
      if (isFullDigital(pin)) { pinMode(pin, OUTPUT); pinModes[pin] = mode; }
      break;
    case PIN_MODE_ANALOG:
      if (analogChannelOfPin(pin) >= 0) { pinModes[pin] = mode; }
      break;
    case PIN_MODE_PWM:
      if (isFullDigital(pin)) { pinModes[pin] = mode; pwmWrite(pin, 0); pinValues[pin] = 0; }
      break;
    case PIN_MODE_I2C:
      pinModes[pin] = mode;
      break;
    default:
      break;
  }
}

static void handleSetDigitalPinValue(uint8_t pin, uint8_t value) {
  if (pin >= TOTAL_PINS) return;
  if (pinModes[pin] == PIN_MODE_OUTPUT) {
    digitalWrite(pin, value ? HIGH : LOW);
    pinValues[pin] = value ? 1 : 0;
  }
}

// Write a whole 8-pin port; only OUTPUT pins are affected.
static void handleDigitalMessage(uint8_t port, uint8_t lsb, uint8_t msb) {
  uint8_t portValue = (lsb & 0x7F) | ((msb & 0x01) << 7);
  for (int i = 0; i < 8; i++) {
    uint8_t pin = port * 8 + i;
    if (pin >= TOTAL_PINS) break;
    if (pinModes[pin] == PIN_MODE_OUTPUT) {
      uint8_t bit = (portValue >> i) & 0x01;
      digitalWrite(pin, bit ? HIGH : LOW);
      pinValues[pin] = bit;
    }
  }
}

// ANALOG_MESSAGE (host -> device) is a PWM write; the channel nibble is the pin.
static void handleAnalogMessage(uint8_t pin, uint8_t lsb, uint8_t msb) {
  if (pin >= TOTAL_PINS) return;
  int value = (lsb & 0x7F) | ((msb & 0x7F) << 7);
  if (pinModes[pin] == PIN_MODE_PWM) {
    pwmWrite(pin, value);
    pinValues[pin] = value;
  }
}

static void handleReportAnalog(uint8_t channel, uint8_t enable) {
  if (channel > 15) return;
  if (enable) analogReportMask |=  (1 << channel);
  else        analogReportMask &= ~(1 << channel);
}

static void handleReportDigital(uint8_t port, uint8_t enable) {
  if (port >= NUM_PORTS) return;
  reportPort[port] = (enable != 0);
  if (reportPort[port]) {
    // Send current state immediately so the host has an initial value.
    uint8_t mask = 0;
    for (int i = 0; i < 8; i++) {
      uint8_t pin = port * 8 + i;
      if (pin >= TOTAL_PINS) break;
      if (!isUsable(pin)) continue;
      uint8_t m = pinModes[pin];
      if (m == PIN_MODE_INPUT || m == PIN_MODE_PULLUP) {
        if (digitalRead(pin)) mask |= (1 << i);
      } else if (m == PIN_MODE_OUTPUT) {
        if (pinValues[pin]) mask |= (1 << i);
      }
    }
    previousPort[port] = mask;
    sendDigitalPort(port, mask);
  }
}

static void handleExtendedAnalog(const uint8_t *data, int len) {
  if (len < 1) return;
  uint8_t pin = data[0];
  if (pin >= TOTAL_PINS) return;
  int value = 0;
  for (int i = 1; i < len; i++) value |= (int)(data[i] & 0x7F) << (7 * (i - 1));
  if (pinModes[pin] == PIN_MODE_PWM) {
    pwmWrite(pin, value);
    pinValues[pin] = value;
  }
}

// ===========================================================================
//  I2C
// ===========================================================================
static void handleI2CConfig(const uint8_t *data, int len) {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (len >= 2) i2cReadDelayUs = (data[0] & 0x7F) | ((data[1] & 0x7F) << 7);
}

static void i2cDoRead(uint16_t address, int reg, uint16_t count) {
  if (reg >= 0) {
    Wire.beginTransmission(address);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
  }
  if (i2cReadDelayUs) delayMicroseconds(i2cReadDelayUs);

  static uint8_t buf[64];
  if (count > sizeof(buf)) count = sizeof(buf);
  int got = Wire.requestFrom((int)address, (int)count);
  int i = 0;
  while (Wire.available() && i < got && i < (int)count) buf[i++] = Wire.read();
  sendI2CReply(address, reg, buf, i);
}

static void addContinuousRead(uint16_t address, int reg, uint16_t count) {
  for (uint8_t i = 0; i < MAX_CONT_READS; i++) {
    if (contReads[i].active && contReads[i].address == address) {
      contReads[i].reg = reg; contReads[i].count = count; return;
    }
  }
  for (uint8_t i = 0; i < MAX_CONT_READS; i++) {
    if (!contReads[i].active) {
      contReads[i] = { address, reg, count, true }; return;
    }
  }
}

static void stopContinuousRead(uint16_t address) {
  for (uint8_t i = 0; i < MAX_CONT_READS; i++)
    if (contReads[i].active && contReads[i].address == address) contReads[i].active = false;
}

static void handleI2CRequest(const uint8_t *data, int len) {
  if (len < 2) return;
  uint16_t address = data[0] & 0x7F;
  uint8_t  control = data[1];
  uint8_t  mode    = (control >> 3) & 0x03;
  bool     tenbit  = control & 0x20;
  if (tenbit) address |= (uint16_t)(control & 0x07) << 7;

  const uint8_t *payload = data + 2;
  int plen = len - 2;

  switch (mode) {
    case 0: {  // WRITE
      Wire.beginTransmission(address);
      for (int i = 0; i + 1 < plen; i += 2) {
        uint8_t b = (payload[i] & 0x7F) | ((payload[i + 1] & 0x7F) << 7);
        Wire.write(b);
      }
      bool restart = control & 0x40;
      Wire.endTransmission(!restart);  // restart -> no STOP
      break;
    }
    case 1: {  // READ_ONCE
      int reg = -1; uint16_t count = 0;
      if (plen >= 4) {        // register + count both present
        reg   = (payload[0] & 0x7F) | ((payload[1] & 0x7F) << 7);
        count = (payload[2] & 0x7F) | ((payload[3] & 0x7F) << 7);
      } else if (plen >= 2) { // count only
        count = (payload[0] & 0x7F) | ((payload[1] & 0x7F) << 7);
      }
      i2cDoRead(address, reg, count);
      break;
    }
    case 2: {  // READ_CONTINUOUS
      int reg = -1; uint16_t count = 0;
      if (plen >= 4) {
        reg   = (payload[0] & 0x7F) | ((payload[1] & 0x7F) << 7);
        count = (payload[2] & 0x7F) | ((payload[3] & 0x7F) << 7);
      } else if (plen >= 2) {
        count = (payload[0] & 0x7F) | ((payload[1] & 0x7F) << 7);
      }
      addContinuousRead(address, reg, count);
      break;
    }
    case 3:    // STOP_READING
      stopContinuousRead(address);
      break;
  }
}

// ===========================================================================
//  SysEx dispatch
// ===========================================================================
static void handleString(const uint8_t *data, int len) {
  String s;
  for (int i = 0; i + 1 < len; i += 2) {
    uint16_t cp = (data[i] & 0x7F) | ((data[i + 1] & 0x7F) << 7);
    if (cp < 128) s += (char)cp;
  }
  Serial.print("[host] "); Serial.println(s);
}

static void processSysex(const uint8_t *buf, int len) {
  if (len < 1) return;
  uint8_t cmd = buf[0];
  const uint8_t *data = buf + 1;
  int dlen = len - 1;

  switch (cmd) {
    case REPORT_FIRMWARE:       sendFirmwareReport();        break;
    case CAPABILITY_QUERY:      sendCapabilityResponse();    break;
    case ANALOG_MAPPING_QUERY:  sendAnalogMappingResponse(); break;
    case PIN_STATE_QUERY:       if (dlen >= 1) sendPinStateResponse(data[0]); break;
    case EXTENDED_ANALOG:       handleExtendedAnalog(data, dlen); break;
    case SAMPLING_INTERVAL:
      if (dlen >= 2) {
        samplingInterval = (data[0] & 0x7F) | ((data[1] & 0x7F) << 7);
        if (samplingInterval < MIN_SAMPLING) samplingInterval = MIN_SAMPLING;
      }
      break;
    case STRING_DATA:           handleString(data, dlen);    break;
    case I2C_CONFIG:            handleI2CConfig(data, dlen);  break;
    case I2C_REQUEST:           handleI2CRequest(data, dlen); break;
    default:                    break;  // unknown SysEx ignored
  }
}

// ===========================================================================
//  Input byte processor (Firmata state machine)
// ===========================================================================
static void processByte(uint8_t inputData) {
  if (parsingSysex) {
    if (inputData == END_SYSEX) {
      parsingSysex = false;
      processSysex(sysexBuffer, sysexBytesRead);
    } else if (sysexBytesRead < SYSEX_MAX) {
      sysexBuffer[sysexBytesRead++] = inputData;
    }
    return;
  }

  if (waitForData > 0 && inputData < 0x80) {
    waitForData--;
    storedInputData[waitForData] = inputData;   // [1]=first byte, [0]=second byte
    if (waitForData == 0 && executeMultiByteCommand != 0) {
      switch (executeMultiByteCommand) {
        case ANALOG_MESSAGE:
          handleAnalogMessage(multiByteChannel, storedInputData[1], storedInputData[0]);
          break;
        case DIGITAL_MESSAGE:
          handleDigitalMessage(multiByteChannel, storedInputData[1], storedInputData[0]);
          break;
        case SET_PIN_MODE:
          handleSetPinMode(storedInputData[1], storedInputData[0]);
          break;
        case SET_DIGITAL_PIN_VALUE:
          handleSetDigitalPinValue(storedInputData[1], storedInputData[0]);
          break;
        case REPORT_ANALOG:
          handleReportAnalog(multiByteChannel, storedInputData[0]);
          break;
        case REPORT_DIGITAL:
          handleReportDigital(multiByteChannel, storedInputData[0]);
          break;
      }
      executeMultiByteCommand = 0;
    }
    return;
  }

  // New command byte.
  uint8_t command;
  if (inputData < 0xF0) {
    command          = inputData & 0xF0;
    multiByteChannel = inputData & 0x0F;
  } else {
    command = inputData;
  }

  switch (command) {
    case ANALOG_MESSAGE:
    case DIGITAL_MESSAGE:
    case SET_PIN_MODE:
    case SET_DIGITAL_PIN_VALUE:
      waitForData = 2; executeMultiByteCommand = command; break;
    case REPORT_ANALOG:
    case REPORT_DIGITAL:
      waitForData = 1; executeMultiByteCommand = command; break;
    case START_SYSEX:
      parsingSysex = true; sysexBytesRead = 0; break;
    case SYSTEM_RESET:
      systemResetState(); break;
    case REPORT_VERSION:
      sendProtocolVersion(); break;
    default:
      break;  // ignore unknown command bytes
  }
}

// ===========================================================================
//  Periodic sampling (device -> host)
// ===========================================================================
static void checkDigitalInputs() {
  for (uint8_t port = 0; port < NUM_PORTS; port++) {
    if (!reportPort[port]) continue;
    uint8_t mask = 0;
    for (int i = 0; i < 8; i++) {
      uint8_t pin = port * 8 + i;
      if (pin >= TOTAL_PINS) break;
      if (!isUsable(pin)) continue;
      uint8_t m = pinModes[pin];
      if (m == PIN_MODE_INPUT || m == PIN_MODE_PULLUP) {
        if (digitalRead(pin)) mask |= (1 << i);
      } else if (m == PIN_MODE_OUTPUT) {
        if (pinValues[pin]) mask |= (1 << i);
      }
    }
    if (mask != previousPort[port]) {
      previousPort[port] = mask;
      sendDigitalPort(port, mask);
    }
  }
}

static void sampleAnalogAndI2C() {
  for (uint8_t ch = 0; ch < NUM_ANALOG; ch++) {
    if (analogReportMask & (1 << ch)) {
      int pin = pinOfAnalogChannel(ch);
      if (pin >= 0) sendAnalogReport(ch, analogRead(pin));
    }
  }
  for (uint8_t i = 0; i < MAX_CONT_READS; i++) {
    if (contReads[i].active)
      i2cDoRead(contReads[i].address, contReads[i].reg, contReads[i].count);
  }
}

// ===========================================================================
//  Reset
// ===========================================================================
static void systemResetState() {
  parsingSysex = false;
  waitForData = 0;
  executeMultiByteCommand = 0;
  sysexBytesRead = 0;
  analogReportMask = 0;
  for (uint8_t i = 0; i < NUM_PORTS; i++) { reportPort[i] = false; previousPort[i] = 0; }
  for (uint8_t i = 0; i < MAX_CONT_READS; i++) contReads[i].active = false;
  for (uint8_t pin = 0; pin < TOTAL_PINS; pin++) {
    if (isUsable(pin)) pinMode(pin, INPUT);
    pinModes[pin]  = PIN_MODE_INPUT;
    pinValues[pin] = 0;
  }
  samplingInterval = 19;
}

// Called when a fresh client/central attaches.
static void onNewConnection() {
  systemResetState();
  sendProtocolVersion();
}

// ===========================================================================
//                            TRANSPORT — Wi-Fi / Bonjour
// ===========================================================================
#if !USE_BLE

static WiFiServer tcpServer(FIRMATA_TCP_PORT);
static WiFiClient tcpClient;

static void wifiConnect() {
  Serial.printf("Connecting to Wi-Fi \"%s\"", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(MDNS_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print('.');
    if (++tries > 75) {           // ~30 s, then reboot and retry
      Serial.println("\nWi-Fi connect failed, restarting...");
      ESP.restart();
    }
  }
  Serial.print("\nConnected. IP = ");
  Serial.println(WiFi.localIP());
}

static void startBonjour() {
  if (!MDNS.begin(MDNS_HOSTNAME)) {
    Serial.println("mDNS start failed");
    return;
  }
  MDNS.addService("firmata", "tcp", FIRMATA_TCP_PORT);   // -> _firmata._tcp
  static String ipStr   = WiFi.localIP().toString();
  static String portStr = String(FIRMATA_TCP_PORT);
  MDNS.addServiceTxt("firmata", "tcp", "ip",   ipStr.c_str());
  MDNS.addServiceTxt("firmata", "tcp", "port", portStr.c_str());
  Serial.printf("Bonjour: _firmata._tcp on %s:%d (instance \"%s\")\n",
                ipStr.c_str(), FIRMATA_TCP_PORT, MDNS_HOSTNAME);
}

static void transportInit() {
  wifiConnect();
  startBonjour();
  tcpServer.begin();
  tcpServer.setNoDelay(true);
}

static void transportPoll() {
  if (WiFi.status() != WL_CONNECTED) {       // recover from a dropped link
    wifiConnect();
    startBonjour();
  }
  // Accept a newly arriving client every poll. If one is already connected, the
  // newcomer replaces it — so a reconnect succeeds even when the previous session
  // wasn't closed cleanly (app force-quit, Mac sleep, half-open TCP, etc.).
  WiFiClient incoming = tcpServer.available();
  if (incoming) {
    if (tcpClient && tcpClient.connected()) {
      tcpClient.stop();
      Serial.println("Replacing previous TCP client");
    }
    tcpClient = incoming;
    tcpClient.setNoDelay(true);
    Serial.println("TCP client connected");
    onNewConnection();
  }
  // Drain whatever is available without blocking.
  for (int guard = 0; tcpClient && tcpClient.available() && guard < 1024; guard++) {
    processByte((uint8_t)tcpClient.read());
  }
}

static void sendFrame(const uint8_t *buf, size_t len) {
  if (tcpClient && tcpClient.connected()) tcpClient.write(buf, len);
}

static bool transportConnected() {
  return tcpClient && tcpClient.connected();
}

#endif // !USE_BLE

// ===========================================================================
//                            TRANSPORT — BLE (Nordic UART Service)
// ===========================================================================
#if USE_BLE

#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // host -> device
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // device -> host

static BLEServer         *bleServer = nullptr;
static BLECharacteristic *txChar    = nullptr;
static volatile bool      bleConnected = false;
static bool               bleWasConnected = false;
static volatile uint16_t  negotiatedMTU = 23;

// Lock-protected ring buffer for bytes arriving in the BLE task context.
static const int RXBUF_SIZE = 2048;
static volatile uint8_t   rxbuf[RXBUF_SIZE];
static volatile int       rxHead = 0, rxTail = 0;
static portMUX_TYPE       rxMux = portMUX_INITIALIZER_UNLOCKED;

static void rxEnqueue(const uint8_t *d, size_t n) {
  portENTER_CRITICAL(&rxMux);
  for (size_t i = 0; i < n; i++) {
    int nh = (rxHead + 1) % RXBUF_SIZE;
    if (nh != rxTail) { rxbuf[rxHead] = d[i]; rxHead = nh; }
  }
  portEXIT_CRITICAL(&rxMux);
}

static int rxDequeue() {
  int r = -1;
  portENTER_CRITICAL(&rxMux);
  if (rxTail != rxHead) { r = rxbuf[rxTail]; rxTail = (rxTail + 1) % RXBUF_SIZE; }
  portEXIT_CRITICAL(&rxMux);
  return r;
}

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    uint8_t *d = c->getData();
    size_t   n = c->getLength();
    if (d && n) rxEnqueue(d, n);
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { bleConnected = true; }
  void onDisconnect(BLEServer *s) override {
    bleConnected = false;
    negotiatedMTU = 23;
    s->getAdvertising()->start();   // allow a new connection
  }
  // Not marked 'override' so the sketch still compiles if a core lacks it.
  void onMtuChanged(BLEServer *, esp_ble_gatts_cb_param_t *param) {
    negotiatedMTU = param->mtu.mtu;
  }
};

static void transportInit() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(517);  // request a large MTU (host has final say)

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService *svc = bleServer->createService(NUS_SERVICE_UUID);

  BLECharacteristic *rxChar = svc->createCharacteristic(
      NUS_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxChar->setCallbacks(new RxCallbacks());

  txChar = svc->createCharacteristic(
      NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());

  svc->start();

  // Put the 128-bit service UUID in the advertisement (required for the
  // client's service-filtered scan); carry the name in the scan response so
  // both fit inside the 31-byte limit.
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  BLEAdvertisementData advData;
  advData.setFlags(0x06);
  advData.setCompleteServices(BLEUUID(NUS_SERVICE_UUID));
  BLEAdvertisementData scanResp;
  scanResp.setName(BLE_DEVICE_NAME);
  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanResp);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);

  BLEDevice::startAdvertising();
  Serial.printf("BLE advertising as \"%s\" (Nordic UART Service)\n", BLE_DEVICE_NAME);
}

static void transportPoll() {
  if (bleConnected && !bleWasConnected) {
    bleWasConnected = true;
    Serial.println("BLE central connected");
    onNewConnection();
  } else if (!bleConnected && bleWasConnected) {
    bleWasConnected = false;
    Serial.println("BLE central disconnected");
  }
  int b;
  int guard = 0;
  while ((b = rxDequeue()) >= 0 && guard++ < 4096) processByte((uint8_t)b);
}

static void sendFrame(const uint8_t *buf, size_t len) {
  if (!bleConnected || !txChar) return;
  size_t chunk = (negotiatedMTU > 23) ? (size_t)(negotiatedMTU - 3) : 20;
  size_t off = 0;
  while (off < len) {
    size_t n = (len - off < chunk) ? (len - off) : chunk;
    txChar->setValue((uint8_t *)(buf + off), n);
    txChar->notify();
    off += n;
    if (off < len) delay(6);  // pace notifications so none are dropped
  }
}

static bool transportConnected() { return bleConnected; }

#endif // USE_BLE

// ===========================================================================
//  Arduino entry points
// ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=== ESP32 Firmata (Swiftata) ===");

  analogReadResolution(12);
#if defined(ADC_11db)
  analogSetAttenuation(ADC_11db);
#endif

  systemResetState();
  transportInit();
}

void loop() {
  transportPoll();

  if (transportConnected()) {
    checkDigitalInputs();
    unsigned long now = millis();
    if (now - lastSampleMs >= samplingInterval) {
      lastSampleMs = now;
      sampleAnalogAndI2C();
    }
  }
}
