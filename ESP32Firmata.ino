/*
 * ESP32Firmata.ino  —  Firmata 2.x firmware for the original ESP32
 * ---------------------------------------------------------------------------
 * Self-contained (no external Firmata library) firmware that speaks the
 * Firmata protocol over BOTH transports at once (set either ENABLE_ define to
 * 0 to force a single one):
 *
 *   ENABLE_WIFI -> Wi-Fi / TCP + Bonjour (mDNS)  ← matches BonjourTransport.swift
 *   ENABLE_BLE  -> BLE Nordic UART Service (NUS)  ← matches BLETransport.swift
 *
 * Only one client controls the board at a time. The policy is LATEST-WINS:
 * a new connection on either transport evicts the current holder (single Firmata
 * master — fully standard-compliant, no extra wire bytes). Scheduler tasks are
 * global and survive eviction/disconnect.
 *
 * It is byte-for-byte compatible with the SwiftFirmataClient package — the
 * tested wire formats (firmware report, capability, analog-mapping, pin-state,
 * digital/analog I/O, extended-analog, I2C) follow the protocol exactly as
 * parsed by FirmataParser.swift.
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

#define ENABLE_WIFI        1          // Wi-Fi/TCP + Bonjour   (set 0 to disable)
#define ENABLE_BLE         1          // BLE Nordic UART Svc   (set 0 to disable)

// --- Wi-Fi / Bonjour settings (used when ENABLE_WIFI == 1) -----------------
#define WIFI_SSID          "YOUR_WIFI_SSID"
#define WIFI_PASS          "YOUR_WIFI_PASSWORD"
#define MDNS_HOSTNAME      "esp32-firmata"   // also the Bonjour instance name
#define FIRMATA_TCP_PORT   3030              // must match BonjourTransport default

// --- BLE settings (used when ENABLE_BLE == 1) -----------------------------
#define BLE_DEVICE_NAME    "Firmata-ESP32"   // optional BLE name filter for clients

#if !ENABLE_WIFI && !ENABLE_BLE
#error "Enable at least one transport (ENABLE_WIFI and/or ENABLE_BLE)."
#endif

// --- Firmware identity (sent in the firmware-report message) --------------
#define FIRMWARE_NAME      "FirmataESP32"
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
#if ENABLE_WIFI
  #include <WiFi.h>
  #include <ESPmDNS.h>
  #include <HTTPClient.h>          // internet actions (scheduler extension)
  #include <WiFiClientSecure.h>    // HTTPS (TLS via ssl_client / mbedTLS)
  // IDF certificate bundle (browser-like root CA set) — validates HTTPS certs.
  // The arduino-esp32 core embeds it (CONFIG_MBEDTLS_CERTIFICATE_BUNDLE).
  extern const uint8_t fm_crt_bundle_start[] asm("_binary_x509_crt_bundle_start");
  extern const uint8_t fm_crt_bundle_end[]   asm("_binary_x509_crt_bundle_end");
#endif
#if ENABLE_BLE
  #include <NimBLEDevice.h>   // NimBLE uses far less heap than Bluedroid — leaves
                              // room for a TLS (HTTPS) handshake alongside Wi-Fi.
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
static const uint8_t SCHEDULER_DATA          = 0x7B;

// Scheduler sub-commands (first payload byte after SCHEDULER_DATA)
static const uint8_t SCHED_CREATE          = 0x00;
static const uint8_t SCHED_DELETE          = 0x01;
static const uint8_t SCHED_ADD             = 0x02;
static const uint8_t SCHED_DELAY           = 0x03;
static const uint8_t SCHED_SCHEDULE        = 0x04;
static const uint8_t SCHED_QUERY_ALL       = 0x05;
static const uint8_t SCHED_QUERY           = 0x06;
static const uint8_t SCHED_RESET           = 0x07;
static const uint8_t SCHED_ERROR_REPLY     = 0x08;
static const uint8_t SCHED_QUERY_ALL_REPLY = 0x09;
static const uint8_t SCHED_QUERY_REPLY     = 0x0A;
static const uint8_t SCHED_EXT_HTTP_REPLY  = 0x0B;   // device -> host: HTTP status + body

// --- Logic extension (see NONSTANDARD.md) -----------------------------------
// On-device registers + if/else so a stored task can make decisions by itself.
// Carried under the standard Scheduler's reserved extension command (0x7F,
// EXTENDED_SCHEDULER_COMMAND) so a base Firmata scheduler ignores it cleanly.
static const uint8_t SCHED_EXT_COMMAND      = 0x7F;  // EXTENDED_SCHEDULER_COMMAND
static const uint8_t SCHED_EXT_SET          = 0x10;  // R[d] = const
static const uint8_t SCHED_EXT_READ_DIGITAL = 0x11;  // R[d] = digitalRead(pin)
static const uint8_t SCHED_EXT_READ_ANALOG  = 0x12;  // R[d] = analogRead(channel)
static const uint8_t SCHED_EXT_IF           = 0x13;  // if !(a op b) skip N bytes
static const uint8_t SCHED_EXT_SKIP         = 0x14;  // unconditional skip N bytes
static const uint8_t SCHED_EXT_HTTP         = 0x15;  // make an HTTP(S) request over Wi-Fi
// Response-inspection ops (operate on the last HTTP response body):
static const uint8_t SCHED_EXT_JSON_NUM         = 0x16;  // R[dst]=json number at path x10^scale; R[found]
static const uint8_t SCHED_EXT_JSON_STR_EQ      = 0x17;  // R[dst]=(json string at path == s)?1:0
static const uint8_t SCHED_EXT_BODY_CONTAINS    = 0x18;  // R[dst]=body contains s?1:0
static const uint8_t SCHED_EXT_JSON_STR_CONTAINS = 0x19; // R[dst]=(json string at path contains s)?1:0

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

// ===========================================================================
//  Types used in function signatures — must be declared before the first
//  function so Arduino's auto-generated prototypes (inserted just above the
//  first function) can see them.
// ===========================================================================
static const int SYSEX_MAX = 512;   // fits an HTTP op's URL + body in one SysEx

// Firmata input-parser state. Two independent instances exist: one for the live
// host connection and one for replaying scheduler tasks — so a half-received
// live message can never be corrupted by task playback (and vice-versa).
struct ParserState {
  bool    parsingSysex = false;
  int     waitForData = 0;
  uint8_t executeMultiByteCommand = 0;
  uint8_t multiByteChannel = 0;
  uint8_t storedInputData[2] = { 0, 0 };
  uint8_t sysexBuffer[SYSEX_MAX];
  int     sysexBytesRead = 0;
};

// Scheduler task storage.
static const uint8_t  MAX_TASKS      = 8;
static const uint16_t MAX_TASK_BYTES = 512;

struct SchedTask {
  bool     used = false;
  uint8_t  id = 0;
  uint32_t time_ms = 0;   // absolute millis() when due; 0 = not scheduled
  uint16_t len = 0;       // bytes of task data stored
  uint16_t pos = 0;       // execution cursor
  uint8_t  data[MAX_TASK_BYTES];
};

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
static bool     pinConfigured[TOTAL_PINS]; // host explicitly set a digital mode
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

// Non-standard scheduler registers: 16 global Int32s shared across tasks.
// (The Scheduler and FirmataProtocol classes, and their instances, are defined
//  further down — after the send helpers they depend on.)
static const uint8_t NUM_SCHED_REGS = 16;

// Scratch buffer used to build outgoing frames.
static uint8_t   frameBuf[2048];
// Max response bytes retained for JSON/string inspection AND echoed to a host.
static const int HTTP_PARSE_MAX = 4096;

// ===========================================================================
//  Dual-transport master arbitration (latest-wins)
// ===========================================================================
// Exactly one transport "owns" the board at a time. A new connection on either
// transport calls claimMaster(), which evicts the other transport's holder.
enum ActiveTransport : uint8_t { TR_NONE = 0, TR_TCP, TR_BLE };
static ActiveTransport activeTransport = TR_NONE;

// ===========================================================================
//  Forward declarations
// ===========================================================================
static void transportInit();
static void transportPoll();
static void sendFrame(const uint8_t *buf, size_t len);
static bool transportConnected();
static void onNewConnection();
static void claimMaster(ActiveTransport who);
#if ENABLE_WIFI
static void tcpDrop();
static void tcpSend(const uint8_t *buf, size_t len);
#endif
#if ENABLE_BLE
static void bleDrop();
static void bleSend(const uint8_t *buf, size_t len);
#endif

static void systemResetState();

// The Firmata protocol handler and the Scheduler are classes (see below); they
// reference each other, so forward-declare both here.
class Scheduler;
class FirmataProtocol;

// PWM uses the Arduino core's analogWrite() (ESP32 core 3.x: LEDC-backed,
// 8-bit). Firmata duty is 0..255, so callers clamp before writing.

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
// I2C device helpers (global; shared by the live handler and periodic sampling).
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

// ===========================================================================
//  Live protocol handler
//  Parses an incoming Firmata byte stream (its own ParserState) and applies
//  each command. One instance drives the live transport; a second drives
//  scheduler task replay. Both act on shared device state and route scheduler
//  SysEx to the shared Scheduler.
// ===========================================================================
class FirmataProtocol {
 public:
  ParserState ps;
  Scheduler*  sched = nullptr;        // wired once at startup (see setup)

  void handleSetPinMode(uint8_t pin, uint8_t mode) {
    if (pin >= TOTAL_PINS) return;
    switch (mode) {
      case PIN_MODE_INPUT:
        if (isUsable(pin)) { pinMode(pin, INPUT); pinModes[pin] = mode; pinConfigured[pin] = true; }
        break;
      case PIN_MODE_PULLUP:
        if (isFullDigital(pin)) { pinMode(pin, INPUT_PULLUP); pinModes[pin] = mode; pinValues[pin] = 1; pinConfigured[pin] = true; }
        break;
      case PIN_MODE_OUTPUT:
        if (isFullDigital(pin)) { pinMode(pin, OUTPUT); pinModes[pin] = mode; pinConfigured[pin] = true; }
        break;
      case PIN_MODE_ANALOG:
        if (analogChannelOfPin(pin) >= 0) { pinModes[pin] = mode; }
        break;
      case PIN_MODE_PWM:
        if (isFullDigital(pin)) { pinModes[pin] = mode; analogWrite(pin, 0); pinValues[pin] = 0; }
        break;
      case PIN_MODE_I2C:
        pinModes[pin] = mode;
        break;
      default:
        break;
    }
  }

  void handleSetDigitalPinValue(uint8_t pin, uint8_t value) {
    if (pin >= TOTAL_PINS) return;
    if (pinModes[pin] == PIN_MODE_OUTPUT) {
      digitalWrite(pin, value ? HIGH : LOW);
      pinValues[pin] = value ? 1 : 0;
    }
  }

  // Write a whole 8-pin port; only OUTPUT pins are affected.
  void handleDigitalMessage(uint8_t port, uint8_t lsb, uint8_t msb) {
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
  void handleAnalogMessage(uint8_t pin, uint8_t lsb, uint8_t msb) {
    if (pin >= TOTAL_PINS) return;
    int value = (lsb & 0x7F) | ((msb & 0x7F) << 7);
    if (pinModes[pin] == PIN_MODE_PWM) {
      analogWrite(pin, value > 255 ? 255 : value);
      pinValues[pin] = value;
    }
  }

  void handleReportAnalog(uint8_t channel, uint8_t enable) {
    if (channel > 15) return;
    if (enable) analogReportMask |=  (1 << channel);
    else        analogReportMask &= ~(1 << channel);
  }

  void handleReportDigital(uint8_t port, uint8_t enable) {
    if (port >= NUM_PORTS) return;
    reportPort[port] = (enable != 0);
    if (reportPort[port]) {
      // Send current state immediately so the host has an initial value.
      uint8_t mask = 0;
      for (int i = 0; i < 8; i++) {
        uint8_t pin = port * 8 + i;
        if (pin >= TOTAL_PINS) break;
        if (!isUsable(pin) || !pinConfigured[pin]) continue;  // skip floating, unconfigured pins
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

  void handleExtendedAnalog(const uint8_t *data, int len) {
    if (len < 1) return;
    uint8_t pin = data[0];
    if (pin >= TOTAL_PINS) return;
    int value = 0;
    for (int i = 1; i < len; i++) value |= (int)(data[i] & 0x7F) << (7 * (i - 1));
    if (pinModes[pin] == PIN_MODE_PWM) {
      analogWrite(pin, value > 255 ? 255 : value);
      pinValues[pin] = value;
    }
  }

  void handleI2CConfig(const uint8_t *data, int len) {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    if (len >= 2) i2cReadDelayUs = (data[0] & 0x7F) | ((data[1] & 0x7F) << 7);
  }

  void handleI2CRequest(const uint8_t *data, int len) {
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

  void handleString(const uint8_t *data, int len) {
    String s;
    for (int i = 0; i + 1 < len; i += 2) {
      uint16_t cp = (data[i] & 0x7F) | ((data[i + 1] & 0x7F) << 7);
      if (cp < 128) s += (char)cp;
    }
    Serial.print("[host] "); Serial.println(s);
  }

  void processSysex(const uint8_t *buf, int len);   // defined after Scheduler

  // Firmata input byte state machine; mutates this handler's own ParserState.
  void process(uint8_t inputData) {
    ParserState &ps = this->ps;
    if (ps.parsingSysex) {
      if (inputData == END_SYSEX) {
        ps.parsingSysex = false;
        processSysex(ps.sysexBuffer, ps.sysexBytesRead);
      } else if (ps.sysexBytesRead < SYSEX_MAX) {
        ps.sysexBuffer[ps.sysexBytesRead++] = inputData;
      }
      return;
    }

    if (ps.waitForData > 0 && inputData < 0x80) {
      ps.waitForData--;
      ps.storedInputData[ps.waitForData] = inputData;   // [1]=first byte, [0]=second byte
      if (ps.waitForData == 0 && ps.executeMultiByteCommand != 0) {
        switch (ps.executeMultiByteCommand) {
          case ANALOG_MESSAGE:
            handleAnalogMessage(ps.multiByteChannel, ps.storedInputData[1], ps.storedInputData[0]);
            break;
          case DIGITAL_MESSAGE:
            handleDigitalMessage(ps.multiByteChannel, ps.storedInputData[1], ps.storedInputData[0]);
            break;
          case SET_PIN_MODE:
            handleSetPinMode(ps.storedInputData[1], ps.storedInputData[0]);
            break;
          case SET_DIGITAL_PIN_VALUE:
            handleSetDigitalPinValue(ps.storedInputData[1], ps.storedInputData[0]);
            break;
          case REPORT_ANALOG:
            handleReportAnalog(ps.multiByteChannel, ps.storedInputData[0]);
            break;
          case REPORT_DIGITAL:
            handleReportDigital(ps.multiByteChannel, ps.storedInputData[0]);
            break;
        }
        ps.executeMultiByteCommand = 0;
      }
      return;
    }

    // New command byte.
    uint8_t command;
    if (inputData < 0xF0) {
      command             = inputData & 0xF0;
      ps.multiByteChannel = inputData & 0x0F;
    } else {
      command = inputData;
    }

    switch (command) {
      case ANALOG_MESSAGE:
      case DIGITAL_MESSAGE:
      case SET_PIN_MODE:
      case SET_DIGITAL_PIN_VALUE:
        ps.waitForData = 2; ps.executeMultiByteCommand = command; break;
      case REPORT_ANALOG:
      case REPORT_DIGITAL:
        ps.waitForData = 1; ps.executeMultiByteCommand = command; break;
      case START_SYSEX:
        ps.parsingSysex = true; ps.sysexBytesRead = 0; break;
      case SYSTEM_RESET:
        systemResetState(); break;
      case REPORT_VERSION:
        sendProtocolVersion(); break;
      default:
        break;  // ignore unknown command bytes
    }
  }
};

// ===========================================================================
//  Firmata Scheduler (SysEx 0x7B) — class
//
//  Stores tasks (recorded Firmata messages + delays) and replays them — through
//  a dedicated FirmataProtocol instance (`replay`) — even with no host
//  connected. Also owns the non-standard logic extension: 16 Int32 registers,
//  if/else, and internet actions. Encoder7Bit helpers live here as methods.
//  Wire format follows the official Firmata Scheduler protocol.
// ===========================================================================
class Scheduler {
 public:
  SchedTask        tasks[MAX_TASKS];
  SchedTask       *running = nullptr;
  int32_t          regs[NUM_SCHED_REGS];
  FirmataProtocol *replay = nullptr;     // wired once at startup (see setup)
  String           httpRespBody;         // last HTTP response (until next request)

  SchedTask *find(uint8_t id) {
    for (uint8_t i = 0; i < MAX_TASKS; i++)
      if (tasks[i].used && tasks[i].id == id) return &tasks[i];
    return nullptr;
  }

  // Encoder7Bit decode: unpack `outBytes` 8-bit bytes from 7-bit `in`.
  void sched7BitDecode(int outBytes, const uint8_t *in, uint8_t *out) {
    for (int i = 0; i < outBytes; i++) {
      int j = i << 3;
      int pos = j / 7;
      uint8_t shift = j % 7;
      out[i] = (in[pos] >> shift) | ((in[pos + 1] << (7 - shift)) & 0xFF);
    }
  }
  int sched7BitOutBytes(int encodedLen) { return (encodedLen * 7) >> 3; }

  // Decode a 32-bit little-endian value from 5 Encoder7Bit-packed bytes.
  uint32_t sched7BitTime(const uint8_t *enc5) {
    uint8_t b[4];
    sched7BitDecode(4, enc5, b);
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
  }

  // Encoder7Bit encode one byte into frameBuf, carrying state in shift/prev.
  void sched7BitPut(int &n, uint8_t &shift, uint8_t &prev, uint8_t d) {
    if (shift == 0) { frameBuf[n++] = d & 0x7F; shift = 1; prev = d >> 7; }
    else {
      frameBuf[n++] = (uint8_t)((((uint16_t)d << shift) & 0x7F) | prev);
      if (shift == 6) { frameBuf[n++] = d >> 1; shift = 0; }
      else { shift++; prev = d >> (8 - shift); }
    }
  }

  void reset() {
    for (uint8_t i = 0; i < MAX_TASKS; i++) tasks[i].used = false;
    for (uint8_t i = 0; i < NUM_SCHED_REGS; i++) regs[i] = 0;
    running = nullptr;
  }

  void sendError(uint8_t id) {
    uint8_t b[5] = { START_SYSEX, SCHEDULER_DATA, SCHED_ERROR_REPLY, id, END_SYSEX };
    sendFrame(b, 5);
  }

  void create(uint8_t id, uint16_t len) {
    if (find(id) || len > MAX_TASK_BYTES) { sendError(id); return; }
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
      if (!tasks[i].used) {
        tasks[i].used = true; tasks[i].id = id;
        tasks[i].time_ms = 0; tasks[i].len = len; tasks[i].pos = 0;
        return;
      }
    }
    sendError(id);  // no free slot
  }

  void deleteTask(uint8_t id) {
    SchedTask *t = find(id);
    if (t) { if (running == t) running = nullptr; t->used = false; }
  }

  void add(uint8_t id, const uint8_t *data, int n) {
    SchedTask *t = find(id);
    if (!t) { sendError(id); return; }
    if ((int)t->pos + n > (int)t->len) return;  // would overflow reserved length
    for (int i = 0; i < n; i++) t->data[t->pos++] = data[i];
  }

  void schedule(uint8_t id, uint32_t delayMs) {
    SchedTask *t = find(id);
    if (!t) { sendError(id); return; }
    t->pos = 0;
    t->time_ms = millis() + delayMs;
    if (t->time_ms == 0) t->time_ms = 1;        // reserve 0 for "not scheduled"
  }

  // A DELAY_TASK encountered while a task is executing.
  void delayRunning(uint32_t delayMs) {
    if (!running) return;
    uint32_t now = millis();
    running->time_ms += delayMs;
    if ((int32_t)(running->time_ms - now) < 0) running->time_ms = now;
    if (running->time_ms == 0) running->time_ms = 1;
  }

  void queryAll() {
    int n = 0;
    frameBuf[n++] = START_SYSEX; frameBuf[n++] = SCHEDULER_DATA; frameBuf[n++] = SCHED_QUERY_ALL_REPLY;
    for (uint8_t i = 0; i < MAX_TASKS; i++)
      if (tasks[i].used) frameBuf[n++] = tasks[i].id;
    frameBuf[n++] = END_SYSEX;
    sendFrame(frameBuf, n);
  }

  void queryTask(uint8_t id) {
    SchedTask *t = find(id);
    if (!t) { sendError(id); return; }
    int n = 0;
    frameBuf[n++] = START_SYSEX; frameBuf[n++] = SCHEDULER_DATA; frameBuf[n++] = SCHED_QUERY_REPLY;
    frameBuf[n++] = id;
    // payload = time_ms(4 LE) + len(2 LE) + pos(2 LE) + data[len], Encoder7Bit-packed
    uint8_t header[8] = {
      (uint8_t)(t->time_ms), (uint8_t)(t->time_ms >> 8),
      (uint8_t)(t->time_ms >> 16), (uint8_t)(t->time_ms >> 24),
      (uint8_t)(t->len), (uint8_t)(t->len >> 8),
      (uint8_t)(t->pos), (uint8_t)(t->pos >> 8)
    };
    uint8_t shift = 0, prev = 0;
    for (int i = 0; i < 8; i++)             sched7BitPut(n, shift, prev, header[i]);
    for (uint16_t i = 0; i < t->len; i++)   sched7BitPut(n, shift, prev, t->data[i]);
    if (shift > 0) frameBuf[n++] = prev;
    frameBuf[n++] = END_SYSEX;
    sendFrame(frameBuf, n);
  }

  // ---- NON-STANDARD logic extension (see NONSTANDARD.md) ----
  bool compare(uint8_t op, int32_t a, int32_t b) {
    switch (op) {
      case 0: return a == b;
      case 1: return a != b;
      case 2: return a <  b;
      case 3: return a >  b;
      case 4: return a <= b;
      case 5: return a >= b;
      default: return false;
    }
  }

  // Read one IF operand starting at payload[i]; advances i. Type byte: 0=register
  // (1 byte index), 1=constant (5 Encoder7Bit bytes of an Int32).
  int32_t readOperand(const uint8_t *payload, int plen, int &i) {
    if (i >= plen) return 0;
    uint8_t type = payload[i++];
    if (type == 0) {                                   // register
      if (i >= plen) return 0;
      return regs[payload[i++] & 0x0F];
    } else {                                           // constant
      if (i + 5 > plen) { i = plen; return 0; }
      int32_t v = (int32_t)sched7BitTime(payload + i);
      i += 5;
      return v;
    }
  }

  // Advance the running task's cursor by `amount` bytes (forward-only; clamped).
  void skip(uint16_t amount) {
    if (!running) return;
    uint32_t p = (uint32_t)running->pos + amount;
    running->pos = (p > running->len) ? running->len : (uint16_t)p;
  }

  // Send the last HTTP result (status + body) to the connected host.
  void sendHttpReply(int status) {
    int rlen = httpRespBody.length();
    if (rlen > HTTP_PARSE_MAX) rlen = HTTP_PARSE_MAX;
    // Dedicated buffer (frameBuf is only 2 KB): header + body as 14-bit pairs.
    uint8_t *out = (uint8_t *)malloc((size_t)rlen * 2 + 8);
    if (!out) return;
    int n = 0;
    out[n++] = START_SYSEX;
    out[n++] = SCHEDULER_DATA;
    out[n++] = SCHED_EXT_HTTP_REPLY;
    out[n++] = status & 0x7F;
    out[n++] = (status >> 7) & 0x7F;
    const char *body = httpRespBody.c_str();
    for (int k = 0; k < rlen; k++) {           // 14-bit LSB/MSB pairs (like STRING_DATA)
      out[n++] = (uint8_t)body[k] & 0x7F;
      out[n++] = ((uint8_t)body[k] >> 7) & 0x7F;
    }
    out[n++] = END_SYSEX;
    sendFrame(out, n);
    free(out);
  }

  // ====== minimal JSON walker over a byte buffer (mirrors the Swift firmware) ==
  int jsonSkipWs(const uint8_t *b, int blen, int i) {
    while (i < blen && (b[i] == ' ' || b[i] == '\t' || b[i] == '\n' || b[i] == '\r')) i++;
    return i;
  }
  int jsonSkipString(const uint8_t *b, int blen, int i) {   // i at opening quote
    int j = i + 1;
    while (j < blen) {
      if (b[j] == '\\') { j += 2; continue; }                // backslash escape
      if (b[j] == '"') return j + 1;
      j++;
    }
    return j;
  }
  int jsonSkipValue(const uint8_t *b, int blen, int i) {
    int j = jsonSkipWs(b, blen, i);
    if (j >= blen) return j;
    uint8_t c = b[j];
    if (c == '"') return jsonSkipString(b, blen, j);
    if (c == '{' || c == '[') {
      uint8_t open = c, close = (c == '{') ? '}' : ']';
      int k = j + 1, depth = 1;
      while (k < blen && depth > 0) {
        uint8_t d = b[k];
        if (d == '"') { k = jsonSkipString(b, blen, k); continue; }
        if (d == open) depth++; else if (d == close) depth--;
        k++;
      }
      return k;
    }
    int k = j;                                               // number/true/false/null
    while (k < blen) {
      uint8_t d = b[k];
      if (d == ',' || d == '}' || d == ']' || d == ' ' || d == '\t' || d == '\n' || d == '\r') break;
      k++;
    }
    return k;
  }
  int jsonObjectMember(const uint8_t *b, int blen, int pos, const uint8_t *key, int keylen) {
    int j = jsonSkipWs(b, blen, pos);
    if (j >= blen || b[j] != '{') return -1;
    j++;
    while (true) {
      j = jsonSkipWs(b, blen, j);
      if (j >= blen || b[j] == '}') return -1;
      if (b[j] != '"') return -1;
      int ks = j + 1;
      int after = jsonSkipString(b, blen, j); int ke = after - 1;
      bool match = (ke - ks) == keylen;
      if (match) { for (int t = 0; t < keylen; t++) if (b[ks + t] != key[t]) { match = false; break; } }
      j = jsonSkipWs(b, blen, after);
      if (j >= blen || b[j] != ':') return -1;
      j = jsonSkipWs(b, blen, j + 1);
      if (match) return j;
      j = jsonSkipWs(b, blen, jsonSkipValue(b, blen, j));
      if (j < blen && b[j] == ',') { j++; continue; }
      return -1;
    }
  }
  int jsonArrayElement(const uint8_t *b, int blen, int pos, int idx) {
    int j = jsonSkipWs(b, blen, pos);
    if (j >= blen || b[j] != '[') return -1;
    j++;
    int cur = 0;
    while (true) {
      j = jsonSkipWs(b, blen, j);
      if (j >= blen || b[j] == ']') return -1;
      if (cur == idx) return j;
      j = jsonSkipWs(b, blen, jsonSkipValue(b, blen, j));
      if (j < blen && b[j] == ',') { j++; cur++; continue; }
      return -1;
    }
  }
  // Resolve a dotted/indexed path (e.g. "a.b[0].c") to the value's byte span.
  bool jsonValueSpan(const uint8_t *b, int blen, const uint8_t *path, int pathlen,
                     int &outStart, int &outEnd) {
    if (blen <= 0) return false;
    int pos = jsonSkipWs(b, blen, 0);
    int pi = 0;
    while (true) {
      if (pi >= pathlen) { outStart = pos; outEnd = jsonSkipValue(b, blen, pos); return true; }
      if (path[pi] == '.') { pi++; continue; }
      if (path[pi] == '[') {                                 // array index
        pi++; int idx = 0;
        while (pi < pathlen && path[pi] >= '0' && path[pi] <= '9') { idx = idx * 10 + (path[pi] - '0'); pi++; }
        if (pi < pathlen && path[pi] == ']') pi++;
        int p2 = jsonArrayElement(b, blen, pos, idx);
        if (p2 < 0) return false;
        pos = p2;
      } else {                                               // object key
        int ks = pi;
        while (pi < pathlen && path[pi] != '.' && path[pi] != '[') pi++;
        int p2 = jsonObjectMember(b, blen, pos, path + ks, pi - ks);
        if (p2 < 0) return false;
        pos = p2;
      }
    }
  }
  // Parse a JSON number at b[i0] into value × 10^scale (truncated). false if not a number.
  bool parseScaledNumber(const uint8_t *b, int blen, int i0, int scale, int32_t &out) {
    int i = jsonSkipWs(b, blen, i0);
    if (i >= blen) return false;
    bool neg = false;
    if (b[i] == '-') { neg = true; i++; }
    long long intPart = 0; bool anyInt = false;
    while (i < blen && b[i] >= '0' && b[i] <= '9') { intPart = intPart * 10 + (b[i] - '0'); anyInt = true; i++; }
    if (!anyInt) return false;
    uint8_t frac[16]; int fracN = 0;
    if (i < blen && b[i] == '.') {
      i++;
      while (i < blen && b[i] >= '0' && b[i] <= '9') { if (fracN < 16) frac[fracN] = b[i] - '0'; fracN++; i++; }
    }
    long long v = intPart;
    for (int s = 0; s < scale; s++) v = v * 10 + (s < fracN && s < 16 ? frac[s] : 0);
    out = (int32_t)(neg ? -v : v);
    return true;
  }
  bool bytesEqual(const uint8_t *b, int s, int e, const uint8_t *needle, int nlen) {
    if (e - s != nlen) return false;
    for (int t = 0; t < nlen; t++) if (b[s + t] != needle[t]) return false;
    return true;
  }
  bool bytesContain(const uint8_t *b, int s, int e, const uint8_t *needle, int nlen) {
    if (nlen == 0) return true;
    if (e - s < nlen) return false;
    for (int i = s; i <= e - nlen; i++) {
      bool m = true;
      for (int t = 0; t < nlen; t++) if (b[i + t] != needle[t]) { m = false; break; }
      if (m) return true;
    }
    return false;
  }

  // 0x16: R[dst] = number at JSON <path> × 10^scale (truncated); R[found] = 0/1.
  void doJsonNum(const uint8_t *p, int plen) {
    if (plen < 6) return;
    int dst = p[1] & 0x0F, foundReg = p[2] & 0x0F, scale = p[3];
    int pathLen = p[4] | (p[5] << 7);
    if (6 + pathLen > plen) return;
    const uint8_t *path = p + 6;
    const uint8_t *b = (const uint8_t *)httpRespBody.c_str();
    int blen = httpRespBody.length();
    int s, e; int32_t v;
    if (jsonValueSpan(b, blen, path, pathLen, s, e) && parseScaledNumber(b, blen, s, scale, v)) {
      regs[dst] = v; regs[foundReg] = 1;
    } else {
      regs[dst] = 0; regs[foundReg] = 0;
    }
  }
  // 0x17 (eq) / 0x19 (contains): compare JSON string at <path> with <str>.
  void doJsonStr(const uint8_t *p, int plen, bool contains) {
    if (plen < 4) return;
    int dst = p[1] & 0x0F;
    int pathLen = p[2] | (p[3] << 7);
    int i = 4 + pathLen;
    if (i + 2 > plen) return;
    const uint8_t *path = p + 4;
    int strLen = p[i] | (p[i + 1] << 7); i += 2;
    if (i + strLen > plen) return;
    const uint8_t *needle = p + i;
    const uint8_t *b = (const uint8_t *)httpRespBody.c_str();
    int blen = httpRespBody.length();
    int32_t result = 0;
    int s, e;
    if (jsonValueSpan(b, blen, path, pathLen, s, e) && s < e && b[s] == '"') {
      int cs = s + 1, ce = e - 1;                            // string content (escapes raw)
      if (ce >= cs) {
        result = (contains ? bytesContain(b, cs, ce, needle, strLen)
                           : bytesEqual(b, cs, ce, needle, strLen)) ? 1 : 0;
      }
    }
    regs[dst] = result;
  }
  // 0x18: R[dst] = whole body contains <str> ? 1 : 0.
  void doBodyContains(const uint8_t *p, int plen) {
    if (plen < 4) return;
    int dst = p[1] & 0x0F;
    int strLen = p[2] | (p[3] << 7);
    if (4 + strLen > plen) return;
    const uint8_t *needle = p + 4;
    const uint8_t *b = (const uint8_t *)httpRespBody.c_str();
    int blen = httpRespBody.length();
    regs[dst] = bytesContain(b, 0, blen, needle, strLen) ? 1 : 0;
  }

  // Internet action: a task (or live host) makes an HTTP(S) request over Wi-Fi.
  // ext payload: 0x15 method statusReg urlLo urlHi url[] bodyLo bodyHi body[].
  // method 0=GET 1=POST. Stores HTTP status in R[statusReg] (0 = Wi-Fi down/error)
  // and retains the full response body for the inspection ops (JSON_NUM/*_STR_*/
  // BODY_*). If a host is connected, status + body return as SCHED_EXT_HTTP_REPLY.
  // https:// is validated against the IDF cert bundle (see README).
  void doHttp(const uint8_t *p, int plen) {
    if (plen < 5) return;
    uint8_t method   = p[1];
    int     statusReg = p[2] & 0x0F;
    int     urlLen    = p[3] | (p[4] << 7);
    int i = 5;
    if (urlLen <= 0 || i + urlLen > plen) return;
    String url; url.reserve(urlLen + 1);
    for (int k = 0; k < urlLen; k++) url += (char)p[i + k];
    i += urlLen;
    int bodyLen = 0;
    if (i + 2 <= plen) { bodyLen = p[i] | (p[i + 1] << 7); i += 2; }
    if (bodyLen < 0 || i + bodyLen > plen) bodyLen = 0;
    String body; body.reserve(bodyLen + 1);
    for (int k = 0; k < bodyLen; k++) body += (char)p[i + k];

    httpRespBody = "";
    int status = 0;
#if ENABLE_WIFI
    if (WiFi.status() == WL_CONNECTED) {
      bool https = url.startsWith("https");
      HTTPClient http;
      http.setConnectTimeout(8000);
      http.setTimeout(8000);
      http.setReuse(false);
      http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
      WiFiClientSecure tls;
      WiFiClient plain;
      bool ok;
      if (https) {
        tls.setCACertBundle(fm_crt_bundle_start,
                            (size_t)(fm_crt_bundle_end - fm_crt_bundle_start));  // validate certs
        ok = http.begin(tls, url);
      } else {
        ok = http.begin(plain, url);
      }
      if (ok) {
        if (method == 1) {
          http.addHeader("Content-Type", "application/json");
          status = http.POST((uint8_t *)body.c_str(), body.length());
        } else {
          status = http.GET();
        }
        if (status > 0) httpRespBody = http.getString();
        http.end();
      }
    }
#endif
    regs[statusReg] = status;
    if (transportConnected()) sendHttpReply(status);
  }

  void handleExt(const uint8_t *payload, int plen) {
    switch (payload[0]) {
      case SCHED_EXT_SET:               // 0x10 reg <const:5>
        if (plen == 7) regs[payload[1] & 0x0F] = (int32_t)sched7BitTime(payload + 2);
        break;
      case SCHED_EXT_READ_DIGITAL:      // 0x11 reg pin
        if (plen == 3) regs[payload[1] & 0x0F] = digitalRead(payload[2]) ? 1 : 0;
        break;
      case SCHED_EXT_READ_ANALOG: {     // 0x12 reg channel
        if (plen == 3) {
          int pin = pinOfAnalogChannel(payload[2]);
          regs[payload[1] & 0x0F] = (pin >= 0) ? analogRead(pin) : 0;
        }
        break;
      }
      case SCHED_EXT_IF: {              // 0x13 op <operandA> <operandB> skipLo skipHi
        int i = 1;
        uint8_t op = payload[i++];
        int32_t a = readOperand(payload, plen, i);
        int32_t b = readOperand(payload, plen, i);
        if (i + 2 > plen) break;
        uint16_t amount = (uint16_t)(payload[i] | (payload[i + 1] << 7));
        if (!compare(op, a, b)) skip(amount);   // condition false -> skip block
        break;
      }
      case SCHED_EXT_SKIP:             // 0x14 skipLo skipHi  (unconditional; for else)
        if (plen == 3) skip((uint16_t)(payload[1] | (payload[2] << 7)));
        break;
      case SCHED_EXT_HTTP:            // 0x15 internet request (see NONSTANDARD.md)
        doHttp(payload, plen);
        break;
      case SCHED_EXT_JSON_NUM:           // 0x16 dst found scale pathLo pathHi path…
        doJsonNum(payload, plen);
        break;
      case SCHED_EXT_JSON_STR_EQ:        // 0x17 dst pathLo pathHi path… strLo strHi str…
        doJsonStr(payload, plen, false);
        break;
      case SCHED_EXT_BODY_CONTAINS:      // 0x18 dst strLo strHi str…
        doBodyContains(payload, plen);
        break;
      case SCHED_EXT_JSON_STR_CONTAINS:  // 0x19 dst pathLo pathHi path… strLo strHi str…
        doJsonStr(payload, plen, true);
        break;
    }
  }

  void handleSysex(const uint8_t *payload, int plen) {
    if (plen < 1) return;
    switch (payload[0]) {
      case SCHED_CREATE:
        if (plen == 4) create(payload[1], (uint16_t)(payload[2] | (payload[3] << 7)));
        break;
      case SCHED_DELETE:
        if (plen == 2) deleteTask(payload[1]);
        break;
      case SCHED_ADD:
        if (plen > 2) {
          int outLen = sched7BitOutBytes(plen - 2);
          static uint8_t dec[MAX_TASK_BYTES];
          if (outLen > (int)sizeof(dec)) outLen = sizeof(dec);
          sched7BitDecode(outLen, payload + 2, dec);
          add(payload[1], dec, outLen);
        }
        break;
      case SCHED_DELAY:
        if (plen == 6) delayRunning(sched7BitTime(payload + 1));
        break;
      case SCHED_SCHEDULE:
        if (plen == 7) schedule(payload[1], sched7BitTime(payload + 2));
        break;
      case SCHED_EXT_COMMAND:            // 0x7F: our logic ops live under the
        if (plen >= 2) handleExt(payload + 1, plen - 1);  // reserved extension cmd
        break;
      case SCHED_QUERY_ALL: queryAll(); break;
      case SCHED_QUERY:     if (plen == 2) queryTask(payload[1]); break;
      case SCHED_RESET:     reset(); break;
      default: break;
    }
  }

  // Replay a task's bytes through the replay handler until a delay reschedules it
  // or it finishes. Returns true if the task should be kept (rescheduled/looping).
  bool execute(SchedTask *t) {
    uint32_t start = t->time_ms;
    running = t;
    replay->ps = ParserState();                 // each run resumes at a message boundary
    while (t->pos < t->len) {
      replay->process(t->data[t->pos++]);
      if (t->time_ms != start) {                // a DELAY_TASK fired
        if (t->pos >= t->len) t->pos = 0;       // trailing delay -> loop from the start
        running = nullptr;
        return true;
      }
    }
    running = nullptr;
    return false;                               // ran to end with no trailing delay -> one-shot
  }

  void tick() {
    uint32_t now = millis();
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
      SchedTask *t = &tasks[i];
      if (t->used && t->time_ms != 0 && (int32_t)(now - t->time_ms) >= 0) {
        if (!execute(t)) t->used = false;  // one-shot finished -> free the slot
      }
    }
  }
};

// Now that Scheduler is complete, define the handler's scheduler-dispatching SysEx.
void FirmataProtocol::processSysex(const uint8_t *buf, int len) {
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
    case SCHEDULER_DATA:        sched->handleSysex(data, dlen); break;
    default:                    break;  // unknown SysEx ignored
  }
}

// Singletons: live handler, replay handler (for task playback), and scheduler.
// Cross-references are wired once in setup().
static Scheduler       scheduler;
static FirmataProtocol liveHandler;
static FirmataProtocol replayHandler;

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
      if (!isUsable(pin) || !pinConfigured[pin]) continue;  // skip floating, unconfigured pins
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
// Light reset for a fresh connection: clears the live parser and reporting
// state but PRESERVES pin modes/values and any running scheduler tasks, so a
// queued task keeps running across client disconnect/reconnect.
static void resetSessionState() {
  liveHandler.ps = ParserState();
  analogReportMask = 0;
  for (uint8_t i = 0; i < NUM_PORTS; i++) { reportPort[i] = false; previousPort[i] = 0; }
  for (uint8_t i = 0; i < MAX_CONT_READS; i++) contReads[i].active = false;
}

// Full reset (Firmata SYSTEM_RESET 0xFF, and at boot): also resets every pin
// and deletes all scheduler tasks.
static void systemResetState() {
  resetSessionState();
  replayHandler.ps = ParserState();
  scheduler.reset();
  for (uint8_t pin = 0; pin < TOTAL_PINS; pin++) {
    if (isUsable(pin)) pinMode(pin, INPUT);
    pinModes[pin]      = PIN_MODE_INPUT;
    pinValues[pin]     = 0;
    pinConfigured[pin] = false;
  }
  samplingInterval = 19;
}

// Called when a fresh client/central attaches. Does NOT wipe pins or tasks.
static void onNewConnection() {
  resetSessionState();
  sendProtocolVersion();
}

// Build the standard STRING_DATA "eviction notice" the board sends to a client
// right before handing the board to a newcomer (latest-wins). The sentinel
// (0x01 + "EVICTED") is recognised by SwiftFirmataClient. Returns the length.
static int buildEvictionFrame(uint8_t *out) {
  static const char *s = "\x01" "EVICTED";
  int n = 0;
  out[n++] = START_SYSEX;
  out[n++] = STRING_DATA;
  for (const char *p = s; *p; ++p) {
    out[n++] = (uint8_t)(*p) & 0x7F;
    out[n++] = ((uint8_t)(*p) >> 7) & 0x7F;
  }
  out[n++] = END_SYSEX;
  return n;
}

// ===========================================================================
//                            TRANSPORT — Wi-Fi / Bonjour
// ===========================================================================
#if ENABLE_WIFI

static WiFiServer tcpServer(FIRMATA_TCP_PORT);
static WiFiClient tcpClient;
static bool       wifiReady = false;

static void startBonjour() {
  MDNS.end();
  if (!MDNS.begin(MDNS_HOSTNAME)) {
    Serial.println("mDNS start failed");
    return;
  }
  MDNS.addService("firmata", "tcp", FIRMATA_TCP_PORT);   // -> _firmata._tcp
  String ip = WiFi.localIP().toString();
  MDNS.addServiceTxt("firmata", "tcp", "ip",   ip.c_str());
  MDNS.addServiceTxt("firmata", "tcp", "port", String(FIRMATA_TCP_PORT).c_str());
  Serial.printf("Bonjour: _firmata._tcp on %s:%d (instance \"%s\")\n",
                ip.c_str(), FIRMATA_TCP_PORT, MDNS_HOSTNAME);
}

static void startTcpServices() {
  startBonjour();
  tcpServer.begin();
  tcpServer.setNoDelay(true);
  wifiReady = true;
  Serial.print("Wi-Fi up. IP = ");
  Serial.println(WiFi.localIP());
}

static void tcpInit() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(MDNS_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to Wi-Fi \"%s\"", WIFI_SSID);
  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {   // ~16 s, non-fatal
    delay(400);
    Serial.print('.');
    tries++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    startTcpServices();
  } else {
    Serial.println("Wi-Fi not up yet; continuing (BLE still available, will retry).");
  }
}

// Drop the current TCP client (used when another transport takes the board).
static void tcpDrop() {
  if (tcpClient && tcpClient.connected()) {
    tcpClient.stop();
    Serial.println("Evicted TCP client (latest-wins)");
  }
}

static void tcpSend(const uint8_t *buf, size_t len) {
  if (tcpClient && tcpClient.connected()) tcpClient.write(buf, len);
}

static void tcpPoll() {
  // Track Wi-Fi up/down without blocking (the stack auto-reconnects).
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiReady) { wifiReady = false; Serial.println("Wi-Fi lost"); }
    return;
  }
  if (!wifiReady) startTcpServices();   // (re)connected — (re)start services

  // Accept a newly arriving client. A new TCP connection always wins: it
  // replaces any previous TCP client and (via claimMaster) evicts a BLE master.
  WiFiClient incoming = tcpServer.available();
  if (incoming) {
    if (tcpClient && tcpClient.connected()) {                  // within-TCP replace
      uint8_t nb[24];
      tcpClient.write(nb, buildEvictionFrame(nb));             // courtesy notice
      tcpClient.stop();
    }
    tcpClient = incoming;
    tcpClient.setNoDelay(true);
    Serial.println("TCP client connected");
    claimMaster(TR_TCP);
  }
  // Release mastership if our client went away.
  if (activeTransport == TR_TCP && (!tcpClient || !tcpClient.connected())) {
    activeTransport = TR_NONE;
  }
  for (int guard = 0; tcpClient && tcpClient.available() && guard < 1024; guard++) {
    liveHandler.process((uint8_t)tcpClient.read());
  }
}

#endif // ENABLE_WIFI

// ===========================================================================
//                            TRANSPORT — BLE (Nordic UART Service)
// ===========================================================================
#if ENABLE_BLE

#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // host -> device
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // device -> host

static NimBLEServer         *bleServer = nullptr;
static NimBLECharacteristic *txChar    = nullptr;
static volatile bool      bleConnected = false;   // a central currently holds the link
static volatile uint16_t  bleConnId   = 0;        // its connection id
static volatile bool      bleNewConnect = false;  // edge flag: a central just connected
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

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &) override {
    NimBLEAttValue v = c->getValue();
    if (v.length()) rxEnqueue(v.data(), v.length());
  }
};

class ServerCallbacks : public NimBLEServerCallbacks {
  // Use the conn handle so we can do latest-wins across multiple centrals.
  void onConnect(NimBLEServer *s, NimBLEConnInfo &connInfo) override {
    uint16_t newConn = connInfo.getConnHandle();
    if (bleConnected && bleConnId != newConn) {
      s->disconnect(bleConnId);     // a new central wins: drop the previous one
    }
    bleConnId     = newConn;
    bleConnected  = true;
    bleNewConnect = true;           // loop() will claim mastership
    NimBLEDevice::startAdvertising();  // keep advertising so the board stays reclaimable
  }
  void onDisconnect(NimBLEServer *s, NimBLEConnInfo &connInfo, int reason) override {
    if (connInfo.getConnHandle() == bleConnId) {   // the *current* master left
      bleConnected = false;
      negotiatedMTU = 23;
    }
    NimBLEDevice::startAdvertising();
  }
  void onMTUChange(uint16_t MTU, NimBLEConnInfo &) override {
    negotiatedMTU = MTU;
  }
};

static void bleInit() {
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setMTU(517);  // request a large MTU (host has final say)

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  NimBLEService *svc = bleServer->createService(NUS_SERVICE_UUID);

  NimBLECharacteristic *rxChar = svc->createCharacteristic(
      NUS_RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rxChar->setCallbacks(new RxCallbacks());

  // NimBLE auto-adds the 0x2902 CCCD for a NOTIFY characteristic.
  txChar = svc->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);

  svc->start();

  // The 128-bit service UUID goes in the advertisement (required for the client's
  // service-filtered scan); the name rides in the scan response so both fit the
  // 31-byte limit (NimBLE moves the name there when scan response is enabled).
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setName(BLE_DEVICE_NAME);
  adv->enableScanResponse(true);
  NimBLEDevice::startAdvertising();
  Serial.printf("BLE advertising as \"%s\" (Nordic UART Service)\n", BLE_DEVICE_NAME);
}

// Drop the current BLE central (used when another transport takes the board).
static void bleDrop() {
  if (bleConnected && bleServer) {
    bleServer->disconnect(bleConnId);
    Serial.println("Evicted BLE central (latest-wins)");
  }
}

static void blePoll() {
  if (bleNewConnect) {
    bleNewConnect = false;
    bleWasConnected = true;
    Serial.println("BLE central connected");
    claimMaster(TR_BLE);
  } else if (!bleConnected && bleWasConnected) {
    bleWasConnected = false;
    if (activeTransport == TR_BLE) activeTransport = TR_NONE;
    Serial.println("BLE central disconnected");
  }
  int b;
  int guard = 0;
  while ((b = rxDequeue()) >= 0 && guard++ < 4096) liveHandler.process((uint8_t)b);
}

static void bleSend(const uint8_t *buf, size_t len) {
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

#endif // ENABLE_BLE

// ===========================================================================
//                  TRANSPORT — unified front-end (master arbitration)
// ===========================================================================

// Make `who` the single board master, evicting the other transport's holder.
static void claimMaster(ActiveTransport who) {
  // Courtesy notice to the outgoing (cross-transport) master before we drop it.
  // sendFrame() still routes to the *old* master here (activeTransport not yet
  // updated). Best-effort; a small delay lets a BLE notify flush before disconnect.
  if (activeTransport != TR_NONE && activeTransport != who) {
    uint8_t nb[24];
    sendFrame(nb, buildEvictionFrame(nb));
    delay(15);
  }
#if ENABLE_WIFI
  if (who != TR_TCP) tcpDrop();
#endif
#if ENABLE_BLE
  if (who != TR_BLE) bleDrop();
#endif
  activeTransport = who;
  onNewConnection();   // fresh session (keeps pins + scheduler tasks)
}

static void transportInit() {
#if ENABLE_WIFI
  tcpInit();
#endif
#if ENABLE_BLE
  bleInit();
#endif
}

static void transportPoll() {
#if ENABLE_WIFI
  tcpPoll();
#endif
#if ENABLE_BLE
  blePoll();
#endif
}

static void sendFrame(const uint8_t *buf, size_t len) {
#if ENABLE_WIFI
  if (activeTransport == TR_TCP) { tcpSend(buf, len); return; }
#endif
#if ENABLE_BLE
  if (activeTransport == TR_BLE) { bleSend(buf, len); return; }
#endif
}

static bool transportConnected() { return activeTransport != TR_NONE; }

// ===========================================================================
//  Arduino entry points
// ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.printf("=== ESP32 Firmata: %s ===\n", FIRMWARE_NAME);

  analogReadResolution(12);
#if defined(ADC_11db)
  analogSetAttenuation(ADC_11db);
#endif

  // Wire the handler/scheduler cross-references (singletons that live forever).
  liveHandler.sched   = &scheduler;
  replayHandler.sched = &scheduler;
  scheduler.replay    = &replayHandler;

  systemResetState();
  transportInit();
}

void loop() {
  transportPoll();

  // Scheduler runs whether or not a client is connected — that is the whole
  // point: queue a task, disconnect, and the board keeps executing it.
  scheduler.tick();

  if (transportConnected()) {
    checkDigitalInputs();
    unsigned long now = millis();
    if (now - lastSampleMs >= samplingInterval) {
      lastSampleMs = now;
      sampleAnalogAndI2C();
    }
  }
}
