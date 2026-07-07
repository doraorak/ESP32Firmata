/*
 * ESP32Firmata.ino — Firmata 2.x firmware for the original ESP32 (C++/Arduino).
 *
 * Self-contained (no external Firmata library). Speaks Firmata over three
 * transports — Wi-Fi/TCP + Bonjour, BLE Nordic UART (NimBLE), and USB serial
 * (first byte claims the session and silences the console) — one master at a
 * time, latest wins; scheduler tasks survive eviction and disconnect. Includes
 * the on-device task extension: registers, branches, arithmetic, HTTP(S) +
 * JSON/string inspection, nested tasks (ext ops 0x10–0x30 — byte-identical to
 * ESP32FirmataSwift; wire spec in that repo's README). ENABLE_WIFI/ENABLE_BLE
 * gate the radio transports at compile time.
 */

/* ==== USER CONFIGURATION ================================================ */

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
#define FIRMWARE_MINOR     15
#define PROTOCOL_MAJOR     2
#define PROTOCOL_MINOR     8

/* ==== Core-version helper (3.x vs 2.x differ for PWM / LEDC APIs) ======= */
#if defined(ESP_ARDUINO_VERSION) && defined(ESP_ARDUINO_VERSION_VAL)
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    #define FIRMATA_CORE3 1
  #endif
#endif

/* ==== Transport-specific includes ======================================= */
#if ENABLE_WIFI
  #include <esp_log.h>
#include <WiFi.h>
  #include <ESPmDNS.h>
  #include <HTTPClient.h>          // internet actions (scheduler extension)
  #include <WiFiClientSecure.h>    // HTTPS (TLS via ssl_client / mbedTLS)
  // IDF certificate bundle (browser-like root CA set) — validates HTTPS certs.
  // The arduino-esp32 core embeds it (CONFIG_MBEDTLS_CERTIFICATE_BUNDLE).
  extern const uint8_t fm_crt_bundle_start[] asm("_binary_x509_crt_bundle_start");
  extern const uint8_t fm_crt_bundle_end[]   asm("_binary_x509_crt_bundle_end");
  #include <Preferences.h>          // NVS — persist BLE-provisioned Wi-Fi creds
  #include <esp_random.h>           // hardware RNG for the provisioning handshake
  #include <mbedtls/ecdh.h>         // X25519 ECDH ─┐ encrypted Wi-Fi provisioning
  #include <mbedtls/ecp.h>          //              │ (see WIFI_CONFIG below)
  #include <mbedtls/bignum.h>       //              │
  #include <mbedtls/gcm.h>          // AES-256-GCM ─┤
  #include <mbedtls/hkdf.h>         // HKDF-SHA256 ─┘
  #include <mbedtls/md.h>
  #include <mbedtls/platform_util.h>  // mbedtls_platform_zeroize
#endif
#if ENABLE_BLE
  #include <NimBLEDevice.h>   // NimBLE uses far less heap than Bluedroid — leaves
                              // room for a TLS (HTTPS) handshake alongside Wi-Fi.
#endif
#include <Wire.h>

/* ==== Firmata protocol constants ======================================== */
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
static const uint8_t SERVO_CONFIG            = 0x70;
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
static const uint8_t SCHED_REG_REPLY       = 0x0C;   // device -> host: R0-15 + F0-7 snapshot

/* --- Logic extension (see NONSTANDARD.md) -----------------------------------
   On-device registers + if/else so a stored task can make decisions by itself.
   Carried under the standard Scheduler's reserved extension command (0x7F,
   EXTENDED_SCHEDULER_COMMAND) so a base Firmata scheduler ignores it cleanly. */
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
static const uint8_t SCHED_EXT_ARITH         = 0x1A;  // R[dst] = A <op> B (int; op 0+ 1- 2* 3/ 4%)
static const uint8_t SCHED_EXT_SET_FLOAT     = 0x1B;  // F[dst] = float const (IEEE754 bits, 5 enc bytes)
static const uint8_t SCHED_EXT_ARITH_FLOAT   = 0x1C;  // F[dst] = A <op> B (float; op 0+ 1- 2* 3/)
static const uint8_t SCHED_EXT_JSON_FLOAT    = 0x1D;  // F[dst] = json float at path; R[found]
static const uint8_t SCHED_EXT_JSON_TYPE     = 0x1E;  // R[dst] = json type at path (0..6)
static const uint8_t SCHED_EXT_JSON_SIZE     = 0x1F;  // R[dst] = byte length of value span at path
static const uint8_t SCHED_EXT_STR_LEN       = 0x20;  // R[dst] = content length of json string at path
static const uint8_t SCHED_EXT_HEAP          = 0x21;  // R[freeReg]=free heap, R[largestReg]=largest block
static const uint8_t SCHED_EXT_REQUEST_COUNT = 0x22;  // R[dst] = current request count (generation)
static const uint8_t SCHED_EXT_SNAPSHOT      = 0x23;  // copy value at path from live body into a slot
static const uint8_t SCHED_EXT_SELECT        = 0x24;  // pick inspection source (0=live, k=snapshot k-1)
static const uint8_t SCHED_EXT_FREE          = 0x25;  // free a snapshot slot
static const uint8_t SCHED_EXT_LAST_STATUS   = 0x26;  // R[dst] = status of last inspection op
static const uint8_t SCHED_EXT_CMP           = 0x27;  // R[dst] = (A <op> B) ? 1 : 0
static const uint8_t SCHED_EXT_STR_BODY_LEN  = 0x28;  // R[dst] = byte length of the selected body
static const uint8_t SCHED_EXT_STR_EQUALS    = 0x29;  // R[dst] = (selected body == s) ? 1 : 0
static const uint8_t SCHED_EXT_STR_INDEXOF   = 0x2A;  // R[dst] = index of s in body, or -1
static const uint8_t SCHED_EXT_STR_TO_NUM    = 0x2B;  // R[dst] = body parsed as int; R[found] = 0/1
static const uint8_t SCHED_EXT_JSON_GET_STRING = 0x2C;  // copy a JSON string's content at path into a snapshot slot
static const uint8_t SCHED_EXT_STR_SET_SLOT    = 0x2D;  // set a snapshot slot's content to a literal string
static const uint8_t SCHED_EXT_STR_COPY_SLOT   = 0x2E;  // copy one snapshot slot's content into another
static const uint8_t SCHED_EXT_I2C_READ        = 0x2F;  // R[dst] = <count> bytes read from I2C addr/reg, big-endian
static const uint8_t SCHED_EXT_EMIT_STRING     = 0x30;  // device -> host: send a STRING_DATA frame to the master
static const uint8_t SCHED_EXT_REG_QUERY       = 0x31;  // report all registers to the host (SCHED_REG_REPLY)
static const uint8_t SCHED_EXT_WRITE_PIN       = 0x32;  // write a pin from an operand: kind(0=digital,1=analog) pin <operand>
static const uint8_t SCHED_EXT_MODULE_OP       = 0x33;  // deliver a payload to a module from a task
static const uint8_t SCHED_EXT_LOOP            = 0x34;  // begin a counted loop: countLo countHi gapLo gapHi skipLo skipHi
static const uint8_t SCHED_EXT_LOOP_END        = 0x35;  // end of a counted loop: decrement, jump back + gap, or exit

// Result-status codes for inspection ops (read with SCHED_EXT_LAST_STATUS).
static const int32_t ST_OK            = 0;
static const int32_t ST_NOT_FOUND     = 1;
static const int32_t ST_STALE         = 2;
static const int32_t ST_TYPE_MISMATCH = 3;
static const int32_t ST_TOO_BIG       = 4;
static const int32_t ST_ALLOC_FAILED  = 5;

/* --- Encrypted Wi-Fi provisioning over any transport (non-standard) ----------
   Top-level SysEx in Firmata's reserved user range (0x00-0x0F). Lets a client
   hand the board its Wi-Fi credentials — typically over BLE before Wi-Fi is up —
   via an ephemeral X25519 ECDH handshake (HKDF-SHA256 -> AES-256-GCM), so a
   passive sniffer never sees the password. Provisioned creds persist in NVS and
   override the compile-time WIFI_SSID/WIFI_PASS on the next boot. */
static const uint8_t MODULE_DATA       = 0x0D;  // module subsystem (user-range SysEx)
static const uint8_t MODULE_QUERY      = 0x00;
static const uint8_t MODULE_LIST_REPLY = 0x7F;
static const uint8_t WIFI_CONFIG = 0x0C;
static const uint8_t WC_SET    = 0x00;  // host->dev: <clientPub><nonce><ct+tag>  (set + connect + save)
static const uint8_t WC_FORGET = 0x01;  // host->dev: clear stored creds
static const uint8_t WC_QUERY  = 0x02;  // host->dev: request current status
static const uint8_t WC_BEGIN  = 0x03;  // host->dev: start handshake (get device pubkey)
static const uint8_t WC_KEY    = 0x7E;  // dev->host: <devicePub>  (32 bytes, ephemeral)
static const uint8_t WC_STATUS = 0x7F;  // dev->host: <code> <ipLen> <ip>  (code 0=down 1=connected 2=rejected)
// Binary fields above are sent as 14-bit LSB/MSB pairs (lo = b&0x7F, hi = b>>7).

// Pin modes (Firmata §2)
static const uint8_t PIN_MODE_INPUT  = 0x00;
static const uint8_t PIN_MODE_OUTPUT = 0x01;
static const uint8_t PIN_MODE_ANALOG = 0x02;
static const uint8_t PIN_MODE_PWM    = 0x03;
static const uint8_t PIN_MODE_SERVO  = 0x04;
static const uint8_t PIN_MODE_I2C    = 0x06;
static const uint8_t PIN_MODE_PULLUP = 0x0B;

/* ==== ESP32 pin model =================================================== */
static const uint8_t TOTAL_PINS = 40;                 // GPIO 0..39
static const uint8_t NUM_PORTS  = (TOTAL_PINS + 7) / 8; // 5 ports

// ADC1 channels broken out on a typical ESP32 dev board -> analog channels A0..A5.
// (ADC2 pins are avoided: ADC2 cannot be read while Wi-Fi is active.)
static const int8_t  ANALOG_PINS[] = { 32, 33, 34, 35, 36, 39 };
static const uint8_t NUM_ANALOG    = sizeof(ANALOG_PINS) / sizeof(ANALOG_PINS[0]);

// Default I2C pins on the original ESP32.
static const uint8_t I2C_SDA_PIN = 21;
static const uint8_t I2C_SCL_PIN = 22;

/* ==== Types used in function signatures — must be declared before the first
    function so Arduino's auto-generated prototypes (inserted just above the
    first function) can see them.
   ==================== */
static const int SYSEX_MAX = 512;   // fits an HTTP op's URL + body in one SysEx

/* Firmata input-parser state. Two independent instances exist: one for the live
   host connection and one for replaying scheduler tasks — so a half-received
   live message can never be corrupted by task playback (and vice-versa). */
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
static const uint8_t  MAX_LOOP_DEPTH = 4;   // per-task counted-loop nesting (SCHED_EXT_LOOP)

struct SchedTask {
  bool     used = false;
  uint8_t  id = 0;
  uint32_t time_ms = 0;   // absolute millis() when due; 0 = not scheduled
  uint16_t len = 0;       // bytes of task data stored
  uint16_t pos = 0;       // execution cursor
  uint8_t  data[MAX_TASK_BYTES];
  // Counted-loop stack (SCHED_EXT_LOOP / _END): iterations left, gap ms, and the byte
  // position to jump back to for each open loop. Persists across delay-suspends.
  uint8_t  loopDepth = 0;
  uint16_t loopRemaining[MAX_LOOP_DEPTH] = {0};
  uint32_t loopGap[MAX_LOOP_DEPTH]       = {0};
  uint16_t loopResume[MAX_LOOP_DEPTH]    = {0};
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

/* ==== Runtime pin / reporting state ===================================== */
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

/* Non-standard scheduler registers: 16 global Int32s shared across tasks.
   (The Scheduler and FirmataProtocol classes, and their instances, are defined
    further down — after the send helpers they depend on.) */
static const uint8_t NUM_SCHED_REGS = 32;
static const uint8_t NUM_FLOAT_REGS = 16;     // F0..F7 (logic extension)
static const int     NUM_SNAP       = 12;    // 2 JSON snapshot slots (0–1) + 10 string slots (2–11)

// Scratch buffer used to build outgoing frames.
/* Servo pulse range per pin (SERVO_CONFIG overrides the 544-2400 us defaults). */
static int32_t servoMinUs[40];
static int32_t servoMaxUs[40];

static uint8_t   frameBuf[2048];
// Max response bytes retained for JSON/string inspection AND echoed to a host.
static const int HTTP_PARSE_MAX = 4096;

/* ==== Dual-transport master arbitration (latest-wins)
   Exactly one transport "owns" the board at a time. A new connection on either
   transport calls claimMaster(), which evicts the other transport's holder.
   ==================== */
enum ActiveTransport : uint8_t { TR_NONE = 0, TR_TCP, TR_BLE, TR_SERIAL };
static ActiveTransport activeTransport = TR_NONE;

/* Console logging is gated: once a host starts speaking Firmata over USB serial,
   log lines would corrupt the binary stream, so the serial claim silences both
   our own prints and the IDF runtime logs for the rest of the session. */
static bool consoleQuiet = false;
#define LOGP(...)  do { if (!consoleQuiet) Serial.print(__VA_ARGS__);   } while (0)
#define LOGLN(...) do { if (!consoleQuiet) Serial.println(__VA_ARGS__); } while (0)
#define LOGF(...)  do { if (!consoleQuiet) Serial.printf(__VA_ARGS__);  } while (0)

/* ==== Forward declarations ============================================== */
static void transportInit();
static void transportPoll();
static void serialPoll();
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

/* ==== Outgoing Firmata messages ========================================= */
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
      frameBuf[n++] = PIN_MODE_SERVO;  frameBuf[n++] = 14;
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

/* ==== Pin I/O handlers
   I2C device helpers (global; shared by the live handler and periodic sampling).
   ==================== */
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

/* ==== Live protocol handler
    Parses an incoming Firmata byte stream (its own ParserState) and applies
    each command. One instance drives the live transport; a second drives
    scheduler task replay. Both act on shared device state and route scheduler
    SysEx to the shared Scheduler.
   ==================== */
class FirmataProtocol {
 public:
  ParserState ps;
  Scheduler*  sched = nullptr;        // wired once at startup (see setup)

  /* Servo value semantics (standard Firmata): < 544 is an angle in degrees
     (0-180 mapped onto the pin's pulse range); >= 544 is a raw pulse width in us.
     LEDC at 50 Hz / 14-bit: duty = us * 2^14 / 20000. */
  void servoOut(uint8_t pin, int value) {
    int32_t us;
    if (value < 544) {
      int32_t a = value < 0 ? 0 : (value > 180 ? 180 : value);
      us = servoMinUs[pin] + (servoMaxUs[pin] - servoMinUs[pin]) * a / 180;
    } else {
      us = value;
      if (us < servoMinUs[pin]) us = servoMinUs[pin];
      if (us > servoMaxUs[pin]) us = servoMaxUs[pin];
    }
    uint32_t duty = (uint32_t)((uint64_t)us * 16384u / 20000u);
    if (duty > 16383u) duty = 16383u;
    ledcWrite(pin, duty);
    pinValues[pin] = us;
  }

  void handleSetPinMode(uint8_t pin, uint8_t mode) {
    if (pin >= TOTAL_PINS) return;
    if (pinModes[pin] == PIN_MODE_SERVO && mode != PIN_MODE_SERVO) ledcDetach(pin);
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
      case PIN_MODE_SERVO:
        if (isFullDigital(pin)) {
          ledcAttach(pin, 50, 14);
          pinModes[pin] = mode; pinValues[pin] = 0; pinConfigured[pin] = true;
        }
        break;
      case PIN_MODE_I2C:
        pinModes[pin] = mode;
        break;
      default:
        break;
    }
  }

  /* SERVO_CONFIG (0x70): pin, minPulse (14-bit LE), maxPulse (14-bit LE). Sets the
     pulse range and puts the pin in servo mode (standard Firmata behaviour). */
  void handleServoConfig(const uint8_t *data, int len) {
    if (len < 5) return;
    uint8_t pin = data[0];
    if (pin >= TOTAL_PINS || !isFullDigital(pin)) return;
    int32_t mn = (int32_t)(data[1] & 0x7F) | ((int32_t)(data[2] & 0x7F) << 7);
    int32_t mx = (int32_t)(data[3] & 0x7F) | ((int32_t)(data[4] & 0x7F) << 7);
    if (mn > 0 && mx > mn) { servoMinUs[pin] = mn; servoMaxUs[pin] = mx; }
    handleSetPinMode(pin, PIN_MODE_SERVO);
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
    } else if (pinModes[pin] == PIN_MODE_SERVO) {
      servoOut(pin, value);
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
    } else if (pinModes[pin] == PIN_MODE_SERVO) {
      servoOut(pin, value);
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
    LOGP("[host] "); LOGLN(s);
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

/* ==== Firmata Scheduler (SysEx 0x7B) — class
    Stores tasks (recorded Firmata messages + delays) and replays them — through
    a dedicated FirmataProtocol instance (`replay`) — even with no host
    connected. Also owns the non-standard logic extension: 16 Int32 registers,
    if/else, and internet actions. Encoder7Bit helpers live here as methods.
    Wire format follows the official Firmata Scheduler protocol.
   ==================== */
class Scheduler {
 public:
  SchedTask        tasks[MAX_TASKS];
  SchedTask       *running = nullptr;
  int32_t          regs[NUM_SCHED_REGS];
  float            fregs[NUM_FLOAT_REGS] = {0};   // F0..F7 (logic extension)
  FirmataProtocol *replay = nullptr;     // wired once at startup (see setup)
  String           httpRespBody;         // last HTTP response (until next request)

  // Response-inspection state (snapshots + staleness, mirrors the Swift firmware).
  int32_t  requestCount = 0;             // ++ on each HTTP request; basis for handle staleness
  int      inspectSel   = 0;             // inspection source: 0 = live body, k = snapshot slot k-1
  bool     inspectStale = false;         // borrowed live source selected after a newer request
  int32_t  lastStatus   = ST_OK;         // result-status of the last inspection op
  uint8_t *snapBuf[NUM_SNAP] = { nullptr, nullptr };   // owned snapshot copies
  int      snapLen[NUM_SNAP] = { 0, 0 };

  // An operand carries both an int and a float view; `isFloat` says which is
  // authoritative (so a comparison/op promotes to float when either side is float).
  struct Operand { bool isFloat; int32_t i; float f; };

  // Trap-free Float -> Int32 (NaN -> 0, overflow clamps).
  static int32_t f2i(float x) {
    if (x != x) return 0;                          // NaN
    if (x >=  2147483520.0f) return INT32_MAX;
    if (x <= -2147483520.0f) return INT32_MIN;
    return (int32_t)x;
  }
  // Reinterpret 32 raw bits as a Float (IEEE754) without aliasing UB.
  static float bitsToFloat(uint32_t b) { union { uint32_t u; float f; } x; x.u = b; return x.f; }

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
    for (uint8_t i = 0; i < NUM_FLOAT_REGS; i++) fregs[i] = 0;
    requestCount = 0;
    inspectSel = 0; inspectStale = false; lastStatus = ST_OK;
    for (int s = 0; s < NUM_SNAP; s++) { free(snapBuf[s]); snapBuf[s] = nullptr; snapLen[s] = 0; }
    running = nullptr;
  }

  void sendError(uint8_t id) {
    uint8_t b[5] = { START_SYSEX, SCHEDULER_DATA, SCHED_ERROR_REPLY, id, END_SYSEX };
    sendFrame(b, 5);
  }

  /* Task bodies may themselves contain CREATE/ADD/SCHEDULE/DELETE messages (the
     client recorder's addTask/deleteTask — a task spawning tasks). They arrive
     here through the replay handler exactly like host messages. The one hazard is
     a task deleting then re-creating its OWN id mid-run: never hand out the
     instance currently being replayed (`running`), whose data/pos execute() is
     iterating. */
  void create(uint8_t id, uint16_t len) {
    if (find(id) || len > MAX_TASK_BYTES) { sendError(id); return; }
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
      if (!tasks[i].used && &tasks[i] != running) {
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
    t->loopDepth = 0;
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
  // Compare two operands; if either side is a float, promote the comparison to float.
  bool compare(uint8_t op, const Operand &a, const Operand &b) {
    if (a.isFloat || b.isFloat) {
      float x = a.f, y = b.f;
      switch (op) {
        case 0: return x == y; case 1: return x != y; case 2: return x <  y;
        case 3: return x >  y; case 4: return x <= y; case 5: return x >= y;
        default: return false;
      }
    }
    int32_t x = a.i, y = b.i;
    switch (op) {
      case 0: return x == y; case 1: return x != y; case 2: return x <  y;
      case 3: return x >  y; case 4: return x <= y; case 5: return x >= y;
      default: return false;
    }
  }

  /* Read one operand starting at payload[i]; advances i. Type byte:
     0 = int register, 1 = int const (5 enc bytes), 2 = float register,
     3 = float const (IEEE754 bits, 5 enc bytes). */
  Operand readOperand(const uint8_t *payload, int plen, int &i) {
    Operand o = { false, 0, 0.0f };
    if (i >= plen) return o;
    uint8_t type = payload[i++];
    switch (type) {
      case 0: {                                        // int register
        int32_t v = (i < plen) ? regs[payload[i] & 0x1F] : 0; i++;
        return { false, v, (float)v };
      }
      case 2: {                                        // float register
        float r = (i < plen) ? fregs[payload[i] & (NUM_FLOAT_REGS - 1)] : 0.0f; i++;
        return { true, f2i(r), r };
      }
      case 3: {                                        // float constant
        if (i + 5 > plen) { i = plen; return o; }
        float fv = bitsToFloat(sched7BitTime(payload + i)); i += 5;
        return { true, f2i(fv), fv };
      }
      default: {                                       // 1 = int constant
        if (i + 5 > plen) { i = plen; return o; }
        int32_t v = (int32_t)sched7BitTime(payload + i); i += 5;
        return { false, v, (float)v };
      }
    }
  }

  // Advance the running task's cursor by `amount` bytes (forward-only; clamped).
  void skip(uint16_t amount) {
    if (!running) return;
    uint32_t p = (uint32_t)running->pos + amount;
    running->pos = (p > running->len) ? running->len : (uint16_t)p;
  }

  /* Counted loop (SCHED_EXT_LOOP / _END). LOOP pushes (iterations, gap, body-start pos);
     LOOP_END decrements and either jumps back (gap ms apart, via the delay-suspend) or pops.
     count == 0 or nesting past MAX_LOOP_DEPTH skips the body outright. */
  void loopBegin(uint16_t count, uint32_t gap, uint16_t skipLen) {
    if (!running) return;
    if (count == 0 || running->loopDepth >= MAX_LOOP_DEPTH) { skip(skipLen); return; }
    running->loopRemaining[running->loopDepth] = count;
    running->loopGap[running->loopDepth]       = gap;
    running->loopResume[running->loopDepth]    = running->pos;   // pos = start of the body
    running->loopDepth++;
  }

  void loopEnd() {
    if (!running || running->loopDepth == 0) return;
    uint8_t d = running->loopDepth - 1;
    running->loopRemaining[d]--;
    if (running->loopRemaining[d] > 0) {
      running->pos = running->loopResume[d];                     // jump back to the body
      if (running->loopGap[d] > 0) delayRunning(running->loopGap[d]);  // pause between iterations
    } else {
      running->loopDepth--;                                      // loop finished: pop
    }
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

  /* ====== Response inspection — pick the source, then walk it in place ======
     Source is the live body or a snapshot slot, chosen by SELECT. Returns the
     buffer + length and sets `stale` when a borrowed (live) source was selected
     against an out-of-date generation. Each op records a result-status (read with
     SCHED_EXT_LAST_STATUS). */
  const uint8_t *inspectBuf(int &blen, bool &stale) {
    if (inspectSel == 0) {
      blen = httpRespBody.length(); stale = inspectStale;
      return (const uint8_t *)httpRespBody.c_str();
    }
    int s = inspectSel - 1; stale = false;
    blen = (s >= 0 && s < NUM_SNAP) ? snapLen[s] : 0;
    return (s >= 0 && s < NUM_SNAP) ? snapBuf[s] : nullptr;
  }

  // 10^e as Float by bounded repeated multiply (no libm dependency).
  float pow10f(int e) {
    float r = 1; int n = e;
    if (n >= 0) { while (n > 0) { r *= 10; n--; } } else { while (n < 0) { r /= 10; n++; } }
    return r;
  }
  // Parse a JSON number — or quoted "593.2", with optional exponent — into a Float.
  bool parseFloat(const uint8_t *b, int blen, int i0, float &out) {
    int i = jsonSkipWs(b, blen, i0);
    if (i < blen && b[i] == '"') i++;                          // tolerate a quoted number
    if (i >= blen) return false;
    bool neg = false;
    if (b[i] == '-') { neg = true; i++; }
    float v = 0; bool anyInt = false;
    while (i < blen && b[i] >= '0' && b[i] <= '9') { v = v * 10 + (b[i] - '0'); anyInt = true; i++; }
    if (!anyInt) return false;
    if (i < blen && b[i] == '.') {
      i++; float scale = 1;
      while (i < blen && b[i] >= '0' && b[i] <= '9') { v = v * 10 + (b[i] - '0'); scale *= 10; i++; }
      v /= scale;
    }
    if (neg) v = -v;
    if (i < blen && (b[i] == 'e' || b[i] == 'E')) {            // exponent
      i++; bool eneg = false;
      if (i < blen && (b[i] == '+' || b[i] == '-')) { eneg = (b[i] == '-'); i++; }
      int e = 0;
      while (i < blen && b[i] >= '0' && b[i] <= '9') { e = e * 10 + (b[i] - '0'); i++; }
      v *= pow10f(eneg ? -e : e);
    }
    out = v;
    return true;
  }

  // 0x16: R[dst] = number at JSON <path> × 10^scale (truncated); R[found] = 0/1.
  void doJsonNum(const uint8_t *p, int plen) {
    if (plen < 6) return;
    int dst = p[1] & 0x1F, foundReg = p[2] & 0x1F, scale = p[3];
    int pathLen = p[4] | (p[5] << 7);
    if (6 + pathLen > plen) return;
    const uint8_t *path = p + 6;
    regs[dst] = 0; regs[foundReg] = 0;
    int blen; bool stale; const uint8_t *b = inspectBuf(blen, stale);
    if (stale) { lastStatus = ST_STALE; return; }
    if (blen <= 0 || !b) { lastStatus = ST_NOT_FOUND; return; }
    int s, e; int32_t v;
    if (!jsonValueSpan(b, blen, path, pathLen, s, e)) { lastStatus = ST_NOT_FOUND; return; }
    if (!parseScaledNumber(b, blen, s, scale, v)) { lastStatus = ST_TYPE_MISMATCH; return; }
    regs[dst] = v; regs[foundReg] = 1; lastStatus = ST_OK;
  }
  // 0x17 (eq) / 0x19 (contains): compare JSON string at <path> with <str>.
  void doJsonStr(const uint8_t *p, int plen, bool contains) {
    if (plen < 4) return;
    int dst = p[1] & 0x1F;
    int pathLen = p[2] | (p[3] << 7);
    int i = 4 + pathLen;
    if (i + 2 > plen) return;
    const uint8_t *path = p + 4;
    int strLen = p[i] | (p[i + 1] << 7); i += 2;
    if (i + strLen > plen) return;
    const uint8_t *needle = p + i;
    regs[dst] = 0;
    int blen; bool stale; const uint8_t *b = inspectBuf(blen, stale);
    if (stale) { lastStatus = ST_STALE; return; }
    if (blen <= 0 || !b) { lastStatus = ST_NOT_FOUND; return; }
    int s, e;
    if (!jsonValueSpan(b, blen, path, pathLen, s, e)) { lastStatus = ST_NOT_FOUND; return; }
    if (!(s < e && b[s] == '"')) { lastStatus = ST_TYPE_MISMATCH; return; }
    int cs = s + 1, ce = e - 1;                              // string content (escapes raw)
    if (ce >= cs) regs[dst] = (contains ? bytesContain(b, cs, ce, needle, strLen)
                                        : bytesEqual(b, cs, ce, needle, strLen)) ? 1 : 0;
    lastStatus = ST_OK;
  }
  // 0x18: R[dst] = whole body contains <str> ? 1 : 0.
  void doBodyContains(const uint8_t *p, int plen) {
    if (plen < 4) return;
    int dst = p[1] & 0x1F;
    int strLen = p[2] | (p[3] << 7);
    if (4 + strLen > plen) return;
    const uint8_t *needle = p + 4;
    regs[dst] = 0;
    int blen; bool stale; const uint8_t *b = inspectBuf(blen, stale);
    if (stale) { lastStatus = ST_STALE; return; }
    if (blen <= 0 || !b) { lastStatus = ST_NOT_FOUND; return; }
    regs[dst] = bytesContain(b, 0, blen, needle, strLen) ? 1 : 0; lastStatus = ST_OK;
  }
  // 0x28: R[dst] = byte length of the selected body.
  void doStrBodyLen(const uint8_t *p, int plen) {
    if (plen < 2) return;
    int dst = p[1] & 0x1F;
    regs[dst] = 0;
    int blen; bool stale; const uint8_t *b = inspectBuf(blen, stale);
    if (stale) { lastStatus = ST_STALE; return; }
    if (!b) { lastStatus = ST_NOT_FOUND; return; }
    regs[dst] = blen; lastStatus = ST_OK;
  }
  // 0x29: R[dst] = (whole selected body == <str>) ? 1 : 0.
  void doStrEquals(const uint8_t *p, int plen) {
    if (plen < 4) return;
    int dst = p[1] & 0x1F;
    int strLen = p[2] | (p[3] << 7);
    if (4 + strLen > plen) return;
    const uint8_t *needle = p + 4;
    regs[dst] = 0;
    int blen; bool stale; const uint8_t *b = inspectBuf(blen, stale);
    if (stale) { lastStatus = ST_STALE; return; }
    if (!b) { lastStatus = ST_NOT_FOUND; return; }
    regs[dst] = bytesEqual(b, 0, blen, needle, strLen) ? 1 : 0; lastStatus = ST_OK;
  }
  // 0x2A: R[dst] = index of <str> in the selected body, or -1.
  void doStrIndexOf(const uint8_t *p, int plen) {
    if (plen < 4) return;
    int dst = p[1] & 0x1F;
    int strLen = p[2] | (p[3] << 7);
    if (4 + strLen > plen) return;
    const uint8_t *needle = p + 4;
    regs[dst] = -1;
    int blen; bool stale; const uint8_t *b = inspectBuf(blen, stale);
    if (stale) { lastStatus = ST_STALE; return; }
    if (!b) { lastStatus = ST_NOT_FOUND; return; }
    if (strLen == 0) { regs[dst] = 0; lastStatus = ST_OK; return; }
    if (blen >= strLen) {
      for (int i = 0; i <= blen - strLen; i++) {
        bool m = true;
        for (int t = 0; t < strLen; t++) if (b[i + t] != needle[t]) { m = false; break; }
        if (m) { regs[dst] = i; break; }
      }
    }
    lastStatus = ST_OK;
  }
  // 0x2B: R[dst] = body parsed as a leading integer; R[found] = 0/1.
  void doStrToNum(const uint8_t *p, int plen) {
    if (plen < 3) return;
    int dst = p[1] & 0x1F, foundReg = p[2] & 0x1F;
    regs[dst] = 0; regs[foundReg] = 0;
    int blen; bool stale; const uint8_t *b = inspectBuf(blen, stale);
    if (stale) { lastStatus = ST_STALE; return; }
    if (!b || blen <= 0) { lastStatus = ST_NOT_FOUND; return; }
    int i = 0;
    while (i < blen && (b[i] == 0x20 || b[i] == 0x09)) i++;   // skip leading whitespace
    bool negative = false;
    if (i < blen && b[i] == 0x2D) { negative = true; i++; }
    else if (i < blen && b[i] == 0x2B) i++;
    int32_t val = 0; bool any = false;
    while (i < blen && b[i] >= 0x30 && b[i] <= 0x39) {
      any = true;
      if (val > 214748364) val = 2147483647;                 // clamp to int32 max
      else val = val * 10 + (int32_t)(b[i] - 0x30);
      i++;
    }
    if (any) { regs[dst] = negative ? -val : val; regs[foundReg] = 1; lastStatus = ST_OK; }
    else lastStatus = ST_TYPE_MISMATCH;
  }
  // 0x1D: F[dst] = json float at <path>; R[found] = 0/1.
  void doJsonFloat(const uint8_t *p, int plen) {
    if (plen < 5) return;
    int dst = p[1] & (NUM_FLOAT_REGS - 1), foundReg = p[2] & 0x1F;
    int pathLen = p[3] | (p[4] << 7);
    if (5 + pathLen > plen) return;
    const uint8_t *path = p + 5;
    fregs[dst] = 0; regs[foundReg] = 0;
    int blen; bool stale; const uint8_t *b = inspectBuf(blen, stale);
    if (stale) { lastStatus = ST_STALE; return; }
    if (blen <= 0 || !b) { lastStatus = ST_NOT_FOUND; return; }
    int s, e; float v;
    if (!jsonValueSpan(b, blen, path, pathLen, s, e)) { lastStatus = ST_NOT_FOUND; return; }
    if (!parseFloat(b, blen, s, v)) { lastStatus = ST_TYPE_MISMATCH; return; }
    fregs[dst] = v; regs[foundReg] = 1; lastStatus = ST_OK;
  }
  // Shared setup for the query ops (JSON_TYPE/SIZE/STR_LEN): resolves dst + source.
  bool queryArgs(const uint8_t *p, int plen, int &dst,
                 const uint8_t *&b, int &blen, const uint8_t *&path, int &pathLen) {
    if (plen < 4) return false;
    dst = p[1] & 0x1F;
    pathLen = p[2] | (p[3] << 7);
    if (4 + pathLen > plen) return false;
    path = p + 4;
    regs[dst] = 0;
    bool stale; b = inspectBuf(blen, stale);
    if (stale) { lastStatus = ST_STALE; return false; }
    if (blen <= 0 || !b) { lastStatus = ST_NOT_FOUND; return false; }
    return true;
  }
  // 0x1E: R[dst] = JSON type at path (0 none,1 obj,2 arr,3 str,4 num,5 bool,6 null).
  void doJsonType(const uint8_t *p, int plen) {
    int dst, blen, pathLen; const uint8_t *b, *path;
    if (!queryArgs(p, plen, dst, b, blen, path, pathLen)) return;
    int s, e;
    if (!jsonValueSpan(b, blen, path, pathLen, s, e) || !(s < e)) { lastStatus = ST_NOT_FOUND; return; }
    uint8_t c = b[s]; int32_t t = 0;
    if (c == '{') t = 1;
    else if (c == '[') t = 2;
    else if (c == '"') t = 3;
    else if ((c >= '0' && c <= '9') || c == '-') t = 4;
    else if (c == 't' || c == 'f') t = 5;
    else if (c == 'n') t = 6;
    regs[dst] = t; lastStatus = ST_OK;
  }
  // 0x1F: R[dst] = byte length of the value span at path (for sizing a snapshot).
  void doJsonSize(const uint8_t *p, int plen) {
    int dst, blen, pathLen; const uint8_t *b, *path;
    if (!queryArgs(p, plen, dst, b, blen, path, pathLen)) return;
    int s, e;
    if (jsonValueSpan(b, blen, path, pathLen, s, e)) { regs[dst] = (int32_t)(e - s); lastStatus = ST_OK; }
    else { regs[dst] = 0; lastStatus = ST_NOT_FOUND; }
  }
  // 0x20: R[dst] = content length of the JSON string at path (0 if not a string).
  void doStrLen(const uint8_t *p, int plen) {
    int dst, blen, pathLen; const uint8_t *b, *path;
    if (!queryArgs(p, plen, dst, b, blen, path, pathLen)) return;
    int s, e;
    if (!jsonValueSpan(b, blen, path, pathLen, s, e)) { regs[dst] = 0; lastStatus = ST_NOT_FOUND; return; }
    if (e - s >= 2 && b[s] == '"') { regs[dst] = (int32_t)(e - s - 2); lastStatus = ST_OK; }
    else { regs[dst] = 0; lastStatus = ST_TYPE_MISMATCH; }
  }
  // 0x1A: R[dst] = A <op> B (int; op 0+ 1- 2* 3/ 4%). 64-bit intermediates; ÷/%0 -> 0.
  void doArith(const uint8_t *p, int plen) {
    if (plen < 3) return;
    uint8_t sub = p[1]; int dst = p[2] & 0x1F;
    int i = 3;
    int32_t a = readOperand(p, plen, i).i;
    int32_t b = readOperand(p, plen, i).i;
    int32_t r = 0;
    switch (sub) {
      case 0: r = a + b; break;
      case 1: r = a - b; break;
      case 2: r = (int32_t)((int64_t)a * (int64_t)b); break;
      case 3: r = (b != 0) ? (int32_t)((int64_t)a / (int64_t)b) : 0; break;
      case 4: r = (b != 0) ? (int32_t)((int64_t)a % (int64_t)b) : 0; break;
      default: r = 0;
    }
    regs[dst] = r;
  }
  // 0x1C: F[dst] = A <op> B (float; op 0+ 1- 2* 3/). ÷0 -> 0.
  void doArithFloat(const uint8_t *p, int plen) {
    if (plen < 3) return;
    uint8_t sub = p[1]; int dst = p[2] & (NUM_FLOAT_REGS - 1);
    int i = 3;
    float a = readOperand(p, plen, i).f;
    float b = readOperand(p, plen, i).f;
    float r = 0;
    switch (sub) {
      case 0: r = a + b; break;
      case 1: r = a - b; break;
      case 2: r = a * b; break;
      case 3: r = (b != 0) ? a / b : 0; break;
      default: r = 0;
    }
    fregs[dst] = r;
  }
  // 0x1B: F[dst] = float constant (IEEE754 bits in 5 Encoder7Bit bytes).
  void doSetFloat(const uint8_t *p, int plen) {
    if (plen != 7) return;
    fregs[p[1] & (NUM_FLOAT_REGS - 1)] = bitsToFloat(sched7BitTime(p + 2));
  }
  // 0x27: R[dst] = (A <op> B) ? 1 : 0 (same operand decoding + float promotion as IF).
  void doCmp(const uint8_t *p, int plen) {
    if (plen < 3) return;
    uint8_t op = p[1]; int dst = p[2] & 0x1F;
    int i = 3;
    Operand a = readOperand(p, plen, i);
    Operand b = readOperand(p, plen, i);
    regs[dst] = compare(op, a, b) ? 1 : 0;
  }
  // 0x22: R[dst] = current request count (generation; ++ per HTTP request).
  void doReadRequestCount(const uint8_t *p, int plen) {
    if (plen < 2) return;
    regs[p[1] & 0x1F] = requestCount;
  }
  // 0x21: R[freeReg] = free heap, R[largestReg] = largest free block.
  void doHeap(const uint8_t *p, int plen) {
    if (plen < 3) return;
    regs[p[1] & 0x1F] = (int32_t)ESP.getFreeHeap();
    regs[p[2] & 0x1F] = (int32_t)ESP.getMaxAllocHeap();
  }
  // 0x26: R[dst] = status of the last inspection op.
  void doLastStatus(const uint8_t *p, int plen) {
    if (plen < 2) return;
    regs[p[1] & 0x1F] = lastStatus;
  }
  // 0x24: select inspection source — 0 = live body (stale if requestCount != R[expGenReg]),
  //       k = snapshot slot k-1 (always valid).
  void doSelect(const uint8_t *p, int plen) {
    if (plen < 3) return;
    inspectSel = p[1];
    inspectStale = (inspectSel == 0) ? (requestCount != regs[p[2] & 0x1F]) : false;
  }
  // 0x25: free snapshot slot <slot>.
  void doFree(const uint8_t *p, int plen) {
    if (plen < 2) return;
    int slot = p[1];
    if (slot >= 0 && slot < NUM_SNAP) { free(snapBuf[slot]); snapBuf[slot] = nullptr; snapLen[slot] = 0; }
  }
  // 0x23: copy the value at <path> from the LIVE body into snapshot slot <slot>
  //       (pathLen 0 = whole body). Grow-only realloc keeps the copy until freed.
  void doSnapshot(const uint8_t *p, int plen) {
    if (plen < 4) return;
    int slot = p[1]; if (slot < 0 || slot >= NUM_SNAP) return;
    int pathLen = p[2] | (p[3] << 7);
    if (4 + pathLen > plen) return;
    const uint8_t *path = p + 4;
    const uint8_t *b = (const uint8_t *)httpRespBody.c_str();
    int blen = httpRespBody.length();
    if (blen <= 0) { lastStatus = ST_NOT_FOUND; return; }
    int s, e;
    if (!jsonValueSpan(b, blen, path, pathLen, s, e) || e <= s) { lastStatus = ST_NOT_FOUND; return; }
    int n = e - s;
    uint8_t *nb = (uint8_t *)realloc(snapBuf[slot], n);
    if (!nb) { lastStatus = ST_ALLOC_FAILED; return; }
    memcpy(nb, b + s, n);
    snapBuf[slot] = nb; snapLen[slot] = n;
    lastStatus = ST_OK;
  }
  // 0x2C: copy the CONTENT (unquoted) of the JSON string at <path> from the LIVE body into
  //       snapshot slot <slot>. Backs board.json.getString → a TaskString for board.string.
  void doJsonGetString(const uint8_t *p, int plen) {
    if (plen < 4) return;
    int slot = p[1]; if (slot < 0 || slot >= NUM_SNAP) return;
    int pathLen = p[2] | (p[3] << 7);
    if (4 + pathLen > plen) return;
    const uint8_t *path = p + 4;
    const uint8_t *b = (const uint8_t *)httpRespBody.c_str();
    int blen = httpRespBody.length();
    if (blen <= 0) { lastStatus = ST_NOT_FOUND; return; }
    int s, e;
    if (!jsonValueSpan(b, blen, path, pathLen, s, e) || e <= s) { lastStatus = ST_NOT_FOUND; return; }
    if (b[s] != '"' || e - 1 <= s) { lastStatus = ST_TYPE_MISMATCH; return; }   // must be a JSON string
    int cs = s + 1, n = (e - 1) - cs;                       // content between the quotes
    uint8_t *nb = (uint8_t *)realloc(snapBuf[slot], n > 0 ? n : 1);
    if (!nb) { lastStatus = ST_ALLOC_FAILED; return; }
    if (n > 0) memcpy(nb, b + cs, n);
    snapBuf[slot] = nb; snapLen[slot] = n;
    lastStatus = ST_OK;
  }
  // 0x2D: set snapshot slot <slot> to the literal string in the payload — backs
  //       board.string.createString (board.string ops then run on the slot).
  void doStrSetSlot(const uint8_t *p, int plen) {
    if (plen < 4) return;
    int slot = p[1]; if (slot < 0 || slot >= NUM_SNAP) return;
    int sLen = p[2] | (p[3] << 7);
    if (4 + sLen > plen) return;
    uint8_t *nb = (uint8_t *)realloc(snapBuf[slot], sLen > 0 ? sLen : 1);
    if (!nb) { lastStatus = ST_ALLOC_FAILED; return; }
    if (sLen > 0) memcpy(nb, p + 4, sLen);
    snapBuf[slot] = nb; snapLen[slot] = sLen;
    lastStatus = ST_OK;
  }
  // 0x2E: copy snapshot slot <src> content into slot <dst> (backs string changeSlot).
  void doStrCopySlot(const uint8_t *p, int plen) {
    if (plen < 3) return;
    int dst = p[1], src = p[2];
    if (dst < 0 || dst >= NUM_SNAP || src < 0 || src >= NUM_SNAP) return;
    if (!snapBuf[src]) { lastStatus = ST_NOT_FOUND; return; }
    int n = snapLen[src];
    uint8_t *nb = (uint8_t *)realloc(snapBuf[dst], n > 0 ? n : 1);
    if (!nb) { lastStatus = ST_ALLOC_FAILED; return; }
    if (n > 0) memcpy(nb, snapBuf[src], n);
    snapBuf[dst] = nb; snapLen[dst] = n;
    lastStatus = ST_OK;
  }

  /* 0x2F: write the register pointer, read <count> (1..4) bytes from the I2C device, and
           store them big-endian in R[dst]. Lets a task act on an I2C sensor with nobody
           connected (the read reply is consumed on-device, not sent to a host). */
  void doI2CReadReg(const uint8_t *p, int plen) {
    if (plen < 6) return;
    uint16_t address = p[1] & 0x7F;
    int reg   = p[2] | (p[3] << 7);
    int count = p[4]; if (count < 1) count = 1; if (count > 4) count = 4;
    int dst   = p[5] & 0x1F;
    Wire.beginTransmission(address);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    if (i2cReadDelayUs) delayMicroseconds(i2cReadDelayUs);
    int got = Wire.requestFrom((int)address, count);
    int32_t v = 0; int i = 0;
    while (Wire.available() && i < got && i < count) { v = (v << 8) | (Wire.read() & 0xFF); i++; }
    regs[dst] = v;
  }

  // 0x30: send a STRING_DATA frame (board -> host) so a running task can report a message
  //       to a connected master (TCP or BLE). No-op if nobody is connected.
  /* 0x32: write a pin from an OPERAND (register or literal) — task values drive
     outputs: kind 0 = digital (non-zero -> HIGH, OUTPUT pins only), kind 1 =
     analog, routed by the pin's mode (PWM duty or servo degrees/us). */
  void writePinOp(const uint8_t *payload, int plen) {
    if (plen < 4) return;
    uint8_t kind = payload[1];
    uint8_t pin = payload[2] & 0x7F;
    if (pin >= TOTAL_PINS) return;
    int i = 3;
    Operand v = readOperand(payload, plen, i);
    int value = (int)(v.isFloat ? f2i(v.f) : v.i);
    if (kind == 0) {
      if (pinModes[pin] == PIN_MODE_OUTPUT) {
        digitalWrite(pin, value != 0 ? HIGH : LOW);
        pinValues[pin] = value != 0 ? 1 : 0;
      }
    } else {
      if (pinModes[pin] == PIN_MODE_PWM) {
        analogWrite(pin, value > 255 ? 255 : (value < 0 ? 0 : value));
        pinValues[pin] = value;
      } else if (pinModes[pin] == PIN_MODE_SERVO) {
        replay->servoOut(pin, value);
      }
    }
  }

  /* 0x31: report every register to the connected host as SCHED_REG_REPLY —
     16 Int32s then 8 float bit-patterns, each as 5 little-endian 7-bit limbs.
     Works live (host polls shared state) or from inside a task. */
  void regReport() {
    int n = 0;
    frameBuf[n++] = START_SYSEX;
    frameBuf[n++] = SCHEDULER_DATA;
    frameBuf[n++] = SCHED_REG_REPLY;
    for (uint8_t i = 0; i < NUM_SCHED_REGS; i++) {
      uint32_t v = (uint32_t)regs[i];
      for (uint8_t k = 0; k < 5; k++) { frameBuf[n++] = v & 0x7F; v >>= 7; }
    }
    for (uint8_t i = 0; i < NUM_FLOAT_REGS; i++) {
      uint32_t v; memcpy(&v, &fregs[i], 4);
      for (uint8_t k = 0; k < 5; k++) { frameBuf[n++] = v & 0x7F; v >>= 7; }
    }
    frameBuf[n++] = END_SYSEX;
    sendFrame(frameBuf, n);
  }

  void doEmitString(const uint8_t *p, int plen) {
    if (plen < 3) return;
    int sLen = p[1] | (p[2] << 7);
    if (3 + sLen > plen) return;
    int n = 0;
    frameBuf[n++] = START_SYSEX;
    frameBuf[n++] = STRING_DATA;
    for (int k = 0; k < sLen; k++) { frameBuf[n++] = p[3 + k] & 0x7F; frameBuf[n++] = 0; }
    frameBuf[n++] = END_SYSEX;
    sendFrame(frameBuf, n);
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
    int     statusReg = p[2] & 0x1F;
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
    requestCount++;                       // the retained body just changed (handle-staleness basis)
    if (transportConnected()) sendHttpReply(status);
  }

  void handleExt(const uint8_t *payload, int plen) {
    switch (payload[0]) {
      case SCHED_EXT_SET:               // 0x10 reg <const:5>
        if (plen == 7) regs[payload[1] & 0x1F] = (int32_t)sched7BitTime(payload + 2);
        break;
      case SCHED_EXT_READ_DIGITAL:      // 0x11 reg pin
        if (plen == 3) regs[payload[1] & 0x1F] = digitalRead(payload[2]) ? 1 : 0;
        break;
      case SCHED_EXT_READ_ANALOG: {     // 0x12 reg channel
        if (plen == 3) {
          int pin = pinOfAnalogChannel(payload[2]);
          regs[payload[1] & 0x1F] = (pin >= 0) ? analogRead(pin) : 0;
        }
        break;
      }
      case SCHED_EXT_IF: {              // 0x13 op <operandA> <operandB> skipLo skipHi
        int i = 1;
        uint8_t op = payload[i++];
        Operand a = readOperand(payload, plen, i);
        Operand b = readOperand(payload, plen, i);
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
      case SCHED_EXT_ARITH:              // 0x1A subop dst <operandA> <operandB>
        doArith(payload, plen);
        break;
      case SCHED_EXT_SET_FLOAT:          // 0x1B fdst <const:5>
        doSetFloat(payload, plen);
        break;
      case SCHED_EXT_ARITH_FLOAT:        // 0x1C subop fdst <operandA> <operandB>
        doArithFloat(payload, plen);
        break;
      case SCHED_EXT_JSON_FLOAT:         // 0x1D fdst found pathLo pathHi path…
        doJsonFloat(payload, plen);
        break;
      case SCHED_EXT_JSON_TYPE:          // 0x1E dst pathLo pathHi path…
        doJsonType(payload, plen);
        break;
      case SCHED_EXT_JSON_SIZE:          // 0x1F dst pathLo pathHi path…
        doJsonSize(payload, plen);
        break;
      case SCHED_EXT_STR_LEN:            // 0x20 dst pathLo pathHi path…
        doStrLen(payload, plen);
        break;
      case SCHED_EXT_HEAP:               // 0x21 freeReg largestReg
        doHeap(payload, plen);
        break;
      case SCHED_EXT_REQUEST_COUNT:      // 0x22 dst
        doReadRequestCount(payload, plen);
        break;
      case SCHED_EXT_SNAPSHOT:           // 0x23 slot pathLo pathHi path…
        doSnapshot(payload, plen);
        break;
      case SCHED_EXT_SELECT:             // 0x24 sel expGenReg
        doSelect(payload, plen);
        break;
      case SCHED_EXT_FREE:               // 0x25 slot
        doFree(payload, plen);
        break;
      case SCHED_EXT_LAST_STATUS:        // 0x26 dst
        doLastStatus(payload, plen);
        break;
      case SCHED_EXT_CMP:                // 0x27 op dst <operandA> <operandB>
        doCmp(payload, plen);
        break;
      case SCHED_EXT_STR_BODY_LEN:      // 0x28 dst
        doStrBodyLen(payload, plen);
        break;
      case SCHED_EXT_STR_EQUALS:        // 0x29 dst strLo strHi str…
        doStrEquals(payload, plen);
        break;
      case SCHED_EXT_STR_INDEXOF:       // 0x2A dst strLo strHi str…
        doStrIndexOf(payload, plen);
        break;
      case SCHED_EXT_STR_TO_NUM:        // 0x2B dst found
        doStrToNum(payload, plen);
        break;
      case SCHED_EXT_JSON_GET_STRING:   // 0x2C slot pathLo pathHi path…
        doJsonGetString(payload, plen);
        break;
      case SCHED_EXT_STR_SET_SLOT:      // 0x2D slot strLo strHi str…
        doStrSetSlot(payload, plen);
        break;
      case SCHED_EXT_STR_COPY_SLOT:     // 0x2E dst src
        doStrCopySlot(payload, plen);
        break;
      case SCHED_EXT_I2C_READ:          // 0x2F addr regLo regHi count dst
        doI2CReadReg(payload, plen);
        break;
      case SCHED_EXT_EMIT_STRING:       // 0x30 lenLo lenHi bytes…
        doEmitString(payload, plen);
        break;
      case SCHED_EXT_REG_QUERY:         // 0x31: snapshot R0-15 + F0-7 to the host
        regReport();
        break;
      case SCHED_EXT_WRITE_PIN:         // 0x32 kind pin <operand>
        writePinOp(payload, plen);
        break;
      case SCHED_EXT_MODULE_OP:         // 0x33 <moduleId> <payload…>
        if (plen >= 2) moduleDispatch(payload[1], payload + 2, plen - 2);
        break;
      case SCHED_EXT_LOOP:              // 0x34 countLo countHi gapLo gapHi skipLo skipHi
        if (plen == 7) {
          uint16_t count   = (uint16_t)payload[1] | ((uint16_t)payload[2] << 7);
          uint32_t gap     = (uint32_t)payload[3] | ((uint32_t)payload[4] << 7);
          uint16_t skipLen = (uint16_t)payload[5] | ((uint16_t)payload[6] << 7);
          loopBegin(count, gap, skipLen);
        }
        break;
      case SCHED_EXT_LOOP_END:          // 0x35
        loopEnd();
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
      if (t->time_ms != start) {                // a DELAY_TASK fired (or a loop's inter-iteration gap)
        if (t->pos >= t->len) { t->pos = 0; t->loopDepth = 0; }   // trailing delay -> loop from the start
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
#if ENABLE_WIFI
static void handleWiFiConfig(const uint8_t *data, int dlen);   // defined in the Wi-Fi section
#endif

void FirmataProtocol::processSysex(const uint8_t *buf, int len) {
  if (len < 1) return;
  uint8_t cmd = buf[0];
  const uint8_t *data = buf + 1;
  int dlen = len - 1;

  switch (cmd) {
#if ENABLE_WIFI
    case MODULE_DATA:           handleModuleData(data, dlen);  break;
    case WIFI_CONFIG:           handleWiFiConfig(data, dlen);  break;
#endif
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
    case SERVO_CONFIG:          handleServoConfig(data, dlen); break;
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

/* ==== Periodic sampling (device -> host) ================================ */
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

/* ==== Reset
   Light reset for a fresh connection: clears the live parser and reporting
   state but PRESERVES pin modes/values and any running scheduler tasks, so a
   queued task keeps running across client disconnect/reconnect.
   ==================== */
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

/* Build the standard STRING_DATA "eviction notice" the board sends to a client
   right before handing the board to a newcomer (latest-wins). The sentinel
   (0x01 + "EVICTED") is recognised by SwiftFirmataClient. Returns the length. */
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

/* ==== TRANSPORT — Wi-Fi / Bonjour ======================================= */
#if ENABLE_WIFI

static WiFiServer tcpServer(FIRMATA_TCP_PORT);
static WiFiClient tcpClient;
static bool       wifiReady = false;

static void startBonjour() {
  MDNS.end();
  if (!MDNS.begin(MDNS_HOSTNAME)) {
    LOGLN("mDNS start failed");
    return;
  }
  MDNS.addService("firmata", "tcp", FIRMATA_TCP_PORT);   // -> _firmata._tcp
  String ip = WiFi.localIP().toString();
  MDNS.addServiceTxt("firmata", "tcp", "ip",   ip.c_str());
  MDNS.addServiceTxt("firmata", "tcp", "port", String(FIRMATA_TCP_PORT).c_str());
  LOGF("Bonjour: _firmata._tcp on %s:%d (instance \"%s\")\n",
                ip.c_str(), FIRMATA_TCP_PORT, MDNS_HOSTNAME);
}

static void startTcpServices() {
  startBonjour();
  tcpServer.begin();
  tcpServer.setNoDelay(true);
  wifiReady = true;
  LOGP("Wi-Fi up. IP = ");
  LOGLN(WiFi.localIP());
}

/* ==== Encrypted Wi-Fi provisioning  (WIFI_CONFIG SysEx — see the constants above)
     Active creds = NVS-provisioned (if any) else the compile-time WIFI_SSID/PASS.
     A client hands over new creds via an ephemeral X25519 ECDH handshake
     (HKDF-SHA256 -> AES-256-GCM), typically over BLE while Wi-Fi is down.
   ==================== */
static Preferences wcPrefs;
static String      g_ssid, g_pass;
static bool        g_credsLoaded = false;

static void wcLoadCreds() {
  if (g_credsLoaded) return;
  wcPrefs.begin("wifiprov", true);                  // read-only
  g_ssid = wcPrefs.getString("ssid", WIFI_SSID);    // fall back to compile-time
  g_pass = wcPrefs.getString("pass", WIFI_PASS);
  wcPrefs.end();
  g_credsLoaded = true;
}

// (Re)connect Wi-Fi with the active creds; restart Bonjour/TCP on success.
static bool wcConnect() {
  // Already on the target network? Don't tear down a working link (this also lets
  // a re-provision over TCP send its reply before the socket would drop).
  if (WiFi.status() == WL_CONNECTED && WiFi.SSID() == g_ssid && WiFi.psk() == g_pass) {
    if (!wifiReady) startTcpServices();
    return true;
  }
  WiFi.disconnect(false, true);
  wifiReady = false;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(MDNS_HOSTNAME);
  WiFi.begin(g_ssid.c_str(), g_pass.c_str());
  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) { delay(400); tries++; }   // ~12 s
  if (WiFi.status() == WL_CONNECTED) { startTcpServices(); return true; }
  return false;
}

/* Try new creds; persist to NVS ONLY if they actually connect. On failure, roll
   back to the previously-working creds and reconnect — so a wrong password can't
   brick the board's connection or get stored. Returns true iff the new creds joined. */
static bool wcApplyCreds(const String &ssid, const String &pass) {
  String prevSsid = g_ssid, prevPass = g_pass;
  bool wasConnected = (WiFi.status() == WL_CONNECTED);
  g_ssid = ssid; g_pass = pass;
  if (wcConnect()) {                           // new creds joined -> persist them
    wcPrefs.begin("wifiprov", false);
    wcPrefs.putString("ssid", ssid);
    wcPrefs.putString("pass", pass);
    wcPrefs.end();
    return true;
  }
  g_ssid = prevSsid; g_pass = prevPass;        // failed -> revert, leave NVS untouched
  if (wasConnected) wcConnect();               // restore the previously-working network
  return false;
}

static void wcForget() {
  wcPrefs.begin("wifiprov", false); wcPrefs.clear(); wcPrefs.end();
  g_ssid = WIFI_SSID; g_pass = WIFI_PASS;
}

// ---- Crypto: ephemeral X25519 ECDH -> HKDF-SHA256 -> AES-256-GCM ------------
static const char WC_HKDF_SALT[] = "firmata-wifi-prov-v1";   // must match the client

static int wcRng(void *, unsigned char *out, size_t len) { esp_fill_random(out, len); return 0; }

static mbedtls_ecp_group wcGrp;
static mbedtls_mpi       wcPriv;
static bool wcGrpInit = false, wcHavePriv = false;

// Start a handshake: fresh ephemeral keypair; output our 32-byte public key.
static bool wcBegin(uint8_t outPub[32]) {
  if (!wcGrpInit) {
    mbedtls_ecp_group_init(&wcGrp);
    if (mbedtls_ecp_group_load(&wcGrp, MBEDTLS_ECP_DP_CURVE25519) != 0) return false;
    wcGrpInit = true;
  }
  if (wcHavePriv) { mbedtls_mpi_free(&wcPriv); wcHavePriv = false; }
  mbedtls_mpi_init(&wcPriv);
  mbedtls_ecp_point Q; mbedtls_ecp_point_init(&Q);
  int rc = mbedtls_ecdh_gen_public(&wcGrp, &wcPriv, &Q, wcRng, nullptr);
  if (rc == 0) rc = mbedtls_mpi_write_binary_le(&Q.MBEDTLS_PRIVATE(X), outPub, 32);   // X25519 u (LE)
  mbedtls_ecp_point_free(&Q);
  if (rc == 0) { wcHavePriv = true; return true; }
  mbedtls_mpi_free(&wcPriv);
  return false;
}

// Finish: ECDH with the peer pubkey, HKDF -> 32-byte AES key. One-shot.
static bool wcDeriveKey(const uint8_t peerPub[32], uint8_t outKey[32]) {
  if (!wcHavePriv) return false;
  mbedtls_ecp_point Qp; mbedtls_ecp_point_init(&Qp);
  mbedtls_mpi z; mbedtls_mpi_init(&z);
  uint8_t secret[32]; bool ok = false;
  if (mbedtls_mpi_read_binary_le(&Qp.MBEDTLS_PRIVATE(X), peerPub, 32) == 0 &&
      mbedtls_mpi_lset(&Qp.MBEDTLS_PRIVATE(Z), 1) == 0 &&
      mbedtls_ecdh_compute_shared(&wcGrp, &z, &Qp, &wcPriv, wcRng, nullptr) == 0 &&
      mbedtls_mpi_write_binary_le(&z, secret, 32) == 0) {
    const mbedtls_md_info_t *md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (md && mbedtls_hkdf(md, (const uint8_t *)WC_HKDF_SALT, sizeof(WC_HKDF_SALT) - 1,
                           secret, 32, nullptr, 0, outKey, 32) == 0) ok = true;
  }
  mbedtls_platform_zeroize(secret, sizeof(secret));
  mbedtls_mpi_free(&z); mbedtls_ecp_point_free(&Qp);
  mbedtls_mpi_free(&wcPriv); wcHavePriv = false;            // ephemeral: single use
  return ok;
}

static bool wcGcmDecrypt(const uint8_t key[32], const uint8_t nonce[12],
                         const uint8_t *ct, size_t ctLen,
                         const uint8_t *tag, uint8_t *outPt) {
  mbedtls_gcm_context g; mbedtls_gcm_init(&g);
  bool ok = (mbedtls_gcm_setkey(&g, MBEDTLS_CIPHER_ID_AES, key, 256) == 0 &&
             mbedtls_gcm_auth_decrypt(&g, ctLen, nonce, 12, nullptr, 0,
                                      tag, 16, ct, outPt) == 0);
  mbedtls_gcm_free(&g);
  return ok;
}

// ---- Wire helpers: binary <-> 14-bit LSB/MSB pairs (SysEx-safe) -------------
static int wcDecodePairs(const uint8_t *in, int inLen, uint8_t *out, int outCap) {
  int n = inLen / 2; if (n > outCap) n = outCap;
  for (int i = 0; i < n; i++) out[i] = (in[2*i] & 0x7F) | ((in[2*i+1] & 0x01) << 7);
  return n;
}
static int wcPutPairs(uint8_t *out, int n, const uint8_t *src, int len) {
  for (int i = 0; i < len; i++) { out[n++] = src[i] & 0x7F; out[n++] = (src[i] >> 7) & 0x01; }
  return n;
}

static void wcSendKey(const uint8_t pub[32]) {
  uint8_t out[3 + 64 + 1]; int n = 0;
  out[n++] = START_SYSEX; out[n++] = WIFI_CONFIG; out[n++] = WC_KEY;
  n = wcPutPairs(out, n, pub, 32);
  out[n++] = END_SYSEX;
  sendFrame(out, n);
}
// code: 0 = Wi-Fi down, 1 = connected, 2 = creds rejected (decrypt/auth failed).
static void wcSendStatusCode(uint8_t code) {
  String ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : String("");
  int il = ip.length(); if (il > 32) il = 32;
  uint8_t out[3 + 1 + 1 + 64 + 1]; int n = 0;
  out[n++] = START_SYSEX; out[n++] = WIFI_CONFIG; out[n++] = WC_STATUS;
  out[n++] = code;
  out[n++] = il & 0x7F;
  n = wcPutPairs(out, n, (const uint8_t *)ip.c_str(), il);
  out[n++] = END_SYSEX;
  sendFrame(out, n);
}
static void wcSendStatus() { wcSendStatusCode(WiFi.status() == WL_CONNECTED ? 1 : 0); }

static void handleWiFiConfig(const uint8_t *data, int dlen) {
  if (dlen < 1) return;
  switch (data[0]) {
    case WC_BEGIN: { uint8_t pub[32]; if (wcBegin(pub)) wcSendKey(pub); break; }
    case WC_QUERY:  wcSendStatus(); break;
    case WC_FORGET: wcForget(); WiFi.disconnect(false, true); wifiReady = false; wcSendStatus(); break;
    case WC_SET: {
      static uint8_t raw[512];
      int n = wcDecodePairs(data + 1, dlen - 1, raw, sizeof(raw));
      if (n < 32 + 12 + 16) { wcSendStatus(); break; }       // too short to be valid
      const uint8_t *peerPub = raw;
      const uint8_t *nonce   = raw + 32;
      int ctLen = (n - 44) - 16;
      const uint8_t *ct  = raw + 44;
      const uint8_t *tag = ct + ctLen;
      uint8_t key[32]; static uint8_t pt[256];
      bool ok = (ctLen > 0 && ctLen <= (int)sizeof(pt)) && wcDeriveKey(peerPub, key)
                && wcGcmDecrypt(key, nonce, ct, ctLen, tag, pt);
      mbedtls_platform_zeroize(key, sizeof(key));
      if (!ok) { wcSendStatusCode(2); break; }               // creds rejected (decrypt/auth failed)
      // plaintext: <ssidLen> <ssid…> <passLen> <pass…>
      String ssid, pass; bool parsed = false; int i = 0;
      if (i < ctLen) { int sl = pt[i++];
        if (i + sl <= ctLen) { for (int k = 0; k < sl; k++) ssid += (char)pt[i++];
          if (i < ctLen) { int pl = pt[i++];
            if (i + pl <= ctLen) { for (int k = 0; k < pl; k++) pass += (char)pt[i++]; parsed = true; } } } }
      mbedtls_platform_zeroize(pt, sizeof(pt));
      bool applied = (parsed && ssid.length() > 0) && wcApplyCreds(ssid, pass);
      wcSendStatusCode(applied ? 1 : 0);   // 1 = joined the requested network, 0 = not (kept old)
      break;
    }
    default: break;
  }
}

static void tcpInit() {
  wcLoadCreds();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(MDNS_HOSTNAME);
  WiFi.begin(g_ssid.c_str(), g_pass.c_str());
  LOGF("Connecting to Wi-Fi \"%s\"", g_ssid.c_str());
  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {   // ~16 s, non-fatal
    delay(400);
    LOGP('.');
    tries++;
  }
  LOGLN();
  if (WiFi.status() == WL_CONNECTED) {
    startTcpServices();
  } else {
    LOGLN("Wi-Fi not up yet; continuing (BLE still available, will retry).");
  }
}

// Drop the current TCP client (used when another transport takes the board).
static void tcpDrop() {
  if (tcpClient && tcpClient.connected()) {
    tcpClient.stop();
    LOGLN("Evicted TCP client (latest-wins)");
  }
}

static void tcpSend(const uint8_t *buf, size_t len) {
  if (tcpClient && tcpClient.connected()) tcpClient.write(buf, len);
}

static void tcpPoll() {
  // Track Wi-Fi up/down without blocking (the stack auto-reconnects).
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiReady) { wifiReady = false; LOGLN("Wi-Fi lost"); }
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
    LOGLN("TCP client connected");
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

/* ==== TRANSPORT — BLE (Nordic UART Service) ============================= */
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

  /* The 128-bit service UUID goes in the advertisement (required for the client's
     service-filtered scan); the name rides in the scan response so both fit the
     31-byte limit (NimBLE moves the name there when scan response is enabled). */
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setName(BLE_DEVICE_NAME);
  adv->enableScanResponse(true);
  NimBLEDevice::startAdvertising();
  LOGF("BLE advertising as \"%s\" (Nordic UART Service)\n", BLE_DEVICE_NAME);
}

// Drop the current BLE central (used when another transport takes the board).
static void bleDrop() {
  if (bleConnected && bleServer) {
    bleServer->disconnect(bleConnId);
    LOGLN("Evicted BLE central (latest-wins)");
  }
}

static void blePoll() {
  if (bleNewConnect) {
    bleNewConnect = false;
    bleWasConnected = true;
    LOGLN("BLE central connected");
    claimMaster(TR_BLE);
  } else if (!bleConnected && bleWasConnected) {
    bleWasConnected = false;
    if (activeTransport == TR_BLE) activeTransport = TR_NONE;
    LOGLN("BLE central disconnected");
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

/* ==== TRANSPORT — unified front-end (master arbitration) ================ */

// Make `who` the single board master, evicting the other transport's holder.
static void claimMaster(ActiveTransport who) {
  /* Courtesy notice to the outgoing (cross-transport) master before we drop it.
     sendFrame() still routes to the *old* master here (activeTransport not yet
     updated). Best-effort; a small delay lets a BLE notify flush before disconnect. */
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

/* Firmata over USB serial (UART0 — the log console port). The first byte a host
   sends claims the session: the console goes quiet and serial stays master until
   another transport claims it or the board reboots (there is no disconnect event). */
static void serialPoll() {
  int guard = 0;
  while (Serial.available() && guard < 1024) {
    int b = Serial.read();
    if (b < 0) break;
    if (activeTransport != TR_SERIAL) {
      consoleQuiet = true;
      esp_log_level_set("*", ESP_LOG_NONE);
      claimMaster(TR_SERIAL);
    }
    liveHandler.process((uint8_t)b);
    guard++;
  }
}

static void transportPoll() {
#if ENABLE_WIFI
  tcpPoll();
#endif
#if ENABLE_BLE
  blePoll();
#endif
  serialPoll();
}

static void sendFrame(const uint8_t *buf, size_t len) {
#if ENABLE_WIFI
  if (activeTransport == TR_TCP) { tcpSend(buf, len); return; }
#endif
#if ENABLE_BLE
  if (activeTransport == TR_BLE) { bleSend(buf, len); return; }
#endif
  if (activeTransport == TR_SERIAL) { Serial.write(buf, len); return; }
}

static bool transportConnected() { return activeTransport != TR_NONE; }


/* ==== Arduino entry points ============================================== */
void setup() {
  for (uint8_t i = 0; i < TOTAL_PINS; i++) { servoMinUs[i] = 544; servoMaxUs[i] = 2400; }
  Serial.begin(115200);
  delay(200);
  LOGLN();
  LOGF("=== ESP32 Firmata: %s ===\n", FIRMWARE_NAME);

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
  moduleTick();

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
