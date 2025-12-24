/*
  GearGuardDualCore.ino (Arduino IDE)

  Library dependency:
    - Adafruit NeoPixel

  Prefer PlatformIO version in: firmware/platformio/
*/

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include "esp_sleep.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// ========================= CONFIG (edit this) =========================
#define GG_OWNER_NUMBER "+10000000000"
#define GG_DEVICE_NAME  "GearGuard"

// PIN MAP (edit to match your wiring)
#define GG_PIN_MPU_INT    14
#define GG_PIN_BTN_ARM    15
#define GG_PIN_BTN_A      16
#define GG_PIN_BTN_B      17
#define GG_PIN_BTN_C      18
#define GG_PIN_MODEM_RI   19
#define GG_PIN_BTN_D      21

#define GG_PIN_I2C_SCL     7
#define GG_PIN_I2C_SDA     8

#define GG_PIN_SIREN      13
#define GG_PIN_NEOPIXEL   47
#define GG_PIN_PHOTO_ADC   1

#define GG_PIN_MODEM_RX    4
#define GG_PIN_MODEM_TX    5
#define GG_MODEM_BAUD 115200

// ========================= FIRMWARE =========================
\
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include "esp_sleep.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// ========================= TUNING / ADJUSTABLES =========================
struct {
  // Motion sampling
  uint16_t sample_hz = 50;
  uint32_t investigate_ms = 6500;

  // linDevG = abs(|accel| - 1.0)
  float bump_peak_g  = 0.08f;
  float warn_peak_g  = 0.18f;
  float carry_peak_g = 0.25f;

  // Time above thresholds inside investigate window (ms)
  uint32_t warn_over_ms  = 1200;
  uint32_t carry_over_ms = 3200;

  // Activity score: sum(linDevG * dt_ms)
  float carry_activity_gms = 1200.0f;

  // Optional “warn twice then alarm”
  bool warning_escalation_enabled = true;
  uint32_t warning_escalation_window_ms = 15000;

  // Code entry / arming UX
  uint32_t arm_hold_ms = 2000;         // hold ARM to arm (while disarmed)
  uint32_t setcode_hold_ms = 6000;     // hold ARM to enter local code-change (while disarmed)
  uint32_t disarm_window_ms = 20000;   // time allowed to enter code after wake/alarm
  uint32_t code_step_timeout_ms = 8000;

  // Siren sweep
  uint16_t siren_lo_hz = 1700;
  uint16_t siren_hi_hz = 3100;
  uint16_t siren_step_hz = 200;
  uint16_t siren_step_ms = 120;
  uint16_t siren_gap_ms  = 40;

  // Light tamper (only evaluated while awake)
  int tamper_adc_trip = 1800;

  // GNSS
  uint32_t gps_fix_timeout_ms = 90000;

} CFG;

struct BeepStep { uint16_t freq; uint16_t on_ms; uint16_t off_ms; };

static const BeepStep PATTERN_BUMP[]       = { {2600, 60, 0} };
static const BeepStep PATTERN_WARN[]       = { {2200, 90, 90}, {2200, 90, 0} };
static const BeepStep PATTERN_ARMED[]      = { {1800, 70, 80}, {2400, 70, 0} };
static const BeepStep PATTERN_DISARMED[]   = { {2400, 70, 80}, {1800, 70, 0} };
static const BeepStep PATTERN_WRONG_CODE[] = { {1200, 120, 80}, {1200, 120, 80}, {1200, 120, 0} };

// ========================= OBJECTS =========================
static Preferences prefs;
static Adafruit_NeoPixel px(1, GG_PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
static HardwareSerial Modem(1);

// ========================= SHARED STATE =========================
enum DeviceState : uint8_t { ST_DISARMED=0, ST_ARMED, ST_WARNING, ST_ALARM, ST_SETCODE };
static volatile DeviceState g_state = ST_DISARMED;
static volatile bool g_modem_busy = false;
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// ========================= EVENTS (to comms task) =========================
enum EventType : uint8_t { EV_NONE=0, EV_SEND_SMS, EV_CARRY_AWAY_ALERT, EV_GPS_REPLY_NOW };

struct Event {
  EventType type;
  char msg[220];
};

static QueueHandle_t g_evtQ;

// ========================= CODE / PERSISTENCE =========================
// Code digits are 1..4 (A/B/C/D).
static uint8_t g_code[4] = {1,2,3,4};
static const uint8_t DEFAULT_CODE[4] = {1,2,3,4};

static void loadCode() {
  prefs.begin("gearguard", true);
  size_t n = prefs.getBytesLength("code");
  if (n == 4) prefs.getBytes("code", g_code, 4);
  else memcpy(g_code, DEFAULT_CODE, 4);
  prefs.end();
}
static void saveCode() {
  prefs.begin("gearguard", false);
  prefs.putBytes("code", g_code, 4);
  prefs.end();
}

// ========================= HELPERS =========================
static void pxSet(uint8_t r, uint8_t g, uint8_t b) {
  px.setPixelColor(0, px.Color(r,g,b));
  px.show();
}
static void pxFlash(uint8_t r, uint8_t g, uint8_t b, uint8_t times=2, uint16_t ms=120) {
  for (uint8_t i=0;i<times;i++){ pxSet(r,g,b); delay(ms); pxSet(0,0,0); delay(ms); }
}

static void toneOn(uint16_t freq_hz) {
  const int ch = 0;
  ledcSetup(ch, 2000, 10);
  ledcAttachPin(GG_PIN_SIREN, ch);
  ledcWriteTone(ch, freq_hz);
}
static void toneOff() {
  const int ch = 0;
  ledcWriteTone(ch, 0);
}

template <size_t N>
static void playPattern(const BeepStep (&steps)[N]) {
  for (size_t i=0;i<N;i++){
    if (steps[i].freq) toneOn(steps[i].freq);
    delay(steps[i].on_ms);
    toneOff();
    delay(steps[i].off_ms);
  }
}

// Non-blocking siren sweep service (Motion/UI task)
static void sirenService(bool enabled) {
  static uint32_t lastMs = 0;
  static uint16_t f = 0;
  static int dir = 1;
  if (!enabled) { toneOff(); return; }
  uint32_t now = millis();
  if (f == 0) f = CFG.siren_lo_hz;
  if (now - lastMs >= CFG.siren_step_ms) {
    lastMs = now;
    toneOn(f);
    f = (uint16_t)(f + dir * CFG.siren_step_hz);
    if (f >= CFG.siren_hi_hz) { f = CFG.siren_hi_hz; dir = -1; }
    if (f <= CFG.siren_lo_hz) { f = CFG.siren_lo_hz; dir =  1; }
  }
}

static void setState(DeviceState s) {
  portENTER_CRITICAL(&g_mux);
  g_state = s;
  portEXIT_CRITICAL(&g_mux);
}
static DeviceState getState() {
  portENTER_CRITICAL(&g_mux);
  DeviceState s = g_state;
  portEXIT_CRITICAL(&g_mux);
  return s;
}

static bool codeMatches(const uint8_t a[4], const uint8_t b[4]) {
  for (int i=0;i<4;i++) if (a[i]!=b[i]) return false;
  return true;
}

// ========================= BUTTONS (wake-capable) =========================
static void buttonsInit() {
  pinMode(GG_PIN_BTN_ARM, INPUT_PULLDOWN);
  pinMode(GG_PIN_BTN_A,   INPUT_PULLDOWN);
  pinMode(GG_PIN_BTN_B,   INPUT_PULLDOWN);
  pinMode(GG_PIN_BTN_C,   INPUT_PULLDOWN);
  pinMode(GG_PIN_BTN_D,   INPUT_PULLDOWN);
  pinMode(GG_PIN_MODEM_RI, INPUT_PULLDOWN);
  pinMode(GG_PIN_MPU_INT, INPUT);
}

static uint8_t readCodeDigitEdge() {
  static uint32_t lastMs = 0;
  static uint32_t lastState = 0;
  uint32_t now = millis();
  if (now - lastMs < 30) return 0;
  lastMs = now;

  uint32_t s =
    ((digitalRead(GG_PIN_BTN_A) ? 1 : 0) << 0) |
    ((digitalRead(GG_PIN_BTN_B) ? 1 : 0) << 1) |
    ((digitalRead(GG_PIN_BTN_C) ? 1 : 0) << 2) |
    ((digitalRead(GG_PIN_BTN_D) ? 1 : 0) << 3);

  uint32_t rising = (~lastState) & s;
  lastState = s;

  if (rising & (1<<0)) return 1;
  if (rising & (1<<1)) return 2;
  if (rising & (1<<2)) return 3;
  if (rising & (1<<3)) return 4;
  return 0;
}

// ========================= LIGHT TAMPER (awake-only) =========================
static bool tamperLightTrip() {
  int v = analogRead(GG_PIN_PHOTO_ADC);
  return v > CFG.tamper_adc_trip;
}

// ========================= MPU6050 (minimal I2C) =========================
static const uint8_t MPU_ADDR = 0x68;
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_CONFIG       = 0x1A;
static const uint8_t REG_SMPLRT_DIV   = 0x19;
static const uint8_t REG_INT_PIN_CFG  = 0x37;
static const uint8_t REG_INT_ENABLE   = 0x38;
static const uint8_t REG_INT_STATUS   = 0x3A;
static const uint8_t REG_MOT_THR      = 0x1F;
static const uint8_t REG_MOT_DUR      = 0x20;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
static bool mpuReadBytes(uint8_t reg, uint8_t* buf, size_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, (int)len) != (int)len) return false;
  for (size_t i=0;i<len;i++) buf[i] = Wire.read();
  return true;
}
static void mpuClearInt() {
  uint8_t v=0;
  (void)mpuReadBytes(REG_INT_STATUS, &v, 1);
}
static bool mpuInit() {
  if (!mpuWrite(REG_PWR_MGMT_1, 0x00)) return false;
  delay(50);
  mpuWrite(REG_ACCEL_CONFIG, 0x00);
  mpuWrite(REG_CONFIG, 0x03);
  mpuWrite(REG_SMPLRT_DIV, 0x04);
  return true;
}
static void mpuEnableMotionWake(uint8_t motThr, uint8_t motDurMs) {
  mpuWrite(REG_MOT_THR, motThr);
  mpuWrite(REG_MOT_DUR, motDurMs);
  mpuWrite(REG_INT_PIN_CFG, 0x20);
  mpuWrite(REG_INT_ENABLE, 0x40);
  mpuClearInt();
}
static bool mpuReadAccelG(float& ax, float& ay, float& az) {
  uint8_t b[6];
  if (!mpuReadBytes(REG_ACCEL_XOUT_H, b, 6)) return false;
  int16_t x = (int16_t)((b[0]<<8) | b[1]);
  int16_t y = (int16_t)((b[2]<<8) | b[3]);
  int16_t z = (int16_t)((b[4]<<8) | b[5]);
  ax = (float)x / 16384.0f;
  ay = (float)y / 16384.0f;
  az = (float)z / 16384.0f;
  return true;
}

// ========================= MOTION CLASSIFICATION =========================
struct MotionSummary {
  float peakLinDevG = 0;
  uint32_t msOverWarn = 0;
  uint32_t msOverCarry = 0;
  float activityGms = 0;
};
static MotionSummary sampleMotion(uint32_t windowMs) {
  MotionSummary ms;
  const uint32_t dtMs = max<uint32_t>(1, 1000UL / CFG.sample_hz);
  uint32_t start = millis();
  while (millis() - start < windowMs) {
    float ax, ay, az;
    if (mpuReadAccelG(ax, ay, az)) {
      float mag = sqrtf(ax*ax + ay*ay + az*az);
      float lin = fabsf(mag - 1.0f);
      ms.peakLinDevG = max(ms.peakLinDevG, lin);
      ms.activityGms += lin * (float)dtMs;
      if (lin >= CFG.warn_peak_g)  ms.msOverWarn  += dtMs;
      if (lin >= CFG.carry_peak_g) ms.msOverCarry += dtMs;
    }
    delay(dtMs);
  }
  return ms;
}

// ========================= DEEP SLEEP =========================
static uint64_t ext1MaskAll() {
  return (1ULL << GG_PIN_MPU_INT) |
         (1ULL << GG_PIN_BTN_ARM) |
         (1ULL << GG_PIN_BTN_A) |
         (1ULL << GG_PIN_BTN_B) |
         (1ULL << GG_PIN_BTN_C) |
         (1ULL << GG_PIN_BTN_D) |
         (1ULL << GG_PIN_MODEM_RI);
}
static void enterDeepSleep() {
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_ext1_wakeup(ext1MaskAll(), ESP_EXT1_WAKEUP_ANY_HIGH);
  delay(50);
  esp_deep_sleep_start();
}

// ========================= MODEM / COMMS =========================
static void modemFlush() { while (Modem.available()) Modem.read(); }

static String modemReadAll(uint32_t timeoutMs) {
  String out;
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (Modem.available()) {
      out += char(Modem.read());
      start = millis();
    }
    delay(5);
  }
  return out;
}

static bool atOK(const String& cmd, uint32_t timeoutMs=2000) {
  modemFlush();
  Modem.print(cmd); Modem.print("\r");
  String r = modemReadAll(timeoutMs);
  return r.indexOf("OK") >= 0;
}

static bool waitAT(uint32_t timeoutMs=8000) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (atOK("AT", 800)) return true;
    delay(250);
  }
  return false;
}

static bool sendSMS(const String& to, const String& body) {
  portENTER_CRITICAL(&g_mux); g_modem_busy = true; portEXIT_CRITICAL(&g_mux);

  atOK("AT+CMGF=1", 2000);

  modemFlush();
  Modem.print("AT+CMGS=\""); Modem.print(to); Modem.print("\"\r");

  uint32_t start = millis();
  bool prompt = false;
  while (millis() - start < 6000) {
    String r = modemReadAll(200);
    if (r.indexOf('>') >= 0) { prompt = true; break; }
    if (r.indexOf("ERROR") >= 0) break;
  }
  if (!prompt) { portENTER_CRITICAL(&g_mux); g_modem_busy=false; portEXIT_CRITICAL(&g_mux); return false; }

  Modem.print(body);
  Modem.write((char)0x1A);

  String r = modemReadAll(15000);
  bool ok = (r.indexOf("OK") >= 0 || r.indexOf("+CMGS:") >= 0);

  portENTER_CRITICAL(&g_mux); g_modem_busy = false; portEXIT_CRITICAL(&g_mux);
  return ok;
}

static bool gnssOn() {
  if (atOK("AT+CGNSSPWR=1", 3000)) return true;
  if (atOK("AT+CGNSPWR=1", 3000)) return true;
  return false;
}
static void gnssOff() {
  atOK("AT+CGNSSPWR=0", 3000);
  atOK("AT+CGNSPWR=0", 3000);
}

static bool parseCGNSINF(const String& resp, double& lat, double& lon, bool& fix) {
  int idx = resp.indexOf("+CGNSINF:");
  if (idx < 0) return false;
  int eol = resp.indexOf('\n', idx);
  String line = (eol > idx) ? resp.substring(idx, eol) : resp.substring(idx);
  int colon = line.indexOf(':');
  if (colon < 0) return false;
  String s = line.substring(colon + 1); s.trim();

  String tok[24]; int n=0; int start=0;
  for (int i=0;i<=s.length() && n<24;i++){
    if (i==s.length() || s[i]==','){
      tok[n++] = s.substring(start,i);
      tok[n-1].trim();
      start=i+1;
    }
  }
  if (n < 6) return false;

  fix = (tok[1].toInt() == 1);
  lat = tok[3].toDouble();
  lon = tok[4].toDouble();
  if (lat < -90 || lat > 90 || lon < -180 || lon > 180) return false;
  return true;
}

static bool getGpsFix(double& lat, double& lon) {
  portENTER_CRITICAL(&g_mux); g_modem_busy = true; portEXIT_CRITICAL(&g_mux);

  gnssOn();
  uint32_t start = millis();
  while (millis() - start < CFG.gps_fix_timeout_ms) {
    modemFlush();
    Modem.print("AT+CGNSINF\r");
    String r = modemReadAll(2500);

    bool fix=false;
    if (parseCGNSINF(r, lat, lon, fix) && fix) {
      gnssOff();
      portENTER_CRITICAL(&g_mux); g_modem_busy=false; portEXIT_CRITICAL(&g_mux);
      return true;
    }
    delay(1200);
  }

  gnssOff();
  portENTER_CRITICAL(&g_mux); g_modem_busy=false; portEXIT_CRITICAL(&g_mux);
  return false;
}

static String mapsLink(double lat, double lon) {
  String s = "https://maps.google.com/?q=";
  s += String(lat, 6); s += ","; s += String(lon, 6);
  return s;
}

static String normalizePhone(String s) {
  s.trim(); s.replace("\"","");
  return s;
}

static bool isOwner(const String& from) {
  String a = normalizePhone(from);
  String b = String(GG_OWNER_NUMBER);
  if (a == b) return true;
  String a10 = a; a10.replace("+1","");
  String b10 = b; b10.replace("+1","");
  if (a10.length() >= 10 && b10.length() >= 10)
    return a10.endsWith(b10.substring(b10.length()-10));
  return false;
}

static bool parseCodeToken(const String& token, uint8_t out[4]) {
  if (token.length() != 4) return false;
  for (int i=0;i<4;i++){
    char c = token[i];
    if (c < '1' || c > '4') return false;
    out[i] = (uint8_t)(c - '0');
  }
  return true;
}

static bool readOneUnreadSMS(String& fromOut, String& bodyOut) {
  modemFlush();
  Modem.print("AT+CMGL=\"REC UNREAD\"\r");
  String r = modemReadAll(6000);

  int h = r.indexOf("+CMGL:");
  if (h < 0) return false;

  int hEnd = r.indexOf('\n', h);
  if (hEnd < 0) return false;
  String header = r.substring(h, hEnd); header.trim();

  int q1 = header.indexOf('\"');
  int q2 = header.indexOf('\"', q1+1);
  int q3 = header.indexOf('\"', q2+1);
  int q4 = header.indexOf('\"', q3+1);
  int q5 = header.indexOf('\"', q4+1);
  int q6 = header.indexOf('\"', q5+1);
  String from = (q5>=0 && q6>q5) ? header.substring(q5+1, q6) : "";

  int bStart = hEnd + 1;
  int bEnd = r.indexOf('\n', bStart);
  if (bEnd < 0) bEnd = r.length();
  String body = r.substring(bStart, bEnd); body.trim();

  fromOut = from;
  bodyOut = body;

  atOK("AT+CMGD=1,4", 6000);
  return true;
}

static void handleSmsCommand(const String& fromRaw, String bodyRaw) {
  String from = normalizePhone(fromRaw);
  if (!isOwner(from)) return;

  bodyRaw.trim();
  bodyRaw.toUpperCase();

  String p[3]; int n=0;
  int start=0;
  for (int i=0;i<=bodyRaw.length() && n<3;i++){
    if (i==bodyRaw.length() || bodyRaw[i]==' ') {
      String part = bodyRaw.substring(start,i); part.trim();
      if (part.length()) p[n++] = part;
      start = i+1;
    }
  }
  if (n < 2) { sendSMS(GG_OWNER_NUMBER, String(GG_DEVICE_NAME)+": command rejected"); return; }

  uint8_t provided[4];
  if (!parseCodeToken(p[1], provided) || !codeMatches(provided, g_code)) {
    sendSMS(GG_OWNER_NUMBER, String(GG_DEVICE_NAME)+": command rejected");
    return;
  }

  if (p[0] == "DISARM") {
    setState(ST_DISARMED);
    sendSMS(GG_OWNER_NUMBER, String(GG_DEVICE_NAME)+": DISARMED");
    return;
  }
  if (p[0] == "ARM") {
    setState(ST_ARMED);
    sendSMS(GG_OWNER_NUMBER, String(GG_DEVICE_NAME)+": ARMED");
    return;
  }
  if (p[0] == "WHERE") {
    Event ev{}; ev.type = EV_GPS_REPLY_NOW;
    xQueueSend(g_evtQ, &ev, 0);
    return;
  }
  if (p[0] == "SETCODE") {
    if (n < 3) { sendSMS(GG_OWNER_NUMBER, String(GG_DEVICE_NAME)+": SETCODE requires old+new"); return; }
    uint8_t newC[4];
    if (!parseCodeToken(p[2], newC)) { sendSMS(GG_OWNER_NUMBER, String(GG_DEVICE_NAME)+": new code must be 4 digits 1-4"); return; }
    memcpy((void*)g_code, newC, 4);
    saveCode();
    sendSMS(GG_OWNER_NUMBER, String(GG_DEVICE_NAME)+": CODE UPDATED");
    return;
  }

  sendSMS(GG_OWNER_NUMBER, String(GG_DEVICE_NAME)+": Unknown cmd (DISARM/ARM/WHERE/SETCODE)");
}

// ========================= TASK: COMMS (Core 1) =========================
static void TaskComms(void* arg) {
  Modem.begin(GG_MODEM_BAUD, SERIAL_8N1, GG_PIN_MODEM_RX, GG_PIN_MODEM_TX);
  delay(200);

  if (waitAT()) {
    atOK("ATE0", 1000);
    atOK("AT+CMGF=1", 1500);
    atOK("AT+CNMI=2,1,0,0,0", 1500);
  }

  for (int i=0;i<5;i++){
    String f,b;
    if (!readOneUnreadSMS(f,b)) break;
    handleSmsCommand(f,b);
  }

  for (;;) {
    Event ev;
    while (xQueueReceive(g_evtQ, &ev, 0) == pdTRUE) {
      if (ev.type == EV_SEND_SMS) {
        sendSMS(GG_OWNER_NUMBER, String(ev.msg));
      } else if (ev.type == EV_CARRY_AWAY_ALERT) {
        sendSMS(GG_OWNER_NUMBER, String(GG_DEVICE_NAME) + ": CARRY-AWAY detected. Getting GPS...");
        double lat=0, lon=0;
        if (getGpsFix(lat, lon)) {
          sendSMS(GG_OWNER_NUMBER, String(GG_DEVICE_NAME) + " GPS: " + mapsLink(lat, lon) + "\n(" + String(lat,6) + "," + String(lon,6) + ")");
        } else {
          sendSMS(GG_OWNER_NUMBER, String(GG_DEVICE_NAME) + ": GPS fix failed (timeout).");
        }
      } else if (ev.type == EV_GPS_REPLY_NOW) {
        sendSMS(GG_OWNER_NUMBER, String(GG_DEVICE_NAME) + ": Getting GPS...");
        double lat=0, lon=0;
        if (getGpsFix(lat, lon)) {
          sendSMS(GG_OWNER_NUMBER, String(GG_DEVICE_NAME) + " GPS: " + mapsLink(lat, lon) + "\n(" + String(lat,6) + "," + String(lon,6) + ")");
        } else {
          sendSMS(GG_OWNER_NUMBER, String(GG_DEVICE_NAME) + ": GPS fix failed (timeout).");
        }
      }
    }

    if (digitalRead(GG_PIN_MODEM_RI)) {
      String f,b;
      while (readOneUnreadSMS(f,b)) {
        handleSmsCommand(f,b);
        vTaskDelay(pdMS_TO_TICKS(20));
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ========================= TASK: MOTION/UI (Core 0) =========================
static void TaskMotionUI(void* arg) {
  uint8_t entry[4] = {0,0,0,0};
  uint8_t entryLen = 0;
  uint32_t lastEntryMs = 0;
  uint32_t disarmWindowUntil = 0;

  enum SetStep { STEP_NONE, STEP_OLD, STEP_NEW1, STEP_NEW2 };
  SetStep setStep = STEP_NONE;
  uint8_t new1[4] = {0,0,0,0};

  uint32_t armDownSince = 0;
  uint32_t lastWarnMs = 0;

  auto openDisarmWindow = [&](){ disarmWindowUntil = millis() + CFG.disarm_window_ms; };
  auto disarmWindowOpen = [&](){ return millis() <= disarmWindowUntil; };
  auto resetEntry = [&](){ entryLen = 0; lastEntryMs = millis(); };
  auto pushDigit = [&](uint8_t d){
    if (!disarmWindowOpen()) return;
    if (millis() - lastEntryMs > CFG.code_step_timeout_ms) resetEntry();
    lastEntryMs = millis();
    if (entryLen < 4) entry[entryLen++] = d;
  };

  for (;;) {
    DeviceState st = getState();

    if (st == ST_DISARMED) pxSet(0,25,0);
    else if (st == ST_ARMED) pxSet(25,0,0);
    else if (st == ST_WARNING) pxSet(25,25,0);
    else if (st == ST_SETCODE) pxSet(0,0,25);
    else if (st == ST_ALARM) {
      static uint32_t t=0; static bool on=false;
      if (millis()-t>200){ t=millis(); on=!on; pxSet(on?25:0,0,0); }
    }

    sirenService(st == ST_ALARM);

    if (digitalRead(GG_PIN_BTN_A) || digitalRead(GG_PIN_BTN_B) || digitalRead(GG_PIN_BTN_C) ||
        digitalRead(GG_PIN_BTN_D) || digitalRead(GG_PIN_BTN_ARM) || digitalRead(GG_PIN_MPU_INT)) {
      openDisarmWindow();
    }

    if (st == ST_ARMED && tamperLightTrip()) {
      setState(ST_ALARM);
      openDisarmWindow();
      Event ev{}; ev.type = EV_CARRY_AWAY_ALERT;
      xQueueSend(g_evtQ, &ev, 0);
    }

    bool armDown = digitalRead(GG_PIN_BTN_ARM);
    if (armDown) {
      if (armDownSince == 0) armDownSince = millis();

      if (st == ST_DISARMED && (millis() - armDownSince) >= CFG.setcode_hold_ms) {
        setState(ST_SETCODE);
        setStep = STEP_OLD;
        openDisarmWindow();
        resetEntry();
        pxFlash(0,0,25,2,80);
        armDownSince = 0;
      }

      if (st == ST_DISARMED && (millis() - armDownSince) >= CFG.arm_hold_ms && getState() != ST_SETCODE) {
        setState(ST_ARMED);
        playPattern(PATTERN_ARMED);
        openDisarmWindow();
        armDownSince = 0;
      }
    } else {
      armDownSince = 0;
    }

    uint8_t d = readCodeDigitEdge();
    if (d && (st == ST_ARMED || st == ST_ALARM || st == ST_WARNING || st == ST_SETCODE)) {
      pushDigit(d);
    }

    if (entryLen > 0 && (millis() - lastEntryMs > CFG.code_step_timeout_ms)) {
      resetEntry();
      if (getState() == ST_WARNING) setState(ST_ARMED);
    }

    if (entryLen == 4) {
      if (st == ST_SETCODE) {
        if (setStep == STEP_OLD) {
          if (!codeMatches(entry, g_code)) { playPattern(PATTERN_WRONG_CODE); setState(ST_DISARMED); setStep = STEP_NONE; resetEntry(); }
          else { setStep = STEP_NEW1; resetEntry(); pxFlash(0,0,25,1,120); }
        } else if (setStep == STEP_NEW1) {
          memcpy(new1, entry, 4);
          setStep = STEP_NEW2;
          resetEntry();
          pxFlash(0,0,25,1,120);
        } else if (setStep == STEP_NEW2) {
          if (!codeMatches(entry, new1)) {
            playPattern(PATTERN_WRONG_CODE);
          } else {
            memcpy((void*)g_code, new1, 4);
            saveCode();
            pxFlash(0,25,0,3,70);
          }
          setState(ST_DISARMED);
          setStep = STEP_NONE;
          resetEntry();
        }
      } else {
        if (disarmWindowOpen() && codeMatches(entry, g_code)) {
          setState(ST_DISARMED);
          playPattern(PATTERN_DISARMED);
        } else {
          playPattern(PATTERN_WRONG_CODE);
        }
        resetEntry();
      }
    }

    if (getState() == ST_ARMED && digitalRead(GG_PIN_MPU_INT)) {
      mpuClearInt();
      MotionSummary ms = sampleMotion(CFG.investigate_ms);

      bool carry = (ms.msOverCarry >= CFG.carry_over_ms) ||
                   (ms.peakLinDevG >= CFG.carry_peak_g) ||
                   (ms.activityGms >= CFG.carry_activity_gms);

      bool warn  = (ms.msOverWarn >= CFG.warn_over_ms) ||
                   (ms.peakLinDevG >= CFG.warn_peak_g);

      if (!warn && ms.peakLinDevG >= CFG.bump_peak_g) playPattern(PATTERN_BUMP);

      if (warn && !carry) {
        setState(ST_WARNING);
        playPattern(PATTERN_WARN);
        if (CFG.warning_escalation_enabled) {
          if (millis() - lastWarnMs < CFG.warning_escalation_window_ms) carry = true;
          lastWarnMs = millis();
        }
        if (!carry) setState(ST_ARMED);
      }

      if (carry) {
        setState(ST_ALARM);
        openDisarmWindow();
        Event ev{}; ev.type = EV_CARRY_AWAY_ALERT;
        xQueueSend(g_evtQ, &ev, 0);
      }
    }

    if (getState() == ST_ARMED) {
      bool busy;
      portENTER_CRITICAL(&g_mux); busy = g_modem_busy; portEXIT_CRITICAL(&g_mux);

      if (!busy && !digitalRead(GG_PIN_MODEM_RI) && !digitalRead(GG_PIN_MPU_INT)) {
        vTaskDelay(pdMS_TO_TICKS(80));
        portENTER_CRITICAL(&g_mux); busy = g_modem_busy; portEXIT_CRITICAL(&g_mux);
        if (!busy && !digitalRead(GG_PIN_MODEM_RI) && !digitalRead(GG_PIN_MPU_INT) && getState() == ST_ARMED) {
          enterDeepSleep();
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ========================= SETUP =========================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(GG_PIN_SIREN, OUTPUT);
  digitalWrite(GG_PIN_SIREN, LOW);

  analogReadResolution(12);
  pinMode(GG_PIN_PHOTO_ADC, INPUT);

  buttonsInit();

  px.begin();
  px.setBrightness(30);
  px.show();

  loadCode();

  Wire.begin(GG_PIN_I2C_SDA, GG_PIN_I2C_SCL);
  Wire.setClock(400000);

  if (!mpuInit()) {
    for (;;) { pxSet(25,0,25); delay(200); pxSet(0,0,0); delay(200); }
  }
  // Tune motion wake sensitivity:
  // motThr 25 ~= 50mg, motDur 40ms.
  mpuEnableMotionWake(25, 40);

  setState(ST_DISARMED);

  g_evtQ = xQueueCreate(10, sizeof(Event));

  xTaskCreatePinnedToCore(TaskMotionUI, "MotionUI", 8192, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(TaskComms,    "Comms",   12288, nullptr, 2, nullptr, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
