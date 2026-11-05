// ============================================================================
//  GuardianX - Central controller + car hub   (ESP32 DevKit V1, 30-pin DOIT)
//  TEAM NEXORA - Don Bosco Institute of Technology, Mumbai
//  Community Engagement Program project
// ============================================================================
//  One program that does everything on the central board:
//   * Creates the car WiFi "GuardianX-Car"  (hub address 192.168.4.1)
//   * Crash detection with the MPU6050 (5 g held for 6 samples in 100 ms)
//   * 10-second cancel window: hold the button 1 s to cancel
//   * SMS alert via SIM800L with Google Maps link + the driver's medical info
//     (sent by its own background task with retries, so the hub never freezes)
//   * GPS location from the Neo-6M
//   * Reads ESP32-CAM 1 (192.168.4.200): who is driving + drowsiness level
//   * Drowsiness buzzer that gets more urgent (levels 1-3), button press silences it
//   * 16x2 LCD status display
//   * Web endpoints for the laptop dashboard:
//       GET  /status   everything, incl. live MPU6050 readings (ax, ay, az, gyro, temperature)
//       POST /driver   the recognised driver's profile (for the SMS)
//       POST /test     circuit-check buttons: beep, lcd, drowsy, crash, cancel,
//                      resolve, sim, sms, mpu, dashcam
//
//  Libraries (Arduino IDE -> Tools -> Manage Libraries):
//     "hd44780" by Bill Perry        "TinyGPSPlus" by Mikal Hart
//  Board: ESP32 Dev Module.   Serial Monitor: 115200 baud, line ending "Newline".
//
//  Wiring (unchanged from the GuardianX pin map):
//     MPU6050 + LCD (I2C)  SDA GPIO21, SCL GPIO22, VCC 3.3V
//     Neo-6M GPS           GPS TX -> GPIO16 (RX2), GPS RX -> GPIO17 (TX2)
//     SIM800L              SIM TX -> GPIO26 (direct), SIM RX <- GPIO27 via 1k/2.2k divider,
//                          own 4V/2A supply, grounds tied together
//     Buzzer               GPIO25 (through BC547 for a loud 5V buzzer)
//     Button               GPIO32 -> GND (internal pull-up)
//
//  Serial Monitor commands (for testing on the bench):
//     c = fake crash        x = cancel (during countdown)     r = clear crash ("I'm OK")
//     0 1 2 3 = test drowsiness level for 15 s                g = live G-force on/off
//     m = check SIM800L     t = test SMS to the backup number
//     s = status            h = help
// ============================================================================

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <math.h>

// ------------------------------------------------------------------ settings
const char* AP_SSID        = "GuardianX-Car";
const char* AP_PASS        = "nexora2026";      // at least 8 characters
const int   AP_CHANNEL     = 6;
const int   AP_MAX_CLIENTS = 6;
const char* CAM1_URL       = "http://192.168.4.200";   // driver camera (face + eyes)
const char* DASHCAM_URL    = "http://192.168.4.201";   // road camera (crash video)

// Used when no recognised driver (and so no relative's number) is known.
// Easier: set "Backup emergency number" on the dashboard's Settings page
// (it is sent to the hub automatically). Or put a number here, e.g. "+919876543210".
const char* DEFAULT_EMERGENCY_NUMBER = "";

const float         CRASH_G          = 5.0;     // crash threshold in g (was 4.0: hand tremors could reach it)

// --- false-positive filters (crash) ---
// A real crash keeps the G-force high for tens of milliseconds. A knock on the
// board, a dropped wire or an I2C glitch lasts only one or two samples.
const int           CRASH_CONFIRM_SAMPLES = 6;    // high-G samples needed (sampled every 5 ms)
const unsigned long CRASH_WINDOW_MS       = 100;  // ...within this time window
const unsigned long STARTUP_IGNORE_MS     = 5000; // ignore everything right after power-on
const float         MIN_VALID_G           = 0.2;  // below this the reading is a glitch (gravity alone is 1 g)

// --- false-positive filters (drowsiness) ---
const unsigned long DROWSY_CONFIRM_MS     = 1000; // camera level must stay this long before we act on it
const int           DROWSY_BEEP_FROM_LEVEL = 2;   // level 1 shows on the LCD only; beeping starts at this level
const unsigned long CANCEL_WINDOW_MS = 10000;   // time the driver has to cancel
const unsigned long CANCEL_HOLD_MS   = 1000;    // hold button this long to cancel
const unsigned long RESOLVE_HOLD_MS  = 3000;    // after an alert: hold this long = "I'm OK"
const unsigned long SILENCE_MS       = 30000;   // button silences drowsiness buzzer this long
const unsigned long ALERT_ALARM_MS   = 20000;   // after the alert is sent: fast alarm this long, then a short beep every 5 s

const bool BUZZER_IS_ACTIVE = true;   // true: buzzer beeps by itself when powered (most modules)
                                      // false: passive piezo (needs a tone)
const int  BUZZER_TONE_HZ   = 2700;

const double TEST_LAT = 19.072800, TEST_LON = 72.882600;   // used by 'c' when GPS has no fix

// ---------------------------------------------------------------------- pins
const int PIN_SDA = 21, PIN_SCL = 22;
const int GPS_RX = 16, GPS_TX = 17;        // ESP32 side
const int SIM_RX = 26, SIM_TX = 27;        // ESP32 side
const int PIN_BUZZER = 25;
const int PIN_BUTTON = 32;
const int PIN_LED = 2;                      // blue LED: steady = WiFi up, blinking = crash countdown
const uint8_t MPU_ADDR = 0x68;

const IPAddress HUB_IP(192, 168, 4, 1);
const IPAddress SUBNET(255, 255, 255, 0);

// ---------------------------------------------------------------- objects
WebServer server(80);
TinyGPSPlus gps;
hd44780_I2Cexp lcd;
#define GPSSerial Serial2
#define SIM Serial1

// ------------------------------------------------------------------ state
struct Driver {
  int faceId = 0;
  String name, age, blood, car, medical, relName, relPhone, sms;
} driver;
String backupPhone = "";                 // from the dashboard Settings page

// sensors
bool  mpuOk = false, lcdOk = false;
volatile bool simOk = false;            // written by the SIM task
float gNow = 1.0;
bool  liveG = false;
float axG = 0, ayG = 0, azG = 1, gxDps = 0, gyDps = 0, gzDps = 0, mpuTempC = 0;
float gPeak = 0;                         // highest g since the dashboard last asked
uint8_t lcdAddr = 0;                     // I2C address the LCD was found at
String simCsq = "", simReg = "";         // last signal / network check

// MPU history for the dashboard graph: one sample every 50 ms
const int MPU_HIST = 16;
struct MpuSample { int16_t ax, ay, az; };   // centi-g
MpuSample mpuHist[MPU_HIST];
uint32_t  mpuSeq = 0;                    // number of samples taken so far

// dashboard test helpers
unsigned long testBeepUntil = 0, lcdMsgUntil = 0;
String lcdMsg1, lcdMsg2;

// crash
enum CrashState { NORMAL, COUNTDOWN, ALERTED };
CrashState crashState = NORMAL;
unsigned long countdownStart = 0, alertedAt = 0;
int    crashId = 0;
float  crashG = 0;
double crashLat = 0, crashLon = 0;
bool   crashHasGps = false;
String smsStatus = "none";

// camera (written by the camera task on core 0)
volatile int  camDriverId = 0, camLevel = 0;
volatile bool dashcamOnline = false;
TaskHandle_t  dashcamTaskHandle = nullptr;
volatile int  dashcamTriggerId = 0;
volatile bool camOnline = false;

// drowsiness test override + silencing
int testLevel = -1;
unsigned long testLevelUntil = 0;
int silencedLevel = 0;
unsigned long silencedUntil = 0;

// buzzer
bool buzzerOn = false;

// SIM800L job queue types. They must stay ABOVE the first function: the
// Arduino IDE puts its automatic function declarations right before the
// first function, and tJob(const SimJob&) needs SimJob to exist there.
enum SimJobKind : uint8_t { JOB_CHECK = 1, JOB_SMS_TEST = 2, JOB_SMS_ALERT = 3 };
struct SimJob {
  uint8_t  kind;
  uint16_t id;
  uint8_t  part, parts;          // alert messages: part 1 of 2, 2 of 2
  char     number[20];
  char     text[170];
};

// ================================================================ helpers
String jsonEscape(const String& s) {
  String o;
  for (size_t i = 0; i < s.length(); i++) {
    char ch = s[i];
    if (ch == '"' || ch == '\\') { o += '\\'; o += ch; }
    else if (ch == '\n') o += "\\n";
    else if ((uint8_t)ch >= 0x20) o += ch;
  }
  return o;
}

int jsonInt(const String& s, const char* key, int def) {
  String k = String("\"") + key + "\":";
  int i = s.indexOf(k);
  return i < 0 ? def : s.substring(i + k.length()).toInt();
}

bool gpsFix() {
  return gps.location.isValid() && gps.location.age() < 5000;
}

// Camera drowsiness level, confirmed: a level only counts once it has been
// reported for DROWSY_CONFIRM_MS. Going back down counts immediately.
int drowsyLevel() {
  if (testLevel >= 0 && millis() < testLevelUntil) return testLevel;
  testLevel = -1;
  static int confirmed = 0, candidate = 0;
  static unsigned long candidateSince = 0;
  int raw = camOnline ? camLevel : 0;
  unsigned long now = millis();
  if (raw < confirmed) { confirmed = raw; candidate = raw; candidateSince = now; }
  else if (raw != candidate) { candidate = raw; candidateSince = now; }
  else if (candidate > confirmed && now - candidateSince >= DROWSY_CONFIRM_MS) confirmed = candidate;
  return confirmed;
}

// ================================================================== MPU6050
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;
  mpuWrite(0x6B, 0x00);   // wake up
  mpuWrite(0x1C, 0x18);   // accelerometer range +-16 g (so a 4 g crash isn't clipped)
  mpuWrite(0x1B, 0x08);   // gyroscope range +-500 deg/s
  mpuWrite(0x1A, 0x02);   // low-pass filter ~94 Hz
  return true;
}

// Reads acceleration (g), rotation (deg/s) and temperature in one go.
bool mpuReadG(float& g) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) return false;
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  int16_t tr = (Wire.read() << 8) | Wire.read();
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();
  if (ax == ay && ay == az) return false;   // all axes identical = corrupted I2C read (e.g. 0x0000 / 0xFFFF)
  const float s = 2048.0;   // LSB per g at +-16 g
  axG = ax / s; ayG = ay / s; azG = az / s;
  gxDps = gx / 65.5f; gyDps = gy / 65.5f; gzDps = gz / 65.5f;   // LSB per deg/s at +-500
  mpuTempC = tr / 340.0f + 36.53f;
  g = sqrtf(axG * axG + ayG * ayG + azG * azG);
  return true;
}

void mpuRecordSample() {
  MpuSample& m = mpuHist[mpuSeq % MPU_HIST];
  m.ax = (int16_t)constrain(lroundf(axG * 100), -3000, 3000);
  m.ay = (int16_t)constrain(lroundf(ayG * 100), -3000, 3000);
  m.az = (int16_t)constrain(lroundf(azG * 100), -3000, 3000);
  mpuSeq++;
}

// The LCD backpack sits at 0x20-0x27 or 0x38-0x3F. Returns 0 if none answers.
uint8_t findLcdAddr() {
  const uint8_t ranges[2][2] = {{0x20, 0x27}, {0x38, 0x3F}};
  for (auto& r : ranges)
    for (uint8_t a = r[0]; a <= r[1]; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) return a;
    }
  return 0;
}

// ================================================================== SIM800L
// Everything that talks to the SIM800L runs in its own task (simTask, core 0).
// The web server and the crash logic only drop a job into a queue and return
// at once. Sending an SMS can take up to a minute; before this change the hub
// handled other web requests from inside the SMS wait, which broke the reply
// to the dashboard ("SIM800L didn't answer / hub offline") even when the hub
// was fine.
// (SimJob and SimJobKind are defined near the top, before the first function)
QueueHandle_t     simQueue = nullptr;
SemaphoreHandle_t simLock  = nullptr;   // guards the shared SIM strings below

// shared with the web server and the LCD (read/write only under simLock)
int      simCsqVal = 99;                // 0-31, 99 = unknown
int      simRegStat = -1;               // 1 / 5 registered, 2 searching, 3 denied, 0 not searching
int      simVoltMv = 0;
String   simOperator, simSmsc, simPin, simProblem;
uint16_t simJobId = 0;
String   simJobKind = "", simJobState = "idle", simJobDetail = "";
String   smsDetail = "";
volatile bool simBusy = false;
uint16_t simNextJob = 0;                // only used on core 1 (loop + web server)
bool     alertPartOk[3] = {false, false, false};

void simSet(String& field, const String& v) {
  xSemaphoreTake(simLock, portMAX_DELAY); field = v; xSemaphoreGive(simLock);
}
String simGet(const String& field) {
  xSemaphoreTake(simLock, portMAX_DELAY); String c = field; xSemaphoreGive(simLock); return c;
}

// "98765 43210", "09876543210", "919876543210", "+91 98765-43210" -> "+919876543210"
String normalizePhone(const String& in) {
  String d;
  bool plus = false;
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c >= '0' && c <= '9') d += c;
    else if (c == '+' && d.length() == 0) plus = true;
  }
  if (d.length() == 10 && d[0] >= '6') return "+91" + d;
  if (d.length() == 11 && d[0] == '0') return "+91" + d.substring(1);
  if (d.length() == 12 && d.startsWith("91")) return "+" + d;
  if (plus && d.length() >= 8 && d.length() <= 15) return "+" + d;
  return "";
}

// SMS text the SIM800L accepts in text mode: plain characters, at most 160.
String smsText(const String& in) {
  String o;
  for (size_t i = 0; i < in.length() && o.length() < 160; i++) {
    char c = in[i];
    if (c == '\n' || c == '\r' || c == '\t') c = ' ';
    if (c < 32 || c > 126) continue;
    if (c == '[' || c == '{') c = '(';
    else if (c == ']' || c == '}') c = ')';
    else if (c == '~' || c == '^' || c == '|' || c == '\\' || c == '`') c = '-';
    o += c;
  }
  return o;
}

String regText(int s) {
  switch (s) {
    case 1: return "registered";
    case 5: return "registered (roaming)";
    case 2: return "searching for network";
    case 3: return "network REJECTED the SIM";
    case 0: return "not searching";
    default: return "unknown";
  }
}

String oneLine(String s) {
  s.replace("\r", " "); s.replace("\n", " ");
  while (s.indexOf("  ") >= 0) s.replace("  ", " ");
  s.trim();
  return s.length() > 80 ? s.substring(0, 80) : s;
}

// Plain-English meaning of the SIM800L's "+CMS ERROR: n" when an SMS fails.
String cmsExplain(const String& r) {
  int k = r.indexOf("+CMS ERROR:");
  if (k < 0) k = r.indexOf("+CME ERROR:");
  if (k < 0) return "";
  int code = r.substring(k + 11).toInt();
  String c = " (error " + String(code) + ")";
  switch (code) {
    case 8: case 38: case 41: case 42: case 47: case 331: case 332:
      return "mobile network busy or no service" + c + ". Try again, check the signal";
    case 21:
      return "network rejected the SMS" + c + ": usually no balance / no SMS pack, or the SIM is barred from SMS";
    case 27: case 28: case 111:
      return "that number doesn't exist or is out of service" + c;
    case 50:
      return "this SIM has no SMS service" + c + ". Recharge it with an SMS pack / validity plan";
    case 96: case 304: case 305:
      return "the number or text was not accepted" + c;
    case 302: case 303:
      return "SIM800L not ready to send" + c + ": not registered on the network yet";
    case 310: case 311: case 313:
      return "SIM card problem" + c + ": missing, PIN-locked or damaged";
    case 330:
      return "SMS centre number missing on the SIM" + c + ". Put the SIM in a phone and send one SMS from it";
    case 500:
      return "network refused the SMS" + c + ": most often no balance / no SMS pack, or the SIM isn't active for SMS";
    default:
      return "network error" + c;
  }
}

// ---- the functions below are used ONLY inside simTask -------------------
static bool tSawRestart = false, tLowVolt = false;

static void tScan(const String& s) {
  if (s.indexOf("RDY") >= 0 || s.indexOf("Call Ready") >= 0 || s.indexOf("NORMAL POWER DOWN") >= 0) tSawRestart = true;
  if (s.indexOf("UNDER-VOLTAGE") >= 0) tLowVolt = true;
}

static String tRead(unsigned long ms, const char* e1, const char* e2 = "ERROR", const char* e3 = nullptr) {
  String buf;
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    while (SIM.available()) buf += (char)SIM.read();
    if (buf.indexOf(e1) >= 0 || (e2 && buf.indexOf(e2) >= 0) || (e3 && buf.indexOf(e3) >= 0)) break;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  vTaskDelay(pdMS_TO_TICKS(20));
  while (SIM.available()) buf += (char)SIM.read();
  tScan(buf);
  return buf;
}

static String tCmd(const String& c, unsigned long ms = 2000) {
  String junk;
  while (SIM.available()) junk += (char)SIM.read();
  tScan(junk);
  SIM.println(c);
  return tRead(ms, "OK");
}

static bool tAlive() {
  for (int i = 0; i < 3; i++) {
    if (tCmd("AT", 800).indexOf("OK") >= 0) return true;
    vTaskDelay(pdMS_TO_TICKS(300));
  }
  return false;
}

// Text mode, plain character set, numeric error codes. Must be repeated after
// every restart of the module (it forgets these settings).
static bool tSetup() {
  if (!tAlive()) return false;
  tCmd("ATE0");
  tCmd("AT+CMEE=1");
  tCmd("AT+CMGF=1");
  tCmd("AT+CSCS=\"GSM\"");
  tCmd("AT+CNMI=0,0,0,0,0");     // don't push incoming SMS to the ESP32
  return true;
}

static int tRegStat() {
  String r = tCmd("AT+CREG?");
  int k = r.indexOf("+CREG:");
  if (k < 0) return -1;
  int c = r.indexOf(',', k);
  return c > 0 ? r.substring(c + 1).toInt() : -1;
}

static String quoted(const String& r) {
  int k = r.indexOf('"');
  if (k < 0) return "";
  int e = r.indexOf('"', k + 1);
  return e > k ? r.substring(k + 1, e) : "";
}

// Signal, network, operator, supply voltage, SIM card, SMS centre.
static void tRefresh() {
  String r = tCmd("AT+CPIN?");
  String pin = r.indexOf("READY") >= 0 ? "ready" : r.indexOf("SIM PIN") >= 0 ? "PIN locked"
             : r.indexOf("ERROR") >= 0 ? "no SIM card" : "unknown";
  r = tCmd("AT+CSQ");
  int csq = 99, k = r.indexOf("+CSQ:");
  if (k >= 0) csq = r.substring(k + 5).toInt();
  int reg = tRegStat();
  String op = quoted(tCmd("AT+COPS?", 4000));
  r = tCmd("AT+CBC");
  int mv = 0; k = r.indexOf("+CBC:");
  if (k >= 0) { int c = r.lastIndexOf(','); if (c > k) mv = r.substring(c + 1).toInt(); }
  String smsc = quoted(tCmd("AT+CSCA?"));

  String problem;
  if (pin != "ready")                problem = pin == "no SIM card" ? String("No SIM card detected (micro-SIM, contacts down, pushed fully in)")
                                                                   : String("SIM card is ") + pin + ". Remove the PIN in a phone first";
  else if (reg == 3)                 problem = "Network rejected the SIM. Jio has no 2G: use an Airtel, Vi or BSNL SIM";
  else if (csq == 99 || csq < 8)     problem = "Signal too weak (" + String(csq) + "/31). Fix the antenna, move near a window";
  else if (reg != 1 && reg != 5)     problem = "Not registered on the network yet (" + regText(reg) + ")";
  else if (mv > 0 && mv < 3500)      problem = "Supply too low (" + String(mv / 1000.0, 2) + " V). It needs 3.7-4.2 V that can give 2 A";
  else if (tLowVolt)                 problem = "SIM800L reported UNDER-VOLTAGE. Its supply sags: use a 2 A supply, thick short wires, 1000 uF capacitor";
  else if (!smsc.length())           problem = "SMS centre number missing on the SIM. Put the SIM in a phone and send one SMS from it";

  xSemaphoreTake(simLock, portMAX_DELAY);
  simCsqVal = csq; simRegStat = reg; simOperator = op; simVoltMv = mv; simSmsc = smsc; simPin = pin;
  simProblem = problem;
  simCsq = "+CSQ: " + String(csq);
  simReg = regText(reg);
  xSemaphoreGive(simLock);
}

static String tSummary() {
  xSemaphoreTake(simLock, portMAX_DELAY);
  String s = "Signal " + String(simCsqVal) + "/31, " + regText(simRegStat);
  if (simOperator.length()) s += " on " + simOperator;
  if (simVoltMv) s += ", supply " + String(simVoltMv / 1000.0, 2) + " V";
  if (simProblem.length()) s += ". PROBLEM: " + simProblem;
  else s += ". Ready to send SMS";
  xSemaphoreGive(simLock);
  return s;
}

// One attempt. Returns true when the network confirms the SMS (+CMGS).
static bool tSendOnce(const String& num, const String& text, String& why) {
  tSawRestart = false;
  if (!tSetup()) { why = "SIM800L not answering (4 V / 2 A supply, TX->GPIO26, RX<-GPIO27 via divider, common GND)"; return false; }
  simOk = true;

  int reg = -1;
  for (int i = 0; i < 10; i++) {               // wait up to ~30 s for the network
    reg = tRegStat();
    if (reg == 1 || reg == 5) break;
    if (reg == 3) { why = "network rejected the SIM. Jio has no 2G: use Airtel, Vi or BSNL"; return false; }
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
  if (reg != 1 && reg != 5) { why = "not registered on the mobile network (" + regText(reg) + "). Check antenna and signal"; return false; }

  while (SIM.available()) SIM.read();
  SIM.print("AT+CMGS=\"");
  SIM.print(num);
  SIM.print("\"\r");
  String r = tRead(8000, ">");
  if (r.indexOf('>') < 0) {
    SIM.write(27);                               // ESC: leave the SMS prompt
    String e = cmsExplain(r);
    if (e.length())                  why = e;
    else if (r.indexOf("ERROR") >= 0) why = String("SIM800L refused the number ") + num;
    else                             why = "no '>' prompt from the SIM800L";
    return false;
  }
  SIM.print(text);
  SIM.write(26);                                 // Ctrl+Z = send
  r = tRead(60000, "+CMGS", "ERROR", "RDY");
  if (r.indexOf("+CMGS") >= 0) return true;

  String e = cmsExplain(r);
  if (e.length()) { why = e; return false; }

  // No confirmation. Did the module restart while transmitting (power dip)?
  SIM.write(27);
  bool restarted = tSawRestart;
  if (!restarted && tAlive()) {
    String m = tCmd("AT+CMGF?");
    restarted = m.indexOf("+CMGF: 0") >= 0;      // text mode lost = the module rebooted
  }
  if (restarted || tLowVolt)
    why = "SIM800L restarted while transmitting: its power supply dips when it sends. "
          "Use 3.7-4.2 V that can give 2 A, short thick wires, 1000 uF capacitor at VCC-GND";
  else if (!r.length())
    why = "no reply from the network within 60 s (weak signal?)";
  else
    why = "failed: " + oneLine(r);
  return false;
}

static bool tSend(const String& num, const String& text, int attempts, String& why) {
  for (int a = 1; a <= attempts; a++) {
    Serial.printf("[SIM] Sending SMS to %s (attempt %d/%d): %s\n", num.c_str(), a, attempts, text.c_str());
    if (tSendOnce(num, text, why)) { Serial.println("[SIM] SMS sent"); return true; }
    Serial.println("[SIM] SMS failed: " + why);
    if (a < attempts) vTaskDelay(pdMS_TO_TICKS(5000));
  }
  return false;
}

static void tJob(const SimJob& j) {
  String num = j.number, text = j.text, why;
  if (j.kind == JOB_CHECK) {
    xSemaphoreTake(simLock, portMAX_DELAY);
    simJobId = j.id; simJobKind = "check"; simJobState = "running"; simJobDetail = "Checking the SIM800L...";
    xSemaphoreGive(simLock);
    bool ok = tSetup();
    simOk = ok;
    String d;
    if (ok) { tRefresh(); d = tSummary(); }
    else d = "SIM800L not answering. Check its 4 V / 2 A supply, TX->GPIO26, RX<-GPIO27 (divider), common ground";
    Serial.println("[SIM] " + d);
    xSemaphoreTake(simLock, portMAX_DELAY);
    simJobState = ok ? "done" : "failed"; simJobDetail = d;
    xSemaphoreGive(simLock);
  } else if (j.kind == JOB_SMS_TEST) {
    xSemaphoreTake(simLock, portMAX_DELAY);
    simJobId = j.id; simJobKind = "sms"; simJobState = "running"; simJobDetail = "Sending test SMS to " + num + "...";
    xSemaphoreGive(simLock);
    bool ok = tSend(num, text, 1, why);   // one try: quick, clear answer for the test button
    tRefresh();
    xSemaphoreTake(simLock, portMAX_DELAY);
    simJobState = ok ? "done" : "failed";
    simJobDetail = ok ? String("Test SMS sent to ") + num + ". Check that phone."
                      : String("SMS to ") + num + " failed: " + why;
    xSemaphoreGive(simLock);
  } else if (j.kind == JOB_SMS_ALERT) {
    if (j.part == 1) { alertPartOk[1] = alertPartOk[2] = false; simSet(smsStatus, "sending"); }
    bool ok = tSend(num, text, 3, why);
    alertPartOk[j.part] = ok;
    simSet(smsDetail, ok ? String("Sent to ") + num : why);
    if (j.part >= j.parts) {
      bool all = alertPartOk[1] && (j.parts < 2 || alertPartOk[2]);
      String st = all ? "sent" : alertPartOk[1] ? "partly sent" : (!simOk ? "no SIM800L" : "failed");
      simSet(smsStatus, st);
      Serial.println(String("[CRASH] SMS alert: ") + st + (all ? String("") : String(" - ") + why));
    }
  }
}

void simTask(void*) {
  vTaskDelay(pdMS_TO_TICKS(2500));               // the SIM800L needs a few seconds after power-on
  unsigned long lastHealth = 0;
  bool first = true;
  for (;;) {
    SimJob j;
    if (xQueueReceive(simQueue, &j, pdMS_TO_TICKS(500)) == pdTRUE) {
      simBusy = true;
      tJob(j);
      simBusy = false;
      lastHealth = millis();
      continue;
    }
    // health check: every 30 s when OK, every 10 s while it isn't answering
    if (first || millis() - lastHealth > (simOk ? 30000UL : 10000UL)) {
      lastHealth = millis();
      bool was = simOk;
      bool alive = tAlive();
      if (alive && (!was || tSawRestart || first)) { tSetup(); tSawRestart = false; }
      simOk = alive;
      if (alive) tRefresh();
      if (alive != was || first) {
        if (alive) Serial.println(String("[SIM] SIM800L OK. ") + tSummary());
        else       Serial.println("[SIM] SIM800L not answering - check its 4 V / 2 A supply, TX/RX wires and divider");
      }
      first = false;
    }
  }
}

// Called from the loop / web server (core 1). Returns the job number, 0 if the queue is full.
uint16_t simEnqueue(uint8_t kind, const String& num, const String& text, bool urgent = false,
                    uint8_t part = 0, uint8_t parts = 0) {
  SimJob j;
  memset(&j, 0, sizeof(j));
  j.kind = kind; j.id = ++simNextJob; j.part = part; j.parts = parts;
  strlcpy(j.number, num.c_str(), sizeof(j.number));
  strlcpy(j.text, text.c_str(), sizeof(j.text));
  BaseType_t ok = urgent ? xQueueSendToFront(simQueue, &j, 0) : xQueueSendToBack(simQueue, &j, 0);
  if (ok != pdTRUE) return 0;
  if (kind != JOB_SMS_ALERT) {
    xSemaphoreTake(simLock, portMAX_DELAY);
    simJobId = j.id; simJobKind = kind == JOB_CHECK ? "check" : "sms"; simJobState = "waiting";
    simJobDetail = simBusy ? "Waiting: the SIM800L is busy with another message" : "Starting...";
    xSemaphoreGive(simLock);
  }
  return j.id;
}

// ===================================================================== LCD
String lastL1, lastL2;

void lcdShow(String l1, String l2) {
  if (!lcdOk) return;
  while (l1.length() < 16) l1 += ' ';
  while (l2.length() < 16) l2 += ' ';
  l1 = l1.substring(0, 16);
  l2 = l2.substring(0, 16);
  if (l1 != lastL1) { lcd.setCursor(0, 0); lcd.print(l1); lastL1 = l1; }
  if (l2 != lastL2) { lcd.setCursor(0, 1); lcd.print(l2); lastL2 = l2; }
}

// Buzzer current spikes can knock the LCD out of sync (it then shows garbage).
// Re-initialising it often while the buzzer is active cleans that up by itself.
void lcdReinit() {
  if (!lcdOk) return;
  lcd.begin(16, 2);
  lcd.backlight();
  lastL1 = "";
  lastL2 = "";
}

void updateLcd() {
  if (crashState == NORMAL && millis() < lcdMsgUntil) {   // test message from the dashboard
    lcdShow(lcdMsg1, lcdMsg2);
    return;
  }
  if (crashState == COUNTDOWN) {
    // line 1: seconds left. line 2: alternates between the instruction and a shrinking bar
    unsigned long el = millis() - countdownStart;
    long left = (long)(CANCEL_WINDOW_MS - el) / 1000 + 1;
    if (left < 0) left = 0;
    char l1[17];
    snprintf(l1, sizeof(l1), "CRASH! SMS in%2lds", left);
    String l2;
    if ((el / 1000) % 2 == 0) {
      l2 = "Hold btn=cancel";
    } else {
      int n = (int)((CANCEL_WINDOW_MS - el) * 16 / CANCEL_WINDOW_MS);
      for (int i = 0; i < 16; i++) l2 += (i < n) ? char(255) : ' ';   // char 255 = solid block
    }
    lcdShow(l1, l2);
  } else if (crashState == ALERTED) {
    String st = simGet(smsStatus);
    String l1 = st == "sending"     ? "SENDING ALERT..."
              : st == "sent"        ? "ALERT SMS SENT"
              : st == "no number"   ? "ALERT: NO NUMBER"
              : st == "no SIM800L"  ? "ALERT: NO SIM800"
              : st == "partly sent" ? "ALERT: SMS PART"
              :                       "ALERT: SMS FAIL";
    lcdShow(l1, "Hold 3s=I'm OK");
  } else {
    String who;
    if (driver.name.length())              who = driver.name;
    else if (camOnline && camDriverId > 0) who = "Driver ID " + String((int)camDriverId);
    else if (camOnline)                    who = "No driver";
    else                                   who = "Camera offline";
    int lvl = drowsyLevel();
    String eyes = (lvl == 0) ? String("Eyes OK") : String("WAKE UP ") + String(lvl);
    lcdShow(who, eyes + (gpsFix() ? "  GPS" : "  ---"));
  }
}

// ================================================================== buzzer
bool buzzerPattern(unsigned long now) {
  if (now < testBeepUntil) return true;                           // "Beep" button on the dashboard
  if (crashState == COUNTDOWN) return (now % 500) < 300;          // urgent
  if (crashState == ALERTED) {
    if (now - alertedAt < ALERT_ALARM_MS) return (now % 200) < 100;  // fast alarm first
    return (now % 5000) < 100;                                      // then a short reminder beep
  }
  int lvl = drowsyLevel();
  if (testLevel < 0 && lvl < DROWSY_BEEP_FROM_LEVEL) return false;  // too mild to beep (LCD still shows it)
  if (lvl == 0) return false;
  if (lvl <= silencedLevel && now < silencedUntil) return false;  // silenced by the button
  if (lvl == 1) return (now % 2000) < 150;
  if (lvl == 2) return (now % 1000) < 300;
  return (now % 300) < 200;                                       // level 3
}

void buzzerOff() {
  buzzerOn = false;
  if (BUZZER_IS_ACTIVE) digitalWrite(PIN_BUZZER, LOW);
  else noTone(PIN_BUZZER);
}

void buzzerUpdate() {
  bool on = buzzerPattern(millis());
  if (on == buzzerOn) return;
  buzzerOn = on;
  if (BUZZER_IS_ACTIVE) digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
  else if (on) tone(PIN_BUZZER, BUZZER_TONE_HZ);
  else noTone(PIN_BUZZER);
}

// =================================================================== crash
void startCountdown(float g, bool test) {
  if (crashState != NORMAL) return;
  crashState = COUNTDOWN;
  countdownStart = millis();
  crashG = g;
  if (gpsFix()) {
    crashLat = gps.location.lat();
    crashLon = gps.location.lng();
    crashHasGps = true;
  } else if (test) {
    crashLat = TEST_LAT; crashLon = TEST_LON; crashHasGps = true;
  } else {
    crashHasGps = false;
  }
  Serial.printf("[CRASH] %.1f g detected%s! Alert in %lu s unless the button is held.\n",
                g, test ? " (test)" : "", CANCEL_WINDOW_MS / 1000);
  dashcamTriggerId = crashId + 1;
  if (dashcamTaskHandle) xTaskNotifyGive(dashcamTaskHandle);   // dashcam saves 5 s before + 7 s after
}

void cancelCountdown() {
  if (crashState != COUNTDOWN) return;
  crashState = NORMAL;
  Serial.println("[CRASH] Cancelled by the driver. No alert sent.");
  buzzerOff();
  lcdShow("Alert cancelled", "Drive safe");
  delay(1200);
}

void sendAlerts() {
  crashState = ALERTED;
  alertedAt = millis();
  crashId++;                                   // the dashboard sees this and logs the crash
  if (gpsFix()) {                              // use the freshest location
    crashLat = gps.location.lat();
    crashLon = gps.location.lng();
    crashHasGps = true;
  }
  Serial.printf("[CRASH] No cancel - ALERT #%d\n", crashId);
  buzzerOff();
  simSet(smsStatus, "sending");
  simSet(smsDetail, "");
  updateLcd();

  // relative's number -> backup number from the dashboard -> number in this sketch
  String number = normalizePhone(driver.relPhone);
  if (!number.length()) number = normalizePhone(backupPhone);
  if (!number.length()) number = normalizePhone(DEFAULT_EMERGENCY_NUMBER);
  if (!number.length()) {
    simSet(smsStatus, "no number");
    simSet(smsDetail, "No emergency number: no recognised driver and no backup number set");
    Serial.println("[CRASH] No emergency number known (no recognised driver, no backup number).");
    return;
  }
  String who = driver.name.length() ? driver.name.substring(0, 24) : String("The driver");
  String loc = crashHasGps
      ? "https://maps.google.com/?q=" + String(crashLat, 6) + "," + String(crashLon, 6)
      : String("GPS location not available");
  String sms1 = smsText("GuardianX CRASH ALERT: " + who + " may need help. " + loc);
  bool two = driver.sms.length() > 0;
  // Sent by the SIM task, so the hub keeps answering the dashboard meanwhile.
  // Crash alerts jump ahead of any test message waiting in the queue.
  if (two) simEnqueue(JOB_SMS_ALERT, number, smsText("GuardianX medical info: " + driver.sms), true, 2, 2);
  if (!simEnqueue(JOB_SMS_ALERT, number, sms1, true, 1, two ? 2 : 1)) {
    simSet(smsStatus, "failed");
    simSet(smsDetail, "SMS queue full");
  }
}

void resolveCrash() {
  crashState = NORMAL;
  Serial.println("[CRASH] Cleared (driver is OK).");
  buzzerOff();
  lcdShow("Driver is OK", "Alert cleared");
  delay(1200);
}

// ================================================================== button
void handleButton() {
  static bool wasDown = false, handled = false;
  static unsigned long downSince = 0;
  bool down = digitalRead(PIN_BUTTON) == LOW;
  unsigned long now = millis();

  if (down && !wasDown) { downSince = now; handled = false; }
  if (down && !handled) {
    unsigned long held = now - downSince;
    if (crashState == COUNTDOWN && held >= CANCEL_HOLD_MS) { cancelCountdown(); handled = true; }
    else if (crashState == ALERTED && held >= RESOLVE_HOLD_MS) { resolveCrash(); handled = true; }
  }
  if (!down && wasDown && !handled && now - downSince >= 40 && crashState == NORMAL) {
    int lvl = drowsyLevel();
    if (lvl > 0) {
      silencedLevel = lvl;
      silencedUntil = now + SILENCE_MS;
      Serial.printf("[DROWSY] Buzzer silenced for %lu s (it sounds again if it gets worse)\n", SILENCE_MS / 1000);
    }
  }
  wasDown = down;
}

// ========================================================= camera (core 0)
void cameraTask(void*) {
  int lastId = -1, lastLevel = -1;
  bool lastOnline = false;
  unsigned long lastOk = 0;
  for (;;) {
    HTTPClient http;
    http.setConnectTimeout(400);
    http.setTimeout(800);
    if (http.begin(String(CAM1_URL) + "/status")) {
      if (http.GET() == 200) {
        String body = http.getString();
        camDriverId = jsonInt(body, "driver_id", 0);
        camLevel = constrain(jsonInt(body, "level", 0), 0, 3);
        camOnline = true;
        lastOk = millis();
      }
      http.end();
    }
    if (millis() - lastOk > 5000) camOnline = false;

    if (camOnline != lastOnline) {
      Serial.println(camOnline ? "[CAM] Camera 1 connected" : "[CAM] Camera 1 not answering");
      lastOnline = camOnline;
    }
    if (camOnline && camDriverId != lastId) {
      if (camDriverId > 0) Serial.printf("[CAM] Driver with camera ID %d at the wheel\n", (int)camDriverId);
      else Serial.println("[CAM] No recognised driver");
      lastId = camDriverId;
    }
    if (camOnline && camLevel != lastLevel) {
      Serial.printf("[CAM] Drowsiness level %d\n", (int)camLevel);
      lastLevel = camLevel;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ========================================================= dashcam (core 0)
// Tells the dashcam to save its clip the moment a crash is detected, and
// checks every few seconds that it is reachable.
void dashcamTask(void*) {
  bool lastOnline = false;
  for (;;) {
    bool trigger = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(3000)) > 0;
    String path = trigger ? "/trigger?id=" + String((int)dashcamTriggerId) : String("/status");
    bool ok = false;
    for (int attempt = 0; attempt < (trigger ? 3 : 1) && !ok; attempt++) {
      HTTPClient http;
      http.setConnectTimeout(400);
      http.setTimeout(800);
      if (http.begin(String(DASHCAM_URL) + path)) {
        ok = http.GET() == 200;
        http.end();
      }
    }
    dashcamOnline = ok;
    if (trigger) Serial.println(ok ? "[DASHCAM] Crash clip recording started"
                                   : "[DASHCAM] Could not reach the dashcam - no video for this crash");
    if (dashcamOnline != lastOnline) {
      Serial.println(dashcamOnline ? "[DASHCAM] Dashcam connected" : "[DASHCAM] Dashcam not answering");
      lastOnline = dashcamOnline;
    }
  }
}

// ======================================================== web endpoints
void handleStatus() {
  long left = crashState == COUNTDOWN
      ? (long)(CANCEL_WINDOW_MS - (millis() - countdownStart)) / 1000 + 1 : 0;
  String j = "{";
  j += "\"crash\":" + String(crashState == ALERTED ? "true" : "false");
  j += ",\"cancelled\":false";
  j += ",\"crash_pending\":" + String(crashState == COUNTDOWN ? "true" : "false");
  j += ",\"seconds_left\":" + String(left);
  j += ",\"crash_id\":" + String(crashId);
  j += ",\"g\":" + String(crashG, 2);
  if (crashHasGps) j += ",\"lat\":" + String(crashLat, 6) + ",\"lon\":" + String(crashLon, 6);
  else j += ",\"lat\":null,\"lon\":null";
  j += ",\"sms\":\"" + jsonEscape(simGet(smsStatus)) + "\"";
  j += ",\"sms_detail\":\"" + jsonEscape(simGet(smsDetail)) + "\"";
  j += ",\"g_now\":" + String(gNow, 2);
  j += ",\"gps_fix\":" + String(gpsFix() ? "true" : "false");
  j += ",\"mpu_ok\":" + String(mpuOk ? "true" : "false");
  j += ",\"sim_ok\":" + String(simOk ? "true" : "false");
  j += ",\"cam_online\":" + String(camOnline ? "true" : "false");
  j += ",\"dashcam_online\":" + String(dashcamOnline ? "true" : "false");
  j += ",\"cam_driver_id\":" + String((int)camDriverId);
  j += ",\"drowsy_level\":" + String(drowsyLevel());
  j += ",\"driver_face_id\":" + String(driver.faceId);
  j += ",\"driver_name\":\"" + jsonEscape(driver.name) + "\"";
  j += ",\"clients\":" + String(WiFi.softAPgetStationNum());
  j += ",\"uptime_s\":" + String(millis() / 1000);

  // ---- circuit check details ----
  j += ",\"lcd_ok\":" + String(lcdOk ? "true" : "false");
  j += ",\"lcd_addr\":" + String(lcdAddr);
  j += ",\"button_down\":" + String(digitalRead(PIN_BUTTON) == LOW ? "true" : "false");
  j += ",\"buzzer_on\":" + String(buzzerOn ? "true" : "false");
  j += ",\"gps_chars\":" + String((unsigned long)gps.charsProcessed());
  j += ",\"gps_sats\":" + String(gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  if (gpsFix()) {
    j += ",\"gps_lat\":" + String(gps.location.lat(), 6) + ",\"gps_lon\":" + String(gps.location.lng(), 6);
    j += ",\"gps_kmph\":" + String(gps.speed.isValid() ? gps.speed.kmph() : 0.0, 1);
  } else {
    j += ",\"gps_lat\":null,\"gps_lon\":null,\"gps_kmph\":null";
  }
  xSemaphoreTake(simLock, portMAX_DELAY);
  j += ",\"sim_csq\":\"" + jsonEscape(simCsq) + "\",\"sim_reg\":\"" + jsonEscape(simReg) + "\"";
  j += ",\"sim\":{\"ok\":" + String(simOk ? "true" : "false") + ",\"busy\":" + String(simBusy ? "true" : "false");
  j += ",\"csq\":" + String(simCsqVal) + ",\"reg\":\"" + regText(simRegStat) + "\"";
  j += ",\"registered\":" + String(simRegStat == 1 || simRegStat == 5 ? "true" : "false");
  j += ",\"operator\":\"" + jsonEscape(simOperator) + "\",\"volt\":" + String(simVoltMv / 1000.0, 2);
  j += ",\"sim_card\":\"" + jsonEscape(simPin) + "\",\"smsc\":\"" + jsonEscape(simSmsc) + "\"";
  j += ",\"problem\":\"" + jsonEscape(simProblem) + "\"}";
  j += ",\"sim_job\":{\"id\":" + String(simJobId) + ",\"kind\":\"" + simJobKind + "\",\"state\":\"" + simJobState + "\"";
  j += ",\"detail\":\"" + jsonEscape(simJobDetail) + "\"}";
  xSemaphoreGive(simLock);
  j += ",\"backup_phone\":\"" + jsonEscape(backupPhone) + "\"";
  j += ",\"crash_g\":" + String(CRASH_G, 1);
  j += ",\"cancel_s\":" + String(CANCEL_WINDOW_MS / 1000);
  j += ",\"cam_level\":" + String((int)camLevel);

  // ---- live MPU6050 (g = 9.81 m/s2) ----
  j += ",\"mpu\":{\"ax\":" + String(axG, 2) + ",\"ay\":" + String(ayG, 2) + ",\"az\":" + String(azG, 2);
  j += ",\"gx\":" + String(gxDps, 1) + ",\"gy\":" + String(gyDps, 1) + ",\"gz\":" + String(gzDps, 1);
  j += ",\"temp\":" + String(mpuTempC, 1) + ",\"peak\":" + String(gPeak > gNow ? gPeak : gNow, 2);
  j += ",\"seq\":" + String(mpuSeq) + ",\"ms\":50,\"hist\":[";
  uint32_t n = mpuSeq < MPU_HIST ? mpuSeq : MPU_HIST;
  for (uint32_t i = 0; i < n; i++) {                 // oldest first; the last one has number seq-1
    const MpuSample& m = mpuHist[(mpuSeq - n + i) % MPU_HIST];
    if (i) j += ',';
    j += '[' + String(m.ax) + ',' + String(m.ay) + ',' + String(m.az) + ']';
  }
  j += "]}";
  j += "}";
  gPeak = 0;
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

// Circuit-check buttons on the dashboard:  POST /test?what=...
void handleTest() {
  String what = server.arg("what");
  String msg;
  bool ok = true;
  uint16_t pendingJob = 0;           // SIM jobs: finished later, see /status -> sim_job
  if (what == "beep") {
    testBeepUntil = millis() + 1000;
    msg = "Buzzer on for 1 second";
  } else if (what == "lcd") {
    lcdReinit();
    lcdMsg1 = "Dashboard test";
    lcdMsg2 = "LCD working :)";
    lcdMsgUntil = millis() + 4000;
    ok = lcdOk;
    msg = lcdOk ? "Test message on the LCD for 4 seconds" : "LCD not found on I2C (check SDA 21 / SCL 22 / power)";
  } else if (what == "drowsy") {
    testLevel = constrain(server.arg("level").toInt(), 0, 3);
    testLevelUntil = millis() + 10000;
    msg = "Drowsiness level " + String(testLevel) + " for 10 seconds (press the button to silence)";
  } else if (what == "crash") {
    ok = crashState == NORMAL;
    if (ok) startCountdown(5.4, true);
    msg = ok ? "Crash countdown started. Hold the button 1 s to cancel, or the SMS is sent."
             : "A crash alert is already running";
  } else if (what == "cancel") {
    ok = crashState == COUNTDOWN;
    if (ok) cancelCountdown();
    msg = ok ? "Countdown cancelled" : "No countdown running";
  } else if (what == "resolve") {
    ok = crashState == ALERTED;
    if (ok) resolveCrash();
    msg = ok ? "Alert cleared" : "No alert to clear";
  } else if (what == "mpu") {
    mpuOk = mpuInit();
    ok = mpuOk;
    msg = ok ? "MPU6050 found and set to +-16 g" : "MPU6050 not answering at 0x68 (check SDA/SCL, 3.3V, AD0 to GND)";
  } else if (what == "sim") {
    // answered at once; the dashboard follows the result in /status -> sim_job
    uint16_t id = simEnqueue(JOB_CHECK, "", "");
    ok = id != 0;
    msg = ok ? "Checking the SIM800L..." : "SIM800L busy, try again in a minute";
    if (ok) { pendingJob = id; }
  } else if (what == "sms") {
    String to = normalizePhone(server.arg("to"));
    ok = to.length() > 0;
    uint16_t id = 0;
    if (ok) id = simEnqueue(JOB_SMS_TEST, to, smsText("GuardianX test SMS from the car. If you got this, crash alerts will reach this number."));
    if (ok && !id) ok = false;
    if (!to.length()) msg = "Give a 10-digit mobile number";
    else if (ok)      msg = String("Sending test SMS to ") + to + "...";
    else              msg = "SIM800L busy, try again in a minute";
    if (ok) { pendingJob = id; }
  } else if (what == "dashcam") {
    dashcamTriggerId = 9000 + (millis() / 1000) % 1000;
    if (dashcamTaskHandle) xTaskNotifyGive(dashcamTaskHandle);
    msg = "Hub asked the dashcam to record a test clip";
  } else {
    ok = false;
    msg = "Unknown test. Use what=beep, lcd, drowsy, crash, cancel, resolve, mpu, sim, sms or dashcam";
  }
  Serial.println("[TEST] " + what + ": " + msg);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String body = "{\"ok\":" + String(ok ? "true" : "false") + ",\"message\":\"" + jsonEscape(msg) + "\"";
  if (pendingJob) body += ",\"pending\":true,\"job\":" + String(pendingJob);
  body += "}";
  server.send(ok ? 200 : 409, "application/json", body);
}

void handleDriver() {
  driver.faceId   = server.arg("face_id").toInt();
  driver.name     = server.arg("name");
  driver.age      = server.arg("age");
  driver.blood    = server.arg("blood_group");
  driver.car      = server.arg("car_reg");
  driver.medical  = server.arg("medical_notes");
  driver.relName  = server.arg("relative_name");
  driver.relPhone = server.arg("relative_phone");
  driver.sms      = server.arg("sms_summary");
  if (server.hasArg("backup_phone")) backupPhone = normalizePhone(server.arg("backup_phone"));
  if (driver.name.length())
    Serial.printf("[HUB] Driver profile: %s (camera ID %d), emergency SMS to %s\n",
                  driver.name.c_str(), driver.faceId, driver.relPhone.c_str());
  else
    Serial.println("[HUB] Driver profile cleared");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleRoot() {
  server.send(200, "text/plain", "GuardianX car hub. Open the GuardianX dashboard on the laptop.");
}

// ================================================================== serial
void printStatus() {
  Serial.println("------------------------------------------------------");
  Serial.printf("WiFi \"%s\" at %s, devices joined: %d\n", AP_SSID,
                WiFi.softAPIP().toString().c_str(), WiFi.softAPgetStationNum());
  Serial.printf("MPU6050: %s, now %.2f g (crash at %.1f g held %d samples / %lu ms)\n",
                mpuOk ? "OK" : "NOT FOUND", gNow, CRASH_G, CRASH_CONFIRM_SAMPLES, CRASH_WINDOW_MS);
  Serial.printf("Drowsiness: camera raw %d, confirmed %d, beeps from level %d\n",
                (int)camLevel, drowsyLevel(), DROWSY_BEEP_FROM_LEVEL);
  if (gpsFix()) Serial.printf("GPS: fix %.6f, %.6f (%d satellites)\n",
                              gps.location.lat(), gps.location.lng(), (int)gps.satellites.value());
  else Serial.printf("GPS: no fix yet (%lu characters received%s)\n", (unsigned long)gps.charsProcessed(),
                     gps.charsProcessed() < 10 ? " - check GPS TX -> GPIO16" : ", needs open sky");
  Serial.printf("SIM800L: %s   LCD: %s\n", simOk ? "OK" : "not found", lcdOk ? "OK" : "not found");
  {
    xSemaphoreTake(simLock, portMAX_DELAY);
    Serial.printf("SIM: signal %d/31, %s %s, supply %.2f V%s%s\n", simCsqVal, regText(simRegStat).c_str(),
                  simOperator.c_str(), simVoltMv / 1000.0, simProblem.length() ? ", PROBLEM: " : "", simProblem.c_str());
    xSemaphoreGive(simLock);
  }
  Serial.printf("Emergency numbers: driver's relative %s, backup %s\n",
                driver.relPhone.length() ? driver.relPhone.c_str() : "(none)", backupPhone.length() ? backupPhone.c_str() : "(none)");
  Serial.printf("Dashcam: %s\n", dashcamOnline ? "online" : "OFFLINE");
  Serial.printf("Camera 1: %s, driver ID %d, drowsiness %d\n",
                camOnline ? "online" : "OFFLINE", (int)camDriverId, (int)camLevel);
  Serial.printf("Driver profile: %s\n", driver.name.length() ? driver.name.c_str() : "(none yet)");
  Serial.printf("Crash: %s, last SMS: %s\n",
                crashState == NORMAL ? "none" : crashState == COUNTDOWN ? "COUNTDOWN" : "ALERTED",
                simGet(smsStatus).c_str());
  Serial.println("------------------------------------------------------");
}

void printHelp() {
  Serial.println("Commands: c=crash  x=cancel  r=I'm OK  0-3=test drowsiness  g=live G  s=status");
  Serial.println("          m=check SIM800L  t=test SMS to the backup number  h=help");
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'c': startCountdown(5.4, true); break;
    case 'x': cancelCountdown(); break;
    case 'r': if (crashState == ALERTED) resolveCrash(); break;
    case '0': case '1': case '2': case '3':
      testLevel = c - '0';
      testLevelUntil = millis() + 15000;
      Serial.printf("[DROWSY] Test level %d for 15 s\n", testLevel);
      break;
    case 'g': liveG = !liveG; break;
    case 'm': simEnqueue(JOB_CHECK, "", ""); Serial.println("[SIM] Checking SIM800L... (result in a few seconds)"); break;
    case 't': {
      String n = normalizePhone(backupPhone);
      if (!n.length()) n = normalizePhone(driver.relPhone);
      if (!n.length()) n = normalizePhone(DEFAULT_EMERGENCY_NUMBER);
      if (!n.length()) { Serial.println("[SIM] No number: set the backup number on the dashboard first"); break; }
      simEnqueue(JOB_SMS_TEST, n, smsText("GuardianX test SMS (Serial Monitor)."));
      Serial.println("[SIM] Test SMS queued to " + n);
      break;
    }
    case 's': printStatus(); break;
    case 'h': printHelp(); break;
    default: break;
  }
}

// =================================================================== setup
void startWifi() {
  WiFi.persistent(false);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(200);
  WiFi.mode(WIFI_AP);
  delay(100);
  bool up = false;
  for (int attempt = 1; attempt <= 3 && !up; attempt++) {
    up = WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, AP_MAX_CLIENTS);
    if (!up) { Serial.printf("[HUB] WiFi start failed (attempt %d)\n", attempt); delay(500); }
  }
  if (!up) {
    Serial.println("[HUB] ERROR: WiFi could not start. LED blinks fast.");
    while (true) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }
  }
  delay(100);
  WiFi.softAPConfig(HUB_IP, HUB_IP, SUBNET);
  WiFi.setSleep(false);
  digitalWrite(PIN_LED, HIGH);
  Serial.printf("[HUB] WiFi \"%s\" is up (password %s), hub address http://%s\n",
                AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n=========== GuardianX central + car hub ===========");

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Wire.begin(PIN_SDA, PIN_SCL);
  lcdAddr = findLcdAddr();
  lcdOk = (lcd.begin(16, 2) == 0);
  if (lcdOk) { lcd.backlight(); lcdShow("GuardianX", "Starting..."); }
  Serial.println(lcdOk ? "[LCD] OK" : "[LCD] not found (check SDA/SCL, 3.3V)");

  mpuOk = mpuInit();
  Serial.println(mpuOk ? "[MPU] MPU6050 OK (+-16 g range)" : "[MPU] MPU6050 not found at 0x68");

  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  SIM.setRxBufferSize(1024);
  SIM.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);
  simLock  = xSemaphoreCreateMutex();
  simQueue = xQueueCreate(6, sizeof(SimJob));

  startWifi();
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/driver", HTTP_POST, handleDriver);
  server.on("/test", HTTP_POST, handleTest);
  server.on("/test", HTTP_GET, handleTest);
  server.begin();

  xTaskCreatePinnedToCore(simTask, "sim800", 8192, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(cameraTask, "camera", 6144, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(dashcamTask, "dashcam", 6144, nullptr, 2, &dashcamTaskHandle, 0);

  // short beep = ready
  digitalWrite(PIN_BUZZER, HIGH); delay(120); digitalWrite(PIN_BUZZER, LOW);
  printHelp();
}

// ==================================================================== loop
void loop() {
  unsigned long now = millis();

  // 1. G-force every 5 ms (crash spikes are short)
  static unsigned long lastMpu = 0, lastMpuRetry = 0;
  if (now - lastMpu >= 5) {
    lastMpu = now;
    if (mpuOk) {
      float g;
      static int badReads = 0, highCount = 0;
      static unsigned long highStart = 0, lastSpikeLog = 0;
      static float highPeak = 0;
      if (mpuReadG(g) && g >= MIN_VALID_G) {
        badReads = 0;
        gNow = g;
        if (g > gPeak) gPeak = g;
        static unsigned long lastHist = 0;
        if (now - lastHist >= 50) { lastHist = now; mpuRecordSample(); }
        if (crashState == NORMAL && now > STARTUP_IGNORE_MS) {
          if (g >= CRASH_G) {
            if (highCount == 0 || now - highStart > CRASH_WINDOW_MS) { highCount = 0; highStart = now; highPeak = 0; }
            highCount++;
            if (g > highPeak) highPeak = g;
            if (highCount >= CRASH_CONFIRM_SAMPLES) { startCountdown(highPeak, false); highCount = 0; }
          } else if (highCount > 0 && now - highStart > CRASH_WINDOW_MS) {
            if (now - lastSpikeLog > 1000) {
              Serial.printf("[MPU] %.1f g spike ignored (too short to be a crash: %d sample(s))\n", highPeak, highCount);
              lastSpikeLog = now;
            }
            highCount = 0;
          }
        }
        if (crashState == COUNTDOWN && g > crashG) crashG = g;   // keep the peak
      } else if (++badReads >= 20) {                              // 20 bad reads in a row = really lost
        mpuOk = false;
        badReads = 0;
        Serial.println("[MPU] Lost contact with the MPU6050");
      }
    } else if (now - lastMpuRetry > 2000) {
      lastMpuRetry = now;
      if ((mpuOk = mpuInit())) Serial.println("[MPU] MPU6050 back");
    }
  }

  // 2. GPS characters
  while (GPSSerial.available()) gps.encode(GPSSerial.read());

  // 3. crash countdown
  if (crashState == COUNTDOWN && now - countdownStart >= CANCEL_WINDOW_MS) sendAlerts();
  digitalWrite(PIN_LED, crashState == COUNTDOWN ? ((now / 250) % 2) : HIGH);

  // 4. driver input, buzzer, web, display
  handleButton();
  buzzerUpdate();
  server.handleClient();
  handleSerial();

  // 5. LCD still on the I2C bus? (checked every 5 s; it is re-initialised when it comes back)
  static unsigned long lastLcdProbe = 0;
  if (now - lastLcdProbe >= 5000 && crashState == NORMAL) {
    lastLcdProbe = now;
    if (!lcdAddr) lcdAddr = findLcdAddr();
    bool present = false;
    if (lcdAddr) { Wire.beginTransmission(lcdAddr); present = Wire.endTransmission() == 0; }
    if (present && !lcdOk) { lcdOk = (lcd.begin(16, 2) == 0); if (lcdOk) { lcdReinit(); Serial.println("[LCD] back"); } }
    else if (!present && lcdOk) { lcdOk = false; Serial.println("[LCD] not answering on I2C"); }
  }

  static unsigned long lastLcd = 0, lastLive = 0, lastLcdReinit = 0;
  const bool noisy = crashState != NORMAL || drowsyLevel() > 0;        // buzzer is beeping
  if (now - lastLcdReinit >= (noisy ? 3000UL : 30000UL)) { lastLcdReinit = now; lcdReinit(); }
  if (now - lastLcd >= 400) { lastLcd = now; updateLcd(); }
  if (liveG && now - lastLive >= 200) { lastLive = now; Serial.printf("G: %.2f\n", gNow); }
}
