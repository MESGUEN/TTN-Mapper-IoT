#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>

// ====== Pins ======
static const int GPS_RX_PIN  = 6;  // ESP32 RX <- GPS TX
static const int GPS_TX_PIN  = 7;  // ESP32 TX -> GPS RX (optionnel, mais on le garde)

static const int LORA_RX_PIN = 4;  // ESP32 RX <- LoRa TX
static const int LORA_TX_PIN = 5;  // ESP32 TX -> LoRa RX

static const int LED_PIN     = 8;  // NeoPixel DevKitM-1

// ====== LoRaWAN ======
static const char* APPKEY = "5E4EDBD5CC029427681B0F38E0C80B51";  // <-- ton APPKEY
static const uint32_t JOIN_TIMEOUT_MS = 15000;
static const uint32_t AT_TIMEOUT_MS   = 2000;
static const uint32_t TX_TIMEOUT_MS   = 10000;
static const uint32_t SEND_INTERVAL_MS = 60000;

// ====== Serial ======
HardwareSerial LORA(1);                // LoRa en UART matériel (Serial1)
SoftwareSerial GPS(GPS_RX_PIN, GPS_TX_PIN); // GPS en SoftwareSerial

// ====== LED simple ======
Adafruit_NeoPixel pixel(1, LED_PIN, NEO_GRB + NEO_KHZ800);
static void ledOff()   { pixel.setPixelColor(0, pixel.Color(0,0,0));  pixel.show(); }
static void ledGreen() { pixel.setPixelColor(0, pixel.Color(0,25,0)); pixel.show(); }
static void ledRed()   { pixel.setPixelColor(0, pixel.Color(25,0,0)); pixel.show(); }

// ====== Cayenne LPP GPS payload (11 octets) ======
uint8_t LPP_GPS[11] = {0};

// --- signed 24-bit big endian ---
static void s24be(int32_t n, uint8_t out[3]) {
  if (n < 0) n = (1 << 24) + n;
  out[0] = (n >> 16) & 0xFF;
  out[1] = (n >>  8) & 0xFF;
  out[2] = (n >>  0) & 0xFF;
}

// --- NMEA ddmm.mmmm -> degrés décimaux ---
static bool ddmm_to_deg(const char* v, const char* hemi, bool isLat, double& out) {
  if (!v || !*v || !hemi || !*hemi) return false;
  int d = isLat ? 2 : 3;

  char degStr[4] = {0};
  strncpy(degStr, v, d);
  int deg = atoi(degStr);
  double minutes = atof(v + d);

  double x = deg + minutes / 60.0;
  if (hemi[0] == 'S' || hemi[0] == 'W') x = -x;
  out = x;
  return true;
}

// --- lecture NMEA (non-bloquante) depuis GPS SoftwareSerial ---
static bool gps_read_line(char* out, size_t outSize) {
  static char buf[128];
  static size_t idx = 0;

  while (GPS.available()) {
    char c = (char)GPS.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[idx] = '\0';
      strncpy(out, buf, outSize);
      out[outSize - 1] = '\0';
      idx = 0;
      return true;
    }
    if (idx < sizeof(buf) - 1) buf[idx++] = c;
    else idx = 0;
  }
  return false;
}

static bool gps_update_lpp(uint8_t channel = 1) {
  char line[128];
  if (!gps_read_line(line, sizeof(line))) return false;

  if (!(strncmp(line, "$GPGGA", 6) == 0 || strncmp(line, "$GNGGA", 6) == 0)) return false;

  char* star = strchr(line, '*');
  if (star) *star = '\0';

  char* f[20] = {0};
  int nf = 0;
  f[nf++] = line;
  for (char* p = line; *p && nf < 20; p++) {
    if (*p == ',') { *p = '\0'; f[nf++] = p + 1; }
  }
  if (nf < 10) return false;

  if (!f[6] || f[6][0] == '0') return false; // no fix

  double lat, lon;
  if (!ddmm_to_deg(f[2], f[3], true, lat)) return false;
  if (!ddmm_to_deg(f[4], f[5], false, lon)) return false;

  double alt = (f[9] && *f[9]) ? atof(f[9]) : 0.0;

  int32_t lat_i = (int32_t)lround(lat * 10000.0);
  int32_t lon_i = (int32_t)lround(lon * 10000.0);
  int32_t alt_i = (int32_t)lround(alt * 100.0);

  LPP_GPS[0] = channel;
  LPP_GPS[1] = 0x88;

  uint8_t t[3];
  s24be(lat_i, t); LPP_GPS[2]=t[0]; LPP_GPS[3]=t[1]; LPP_GPS[4]=t[2];
  s24be(lon_i, t); LPP_GPS[5]=t[0]; LPP_GPS[6]=t[1]; LPP_GPS[7]=t[2];
  s24be(alt_i, t); LPP_GPS[8]=t[0]; LPP_GPS[9]=t[1]; LPP_GPS[10]=t[2];

  return true;
}

static String hexlify(const uint8_t* b, size_t len) {
  static const char* h = "0123456789abcdef";
  String s; s.reserve(len * 2);
  for (size_t i = 0; i < len; i++) {
    s += h[(b[i] >> 4) & 0xF];
    s += h[b[i] & 0xF];
  }
  return s;
}

// ====== LoRa AT minimal (sur LORA matériel) ======
static void lora_flush() {
  while (LORA.available()) (void)LORA.read();
}

static bool lora_cmd(const String& cmd, uint32_t waitMs = AT_TIMEOUT_MS) {
  lora_flush();
  LORA.print(cmd); LORA.print("\r\n");
  bool response = false;
  uint32_t t0 = millis();
  while (millis() - t0 < waitMs) {
    if (LORA.available()) response = true; // au moins 1 octet reçu
    delay(10);
  }
  return response;
}

static String lora_join() {
  lora_flush();
  LORA.print("AT+JOIN\r\n");

  uint32_t t0 = millis();
  String buf;
  while (millis() - t0 < JOIN_TIMEOUT_MS) {
    while (LORA.available()) {
      char c = (char)LORA.read();
      buf += c;
      String low = buf; low.toLowerCase();
      if (low.indexOf("joined") >= 0) return "JOINED";
      if (low.indexOf("failed") >= 0) return "FAILED";
    }
    delay(10);
  }
  return "TIMEOUT";
}

static String lora_send_hex(const String& hexPayload) {
  lora_flush();
  LORA.print("AT+MSGHEX=\""); LORA.print(hexPayload); LORA.print("\"\r\n");

  uint32_t t0 = millis();
  String buf;
  while (millis() - t0 < TX_TIMEOUT_MS) {
    while (LORA.available()) {
      char c = (char)LORA.read();
      buf += c;

      if (buf.indexOf("Please join") >= 0) return "NOT_JOINED";
      if (buf.indexOf("Done") >= 0) return "DONE";

      int idx = buf.indexOf("RX: \"");
      if (idx >= 0) {
        int start = idx + 5;
        int end = buf.indexOf("\"", start);
        if (end > start) return buf.substring(start, end);
      }
    }
    delay(10);
  }
  return "TIMEOUT";
}

// ====== Main ======
static void fatal(const char* why) {
  Serial.print("DEFAUT: "); Serial.println(why);
  ledRed();
  while (true) delay(1000);
}

int state = 1;
uint32_t lastSend = 0;

static void config_lora() {
  ledOff();
  Serial.println("AT+FDEFAULT");
  if (!lora_cmd("AT+FDEFAULT",3000)) fatal("AT+FDEFAULT");
  Serial.println("AT+RESET");
  if (!lora_cmd("AT+RESET",5000)) fatal("AT+RESET");
  delay(2000);
  Serial.println("AT+MODE=LWOTAA");
  if (!lora_cmd("AT+MODE=LWOTAA")) fatal("AT+MODE=LWOTAA");
  Serial.println("AT+DR=EU868");
  if (!lora_cmd("AT+DR=EU868"))    fatal("AT+DR=EU868");
  Serial.println("AT+CH=NUM,0-2");
  if (!lora_cmd("AT+CH=NUM,0-2"))  fatal("AT+CH=NUM,0-2");
  String keyCmd = String("AT+KEY=APPKEY,\"") + APPKEY + "\"";
  Serial.println(keyCmd);
  if (!lora_cmd(keyCmd)) fatal("AT+KEY=APPKEY");
  Serial.println("AT+CLASS=A");
  if (!lora_cmd("AT+CLASS=A"))    fatal("AT+CLASS=A");
  Serial.println("AT+PORT=8");
  if (!lora_cmd("AT+PORT=8"))     fatal("AT+PORT=8");
  Serial.println("AT+LW=JDC,OFF");
  if (!lora_cmd("AT+LW=JDC,OFF")) fatal("AT+LW=JDC,OFF");

  state = 2;
}

static void join_lora() {
  for (int tries = 0; tries < 10; tries++) {
    String res = lora_join();
    Serial.println("JOIN: " + res);

    if (res == "JOINED") {
      ledGreen();
      state = 3;
      return;
    }
    if (res == "TIMEOUT") fatal("JOIN TIMEOUT");
    delay(30000);
  }
  fatal("JOIN impossible apres 10 essais");
}

static void send_payload() {
  String payload = hexlify(LPP_GPS, 11);
  Serial.println("TX HEX: " + payload);

  String ret = lora_send_hex(payload);
  Serial.println("TX RET: " + ret);

  if (ret == "DONE") { state = 3; return; }
  if (ret == "NOT_JOINED") { state = 1; return; }
  if (ret == "TIMEOUT") fatal("TX TIMEOUT");

  delay(30000);
  state = 3;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pixel.begin();
  pixel.setBrightness(25);
  ledOff();

  // LoRa en UART matériel (Serial1)
  LORA.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

  // GPS en SoftwareSerial
  GPS.begin(9600);

  delay(1000);
  Serial.println("Boot OK");
}

void loop() {
  switch (state) {
    case 1: config_lora(); break;
    case 2: join_lora();   break;

    case 3:
      if (gps_update_lpp(1)) {
        uint32_t now = millis();
        if (now - lastSend > SEND_INTERVAL_MS) {
          lastSend = now;
          state = 4;
        }
      }
      break;

    case 4: send_payload(); break;

    default: fatal("Etat inconnu");
  }

  delay(10);
}
