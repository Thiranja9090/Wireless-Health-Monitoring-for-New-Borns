#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MPU6050.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <math.h>

// ----- OLED -----
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------------- WiFi ----------------
const char* ssid     = "TP-Link_3C98";
const char* password = "a1l2o3k4a";

ESP8266WebServer server(80);

// --------------- Pins -----------------
#define SDA_PIN       4      // GPIO4 (I2C SDA)
#define SCL_PIN       5      // GPIO5 (I2C SCL)
#define ONE_WIRE_BUS 14      // DS18B20 data pin (GPIO14)

// ------------- DS18B20 ----------------
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);

// ------------- MPU6050 ----------------
MPU6050 mpu;

// ------------- MAX30102 ---------------
MAX30105 particleSensor;
const int SAMPLES = 100;
uint32_t irBuffer[SAMPLES];
uint32_t redBuffer[SAMPLES];

// --------- Latest sensor values -------
float   lastTempC       = NAN;
int32_t lastHR          = -1;
int32_t lastSpO2        = -1;
bool    lastHRValid     = false;
bool    lastSpO2Valid   = false;
float   lastMotionLevel = 0.0f;

// ---------- 24h HISTORY (1 sample / minute) ----------
const int HISTORY_SIZE = 1440; // 24h * 60 min
float hrHist[HISTORY_SIZE];
float spo2Hist[HISTORY_SIZE];
float tempHist[HISTORY_SIZE];
float motionHist[HISTORY_SIZE];
int   histIndex = 0;   // points to the next slot to write
int   histCount = 0;   // how many valid samples we actually have
int   secondsCounter = 0; // counts seconds since start (approx)

// timing
unsigned long lastUpdateMs = 0;
// 100 samples at 50 Hz ≈ 2 seconds update interval (we'll still read 100 samples;
// at 100 Hz this will be ~1 second, but we keep 2000 ms between updates)
const unsigned long UPDATE_INTERVAL_MS = 4000;

// ---------------- HTML: MAIN PAGE (served at /) ----------------
const char MAIN_page[] PROGMEM =
"<!DOCTYPE html>\n"
"<html>\n"
"<head>\n"
"  <meta charset=\"utf-8\" />\n"
"  <title>Newborn Monitor Dashboard</title>\n"
"\n"
"  <!-- Chart.js CDN -->\n"
"  <script src=\"https://cdn.jsdelivr.net/npm/chart.js\"></script>\n"
"\n"
"  <style>\n"
"    body {\n"
"      font-family: sans-serif;\n"
"      background: #0d1117;\n"
"      color: #eee;\n"
"      text-align: center;\n"
"      padding-bottom: 40px;\n"
"    }\n"
"\n"
"    h1 { margin-top: 20px; }\n"
"\n"
"    .card {\n"
"      border-radius: 12px;\n"
"      padding: 15px;\n"
"      width: 280px;\n"
"      margin: 15px auto;\n"
"      color: #fff;\n"
"      font-weight: bold;\n"
"    }\n"
"\n"
"    .value { font-size: 32px; margin: 10px 0; }\n"
"    .status { font-size: 14px; opacity: 0.8; }\n"
"\n"
"    /* Card colours */\n"
"    .hr     { background: #c62828; }\n"
"    .spo2   { background: #1565c0; }\n"
"    .temp   { background: #ef6c00; }\n"
"    .motion { background: #2e7d32; }\n"
"\n"
"    /* Chart container styling */\n"
"    .chart-container {\n"
"      width: 90%;\n"
"      max-width: 600px;\n"
"      margin: 25px auto;\n"
"      background: #111;\n"
"      padding: 15px;\n"
"      border-radius: 12px;\n"
"    }\n"
"    a.download-link { color: #4af; text-decoration: none; }\n"
"  </style>\n"
"</head>\n"
"\n"
"<body>\n"
"\n"
"  <h1>Newborn Monitor - Live</h1>\n"
"  <div id=\"status\">Loading...</div>\n"
"  <p><a class=\"download-link\" href=\"/csv\">Download last 24h CSV</a></p>\n"
"\n"
"  <!-- Cards -->\n"
"  <div class=\"card hr\">\n"
"    <div>Heart Rate</div>\n"
"    <div class=\"value\" id=\"hr\">--</div>\n"
"    <div class=\"status\" id=\"hrValid\"></div>\n"
"  </div>\n"
"\n"
"  <div class=\"card spo2\">\n"
"    <div>SpO2</div>\n"
"    <div class=\"value\" id=\"spo2\">--</div>\n"
"    <div class=\"status\" id=\"spo2Valid\"></div>\n"
"  </div>\n"
"\n"
"  <div class=\"card temp\">\n"
"    <div>Temperature</div>\n"
"    <div class=\"value\" id=\"temp\">--</div>\n"
"    <div class=\"status\" id=\"tempStatus\"></div>\n"
"  </div>\n"
"\n"
"  <div class=\"card motion\">\n"
"    <div>Motion</div>\n"
"    <div class=\"value\" id=\"motion\">--</div>\n"
"    <div class=\"status\" id=\"motionStatus\"></div>\n"
"  </div>\n"
"\n"
"  <!-- Graphs -->\n"
"  <div class=\"chart-container\">\n"
"    <canvas id=\"hrChart\"></canvas>\n"
"  </div>\n"
"\n"
"  <div class=\"chart-container\">\n"
"    <canvas id=\"spo2Chart\"></canvas>\n"
"  </div>\n"
"\n"
"  <div class=\"chart-container\">\n"
"    <canvas id=\"tempChart\"></canvas>\n"
"  </div>\n"
"\n"
"  <div class=\"chart-container\">\n"
"    <canvas id=\"motionChart\"></canvas>\n"
"  </div>\n"
"\n"
"  <script>\n"
"    function createChart(canvasId, label, color) {\n"
"      return new Chart(document.getElementById(canvasId), {\n"
"        type: \"line\",\n"
"        data: {\n"
"          labels: [],\n"
"          datasets: [{\n"
"            label: label,\n"
"            data: [],\n"
"            borderColor: color,\n"
"            borderWidth: 2,\n"
"            fill: false,\n"
"            tension: 0.2\n"
"          }]\n"
"        },\n"
"        options: {\n"
"          responsive: true,\n"
"          animation: false,\n"
"          scales: {\n"
"            x: { title: { display: false } },\n"
"            y: { beginAtZero: false }\n"
"          }\n"
"        }\n"
"      });\n"
"    }\n"
"\n"
"    const hrChart     = createChart(\"hrChart\",     \"Heart Rate\",   \"#ff5252\");\n"
"    const spo2Chart   = createChart(\"spo2Chart\",   \"SpO2\",         \"#42a5f5\");\n"
"    const tempChart   = createChart(\"tempChart\",   \"Temperature\",  \"#ff9800\");\n"
"    const motionChart = createChart(\"motionChart\", \"Motion\",       \"#66bb6a\");\n"
"\n"
"    function addData(chart, value) {\n"
"      const now = new Date().toLocaleTimeString();\n"
"      chart.data.labels.push(now);\n"
"      chart.data.datasets[0].data.push(value);\n"
"\n"
"      if (chart.data.labels.length > 60) {\n"
"        chart.data.labels.shift();\n"
"        chart.data.datasets[0].data.shift();\n"
"      }\n"
"\n"
"      chart.update();\n"
"    }\n"
"\n"
"    async function fetchLatest() {\n"
"      const statusEl = document.getElementById(\"status\");\n"
"\n"
"      try {\n"
"        const res = await fetch(\"/data\");\n"
"        if (!res.ok) {\n"
"          statusEl.textContent = \"ESP error: \" + res.status;\n"
"          return;\n"
"        }\n"
"\n"
"        const data = await res.json();\n"
"        statusEl.textContent = \"Last update: \" + new Date().toLocaleTimeString();\n"
"\n"
"        if (data.hr >= 0) {\n"
"          document.getElementById(\"hr\").textContent = data.hr;\n"
"        } else {\n"
"          document.getElementById(\"hr\").textContent = \"--\";\n"
"        }\n"
"        document.getElementById(\"hrValid\").textContent = data.hrStatus;\n"
"\n"
"        if (data.spo2 >= 0) {\n"
"          document.getElementById(\"spo2\").textContent = data.spo2;\n"
"        } else {\n"
"          document.getElementById(\"spo2\").textContent = \"--\";\n"
"        }\n"
"        document.getElementById(\"spo2Valid\").textContent = data.spo2Status;\n"
"\n"
"        if (!isNaN(data.temp)) {\n"
"          document.getElementById(\"temp\").textContent = data.temp.toFixed(1) + \" C\";\n"
"        } else {\n"
"          document.getElementById(\"temp\").textContent = \"--\";\n"
"        }\n"
"        document.getElementById(\"tempStatus\").textContent = data.tempStatus;\n"
"\n"
"        document.getElementById(\"motion\").textContent = data.motionLevel.toFixed(3);\n"
"        document.getElementById(\"motionStatus\").textContent = data.motionStatus;\n"
"\n"
"        if (data.hr >= 0)       addData(hrChart, data.hr);\n"
"        if (data.spo2 >= 0)     addData(spo2Chart, data.spo2);\n"
"        if (!isNaN(data.temp))  addData(tempChart, data.temp);\n"
"        addData(motionChart, data.motionLevel);\n"
"\n"
"      } catch (err) {\n"
"        console.error(err);\n"
"        statusEl.textContent = \"Error fetching data\";\n"
"      }\n"
"    }\n"
"\n"
"    fetchLatest();\n"
"    setInterval(fetchLatest, 2000);\n"
"  </script>\n"
"\n"
"</body>\n"
"</html>\n";

// ----------- Classify Helpers ------------
String classifyHR(int32_t hr, bool valid) {
  if (!valid || hr <= 0) return "No reading";
  if (hr < 90)  return "Low";
  if (hr > 180) return "High";
  return "Normal";
}

String classifySpO2(int32_t s, bool valid) {
  if (!valid || s <= 0) return "No reading";
  if (s < 90) return "Very low";
  if (s < 95) return "Low";
  return "Normal";
}

String classifyTemp(float t) {
  if (isnan(t)) return "No reading";
  if (t < 36.0) return "Low";
  if (t <= 37.5) return "Normal";
  if (t < 38.0) return "Mild fever";
  return "Fever";
}

String classifyMotion(float ml) {
  if (ml < 0.1) return "Still";
  if (ml < 0.4) return "Normal";
  if (ml < 0.8) return "High";
  return "Very strong";
}

// --------- History handling (one sample per ~16s) ---------
void addToHistory() {
  // UPDATE_INTERVAL_MS is 2000 ms => ~2s per call to updateSensors()
  // You set %8 => ~8 * 2s = 16s between stored CSV samples
  if (secondsCounter % 8 != 0) return;

  // store latest readings; use NAN for invalid ones
  hrHist[histIndex]     = (lastHRValid    ? lastHR     : NAN);
  spo2Hist[histIndex]   = (lastSpO2Valid  ? lastSpO2   : NAN);
  tempHist[histIndex]   = (!isnan(lastTempC) ? lastTempC : NAN);
  motionHist[histIndex] = lastMotionLevel;

  histIndex = (histIndex + 1) % HISTORY_SIZE;
  if (histCount < HISTORY_SIZE) histCount++;
}

// -------- OLED Update --------
void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("HR: ");
  if (lastHRValid) display.print(lastHR);
  else display.print("--");

  display.setCursor(0, 12);
  display.print("SpO2: ");
  if (lastSpO2Valid) display.print(lastSpO2);
  else display.print("--");

  display.setCursor(0, 24);
  display.print("Temp: ");
  if (!isnan(lastTempC)) display.print(lastTempC, 1);
  else display.print("--");

  display.setCursor(0, 36);
  display.print("Motion: ");
  display.print(lastMotionLevel, 2);

  display.display();
}

// ------------- Sensor update ----------------
void updateSensors() {
  // DS18B20
  tempSensors.requestTemperatures();
  lastTempC = tempSensors.getTempCByIndex(0);

  // MPU6050
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float axg = ax / 16384.0;
  float ayg = ay / 16384.0;
  float azg = az / 16384.0;
  lastMotionLevel = fabs(sqrt(axg * axg + ayg * ayg + azg * azg) - 1.0);

  // MAX30102 (100 samples)
  for (int i = 0; i < 100; i++) {
    while (!particleSensor.available()) {
      particleSensor.check();
      yield();
    }
    irBuffer[i]  = particleSensor.getIR();
    redBuffer[i] = particleSensor.getRed();
    particleSensor.nextSample();
    yield();
  }

  int32_t spo2, hr;
  int8_t spo2Valid, hrValid;

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, SAMPLES, redBuffer,
    &spo2, &spo2Valid, &hr, &hrValid
  );

  lastHR        = hr;
  lastSpO2      = spo2;
  lastHRValid   = (hrValid   != 0);
  lastSpO2Valid = (spo2Valid != 0);

  // approximate seconds counter (every ~2s)
  secondsCounter++;
  addToHistory();

  updateOLED();
}

// ------------ Web Handlers --------------
void handleRoot() {
  server.send_P(200, "text/html", MAIN_page);
}

void handleData() {
  String json = "{";
  json += "\"hr\":"          + String(lastHR)            + ",";
  json += "\"spo2\":"        + String(lastSpO2)          + ",";
  json += "\"temp\":"        + String(lastTempC, 2)      + ",";
  json += "\"motionLevel\":" + String(lastMotionLevel, 3) + ",";
  json += "\"hrStatus\":\""      + classifyHR(lastHR, lastHRValid)         + "\",";
  json += "\"spo2Status\":\""    + classifySpO2(lastSpO2, lastSpO2Valid)   + "\",";
  json += "\"tempStatus\":\""    + classifyTemp(lastTempC)                 + "\",";
  json += "\"motionStatus\":\""  + classifyMotion(lastMotionLevel)         + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

// New: CSV download handler
void handleCSV() {
  // Build CSV in RAM; for 1440 lines this is fine
  String csv = "index,hr,spo2,temp,motion\n";

  for (int i = 0; i < histCount; i++) {
    int idx = (histIndex - histCount + i + HISTORY_SIZE) % HISTORY_SIZE;

    csv += String(i);  // index / relative sample number
    csv += ",";

    // hr
    if (isnan(hrHist[idx])) csv += "";
    else csv += String(hrHist[idx], 1);
    csv += ",";

    // spo2
    if (isnan(spo2Hist[idx])) csv += "";
    else csv += String(spo2Hist[idx], 0);
    csv += ",";

    // temp
    if (isnan(tempHist[idx])) csv += "";
    else csv += String(tempHist[idx], 2);
    csv += ",";

    // motion (we always have this)
    csv += String(motionHist[idx], 3);

    csv += "\n";
  }

  // Suggest a filename and force download
  server.sendHeader("Content-Disposition", "attachment; filename=\"newborn_24h.csv\"");
  server.send(200, "text/csv", csv);
}

// ------------------ SETUP -------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(SDA_PIN, SCL_PIN);

  // OLED INIT
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED NOT FOUND");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("OLED OK");
    display.display();
  }

  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
  tempSensors.begin();

  mpu.initialize();

  // MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found!");
  } else {
    // Increased sampling rate and LED intensity
    particleSensor.setup();              // default config as base
    particleSensor.setLEDMode(2);        // 2 = Red + IR
    particleSensor.setSampleRate(50);   // was 50; now 100 samples per second
    particleSensor.setPulseWidth(411);   // long pulse width for better SNR
    particleSensor.setPulseAmplitudeIR(0x4F);
    particleSensor.setPulseAmplitudeRed(0x4F);
  }

  // WiFi
  Serial.println("Connecting...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected.");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Web server routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/csv", handleCSV);   // CSV download
  server.begin();
  Serial.println("Web server started");

  updateSensors();
  lastUpdateMs = millis();
}

// ------------------ LOOP -------------------
void loop() {
  server.handleClient();

  if (millis() - lastUpdateMs > UPDATE_INTERVAL_MS) {
    updateSensors();
    lastUpdateMs = millis();
  }
}
