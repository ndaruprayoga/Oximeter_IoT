/*
  =====================================================================
  Firmware Oximeter ESP32 - Full System Auto Firebase + OLED Waveform
  =====================================================================
  Hardware : ESP32 + MAX30102 + OLED SSD1306 (SDA=18, SCL=19)
  Core 1   : Pembacaan Sensor MAX30102 & Animasi OLED
  Core 0   : Komunikasi WiFi & Firebase REST API (Background)
  =====================================================================
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

// ------------------- KONFIGURASI WIFI & FIREBASE -------------------
const char* WIFI_SSID     = "Blackbull";
const char* WIFI_PASSWORD = "12345678";

const String FIREBASE_HOST = "https://lexshin-project-default-rtdb.asia-southeast1.firebasedatabase.app";
const String FIREBASE_AUTH = ""; 

// ------------------- KONFIGURASI HARDWARE & OLED -------------------
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define OLED_I2C_ADDR  0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MAX30105 particleSensor;

// ------------------- VARIABEL SENSOR & ALGORITMA -------------------
uint32_t irBuffer[100];
uint32_t redBuffer[100];
int32_t  bufferLength = 100;
int32_t  spo2;
int8_t   validSPO2;
int32_t  heartRate;
int8_t   validHeartRate;

int sampleFillIndex = 0;

// Variabel Bersama (Volatile untuk Akses Multi-Core)
volatile int   lastSpo2      = 0;
volatile int   lastBpm       = 0;
volatile float lastPI        = 0.0;
volatile bool  lastAbnormal  = false;
volatile bool  fingerPresent = false;
volatile bool  isMeasuring   = false;

String activePatientId   = "";
String activePatientName = "Belum Dipilih";

// ------------------- GRAFIK GELOMBANG (LOGIKA ORIGINAL) -------------------
#define WAVE_X_START 65     // Posisi awal horizontal
#define WAVE_WIDTH   60     // Lebar grafik
#define WAVE_Y_TOP   16     // Titik puncak paling atas grafik
#define WAVE_HEIGHT  40     // Tinggi grafik (40px)

uint32_t waveBuffer[WAVE_WIDTH];
int waveBufIndex = 0;
int waveDecimateCounter = 0;
const int WAVE_DECIMATE = 5; // Ambil 1 dari 5 sampel mentah

const unsigned long DISPLAY_REFRESH_MS = 60; // Refresh rate ~16 FPS
unsigned long lastDisplayRefresh = 0;

TaskHandle_t TaskFirebaseH;

// Prototipe Fungsi
void setupSensor();
void setupOLED();
void tampilkanPesan(const char* line1, const char* line2, const char* line3);
void pushWaveformSample(uint32_t irValue);
void gambarWaveform();
void tampilkanHasilOLED(int spo2Value, int bpmValue, float piValue, bool isAbnormal);
void bacaSatuSampel();
void hitungSpO2DanBPM();
void refreshDisplayJikaWaktunya();
void refreshDisplayStandby();
void bacaPasienAktifDariFirebase();
void kirimLiveReadingsFirebase(int spo2Val, int bpmVal, bool finger);
void taskFirebase(void * pvParameters);

// =====================================================================
void setup() {
  Serial.begin(115200);

  // Inisialisasi I2C pada Pin SDA=18, SCL=19
  Wire.begin(18, 19);

  setupOLED();
  tampilkanPesan("Menghubungkan WiFi", WIFI_SSID, "Mohon tunggu...");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Terhubung! IP: " + WiFi.localIP().toString());

  setupSensor();

  // Jalankan Task Firebase di Core 0 (Latar Belakang)
  xTaskCreatePinnedToCore(
    taskFirebase,      
    "TaskFirebase",    
    10000,             
    NULL,              
    1,                 
    &TaskFirebaseH,    
    0                  // Core 0
  );

  tampilkanPesan("Sistem Siap", "Pilih Pasien", "di Web App...");
}

// =====================================================================
// CORE 1 (loop): Fokus 100% pada Pembacaan Sensor & Animasi Grafik
// =====================================================================
void loop() {
  if (isMeasuring) {
    bacaSatuSampel();
    refreshDisplayJikaWaktunya();
  } else {
    refreshDisplayStandby();
    delay(100);
  }
}

// =====================================================================
// CORE 0 (taskFirebase): Komunikasi Firebase Tanpa Mengganggu Sensor
// =====================================================================
void taskFirebase(void * pvParameters) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      bacaPasienAktifDariFirebase();

      // Selalu kirim status ke Firebase saat mode ukur aktif (termasuk nilai 0 saat jari dilepas)
      if (isMeasuring) {
        kirimLiveReadingsFirebase(lastSpo2, lastBpm, fingerPresent);
      }
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Kirim data tiap 2 detik
  }
}

// =====================================================================
void setupSensor() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 tidak terdeteksi. Periksa Wiring!");
    tampilkanPesan("ERROR:", "Sensor MAX30102", "Tidak Ditemukan!");
    while (1) delay(10);
  }

  byte ledBrightness = 0x7F;
  byte sampleAverage  = 4;
  byte ledMode        = 2;
  byte sampleRate     = 200;
  int  pulseWidth     = 411;
  int  adcRange       = 16384;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

// =====================================================================
void setupOLED() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    Serial.println("OLED Gagal Inisialisasi");
    return;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.display();
}

// =====================================================================
void tampilkanPesan(const char* line1, const char* line2, const char* line3) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(line1);
  display.setCursor(0, 20);
  display.println(line2);
  display.setCursor(0, 40);
  display.println(line3);
  display.display();
}

// =====================================================================
void pushWaveformSample(uint32_t irValue) {
  waveDecimateCounter++;
  if (waveDecimateCounter < WAVE_DECIMATE) return;
  waveDecimateCounter = 0;

  waveBuffer[waveBufIndex] = irValue;
  waveBufIndex = (waveBufIndex + 1) % WAVE_WIDTH;
}

void gambarWaveform() {
  uint32_t minVal = waveBuffer[0];
  uint32_t maxVal = waveBuffer[0];

  for (int i = 1; i < WAVE_WIDTH; i++) {
    if (waveBuffer[i] < minVal) minVal = waveBuffer[i];
    if (waveBuffer[i] > maxVal) maxVal = waveBuffer[i];
  }

  int yBottom = SCREEN_HEIGHT - 1; 

  for (int col = 0; col < WAVE_WIDTH; col++) {
    int idx = (waveBufIndex + col) % WAVE_WIDTH;
    uint32_t val = waveBuffer[idx];

    int yPeak;
    if (val == 0 || maxVal == minVal) {
      yPeak = yBottom;
    } else {
      yPeak = yBottom - (int)(((float)(val - minVal) / (float)(maxVal - minVal)) * (WAVE_HEIGHT - 1));
    }

    int x = WAVE_X_START + col;
    display.drawFastVLine(x, yPeak, yBottom - yPeak + 1, SSD1306_WHITE);
  }
}

// =====================================================================
void tampilkanHasilOLED(int spo2Value, int bpmValue, float piValue, bool isAbnormal) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // ---- %SpO2 ----
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("%SpO2");
  display.setTextSize(2);
  display.setCursor(0, 8);
  display.print(spo2Value);

  // ---- PRbpm ----
  display.setTextSize(1);
  display.setCursor(0, 24);
  display.print("PRbpm");
  display.setTextSize(2);
  display.setCursor(0, 32);
  display.print(bpmValue);

  // ---- PI% ----
  display.setTextSize(1);
  display.setCursor(0, 48);
  display.print("PI:");
  display.print(piValue, 1);
  display.print("%");

  // ---- Status ----
  display.setCursor(0, 56);
  display.print(isAbnormal ? "Status:LOW!" : "Status:OK");

  // Garis Pemisah Vertikal
  display.drawLine(WAVE_X_START - 3, 0, WAVE_X_START - 3, SCREEN_HEIGHT - 1, SSD1306_WHITE);

  // Grafik Gelombang
  gambarWaveform();

  display.display();
}

// =====================================================================
void bacaSatuSampel() {
  particleSensor.check();
  if (!particleSensor.available()) return;

  uint32_t red = particleSensor.getRed();
  uint32_t ir  = particleSensor.getIR();
  particleSensor.nextSample();

  if (ir > 50000 && validHeartRate) {
    pushWaveformSample(ir);
  } else {
    pushWaveformSample(0);
  }

  redBuffer[sampleFillIndex] = red;
  irBuffer[sampleFillIndex]  = ir;
  sampleFillIndex++;

  if (sampleFillIndex >= bufferLength) {
    hitungSpO2DanBPM();
    sampleFillIndex = 0;
  }
}

void hitungSpO2DanBPM() {
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, bufferLength, redBuffer,
    &spo2, &validSPO2, &heartRate, &validHeartRate
  );

  long irSum = 0;
  uint32_t irMin = irBuffer[0], irMax = irBuffer[0];
  for (int i = 0; i < bufferLength; i++) {
    irSum += irBuffer[i];
    if (irBuffer[i] < irMin) irMin = irBuffer[i];
    if (irBuffer[i] > irMax) irMax = irBuffer[i];
  }
  long irMean = irSum / bufferLength;

  fingerPresent = (irMean > 50000);

  // Jika tidak ada jari, reset nilai variabel ke 0
  if (!fingerPresent) {
    lastSpo2 = 0;
    lastBpm = 0;
    lastPI = 0.0;
    lastAbnormal = false;
    return;
  }

  float ac = (float)(irMax - irMin);
  float dc = (float)irMean;
  float piPercent = (dc > 0) ? (ac / dc) * 100.0 : 0.0;

  if (validSPO2 && validHeartRate && spo2 <= 100 && heartRate > 30 && heartRate < 220) {
    lastSpo2 = spo2;
    lastBpm  = heartRate;
    lastPI   = piPercent;
    lastAbnormal = (lastSpo2 < 95) || (lastBpm < 60) || (lastBpm > 100);
  }
}

// =====================================================================
void refreshDisplayJikaWaktunya() {
  if (millis() - lastDisplayRefresh < DISPLAY_REFRESH_MS) return;
  lastDisplayRefresh = millis();

  // Tampilan khusus jika pasien sudah dipilih tetapi JARI BELUM DITEMPEL
  if (!fingerPresent) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    
    display.setCursor(0, 0);
    display.println("--- SIAP MENGUKUR ---");
    
    display.setCursor(0, 18);
    display.print("Pasien: ");
    display.println(activePatientName);
    
    display.setCursor(0, 36);
    display.println("Tempelkan Jari Anda");
    display.setCursor(0, 48);
    display.println("pada Sensor Oximeter");
    
    display.display();
    return;
  }

  tampilkanHasilOLED(lastSpo2, lastBpm, lastPI, lastAbnormal);
}

// =====================================================================
void bacaPasienAktifDariFirebase() {
  HTTPClient http;
  String url = FIREBASE_HOST + "/system.json";
  if (FIREBASE_AUTH != "") url += "?auth=" + FIREBASE_AUTH;

  http.begin(url);
  int httpCode = http.GET();

  if (httpCode == 200) {
    String payload = http.getString();
    
    int idIdx = payload.indexOf("\"active_patient_id\":\"");
    if (idIdx != -1) {
      int endIdx = payload.indexOf("\"", idIdx + 21);
      activePatientId = payload.substring(idIdx + 21, endIdx);
    }
    int nameIdx = payload.indexOf("\"active_patient_name\":\"");
    if (nameIdx != -1) {
      int endIdx = payload.indexOf("\"", nameIdx + 23);
      activePatientName = payload.substring(nameIdx + 23, endIdx);
    }

    isMeasuring = (activePatientId != "" && activePatientId != "null");
  }
  http.end();
}

void kirimLiveReadingsFirebase(int spo2Val, int bpmVal, bool finger) {
  HTTPClient http;
  String url = FIREBASE_HOST + "/live_readings.json";
  if (FIREBASE_AUTH != "") url += "?auth=" + FIREBASE_AUTH;

  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  String jsonPayload = "{";
  jsonPayload += "\"spo2\":" + String(spo2Val) + ",";
  jsonPayload += "\"bpm\":" + String(bpmVal) + ",";
  jsonPayload += "\"finger_present\":" + String(finger ? "true" : "false") + ",";
  jsonPayload += "\"is_measuring\":" + String(isMeasuring ? "true" : "false") + ",";
  jsonPayload += "\"patient_id\":\"" + activePatientId + "\",";
  jsonPayload += "\"timestamp\":{\".sv\":\"timestamp\"}";
  jsonPayload += "}";

  http.PUT(jsonPayload);
  http.end();
}

// Tampilan STANDBY saat belum ada pasien yang dipilih dari Web
void refreshDisplayStandby() {
  static unsigned long lastAnim = 0;
  if (millis() - lastAnim < 1000) return;
  lastAnim = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("[ OXIMETER STANDBY ]");
  display.setCursor(0, 20);
  display.println("Pilih profil pasien");
  display.setCursor(0, 36);
  display.println("pada Web Dashboard");
  display.setCursor(0, 48);
  display.println("untuk memulai...");
  display.display();
}