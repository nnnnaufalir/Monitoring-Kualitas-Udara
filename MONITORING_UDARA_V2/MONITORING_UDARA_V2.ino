// ===================================================================================
// ===         MONITORING KUALITAS UDARA - DEBU, SUHU, KELEMBAPAN, CO2, NH3        ===
// ===================================================================================

// ------ PxMatrix Panel Configuration ------
#define PxMATRIX_OE_INVERT 1
#define PxMATRIX_DATA_INVERT 1
#define PxMATRIX_GAMMA_PRESET 3
#define PxMATRIX_DOUBLE_BUFFER 1

// ------ Library Includes ------
#include <PxMatrix.h>
#include <Fonts/TomThumb.h>
#include "RTClib.h"        // Untuk RTC DS3231
#include "DHTesp.h"        // Untuk Sensor DHT
#include <AverageValue.h>  // Untuk Sensor MQ135
#include "Secrets.h"
#include "thingProperties.h"

// ------ Pin Definitions ------
// P10 Panel Pins
#define P_A 2
#define P_B 16
#define P_LAT 19
#define P_OE 17
// SPI pins
#define SPI_MOSI 23
#define SPI_SCK 18

// DHT22 Sensor Pin
#define PIN_DHT22 26

// MQ-135 Sensor Pin
#define PIN_MQ_A0 35

// DSM501 Sensor Pin
#define PIN_DSM501 25

// ------ Panel Dimensions ------
#define P10_WIDTH 128
#define P10_HEIGHT 16

// ------ Global Object Declarations ------
PxMATRIX display(P10_WIDTH, P10_HEIGHT, P_LAT, P_OE, P_A, P_B);
RTC_DS3231 rtc;
DHTesp dhtSensor;

// ------ MQ-135 Sensor Constants ------
// CO2 Calibration Constants
const int mqR_CO2 = 15000;      // Resistor pull-down (Ohm)
float nilaiRo_CO2 = 568520.94;  // Nilai Ro CO2 hasil kalibrasi (Ohm)
const float a_CO2 = 116.6020682;
const float b_CO2 = -2.769034857;

// NH3 Calibration Constants
const float RL_NH3 = 15.0;  // Resistor beban (kΩ)
const float Vcc_NH3 = 3.3;  // Tegangan catu daya (Volt)
const float Ro_NH3 = 57.0;  // Ro NH3 hasil kalibrasi (kΩ)
const float m_NH3 = -0.417;
const float b_NH3_const = 0.858;

// Averaging Object Configuration
const int dataAverageCount = 10;  // Tetap global karena dipakai di bbrp inisialisasi static lokal

// DSM501 Configuration Constant
const unsigned long samplingTime = 15000;  // Dijadikan const, nama tetap

// ------ Global Structures ------
struct date_time_t {
  int tahun, bulan, hari, jam, menit, detik;
};

// ------ Global Variables ------
// RTC Time
struct date_time_t currentTime;

// Sensor Data
int temp = 0;
int humi = 0;
int co2 = 0;
float nh3 = 0.0;
float pm25 = 0.0;  // Variabel utama untuk nilai PM2.5

// Sensor Threshold
int temp_thres = 35;
int humi_thres = 90;
int co2_thres = 600;
float nh3_thres = 5.0;
float pm25_thres = 35.0;

// ------ Function Prototypes (Forward Declarations for Tasks) ------
void TaskDisplay(void* pvParameters);
void TaskSensor(void* pvParameters);
void TaskRefresh(void* pvParameters);
void TaskUpdateRTC(void* pvParameters);
void TaskUpdateIoT(void* pvParameters);

// ===================================================================================
// ===                      KODE KONTROL PANEL P10 MATRIX                          ===
// ===================================================================================
void P10_bottom(int val_temp, int val_humi, float val_dust) {
  display.drawFastHLine(0, 0, 64, 0xFF);  // Garis pembatas
  display.drawFastHLine(21, 8, 43, 0xFF);
  display.drawFastVLine(21, 0, 16, 0xFF);

  display.setFont(&TomThumb);
  display.setTextColor(0xFF);  // Set warna teks default untuk bagian ini

  display.setCursor(29, 15);
  display_jam();

  if (val_temp > temp_thres || val_humi > humi_thres) {
    P10_fill(0, 21, 8);
    P10_fill(0, 21, 16);
    display.setTextColor(0x00);
    display.setCursor(1, 7);
    display.print("T:");
    display.print(val_temp);
    display.setCursor(16, 7);
    display.print("C");

    display.setCursor(1, 15);
    display.print("H:");
    display.print(val_humi);
    display.setCursor(16, 15);
    display.print("%");
  }

  else {
    display.setTextColor(0xFF);
    display.setCursor(1, 7);
    display.print("T:");
    display.print(val_temp);
    display.setCursor(16, 7);
    display.print("C");

    display.setCursor(1, 15);
    display.print("H:");
    display.print(val_humi);
    display.setCursor(16, 15);
    display.print("%");
  }

  if (val_dust > pm25_thres) {
    P10_fill(21, 43, 8);
    display.setTextColor(0x00);
    display.setCursor(23, 7);
    display.print("PM2.5:  ");
    display.print(val_dust, 1);
  }

  else {
    display.setTextColor(0xFF);
    display.setCursor(23, 7);
    display.print("PM2.5:  ");
    display.print(val_dust, 1);  // PM2.5 dengan 1 desimal
  }
}

void P10_top(int val_co2, float val_nh3) {
  display.setFont();

  if (val_co2 > co2_thres) {
    P10_fill(64, 64, 8);
    display.setTextColor(0x00);
    display.setCursor(65, 0);
    display.print("CO2:");
    display.print(val_co2);
    display.setCursor(104, 0);
    display.print(" PPM");
  }

  else {
    display.setTextColor(0xFF);
    display.setCursor(65, 0);
    display.print("CO2:");
    display.print(val_co2);
    display.setCursor(104, 0);
    display.print(" PPM");
  }

  if (val_nh3 > nh3_thres) {
    P10_fill(64, 64, 16);
    display.setTextColor(0x00);
    display.setCursor(65, 8);
    display.print("NH3:");
    display.print(val_nh3, 1);  // NH3 dengan 1 desimal
    display.setCursor(104, 8);
    display.print(" PPM");
  }

  else {
    display.setTextColor(0xFF);
    display.setCursor(65, 8);
    display.print("NH3:");
    display.print(val_nh3, 1);  // NH3 dengan 1 desimal
    display.setCursor(104, 8);
    display.print(" PPM");
  }
}

void P10_fill(int x, int y, int blocks) {
  display.drawFastHLine(x, blocks - 8, y, 0xFF);
  display.drawFastHLine(x, blocks - 7, y, 0xFF);
  display.drawFastHLine(x, blocks - 6, y, 0xFF);
  display.drawFastHLine(x, blocks - 5, y, 0xFF);
  display.drawFastHLine(x, blocks - 4, y, 0xFF);
  display.drawFastHLine(x, blocks - 3, y, 0xFF);
  display.drawFastHLine(x, blocks - 2, y, 0xFF);
  display.drawFastHLine(x, blocks - 1, y, 0xFF);
}

void P10_clear() {
  display.clearDisplay();  // Biasanya mengisi dengan 0x00 (hitam/mati)
}

void P10_show() {
  display.showBuffer();
}

void P10_begin() {
  display.begin(4);  // Sesuaikan dengan jumlah baris per scan panel Anda
  P10_clear();
  display.setBrightness(255);  // Kecerahan (0-255)
  display.setTextColor(0xFF);  // Atur warna teks default global setelah clear
}

// ===================================================================================
// ===                         KODE KONTROL RTC (DS3231)                           ===
// ===================================================================================
void display_jam() {
  int jam_disp, menit_disp, detik_disp;

  jam_disp = currentTime.jam;
  menit_disp = currentTime.menit;
  detik_disp = currentTime.detik;

  if (jam_disp < 10) display.print("0");
  display.print(jam_disp);
  display.print(":");
  if (menit_disp < 10) display.print("0");
  display.print(menit_disp);
  display.print(":");
  if (detik_disp < 10) display.print("0");
  display.print(detik_disp);
}

void RTC_begin() {
  Serial.println("Initializing RTC...");
  if (!rtc.begin()) {
    Serial.println("RTC tidak terdeteksi!");
    // while (1) delay(1); // Hentikan jika RTC kritis
  }

  if (rtc.lostPower()) {
    Serial.println("RTC kehilangan daya, mengatur ulang waktu...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

float RTC_update_time_and_get_temp() {
  DateTime now = rtc.now();
  currentTime.tahun = now.year();
  currentTime.bulan = now.month();
  currentTime.hari = now.day();
  currentTime.jam = now.hour();
  currentTime.menit = now.minute();
  currentTime.detik = now.second();
  return rtc.getTemperature();
}

// ===================================================================================
// ===                      KODE KONTROL SENSOR DHT22                              ===
// ===================================================================================
void DHT22_begin() {
  Serial.println("Initializing DHT22 Sensor...");
  dhtSensor.setup(PIN_DHT22, DHTesp::DHT22);
}

float get_humidity_dht22() {
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  float humi_val = data.humidity;

  if (isnan(humi_val)) {
    Serial.println("Failed to read humidity from DHT sensor!");
    return -1;  // Indikasi error
  }
  if (humi_val >= 99.0) {
    humi_val = 99.0;
  }
  return humi_val;
}

float get_temperature_dht22() {
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  float temp_val = data.temperature;

  if (isnan(temp_val)) {
    Serial.println("Failed to read temperature from DHT sensor!");
    return -1;  // Indikasi error
  }
  if (temp_val >= 99.0) {
    temp_val = 99.0;
  }
  return temp_val;
}

// ===================================================================================
// ===                 KODE KONTROL SENSOR MQ-135 (CO2 & NH3)                      ===
// ===================================================================================
void MQ135_begin() {
  Serial.println("Initializing MQ-135 Sensor...");
  pinMode(PIN_MQ_A0, INPUT);
  analogReadResolution(12);  // ADC 12-bit (0–4095)
}

float get_nh3_mq135() {
  static AverageValue<float> averageNH3(dataAverageCount);  // Dijadikan static lokal, nama tetap
  int adcValue = analogRead(PIN_MQ_A0);
  if (adcValue == 0) return nh3;  // Kembalikan nilai lama jika error

  float Vrl = adcValue * (Vcc_NH3 / 4095.0f);  // Gunakan 4095.0f untuk float division
  if (Vrl == 0) return nh3;                    // Hindari pembagian dengan nol

  float Rs = (Vcc_NH3 - Vrl) * RL_NH3 / Vrl;
  float ratio_mq = Rs / Ro_NH3;  // Ganti nama lokal 'ratio' agar tidak konflik dengan global (yang sudah dihapus)

  if (ratio_mq <= 0) return nh3;  // log10 dari non-positif tidak valid

  float ppmNH3 = pow(10, (log10(ratio_mq) - b_NH3_const) / m_NH3);
  averageNH3.push(ppmNH3);  // Menggunakan objek static lokal averageNH3
  float avg_nh3 = averageNH3.average();
  return avg_nh3 > 0 ? avg_nh3 : 0;
}

float get_co2_mq135() {
  static AverageValue<float> averageCO2(dataAverageCount);  // Dijadikan static lokal, nama tetap
  float adc = analogRead(PIN_MQ_A0);
  if (adc == 0) return co2;  // Kembalikan nilai lama jika error

  float rS = ((4095.0f * mqR_CO2) / adc) - mqR_CO2;  // Gunakan 4095.0f
  if (nilaiRo_CO2 == 0) return co2;                  // Hindari pembagian dengan nol jika Ro tidak valid

  float ppm = a_CO2 * pow((float)rS / nilaiRo_CO2, b_CO2);
  averageCO2.push(ppm);  // Menggunakan objek static lokal averageCO2

  float ppmCO2 = averageCO2.average();
  if (ppmCO2 < 400) { ppmCO2 = 400; }  // Batas bawah PPM
  if (ppmCO2 > 999) { ppmCO2 = 999; }  // Batas atas PPM (sesuaikan)
  return ppmCO2;
}

// ===================================================================================
// ===                      KODE SENSOR DSM501A (PM2.5)                            ===
// ===================================================================================
void DSM501A_begin() {
  Serial.println("Initializing DSM501A (PM2.5 Sensor)...");  // Placeholder dihilangkan
  pinMode(PIN_DSM501, INPUT);
}

// Fungsi DSM501A_read diubah untuk me-return float, global lastPM25 dihilangkan
float DSM501A_read() {
  unsigned long duration_local;               // Variabel lokal, nama tidak konflik
  unsigned long lowPulseOccupancy_local = 0;  // Variabel lokal
  unsigned long startTime = millis();

  while (millis() - startTime < samplingTime) {
    duration_local = pulseInLong(PIN_DSM501, LOW, 100000);  // Timeout 100ms
    lowPulseOccupancy_local += duration_local;
    vTaskDelay(20 / portTICK_PERIOD_MS);  // jangan terlalu intens, beri kesempatan task lain
  }

  float ratio_local = (lowPulseOccupancy_local / (samplingTime * 10.0f));  // Gunakan .0f untuk float context
  // Batasi nilai LPO percentage antara 0% dan 100% (jika 'ratio' ini memang persentase)
  // Perlu validasi apakah perhitungan 'ratio' ini sudah benar menghasilkan persentase untuk formula di bawah
  if (ratio_local > 100.0f) {
    ratio_local = 100.0f;
  }
  if (ratio_local < 0.0f) {
    ratio_local = 0.0f;
  }
  // float concentration = 1.1 * pow(ratio_local, 3) - 3.8 * pow(ratio_local, 2) + 520 * ratio_local + 0.62; // Formula alternatif
  return (20.887f * ratio_local) + 20.667f;  // Langsung return hasil PM2.5
}

// ===================================================================================
// ===                 MANAJEMEN TASK (FreeRTOS) & SETUP/LOOP UTAMA                ===
// ===================================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\nSystem Starting...");

  P10_begin();
  RTC_begin();
  DHT22_begin();
  MQ135_begin();
  DSM501A_begin();

  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  setDebugMessageLevel(4);
  ArduinoCloud.printDebugInfo();

  Serial.println("Creating FreeRTOS Tasks...");
  xTaskCreatePinnedToCore(TaskUpdateIoT, "TaskUpdateIoT", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskUpdateRTC, "TaskUpdateRTC", 2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskSensor, "TaskSensor", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskDisplay, "TaskDisplay", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskRefresh, "TaskRefresh", 2048, NULL, 3, NULL, 1);

  Serial.println("Setup complete. Tasks running.");
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// ===================================================================================
// ===                      IMPLEMENTASI TASK RTOS                                 ===
// ===================================================================================
void TaskUpdateIoT(void* pvParameters) {
  for (;;) {
    ArduinoCloud.update();

    vTaskDelay(15000 / portTICK_PERIOD_MS);  // Update waktu setiap detik
  }
}

void TaskUpdateRTC(void* pvParameters) {
  for (;;) {
    RTC_update_time_and_get_temp();  // Fungsi ini memperbarui global 'currentTime'

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Update waktu setiap detik
  }
}

void TaskSensor(void* pvParameters) {
  for (;;) {
    float current_temp_dht = get_temperature_dht22();
    float current_humi_dht = get_humidity_dht22();
    if (current_temp_dht != -1) { temp = (int)round(current_temp_dht); }
    if (current_humi_dht != -1) { humi = (int)round(current_humi_dht); }

    tEMPERATURE = temp;
    hUMIDITY = humi;

    co2 = (int)round(get_co2_mq135());
    cO2 = co2;

    nh3 = get_nh3_mq135();
    nH3 = nh3;

    pm25 = DSM501A_read();  // Memanggil fungsi yang sudah me-return float
    pM25 = pm25;

    Serial.println("=======================================================");
    Serial.print("S : ");
    Serial.print(temp);
    Serial.print("  ||  H : ");
    Serial.println(humi);
    Serial.print("CO2 : ");
    Serial.print(co2);
    Serial.print("  ||  NH3 : ");
    Serial.println(nh3);
    Serial.print("pm25 : ");
    Serial.println(pm25);
    Serial.println("=======================================================");
    Serial.println(" ");

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Baca sensor setiap 1 detik (DSM501A akan mendominasi siklus ini)
  }
}

void TaskDisplay(void* pvParameters) {
  int lastTemp_disp = -99, lastHumi_disp = -99, lastCO2_disp = -99;  // Variabel lokal untuk state yang ditampilkan
  float lastNH3_disp = -99.0f, lastPM25_disp_val = -99.0f;           // Nama var lokal dibedakan
  struct date_time_t lastTime_disp = { 0, 0, 0, 0, 0, -1 };          // State waktu yang terakhir ditampilkan

  for (;;) {
    bool should_redraw = false;  // Flag apakah perlu menggambar ulang

    // Ambil snapshot waktu saat ini dengan aman untuk perbandingan
    int current_jam_snapshot, current_menit_snapshot, current_detik_snapshot;

    current_jam_snapshot = currentTime.jam;
    current_menit_snapshot = currentTime.menit;
    current_detik_snapshot = currentTime.detik;

    // Cek apakah data sensor yang akan ditampilkan perlu diupdate dari global
    if (temp != lastTemp_disp || humi != lastHumi_disp || co2 != lastCO2_disp || abs(nh3 - lastNH3_disp) > 0.05f || abs(pm25 - lastPM25_disp_val) > 0.05f) {
      should_redraw = true;
    }

    // Cek apakah tampilan jam perlu diupdate
    if (current_jam_snapshot != lastTime_disp.jam || current_menit_snapshot != lastTime_disp.menit || current_detik_snapshot != lastTime_disp.detik) {
      should_redraw = true;
    }

    if (should_redraw) {
      P10_clear();
      P10_bottom(temp, humi, pm25);  // Gambar dengan data global terbaru
      P10_top(co2, nh3);             // Gambar dengan data global terbaru
      P10_show();

      // Update state terakhir yang BARU SAJA ditampilkan
      lastTemp_disp = temp;
      lastHumi_disp = humi;
      lastCO2_disp = co2;
      lastNH3_disp = nh3;
      lastPM25_disp_val = pm25;

      // Update lastTime_disp dengan waktu yang baru saja ditampilkan
      // Tidak perlu critical section di sini karena hanya mengassign dari snapshot
      lastTime_disp.jam = current_jam_snapshot;
      lastTime_disp.menit = current_menit_snapshot;
      lastTime_disp.detik = current_detik_snapshot;
      // Tidak perlu update tahun, bulan, hari jika hanya perbandingan detik/menit/jam untuk display
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Cek untuk update tampilan ~10x per detik
  }
}

void TaskRefresh(void* pvParameters) {
  for (;;) {
    display.display(5);                  // Waktu scan PxMatrix per baris (microseconds)
    vTaskDelay(1 / portTICK_PERIOD_MS);  // Refresh rate P10
  }
}

// ====== KIRIM KE CLOUD ======
void onCO2Change() {}
void onNH3Change() {}
void onPM25Change() {}
void onSUHUChange() {}
void onHUMIDITYChange() {}
void onTEMPERATUREChange() {}