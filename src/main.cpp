#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PZEM004Tv30.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_pm.h"
#include "esp_sleep.h"

// Konfigurasi WiFi
const char* ssid = "b401_wifi";
const char* password = "b401juara1";

// Konfigurasi PZEM-004T
#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17

// Konfigurasi Pin Relay
#define RELAYPIN 25

PZEM004Tv30 pzem(Serial2, PZEM_RX_PIN, PZEM_TX_PIN);

// Node-RED endpoint
const char* nodeRedURL = "http://192.168.200.118:1880/sensor";

// ===== THRESHOLD PERUBAHAN SIGNIFIKAN =====
// Atur nilai threshold untuk mendeteksi perubahan signifikan
const float VOLTAGE_THRESHOLD = 1.0;      // Volt (V)
const float CURRENT_THRESHOLD = 0.05;     // Ampere (A)
const float POWER_THRESHOLD = 5.0;        // Watt (W)
const float ENERGY_THRESHOLD = 0.01;      // kWh
const float FREQUENCY_THRESHOLD = 0.5;    // Hz
const float PF_THRESHOLD = 0.05;          // Power Factor

// Queue untuk komunikasi antar task
QueueHandle_t sensorQueue;

// Struct untuk data sensor PZEM
struct PZEMData {
  float voltage;
  float current;
  float power;
  float energy;
  float frequency;
  float powerFactor;
  unsigned long timestamp;
  bool isValid;
};

// Data terakhir yang berhasil dikirim
PZEMData lastSentData = {0, 0, 0, 0, 0, 0, 0, false};
bool hasLastData = false;

// Konfigurasi Power Management untuk Tickless
void setupPowerManagement() {
  esp_pm_config_esp32_t pm_config = {
    .max_freq_mhz = 240,
    .min_freq_mhz = 80,
    .light_sleep_enable = true
  };
  
  esp_err_t err = esp_pm_configure(&pm_config);
  if (err == ESP_OK) {
    Serial.println("✓ Tickless mode (Power Management) berhasil diaktifkan!");
    Serial.println("  - Max freq: 240 MHz");
    Serial.println("  - Min freq: 80 MHz");
    Serial.println("  - Light sleep: ENABLED");
  } else {
    Serial.printf("✗ Gagal mengaktifkan power management: %d\n", err);
  }
}

// Fungsi untuk mengecek apakah ada perubahan signifikan
bool hasSignificantChange(const PZEMData& newData, const PZEMData& oldData) {
  // Jika belum pernah kirim data, kirim data pertama
  if (!hasLastData) {
    return true;
  }
  
  // Cek setiap parameter apakah ada perubahan signifikan
  bool voltageChanged = abs(newData.voltage - oldData.voltage) >= VOLTAGE_THRESHOLD;
  bool currentChanged = abs(newData.current - oldData.current) >= CURRENT_THRESHOLD;
  bool powerChanged = abs(newData.power - oldData.power) >= POWER_THRESHOLD;
  bool energyChanged = abs(newData.energy - oldData.energy) >= ENERGY_THRESHOLD;
  bool frequencyChanged = abs(newData.frequency - oldData.frequency) >= FREQUENCY_THRESHOLD;
  bool pfChanged = abs(newData.powerFactor - oldData.powerFactor) >= PF_THRESHOLD;
  
  // Jika salah satu ada perubahan signifikan
  if (voltageChanged || currentChanged || powerChanged || 
      energyChanged || frequencyChanged || pfChanged) {
    
    Serial.println(">> Perubahan signifikan terdeteksi:");
    if (voltageChanged) {
      Serial.printf("   Voltage: %.2f → %.2f V (Δ%.2f)\n", 
                    oldData.voltage, newData.voltage, 
                    abs(newData.voltage - oldData.voltage));
    }
    if (currentChanged) {
      Serial.printf("   Current: %.3f → %.3f A (Δ%.3f)\n", 
                    oldData.current, newData.current,
                    abs(newData.current - oldData.current));
    }
    if (powerChanged) {
      Serial.printf("   Power: %.2f → %.2f W (Δ%.2f)\n", 
                    oldData.power, newData.power,
                    abs(newData.power - oldData.power));
    }
    if (energyChanged) {
      Serial.printf("   Energy: %.3f → %.3f kWh (Δ%.3f)\n", 
                    oldData.energy, newData.energy,
                    abs(newData.energy - oldData.energy));
    }
    if (frequencyChanged) {
      Serial.printf("   Frequency: %.1f → %.1f Hz (Δ%.1f)\n", 
                    oldData.frequency, newData.frequency,
                    abs(newData.frequency - oldData.frequency));
    }
    if (pfChanged) {
      Serial.printf("   Power Factor: %.2f → %.2f (Δ%.2f)\n", 
                    oldData.powerFactor, newData.powerFactor,
                    abs(newData.powerFactor - oldData.powerFactor));
    }
    
    return true;
  }
  
  Serial.println(">> Tidak ada perubahan signifikan, data tidak dikirim");
  return false;
}

// Fungsi untuk reconnect WiFi
bool reconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }
  
  Serial.println("⚠ WiFi terputus, mencoba koneksi ulang...");
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi terhubung kembali!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("\n✗ Gagal koneksi WiFi");
    return false;
  }
}

// Task untuk membaca sensor PZEM
void readSensorTask(void *parameter) {
  PZEMData data;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(2000); // 2 detik
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // Baca semua parameter dari PZEM
    float voltage = pzem.voltage();
    float current = pzem.current();
    float power = pzem.power();
    float energy = pzem.energy();
    float frequency = pzem.frequency();
    float pf = pzem.pf();
    
    // Cek apakah pembacaan valid
    if (!isnan(voltage) && !isnan(current)) {
      data.voltage = voltage;
      data.current = current;
      data.power = power;
      data.energy = energy;
      data.frequency = frequency;
      data.powerFactor = pf;
      data.timestamp = millis();
      data.isValid = true;
      
      // Tampilkan data di Serial Monitor
      Serial.println("========== DATA PZEM-004T ==========");
      Serial.printf("Tegangan    : %.2f V\n", voltage);
      Serial.printf("Arus        : %.3f A\n", current);
      Serial.printf("Daya        : %.2f W\n", power);
      Serial.printf("Energi      : %.3f kWh\n", energy);
      Serial.printf("Frekuensi   : %.1f Hz\n", frequency);
      Serial.printf("Power Factor: %.2f\n", pf);
      Serial.println("====================================");
      
      // Cek apakah ada perubahan signifikan
      if (hasSignificantChange(data, lastSentData)) {
        // Kirim ke queue hanya jika ada perubahan signifikan
        if (xQueueSend(sensorQueue, &data, pdMS_TO_TICKS(100)) != pdPASS) {
          Serial.println("✗ Gagal mengirim ke queue");
        } else {
          Serial.println("✓ Data dikirim ke queue untuk transmisi");
        }
      }
      Serial.println();
      
    } else {
      Serial.println("✗ Gagal membaca sensor PZEM-004T");
      Serial.println("Periksa koneksi kabel dan alamat sensor\n");
      
      // Kirim data invalid ke queue
      data.isValid = false;
      data.timestamp = millis();
      xQueueSend(sensorQueue, &data, pdMS_TO_TICKS(100));
    }
  }
}

// Task untuk mengirim data ke Node-RED
void sendDataTask(void *parameter) {
  PZEMData data;
  
  while(1) {
    // Blok hingga ada data di queue
    if (xQueueReceive(sensorQueue, &data, portMAX_DELAY) == pdPASS) {
      
      // CEK WIFI HANYA SAAT AKAN MENGIRIM DATA
      Serial.println("\n>> Mempersiapkan pengiriman data...");
      Serial.println(">> Memeriksa koneksi WiFi...");
      
      if (!reconnectWiFi()) {
        Serial.println("✗ WiFi tidak terhubung, data tidak dikirim\n");
        continue;
      }
      
      Serial.println("✓ WiFi OK, mengirim data...");
      
      if(data.isValid) {
        HTTPClient http;
        
        http.begin(nodeRedURL);
        http.addHeader("Content-Type", "application/json");
        http.setTimeout(5000);
        
        // Buat JSON payload
        String jsonData = "{";
        jsonData += "\"voltage\":" + String(data.voltage, 2) + ",";
        jsonData += "\"current\":" + String(data.current, 3) + ",";
        jsonData += "\"power\":" + String(data.power, 2) + ",";
        jsonData += "\"energy\":" + String(data.energy, 3) + ",";
        jsonData += "\"frequency\":" + String(data.frequency, 1) + ",";
        jsonData += "\"powerFactor\":" + String(data.powerFactor, 2) + ",";
        jsonData += "\"timestamp\":" + String(data.timestamp);
        jsonData += "}";
        
        int httpResponseCode = http.POST(jsonData);
        
        if (httpResponseCode > 0) {
          String response = http.getString();
          Serial.printf("✓ Data terkirim ke Node-RED! Response: %d\n", httpResponseCode);
          Serial.println("Response: " + response);
          
          // Update data terakhir yang berhasil dikirim
          lastSentData = data;
          hasLastData = true;
        } else {
          Serial.printf("✗ Error mengirim data: %s\n", http.errorToString(httpResponseCode).c_str());
        }
        
        http.end();
        
      } else {
        // Data tidak valid, kirim dummy
        Serial.println("Data tidak valid, kirim dummy");
        
        HTTPClient http;
        
        http.begin(nodeRedURL);
        http.addHeader("Content-Type", "application/json");
        http.setTimeout(5000);

        String jsonData = "{";
        jsonData += "\"voltage\":" + String(7, 2) + ",";
        jsonData += "\"current\":" + String(7, 3) + ",";
        jsonData += "\"power\":" + String(7, 2) + ",";
        jsonData += "\"energy\":" + String(7, 3) + ",";
        jsonData += "\"frequency\":" + String(7, 1) + ",";
        jsonData += "\"powerFactor\":" + String(7, 2) + ",";
        jsonData += "\"timestamp\":" + String(data.timestamp);
        jsonData += "}";

        int httpResponseCode = http.POST(jsonData);
        
        if (httpResponseCode > 0) {
          String response = http.getString();
          Serial.printf("✓ Data dummy terkirim! Response: %d\n", httpResponseCode);
          Serial.println("Response: " + response);
        } else {
          Serial.printf("✗ Error mengirim data: %s\n", http.errorToString(httpResponseCode).c_str());
        }
        
        http.end();
      }
      
      Serial.println();
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  // Inisialisasi Serial2 untuk PZEM
  Serial2.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
  
  Serial.println("\n========================================");
  Serial.println("ESP32 PZEM-004T Tickless Mode");
  Serial.println("dengan Smart WiFi & Deteksi Perubahan");
  Serial.println("========================================");
  
  // Setup Power Management (Tickless)
  setupPowerManagement();
  
  // Koneksi WiFi awal
  Serial.println("\nMenghubungkan ke WiFi...");
  WiFi.begin(ssid, password);
  WiFi.setSleep(true); // Enable WiFi modem sleep
  
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi terhubung!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Node-RED URL: ");
    Serial.println(nodeRedURL);
    Serial.println("✓ WiFi modem sleep: ENABLED");
  } else {
    Serial.println("\n⚠ WiFi tidak terhubung saat startup");
    Serial.println("WiFi akan dicek otomatis saat mengirim data");
  }
  
  // Tampilkan konfigurasi threshold
  Serial.println("\n========== THRESHOLD SETTINGS ==========");
  Serial.printf("Voltage    : ±%.2f V\n", VOLTAGE_THRESHOLD);
  Serial.printf("Current    : ±%.3f A\n", CURRENT_THRESHOLD);
  Serial.printf("Power      : ±%.2f W\n", POWER_THRESHOLD);
  Serial.printf("Energy     : ±%.3f kWh\n", ENERGY_THRESHOLD);
  Serial.printf("Frequency  : ±%.1f Hz\n", FREQUENCY_THRESHOLD);
  Serial.printf("Power Factor: ±%.2f\n", PF_THRESHOLD);
  Serial.println("========================================");
  Serial.println("Data hanya dikirim jika ada perubahan signifikan!");
  Serial.println("========================================\n");
  
  // Buat queue untuk 10 data
  sensorQueue = xQueueCreate(10, sizeof(PZEMData));
  
  if(sensorQueue == NULL) {
    Serial.println("✗ Gagal membuat queue");
    return;
  }
  
  Serial.println("Membuat FreeRTOS tasks...");
  
  // Hanya buat 2 tasks (tanpa WiFi monitor task)
  xTaskCreatePinnedToCore(
    readSensorTask,
    "ReadPZEM",
    4096,
    NULL,
    2,      // Priority tinggi
    NULL,
    1       // Core 1
  );
  
  xTaskCreatePinnedToCore(
    sendDataTask,
    "SendData",
    8192,
    NULL,
    1,      // Priority sedang
    NULL,
    1       // Core 1
  );
  
  Serial.println("✓ Semua tasks berhasil dibuat!");
  Serial.println("========================================");
  Serial.println("FITUR AKTIF:");
  Serial.println("✓ Tickless dengan Automatic Light Sleep");
  Serial.println("✓ WiFi check on-demand (saat kirim data)");
  Serial.println("✓ Kirim data hanya jika ada perubahan");
  Serial.println("✓ WiFi auto-reconnect saat diperlukan");
  Serial.println("========================================\n");
}

void loop() {
  // Loop kosong, semua berjalan di FreeRTOS tasks
  // ESP32 akan otomatis masuk light sleep saat idle
  vTaskDelay(pdMS_TO_TICKS(1000));
}