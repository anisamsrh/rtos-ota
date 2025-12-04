#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_pm.h"
#include "esp_sleep.h"

// --- Konfigurasi ---
const char* ssid = "ITS-WIFI-TW2";
const char* password = "itssurabaya";

const String firmwareURL = "https://raw.githubusercontent.com/anisamsrh/rtos-ota/v2/firmware/firmware.bin";
const String versionURL = "https://raw.githubusercontent.com/anisamsrh/rtos-ota/v2/firmware/version.txt";
const String currentVersion = "1.0.4";
const char* nodeRedURL = "http://10.4.68.11:1880/sensor";

#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17
#define RELAYPIN 25

// Threshold Perubahan
const float VOLTAGE_THRESHOLD = 1.0;
const float CURRENT_THRESHOLD = 0.05;
const float POWER_THRESHOLD = 5.0;
const float ENERGY_THRESHOLD = 0.01;
const float FREQUENCY_THRESHOLD = 0.5;
const float PF_THRESHOLD = 0.05;

// Command Modbus PZEM (SlaveID 0x01, Read Input Register 0x04, 10 Register)
const uint8_t pzemRequestCmd[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x70, 0x0D};

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

QueueHandle_t sensorQueue;
PZEMData lastSentData = {0};
bool hasLastData = false;

// --- Helper Functions ---

void setupPowerManagement() {
  esp_pm_config_esp32_t pm_config = {
    .max_freq_mhz = 240,
    .min_freq_mhz = 80,
    .light_sleep_enable = true
  };
  esp_pm_configure(&pm_config);
}

bool reconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;
  
  Serial.println("⚠ Reconnecting WiFi...");
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    vTaskDelay(pdMS_TO_TICKS(500));
    attempts++;
  }
  return (WiFi.status() == WL_CONNECTED);
}

void updateFirmware() {
  WiFiClientSecure client;
  client.setInsecure(); 
  HTTPClient http;

  // 1. Cek Versi
  http.begin(client, versionURL);
  if (http.GET() == HTTP_CODE_OK) {
    String newVersion = http.getString();
    newVersion.trim();
    if (newVersion.equals(currentVersion)) {
      Serial.println("✓ Firmware Up-to-date.");
      http.end();
      return; 
    }
    Serial.printf("Update found: %s -> %s\n", currentVersion.c_str(), newVersion.c_str());
  } else {
    http.end();
    Serial.println("Cannot Access");
    return;
  }
  http.end();

  // 2. Proses Update
  httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  t_httpUpdate_return ret = httpUpdate.update(client, firmwareURL);

  if (ret == HTTP_UPDATE_OK) {
    Serial.println("Update Success! Restarting...");
    ESP.restart();
  } else {
    Serial.printf("Update Failed: %s\n", httpUpdate.getLastErrorString().c_str());
  }
}

bool hasSignificantChange(const PZEMData& newData, const PZEMData& oldData) {
  if (!hasLastData) return true;
  
  if (abs(newData.voltage - oldData.voltage) >= VOLTAGE_THRESHOLD ||
      abs(newData.current - oldData.current) >= CURRENT_THRESHOLD ||
      abs(newData.power - oldData.power) >= POWER_THRESHOLD ||
      abs(newData.energy - oldData.energy) >= ENERGY_THRESHOLD ||
      abs(newData.frequency - oldData.frequency) >= FREQUENCY_THRESHOLD ||
      abs(newData.powerFactor - oldData.powerFactor) >= PF_THRESHOLD) {
    return true;
  }
  return false;
}

// --- Tasks ---

void readSensorTask(void *parameter) {
  PZEMData data;
  const TickType_t xFrequency = pdMS_TO_TICKS(2000);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // Flush & Request
    while (Serial2.available()) Serial2.read();
    Serial2.write(pzemRequestCmd, 8);
    
    // Tunggu respon (max 100ms)
    unsigned long startTime = millis();
    while (Serial2.available() < 25 && millis() - startTime < 100) { vTaskDelay(1); }
    
    if (Serial2.available() >= 25) {
      uint8_t buffer[25];
      Serial2.readBytes(buffer, 25);
      
      // Validasi Header (SlaveID & FuncCode)
      if (buffer[0] == 0x01 && buffer[1] == 0x04) {
        // Parsing Data Manual (Big Endian to ESP32 Little Endian)
        data.voltage = ((buffer[3] << 8) | buffer[4]) * 0.1;
        
        // Current 32bit logic
        data.current = ((uint32_t)((buffer[6] | (buffer[5] << 8))) | 
                       ((uint32_t)(buffer[8] | (buffer[7] << 8)) << 16)) * 0.001;

        data.power = ((uint32_t)((buffer[10] | (buffer[9] << 8))) | 
                     ((uint32_t)(buffer[12] | (buffer[11] << 8)) << 16)) * 0.1;
        
        data.energy = ((uint32_t)((buffer[14] | (buffer[13] << 8))) | 
                      ((uint32_t)(buffer[16] | (buffer[15] << 8)) << 16)) * 1.0;
        
        data.frequency = ((buffer[17] << 8) | buffer[18]) * 0.1;
        data.powerFactor = ((buffer[19] << 8) | buffer[20]) * 0.01;
        
        data.isValid = true;
        data.timestamp = millis();

        Serial.printf("V:%.1f A:%.3f W:%.1f\n", data.voltage, data.current, data.power);

        if (hasSignificantChange(data, lastSentData)) {
           xQueueSend(sensorQueue, &data, pdMS_TO_TICKS(100));
        }
      } else {
         Serial.println("✗ Modbus Header Error");
      }
    } else {
      Serial.println("✗ PZEM Timeout");
    }
  }
}

void sendDataTask(void *parameter) {
  PZEMData data;
  char jsonBuffer[256]; 
  
  while(1) {
    if (xQueueReceive(sensorQueue, &data, portMAX_DELAY) == pdPASS) {
      if (!reconnectWiFi()) continue;
      
      // Persiapkan JSON string (Safe formatting)
      if (data.isValid) {
        snprintf(jsonBuffer, sizeof(jsonBuffer), 
            "{\"voltage\":%.2f,\"current\":%.3f,\"power\":%.2f,\"energy\":%.3f,\"freq\":%.1f,\"pf\":%.2f,\"ts\":%lu}",
            data.voltage, data.current, data.power, data.energy, data.frequency, data.powerFactor, data.timestamp);
      } else {
        // Dummy data jika invalid (tetap pakai snprintf)
        snprintf(jsonBuffer, sizeof(jsonBuffer), 
            "{\"voltage\":7.0,\"current\":7.0,\"power\":7.0,\"energy\":7.0,\"freq\":7.0,\"pf\":7.0,\"ts\":%lu}",
            data.timestamp);
      }

      // Kirim HTTP POST
      HTTPClient http;
      http.begin(nodeRedURL);
      http.addHeader("Content-Type", "application/json");
      http.setTimeout(5000);
      
      int httpCode = http.POST(jsonBuffer);
      
      if (httpCode > 0) {
        String response = http.getString();
        Serial.printf("✓ Sent! Code: %d, Resp: %s\n", httpCode, response.c_str());
        
        if (response == "RESTART") {
            Serial.println("Restart Command Received.");
            delay(500); 
            ESP.restart();
        }
        
        if (data.isValid) {
            lastSentData = data;
            hasLastData = true;
        }
      } else {
        Serial.printf("✗ Send Error: %s\n", http.errorToString(httpCode).c_str());
      }
      http.end();
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
  
  setupPowerManagement();
  
  // Koneksi WiFi Awal
  WiFi.begin(ssid, password);
  WiFi.setSleep(true); 
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500); attempts++;
  }

  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("✓ WiFi Connected");
    WiFi.setSleep(false); // Disable sleep sebentar untuk OTA
    updateFirmware(); 
    WiFi.setSleep(true);
  }

  sensorQueue = xQueueCreate(10, sizeof(PZEMData));
  
  // Create Tasks (Stack size disesuaikan untuk HTTP/SSL)
  xTaskCreatePinnedToCore(readSensorTask, "ReadPZEM", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(sendDataTask, "SendData", 12288, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelete(NULL); 
}