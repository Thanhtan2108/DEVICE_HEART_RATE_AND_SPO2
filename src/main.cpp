#include <Arduino.h>
#include <Wire.h>
#include "Max30102Sensor.h"
#include "ConnectToFirebase.h"
#include "DisplayOLED.h"
#include "ButtonPower.h"
#include "config.h"

// ESP32 typical I2C pins on devkit (adjust if you wired differently)
#define SDA_PIN 21
#define SCL_PIN 22

ConnectToFirebase firebase;
Max30102Sensor sensor;
DisplayOLED oled;
ButtonPower powerCtl;

bool fingerPresent = false;

void setup() {
  Serial.begin(115200);
  delay(500); // đảm bảo ổn định Serial

  // --- Kết nối WiFi + Firebase ---
  if (!firebase.begin()) {
    Serial.println("❌ Firebase init failed! Check WiFi or credentials.");
  } else {
    Serial.println("✅ Firebase connected successfully!");
  }

  Serial.println("\n=== MAX30102 Heart Rate & SpO2 Monitor (ESP32) ===");

  // --- Khởi tạo I2C ---
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(200);

  // --- OLED ---
  if (!oled.begin(OLED_ADDR)) {
    Serial.println("⚠️ OLED init failed!");
  } else {
    oled.showStatus("OLED init successfully");
  }

  // --- Khởi tạo cảm biến MAX30102 ---
  if (!sensor.begin(Wire, MAX30102_ADDR)) {
    Serial.println("❌ MAX30102 not found! Check wiring.");
    while (1) delay(500);
  }

  sensor.setIRThreshold(5000);

  // --- Power Control ---
  powerCtl.begin(false); // start inactive/off
  oled.powerOff();
  sensor.powerOff();

  Serial.println("✅ Place your finger on the sensor to start measuring...");
}

void loop() {
  powerCtl.loop();

  // Handle power state transitions via module API
  bool active = powerCtl.ensureActive(&oled, &sensor);

  if (!active) {
    delay(50);
    return;
  }

  sensor.update();

  // --- Lấy dữ liệu đo ---
  int bpm = (int) round(sensor.getBPM());       // làm tròn và ép kiểu int
  float spo2 = round(sensor.getSpO2() * 10) / 10.0f; // 1 chữ số thập phân

  // --- Hiển thị theo trạng thái ngón tay ---
  if (sensor.isFingerDetected()) {
    oled.showVitals(bpm, spo2);
  } else {
    // Không hiển thị giá trị cũ, nhắc đặt ngón tay
    oled.showStatus("Place finger", "to measure");
  }

  // --- Gửi dữ liệu local lên Firebase khi có ngón tay ---
  if (sensor.isFingerDetected()) {
    firebase.sendData(bpm, spo2);
  }

  delay(20); // vòng lặp 50Hz
}
