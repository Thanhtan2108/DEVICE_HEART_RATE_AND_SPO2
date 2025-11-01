#ifndef CONNECT_TO_FIREBASE_H
#define CONNECT_TO_FIREBASE_H

#include <Arduino.h>
#include <WiFi.h>                // Dành cho ESP32
#include <time.h>                // NTP time
#include <Firebase_ESP_Client.h>
#include "config.h"

class ConnectToFirebase {
public:
  ConnectToFirebase();

  // === API chính ===
  bool begin();                               // Khởi tạo WiFi + Firebase + NTP
  bool isConnected();                         // Kiểm tra kết nối
  bool sendData(float bpm, float spo2);       // Gửi dữ liệu lên Firebase
  bool readData(int &bpmOut, float &spo2Out); // Đọc dữ liệu từ Firebase

private:
  // === Helper nội bộ ===
  void connectWiFi();
  void syncNTP();
  void connectFirebase();
  String buildPath(const String &key) const;  // tạo đường dẫn như "/BPM"

  // === Firebase components ===
  FirebaseData fbdo;
  FirebaseAuth auth;
  FirebaseConfig fbConfig;

  // === Cấu hình hệ thống ===
  static constexpr unsigned long FIREBASE_REPORT_INTERVAL_MS = 3000UL;
  unsigned long lastSend = 0;
  unsigned long lastReadMs = 0;

  // === Đường dẫn gốc trên Firebase ===
  // Đặt "/" để lưu ngay root
  const String FIREBASE_PATH_ROOT = "/";
};

#endif // CONNECT_TO_FIREBASE_H
