#ifndef FIREBASEUPLOAD_H
#define FIREBASEUPLOAD_H

// Link DRTDB: https://console.firebase.google.com/project/heart-rate-and-spo2-78e6f/database/heart-rate-and-spo2-78e6f-default-rtdb/data?fb_gclid=CjwKCAiA8vXIBhAtEiwAf3B-gzDpvHKH2u4PbxZVxgiazZNWHXdaMmdmN9DKhYXLGxfRSAQtwdFs9BoCCl0QAvD_BwE

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <string.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>

// Wifi
static const char WIFI_SSID[]     = "P203&P204&P205";
static const char WIFI_PASSWORD[] = "102duylam";

// URL realtime database
#define FIREBASE_DATABASE_URL "https://heart-rate-and-spo2-78e6f-default-rtdb.asia-southeast1.firebasedatabase.app/"

// Database secret - Dùng cho không có app hay web
#define FIREBASE_AUTH "YA9wnCYcl705LqGr13GTImQLGmTAit1FA6DEv9MG"

/* Khuyến nghị nên dùng để kết nối Firebase khi có kết nối app hoặc web
#define FIREBASE_API_KEY ""

Dùng Email + Password (KHUYẾN CÁO - bảo mật tốt nhất) - Dùng khi có app hoặc web
#define FIREBASE_USER_EMAIL     "esp32@yourproject.com"   // Email bạn tạo trong Authentication
#define FIREBASE_USER_PASSWORD  "12345678"                // Mật khẩu >= 6 ký tự
*/

// Path root
const String FIREBASE_PATH_ROOT = "/";

// Đối tượng chứa dữ liệu giao tiếp với Firebase
extern FirebaseData fbdo;

// Cấu hình dự án Firebase
extern FirebaseAuth auth;

// Config kết nối
extern FirebaseConfig config;

// ===== FreeRTOS Kernel Objects =====
// Cấu trúc dữ liệu cho queue
typedef struct {
    int heartRate;
    int spo2;
    TickType_t timestamp;
} SensorData_t;

// Queue để gửi dữ liệu từ task sensor sang task Firebase
extern QueueHandle_t firebaseDataQueue;

// Task handle
extern TaskHandle_t firebaseTaskHandle;

// ===== Function APIs =====
// API kết nối Wifi
void connectToWifi();
// API kết nối Firebase
void connectToFirebase();
// API chỉ định đường dẫn trên Firebase RTDB
String buildPath(const String &key);
// API ghi dữ liệu nhịp tim và SpO2 lên Firabse
void sendDataToRTDB(int heartRate, int spo2);
// API khởi tạo FreeRTOS task upload Firebase
void firebaseUploadInit();
// API gửi dữ liệu lên queue để task xử lý
bool sendSensorDataToFirebaseQueue(int heartRate, int spo2, TickType_t ticksToWait = portMAX_DELAY);
// API điều khiển tạm dừng/tiếp tục Firebase task
void firebaseUploadSetEnabled(bool enabled);
// API lấy trạng thái Firebase task
bool firebaseUploadIsEnabled();

#endif
