#ifndef CONNECTWIFI_H
#define CONNECTWIFI_H

#include <WiFi.h>

// Thay bằng SSID và PASSWORD của bạn
extern const char* WIFI_SSID;
extern const char* WIFI_PASSWORD;

// Cấu hình thời gian (ms)
const unsigned long CONNECT_TIMEOUT_MS = 10000;  // timeout cho lần thử kết nối ban đầu
const unsigned long RECONNECT_INTERVAL_MS = 5000; // khoảng chờ giữa các lần thử reconnect

void connectWiFi(); // Kết nối WiFi (blocking với timeout)
void handleWiFiReconnect(); // Gọi trong loop, tự reconnect nếu mất kết nối

#endif
