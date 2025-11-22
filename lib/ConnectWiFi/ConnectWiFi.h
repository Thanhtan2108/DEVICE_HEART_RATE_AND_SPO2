#ifndef CONNECTWIFI_H
#define CONNECTWIFI_H

#include <WiFi.h>

// Cấu hình thời gian (ms)
const unsigned long CONNECT_TIMEOUT_MS = 10000;  // timeout cho lần thử kết nối ban đầu
const unsigned long RECONNECT_INTERVAL_MS = 5000; // khoảng chờ giữa các lần thử reconnect

void WiFi_Connect(const char* WIFI_SSID, const char* WIFI_PASSWORD); // Kết nối WiFi (blocking với timeout)
void WiFi_Reconnect(const char* WIFI_SSID, const char* WIFI_PASSWORD); // Gọi trong loop, tự reconnect nếu mất kết nối

#endif
