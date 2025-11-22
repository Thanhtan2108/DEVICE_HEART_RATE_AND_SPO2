#include "ConnectWiFi.h"

// Thay bằng SSID và PASSWORD của bạn
const char* WIFI_SSID     = "P203&P204&P205";
const char* WIFI_PASSWORD = "102duylam";

static unsigned long lastReconnectAttempt = 0; // biến private của file

void connectWiFi() {
    Serial.println("\n📶 Connecting to WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Time out cho làn kết nối đầu tiên
    unsigned long startTime = millis();

    while (WiFi.status() != WL_CONNECTED &&
           (millis() - startTime < CONNECT_TIMEOUT_MS))
    {
        Serial.print(".");
        delay(500);
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ WiFi connected!");
        Serial.print("📡 IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\n❌ WiFi connection failed (timeout)!");
    }
}

void handleWiFiReconnect() {
    // Nếu đang kết nối → không làm gì
    if (WiFi.status() == WL_CONNECTED) return;

    // Mất kết nối
    // Nếu chưa đến thời gian thử lại
    if (millis() - lastReconnectAttempt < RECONNECT_INTERVAL_MS) return;

    // Đủ 5s để kết nối lại
    lastReconnectAttempt = millis();

    Serial.println("\n🔄 WiFi disconnected — retrying...");
    connectWiFi();
}
