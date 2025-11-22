#include <Arduino.h>
#include <unity.h>
#include "ConnectWiFi.h"

const char* WIFI_SSID     = "P203&P204&P205";
const char* WIFI_PASSWORD = "102duylam";

// Test kết nối WiFi blocking
void test_WiFi_Connect() {
    WiFi_Connect(WIFI_SSID, WIFI_PASSWORD); // Gọi hàm connect blocking

    // Kiểm tra trạng thái
    TEST_ASSERT_TRUE_MESSAGE(WiFi.status() == WL_CONNECTED, "WiFi should be connected");

    // Kiểm tra IP không rỗng
    String ip = WiFi.localIP().toString();
    TEST_ASSERT_TRUE_MESSAGE(ip.length() > 0, "WiFi IP should not be empty");

    Serial.printf("Connected IP: %s\n", ip.c_str());
}

// Test reconnect tự động khi mất kết nối
void test_WiFi_Reconnect() {
    // Giả lập mất kết nối
    WiFi.disconnect(true);
    delay(1000); // chờ một chút cho WiFi disconnect

    // Lần đầu gọi handle → sẽ trigger reconnect nếu đủ thời gian
    WiFi_Reconnect(WIFI_SSID, WIFI_PASSWORD);
    
    // Chờ đủ CONNECT_TIMEOUT_MS + 500ms để connect xong
    delay(CONNECT_TIMEOUT_MS + 500);

    WiFi_Reconnect(WIFI_SSID, WIFI_PASSWORD); // đảm bảo gọi handle sau reconnect

    TEST_ASSERT_TRUE_MESSAGE(WiFi.status() == WL_CONNECTED, "WiFi should reconnect");

    Serial.printf("Reconnected IP: %s\n", WiFi.localIP().toString().c_str());
}

void setup() {
    Serial.begin(115200);
    delay(200); // cho Serial sẵn sàng

    UNITY_BEGIN();
    RUN_TEST(test_WiFi_Connect);
    RUN_TEST(test_WiFi_Reconnect);
    UNITY_END();
}

void loop() {
    // Không cần làm gì trong loop
}
