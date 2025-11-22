#include <Arduino.h>
#include <unity.h>
#include "ConnectWiFi.h"

// Test kết nối WiFi blocking
void test_connectWiFi_blocking() {
    connectWiFi(); // Gọi hàm connect blocking

    // Kiểm tra trạng thái
    TEST_ASSERT_TRUE_MESSAGE(WiFi.status() == WL_CONNECTED, "WiFi should be connected");

    // Kiểm tra IP không rỗng
    String ip = WiFi.localIP().toString();
    TEST_ASSERT_TRUE_MESSAGE(ip.length() > 0, "WiFi IP should not be empty");

    Serial.printf("Connected IP: %s\n", ip.c_str());
}

// Test reconnect tự động khi mất kết nối
void test_handleWiFiReconnect() {
    // Giả lập mất kết nối
    WiFi.disconnect(true);
    delay(1000); // chờ một chút cho WiFi disconnect

    // Lần đầu gọi handle → sẽ trigger reconnect nếu đủ thời gian
    handleWiFiReconnect();
    
    // Chờ đủ CONNECT_TIMEOUT_MS + 500ms để connect xong
    delay(CONNECT_TIMEOUT_MS + 500);

    handleWiFiReconnect(); // đảm bảo gọi handle sau reconnect

    TEST_ASSERT_TRUE_MESSAGE(WiFi.status() == WL_CONNECTED, "WiFi should reconnect");

    Serial.printf("Reconnected IP: %s\n", WiFi.localIP().toString().c_str());
}

void setup() {
    Serial.begin(115200);
    delay(200); // cho Serial sẵn sàng

    UNITY_BEGIN();
    RUN_TEST(test_connectWiFi_blocking);
    RUN_TEST(test_handleWiFiReconnect);
    UNITY_END();
}

void loop() {
    // Không cần làm gì trong loop
}
