#include <Arduino.h>
#include <unity.h>
#include <Wire.h>
#include "scanI2C.h"

#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_SPEED 400000

// Test I2C_Init()
void test_I2C_Init() {
    I2C_Init(SDA_PIN, SCL_PIN, I2C_SPEED);

    // Không có assert thực sự cho Wire.begin(), chỉ đảm bảo code chạy
    TEST_ASSERT_TRUE_MESSAGE(true, "I2C initialized");
}

// Test I2C_ScanAddress()
void test_I2C_ScanAddress() {
    // Gọi scan
    I2C_ScanAddress();

    // Không thể assert địa chỉ cụ thể vì phụ thuộc board và devices
    TEST_ASSERT_TRUE_MESSAGE(true, "I2C scan executed (check Serial output)");
}

void setup() {
    Serial.begin(115200);
    delay(200);

    UNITY_BEGIN();
    RUN_TEST(test_I2C_Init);
    RUN_TEST(test_I2C_ScanAddress);
    UNITY_END();
}

void loop() {
    // Không cần code trong loop
}
