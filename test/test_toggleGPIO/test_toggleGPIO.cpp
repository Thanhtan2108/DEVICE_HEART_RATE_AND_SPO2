#include <Arduino.h>
#include <unity.h>
#include "ToggleGPIO.h"

#define LED_PIN 2

void test_digitalTogglePin() {
    digitalTogglePin(LED_PIN);

    TEST_ASSERT_TRUE_MESSAGE(true, "toggle pin successful");
}

void setup() {
    Serial.begin(115200);
    delay(200);

    UNITY_BEGIN();
    RUN_TEST(test_digitalTogglePin);
    UNITY_END();
}

void loop() {
    // Không cần code trong loop
}
