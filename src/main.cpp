// ======================Library default of Framework================= //
#include <Arduino.h>

// =====================Library install or self-development================= //
#include <ToggleGPIO.h>
#include <ScanI2C.h>

// =========================Define, config========================= //
// I2C
#define SDA_PIN 21
#define SCL_PIN 22

// GPIO
#define LED_PIN 2

// =========================Variable global========================= //


// ==========================Init object=========================== //


// =========================Function prototype==================== //


// ==========================Init program run once================ //
void setup() {
    // initialize Serial monitor to debug
    Serial.begin(115200);
    delay(1000);

    // setup function of GPIO and logic lenivel itial of GPIO //
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Function initiallize object from Library Self-development
    I2C_Init(SDA_PIN, SCL_PIN);

    // Function implement task once
    I2C_ScanAddress();
}

// ======================Init program run forever================ //
void loop() {
    // implement task of program
    digitalTogglePin(LED_PIN);
}

// =========================Build function======================= //
