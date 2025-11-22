#include <Arduino.h>
#include <ConnectWiFi.h>
#include <ScanI2C.h>

#define SDA_PIN 21
#define SCL_PIN 22

const char* WIFI_SSID     = "P203&P204&P205";
const char* WIFI_PASSWORD = "102duylam";

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.print("\n");
    Serial.println("[==========START PROJECT==========]");
    I2C_Init(SDA_PIN, SCL_PIN);
    I2C_ScanAddress();

    WiFi_Connect(WIFI_SSID, WIFI_PASSWORD);
}

void loop() {
    WiFi_Reconnect(WIFI_SSID, WIFI_PASSWORD);
    
}
