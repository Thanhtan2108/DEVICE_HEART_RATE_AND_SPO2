#include <Arduino.h>
#include "ConnectWiFi.h"
#include "ScanI2C.h"
#include "OledSSD1306.h"

#define SDA_PIN 21
#define SCL_PIN 22

const char* WIFI_SSID     = "P203&P204&P205";
const char* WIFI_PASSWORD = "102duylam";

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n[==========START PROJECT==========]");

    I2C_Init(SDA_PIN, SCL_PIN);
    I2C_ScanAddress();

    WiFi_Connect(WIFI_SSID, WIFI_PASSWORD);

    // Init OLED with SDA,SCL,address
    OLEDSSD1306_Init(SDA_PIN, SCL_PIN, 0x3C);
}

void loop() {
    WiFi_Reconnect(WIFI_SSID, WIFI_PASSWORD); // assume this uses stored credentials

    for (int i = 0 ; i < 2 ; i++) {
        for (int j = 0 ; j < 3 ; j++) {
            // Clear display for fresh update
            OLEDSSD1306_ClearDisplay();
            
            // Display title
            OLEDSSD1306_SetLocation(4, 1); // col 4 , row 1 (pixel)
            OLEDSSD1306_SetTextSize(1);
            OLEDSSD1306_DisplayNotify("Heart Rate And SpO2");

            // Display data below title
            OLEDSSD1306_SetLocation(4, 20); // col 0, row 20 (pixel)
            OLEDSSD1306_SetTextSize(1.5);
            OLEDSSD1306_DisplayData("Heart Rate : ", 60 + i);
            OLEDSSD1306_DisplayData("SpO2 : ", 95 - j);
            
            // Update display once at the end (no flicker)
            OLEDSSD1306_UpdateDisplay();            
            delay(500);
        }
    }
}
