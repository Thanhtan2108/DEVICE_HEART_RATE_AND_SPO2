#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ConnectWiFi.h"
#include "ScanI2C.h"
#include "OledSSD1306.h"

#define SDA_PIN 21
#define SCL_PIN 22

const char* WIFI_SSID     = "P203&P204&P205";
const char* WIFI_PASSWORD = "102duylam";

// --- Task handles
TaskHandle_t TaskI2CHandle   = NULL;
TaskHandle_t TaskWiFiHandle  = NULL;
TaskHandle_t TaskOLEDHandle  = NULL;

// --- Task I2C
void TaskI2C(void *pvParameters) {
    (void) pvParameters;

    // setup stage
    I2C_Init(SDA_PIN, SCL_PIN);
    I2C_ScanAddress();

    // loop stage
    while(1) {
        // I2C task không cần làm gì, chỉ cho phép I2C hoạt động
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// --- Task WiFi
void TaskWiFi(void *pvParameters) {
    (void) pvParameters;

    // Kết nối WiFi lần đầu
    WiFi_Connect(WIFI_SSID, WIFI_PASSWORD);

    while(1) {
        WiFi_Reconnect(WIFI_SSID, WIFI_PASSWORD);    // kiểm tra và reconnect nếu cần
        vTaskDelay(pdMS_TO_TICKS(5000)); // check mỗi 5s
    }
}

// --- Task OLED Display
void TaskOLED(void *pvParameters) {
    (void) pvParameters;

    // setup stage - init OLED
    OLEDSSD1306_Init(SDA_PIN, SCL_PIN, 0x3C);

    // loop stage - display data continuously
    int displayCounter = 0;
    while(1) {
        // Clear display for fresh update
        OLEDSSD1306_ClearDisplay();
        
        // Display title
        OLEDSSD1306_SetLocation(4, 1);
        OLEDSSD1306_SetTextSize(1);
        OLEDSSD1306_DisplayNotify("Heart Rate And SpO2");

        // Display data below title
        OLEDSSD1306_SetLocation(0, 20);
        OLEDSSD1306_SetTextSize(1.5);
        OLEDSSD1306_DisplayData("Heart Rate : ", 60 + displayCounter);
        OLEDSSD1306_DisplayData("SpO2 : ", 95 - displayCounter);
        
        // Update display once
        OLEDSSD1306_UpdateDisplay();
        
        displayCounter++;
        vTaskDelay(pdMS_TO_TICKS(500)); // update display every 500ms
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("[==========START PROJECT==========]");

    // --- Tạo task I2C
    xTaskCreate(
        TaskI2C,        // function
        "I2C Task",     // name
        4096,           // stack size (bytes)
        NULL,           // parameter
        1,              // priority
        &TaskI2CHandle  // task handle
    );

    // --- Tạo task WiFi
    xTaskCreate(
        TaskWiFi,
        "WiFi Task",
        4096,
        NULL,
        1,
        &TaskWiFiHandle
    );

    // --- Tạo task OLED
    xTaskCreate(
        TaskOLED,
        "OLED Task",
        4096,
        NULL,
        1,
        &TaskOLEDHandle
    );
}

void loop() {
    // Không cần code gì trong loop
    vTaskDelay(pdMS_TO_TICKS(1000)); // tránh loop chạy liên tục
}
