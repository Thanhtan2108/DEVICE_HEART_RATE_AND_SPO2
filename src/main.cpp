#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_task_wdt.h>
#include "max30102.h"
#include "oled.h"

struct Measurement {
    int heart_rate;
    int spo2;
    long ir_value;
    long red_value;
    bool has_finger;
    bool hr_valid;
    bool spo2_valid;
};

static QueueHandle_t measurementQueue = nullptr;
static SemaphoreHandle_t i2cMutex = nullptr;
static TaskHandle_t sensorTaskHandle = nullptr;
static TaskHandle_t displayTaskHandle = nullptr;

static const TickType_t SENSOR_TASK_DELAY = pdMS_TO_TICKS(40);
static const TickType_t DISPLAY_TASK_DELAY = pdMS_TO_TICKS(200);
static const uint32_t WDT_TIMEOUT_S = 8;

void logMeasurement(const Measurement& data) {
    Serial.print("IR: "); Serial.print(data.ir_value);
    Serial.print(" | Red: "); Serial.print(data.red_value);
    Serial.print(" | HR: "); Serial.print(data.heart_rate);
    Serial.print(" (valid: "); Serial.print(data.hr_valid);
    Serial.print(") | SpO2: "); Serial.print(data.spo2);
    Serial.print(" (valid: "); Serial.print(data.spo2_valid);
    Serial.print(") | Finger: "); Serial.println(data.has_finger ? "Y" : "N");
}

void sensorTask(void* pvParameters) {
    (void)pvParameters;
    esp_task_wdt_add(NULL);

    Measurement latest = {};

    for (;;) {
        long ir_value = 0;
        long red_value = 0;
        bool newSample = false;

        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            newSample = max30102_read_data(&ir_value, &red_value);
            xSemaphoreGive(i2cMutex);
        }

        if (newSample) {
            latest.ir_value = ir_value;
            latest.red_value = red_value;
            latest.has_finger = max30102_has_finger(ir_value);
            latest.heart_rate = latest.has_finger ? max30102_get_heart_rate(ir_value) : 0;
            latest.spo2 = latest.has_finger ? max30102_get_spo2(ir_value, red_value) : 0;
            latest.hr_valid = max30102_is_heart_rate_valid();
            latest.spo2_valid = max30102_is_spo2_valid();

            xQueueOverwrite(measurementQueue, &latest);
            logMeasurement(latest);
        }

        esp_task_wdt_reset();
        vTaskDelay(SENSOR_TASK_DELAY);
    }
}

void displayTask(void* pvParameters) {
    (void)pvParameters;
    esp_task_wdt_add(NULL);

    Measurement current = {};
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        oled_show_waiting();
        xSemaphoreGive(i2cMutex);
    }

    for (;;) {
        if (xQueueReceive(measurementQueue, &current, pdMS_TO_TICKS(100)) == pdPASS) {
            // new data received
        }

        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            oled_show_data(current.heart_rate, current.spo2, current.ir_value, current.has_finger);
            xSemaphoreGive(i2cMutex);
        }

        esp_task_wdt_reset();
        vTaskDelay(DISPLAY_TASK_DELAY);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting Heart Rate & SpO2 Monitor...");

    Wire.begin(21, 22); // ESP32 DOIT: SDA=21, SCL=22

    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == nullptr) {
        Serial.println("Failed to create I2C mutex!");
        while (true) { delay(1000); }
    }

    if (!oled_init()) {
        Serial.println("Failed to initialize OLED!");
        while(1) { delay(1000); }
    }

    if (!max30102_init()) {
        oled_show_error("MAX30102 not found!");
        Serial.println("Failed to initialize MAX30102!");
        while(1) { delay(1000); }
    }

    measurementQueue = xQueueCreate(1, sizeof(Measurement));
    if (measurementQueue == nullptr) {
        Serial.println("Failed to create measurement queue!");
        while(true) { delay(1000); }
    }

    esp_task_wdt_init(WDT_TIMEOUT_S, true);

    BaseType_t sensorCreated = xTaskCreatePinnedToCore(
        sensorTask,
        "SensorTask",
        4096,
        nullptr,
        3,
        &sensorTaskHandle,
        1
    );

    BaseType_t displayCreated = xTaskCreatePinnedToCore(
        displayTask,
        "DisplayTask",
        4096,
        nullptr,
        2,
        &displayTaskHandle,
        1
    );

    if (sensorCreated != pdPASS || displayCreated != pdPASS) {
        Serial.println("Failed to create tasks!");
        while(true) { delay(1000); }
    }

    Serial.println("System ready! Tasks started.");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
