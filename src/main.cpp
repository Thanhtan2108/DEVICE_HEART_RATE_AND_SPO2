#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_task_wdt.h>
#include "max30102.h"
#include "oled.h"
#include "powerControl.h"

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
static const uint32_t WDT_TIMEOUT_S = 15; // Tăng timeout lên 15 giây

// Callback để tắt OLED khi hệ thống tắt
static void oledClearCallback() {
    Serial.println("Clearing OLED via callback...");
    display.clearDisplay();
    display.display();
    // Tắt hoàn toàn OLED
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    Serial.println("OLED turned off completely");
}

// THÊM CALLBACK FUNCTION - ĐẶT TRƯỚC setup()
static void systemStateChangedCallback(SystemState_t newState) {
    Serial.printf("MAIN: System state changed to: %s\n", 
                  powerControl_getStateString(newState));
    
    // Có thể thêm xử lý khác ở đây nếu cần
}

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
        // Kiểm tra trạng thái power trước khi xử lý
        if (!powerControl_getState()) {
            // Khi tắt, chỉ reset watchdog và delay, không làm gì cả
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(1000)); // Delay dài khi tắt
            continue;
        }

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
    
    // Hiển thị màn hình chờ ban đầu
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Bật OLED trước khi hiển thị
        display.ssd1306_command(SSD1306_DISPLAYON);
        oled_show_waiting();
        xSemaphoreGive(i2cMutex);
    }

    for (;;) {
        // Kiểm tra trạng thái power trước khi xử lý
        if (!powerControl_getState()) {
            // Khi hệ thống tắt, đảm bảo OLED tắt
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                display.clearDisplay();
                display.display();
                display.ssd1306_command(SSD1306_DISPLAYOFF);
                xSemaphoreGive(i2cMutex);
            }
            // Khi tắt, chỉ reset watchdog và delay, không làm gì cả
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(1000)); // Delay dài khi tắt
            continue;
        }

        // Khi hệ thống bật, đảm bảo OLED bật
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            display.ssd1306_command(SSD1306_DISPLAYON);
            xSemaphoreGive(i2cMutex);
        }

        // Nhận dữ liệu mới từ queue (nếu có)
        if (xQueueReceive(measurementQueue, &current, pdMS_TO_TICKS(50)) == pdPASS) {
            // Có dữ liệu mới, tiếp tục xử lý
        }

        // Hiển thị dữ liệu lên OLED
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
    delay(1000);
    Serial.println("\n\n=== Starting Heart Rate & SpO2 Monitor ===");
    Serial.println("System: Initializing...");

    // GIẢI QUYẾT LỖI I2C_BUFFER_LENGTH - đặt buffer size trước khi begin
    Wire.setBufferSize(128);
    
    // Khởi tạo I2C với chân ESP32
    Wire.begin(21, 22);
    Serial.println("I2C: Initialized (SDA=21, SCL=22)");

    // Tạo mutex cho I2C
    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == nullptr) {
        Serial.println("ERROR: Failed to create I2C mutex!");
        while (true) { delay(1000); }
    }
    Serial.println("I2C: Mutex created");

    // Khởi tạo OLED
    if (!oled_init()) {
        Serial.println("ERROR: Failed to initialize OLED!");
        while(1) { delay(1000); }
    }
    Serial.println("OLED: Initialized successfully");

    // Khởi tạo MAX30102
    if (!max30102_init()) {
        Serial.println("ERROR: Failed to initialize MAX30102!");
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            oled_show_error("MAX30102 not found!");
            xSemaphoreGive(i2cMutex);
        }
        while(1) { delay(1000); }
    }
    Serial.println("MAX30102: Initialized successfully");

    // Tạo queue cho measurements
    measurementQueue = xQueueCreate(1, sizeof(Measurement));
    if (measurementQueue == nullptr) {
        Serial.println("ERROR: Failed to create measurement queue!");
        while(true) { delay(1000); }
    }
    Serial.println("Queue: Measurement queue created");

    // Khởi tạo watchdog
    esp_task_wdt_init(WDT_TIMEOUT_S, true);
    Serial.println("Watchdog: Initialized");

    // Tạo sensor task
    BaseType_t sensorCreated = xTaskCreatePinnedToCore(
        sensorTask,
        "SensorTask",
        4096,
        nullptr,
        3,
        &sensorTaskHandle,
        1
    );

    // Tạo display task  
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
        Serial.println("ERROR: Failed to create tasks!");
        while(true) { delay(1000); }
    }
    Serial.println("Tasks: Sensor and Display tasks created");

    // === KHỞI TẠO POWER CONTROL MODULE ===
    PowerControlConfig powerConfig;
    powerConfig.sensorTaskHandle = &sensorTaskHandle;
    powerConfig.displayTaskHandle = &displayTaskHandle;
    powerConfig.i2cMutex = &i2cMutex;
    powerConfig.oledClearCallback = oledClearCallback;
    powerConfig.systemStateChangedCallback = systemStateChangedCallback; // ĐÃ SỬA LỖI
    
    if (!powerControl_init(&powerConfig)) {
        Serial.println("ERROR: Failed to initialize power control!");
        while(true) { delay(1000); }
    }
    Serial.println("PowerControl: Initialized successfully");
    
    // Khởi động power control task
    if (!powerControl_startTask()) {
        Serial.println("ERROR: Failed to start power control task!");
        while(true) { delay(1000); }
    }
    Serial.println("PowerControl: Task started");

    Serial.println("\n=== SYSTEM READY ===");
    Serial.println("Button: Use GPIO 5 to toggle power");
    Serial.println("LED: GPIO 4 shows power status (ON=HIGH, OFF=LOW)");
    Serial.println("Watchdog: Enabled with 15s timeout");
    Serial.println("==========================================\n");
}

void loop() {
    // Main loop không cần watchdog
    static uint32_t lastStatusTime = 0;
    uint32_t now = millis();
    
    if (now - lastStatusTime > 30000) { // Chỉ log mỗi 30 giây
        lastStatusTime = now;
        
        bool powerState = powerControl_getState();
        bool buttonState = digitalRead(POWER_BUTTON_GPIO);
        
        Serial.println("\n=== SYSTEM STATUS ===");
        Serial.print("Power: "); Serial.println(powerState ? "ON" : "OFF");
        Serial.print("Button: "); Serial.println(buttonState ? "HIGH (not pressed)" : "LOW (pressed)");
        Serial.print("CPU: "); Serial.print(getCpuFrequencyMhz()); Serial.println(" MHz");
        Serial.print("Heap: "); Serial.print(esp_get_free_heap_size()); Serial.println(" bytes");
        Serial.println("=====================\n");
    }
    
    vTaskDelay(pdMS_TO_TICKS(5000)); // Giảm tần suất loop
}
