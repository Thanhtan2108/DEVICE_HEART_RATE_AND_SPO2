#include <Arduino.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include "max30102.h"
#include "oled.h"
#include "firebaseUpload.h"
#include "powerControl.h"

// ===== FreeRTOS Objects =====
SemaphoreHandle_t i2cMutex = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;

// ===== Configuration =====
#define SENSOR_TASK_PRIORITY 3
#define DISPLAY_TASK_PRIORITY 2
#define SENSOR_TASK_STACK_SIZE 4096
#define DISPLAY_TASK_STACK_SIZE 4096

#define UPLOAD_INTERVAL_MS 5000  // Upload mỗi 5 giây
#define DISPLAY_UPDATE_INTERVAL_MS 1000  // Update display mỗi 1 giây

// ===== Global State =====
static int g_heartRate = 0;
static int g_spo2 = 0;
static long g_irValue = 0;
static bool g_hasFingerDetected = false;
static uint32_t g_lastUploadTime = 0;

// ===== System State Callback =====
void onSystemStateChanged(SystemState_t newState) {
    Serial.printf("📌 Main: System state changed to %s\n", 
                 powerControl_getStateString(newState));
    
    if (newState == SYSTEM_STATE_OFF) {
        // Tắt các module không cần thiết
        firebaseUploadSetEnabled(false);
        oled_turn_off();
        Serial.println("💤 System entering low-power mode");
    } else if (newState == SYSTEM_STATE_ON) {
        // Bật lại các module
        firebaseUploadSetEnabled(true);
        oled_turn_on();
        oled_show_waiting();
        Serial.println("🚀 System resuming normal operation");
    }
}

// ===== Sensor Task =====
void sensorTask(void *pvParameters) {
    (void)pvParameters;
    
    Serial.println("🔬 Sensor Task started");
    
    // Thêm vào watchdog
    esp_task_wdt_add(NULL);
    
    long irValue = 0;
    long redValue = 0;
    uint32_t lastReadTime = 0;
    const uint32_t READ_INTERVAL_MS = 100; // Đọc sensor mỗi 100ms
    
    while (1) {
        uint32_t currentTime = millis();
        
        // Chỉ đọc sensor khi hệ thống ON
        if (powerControl_getSystemState() == SYSTEM_STATE_ON) {
            
            if (currentTime - lastReadTime >= READ_INTERVAL_MS) {
                lastReadTime = currentTime;
                
                // Lấy I2C mutex trước khi đọc
                if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    
                    // Đọc dữ liệu từ sensor
                    bool dataUpdated = max30102_read_data(&irValue, &redValue);
                    
                    if (dataUpdated) {
                        // Kiểm tra ngón tay
                        bool hasFingerNow = max30102_has_finger(irValue);
                        
                        // Tính toán HR và SpO2
                        int hrValue = max30102_get_heart_rate(irValue);
                        int spo2Value = max30102_get_spo2(irValue, redValue);
                        
                        // Cập nhật global state (thread-safe với mutex)
                        g_irValue = irValue;
                        g_hasFingerDetected = hasFingerNow;
                        
                        if (hasFingerNow && max30102_is_heart_rate_valid()) {
                            g_heartRate = hrValue;
                        }
                        if (hasFingerNow && max30102_is_spo2_valid()) {
                            g_spo2 = spo2Value;
                        }
                        
                        // Upload lên Firebase với interval
                        if (hasFingerNow && 
                            hrValue > 0 && 
                            spo2Value > 0 &&
                            (currentTime - g_lastUploadTime >= UPLOAD_INTERVAL_MS)) {
                            
                            // Gửi vào queue (không block)
                            if (sendSensorDataToFirebaseQueue(hrValue, spo2Value, 0)) {
                                g_lastUploadTime = currentTime;
                                Serial.printf("📊 Queued: HR=%d, SpO2=%d\n", hrValue, spo2Value);
                            }
                        }
                    }
                    
                    xSemaphoreGive(i2cMutex);
                }
            }
        }
        
        // Reset watchdog
        esp_task_wdt_reset();
        
        // Delay để không chiếm hết CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ===== Display Task =====
void displayTask(void *pvParameters) {
    (void)pvParameters;
    
    Serial.println("🖥️ Display Task started");
    
    // Thêm vào watchdog
    esp_task_wdt_add(NULL);
    
    uint32_t lastUpdateTime = 0;
    
    while (1) {
        uint32_t currentTime = millis();
        
        // Chỉ update display khi hệ thống ON
        if (powerControl_getSystemState() == SYSTEM_STATE_ON) {
            
            if (currentTime - lastUpdateTime >= DISPLAY_UPDATE_INTERVAL_MS) {
                lastUpdateTime = currentTime;
                
                // Lấy I2C mutex
                if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    
                    if (g_hasFingerDetected) {
                        oled_show_data(g_heartRate, g_spo2, g_irValue, g_hasFingerDetected);
                    } else {
                        oled_show_waiting();
                    }
                    
                    xSemaphoreGive(i2cMutex);
                }
            }
        }
        
        // Reset watchdog
        esp_task_wdt_reset();
        
        // Delay để không chiếm hết CPU
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ===== Setup =====
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n=================================");
    Serial.println("   Heart Rate & SpO2 Monitor");
    Serial.println("=================================\n");
    
    // Cấu hình watchdog timeout (10 giây)
    esp_task_wdt_init(10, true);
    
    // Tạo I2C mutex
    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL) {
        Serial.println("❌ Failed to create I2C mutex!");
        while(1) delay(1000);
    }
    Serial.println("✅ I2C Mutex created");
    
    // Khởi tạo I2C
    Wire.begin();
    Wire.setClock(400000); // 400kHz Fast Mode
    Serial.println("✅ I2C initialized");
    
    // Khởi tạo OLED
    if (!oled_init()) {
        Serial.println("❌ OLED initialization failed!");
        while(1) delay(1000);
    }
    oled_show_waiting();
    
    // Khởi tạo MAX30102
    if (!max30102_init()) {
        Serial.println("❌ MAX30102 initialization failed!");
        oled_show_error("MAX30102 Error");
        while(1) delay(1000);
    }
    
    // Khởi tạo Firebase (sẽ tạo task riêng)
    Serial.println("\n📡 Initializing Firebase...");
    firebaseUploadInit();
    
    // Đợi một chút để Firebase kết nối
    delay(2000);
    
    // Cấu hình Power Control
    PowerControlConfig powerConfig;
    powerConfig.sensorTaskHandle = &sensorTaskHandle;
    powerConfig.displayTaskHandle = &displayTaskHandle;
    powerConfig.i2cMutex = &i2cMutex;
    powerConfig.oledClearCallback = oled_turn_off;
    powerConfig.systemStateChangedCallback = onSystemStateChanged;
    
    if (!powerControl_init(&powerConfig)) {
        Serial.println("❌ Power Control initialization failed!");
        while(1) delay(1000);
    }
    
    // Tạo Sensor Task (Priority cao)
    xTaskCreatePinnedToCore(
        sensorTask,
        "SensorTask",
        SENSOR_TASK_STACK_SIZE,
        NULL,
        SENSOR_TASK_PRIORITY,
        &sensorTaskHandle,
        1  // Core 1
    );
    
    // Tạo Display Task (Priority thấp hơn)
    xTaskCreatePinnedToCore(
        displayTask,
        "DisplayTask",
        DISPLAY_TASK_STACK_SIZE,
        NULL,
        DISPLAY_TASK_PRIORITY,
        &displayTaskHandle,
        1  // Core 1
    );
    
    // Start Power Control Task
    if (!powerControl_startTask()) {
        Serial.println("❌ Power Control task failed!");
        while(1) delay(1000);
    }
    
    Serial.println("\n✅ All systems initialized!");
    Serial.println("=================================\n");
    
    // Thêm loop task vào watchdog
    esp_task_wdt_add(NULL);
}

// ===== Loop =====
void loop() {
    // Loop task chỉ để monitor và reset watchdog
    static uint32_t lastStatusPrint = 0;
    uint32_t currentTime = millis();
    
    if (currentTime - lastStatusPrint >= 30000) { // Mỗi 30 giây
        lastStatusPrint = currentTime;
        
        Serial.println("\n━━━━━━━━━━ System Status ━━━━━━━━━━");
        Serial.printf("⚡ System State: %s\n", powerControl_getCurrentStateString());
        Serial.printf("📡 Firebase Ready: %s\n", firebaseUploadIsReady() ? "YES" : "NO");
        Serial.printf("📶 WiFi: %s (RSSI: %d)\n", 
                     WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected",
                     WiFi.RSSI());
        Serial.printf("🫀 Last HR: %d BPM\n", g_heartRate);
        Serial.printf("🩸 Last SpO2: %d %%\n", g_spo2);
        Serial.printf("💾 Free Heap: %d bytes\n", ESP.getFreeHeap());
        Serial.printf("⚠️ Firebase Failures: %lu\n", firebaseUploadGetConsecutiveFailures());
        Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    }
    
    // Reset watchdog cho loop task
    esp_task_wdt_reset();
    
    // Delay để không spam
    delay(1000);
}
