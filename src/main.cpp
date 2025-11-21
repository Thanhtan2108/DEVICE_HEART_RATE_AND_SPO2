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

#define UPLOAD_INTERVAL_MS 1000  // FIXED: Upload mỗi 1 giây (real-time hơn)
#define DISPLAY_UPDATE_INTERVAL_MS 500  // FIXED: Update display mỗi 0.5 giây
#define SERIAL_DEBUG_INTERVAL_MS 1000  // Debug serial mỗi 1 giây

// ===== Global State với Mutex Protection =====
static SemaphoreHandle_t dataMutex = NULL;
static int g_heartRate = 0;
static int g_spo2 = 0;
static long g_irValue = 0;
static bool g_hasFingerDetected = false;
static uint32_t g_lastUploadTime = 0;
static uint32_t g_lastSerialDebugTime = 0;

// THÊM: Biến đếm số lần đọc thành công
static uint32_t g_totalReadings = 0;
static uint32_t g_successfulUploads = 0;

// ===== Helper Functions =====
void safeUpdateGlobalData(int hr, int spo2, long ir, bool hasFinger) {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_heartRate = hr;
        g_spo2 = spo2;
        g_irValue = ir;
        g_hasFingerDetected = hasFinger;
        xSemaphoreGive(dataMutex);
    }
}

void safeReadGlobalData(int *hr, int *spo2, long *ir, bool *hasFinger) {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        *hr = g_heartRate;
        *spo2 = g_spo2;
        *ir = g_irValue;
        *hasFinger = g_hasFingerDetected;
        xSemaphoreGive(dataMutex);
    }
}

// ===== System State Callback =====
void onSystemStateChanged(SystemState_t newState) {
    Serial.println("\n📌 ========================================");
    Serial.printf("📌 SYSTEM STATE CALLBACK: %s\n", powerControl_getStateString(newState));
    Serial.println("📌 ========================================\n");
    
    if (newState == SYSTEM_STATE_OFF) {
        Serial.println("💤 Entering OFF mode...");
        
        // 1. Tạm dừng Firebase upload TRƯỚC
        Serial.println("  [1/5] Disabling Firebase...");
        firebaseUploadSetEnabled(false);
        vTaskDelay(pdMS_TO_TICKS(100)); // Đợi Firebase task xử lý
        
        // 2. SUSPEND sensor task trước (ngừng đọc dữ liệu)
        Serial.println("  [2/5] Suspending Sensor Task...");
        if (sensorTaskHandle != NULL) {
            eTaskState sensorState = eTaskGetState(sensorTaskHandle);
            Serial.printf("      Sensor state before: %d\n", sensorState);
            vTaskSuspend(sensorTaskHandle);
            vTaskDelay(pdMS_TO_TICKS(100)); // Đợi task suspend
            sensorState = eTaskGetState(sensorTaskHandle);
            Serial.printf("      Sensor state after: %d (2=Suspended)\n", sensorState);
        }
        
        // 3. SUSPEND display task
        Serial.println("  [3/5] Suspending Display Task...");
        if (displayTaskHandle != NULL) {
            eTaskState displayState = eTaskGetState(displayTaskHandle);
            Serial.printf("      Display state before: %d\n", displayState);
            vTaskSuspend(displayTaskHandle);
            vTaskDelay(pdMS_TO_TICKS(100)); // Đợi task suspend
            displayState = eTaskGetState(displayTaskHandle);
            Serial.printf("      Display state after: %d (2=Suspended)\n", displayState);
        }
        
        // 4. Đợi đảm bảo không còn I2C activity
        Serial.println("  [4/5] Waiting for I2C to be idle...");
        vTaskDelay(pdMS_TO_TICKS(200));
        
        // 5. Tắt OLED sau khi display task đã CHẮC CHẮN suspend
        Serial.println("  [5/5] Turning off OLED...");
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
            Serial.println("      Got I2C mutex");
            oled_turn_off();
            Serial.println("      OLED turn_off() called");
            xSemaphoreGive(i2cMutex);
            Serial.println("      I2C mutex released");
        } else {
            Serial.println("      ⚠️ Failed to get I2C mutex!");
        }
        
        Serial.println("\n✅ System OFF completed!\n");
        
    } else if (newState == SYSTEM_STATE_ON) {
        Serial.println("🚀 Entering ON mode...");
        
        // 1. Bật lại OLED trước
        Serial.println("  [1/4] Turning on OLED...");
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
            oled_turn_on();
            vTaskDelay(pdMS_TO_TICKS(50));
            oled_show_waiting();
            xSemaphoreGive(i2cMutex);
            Serial.println("      OLED turned on");
        }
        
        // 2. RESUME sensor task
        Serial.println("  [2/4] Resuming Sensor Task...");
        if (sensorTaskHandle != NULL) {
            vTaskResume(sensorTaskHandle);
            vTaskDelay(pdMS_TO_TICKS(50));
            Serial.printf("      Sensor state: %d\n", eTaskGetState(sensorTaskHandle));
        }
        
        // 3. RESUME display task
        Serial.println("  [3/4] Resuming Display Task...");
        if (displayTaskHandle != NULL) {
            vTaskResume(displayTaskHandle);
            vTaskDelay(pdMS_TO_TICKS(50));
            Serial.printf("      Display state: %d\n", eTaskGetState(displayTaskHandle));
        }
        
        // 4. Bật lại Firebase upload
        Serial.println("  [4/4] Enabling Firebase...");
        firebaseUploadSetEnabled(true);
        
        Serial.println("\n✅ System ON completed!\n");
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
    const uint32_t READ_INTERVAL_MS = 50; // FIXED: Đọc sensor mỗi 50ms để không bỏ sót
    
    while (1) {
        uint32_t currentTime = millis();
        
        // REMOVED: Không cần check system state vì task sẽ bị suspend khi OFF
        
        if (currentTime - lastReadTime >= READ_INTERVAL_MS) {
            lastReadTime = currentTime;
            
            // Lấy I2C mutex trước khi đọc
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                
                // Đọc dữ liệu từ sensor
                bool dataUpdated = max30102_read_data(&irValue, &redValue);
                
                xSemaphoreGive(i2cMutex);
                
                if (dataUpdated) {
                    g_totalReadings++;
                    
                    // Kiểm tra ngón tay
                    bool hasFingerNow = max30102_has_finger(irValue);
                    
                    // Tính toán HR và SpO2
                    int hrValue = max30102_get_heart_rate(irValue);
                    int spo2Value = max30102_get_spo2(irValue, redValue);
                    
                    // Cập nhật global state (thread-safe)
                    int displayHR = 0;
                    int displaySpO2 = 0;
                    
                    if (hasFingerNow && max30102_is_heart_rate_valid()) {
                        displayHR = hrValue;
                    }
                    if (hasFingerNow && max30102_is_spo2_valid()) {
                        displaySpO2 = spo2Value;
                    }
                    
                    safeUpdateGlobalData(displayHR, displaySpO2, irValue, hasFingerNow);
                    
                    // THÊM: Debug serial theo interval
                    if (currentTime - g_lastSerialDebugTime >= SERIAL_DEBUG_INTERVAL_MS) {
                        g_lastSerialDebugTime = currentTime;
                        Serial.printf("📊 [%lu] IR=%ld, Finger=%s, HR=%d BPM, SpO2=%d%%, Uploads=%lu/%lu\n",
                                     currentTime/1000,
                                     irValue,
                                     hasFingerNow ? "YES" : "NO",
                                     displayHR,
                                     displaySpO2,
                                     g_successfulUploads,
                                     g_totalReadings);
                    }
                    
                    // FIXED: Upload NGAY khi có dữ liệu hợp lệ (real-time)
                    if (hasFingerNow && 
                        displayHR > 0 && 
                        displaySpO2 > 0 &&
                        (currentTime - g_lastUploadTime >= UPLOAD_INTERVAL_MS)) {
                        
                        // Gửi vào queue (không block)
                        if (sendSensorDataToFirebaseQueue(displayHR, displaySpO2, 0)) {
                            g_lastUploadTime = currentTime;
                            g_successfulUploads++;
                            Serial.printf("✅ Queued #%lu: HR=%d, SpO2=%d\n", 
                                         g_successfulUploads, displayHR, displaySpO2);
                        } else {
                            Serial.println("⚠️ Queue full - data dropped!");
                        }
                    }
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
        
        // REMOVED: Không cần check system state vì task sẽ bị suspend khi OFF
        
        if (currentTime - lastUpdateTime >= DISPLAY_UPDATE_INTERVAL_MS) {
            lastUpdateTime = currentTime;
            
            // Đọc dữ liệu an toàn
            int hr, spo2;
            long ir;
            bool hasFinger;
            safeReadGlobalData(&hr, &spo2, &ir, &hasFinger);
            
            // Lấy I2C mutex
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                
                if (hasFinger) {
                    oled_show_data(hr, spo2, ir, hasFinger);
                } else {
                    oled_show_waiting();
                }
                
                xSemaphoreGive(i2cMutex);
            }
        }
        
        // Reset watchdog
        esp_task_wdt_reset();
        
        // Delay để không chiếm hết CPU
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ===== Setup =====
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n=================================");
    Serial.println("   Heart Rate & SpO2 Monitor");
    Serial.println("      with Real-time Upload");
    Serial.println("=================================\n");
    
    // Cấu hình watchdog timeout (10 giây)
    esp_task_wdt_init(10, true);
    
    // Tạo data mutex cho global variables
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
        Serial.println("❌ Failed to create data mutex!");
        while(1) delay(1000);
    }
    Serial.println("✅ Data Mutex created");
    
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
    delay(3000);
    
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
    Serial.println("📝 Instructions:");
    Serial.println("  - Place finger on sensor");
    Serial.println("  - Press button to toggle power");
    Serial.println("  - Data uploads every 1 second");
    Serial.println("=================================\n");
    
    // Thêm loop task vào watchdog
    esp_task_wdt_add(NULL);
}

// ===== Loop =====
void loop() {
    // Loop task chỉ để monitor và reset watchdog
    static uint32_t lastStatusPrint = 0;
    uint32_t currentTime = millis();
    
    // Debug on demand - gửi 'd' qua Serial Monitor
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'd' || cmd == 'D') {
            // Debug task states
            Serial.println("\n━━━━━━━━━━ Task States Debug ━━━━━━━━━━");
            
            if (sensorTaskHandle != NULL) {
                eTaskState state = eTaskGetState(sensorTaskHandle);
                const char* stateStr[] = {"Running", "Ready", "Blocked", "Suspended", "Deleted"};
                Serial.printf("Sensor Task: %s (%d)\n", stateStr[state], state);
                UBaseType_t stackFree = uxTaskGetStackHighWaterMark(sensorTaskHandle);
                Serial.printf("  Stack free: %u bytes\n", stackFree * 4);
            }
            
            if (displayTaskHandle != NULL) {
                eTaskState state = eTaskGetState(displayTaskHandle);
                const char* stateStr[] = {"Running", "Ready", "Blocked", "Suspended", "Deleted"};
                Serial.printf("Display Task: %s (%d)\n", stateStr[state], state);
                UBaseType_t stackFree = uxTaskGetStackHighWaterMark(displayTaskHandle);
                Serial.printf("  Stack free: %u bytes\n", stackFree * 4);
            }
            
            if (firebaseTaskHandle != NULL) {
                eTaskState state = eTaskGetState(firebaseTaskHandle);
                const char* stateStr[] = {"Running", "Ready", "Blocked", "Suspended", "Deleted"};
                Serial.printf("Firebase Task: %s (%d)\n", stateStr[state], state);
                UBaseType_t stackFree = uxTaskGetStackHighWaterMark(firebaseTaskHandle);
                Serial.printf("  Stack free: %u bytes\n", stackFree * 4);
            }
            
            Serial.printf("System State: %s\n", powerControl_getCurrentStateString());
            Serial.printf("Free Heap: %u bytes\n", ESP.getFreeHeap());
            Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
        }
        else if (cmd == 't' || cmd == 'T') {
            // Test OLED toggle manual
            Serial.println("🧪 Testing OLED toggle manually...");
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                Serial.println("  Turning off...");
                oled_turn_off();
                xSemaphoreGive(i2cMutex);
                delay(2000);
                if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                    Serial.println("  Turning on...");
                    oled_turn_on();
                    oled_show_waiting();
                    xSemaphoreGive(i2cMutex);
                }
            }
            Serial.println("✅ Test completed");
        }
    }
    
    if (currentTime - lastStatusPrint >= 30000) { // Mỗi 30 giây
        lastStatusPrint = currentTime;
        
        Serial.println("\n━━━━━━━━━━ System Status ━━━━━━━━━━");
        Serial.printf("⚡ System State: %s\n", powerControl_getCurrentStateString());
        Serial.printf("📡 Firebase Ready: %s\n", firebaseUploadIsReady() ? "YES" : "NO");
        Serial.printf("📶 WiFi: %s (RSSI: %d dBm)\n", 
                     WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected",
                     WiFi.RSSI());
        
        int hr, spo2;
        long ir;
        bool hasFinger;
        safeReadGlobalData(&hr, &spo2, &ir, &hasFinger);
        
        Serial.printf("🫀 Current HR: %d BPM\n", hr);
        Serial.printf("🩸 Current SpO2: %d %%\n", spo2);
        Serial.printf("👆 Finger: %s\n", hasFinger ? "Detected" : "Not detected");
        Serial.printf("📊 Total Readings: %lu\n", g_totalReadings);
        Serial.printf("✅ Successful Uploads: %lu\n", g_successfulUploads);
        Serial.printf("💾 Free Heap: %d bytes\n", ESP.getFreeHeap());
        Serial.printf("⚠️ Firebase Failures: %lu\n", firebaseUploadGetConsecutiveFailures());
        
        // Kiểm tra stack usage
        if (sensorTaskHandle != NULL) {
            UBaseType_t sensorStack = uxTaskGetStackHighWaterMark(sensorTaskHandle);
            Serial.printf("📚 Stack Free - Sensor: %u bytes", sensorStack * 4);
            if (sensorStack * 4 < 512) Serial.print(" ⚠️ LOW!");
            Serial.println();
        }
        
        if (displayTaskHandle != NULL) {
            UBaseType_t displayStack = uxTaskGetStackHighWaterMark(displayTaskHandle);
            Serial.printf("📚 Stack Free - Display: %u bytes", displayStack * 4);
            if (displayStack * 4 < 512) Serial.print(" ⚠️ LOW!");
            Serial.println();
        }
        
        Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
        Serial.println("💡 Tip: Send 'd' for debug info, 't' to test OLED toggle");
    }
    
    // Reset watchdog cho loop task
    esp_task_wdt_reset();
    
    // Delay để không spam
    delay(1000);
}
