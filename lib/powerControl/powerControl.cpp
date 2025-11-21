#include "powerControl.h"
#include <esp_task_wdt.h>
#include <driver/gpio.h>
#include "oled.h"

static PowerControlConfig* g_config = nullptr;
static volatile SystemState_t systemState = SYSTEM_STATE_ON;
static SemaphoreHandle_t powerStateMutex = nullptr;
static TaskHandle_t powerControlTaskHandle = nullptr;

// Biến debounce - SIMPLIFIED
static volatile uint32_t lastButtonChangeTime = 0;
static volatile bool lastButtonState = HIGH;
static volatile bool buttonProcessed = false;

// THÊM: Safe callback wrapper
static void safeCallStateCallback(SystemState_t newState) {
    // Kiểm tra kỹ trước khi gọi callback
    if (g_config == nullptr) {
        Serial.println("⚠️ Config is NULL - cannot call callback");
        return;
    }
    
    if (g_config->systemStateChangedCallback == nullptr) {
        Serial.println("⚠️ Callback is NULL - skipping");
        return;
    }
    
    // Gọi callback an toàn
    Serial.println("📞 Calling state callback...");
    g_config->systemStateChangedCallback(newState);
    Serial.println("✅ Callback completed");
}

static void handleSystemStateChange(SystemState_t newState) {
    if (powerStateMutex == nullptr) {
        Serial.println("⚠️ PowerStateMutex is NULL!");
        return;
    }
    
    // Lấy mutex với timeout dài hơn
    if (xSemaphoreTake(powerStateMutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        Serial.println("⚠️ Failed to acquire powerStateMutex!");
        return;
    }
    
    SystemState_t oldState = systemState;
    
    if (oldState == newState) {
        xSemaphoreGive(powerStateMutex);
        Serial.println("ℹ️ State unchanged - skipping");
        return; // Không có thay đổi
    }
    
    systemState = newState;
    
    // Cập nhật LED trạng thái
    digitalWrite(STATUS_LED_GPIO, (newState == SYSTEM_STATE_ON) ? HIGH : LOW);
    
    Serial.println();
    Serial.println("╔════════════════════════════════╗");
    Serial.printf("║  STATE: %-4s → %-4s         ║\n", 
                 powerControl_getStateString(oldState),
                 powerControl_getStateString(newState));
    Serial.println("╚════════════════════════════════╝");
    Serial.println();
    
    // RELEASE MUTEX TRƯỚC KHI GỌI CALLBACK
    xSemaphoreGive(powerStateMutex);
    
    // Delay để đảm bảo mutex được release hoàn toàn
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Gọi callback SAU khi đã release mutex
    safeCallStateCallback(newState);
}

bool powerControl_init(PowerControlConfig* config) {
    if (config == nullptr) {
        Serial.println("❌ PowerControl: Config is null!");
        return false;
    }
    
    g_config = config;
    
    // Validate config
    Serial.println("🔍 Validating PowerControl config...");
    Serial.printf("  sensorTaskHandle: %p\n", config->sensorTaskHandle);
    Serial.printf("  displayTaskHandle: %p\n", config->displayTaskHandle);
    Serial.printf("  i2cMutex: %p\n", config->i2cMutex);
    Serial.printf("  oledClearCallback: %p\n", config->oledClearCallback);
    Serial.printf("  systemStateChangedCallback: %p\n", config->systemStateChangedCallback);
    
    // Khởi tạo GPIO
    pinMode(POWER_BUTTON_GPIO, INPUT_PULLUP);
    pinMode(STATUS_LED_GPIO, OUTPUT);
    digitalWrite(STATUS_LED_GPIO, HIGH); // Mặc định bật
    
    Serial.println("✅ PowerControl: GPIO initialized");
    Serial.printf("   Button: GPIO %d (INPUT_PULLUP)\n", POWER_BUTTON_GPIO);
    Serial.printf("   LED: GPIO %d (OUTPUT)\n", STATUS_LED_GPIO);
    
    // Tạo mutex
    powerStateMutex = xSemaphoreCreateMutex();
    if (powerStateMutex == nullptr) {
        Serial.println("❌ PowerControl: Failed to create mutex!");
        return false;
    }
    
    systemState = SYSTEM_STATE_ON;
    lastButtonState = digitalRead(POWER_BUTTON_GPIO);
    lastButtonChangeTime = millis();
    buttonProcessed = false;
    
    Serial.println("✅ PowerControl: Initialized successfully");
    return true;
}

bool powerControl_getState() {
    return (systemState == SYSTEM_STATE_ON);
}

void powerControl_setState(bool state) {
    handleSystemStateChange(state ? SYSTEM_STATE_ON : SYSTEM_STATE_OFF);
}

bool powerControl_startTask() {
    if (g_config == nullptr) {
        Serial.println("❌ PowerControl: Cannot start task - config is null!");
        return false;
    }
    
    BaseType_t result = xTaskCreatePinnedToCore(
        powerControlTask,
        "PowerControl",
        POWER_TASK_STACK_SIZE,
        nullptr,
        POWER_TASK_PRIORITY,
        &powerControlTaskHandle,
        1 // Core 1
    );
    
    if (result != pdPASS) {
        Serial.println("❌ PowerControl: Failed to create task!");
        return false;
    }
    
    Serial.println("✅ PowerControl: Task created on Core 1");
    return true;
}

SystemState_t powerControl_getSystemState() {
    if (powerStateMutex == nullptr) return SYSTEM_STATE_ON;
    
    SystemState_t state = SYSTEM_STATE_ON;
    if (xSemaphoreTake(powerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        state = systemState;
        xSemaphoreGive(powerStateMutex);
    }
    return state;
}

const char* powerControl_getStateString(SystemState_t state) {
    switch(state) {
        case SYSTEM_STATE_OFF: return "OFF";
        case SYSTEM_STATE_ON: return "ON";
        case SYSTEM_STATE_SLEEP: return "SLEEP";
        default: return "UNKNOWN";
    }
}

const char* powerControl_getCurrentStateString() {
    return powerControl_getStateString(powerControl_getSystemState());
}

void powerControlTask(void* pvParameters) {
    (void)pvParameters;
    
    Serial.println("🎛️ PowerControl Task started");
    
    // THÊM TASK VÀO WATCHDOG
    esp_task_wdt_add(NULL);
    
    bool currentReading = HIGH;
    bool stableState = HIGH;
    uint32_t debounceStartTime = 0;
    bool debouncing = false;
    
    uint32_t taskLoopCount = 0;
    uint32_t lastHealthPrint = 0;
    
    // THÊM: Delay khởi động để đảm bảo tất cả tasks đã sẵn sàng
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("🎛️ PowerControl: Ready to handle button events");
    
    for (;;) {
        taskLoopCount++;
        
        // Debug task health
        if (millis() - lastHealthPrint > 60000) {
            lastHealthPrint = millis();
            Serial.printf("🎛️ PowerControl Task Health: %lu loops, State=%s\n",
                         taskLoopCount, powerControl_getCurrentStateString());
            taskLoopCount = 0;
        }
        
        // Đọc trạng thái nút
        currentReading = digitalRead(POWER_BUTTON_GPIO);
        uint32_t now = millis();
        
        // Phát hiện thay đổi
        if (currentReading != stableState) {
            if (!debouncing) {
                // Bắt đầu debounce
                debouncing = true;
                debounceStartTime = now;
            } else {
                // Kiểm tra đã đủ thời gian debounce chưa
                if (now - debounceStartTime >= BUTTON_DEBOUNCE_MS) {
                    // State đã ổn định sau debounce
                    stableState = currentReading;
                    debouncing = false;
                    
                    // Xử lý khi nút được NHẤN (HIGH -> LOW)
                    if (stableState == LOW && !buttonProcessed) {
                        buttonProcessed = true;
                        
                        Serial.println("🔘 Button PRESSED - toggling system state...");
                        
                        // Reset watchdog trước khi xử lý
                        esp_task_wdt_reset();
                        
                        // Toggle state
                        SystemState_t currentState = powerControl_getSystemState();
                        SystemState_t newState = (currentState == SYSTEM_STATE_ON) ? 
                                                  SYSTEM_STATE_OFF : SYSTEM_STATE_ON;
                        
                        Serial.printf("   Current: %s, Target: %s\n",
                                     powerControl_getStateString(currentState),
                                     powerControl_getStateString(newState));
                        
                        // Thực hiện thay đổi
                        handleSystemStateChange(newState);
                        
                        // Reset watchdog sau khi xử lý
                        esp_task_wdt_reset();
                        
                        Serial.println("✅ State change completed");
                    }
                    // Reset flag khi nút được NHẢ (LOW -> HIGH)
                    else if (stableState == HIGH) {
                        buttonProcessed = false;
                    }
                }
            }
        } else {
            // Không có thay đổi, reset debounce
            debouncing = false;
        }
        
        // RESET WATCHDOG - QUAN TRỌNG!
        esp_task_wdt_reset();
        
        // Delay hợp lý
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
