#include "powerControl.h"
#include <esp_task_wdt.h>
#include <driver/gpio.h>
#include "oled.h"

static PowerControlConfig* g_config = nullptr;
static volatile SystemState_t systemState = SYSTEM_STATE_ON;
static SemaphoreHandle_t powerStateMutex = nullptr;
static TaskHandle_t powerControlTaskHandle = nullptr;
static TimerHandle_t buttonDebounceTimer = nullptr;

// Biến debounce
static volatile uint32_t lastButtonPressTime = 0;
static volatile bool buttonPressHandled = true;
static volatile bool longPressDetected = false;

// Hàm debounce timer callback
void buttonDebounceTimerCallback(TimerHandle_t xTimer) {
    // Timer hết hạn, reset trạng thái nút nhấn
    buttonPressHandled = true;
    longPressDetected = false;
}

static void handleSystemStateChange(SystemState_t newState) {
    if (powerStateMutex == nullptr) return;
    
    if (xSemaphoreTake(powerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        SystemState_t oldState = systemState;
        systemState = newState;
        
        // Cập nhật LED trạng thái
        digitalWrite(STATUS_LED_GPIO, (newState == SYSTEM_STATE_ON) ? HIGH : LOW);
        
        Serial.printf("=== System State Changed: %s -> %s ===\n", 
                     powerControl_getStateString(oldState),
                     powerControl_getStateString(newState));
        
        // Gọi callback nếu có
        if (g_config != nullptr && g_config->systemStateChangedCallback != nullptr) {
            g_config->systemStateChangedCallback(newState);
        }
        
        xSemaphoreGive(powerStateMutex);
    }
}

static void processButtonPress(bool isLongPress) {
    if (!buttonPressHandled) {
        return; // Đang chờ xử lý, bỏ qua
    }
    
    buttonPressHandled = false;
    
    if (isLongPress) {
        Serial.println("LONG PRESS detected - potential future feature");
        // Có thể thêm chức năng cho long press ở đây
        buttonPressHandled = true;
        return;
    }
    
    // SHORT PRESS - Toggle power state
    SystemState_t currentState = powerControl_getSystemState();
    SystemState_t newState = (currentState == SYSTEM_STATE_ON) ? SYSTEM_STATE_OFF : SYSTEM_STATE_ON;
    
    Serial.printf("SHORT PRESS - Toggling power: %s -> %s\n",
                 powerControl_getStateString(currentState),
                 powerControl_getStateString(newState));
    
    handleSystemStateChange(newState);
    
    // Khởi động lại debounce timer
    if (buttonDebounceTimer != nullptr) {
        xTimerReset(buttonDebounceTimer, 0);
    }
}

bool powerControl_init(PowerControlConfig* config) {
    if (config == nullptr) {
        Serial.println("PowerControl: Config is null!");
        return false;
    }
    
    g_config = config;
    
    // Khởi tạo GPIO
    pinMode(POWER_BUTTON_GPIO, INPUT_PULLUP);
    pinMode(STATUS_LED_GPIO, OUTPUT);
    digitalWrite(STATUS_LED_GPIO, HIGH); // Mặc định bật
    
    Serial.println("PowerControl: GPIO initialized");
    Serial.printf("  Button: GPIO %d (INPUT_PULLUP)\n", POWER_BUTTON_GPIO);
    Serial.printf("  LED: GPIO %d (OUTPUT)\n", STATUS_LED_GPIO);
    
    // Tạo mutex
    powerStateMutex = xSemaphoreCreateMutex();
    if (powerStateMutex == nullptr) {
        Serial.println("PowerControl: Failed to create mutex!");
        return false;
    }
    
    // Tạo software timer cho debounce
    buttonDebounceTimer = xTimerCreate(
        "ButtonDebounce",
        pdMS_TO_TICKS(500), // 500ms debounce period
        pdFALSE, // One-shot timer
        (void*)0,
        buttonDebounceTimerCallback
    );
    
    if (buttonDebounceTimer == nullptr) {
        Serial.println("PowerControl: Failed to create debounce timer!");
        return false;
    }
    
    systemState = SYSTEM_STATE_ON;
    buttonPressHandled = true;
    lastButtonPressTime = 0;
    
    Serial.println("PowerControl: Initialized successfully");
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
        Serial.println("PowerControl: Cannot start task - config is null!");
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
        Serial.println("PowerControl: Failed to create task!");
        return false;
    }
    
    Serial.println("PowerControl: Task created successfully");
    return true;
}

SystemState_t powerControl_getSystemState() {
    if (powerStateMutex == nullptr) return SYSTEM_STATE_ON;
    
    if (xSemaphoreTake(powerStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        SystemState_t state = systemState;
        xSemaphoreGive(powerStateMutex);
        return state;
    }
    return SYSTEM_STATE_ON;
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
    return powerControl_getStateString(systemState);
}

void powerControlTask(void* pvParameters) {
    (void)pvParameters;
    
    // THÊM TASK VÀO WATCHDOG - QUAN TRỌNG!
    esp_task_wdt_add(NULL);
    
    bool lastButtonState = digitalRead(POWER_BUTTON_GPIO);
    uint32_t buttonPressStartTime = 0;
    bool buttonPressed = false;
    
    Serial.println("PowerControl: Task started - monitoring button...");
    
    for (;;) {
        bool currentButtonState = digitalRead(POWER_BUTTON_GPIO);
        uint32_t currentTime = millis();
        
        // Phát hiện nhấn nút (HIGH -> LOW)
        if (currentButtonState == LOW && lastButtonState == HIGH) {
            buttonPressed = true;
            buttonPressStartTime = currentTime;
            Serial.println("PowerControl: Button pressed (debouncing...)");
        }
        
        // Phát hiện nhả nút (LOW -> HIGH) và xử lý
        if (currentButtonState == HIGH && lastButtonState == LOW && buttonPressed) {
            uint32_t pressDuration = currentTime - buttonPressStartTime;
            
            // Kiểm tra debounce và xác định loại nhấn
            if (pressDuration >= BUTTON_DEBOUNCE_MS) {
                bool isLongPress = (pressDuration >= BUTTON_LONG_PRESS_MS);
                
                if (!isLongPress) {
                    // Short press - toggle power
                    processButtonPress(false);
                } else {
                    // Long press - có thể dùng cho chức năng khác
                    processButtonPress(true);
                }
            }
            
            buttonPressed = false;
        }
        
        // Xử lý long press đang diễn ra
        if (buttonPressed && !longPressDetected) {
            uint32_t pressDuration = currentTime - buttonPressStartTime;
            if (pressDuration >= BUTTON_LONG_PRESS_MS) {
                longPressDetected = true;
                Serial.println("PowerControl: Long press detected");
                // Có thể thêm feedback (ví dụ: LED nhấp nháy) ở đây
            }
        }
        
        lastButtonState = currentButtonState;
        
        // RESET WATCHDOG - QUAN TRỌNG!
        esp_task_wdt_reset();
        
        // Delay hợp lý để giảm CPU usage
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
