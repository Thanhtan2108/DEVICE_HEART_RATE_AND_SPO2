#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>

// Cấu hình GPIO
#define POWER_BUTTON_GPIO 5
#define STATUS_LED_GPIO 4
#define BUTTON_DEBOUNCE_MS 50
#define BUTTON_LONG_PRESS_MS 2000
#define POWER_TASK_STACK_SIZE 4096
#define POWER_TASK_PRIORITY 5

// Trạng thái hệ thống
typedef enum {
    SYSTEM_STATE_OFF = 0,
    SYSTEM_STATE_ON = 1,
    SYSTEM_STATE_SLEEP = 2
} SystemState_t;

// Cấu trúc để truyền thông tin task handles
struct PowerControlConfig {
    TaskHandle_t* sensorTaskHandle;
    TaskHandle_t* displayTaskHandle;
    SemaphoreHandle_t* i2cMutex;
    void (*oledClearCallback)(void);
    void (*systemStateChangedCallback)(SystemState_t newState);
};

// Hàm khởi tạo
bool powerControl_init(PowerControlConfig* config);
bool powerControl_startTask();
bool powerControl_getState();
void powerControl_setState(bool state);
void powerControlTask(void* pvParameters);

// Hàm utility
SystemState_t powerControl_getSystemState();
const char* powerControl_getStateString(SystemState_t state);
const char* powerControl_getCurrentStateString();

#endif
