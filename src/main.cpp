#include <Arduino.h>
#include "max30102.h"
#include "../lib/common/pinConfig.h"
#include "../lib/common/sensorConfig.h"
#include "../lib/common/systemConfig.h"
#include "../lib/common/i2c_mutex.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <esp_task_wdt.h>

// Define the global i2cMutex declared in i2c_mutex.h
SemaphoreHandle_t i2cMutex = nullptr;

// Global sensor object (used by tasks)
Max30102Sensor sensor;

// Simple print task — reads latest results and prints periodically
static TaskHandle_t printTaskHandle = nullptr;

static void printTask(void *pvParameters) {
  (void)pvParameters;
  // Register to WDT (optional)
  esp_task_wdt_add(NULL);

  const TickType_t delayTicks = pdMS_TO_TICKS(2000);
  
  for (;;) {
    // Feed WDT for this task
    esp_task_wdt_reset();

    int32_t spo2; bool spo2_ok;
    int32_t hr;   bool hr_ok;
    sensor.getLatest(spo2, spo2_ok, hr, hr_ok);

    Serial.print("[PRINT TASK] SPO2: ");
    if (spo2_ok) Serial.print(spo2); else Serial.print("Invalid");
    Serial.print("   HR: ");
    if (hr_ok) Serial.println(hr); else Serial.println("Invalid");

    vTaskDelay(delayTicks);
  }

  // cleanup
  esp_task_wdt_delete(NULL);
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("=== MAX30102 production-ready example (no lib serial) ===");

  // Create I2C mutex before any I2C ops
  i2cMutex = xSemaphoreCreateMutex();
  if (!i2cMutex) {
    Serial.println("Warning: failed to create i2cMutex - I2C will not be protected");
  }

  // Initialize Task WDT (timeout seconds, panic->true to reboot)
  esp_err_t err = esp_task_wdt_init(WDT_TIMEOUT_S, true);
  if (err == ESP_OK) {
    Serial.printf("Task WDT initialized: %d s\n", WDT_TIMEOUT_S);
  } else if (err == ESP_ERR_INVALID_STATE) {
    Serial.println("WDT already initialized");
  } else {
    Serial.printf("WDT init error: %d\n", err);
  }

  // Initialize sensor hardware (config registers)
  if (!sensor.begin()) {
    Serial.println("sensor.begin() failed. Halting.");
    while (1) delay(1000);
  }

  // Start sensor read task (this will create the readTask and attach INT)
  if (!sensor.start()) {
    Serial.println("sensor.start() failed. Halting.");
    while (1) delay(1000);
  }

  // Create print task to show results periodically (separate from main loop)
  BaseType_t ret = xTaskCreatePinnedToCore(
    printTask,
    "print_task",
    4096,
    NULL,
    1,
    &printTaskHandle,
    0
  );
  if (ret == pdPASS) {
    Serial.println("Print task started");
  } else {
    Serial.println("Failed to start print task");
  }
}

void loop() {
  // main loop is idle — RTOS tasks do the work
  vTaskDelay(pdMS_TO_TICKS(1000));
}
