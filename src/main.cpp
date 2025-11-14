#include <Arduino.h>
#include "max30102.h"
#include "../lib/common/sensorConfig.h"
#include "../lib/common/pinConfig.h"
#include "../lib/common/systemConfig.h"

#include <esp_task_wdt.h>

Max30102Sensor sensor;

// Example additional app task
static TaskHandle_t appTaskHandle = nullptr;
void appTask(void *pvParameters) {
  esp_task_wdt_add(NULL); // register this task to WDT

  uint32_t t = millis();
  for (;;) {
    esp_task_wdt_reset(); // feed WDT for this task
    if (millis() - t > 2000) {
      t = millis();
      Serial.println("[APP TASK] alive");
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== MAX30102 + ESP32 + INT + FreeRTOS + WDT Example ===");

  // Init Task WDT (timeout seconds, panic->true to reboot)
  esp_err_t err = esp_task_wdt_init(WDT_TIMEOUT_S, true);
  if (err == ESP_OK) {
    Serial.printf("Task WDT initialized: %d s\n", WDT_TIMEOUT_S);
  } else if (err == ESP_ERR_INVALID_STATE) {
    Serial.println("WDT already initialized");
  } else {
    Serial.printf("WDT init error: %d\n", err);
  }

  // Begin sensor
  if (!sensor.begin()) {
    Serial.println("Sensor init failed. Halting.");
    while (1) delay(1000);
  }
  sensor.enableSerialPrint(true);

  // Start sensor read task
  if (!sensor.start()) {
    Serial.println("Failed to start sensor task. Halting.");
    while (1) delay(1000);
  }

  // Start other example app task
  BaseType_t ret = xTaskCreatePinnedToCore(
    appTask,
    "app_task",
    APP_TASK_STACK_SIZE,
    NULL,
    APP_TASK_PRIORITY,
    &appTaskHandle,
    APP_TASK_CORE
  );

  if (ret == pdPASS) Serial.println("App task started");
  else Serial.println("App task failed to start");
}

void loop() {
  // Periodically print latest computed result from sensor
  static uint32_t last = 0;
  if (millis() - last > 2000) {
    last = millis();
    int32_t spo2; bool spo2_ok;
    int32_t hr;   bool hr_ok;
    sensor.getLatest(spo2, spo2_ok, hr, hr_ok);

    Serial.print("[MAIN] SPO2: ");
    if (spo2_ok) Serial.print(spo2); else Serial.print("Invalid");
    Serial.print("   HR: ");
    if (hr_ok) Serial.println(hr); else Serial.println("Invalid");
  }
  vTaskDelay(pdMS_TO_TICKS(500));
}
