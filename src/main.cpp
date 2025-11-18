#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "scanI2C.h"

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n\n==================================");
  Serial.println("  Heart Rate & SpO2 System");
  Serial.println("  FreeRTOS initialized");
  Serial.println("==================================");
  
  // Initialize FreeRTOS task for ScanI2C
  createScanI2CTask();
  
  Serial.println("[Setup] Initialization completed\n");
}

void loop() {
  // Main loop does nothing in FreeRTOS-based system
  // All work is handled by FreeRTOS tasks
  delay(1000);
}
