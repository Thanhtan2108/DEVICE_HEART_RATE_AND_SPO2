#ifndef SCANI2C_H
#define SCANI2C_H

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define SDA_PIN 21
#define SCL_PIN 22

// FreeRTOS Task Configuration
#define SCANI2C_TASK_PRIORITY 2           // Priority (0-24, higher = more important)
#define SCANI2C_TASK_STACK_SIZE 4096      // Stack size in bytes
#define SCANI2C_CORE 0                    // Core 0 (or 1 if using second core)
#define SCANI2C_TASK_DELAY 5000           // Delay 5 seconds between scans

void setupI2C(uint8_t sdaPin = SDA_PIN, uint8_t sclPin = SCL_PIN);
void scanI2C();
void scanI2CTask(void *parameter);
void createScanI2CTask();

#endif
