#pragma once

#ifndef I2C_MUTEX_H
#define I2C_MUTEX_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Global I2C mutex handle. Create it once in main() before any I2C operations.
extern SemaphoreHandle_t i2cMutex;

// Helper macro wrappers (optional) for convenience
#define I2C_MUTEX_TAKE(timeout)  ( (i2cMutex) ? xSemaphoreTake(i2cMutex, (timeout)) : pdTRUE )
#define I2C_MUTEX_GIVE()         do { if(i2cMutex) xSemaphoreGive(i2cMutex); } while(0)

#endif // I2C_MUTEX_H
