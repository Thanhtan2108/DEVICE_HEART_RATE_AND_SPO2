#ifndef SCANI2C_H
#define SCANI2C_H

#include <Arduino.h>
#include <Wire.h>

void I2C_Init(uint8_t sdaPin, uint8_t sclPin, int speedI2C = 100000);
void I2C_ScanAddress();

#endif
