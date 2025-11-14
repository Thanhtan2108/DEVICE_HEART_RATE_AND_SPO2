#pragma once

#ifndef PINCONFIG_H
#define PINCONFIG_H

// I2C pins (change if your wiring differs)
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// INT pin from MAX30102 (active LOW). Choose GPIO that supports interrupts.
// Example: GPIO34. Change if needed.
#define MAX30102_INT_PIN 34

// I2C clock speed (Hz)
#define I2C_CLOCK_HZ 400000UL

#endif // PINCONFIG_H
