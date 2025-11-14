#pragma once

#ifndef MAX30102_SENSOR_H
#define MAX30102_SENSOR_H

#include <Arduino.h>

class Max30102Sensor {
public:
  Max30102Sensor();

  // Initialize I2C and sensor
  bool begin();

  // Start FreeRTOS read task (non-blocking)
  bool start();

  // Stop read task
  void stop();

  // Get latest computed SPO2 and HR (atomic-ish copy)
  bool getLatest(int32_t &spo2, bool &spo2_valid, int32_t &hr, bool &hr_valid);

  // Enable/disable internal serial printing
  void enableSerialPrint(bool en);
};

#endif // MAX30102_SENSOR_H
