#pragma once

#ifndef MAX30102_SENSOR_H
#define MAX30102_SENSOR_H

#include <Arduino.h>

class Max30102Sensor {
public:
  Max30102Sensor();

  // Initialize I2C and sensor (configure registers). Does NOT create the FreeRTOS read task.
  // Returns true if sensor detected and configured.
  bool begin();

  // Start FreeRTOS read task (non-blocking). This will also attach INT -> notify the task.
  // Returns true if task creation succeeded.
  bool start();

  // Stop read task (requests task exit, waits for it to finish).
  void stop();

  // Get latest computed SPO2 and HR (atomic-ish copy)
  bool getLatest(int32_t &spo2, bool &spo2_valid, int32_t &hr, bool &hr_valid);

  // Enable/disable internal serial printing of results
  void enableSerialPrint(bool en);

private:
  // Nothing public
};

#endif // MAX30102_SENSOR_H
