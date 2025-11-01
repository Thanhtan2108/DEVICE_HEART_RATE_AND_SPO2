#ifndef BUTTON_POWER_H
#define BUTTON_POWER_H

#include <Arduino.h>
#include "config.h"
#include "DisplayOLED.h"
#include "Max30102Sensor.h"

class ButtonPower {
public:
  ButtonPower(uint8_t buttonPin = BUTTON_PIN, uint8_t ledPin = LED_PIN);

  bool begin(bool initialActive = false);
  void loop();

  bool isActive() const { return _active; }
  void setActive(bool active);

  // Apply power state transitions to peripherals; returns current active state
  bool ensureActive(DisplayOLED *oled, Max30102Sensor *sensor);

private:
  void applyLed();

  const uint8_t _buttonPin;
  const uint8_t _ledPin;

  bool _active;
  bool _lastStableLevel;
  bool _prevStableLevel;
  bool _lastRawLevel;
  unsigned long _lastChangeMs;
  static const unsigned long DEBOUNCE_MS = 50;

  // Interrupt-based toggling
  static ButtonPower* _instance;
  static void IRAM_ATTR isrHandler();
  volatile bool _toggleRequested;
  volatile unsigned long _lastIsrMs;
  bool _lastAppliedActive = false;
};

#endif
