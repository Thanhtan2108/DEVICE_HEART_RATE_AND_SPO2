#include "ButtonPower.h"
#include "DisplayOLED.h"
#include "Max30102Sensor.h"

ButtonPower* ButtonPower::_instance = nullptr;

ButtonPower::ButtonPower(uint8_t buttonPin, uint8_t ledPin)
  : _buttonPin(buttonPin), _ledPin(ledPin), _active(false),
    _lastStableLevel(HIGH), _prevStableLevel(HIGH), _lastRawLevel(HIGH),
    _lastChangeMs(0), _toggleRequested(false), _lastIsrMs(0) {}

bool ButtonPower::begin(bool initialActive) {
  pinMode(_buttonPin, INPUT_PULLUP); // active LOW
  pinMode(_ledPin, OUTPUT);
  _active = initialActive;
  applyLed();
  _lastStableLevel = digitalRead(_buttonPin);
  _prevStableLevel = _lastStableLevel;
  _lastRawLevel = _lastStableLevel;
  _lastChangeMs = millis();

  // Attach interrupt on falling edge (button press, INPUT_PULLUP)
  _instance = this;
  attachInterrupt(digitalPinToInterrupt(_buttonPin), ButtonPower::isrHandler, FALLING);
  return true;
}

void ButtonPower::setActive(bool active) {
  _active = active;
  applyLed();
}

void ButtonPower::applyLed() {
  digitalWrite(_ledPin, _active ? HIGH : LOW);
}

void ButtonPower::loop() {
  // Handle interrupt-triggered toggle with debounce in main context
  if (_toggleRequested) {
    noInterrupts();
    bool handle = _toggleRequested;
    _toggleRequested = false;
    interrupts();
    if (handle) {
      _active = !_active;
      applyLed();
    }
  }

  bool raw = digitalRead(_buttonPin);
  if (raw != _lastRawLevel) {
    _lastRawLevel = raw;
    _lastChangeMs = millis();
  }

  if (millis() - _lastChangeMs >= DEBOUNCE_MS) {
    if (raw != _lastStableLevel) {
      _prevStableLevel = _lastStableLevel;
      _lastStableLevel = raw;
      // Optional: keep also a polling-based toggle as fallback
      if (_prevStableLevel == HIGH && _lastStableLevel == LOW) {
        _active = !_active;
        applyLed();
      }
    }
  }
}

void IRAM_ATTR ButtonPower::isrHandler() {
  if (_instance == nullptr) return;
  unsigned long now = millis();
  if (now - _instance->_lastIsrMs >= DEBOUNCE_MS) {
    _instance->_lastIsrMs = now;
    _instance->_toggleRequested = true;
  }
}

bool ButtonPower::ensureActive(DisplayOLED *oled, Max30102Sensor *sensor) {
  bool active = _active;
  if (active != _lastAppliedActive) {
    _lastAppliedActive = active;
    if (active) {
      if (oled) { oled->powerOn(); oled->showStatus("Device Active", "Place finger..."); }
      if (sensor) { sensor->reset(); sensor->powerOn(); }
    } else {
      if (oled) { oled->powerOff(); }
      if (sensor) { sensor->powerOff(); sensor->reset(); }
    }
  }
  return active;
}
