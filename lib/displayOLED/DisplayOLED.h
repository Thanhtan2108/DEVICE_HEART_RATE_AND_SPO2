#ifndef DISPLAY_OLED_H
#define DISPLAY_OLED_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"

class DisplayOLED {
public:
  // width/height in pixels, TwoWire pointer (default &Wire), rst pin default -1.
  DisplayOLED(uint8_t width = 128, uint8_t height = 64, TwoWire *wire = &Wire, int8_t rst_pin = -1);

  // Initialize display with 7-bit I2C address (default 0x3C)
  bool begin(uint8_t i2cAddr = OLED_ADDR);

  // Simple helpers to show status or vitals
  void showStatus(const String &line1, const String &line2 = String());
  void showVitals(int bpm, float spo2);

  // Power controls (SSD1306 commands)
  void powerOn();
  void powerOff();

private:
  Adafruit_SSD1306 _display;
};

#endif // DISPLAY_OLED_H
