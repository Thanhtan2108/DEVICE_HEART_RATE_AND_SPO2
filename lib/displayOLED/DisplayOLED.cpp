#include "DisplayOLED.h"

DisplayOLED::DisplayOLED(uint8_t width, uint8_t height, TwoWire *wire, int8_t rst_pin)
  : _display(width, height, wire, rst_pin) {}

bool DisplayOLED::begin(uint8_t i2cAddr) {
  // On ESP32 the Adafruit SSD1306 begin signature is the same.
  if (!_display.begin(SSD1306_SWITCHCAPVCC, i2cAddr)) {
    return false;
  }
  _display.clearDisplay();
  _display.setTextSize(1);
  _display.setTextColor(SSD1306_WHITE);
  _display.setCursor(0, 0);
  _display.println(F("OLED Ready"));
  _display.display();
  return true;
}

void DisplayOLED::showStatus(const String &line1, const String &line2) {
  _display.clearDisplay();
  _display.setTextSize(1);
  _display.setTextColor(SSD1306_WHITE);
  _display.setCursor(0, 0);
  _display.println(line1);
  if (line2.length() > 0) {
    _display.setCursor(0, 12);
    _display.println(line2);
  }
  _display.display();
}

void DisplayOLED::showVitals(int bpm, float spo2) {
  _display.clearDisplay();
  _display.setTextColor(SSD1306_WHITE);

  _display.setTextSize(2);
  _display.setCursor(0, 0);
  _display.print(F("HR:"));
  _display.print(bpm);
  _display.println(F(" BPM"));

  _display.setTextSize(2);
  _display.setCursor(0, 32);
  _display.print(F("SpO2:"));
  _display.print(spo2, 1);
  _display.println(F("%"));

  _display.display();
}

void DisplayOLED::powerOn() {
  _display.ssd1306_command(SSD1306_DISPLAYON);
}

void DisplayOLED::powerOff() {
  _display.ssd1306_command(SSD1306_DISPLAYOFF);
}
