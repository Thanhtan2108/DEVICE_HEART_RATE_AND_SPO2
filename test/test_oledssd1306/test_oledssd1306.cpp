#include <Arduino.h>
#include <unity.h>
#include <Wire.h>
#include "OledSSD1306.h"

// I2C address thường dùng cho SSD1306
static const uint8_t OLED_I2C_ADDR = 0x3C;
// Thời gian pause để quan sát (ms)
static const unsigned long VISUAL_PAUSE_MS = 1200;

// Kiểm tra xem có thiết bị I2C ở address nhất định
static bool i2c_device_present(uint8_t addr) {
  Wire.begin(); // đảm bảo Wire đã init (dùng default pins hoặc caller đã init trước)
  Wire.beginTransmission(addr);
  uint8_t err = Wire.endTransmission();
  return (err == 0);
}

void test_oled_probe_and_init() {
  // Nếu không thấy OLED ở 0x3C thì bỏ qua test này (skip)
  if (!i2c_device_present(OLED_I2C_ADDR)) {
    TEST_IGNORE_MESSAGE("No I2C device at 0x3C — skipping OLED tests");
    return;
  }

  // Init OLED (mặc định Wire.get pins nếu bạn đã gọi Wire.begin(sda,scl) ở sketch)
  OLEDSSD1306_Init();
  OLEDSSD1306_DisplayNotify("OLED Init OK");
  delay(VISUAL_PAUSE_MS);

  TEST_ASSERT_TRUE_MESSAGE(true, "OLED init executed (visual verification required)");
}

void test_oled_font_colors_and_notify() {
  // White
  OLEDSSD1306_SetFontColor(SSD1306_WHITE);
  OLEDSSD1306_DisplayNotify("Font: WHITE");
  delay(VISUAL_PAUSE_MS);

  // Inverse (invert)
  OLEDSSD1306_SetFontColor(SSD1306_INVERSE);
  OLEDSSD1306_DisplayNotify("Font: INVERSE");
  delay(VISUAL_PAUSE_MS);

  // Back to white
  OLEDSSD1306_SetFontColor(SSD1306_WHITE);
  OLEDSSD1306_DisplayNotify("Font: WHITE again");
  delay(VISUAL_PAUSE_MS);

  TEST_ASSERT_TRUE_MESSAGE(true, "Font color variants displayed (visual check)");
}

void test_oled_position_and_textsize_and_data_display() {
  // Demonstrate cursor + text size + data display
  // Note: setCursor expects pixel coordinates. If your lib expects columns/rows adjust accordingly.
  OLEDSSD1306_SetLocation(0, 0);
  OLEDSSD1306_SetTextSize(1); // integer sizes — if your lib uses float it will be truncated
  OLEDSSD1306_DisplayNotify("Position (0,0) size 1");
  delay(VISUAL_PAUSE_MS);

  OLEDSSD1306_SetLocation(0, 10);
  OLEDSSD1306_SetTextSize(2);
  OLEDSSD1306_DisplayNotify("Position (0,10) size 2");
  delay(VISUAL_PAUSE_MS);

  // Display numeric data
  OLEDSSD1306_SetTextSize(1);
  for (int hr = 60; hr <= 64; hr++) {
    OLEDSSD1306_DisplayData(hr, 98);
    delay(500);
  }

  TEST_ASSERT_TRUE_MESSAGE(true, "Position/textsize/data displayed (visual check)");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  UNITY_BEGIN();

  RUN_TEST(test_oled_probe_and_init);
  RUN_TEST(test_oled_font_colors_and_notify);
  RUN_TEST(test_oled_position_and_textsize_and_data_display);

  UNITY_END();
}

void loop() {
  // Nothing to do here
  delay(1000);
}
