#ifndef OLED_H
#define OLED_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// Biến toàn cục
extern Adafruit_SSD1306 display;

// Hàm khởi tạo
bool oled_init();
// Hiển thị màn hình chờ
void oled_show_waiting();
// Hiển thị dữ liệu
void oled_show_data(int heart_rate, int spo2, long ir_value, bool has_finger);
// Hiển thị lỗi
void oled_show_error(const char* error_msg);
// Hàm mới: tắt/bật OLED hoàn toàn
void oled_turn_off();
void oled_turn_on();

#endif
