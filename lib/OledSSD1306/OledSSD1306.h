#ifndef OLEDSSD1306_H
#define OLEDSSD1306_H

#include <stdint.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Public API (procedural)
void OLEDSSD1306_Init(uint8_t sda = 21, uint8_t scl = 22, uint8_t address = 0x3C);
void OLEDSSD1306_SetFontColor(uint16_t color);
void OLEDSSD1306_SetLocation(uint8_t x_pixel, uint8_t y_pixel);
void OLEDSSD1306_SetTextSize(float size); // integer sizes only
void OLEDSSD1306_DisplayNotify(const char* stringMessage);
void OLEDSSD1306_DisplayData(const char* stringMessage, int DataValue);
void OLEDSSD1306_ClearDisplay(void);
void OLEDSSD1306_UpdateDisplay(void);

#endif // OLEDSSD1306_H
