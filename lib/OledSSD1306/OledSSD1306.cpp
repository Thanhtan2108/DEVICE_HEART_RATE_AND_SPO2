#include "OledSSD1306.h"
#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
static uint16_t fontColor = SSD1306_WHITE;

void OLEDSSD1306_Init(uint8_t sda, uint8_t scl, uint8_t address) {
    // Optional: explicitly init Wire with chosen pins on ESP32
    Wire.begin(sda, scl);
    if (!display.begin(SSD1306_SWITCHCAPVCC, address)) {
        Serial.println(F("[OLED] SSD1306 allocation failed"));
        // thay vì for(;;) bạn có thể xử lý khác; ở đây giữ nguyên để debug
        for (;;);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(fontColor);
    display.setCursor(0, 0);
    display.display();
    Serial.println("[OLED] Initialized");
}

void OLEDSSD1306_SetFontColor(uint16_t color) {
    fontColor = color;
    display.setTextColor(fontColor);
}

void OLEDSSD1306_SetLocation(uint8_t x_pixel, uint8_t y_pixel) {
    display.setCursor(x_pixel, y_pixel);
}

void OLEDSSD1306_SetTextSize(float size) {
    if (size < 1) size = 1;
    display.setTextSize(size);
}

void OLEDSSD1306_DisplayNotify(const char* stringMessage) {
    display.print(stringMessage);
}

void OLEDSSD1306_DisplayData(const char* stringMessage, int DataValue) {
    OLEDSSD1306_DisplayNotify(stringMessage);
    display.println(DataValue);
    display.println("");
}

void OLEDSSD1306_ClearDisplay(void) {
    display.clearDisplay();
    display.setTextColor(fontColor);
    display.setCursor(0, 0);
}

void OLEDSSD1306_UpdateDisplay(void) {
    display.display();
}
