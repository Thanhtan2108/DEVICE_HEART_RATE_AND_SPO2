#include "oled.h"

Adafruit_SSD1306 display(128, 64, &Wire, -1);

bool oled_init() {
    Serial.println("Initializing OLED...");
    
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("OLED not found! Check wiring.");
        return false;
    }
    
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    
    Serial.println("OLED initialized successfully!");
    return true;
}

void oled_show_waiting() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("Heart Rate & SpO2");
    display.println("Monitor");
    display.println("");
    display.println("Place finger on");
    display.println("sensor...");
    display.display();
}

void oled_show_data(int heart_rate, int spo2, long ir_value, bool has_finger) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    
    // Hiển thị nhịp tim
    display.print("Heart Rate: ");
    if (heart_rate > 0 && has_finger) {
        display.print(heart_rate);
        display.println(" BPM");
    } else {
        display.println("-- BPM");
    }
    
    // Hiển thị SpO2
    display.print("SpO2: ");
    if (spo2 > 0 && has_finger) {
        display.print(spo2);
        display.println(" %");
    } else {
        display.println("-- %");
    }
    
    display.println("-------------");
    
    // Hiển thị thông tin debug
    display.print("IR: ");
    display.println(ir_value);
    
    if (!has_finger) {
        display.setTextSize(1);
        display.setCursor(0, 50);
        display.println("No finger detected!");
    } else {
        display.setTextSize(1);
        display.setCursor(0, 50);
        display.println("Finger detected  ");
    }
    
    display.display();
}

void oled_show_error(const char* error_msg) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("ERROR:");
    display.println("");
    display.println(error_msg);
    display.println("");
    display.println("Check wiring and");
    display.println("restart device.");
    display.display();
}

// HÀM MỚI: Tắt hoàn toàn màn hình OLED
void oled_turn_off() {
    display.clearDisplay();
    display.display();
    // Một số OLED cần lệnh đặc biệt để thực sự tắt
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    Serial.println("OLED turned off completely");
}

// HÀM MỚI: Bật lại màn hình OLED
void oled_turn_on() {
    display.ssd1306_command(SSD1306_DISPLAYON);
    Serial.println("OLED turned on");
}
