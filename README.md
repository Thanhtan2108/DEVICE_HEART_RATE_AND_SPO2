# THIẾT BỊ ĐO NHỊP TIM VÀ SPO2 TRONG MÁU

## A. Phân tích, tìm hiều về MAX30102 - Chuẩn bị cho phát triển project

### Nội dung phân tích bên dưới

- [Phân tích datasheet MAX30102](./folderMD/ANALYST_DATASHEET_MAX30102.md)

## B. Phát triển project

>### Lưu ý : Project được phát triển trên hệ điều hành thời gian thực FreeRTOS để đảm bảo hệ thống chạy ổn định và đáp ứng yêu cầu thời gian thực

### 1. Thực hiện kết nối phần cứng

- [Phần cứng được kết nối với ESP32](./folderMD/WIRING.md)

### 2. Tạo project bằng PlatformIO

- Mở PlatformIO -> New Project

- Đặt tên Project (Không chứa ký tự đặc biệt, không cách, không dấu)

- Chọn Board : DOIT ESP32 DEVKIT V1

- Framework : arduino

- Chọn nơi lưu project

- Nhấn nút finish để bắt đầu tạo project

### 3. Cấu hình cho project

- Mở file platformio.ini

- Set tốc độ baud cho project để thực hiện debug trên Monitor

```ini
monitor_speed = 115200
```

- Cài đặt các thư viện hỗ trợ cho project

  - [Cài đặt thư viện hỗ trợ](./folderMD/LIBRARY_FOR_PROJECT.md)

- Cuối cùng file platformio.ini như sau:

```ini
[env:esp32doit-devkit-v1]
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino
monitor_speed = 115200
lib_deps = 
    sparkfun/SparkFun MAX3010x Pulse and Proximity Sensor Library@^1.1.2
    adafruit/Adafruit SSD1306@^2.5.15
    adafruit/Adafruit GFX Library@^1.12.3
```

### 4. Kiểm tra địa chỉ I2C của MAX30102 và OLED

- Sau khi kết nối phần cứng, để chắc chắn các module hoạt động được, cần kiểm tra xem phần cứng đã được kết nối theo I2C thành công chưa

- Phát triển 1 module để scan địa chỉ I2C

  - [Xem module scan I2C tại đây](./lib/scanI2CAddress/)

- Khi đã xác nhận:

  - Địa chỉ của MAX30102 là 0x37

  - Địa chỉ của OLED là 0x3C

- Tiếp tục phát triển các module khác

### **Chú ý quan trọng trong phát triển project**

- Để cho hệ thống thực hiện được các chức năng 1 cách độc lập, có thể chạy gần như là song song với nhau, dựa trên cơ chế lập lịch (schedular) của FreeRTOS

  -> Tạo các task vụ để chạy độc lập

- Vì phát triển hệ thống bằng FreeRTOS, module MAX30102 và OLED đều dùng chung kết nối I2C (cùng truy cập tài nguyên) nên cần 1 cơ chế bảo vệ tránh gây xung đột tài nguyên

  -> Sử dụng mutex

- Task hiển thị lên OLED cần phải chờ có dữ liệu từ task MAX30102 thì mới hiển thị được nên cần cơ chế đồng bộ

  -> Sử dụng Semaphore

- 2 task cần giao tiếp dữ liệu với nhau

  -> Sử dụng Queue

- Để tránh việc các task bị treo trong quá trình hoạt động làm hỏng hệ thống

  -> Sử dụng WDT

### 5. Phát triển module cho MAX30102

- Đây là module thực hiện nhiệm vụ giao tiếp ESP32 với MAX30102. Chức năng là đo nhịp tim và SpO2 trong máu thông qua ngón tay đặt lên cảm biến, sau đó ESP32 sẽ đọc dữ liệu này và xử lý dữ liệu

- [Xem module Max30102 tại đây](./lib/max30102Sensor/)

### 6. Phát triển module cho OLED

- Đây là module thực hiện chức năng hiển thị dữ liệu nhịp tim và SpO2 sau khi được ESP32 xử lý lên màn hình OLED. Ngoài ra còn hiển thị các thông báo

- [Xem module OLED tại đây](./lib/oledScreen/)

### 7. Phát triển module nút bật/tắt toàn bộ hệ thống

- Đây là module thực hiện chức năng bật/tắt toàn bộ hệ thống qua 1 nút nhấn, có led báo trạng thái hoạt động của hệ thống

- Khi tắt, ESP32 hoạt động ở chế độ tiết kiệm năng lượng

- [Xem module bật/tắt hệ thống tại đây](./lib/powerControl/)

## [C. Các vấn đề thắc mắc trong suốt quá trình phát triển project của bản thân người phát triển được ghi lại tại đây](./folderMD/EXPLAIN.md)
