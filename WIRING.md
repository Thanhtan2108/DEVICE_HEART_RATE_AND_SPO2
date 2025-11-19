# KẾT NỐI MODULE MAX30102, OLED VỚI ESP32 WROOM 32

## Phân tích các chân trên module MAX30102

VIN: Nguồn vào (3.3V)

SCL: I2C Clock

SDA: I2C Data

INT: Interrupt (ngắt)

IR0: Có thể là IR LED driver hoặc output

R0: Có thể là Red LED driver hoặc output

GND: Ground

## Sơ đồ kết nối đề xuất

```text
ESP32 WROOM-32    →    MAX30102 Module          →    OLED 0.96"
==================    ==========================    =============
3.3V              →    VIN                      →    VCC
GND               →    GND                      →    GND
GPIO 22           →    SCL                      →    SCL
GPIO 21           →    SDA                      →    SDA
GPIO 4            →    INT (Không kết nối)*     →    (Không kết nối)
-                 →    IR0 (Không kết nối)      →    (Không kết nối)
-                 →    R0 (Không kết nối)       →    (Không kết nối)
```

>*Nếu muốn dùng ngắt thì kết nối

### Trường hợp NÊN dùng INT

NÊN dùng INT nếu:

- Muốn thêm chế độ sleep để tiết kiệm pin

- Nâng sample rate lên cao (>1000 sps) để cho hiệu suất cao

- Cần phản hồi real-time cho ứng dụng y tế

### 🔧 CÁC LOẠI INTERRUPT TRONG MAX30102

- FIFO Almost Full (A_FULL):

```cpp
// Kích hoạt khi FIFO còn X samples trống
particleSensor.enableAFULL();
// Set ngưỡng: 0x0 = 0 samples trống, 0xF = 15 samples trống
```

- New Data Ready (PPG_RDY):

```cpp
// Mỗi sample mới đều tạo interrupt
particleSensor.enableDATARDY();
```

- Proximity Detect (PROX_INT):

```cpp
// Khi phát hiện vật thể (ngón tay)
particleSensor.enablePROXIMITY();
```
