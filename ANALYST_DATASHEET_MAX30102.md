# PHÂN TÍCH DATASHEET CỦA CẢM BIẾN MAX30102

## [DATASHEET MAX30102](./datasheet/MAX30102.PDF)

## [BÀI THAM KHẢO 1](https://lastminuteengineers.com/max30100-pulse-oximeter-heart-rate-sensor-arduino-tutorial/)

## [BÀI THAM KHẢO 2](https://mecsu.vn/ho-tro-ky-thuat/may-do-oxy-xung-max30100-va-cam-bien-nhip-tim-voi-esp32.gOZ?srsltid=AfmBOopQd_psnvHr7vgtEFqF4KIfYurhH_Lzn1vLqB3L_ZQTeOlQoLth)

## 1. Chức năng của cảm biến MAX30102

MAX30102 là một module tích hợp cảm biến đo nồng độ oxy trong máu (SpO₂) và nhịp tim chuyên dụng cho các thiết bị đeo và thiết bị hỗ trợ thể thao.

Các chức năng chính:

- Đo nồng độ oxy trong máu (Pulse Oximetry)

- Đo nhịp tim (Heart Rate Monitoring)

- Chức năng tiệm cận (Proximity Detection)

- Đo nhiệt độ bên trong chip

## 2. Nguyên lý hoạt động

Cảm biến hoạt động dựa trên nguyên lý quang phản xạ (Reflective Solution):

Các bộ phận chính tham gia:

- LED đỏ (660nm) và LED hồng ngoại (880nm)

- Photodetector (cảm biến ánh sáng)

- Bộ xử lý tín hiệu tích hợp

Cơ chế hoạt động:

- LED chiếu ánh sáng vào mô cơ thể (thường là ngón tay, cổ tay)

- Photodetector nhận ánh sáng phản xạ lại

- Sự thay đổi thể tích máu theo nhịp tim làm thay đổi lượng ánh sáng hấp thụ

- Tín hiệu được xử lý qua ADC 18-bit và các bộ lọc số

- Dữ liệu được lưu trong FIFO để đọc qua I²C

## 3. Điện áp hoạt động

Hai nguồn điện riêng biệt:

- VDD = 1.8V - Điện áp cho phần logic và analog

  - Dải điện áp: 1.7V - 2.0V

- VLED+ = 3.1V - 5.25V - Điện áp cho LED (khuyến nghị 5.0V)

Dòng tiêu thụ:

- Chế độ hoạt động: 600μA - 1200μA

- Chế độ shutdown: 0.7μA (typ)

## 4. Giao thức giao tiếp

Giao tiếp qua chuẩn I²C

- Tốc độ: lên đến 400kHz

- Địa chỉ I2C : 0x37

- Địa chỉ ghi: 0xAE

- Địa chỉ đọc: 0xAF

- Chân SDA (dữ liệu) và SCL (xung nhịp)

## 5. Timing và Flow giao tiếp I²C

Các bước giao tiếp cơ bản:

### A. Ghi dữ liệu vào thanh ghi

```text
START → Gửi địa chỉ ghi (0xAE) + ACK → 
Gửi địa chỉ thanh ghi + ACK → 
Ghi dữ liệu + ACK → 
STOP
```

### B. Đọc dữ liệu từ thanh ghi

```text
START → Gửi địa chỉ ghi (0xAE) + ACK → 
Gửi địa chỉ thanh ghi + ACK → 
REPEATED START → 
Gửi địa chỉ đọc (0xAF) + ACK → 
Đọc dữ liệu + ACK/NACK → 
STOP
```

### C. Đọc dữ liệu từ FIFO (quan trọng)

```text
1. Đọc FIFO_WR_PTR để biết số mẫu có sẵn
2. Tính số mẫu cần đọc: NUM_AVAILABLE = FIFO_WR_PTR - FIFO_RD_PTR
3. Đọc FIFO_DATA với burst read:
   - Mỗi mẫu gồm 6 byte (3 byte RED + 3 byte IR)
   - Đọc liên tiếp 6 byte cho mỗi mẫu
4. Cập nhật FIFO_RD_PTR sau khi đọc xong
```

Các tham số timing quan trọng:

- Tần số SCL: 0-400kHz

- Thời gian setup data (t_SU;DAT): 100ns

- Thời gian bus free (t_BUF): 1.3μs

- Độ rộng xung SCL low (t_LOW): 1.3μs

- Độ rộng xung SCL high (t_HIGH): 0.6μs

Flow hoạt động điển hình:

- Khởi tạo cảm biến (thiết lập các thanh ghi)

- Kích hoạt chế độ đo (SpO₂ hoặc Heart Rate)

- Chờ interrupt từ cảm biến (FIFO almost full, data ready)

- Đọc dữ liệu từ FIFO

- Xử lý dữ liệu để tính toán SpO₂ và nhịp tim

## 6. Chi tiết chuyên sâu

### 1. PHÂN TÍCH TIMING CHI TIẾT

#### A. Timing I²C (Bit-level và Byte-level)

Bit Timing (trang 4-5):

```text
SCL Period = t_LOW + t_HIGH = 1.3µs + 0.6µs = 1.9µs (min)
→ Tốc độ tối đa: 1/1.9µs ≈ 526kHz (thực tế giới hạn 400kHz)
```

Frame Timing cho 1 byte:

```text
START (t_HD;STA = 0.6µs) 
→ 8 bit data (mỗi bit 1 SCL cycle) 
→ ACK bit 
→ STOP (t_SU;STO = 0.6µs)
```

Data Setup/Hold Time:

- t_SU;DAT = 100ns (data phải ổn định trước SCL rising edge)

- t_HD;DAT = 0-900ns (data giữ ổn định sau SCL falling edge)

#### B. Timing Sampling Data (ADC)

Pulse Width vs Sample Rate (trang 19, 23):

| LED_PW | Pulse Width | ADC Resolution | Sample Rate tối đa |
|--------|-------------|----------------|--------------------|
| 00     | 69µs        | 15-bit         | 3200 sps           |
| 01     | 118µs       | 16-bit         | 3200 sps           |
| 10     | 215µs       | 17-bit         | 1600 sps           |
| 11     | 411µs       | 18-bit         | 1000 sps           |

Slot Timing (trang 25):

```text
Red Pulse ──┐    ┌── IR Pulse
           │    │
           ▼    ▼
┌─────────┬────┬─────────┐
│   RED   │Slot│   IR    │
└─────────┴────┴─────────┘
         ↑    ↑
     427µs  525µs (với LED_PW=01)
```

#### C. Timing FIFO Data Collection

Ví dụ với sample rate 100sps:

- Thời gian giữa các sample: 10ms

- FIFO chứa 32 samples → thời gian đầy FIFO: 320ms

- FIFO Almost Full interrupt kích hoạt khi còn 17 samples trống (nếu set = 0xF)

### 2. FLOW HOẠT ĐỘNG CHI TIẾT

#### Flow Khởi tạo hệ thống

```text
1. POWER ON RESET
   ↓
2. Chờ PWR_RDY interrupt (tự động)
   ↓
3. Cấu hình cơ bản:
   - FIFO Config (0x08)
   - SpO2 Config (0x0A) 
   - LED Pulse Amplitude (0x0C, 0x0D)
   ↓
4. Set MODE register (0x09) để bắt đầu measurement
   ↓
5. Enable interrupts (0x02, 0x03)
```

#### Flow Data Collection (SpO2 Mode)

```text
1. PROXIMITY MODE (nếu enabled)
   - IR LED chiếu với current thấp (PILOT_PA)
   - Chờ vượt ngưỡng PROX_INT_THRESH
   ↓
2. Chuyển sang SpO2 MODE
   ↓
3. FIFO bắt đầu fill data
   ↓
4. Khi FIFO Almost Full → A_FULL interrupt
   ↓
5. Đọc FIFO_DATA (32 samples × 6 bytes = 192 bytes)
   ↓
6. Cập nhật FIFO_RD_PTR
   ↓
7. Lặp lại từ bước 3
```

#### Flow Temperature Reading

```text
1. Set TEMP_EN = 1 (0x21)
   ↓
2. Chờ 29ms (acquisition time)
   ↓
3. DIE_TEMP_RDY interrupt
   ↓
4. Đọc TINT (0x1F) + TFRAC (0x20)
   ↓
5. Tính temperature: T = TINT + TFRAC × 0.0625
```

### 3. PHÂN TÍCH REGISTER MAP CHI TIẾT

#### A. Status & Interrupt Registers (0x00-0x03)

Interrupt Status 1 (0x00):

- A_FULL: FIFO sắp đầy (set bởi FIFO_A_FULL[3:0])

- PPG_RDY: Data mới trong FIFO

- ALC_OVF: Ambient light cancellation bão hòa

- PROX_INT: Vượt ngưỡng proximity

- PWR_RDY: Nguồn đã sẵn sàng

Interrupt Enable (0x02-0x03): Enable các interrupt tương ứng

#### B. FIFO Registers (0x04-0x07)

FIFO_WR_PTR (0x04): Con trỏ ghi (0-31)

OVF_COUNTER (0x05): Đếm samples bị mất do tràn

FIFO_RD_PTR (0x06): Con trỏ đọc (0-31)

FIFO_DATA (0x07): Data register (đọc liên tục không auto-increment)

#### C. Configuration Registers (Quan trọng)

FIFO Configuration (0x08):

```text
[7:5] SMP_AVE - Sample averaging (1, 2, 4, 8, 16, 32 samples)
[4]   FIFO_ROLLOVER_EN - Cho phép ghi đè khi đầy
[3:0] FIFO_A_FULL - Ngưỡng almost full interrupt
```

Mode Configuration (0x09):

```text
[7] SHDN - Shutdown mode (1 = shutdown)
[6] RESET - Software reset (self-clearing)
[2:0] MODE - 010=HR, 011=SpO2, 111=Multi-LED
```

SpO2 Configuration (0x0A):

```text
[6:5] SPO2_ADC_RGE - ADC range (2048nA, 4096nA, 8192nA, 16384nA)
[4:2] SPO2_SR - Sample rate (50-3200 sps)
[1:0] LED_PW - Pulse width & ADC resolution
```

#### D. LED Control Registers

LED Pulse Amplitude (0x0C-0x0D):

- LED1_PA: Red LED current (0x00-0xFF = 0-50mA)

- LED2_PA: IR LED current

Multi-LED Mode Control (0x11-0x12):

- Cấu hình LED cho 4 time slots

- Mỗi slot có thể: None, Red, IR với current khác nhau

#### E. Temperature & Proximity

Temperature Registers (0x1F-0x21):

- TINT: Integer part (2's complement)

- TFRAC: Fractional part (0.0625°C/step)

Proximity Interrupt Threshold (0x30):

- Ngưỡng ADC count để kích hoạt proximity interrupt

### 4. ÁNH XẠ REGISTER - FLOW

| Flow Stage         | Registers liên quan                   | Chức năng            |
|--------------------|----------------------------------------|-----------------------|
| Khởi tạo           | 0x09 (RESET), 0x08, 0x0A, 0x0C, 0x0D    | Cấu hình hệ thống     |
| Proximity          | 0x10 (PILOT_PA), 0x30 (THRESH)         | Phát hiện vật thể     |
| Data Collection    | 0x07 (FIFO_DATA), 0x04–0x06 (Pointers) | Đọc dữ liệu           |
| Interrupt Handling | 0x00–0x03 (Status/Enable)              | Quản lý ngắt          |
| Temperature        | 0x1F–0x21, 0x03 (Enable)               | Đo nhiệt độ           |
| Power Management   | 0x09 (SHDN)                            | Quản lý nguồn         |

Ví dụ Flow điển hình với Register sequence:

```text
1. 0x09 ← 0x40 (RESET) → Reset system
2. 0x08 ← 0x4F (FIFO config: avg=4, rollover, A_FULL=15)
3. 0x0A ← 0x27 (SpO2: range=4096nA, 100sps, PW=411µs)
4. 0x0C ← 0x24 (Red LED = 7.2mA)
5. 0x0D ← 0x24 (IR LED = 7.2mA)
6. 0x02 ← 0x40 (Enable A_FULL interrupt)
7. 0x09 ← 0x03 (Start SpO2 mode)```
