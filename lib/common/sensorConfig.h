#pragma once

#ifndef SENSORCONFIG_H
#define SENSORCONFIG_H

// LED amplitude (0x00..0xFF). Typical: 0x3F (~12mA), 0x7F (~25mA), 0xFF (~50mA)
#define SENSOR_LED_POWER 0x3F // ledBrightness

// FIFO sample average (1,2,4,8,16,32). Use 1 for no averaging.
#define SENSOR_FIFO_SAMPLEAVG 1

// LED mode: 1=RED only, 2=RED+IR, 3=RED+IR+GREEN
#define SENSOR_LED_MODE 2

// SENSOR sampling & decimation
// Note: spo2_algorithm.h expects FreqS = 25 (25 Hz). Set SENSOR_SAMPLE_RATE to an allowed device sample rate (e.g., 100,200).
#define SENSOR_SAMPLE_RATE 100
#define ALGO_FREQ 25
#define DECIMATE_FACTOR (SENSOR_SAMPLE_RATE / ALGO_FREQ)

// Pulse width (69,118,215,411 us). 411 -> best SNR (18-bit)
#define SENSOR_PULSE_WIDTH 411

// ADC range: 2048, 4096, 8192, 16384
#define SENSOR_ADC_RANGE 4096

// FIFO almost full threshold (0x00=32 samples, 0x0F=17). Choose small so INT triggers often, e.g. 5~8
#define SENSOR_FIFO_A_FULL 5

#endif // SENSORCONFIG_H

/*=====================EXPLAIN===================*/
/*
Giải thích rõ ràng – logic – theo datasheet MAX30102 + code SparkFun + yêu cầu của thuật toán SPO2 Maxim về tại sao lại chọn đúng những giá trị cấu hình này, và tại sao chúng đi chung với nhau để đảm bảo độ chính xác cao, tín hiệu ổn định, không mất mẫu, và tối ưu cho FreeRTOS + ISR + INT.

💡 1) LED Amplitude — SENSOR_LED_POWER 0x3F (~12mA)
✔ Ý nghĩa

Điều chỉnh cường độ chiếu LED đỏ & IR.

Giá trị lớn → tín hiệu mạnh, SNR tốt → nhưng tăng tiêu thụ điện, nóng cảm biến, dễ saturate ADC (tràn 18-bit).

Giá trị nhỏ → tiết kiệm điện nhưng nhiễu nhiều.

✔ Tại sao chọn 0x3F (~12 mA)?

Vì:

| Lý do                                               | Giải thích                                                                          |
| --------------------------------------------------- | ----------------------------------------------------------------------------------- |
| Tránh saturate ADC                                  | Phần lớn da tay người Việt có độ hấp thụ trung bình → 12mA đủ mạnh nhưng không tràn |
| Đạt SNR đủ để thuật toán Maxim hoạt động tốt        | SNR đủ lớn để phân biệt peak- valley nhịp tim                                       |
| Giảm nhiệt & tiêu thụ                               | MAX30102 rất dễ nóng nếu bật LED >25mA (đặc biệt LED đỏ)                            |
| Mặc định SparkFun library cũng dùng mức tương đương | Tương thích tốt với cảm biến phổ thông                                              |

👉 Giá trị 0x3F là mức thực tế dùng nhiều nhất trên wearable (12–15 mA).

💡 2) FIFO Sample Averaging — SENSOR_FIFO_SAMPLEAVG 1

(= không lọc trung bình hardware)

✔ Ý nghĩa

MAX30102 có thể tự động trung bình 2–32 mẫu để giảm nhiễu.

✔ Tại sao đặt = 1 (không dùng)?

| Lý do                                                      | Giải thích                                    |
| ---------------------------------------------------------- | --------------------------------------------- |
| Thuật toán Maxim SPO2 **đã tự làm lọc (ALGORITHM FILTER)** | Tránh 2 lớp lọc → gây méo sóng PPG            |
| Averaging hardware làm trễ pha → giảm chính xác HR         | Không phù hợp khi cần đúng shape của waveform |
| Decimation (downsampling) đã làm trung bình rồi            | DECIMATE_FACTOR đã averaging @100 → 25 Hz     |

👉 FIFO average = 1 là đúng chuẩn khi dùng thuật toán SPO2 MAXIM gốc.

💡 3) LED Mode — SENSOR_LED_MODE 2 (RED + IR)
✔ ý nghĩa

1: chỉ RED → chỉ đo nhịp tim (không đo SpO2).

2: RED + IR → cần để tính SpO2 bằng thuật toán Maxim.

3: thêm GREEN → cho wearable HR cao cấp nhưng MAX30102 không dùng thuật toán MAXIM với GREEN.

✔ Tại sao chọn mode 2?

Để đọc được SpO2 (thuật toán cần cả IR và RED).

Tối ưu tốc độ (GREEN làm sampleRate/ việc xử lý nặng hơn).

Tối ưu điện năng.

👉 Mode 2 là cấu hình chuẩn cho MAX30102 để đọc SPO2 và HR.

💡 4) Sample Rate — SENSOR_SAMPLE_RATE 100 Hz
✔ Ý nghĩa

MAX30102 cho phép: 50 / 100 / 200 / 400 / 800 / 1000 / 1600 / 3200 Hz.

✔ Tại sao chọn 100 Hz?

| Lý do                               | Giải thích                                       |
| ----------------------------------- | ------------------------------------------------ |
| Thuật toán Maxim SPO2 yêu cầu 25 Hz | Bạn cần DECIMATE để giảm từ 100 → 25 Hz          |
| 100 Hz > đủ để capture waveform PPG | Nhịp tim người là 1–3 Hz, cần oversample ~20–50x |
| 200–400 Hz → noise nhiều + tốn pin  | Không cần cho SPO2/HR                            |
| 50Hz < không đủ oversampling        | Thậm chí thuật toán có thể fail                  |

👉 100 Hz = tốc độ tối ưu cho wearable SPO2 vì:

đủ nhanh → tín hiệu sạch

không quá nhanh → tiết kiệm điện

dễ chia về 25 Hz (DECIMATE_FACTOR = 4)

💡 5) Thuật toán Maxim SPO2 cần 25 Hz

```cpp
ALGO_FREQ = 25
DECIMATE_FACTOR = SENSOR_SAMPLE_RATE / ALGO_FREQ = 100 / 25 = 4
```

✔ Tại sao?

Maxim SPO2 algorithm (file spo2_algorithm.c) cứng mặc định với input 25 Hz.

Nếu đưa tốc độ khác → HR/SPO2 sẽ sai.

👉 Đây là lý do chọn 100 Hz, không phải 200 / 50.

💡 6) Pulse Width — SENSOR_PULSE_WIDTH 411 us
✔ Ý nghĩa

Pulse width tương ứng với độ phân giải ADC:

| Pulse Width | ADC Resolution      |
| ----------- | ------------------- |
| 69 µs       | 15-bit              |
| 118 µs      | 16-bit              |
| 215 µs      | 17-bit              |
| **411 µs    | 18-bit (tốt nhất)** |

✔ Tại sao chọn 411 µs?

| Lý do                      | Giải thích                              |
| -------------------------- | --------------------------------------- |
| Tín hiệu mượt nhất         | 18-bit → phân giải rất cao              |
| SNR tăng                   | Do nhiều photon hơn                     |
| Tương thích với SPO2 Maxim | Thuật toán xử lý tốt hơn khi nhiễu thấp |

👉 Đây là lựa chọn chuẩn nhất để lấy dữ liệu SPO2 chính xác.

💡 7) ADC Range — SENSOR_ADC_RANGE 4096
✔ Ý nghĩa

Dải ADC rộng → ít saturate nhưng độ phân giải thấp hơn.

Dải ADC hẹp → độ phân giải cao nhưng dễ saturate.

✔ Tại sao chọn 4096 (trung bình)?

| Lý do                             | Giải thích                               |
| --------------------------------- | ---------------------------------------- |
| 2048 dễ saturate khi LED 12mA     | Đặt thấp sẽ gây clipping                 |
| 8192–16384 → giảm độ phân giải    | Không cần thiết cho SPO2                 |
| 4096 cân bằng giữa SNR & headroom | Lý tưởng cho LED 12mA + pulsewidth 411us |

👉 4096 là mức “sweet spot”.

💡 8) FIFO Almost Full — SENSOR_FIFO_A_FULL 5
✔ Ý nghĩa

Khi FIFO còn 5 sample slots trống → phát INT.

✔ Tại sao chọn 5?

| Lý do                                      | Giải thích                                |
| ------------------------------------------ | ----------------------------------------- |
| Cho INT “bắn” liên tục nhưng không quá dày | INT ~25 Hz sau decimation                 |
| Không quá nhiều mẫu dồn → giảm trễ         | Task phản ứng nhanh với FIFO              |
| Không làm nghẽn đọc I²C                    | Nếu 0 → phải đọc 32 mẫu/lần, load CPU cao |

👉 Giá trị 5 = tối ưu cho FreeRTOS + ISR.

🎯 Kết luận — Tại sao các giá trị này đi cùng nhau?

| Tham số     | Giá trị | Vai trò                | Tại sao phù hợp với cấu hình còn lại |
| ----------- | ------- | ---------------------- | ------------------------------------ |
| LED Power   | 12mA    | SNR & an toàn          | Không saturate khi pulsewidth 411us  |
| FIFO Avg    | 1       | Không lọc HW           | Vì ta đã decimate & thuật toán xử lý |
| LED Mode    | 2       | Đo SPO2                | Cần RED+IR cho thuật toán Maxim      |
| Sample Rate | 100 Hz  | Tốc độ đọc thô         | Dễ chia về 25 Hz                     |
| ALGO_FREQ   | 25 Hz   | Chuẩn thuật toán Maxim | DECIMATE_FACTOR = 4 hợp lý           |
| Pulse Width | 411 µs  | 18-bit                 | Độ phân giải cao nhất                |
| ADC Range   | 4096    | Tránh saturate         | Tương thích LED=12mA & PW 411us      |
| FIFO A_FULL | 5       | Interrupt timing       | Tối ưu cho FreeRTOS task             |

👉 Tất cả các giá trị đều được chọn theo 3 tiêu chí:

Tối ưu chất lượng tín hiệu cho thuật toán SPO2 của Maxim

Tránh saturate / noise / artifact

Tối ưu hiệu suất cho ESP32 + FreeRTOS + I²C (100Hz → 25Hz decimation)

*/
