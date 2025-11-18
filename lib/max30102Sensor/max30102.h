#ifndef MAX30102_H
#define MAX30102_H

#include <Arduino.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

// Biến toàn cục
extern MAX30105 particleSensor;

// Hàm khởi tạo
bool max30102_init();
// Đọc dữ liệu mới từ FIFO (trả về true khi có mẫu mới)
bool max30102_read_data(long *ir_value, long *red_value);
// Kiểm tra có ngón tay không với mẫu vừa đọc
bool max30102_has_finger(long ir_value);
// Tính toán nhịp tim dựa trên mẫu IR mới nhất
int max30102_get_heart_rate(long ir_value);
// Tính SpO2 (đơn giản)
int max30102_get_spo2(long ir_value, long red_value);
bool max30102_is_heart_rate_valid();
bool max30102_is_spo2_valid();

#endif
