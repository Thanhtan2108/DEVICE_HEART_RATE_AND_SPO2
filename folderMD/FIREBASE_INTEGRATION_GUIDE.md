# 📤 Firebase Integration Guide

## 📋 Tổng Quan

Đã tích hợp module `firebaseUpload` vào project nhằm gửi dữ liệu **Heart Rate, SpO2, và timestamp** lên Firebase Realtime Database.

---

## 🏗️ Kiến Trúc Tích Hợp

### **FreeRTOS Task Structure**

```text
sensorTask (Priority 3 - Core 1)
    ↓
    ├─ Đọc dữ liệu MAX30102
    ├─ Ghi vào measurementQueue (OLED)
    └─ Gửi dữ liệu hợp lệ lên firebaseDataQueue
         ↓
    firebaseUploadTask (Priority 1 - Core 0)
         ↓
         ├─ Chờ dữ liệu từ firebaseDataQueue
         └─ Gửi lên Firebase RTDB (JSON format)
```

### **Kernel Objects Được Sử Dụng**

| Kernel Object        | Mục Đích                                    | Kích Thước       |
| -------------------- | ------------------------------------------- | ---------------- |
| `firebaseDataQueue`  | Queue gửi dữ liệu từ sensor → Firebase task | 10 phần tử       |
| `firebaseTaskHandle` | Handle quản lý Firebase task                | -                |
| `firebaseUploadTask` | Task xử lý upload Firebase                  | 4096 bytes stack |

### **Power Control Integration**

- **ON state**: Firebase task hoạt động bình thường
- **OFF state**: Firebase task tạm dừng (disable) để tiết kiệm power
- **Callback**: `systemStateChangedCallback()` điều khiển bật/tắt task

---

## 🔧 Thay Đổi Được Thực Hiện

### **1. firebaseUpload.h**

- ✅ Thêm `extern firebaseTaskHandle`
- ✅ Thêm `firebaseUploadSetEnabled()` - điều khiển enable/disable task
- ✅ Thêm `firebaseUploadIsEnabled()` - lấy trạng thái task

### **2. firebaseUpload.cpp**

- ✅ Thêm `firebaseTaskEnabled` flag - quản lý trạng thái hoạt động
- ✅ Thêm watchdog support (`esp_task_wdt_add/reset`)
- ✅ Thêm power state check trong task loop
- ✅ Timeout queue: 2000ms (tăng từ 1000ms để chủ động hơn)

### **3. main.cpp**

- ✅ Thêm `#include "firebaseUpload.h"`
- ✅ Cập nhật `systemStateChangedCallback()` - gọi `firebaseUploadSetEnabled()`
- ✅ Sửa `sensorTask()` - thêm gọi `sendSensorDataToFirebaseQueue()` khi dữ liệu hợp lệ
- ✅ Khởi tạo Firebase trong `setup()` - gọi `firebaseUploadInit()`

---

## 📊 Dữ Liệu Được Gửi Lên Firebase

```json
{
  "Heart_Rate": 75,
  "SpO2": 98,
  "timestamp": 123456789,
  "updated_at": "2025-11-20T10:30:45Z"
}
```

### **Điều Kiện Gửi Dữ Liệu**

- ✅ Ngón tay được phát hiện (`has_finger == true`)
- ✅ HR dữ liệu hợp lệ (`hr_valid == true`)
- ✅ SpO2 dữ liệu hợp lệ (`spo2_valid == true`)
- ✅ Hệ thống đang bật (`powerControl_getState() == true`)
- ✅ Firebase task được enable

---

## 🔐 Credentials Firebase

**File:** `lib/firebaseUpload/firebaseUpload.h`

```cpp
// WiFi
WIFI_SSID = "P203&P204&P205"
WIFI_PASSWORD = "102duylam"

// Database URL
FIREBASE_DATABASE_URL = "https://heart-rate-and-spo2-78e6f-default-rtdb.asia-southeast1.firebasedatabase.app/"

// Auth (Legacy Token)
FIREBASE_AUTH = "YA9wnCYcl705LqGr13GTImQLGmTAit1FA6DEv9MG"
```

⚠️ **LƯU Ý:** Thay đổi credentials trong production!

---

## 🚨 Khả Năng Xảy Ra Lỗi & Giải Pháp

| Lỗi                              | Nguyên Nhân                          | Giải Pháp                                    |
| -------------------------------- | ------------------------------------ | -------------------------------------------- |
| "Firebase Queue not initialized" | `firebaseUploadInit()` chưa được gọi | Kiểm tra `setup()`                           |
| "Firebase Queue is full"         | Dữ liệu gửi quá nhanh                | Tăng kích thước queue hoặc giảm tần suất gửi |
| "Firebase chưa sẵn sàng"         | Không kết nối WiFi/Firebase          | Kiểm tra WiFi signal, credentials            |
| "THẤT BẠI! (HTTP Code: xxx)"     | Lỗi Firebase request                 | Kiểm tra logs, auth token, database rules    |
| Task bị crash                    | Stack size không đủ                  | Tăng `POWER_TASK_STACK_SIZE`                 |

---

## 📈 Performance Metrics

| Metric                 | Giá Trị      |
| ---------------------- | ------------ |
| Firebase Task Priority | 1 (thấp)     |
| Firebase Task Stack    | 4096 bytes   |
| Queue Size             | 10 items     |
| Queue Timeout          | 2000ms       |
| Sensor Task Priority   | 3 (cao hơn)  |
| Display Task Priority  | 2            |
| Power Control Priority | 5 (cao nhất) |

---

## 🧪 Testing Checklist

- [ ] WiFi kết nối thành công (kiểm tra Serial output)
- [ ] Firebase kết nối thành công (kiểm tra logs)
- [ ] Khi có ngón tay: dữ liệu được gửi lên (kiểm tra Firebase RTDB)
- [ ] Khi không có ngón tay: dữ liệu không được gửi (chỉ timestamp cũ)
- [ ] Khi hệ thống OFF: Firebase task tạm dừng (CPU tiết kiệm)
- [ ] Watchdog không trigger (kiểm tra resets)

---

## 📝 Notes

- **Firebase library:** Chạy chủ yếu trên Core 0 (WiFi), không xung đột I2C (Core 1)
- **Timestamp:** Sử dụng `millis()` (local) + `Firebase.getCurrentTime()` (server time)
- **Power Efficiency:** Firebase task disable khi system OFF, tiết kiệm pin
- **Queue Design:** Overwrite queue tránh dữ liệu cũ, Firebase queue kịp theo

---

**Version:** 1.0  
**Last Updated:** 2025-11-20  
**Author:** Integration Notes
