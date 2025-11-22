# THƯ VIỆN KẾT NỐI WIFI

Đây là thư viện thực hiện chức năng kết nối wifi thông qua nhập tên và mật khẩu wifi vào `WIFI_SSID` và `WIFI_PASSWORD` trong file `ConnectWiFi.h`

API được xây dựng

- `connectWiFi()` dùng để kết nối WiFi, time out cho lần kết nối là 10s

- `handleWiFiReconnect()` dùng để kết nối lại khi WiFi mất kết nối, mỗi lần kết nối lại cách nhau 5s

Các API hỗ trợ xây dựng

- Thư viện `WiFi.h`

  - `WiFi.mode(WIFI_STA)` dùng để bật chức năng kết nối WiFi trên ESP32

  - `WiFi.begin(WIFI_SSID, WIFI_PASSWORD)` dùng để kết nối WiFi

  - `WiFi.status()` dùng để lấy trạng thái kết nối WiFi hiện tại

  - `WL_CONNECTED` thể hiện trạng thái kết nối thành công

  - `WL_CONNECTED_FAIL` thể hiện trạng thái kết nối không thành công

  - `WiFi.localIP()` dùng để lấy địa chỉ IPv4 của WiFi
