# THƯ VIỆN THỰC HIỆN TOGGLE TRẠNG THÁI CỦA 1 CHÂN GPIO

Thư viện sử dụng API (Hàm) `digitalTogglePin();` để toggle 1 chân GPIO bất kỳ

- Tham số truyền vào là 1 chân GPIO bất kỳ

- Chân GPIO này sẽ được toggle 500ms/1 lần

API dùng để xây dựng

- `digitalWrite(pin, level_loggic_of_pin)` dùng để ghi 1 mức logic ra 1 chân pin được chỉ định

- `digitalRead(pin)` dùng để đọc mức logic hiện tại trên 1 chân pin được chỉ định

- `delay(time_ms)` delay 1 khoảng mili giây, API này sẽ gây block
