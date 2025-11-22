# THƯ VIỆN QUÉT KIỂM TRA KIỂM TRA ĐỊA CHỈ I2C CỦA CÁC SENSOR

Đây là thư viện dùng để quét, kiểm tra địa chỉ I2C của các sensor được kết nối với Vi điều khiển

## I2C_Init()

- Đây là API (hàm) khởi tạo I2C để bắt đầu sử dụng

- API `I2C_Init()`

  - Tham số truyền vào là 2 chân GPIO `SDA, SCL`

  - Tốc độ I2C có thể được truyền vào để chỉ định, mặc định là 100000 Hz

## I2C_ScanAddress()

- Đây là API (hàm) thực hiện quét kiểm tra địa chỉ I2C của Sensor

- API `I2C_ScanAddress()`

## Các API xây dựng

- Thư viện `Wire.h`

  - `Wire.begin(sdaPin, sclPin)` dùng để bắt đầu thực hiện giao tiếp I2C thông qua 2 chân SDA, SCL. Tường đương với `Start condition`

  - `Wire.setClock(speedI2C)` dùng để chỉ định tốc độ I2C, mặc định khi dùng thư viện `Wire.h`, nếu không chỉ định tốc độ thì mặc định là 100000 Hz

  - `Wire.beginTransmission(address)` dùng để bắt đầu truyền dữ liệu trên đường truyền I2C, mỗi dữ liệu truyền là 1 byte. Tương đương với `Frame Data`

  - `Wire.endTransmission()` dùng để kết thúc việc truyền dữ liệu trên I2C. Tương đương với `Stop condition`

- Thư viện `Arduino.h`

  - `Serial.println("")` dùng để in ra 1 chuỗi, kết thúc chuỗi có ký tự `\n` để xuống dòng

  - `Serial.print("")` dùng để in ra 1 chuỗi, kết thúc chuỗi không xuống dòng
