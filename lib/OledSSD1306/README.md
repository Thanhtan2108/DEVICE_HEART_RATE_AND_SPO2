# THƯ VIỆN HIỂN THỊ OLED SSD1306

Đây là thư viện có chức năng hiển thị thông tin, dữ liệu lên màn hình OLED

Các API được xây dựng

- `OLEDSSD1306_Init()` dùng để khởi tạo màn hình OLED, bắt đầu cho màn hình hoạt động

- `OLEDSSD1306_SetFontColor()` dùng để set màu chữ hiển thị trên màn hình

- `OLEDSSD1306_SetLocation()` dùng để thiết lập vị trí hiển thị trên màn hình ở cột nào hàng nào

- `OLEDSSD1306_SetTextSize()` dùng để thiết lập cỡ chữ

- `OLEDSSD1306_DisplayNotify()` dùng để hiển thị 1 chuỗi text ra màn hình OLED

- `OLEDSSD1306_DisplayData()` dùng để hiển thị dữ liệu của 2 đại lượng lên màn hình

- `OLEDSSD1306_ClearDisplay()` dùng để clear màn hình để hiển thị thông tin mới lên màn hình

- `OLEDSSD1306_UpdateDisplay()` dùng để cập nhật lại màn hình mới

Các API hỗ trợ xây dựng

- Thư viện `Adafruit_SSD1306.h`

  - `display.begin(SSD1306_SWITCHCAPVCC, OLEDSSD1306_ADDR)` dùng để bắt đầu thiết lập hiển thị màn hình

  - `display.clearDisplay()` dùng để xóa màn hình hiển thị

  - `display.display()` dùng để bật hiển thị màn hình

  - `display.setTextSize()` dùng để set cỡ chữ cho chuỗi text

  - display.setTextColor()` dùng để set màu hiển thị cho chuỗ text

    - `SSD1306_WHITE` màu trắng (pixel sáng)

    - `SSD1306_BLACK` màu đen (pixel tối)

    - `SSD1306_INVERSE` pixel đổi trạng thái

  - `display.setCursor()` set vị trí con trỏ hiển thị trên màn hình theo chiều x, y
