/**
 * @file main.cpp
 * @brief Đo nhịp tim và SpO2 bằng cảm biến MAX30102 trên ESP32
 * @author Senior Embedded Engineer
 * @date 2024
 * 
 * Hardware:
 * - Board: DOIT ESP32 DEVKIT V1
 * - Sensor: MAX30102 Module
 * 
 * Connections:
 * MAX30102 -> ESP32
 * VCC -> 3.3V
 * GND -> GND
 * SDA -> GPIO 21
 * SCL -> GPIO 22
 */

#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

// ============================================================================
// CONSTANTS & CONFIGURATIONS
// ============================================================================

// I2C Pins cho ESP32 (mặc định)
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 400000  // 400kHz Fast Mode

// Cấu hình đo lường
#define SAMPLE_AVG 4        // Số mẫu trung bình (1, 2, 4, 8, 16, 32)
#define LED_MODE 2          // 2 = Red + IR cho SpO2
#define SAMPLE_RATE 100     // Sample rate (50, 100, 200, 400, 800, 1000, 1600, 3200)
#define PULSE_WIDTH 411     // LED pulse width (69, 118, 215, 411 µs)
#define LED_BRIGHTNESS 60   // LED brightness (0-255)
#define ADC_RANGE 4096      // ADC range (2048, 4096, 8192, 16384)

// Cấu hình bộ lọc và phát hiện
#define FINGER_THRESHOLD 50000   // Ngưỡng phát hiện ngón tay
#define RATE_SIZE 4              // Kích thước buffer tính BPM trung bình
#define SPO2_WINDOW_SIZE 100     // Kích thước cửa sổ tính SpO2

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

MAX30105 particleSensor;

// Biến lưu trữ dữ liệu đo
uint32_t irBuffer[SPO2_WINDOW_SIZE];   // Buffer IR
uint32_t redBuffer[SPO2_WINDOW_SIZE];  // Buffer Red
int32_t bufferLength = SPO2_WINDOW_SIZE;

// Biến tính toán Heart Rate
byte rates[RATE_SIZE];      // Mảng lưu các giá trị HR
byte rateSpot = 0;
long lastBeat = 0;          // Thời điểm nhịp tim cuối
float beatsPerMinute = 0;
int beatAvg = 0;

// Biến tính toán SpO2
int32_t spo2 = 0;
int8_t validSPO2 = 0;
int32_t heartRate = 0;
int8_t validHeartRate = 0;

// Biến trạng thái
bool fingerDetected = false;
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 1000;  // In thông tin mỗi 1 giây

// Biến cho việc đọc dữ liệu từ FIFO
uint32_t currentIR = 0;
uint32_t currentRed = 0;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

bool initializeSensor();
void readAndProcessData();
void calculateHeartRate(uint32_t irValue);
void calculateSpO2();
void printResults();
void printSeparator();

// ============================================================================
// SETUP FUNCTION
// ============================================================================

void setup() {
    // Khởi tạo Serial Monitor
    Serial.begin(115200);
    delay(1000);
    
    Serial.println();
    printSeparator();
    Serial.println("💓 MAX30102 Heart Rate & SpO2 Monitoring System");
    Serial.println("   Board: DOIT ESP32 DEVKIT V1");
    Serial.println("   Sensor: MAX30102 Pulse Oximeter");
    printSeparator();
    Serial.println();
    
    // Khởi tạo I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_FREQ);
    
    Serial.println("🔧 Initializing I2C...");
    Serial.printf("   SDA: GPIO %d\n", I2C_SDA);
    Serial.printf("   SCL: GPIO %d\n", I2C_SCL);
    Serial.printf("   Frequency: %d Hz\n", I2C_FREQ);
    Serial.println();
    
    // Khởi tạo cảm biến MAX30102
    if (!initializeSensor()) {
        Serial.println("❌ FATAL ERROR: Sensor initialization failed!");
        Serial.println("   Please check:");
        Serial.println("   - Wiring connections");
        Serial.println("   - Power supply (3.3V)");
        Serial.println("   - I2C address (0x57)");
        while (1) {
            delay(1000);
        }
    }
    
    Serial.println("✅ Sensor initialized successfully!");
    Serial.println();
    printSeparator();
    Serial.println("📌 INSTRUCTIONS:");
    Serial.println("   1. Place your finger on the sensor");
    Serial.println("   2. Apply gentle pressure");
    Serial.println("   3. Keep finger still");
    Serial.println("   4. Wait for stable readings");
    printSeparator();
    Serial.println();
    Serial.println("⏳ Waiting for finger detection...\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Đọc và xử lý dữ liệu từ cảm biến
    readAndProcessData();
    
    // Kiểm tra có ngón tay hay không
    if (currentIR > FINGER_THRESHOLD) {
        // Phát hiện ngón tay
        if (!fingerDetected) {
            fingerDetected = true;
            Serial.println("👆 Finger detected! Starting measurements...\n");
            
            // Reset các biến khi bắt đầu đo mới
            beatAvg = 0;
            rateSpot = 0;
            lastBeat = 0;
            memset(rates, 0, sizeof(rates));
        }
        
        // Tính toán Heart Rate từ IR signal
        calculateHeartRate(currentIR);
        
        // In kết quả định kỳ
        if (millis() - lastPrintTime >= PRINT_INTERVAL) {
            printResults();
            lastPrintTime = millis();
        }
    } else {
        // Không phát hiện ngón tay
        if (fingerDetected) {
            fingerDetected = false;
            Serial.println("\n❌ Finger removed! Waiting for finger...\n");
            
            // Reset các biến
            beatAvg = 0;
            rateSpot = 0;
            lastBeat = 0;
            memset(rates, 0, sizeof(rates));
        }
    }
    
    delay(10);  // Delay nhỏ để tránh quá tải CPU
}

// ============================================================================
// FUNCTION IMPLEMENTATIONS
// ============================================================================

/**
 * @brief Khởi tạo cảm biến MAX30102
 * @return true nếu thành công, false nếu thất bại
 */
bool initializeSensor() {
    Serial.println("🔍 Scanning for MAX30102 sensor...");
    
    // Kiểm tra kết nối I2C
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        return false;
    }
    
    Serial.println("   ✓ Sensor found at address 0x57");
    
    // Đọc Part ID và Revision ID
    byte partID = particleSensor.readPartID();
    byte revID = particleSensor.getRevisionID();
    
    Serial.printf("   ✓ Part ID: 0x%02X\n", partID);
    Serial.printf("   ✓ Revision ID: 0x%02X\n", revID);
    
    // Cấu hình cảm biến
    Serial.println("\n⚙️  Configuring sensor...");
    
    particleSensor.setup(
        LED_BRIGHTNESS,  // LED brightness
        SAMPLE_AVG,      // Sample averaging
        LED_MODE,        // LED mode (2 = Red + IR)
        SAMPLE_RATE,     // Sample rate
        PULSE_WIDTH,     // Pulse width
        ADC_RANGE        // ADC range
    );
    
    Serial.printf("   ✓ LED Brightness: %d\n", LED_BRIGHTNESS);
    Serial.printf("   ✓ Sample Average: %d\n", SAMPLE_AVG);
    Serial.printf("   ✓ LED Mode: %d (Red + IR)\n", LED_MODE);
    Serial.printf("   ✓ Sample Rate: %d Hz\n", SAMPLE_RATE);
    Serial.printf("   ✓ Pulse Width: %d µs\n", PULSE_WIDTH);
    Serial.printf("   ✓ ADC Range: 4096 nA\n");
    
    // Cấu hình bổ sung
    particleSensor.setPulseAmplitudeRed(LED_BRIGHTNESS);
    particleSensor.setPulseAmplitudeIR(LED_BRIGHTNESS);
    
    // Đặt chế độ FIFO
    particleSensor.enableFIFORollover();  // Cho phép FIFO ghi đè khi đầy
    
    // Clear FIFO buffer
    particleSensor.clearFIFO();
    
    return true;
}

/**
 * @brief Đọc và xử lý dữ liệu từ cảm biến
 * 
 * Hàm này thực hiện:
 * 1. Kiểm tra xem có dữ liệu mới trong FIFO không
 * 2. Đọc giá trị IR và Red từ cảm biến
 * 3. Cập nhật vào biến global để sử dụng
 * 
 * Note: Thư viện SparkFun tự động quản lý FIFO buffer
 * và đọc dữ liệu mới nhất khi gọi getIR() và getRed()
 */
void readAndProcessData() {
    // Phương án 1: Đọc trực tiếp (đơn giản)
    // Thư viện tự động đọc từ FIFO khi gọi các hàm này
    currentIR = particleSensor.getIR();
    currentRed = particleSensor.getRed();
    
    /* 
    // Phương án 2: Kiểm tra FIFO trước khi đọc (nâng cao)
    // Sử dụng check() để kiểm tra có dữ liệu mới không
    if (particleSensor.check() > 0) {
        // Có dữ liệu mới trong FIFO
        currentIR = particleSensor.getFIFOIR();
        currentRed = particleSensor.getFIFORed();
        
        // Xóa FIFO sau khi đọc để tránh tràn
        particleSensor.nextSample();
    }
    */
    
    /*
    // Phương án 3: Đọc nhiều mẫu từ FIFO (cho xử lý tín hiệu nâng cao)
    // Sử dụng khi cần buffer để tính toán SpO2 chính xác
    static int bufferIndex = 0;
    
    // Kiểm tra số lượng samples có sẵn trong FIFO
    byte availableSamples = particleSensor.available();
    
    if (availableSamples > 0) {
        // Đọc tất cả samples có sẵn
        for (int i = 0; i < availableSamples; i++) {
            // Lưu vào buffer
            irBuffer[bufferIndex] = particleSensor.getFIFOIR();
            redBuffer[bufferIndex] = particleSensor.getFIFORed();
            
            bufferIndex++;
            if (bufferIndex >= SPO2_WINDOW_SIZE) {
                bufferIndex = 0;  // Wrap around
            }
            
            // Di chuyển đến sample tiếp theo trong FIFO
            particleSensor.nextSample();
        }
        
        // Cập nhật giá trị hiện tại (mẫu mới nhất)
        int lastIndex = (bufferIndex - 1 + SPO2_WINDOW_SIZE) % SPO2_WINDOW_SIZE;
        currentIR = irBuffer[lastIndex];
        currentRed = redBuffer[lastIndex];
    }
    */
}

/**
 * @brief Tính toán Heart Rate từ IR signal
 * @param irValue Giá trị IR hiện tại
 * 
 * Thuật toán:
 * 1. Sử dụng checkForBeat() từ thư viện để phát hiện đỉnh
 * 2. Tính khoảng thời gian giữa 2 đỉnh liên tiếp
 * 3. Chuyển đổi thành BPM = 60 / (delta_time)
 * 4. Lọc nhiễu bằng moving average filter
 */
void calculateHeartRate(uint32_t irValue) {
    // Sử dụng thuật toán phát hiện nhịp tim từ thư viện SparkFun
    if (checkForBeat(irValue) == true) {
        // Phát hiện một nhịp đập
        long delta = millis() - lastBeat;
        lastBeat = millis();
        
        // Tính BPM từ khoảng thời gian giữa 2 nhịp
        beatsPerMinute = 60 / (delta / 1000.0);
        
        // Lọc nhiễu: chỉ chấp nhận giá trị hợp lý (40-200 BPM)
        // Đây là range bình thường cho người trưởng thành
        if (beatsPerMinute > 40 && beatsPerMinute < 200) {
            // Thêm vào buffer để tính trung bình
            rates[rateSpot++] = (byte)beatsPerMinute;
            rateSpot %= RATE_SIZE;
            
            // Tính trung bình động (Moving Average)
            beatAvg = 0;
            int validCount = 0;
            for (byte x = 0; x < RATE_SIZE; x++) {
                if (rates[x] > 0) {  // Chỉ tính các giá trị hợp lệ
                    beatAvg += rates[x];
                    validCount++;
                }
            }
            if (validCount > 0) {
                beatAvg /= validCount;
            }
        }
    }
}

/**
 * @brief Tính toán SpO2 từ tín hiệu Red và IR
 * 
 * Công thức đơn giản hóa:
 * R = (AC_Red / DC_Red) / (AC_IR / DC_IR)
 * SpO2 = 110 - 25 × R
 * 
 * Note: Đây là công thức ước tính. Để có độ chính xác cao:
 * - Cần phân tích AC/DC components bằng FFT
 * - Calibrate với thiết bị chuẩn
 * - Compensate nhiệt độ, ánh sáng môi trường
 */
void calculateSpO2() {
    if (currentIR > FINGER_THRESHOLD && currentRed > FINGER_THRESHOLD) {
        // Tính tỷ lệ R = Red/IR
        // Trong thực tế cần tách AC và DC components
        float ratio = (float)currentRed / (float)currentIR;
        
        // Công thức ước tính SpO2 từ ratio (thực nghiệm)
        // Công thức này được rút ra từ nghiên cứu y khoa
        spo2 = 110 - 25 * ratio;
        
        // Giới hạn giá trị trong khoảng hợp lý (70-100%)
        if (spo2 < 70) spo2 = 70;
        if (spo2 > 100) spo2 = 100;
        
        validSPO2 = 1;
    } else {
        validSPO2 = 0;
        spo2 = 0;
    }
}

/**
 * @brief In kết quả đo lường ra Serial Monitor
 * 
 * Format output đẹp mắt, dễ đọc với:
 * - Giá trị Raw ADC (IR, Red)
 * - Heart Rate (BPM instant và average)
 * - SpO2 (%)
 * - Đánh giá trạng thái (Normal/Low/High)
 */
void printResults() {
    // Tính SpO2 trước khi in
    calculateSpO2();
    
    Serial.println("┌─────────────────────────────────────────────────┐");
    Serial.println("│           📊 MEASUREMENT RESULTS                │");
    Serial.println("├─────────────────────────────────────────────────┤");
    
    // Hiển thị giá trị Raw ADC
    Serial.printf("│ IR Signal    : %7lu (Raw ADC)              │\n", currentIR);
    Serial.printf("│ Red Signal   : %7lu (Raw ADC)              │\n", currentRed);
    Serial.println("├─────────────────────────────────────────────────┤");
    
    // Hiển thị Heart Rate
    if (beatAvg > 0) {
        Serial.printf("│ 💓 Heart Rate : %3d BPM (Avg)                 │\n", beatAvg);
        Serial.printf("│    Instant BPM: %3.0f BPM                      │\n", beatsPerMinute);
        
        // Đánh giá Heart Rate dựa trên chuẩn y khoa
        if (beatAvg < 60) {
            Serial.println("│    Status     : ⬇️  Low (Bradycardia)          │");
        } else if (beatAvg > 100) {
            Serial.println("│    Status     : ⬆️  High (Tachycardia)         │");
        } else {
            Serial.println("│    Status     : ✅ Normal                      │");
        }
    } else {
        Serial.println("│ 💓 Heart Rate : --- BPM (Calculating...)      │");
    }
    
    Serial.println("├─────────────────────────────────────────────────┤");
    
    // Hiển thị SpO2
    if (validSPO2 && beatAvg > 0) {
        Serial.printf("│ 🫁 SpO2       : %3ld %%                        │\n", spo2);
        
        // Đánh giá SpO2 dựa trên chuẩn y khoa
        if (spo2 >= 95) {
            Serial.println("│    Status     : ✅ Normal                      │");
        } else if (spo2 >= 90) {
            Serial.println("│    Status     : ⚠️  Low Normal                 │");
        } else {
            Serial.println("│    Status     : ❌ Low (Hypoxemia)            │");
        }
    } else {
        Serial.println("│ 🫁 SpO2       : --- % (Calculating...)        │");
    }
    
    Serial.println("└─────────────────────────────────────────────────┘");
    Serial.println();
}

/**
 * @brief In dòng phân cách để format output đẹp hơn
 */
void printSeparator() {
    Serial.println("═══════════════════════════════════════════════════");
}
