#include "firebaseUpload.h"

// Đối tượng chứa dữ liệu giao tiếp với Firebase
FirebaseData fbdo;

// Cấu hình dự án Firebase
FirebaseAuth auth;

// Config kết nối
FirebaseConfig config;

// ===== FreeRTOS Objects =====
QueueHandle_t firebaseDataQueue = NULL;
TaskHandle_t firebaseTaskHandle = NULL;
static SemaphoreHandle_t firebaseEnabledSemaphore = NULL;
static bool firebaseTaskEnabled = true;

// ===== THÊM: Biến kiểm soát =====
static volatile bool firebaseReady = false;
static uint32_t lastSuccessfulUpload = 0;
static uint32_t consecutiveFailures = 0;
static const uint32_t MAX_CONSECUTIVE_FAILURES = 5;
static const uint32_t FAILURE_BACKOFF_MS = 30000; // 30s

void connectToWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 20000) {
        delay(500);
        Serial.print(".");
        esp_task_wdt_reset(); // QUAN TRỌNG: Reset watchdog
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("📡 IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nWiFi connection failed!");
    }
}

void connectToFirebase() {
    Serial.println("🔥 Connecting to Firebase...");

    config.database_url = FIREBASE_DATABASE_URL;
    config.signer.tokens.legacy_token = FIREBASE_AUTH;
    
    // THÊM: Timeout configuration
    config.timeout.serverResponse = 10 * 1000; // 10 seconds
    config.timeout.rtdbKeepAlive = 45 * 1000;
    config.timeout.rtdbStreamReconnect = 1 * 1000;
    config.timeout.rtdbStreamError = 3 * 1000;

    Firebase.reconnectWiFi(true);
    
    // THÊM: Set max retry
    fbdo.setBSSLBufferSize(1024, 1024); // Giảm buffer để tiết kiệm RAM
    
    Firebase.begin(&config, &auth);

    // Đợi xác thực thành công với timeout
    Serial.print("Đang xác thực");
    uint32_t authStart = millis();
    while (!Firebase.ready() && (millis() - authStart < 30000)) {
        Serial.print(".");
        delay(500);
        esp_task_wdt_reset(); // QUAN TRỌNG
    }
    
    if (Firebase.ready()) {
        Serial.println("\nFirebase kết nối thành công!");
        firebaseReady = true;
        consecutiveFailures = 0;
    } else {
        Serial.println("\nFirebase kết nối thất bại!");
        firebaseReady = false;
    }
}

String buildPath(const String &key) {
    // FIXED: Sử dụng static buffer để tránh memory fragmentation
    static char pathBuffer[128];
    
    const char* root = FIREBASE_PATH_ROOT.c_str();
    const char* keyStr = key.c_str();
    
    // Tính toán độ dài
    int rootLen = strlen(root);
    int keyLen = strlen(keyStr);
    
    // Kiểm tra overflow
    if (rootLen + keyLen + 2 > sizeof(pathBuffer)) {
        Serial.println("⚠️ Path too long!");
        return String(root); // Fallback
    }
    
    // Build path
    strcpy(pathBuffer, root);
    if (pathBuffer[rootLen-1] != '/' && keyStr[0] != '/') {
        strcat(pathBuffer, "/");
    }
    if (keyStr[0] == '/') {
        strcat(pathBuffer, keyStr + 1);
    } else {
        strcat(pathBuffer, keyStr);
    }
    
    return String(pathBuffer);
}

void sendDataToRTDB(int heartRate, int spo2) {
    if (!firebaseReady || !Firebase.ready()) {
        Serial.println("Firebase chưa sẵn sàng!");
        return;
    }

    // Kiểm tra backoff sau nhiều lỗi liên tiếp
    if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
        if (millis() - lastSuccessfulUpload < FAILURE_BACKOFF_MS) {
            return; // Skip upload during backoff
        }
        // Reset sau backoff period
        consecutiveFailures = 0;
        Serial.println("⚠️ Retrying after backoff period...");
    }

    String rootPath = FIREBASE_PATH_ROOT;

    FirebaseJson json;
    json.set("Heart_Rate", heartRate);
    json.set("SpO2", spo2);
    json.set("timestamp", (int)millis());
    
    String path = buildPath("");

    Serial.printf("Đang gửi dữ liệu: HR=%d, SpO2=%d ... ", heartRate, spo2);

    // THÊM: Timeout cho operation
    fbdo.setResponseSize(1024); // Giảm response buffer
    
    if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
        Serial.println("THÀNH CÔNG!");
        lastSuccessfulUpload = millis();
        consecutiveFailures = 0;
    } else {
        Serial.println("THẤT BẠI!");
        Serial.printf("Lý do: %s (HTTP Code: %d)\n", 
                     fbdo.errorReason().c_str(), 
                     fbdo.httpCode());
        consecutiveFailures++;
        
        // Nếu lỗi nghiêm trọng, thử reconnect
        if (fbdo.httpCode() <= 0 || fbdo.httpCode() == 401) {
            Serial.println("⚠️ Attempting to reconnect...");
            firebaseReady = false;
            // Sẽ được reconnect trong task loop
        }
    }
}

// ===== FreeRTOS Task Function =====
static void firebaseUploadTask(void *pvParameters) {
    SensorData_t receivedData;
    
    Serial.println("📤 Firebase Upload Task started");
    
    // Thêm vào watchdog
    esp_task_wdt_add(NULL);
    
    uint32_t lastReconnectAttempt = 0;
    const uint32_t RECONNECT_INTERVAL = 60000; // 1 phút
    
    while (1) {
        // Reset watchdog ĐẦU TIÊN
        esp_task_wdt_reset();
        
        // Kiểm tra WiFi và reconnect nếu cần
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("⚠️ WiFi disconnected! Reconnecting...");
            connectToWifi();
            firebaseReady = false;
        }
        
        // Kiểm tra Firebase và reconnect nếu cần
        if (!firebaseReady && (millis() - lastReconnectAttempt > RECONNECT_INTERVAL)) {
            lastReconnectAttempt = millis();
            Serial.println("⚠️ Firebase not ready! Attempting reconnect...");
            connectToFirebase();
        }
        
        // Kiểm tra trạng thái enabled
        if (firebaseTaskEnabled && firebaseReady) {
            // Chờ dữ liệu từ queue với timeout ngắn
            if (xQueueReceive(firebaseDataQueue, &receivedData, pdMS_TO_TICKS(1000)) == pdTRUE) {
                // Reset watchdog trước khi gửi
                esp_task_wdt_reset();
                
                // Gửi lên Firebase
                sendDataToRTDB(receivedData.heartRate, receivedData.spo2);
                
                // Reset watchdog sau khi gửi
                esp_task_wdt_reset();
                
                // Delay nhỏ để tránh spam Firebase
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        } else {
            // Khi task bị disable hoặc Firebase chưa ready
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
        // Reset watchdog cuối vòng lặp
        esp_task_wdt_reset();
    }
    
    // Task này không bao giờ delete
    vTaskDelete(NULL);
}

// ===== API Functions =====
void firebaseUploadInit() {
    // Khởi tạo Queue với kích thước 10 phần tử
    firebaseDataQueue = xQueueCreate(10, sizeof(SensorData_t));
    
    if (firebaseDataQueue == NULL) {
        Serial.println("Failed to create Firebase Data Queue!");
        return;
    }
    
    Serial.println("Firebase Data Queue created successfully");
    
    // Kết nối WiFi
    connectToWifi();
    
    // Kết nối Firebase
    if (WiFi.status() == WL_CONNECTED) {
        connectToFirebase();
    }
    
    // Tạo FreeRTOS task upload Firebase
    // FIXED: Tăng stack size lên 8192 bytes (8KB)
    BaseType_t xReturn = xTaskCreatePinnedToCore(
        firebaseUploadTask,           // Function của task
        "FirebaseUploadTask",         // Tên task
        8192,                         // Stack size (TĂNG TỪ 4096)
        NULL,                         // Parameter
        1,                            // Priority (0 = lowest, 25 = highest)
        &firebaseTaskHandle,          // Task handle
        0                             // FIXED: Pin to Core 0 (tránh conflict với WiFi)
    );
    
    if (xReturn == pdPASS) {
        Serial.println("Firebase Upload Task created successfully");
    } else {
        Serial.println("Failed to create Firebase Upload Task!");
    }
}

bool sendSensorDataToFirebaseQueue(int heartRate, int spo2, TickType_t ticksToWait) {
    if (firebaseDataQueue == NULL) {
        Serial.println("Firebase Queue not initialized!");
        return false;
    }
    
    SensorData_t data;
    data.heartRate = heartRate;
    data.spo2 = spo2;
    data.timestamp = xTaskGetTickCount();
    
    // Gửi dữ liệu vào queue
    if (xQueueSend(firebaseDataQueue, &data, ticksToWait) == pdTRUE) {
        return true;
    } else {
        Serial.println("⚠️ Firebase Queue is full, data dropped!");
        return false;
    }
}

// ===== Power Control Functions =====
void firebaseUploadSetEnabled(bool enabled) {
    firebaseTaskEnabled = enabled;
    if (enabled) {
        Serial.println("✅ Firebase Upload Task resumed");
    } else {
        Serial.println("⏸️ Firebase Upload Task paused");
    }
}

bool firebaseUploadIsEnabled() {
    return firebaseTaskEnabled;
}

// ===== THÊM: Hàm utility =====
bool firebaseUploadIsReady() {
    return firebaseReady && Firebase.ready();
}

uint32_t firebaseUploadGetConsecutiveFailures() {
    return consecutiveFailures;
}

void firebaseUploadForceReconnect() {
    firebaseReady = false;
    consecutiveFailures = 0;
}
