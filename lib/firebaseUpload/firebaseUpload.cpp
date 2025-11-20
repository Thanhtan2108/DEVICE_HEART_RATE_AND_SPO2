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

void connectToWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 20000) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("📡 IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nWiFi connection failed!");
        // Có thể restart ESP ở đây nếu muốn
        // ESP.restart();
    }
}

void connectToFirebase() {
    Serial.println("🔥 Connecting to Firebase...");

    config.database_url = FIREBASE_DATABASE_URL;

    // Khuyến nghị nên dùng để kết nối Firebase khi không có kết nối app hoặc web
    config.signer.tokens.legacy_token = FIREBASE_AUTH;

    /* Khuyến khích dùng để kết nối Firebase khi có kết nối app hoặc web
    config.api_key = FIREBASE_API_KEY;

    Cách Email + Password (ổn định và bảo mật)
    auth.user.email = FIREBASE_USER_EMAIL;
    auth.user.password = FIREBASE_USER_PASSWORD;
    Serial.println("Sử dụng Email/Password Authentication");
    */

    Firebase.reconnectWiFi(true);
    Firebase.begin(&config, &auth);

    // Đợi xác thực thành công
    Serial.print("Đang xác thực");
    while (!Firebase.ready()) {
        Serial.print(".");
        delay(500);
    }
    Serial.println("\nFirebase kết nối thành công!");
}

String buildPath(const String &key) {
  String path = FIREBASE_PATH_ROOT;
  if (!path.endsWith("/")) path += "/";
  if (key.startsWith("/")) path += key.substring(1);
  else path += key;
  return path;
}

// Gửi dữ liệu trực tiếp bằng kiểu của dữ liệu
// void sendDataToRTDB(int heartRate, int SpO2) {
//     if (!Firebase.ready()) {
//         Serial.println("Firebase not ready!");
//         return;
//     }

//     String pathHR = buildPath("Heart_Rate");
//     String pathSpO2 = buildPath("SpO2");

//     bool success = true;

//     if (!Firebase.RTDB.setInt(&fbdo, pathHR.c_str(), heartRate)) {
//         Serial.printf("Failed HeartRate: %s\n", fbdo.errorReason().c_str());
//         success = false;
//     }

//     if (!Firebase.RTDB.setInt(&fbdo, pathSpO2.c_str(), SpO2)) {
//         Serial.printf("Failed SpO2: %s\n", fbdo.errorReason().c_str());
//         success = false;
//     }

//     if (success) {
//         Serial.println("Sent data successfully!");
//     }
// }


// Gửi dữ liệu dạng JSON một lần (nhanh hơn, gọn hơn, ít request hơn)
void sendDataToRTDB(int heartRate, int spo2) {
  if (!Firebase.ready()) {
    Serial.println("Firebase chưa sẵn sàng!");
    return;
  }

  String rootPath = FIREBASE_PATH_ROOT;

  FirebaseJson json;
  json.set("Heart_Rate", heartRate);
  json.set("SpO2", spo2);
  json.set("timestamp", millis());
  json.set("updated_at", Firebase.getCurrentTime());  // Thời gian từ Firebase server

  String path = buildPath("");  // Gửi vào root của device

  Serial.printf("Đang gửi dữ liệu: HR=%d, SpO2=%d ... ", heartRate, spo2);

  if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
    Serial.println("THÀNH CÔNG!");
  } else {
    Serial.println("THẤT BẠI!");
    Serial.printf("Lý do: %s (HTTP Code: %d)\n", fbdo.errorReason().c_str(), fbdo.httpCode());
  }
}

// ===== FreeRTOS Task Function =====
// Task này sẽ chạy liên tục và xử lý upload dữ liệu lên Firebase
static void firebaseUploadTask(void *pvParameters) {
    SensorData_t receivedData;
    
    Serial.println("📤 Firebase Upload Task started");
    
    // Thêm vào watchdog
    esp_task_wdt_add(NULL);
    
    while (1) {
        // Kiểm tra trạng thái enabled
        if (firebaseTaskEnabled) {
            // Chờ dữ liệu từ queue (timeout = 2000ms)
            if (xQueueReceive(firebaseDataQueue, &receivedData, pdMS_TO_TICKS(2000)) == pdTRUE) {
                // Nếu nhận được dữ liệu thì gửi lên Firebase
                sendDataToRTDB(receivedData.heartRate, receivedData.spo2);
            }
        } else {
            // Khi task bị disable, delay để không lãng phí CPU
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        // Reset watchdog
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
    connectToFirebase();
    
    // Tạo FreeRTOS task upload Firebase
    // Priority: 1 (thấp hơn task sensor)
    // Stack size: 4096 bytes
    BaseType_t xReturn = xTaskCreate(
        firebaseUploadTask,           // Function của task
        "FirebaseUploadTask",         // Tên task
        4096,                         // Stack size (bytes)
        NULL,                         // Parameter
        1,                            // Priority (0 = lowest, 25 = highest)
        &firebaseTaskHandle           // Task handle
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
