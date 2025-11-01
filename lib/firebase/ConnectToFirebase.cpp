#include "ConnectToFirebase.h"

ConnectToFirebase::ConnectToFirebase() {}

/* =====================
   === 1. Khởi tạo ===
   ===================== */
bool ConnectToFirebase::begin() {
  connectWiFi();
  syncNTP();
  connectFirebase();
  return isConnected();
}

/* =====================
   === 2. WiFi ===
   ===================== */
void ConnectToFirebase::connectWiFi() {
  Serial.println("\n📶 Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint8_t retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 40) {
    delay(500);
    Serial.print(".");
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi connected!");
    Serial.print("📡 IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ WiFi connection failed!");
  }
}

/* =====================
   === 3. NTP Time ===
   ===================== */
void ConnectToFirebase::syncNTP() {
  Serial.println("⏳ Syncing NTP time...");
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char buf[25];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    Serial.printf("✅ NTP synced: %s\n", buf);
  } else {
    Serial.println("⚠️ Failed to obtain time!");
  }
}

/* =====================
   === 4. Firebase ===
   ===================== */
void ConnectToFirebase::connectFirebase() {
  Serial.println("🔥 Connecting to Firebase...");

  fbConfig.database_url = FIREBASE_HOST;
  fbConfig.signer.tokens.legacy_token = FIREBASE_AUTH;

  Firebase.begin(&fbConfig, &auth);
  Firebase.reconnectWiFi(true);

  Serial.println("✅ Firebase ready!");
}

/* =====================
   === 5. Kiểm tra ===
   ===================== */
bool ConnectToFirebase::isConnected() {
  return (WiFi.status() == WL_CONNECTED && Firebase.ready());
}

/* =====================
   === 6. Helper ===
   ===================== */
String ConnectToFirebase::buildPath(const String &key) const {
  if (FIREBASE_PATH_ROOT == "/") {
    String result = "/";
    result.concat(key);
    return result;
  }

  String path = FIREBASE_PATH_ROOT;
  if (!path.endsWith("/")) path.concat("/");
  path.concat(key);
  return path;
}

/* =====================
   === 7. Gửi dữ liệu ===
   ===================== */
bool ConnectToFirebase::sendData(float bpm, float spo2) {
  if (millis() - lastSend < FIREBASE_REPORT_INTERVAL_MS)
    return false;
  lastSend = millis();

  if (!isConnected()) {
    Serial.println("⚠️ Firebase not connected, skip send...");
    return false;
  }

  // Format time
  char timeStr[25];
  struct tm timeinfo;
  if (getLocalTime(&timeinfo))
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
  else
    strcpy(timeStr, "unknown_time");

  // Normalize data
  int bpmInt = max(0, (int)round(bpm));
  float spo2Rounded = ((int)(spo2 * 10 + 0.5)) / 10.0f;

  bool success = true;

  // Upload BPM
  String pathBPM = buildPath("BPM");
  if (Firebase.RTDB.setInt(&fbdo, pathBPM.c_str(), bpmInt)) {
    Serial.printf("✅ [Firebase] BPM → %s = %d\n", pathBPM.c_str(), bpmInt);
  } else {
    Serial.printf("❌ BPM upload failed: %s\n", fbdo.errorReason().c_str());
    success = false;
  }

  // Upload SpO2
  String pathSpO2 = buildPath("SpO2");
  if (Firebase.RTDB.setFloat(&fbdo, pathSpO2.c_str(), spo2Rounded)) {
    Serial.printf("✅ [Firebase] SpO₂ → %s = %.1f\n", pathSpO2.c_str(), spo2Rounded);
  } else {
    Serial.printf("❌ SpO₂ upload failed: %s\n", fbdo.errorReason().c_str());
    success = false;
  }

  Serial.printf("[%s] ❤️ HR: %d | 🩸 SpO₂: %.1f%%\n\n",
                timeStr, bpmInt, spo2Rounded);
  return success;
}

/* =====================
   === 8. Đọc dữ liệu ===
   ===================== */
bool ConnectToFirebase::readData(int &bpmOut, float &spo2Out) {
  if (millis() - lastReadMs < FIREBASE_REPORT_INTERVAL_MS)
    return false;
  lastReadMs = millis();

  if (!isConnected()) {
    Serial.println("⚠️ Firebase not connected, skip read...");
    return false;
  }

  bool ok = true;

  // Read BPM
  String pathBPM = buildPath("BPM");
  if (Firebase.RTDB.getInt(&fbdo, pathBPM.c_str())) {
    bpmOut = fbdo.intData();
  } else {
    ok = false;
  }

  // Read SpO₂
  String pathSpO2 = buildPath("SpO2");
  if (Firebase.RTDB.getFloat(&fbdo, pathSpO2.c_str())) {
    spo2Out = fbdo.floatData();
  } else {
    ok = false;
  }

  return ok;
}
