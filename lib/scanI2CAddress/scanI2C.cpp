#include "scanI2C.h"

// Global variable to store task handle
static TaskHandle_t scanI2CTaskHandle = NULL;

void setupI2C(uint8_t sdaPin, uint8_t sclPin) {
    Wire.begin(sdaPin, sclPin);
    Serial.println("[I2C] Setup completed...");
}

void scanI2C() {
    uint8_t error, address;
    int nDevices = 0;

    Serial.println("[ScanI2C] Scanning I2C bus...");

    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("[ScanI2C] I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
            nDevices++;
        } else if (error == 4) {
            Serial.print("[ScanI2C] Unknown error at address 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
        }
    }

    if (nDevices == 0) Serial.println("[ScanI2C] No I2C devices found");
    else {
        Serial.print("[ScanI2C] Found ");
        Serial.print(nDevices);
        Serial.println(" I2C device(s)");
    }
}

// FreeRTOS Task function for ScanI2C
void scanI2CTask(void *parameter) {
    Serial.print("[FreeRTOS] ScanI2C Task started on Core ");
    Serial.print(xPortGetCoreID());
    Serial.print(" - Priority: ");
    Serial.println(uxTaskPriorityGet(NULL));
    
    setupI2C(SDA_PIN, SCL_PIN);
    scanI2C();
    
    // Infinite task loop
    while (1) {
        // Non-blocking delay using FreeRTOS tick delay
        vTaskDelay(pdMS_TO_TICKS(SCANI2C_TASK_DELAY));
    }
    
    // Delete task if exit loop (normally doesn't happen)
    vTaskDelete(NULL);
}

// Function to create FreeRTOS task for ScanI2C
void createScanI2CTask() {
    Serial.println("[FreeRTOS] Creating ScanI2C Task...");
    
    xTaskCreatePinnedToCore(
        scanI2CTask,                    // Task function
        "ScanI2CTask",                  // Task name
        SCANI2C_TASK_STACK_SIZE,        // Stack size (bytes)
        NULL,                           // Task parameter
        SCANI2C_TASK_PRIORITY,          // Priority (0-24)
        &scanI2CTaskHandle,             // Task handle
        SCANI2C_CORE                    // Core ID (0 or 1)
    );
    
    if (scanI2CTaskHandle != NULL) {
        Serial.println("[FreeRTOS] ScanI2C Task created successfully");
    } else {
        Serial.println("[FreeRTOS] ERROR: Failed to create ScanI2C Task");
    }
}
