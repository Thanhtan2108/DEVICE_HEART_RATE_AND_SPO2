#include "scanI2C.h"

void I2C_Init(uint8_t sdaPin, uint8_t sclPin) {
    Wire.begin(sdaPin, sclPin);
    Serial.println("[I2C] Setup completed...");
}

void I2C_ScanAddress() {
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
