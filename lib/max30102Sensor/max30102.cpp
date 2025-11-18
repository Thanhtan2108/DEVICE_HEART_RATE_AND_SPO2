#include "max30102.h"

MAX30105 particleSensor;

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
static uint16_t irBuffer[BUFFER_SIZE];
static uint16_t redBuffer[BUFFER_SIZE];
static uint16_t workingIr[BUFFER_SIZE];
static uint16_t workingRed[BUFFER_SIZE];
#else
static uint32_t irBuffer[BUFFER_SIZE];
static uint32_t redBuffer[BUFFER_SIZE];
static uint32_t workingIr[BUFFER_SIZE];
static uint32_t workingRed[BUFFER_SIZE];
#endif

static const uint8_t SAMPLE_STEP = 18;
static uint16_t samplesStored = 0;
static uint16_t bufferHead = 0;
static uint8_t newSamplesSinceCalc = 0;
static bool bufferPrimed = false;
static bool fingerDetected = false;
static int8_t fingerConfidence = 0;

static int32_t algoHeartRate = 0;
static uint32_t algoHeartRateTs = 0;

static int32_t algoSpO2 = 0;
static uint32_t algoSpO2Ts = 0;

static int32_t quickSpO2 = 0;
static uint32_t quickSpO2Ts = 0;

static const uint32_t FINGER_ON_THRESHOLD = 60000;
static const uint32_t FINGER_OFF_THRESHOLD = 30000;
static const uint8_t FINGER_CONFIDENCE_MAX = 6;
static const uint32_t VALUE_HOLD_MS_FINGER = 6000;
static const uint32_t VALUE_HOLD_MS_NO_FINGER = 800;
static const uint32_t QUICK_SPO2_HOLD_MS = 1500;

static void storeSample(uint32_t ir, uint32_t red) {
    irBuffer[bufferHead] = ir;
    redBuffer[bufferHead] = red;
    bufferHead = (bufferHead + 1) % BUFFER_SIZE;

    if (samplesStored < BUFFER_SIZE) {
        samplesStored++;
        if (samplesStored == BUFFER_SIZE) {
            bufferPrimed = true;
            newSamplesSinceCalc = SAMPLE_STEP; // chạy thuật toán ngay khi đủ dữ liệu
        }
    } else if (bufferPrimed) {
        if (newSamplesSinceCalc < 255) {
            newSamplesSinceCalc++;
        }
    }
}

static void updateFingerState(long irValue) {
    if (irValue > (long)FINGER_ON_THRESHOLD) {
        if (fingerConfidence < FINGER_CONFIDENCE_MAX)
            fingerConfidence++;
    } else if (irValue < (long)FINGER_OFF_THRESHOLD) {
        if (fingerConfidence > -FINGER_CONFIDENCE_MAX)
            fingerConfidence--;
    }

    if (fingerConfidence >= FINGER_CONFIDENCE_MAX - 1) {
        fingerDetected = true;
    } else if (fingerConfidence <= -(FINGER_CONFIDENCE_MAX - 1)) {
        fingerDetected = false;
    }
}

static void runAlgorithmIfNeeded() {
    if (!bufferPrimed) return;
    if (newSamplesSinceCalc < SAMPLE_STEP) return;

    newSamplesSinceCalc = 0;

    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
        uint16_t idx = (bufferHead + i) % BUFFER_SIZE;
        workingIr[i] = irBuffer[idx];
        workingRed[i] = redBuffer[idx];
    }

    int32_t tempSpo2 = 0;
    int8_t tempSpo2Valid = 0;
    int32_t tempHeartRate = 0;
    int8_t tempHeartRateValid = 0;

    maxim_heart_rate_and_oxygen_saturation(workingIr, BUFFER_SIZE, workingRed, &tempSpo2, &tempSpo2Valid, &tempHeartRate, &tempHeartRateValid);

    uint32_t now = millis();

    if (tempHeartRateValid && tempHeartRate >= 30 && tempHeartRate <= 220) {
        algoHeartRate = tempHeartRate;
        algoHeartRateTs = now;
    }

    if (tempSpo2Valid && tempSpo2 >= 70 && tempSpo2 <= 100) {
        algoSpO2 = tempSpo2;
        algoSpO2Ts = now;
    }
}

bool max30102_init() {
    Serial.println("Initializing MAX30102...");
    
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30102 not found! Check wiring.");
        return false;
    }
    
    Serial.println("MAX30102 found! Configuring...");
    
    byte ledBrightness = 0x3F;
    byte sampleAverage = 4;
    byte ledMode = 2;
    int sampleRate = 100;
    int pulseWidth = 411;
    int adcRange = 4096;
    
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    particleSensor.enableDIETEMPRDY();
    particleSensor.disableAFULL();
    particleSensor.disableDATARDY();
    
    samplesStored = 0;
    bufferHead = 0;
    newSamplesSinceCalc = 0;
    bufferPrimed = false;
    algoHeartRate = 0;
    algoHeartRateTs = 0;
    algoSpO2 = 0;
    algoSpO2Ts = 0;
    quickSpO2 = 0;
    quickSpO2Ts = 0;
    fingerDetected = false;
    fingerConfidence = 0;
    
    Serial.println("MAX30102 configured successfully!");
    return true;
}

static void updateQuickSpO2(long ir, long red) {
    if (!fingerDetected) return;
    if (ir <= 0 || red <= 0) return;
    float ratio = (float)red / (float)ir;
    int estimate = 105 - (int)(ratio * 35.0f);
    if (estimate > 99) estimate = 99;
    if (estimate < 70) estimate = 0;
    if (estimate == 0) return;
    quickSpO2 = estimate;
    quickSpO2Ts = millis();
}

bool max30102_read_data(long *ir_value, long *red_value) {
    bool updated = false;

    particleSensor.check();

    while (particleSensor.available()) {
        long red = particleSensor.getRed();
        long ir = particleSensor.getIR();
        particleSensor.nextSample();

        *red_value = red;
        *ir_value = ir;
        updateFingerState(ir);
        updateQuickSpO2(ir, red);
        storeSample((uint32_t)ir, (uint32_t)red);
        updated = true;
    }

    if (updated) {
        runAlgorithmIfNeeded();
    }

    return updated;
}

static bool valueIsFresh(uint32_t timestamp, uint32_t limit) {
    if (timestamp == 0) return false;
    uint32_t now = millis();
    return (now - timestamp) <= limit;
}

bool max30102_has_finger(long ir_value) {
    (void)ir_value;
    return fingerDetected;
}

int max30102_get_heart_rate(long /*irValue*/) {
    uint32_t limit = fingerDetected ? VALUE_HOLD_MS_FINGER : VALUE_HOLD_MS_NO_FINGER;
    return valueIsFresh(algoHeartRateTs, limit) ? (int)algoHeartRate : 0;
}

int max30102_get_spo2(long /*ir_value*/, long /*red_value*/) {
    uint32_t limit = fingerDetected ? VALUE_HOLD_MS_FINGER : VALUE_HOLD_MS_NO_FINGER;
    if (valueIsFresh(algoSpO2Ts, limit)) {
        return (int)algoSpO2;
    }
    if (fingerDetected && valueIsFresh(quickSpO2Ts, QUICK_SPO2_HOLD_MS)) {
        return (int)quickSpO2;
    }
    return 0;
}

bool max30102_is_heart_rate_valid() {
    uint32_t limit = fingerDetected ? VALUE_HOLD_MS_FINGER : VALUE_HOLD_MS_NO_FINGER;
    return valueIsFresh(algoHeartRateTs, limit);
}

bool max30102_is_spo2_valid() {
    uint32_t limit = fingerDetected ? VALUE_HOLD_MS_FINGER : VALUE_HOLD_MS_NO_FINGER;
    if (valueIsFresh(algoSpO2Ts, limit)) return true;
    if (fingerDetected && valueIsFresh(quickSpO2Ts, QUICK_SPO2_HOLD_MS)) return true;
    return false;
}
