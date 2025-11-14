#include "max30102.h"
#include "../common/pinConfig.h"
#include "../common/systemConfig.h"
#include "../common/sensorConfig.h"

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

#include "freertos/semphr.h"
#include <esp_task_wdt.h>

static MAX30105 particleSensor;

// RTOS objects
static TaskHandle_t s_taskHandle = nullptr;
static SemaphoreHandle_t s_intSemaphore = nullptr;

// Algorithm buffers (BUFFER_SIZE from spo2_algorithm.h)
static uint32_t ir_buffer[BUFFER_SIZE];
static uint32_t red_buffer[BUFFER_SIZE];
static volatile int buf_index = 0;

// Decimation accumulators
static uint32_t acc_ir = 0;
static uint32_t acc_red = 0;
static uint8_t acc_count = 0;

// Latest computed results
static volatile int32_t latest_spo2 = -999;
static volatile int32_t latest_hr = -999;
static volatile bool latest_spo2_valid = false;
static volatile bool latest_hr_valid = false;
static volatile bool serial_print_enabled = true;

// Forward
static void readTask(void *pvParameters);
static void IRAM_ATTR intISRHandler(void);

// Constructor
Max30102Sensor::Max30102Sensor() {}

// ISR: minimal - give semaphore
static void IRAM_ATTR intISRHandler(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (s_intSemaphore) {
    xSemaphoreGiveFromISR(s_intSemaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
  }
}

bool Max30102Sensor::begin() {
  // sanity check DECIMATE_FACTOR (compile-time ideally, runtime as backup)
  if (DECIMATE_FACTOR <= 0) {
    Serial.println("DECIMATE_FACTOR invalid. Check SENSOR_SAMPLE_RATE and ALGO_FREQ.");
    return false;
  }

  // Init I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);

  // Soft reset device to ensure known state (SparkFun lib provides softReset())
  particleSensor.softReset();
  delay(10);

  // Begin sensor (checks PART ID inside)
  if (!particleSensor.begin(Wire, I2C_CLOCK_HZ, MAX30105_ADDRESS)) {
    Serial.println("MAX30105/02 not found. Check wiring/power.");
    return false;
  }

  // Setup sensor using sensorConfig defaults
  byte powerLevel = SENSOR_LED_POWER;
  byte sampleAverage = (SENSOR_FIFO_SAMPLEAVG == 1) ? 1 : SENSOR_FIFO_SAMPLEAVG;
  byte ledMode = SENSOR_LED_MODE;
  int sampleRate = SENSOR_SAMPLE_RATE;
  int pulseWidth = SENSOR_PULSE_WIDTH;
  int adcRange = SENSOR_ADC_RANGE;

  particleSensor.setup(powerLevel, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  // FIFO configure
  particleSensor.clearFIFO();
  particleSensor.enableFIFORollover(); // optional: allows continuous operation without manual pointer management
  particleSensor.setFIFOAlmostFull(SENSOR_FIFO_A_FULL);

  // Enable interrupts: Almost Full and Data Ready
  particleSensor.enableAFULL();
  particleSensor.enableDATARDY();

  // Clear any pending interrupts by reading INT status registers
  // (SparkFun wrapper getINT1/getINT2 simply read INTSTAT1/2)
  (void) particleSensor.getINT1();
  (void) particleSensor.getINT2();

  // Create binary semaphore for INT handling
  s_intSemaphore = xSemaphoreCreateBinary();
  if (!s_intSemaphore) {
    Serial.println("Failed to create semaphore for INT - falling back to polling");
  }

  // Attach ISR to INT pin only if semaphore created and pin valid
  if (MAX30102_INT_PIN >= 0 && s_intSemaphore) {
    // Note: on ESP32, some pins (34..39) are input-only and may not provide reliable internal pull-ups.
    // If using such a pin, ensure the INT line has an external pull-up to VCC on your board.
    // Alternatively choose a GPIO with internal pull-up support.
    pinMode(MAX30102_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MAX30102_INT_PIN), intISRHandler, FALLING);
    Serial.printf("INT pin attached: %d\n", MAX30102_INT_PIN);
  } else if (MAX30102_INT_PIN >= 0) {
    Serial.printf("INT pin %d provided but semaphore not created: using polling\n", MAX30102_INT_PIN);
  } else {
    Serial.println("INT pin disabled (MAX30102_INT_PIN < 0) - using polling");
  }

  // Reset buffers and accumulators
  buf_index = 0;
  acc_ir = acc_red = 0;
  acc_count = 0;

  // Reset latest values
  latest_spo2 = -999;
  latest_hr = -999;
  latest_spo2_valid = false;
  latest_hr_valid = false;

  // All set
  return true;
}

// process buffers when full (call algorithm)
static void max30102_process_buffers_when_full() {
  int32_t spo2 = -999;
  int32_t hr = -999;
  int8_t spo2_valid = 0;
  int8_t hr_valid = 0;

  maxim_heart_rate_and_oxygen_saturation(ir_buffer, BUFFER_SIZE, red_buffer, &spo2, &spo2_valid, &hr, &hr_valid);

  latest_spo2 = spo2;
  latest_hr = hr;
  latest_spo2_valid = (spo2_valid != 0);
  latest_hr_valid = (hr_valid != 0);

  if (serial_print_enabled) {
    Serial.print(">> SPO2: ");
    if (latest_spo2_valid) Serial.print(latest_spo2); else Serial.print("Invalid");
    Serial.print("  HR: ");
    if (latest_hr_valid) Serial.println(latest_hr); else Serial.println("Invalid");
  }
}

// readTask: wait on semaphore (INT) or poll; read FIFO burst, decimate, fill algorithm buffer
static void readTask(void *pvParameters) {
  (void) pvParameters;

  // register task to WDT
  esp_task_wdt_add(NULL);

  const TickType_t waitTicks = pdMS_TO_TICKS(1000); // how long to wait for INT sem
  const TickType_t shortDelay = pdMS_TO_TICKS(5);

  for (;;) {
    // Reset WDT periodically
    esp_task_wdt_reset();

    bool triggered = false;
    if (s_intSemaphore) {
      // Wait for interrupt or timeout
      if (xSemaphoreTake(s_intSemaphore, waitTicks) == pdTRUE) {
        triggered = true;
      }
    } else {
      // no semaphore => polling path; small delay
      vTaskDelay(shortDelay);
    }

    // Either triggered by INT, or timed out (fallback polling). Call check() to get new data
    uint16_t n = particleSensor.check(); // this reads FIFO_DATA into library internal buffer
    if (n == 0) {
      // nothing new
      if (!triggered) {
        // continue polling
        vTaskDelay(shortDelay);
        continue;
      }
    }

    // Read whatever values available in library internal ring buffer
    while (particleSensor.available()) {
      uint32_t r = particleSensor.getFIFORed();
      uint32_t i = particleSensor.getFIFOIR();

      // accumulate for decimation: average DECIMATE_FACTOR samples
      acc_red += r;
      acc_ir += i;
      acc_count++;

      // advance library tail
      particleSensor.nextSample();

      if (acc_count >= DECIMATE_FACTOR) {
        uint32_t dec_red = acc_red / acc_count;
        uint32_t dec_ir  = acc_ir / acc_count;

        int idx = buf_index;
        red_buffer[idx] = dec_red;
        ir_buffer[idx]  = dec_ir;
        buf_index++;

        // reset accumulators
        acc_count = 0;
        acc_red = 0;
        acc_ir = 0;

        if (buf_index >= BUFFER_SIZE) {
          max30102_process_buffers_when_full();
          buf_index = 0;
        }
      }
    }

    // small yield
    vTaskDelay(shortDelay);
  }

  // never reached
  // esp_task_wdt_delete(NULL);
  // vTaskDelete(NULL);
}

bool Max30102Sensor::start() {
  if (s_taskHandle) return true;

  BaseType_t ret = xTaskCreatePinnedToCore(
    readTask,
    "max30102_read",
    MAX30102_TASK_STACK_SIZE,
    nullptr,
    MAX30102_TASK_PRIORITY,
    &s_taskHandle,
    MAX30102_TASK_CORE
  );

  return (ret == pdPASS);
}

void Max30102Sensor::stop() {
  if (s_taskHandle) {
    // remove WDT registration for this task if called from same task; otherwise leave as-is
    // vTaskDelete must be called from different context, so delete here:
    vTaskDelete(s_taskHandle);
    s_taskHandle = nullptr;
  }

  // detach INT if attached
  if (MAX30102_INT_PIN >= 0) {
    detachInterrupt(digitalPinToInterrupt(MAX30102_INT_PIN));
  }
  if (s_intSemaphore) {
    vSemaphoreDelete(s_intSemaphore);
    s_intSemaphore = nullptr;
  }
}

bool Max30102Sensor::getLatest(int32_t &spo2, bool &spo2_valid, int32_t &hr, bool &hr_valid) {
  spo2 = latest_spo2;
  hr = latest_hr;
  spo2_valid = latest_spo2_valid;
  hr_valid = latest_hr_valid;
  return true;
}

void Max30102Sensor::enableSerialPrint(bool en) {
  serial_print_enabled = en;
}
