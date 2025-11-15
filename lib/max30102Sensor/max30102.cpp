#include "max30102.h"

#include "pinConfig.h"
#include "systemConfig.h"
#include "sensorConfig.h"
#include "i2c_mutex.h"

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <esp_task_wdt.h>

// Debug macro: set 1 to enable Serial prints inside this file (not recommended in production)
#ifndef MAX30102_DEBUG
#define MAX30102_DEBUG 0
#endif

#if MAX30102_DEBUG
  #define LOG(x)   Serial.print(x)
  #define LOGLN(x) Serial.println(x)
#else
  #define LOG(x)
  #define LOGLN(x)
#endif

static MAX30105 particleSensor;

// FreeRTOS task handle for read task
static TaskHandle_t s_taskHandle = nullptr;

// Running flag for controlled stop
static volatile bool s_taskRunning = false;

// Algorithm buffers (BUFFER_SIZE comes from spo2_algorithm.h)
static uint32_t ir_buffer[BUFFER_SIZE];
static uint32_t red_buffer[BUFFER_SIZE];
static volatile int buf_index = 0;

// Decimation accumulators
static uint32_t acc_ir = 0;
static uint32_t acc_red = 0;
static uint8_t acc_count = 0;

// Latest computed results (volatile for safe single-word reads)
static volatile int32_t latest_spo2 = -999;
static volatile int32_t latest_hr = -999;
static volatile bool latest_spo2_valid = false;
static volatile bool latest_hr_valid = false;

// Forward declarations
static void readTask(void *pvParameters);
static void IRAM_ATTR intISRHandler(void);

// Constructor
Max30102Sensor::Max30102Sensor() {}

// ISR (minimal): notify the read task via task notification
static void IRAM_ATTR intISRHandler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (s_taskHandle != nullptr) {
    vTaskNotifyGiveFromISR(s_taskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
  }
}

// begin(): configure I2C and sensor (but DO NOT attach ISR or create task here)
bool Max30102Sensor::begin()
{
  // Basic sanity
  if (DECIMATE_FACTOR <= 0) {
    LOGLN("DECIMATE_FACTOR invalid.");
    return false;
  }

  // Initialize I2C (take mutex if available)
  if (!I2C_MUTEX_TAKE(pdMS_TO_TICKS(100))) {
    LOGLN("I2C mutex take failed in begin");
    return false;
  }
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  I2C_MUTEX_GIVE();

  // Soft reset (protect the I2C ops with mutex)
  if (!I2C_MUTEX_TAKE(pdMS_TO_TICKS(100))) return false;
  particleSensor.softReset();
  I2C_MUTEX_GIVE();
  vTaskDelay(pdMS_TO_TICKS(10));

  // Begin sensor (checks PART ID)
  if (!I2C_MUTEX_TAKE(pdMS_TO_TICKS(100))) return false;
  bool ok = particleSensor.begin(Wire, I2C_CLOCK_HZ, MAX30105_ADDRESS);
  I2C_MUTEX_GIVE();
  if (!ok) {
    LOGLN("MAX30105/02 not found.");
    return false;
  }

  // Setup config sensor with values from sensorConfig.h
  byte powerLevel = SENSOR_LED_POWER;
  byte sampleAverage = (SENSOR_FIFO_SAMPLEAVG == 1) ? 1 : SENSOR_FIFO_SAMPLEAVG;
  byte ledMode = SENSOR_LED_MODE;
  int sampleRate = SENSOR_SAMPLE_RATE;
  int pulseWidth = SENSOR_PULSE_WIDTH;
  int adcRange = SENSOR_ADC_RANGE;

  if (!I2C_MUTEX_TAKE(pdMS_TO_TICKS(100))) return false;
  particleSensor.setup(powerLevel, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  // FIFO configuration
  particleSensor.clearFIFO();
  particleSensor.enableFIFORollover(); // optional
  particleSensor.setFIFOAlmostFull(SENSOR_FIFO_A_FULL);

  // Enable interrupts of interest (device-level)
  particleSensor.enableAFULL();
  particleSensor.enableDATARDY();

  // Clear pending interrupt state
  (void)particleSensor.getINT1();
  (void)particleSensor.getINT2();
  I2C_MUTEX_GIVE();

  // Reset buffers and state
  buf_index = 0;
  acc_ir = acc_red = 0;
  acc_count = 0;

  latest_spo2 = -999;
  latest_hr = -999;
  latest_spo2_valid = false;
  latest_hr_valid = false;

  return true;
}

// Called from main to start the read task and attach ISR afterwards.
// We attach ISR only after task is created so notifications have a target.
bool Max30102Sensor::start()
{
  if (s_taskHandle) return true; // already running

  s_taskRunning = true;

  BaseType_t ret = xTaskCreatePinnedToCore(
    readTask,
    "max30102_read",
    MAX30102_TASK_STACK_SIZE,
    nullptr,
    MAX30102_TASK_PRIORITY,
    &s_taskHandle,
    MAX30102_TASK_CORE
  );

  if (ret != pdPASS) {
    s_taskHandle = nullptr;
    s_taskRunning = false;
    LOGLN("Failed to create max30102 read task");
    return false;
  }

  // Give task a moment to start up
  vTaskDelay(pdMS_TO_TICKS(20));

  // Attach ISR to INT pin if pin >= 0
  if (MAX30102_INT_PIN >= 0) {
    // Note: some ESP32 pins (34..39) are input-only and may not have reliable internal pull-ups.
    // If using those, ensure external pull-up resistor on the INT line.
    pinMode(MAX30102_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MAX30102_INT_PIN), intISRHandler, FALLING);
  } 
  return true;
}

// Ask the read task to stop, wait for it to exit cleanly
void Max30102Sensor::stop()
{
  if (!s_taskHandle) return;

  // Request stop
  s_taskRunning = false;

  // Wake the task (in case it is waiting on notify)
  xTaskNotifyGive(s_taskHandle);

  // Clear any pending interrupts for safety
  if (MAX30102_INT_PIN >= 0) {
    detachInterrupt(digitalPinToInterrupt(MAX30102_INT_PIN));
  }

  // give task a moment to cleanup
  vTaskDelay(pdMS_TO_TICKS(50));
  // if still not stopped, force delete
  if (s_taskHandle) {
    vTaskDelete(s_taskHandle);
    s_taskHandle = nullptr;
  }
}

// Process full buffers through Maxim algorithm and store results
static void max30102_process_buffers_when_full()
{
  int32_t spo2 = -999;
  int32_t hr = -999;
  int8_t spo2_valid = 0;
  int8_t hr_valid = 0;

  maxim_heart_rate_and_oxygen_saturation(ir_buffer, BUFFER_SIZE, red_buffer, &spo2, &spo2_valid, &hr, &hr_valid);

  latest_spo2 = spo2;
  latest_hr = hr;
  latest_spo2_valid = (spo2_valid != 0);
  latest_hr_valid = (hr_valid != 0);

  // library no longer prints — host application will read via getLatest()
  LOG("Processed buffer -> ");
  LOG("SPO2: "); LOG(spo2); LOG(" HR: "); LOG(hr); LOGLN("");
}

// readTask: main worker — waits for notification (from ISR) or times out and polls
// readTask: worker loop (uses I2C mutex)
static void readTask(void *pvParameters)
{
  (void)pvParameters;

  // Register this task with WDT
  esp_task_wdt_add(NULL);

  const TickType_t notifyTimeout = pdMS_TO_TICKS(1000); // wait up to 1s for ISR notification
  const TickType_t shortDelay = pdMS_TO_TICKS(5);

  // Initial small delay
  vTaskDelay(pdMS_TO_TICKS(10));

  while (s_taskRunning) {
    // Feed WDT
    esp_task_wdt_reset();

    // wait for notification (from ISR) or timeout
    ulTaskNotifyTake(pdTRUE, notifyTimeout); // ignore return value; we'll call check() anyway

    // Protect I2C sequence
    if (!I2C_MUTEX_TAKE(pdMS_TO_TICKS(200))) {
      // cannot acquire mutex; yield
      vTaskDelay(shortDelay);
      continue;
    }

    // call check() to move FIFO data into library buffers
    uint16_t n = particleSensor.check();
    I2C_MUTEX_GIVE();

    if (n == 0) {
      // nothing new, small sleep
      vTaskDelay(shortDelay);
      continue;
    }

    // Read whatever new samples the library exposes
    while (particleSensor.available()) {
      // Acquire I2C to get samples
      if (!I2C_MUTEX_TAKE(pdMS_TO_TICKS(200))) break;
      uint32_t r = particleSensor.getFIFORed();
      uint32_t i = particleSensor.getFIFOIR();
      particleSensor.nextSample();
      I2C_MUTEX_GIVE();

      // Accumulate for decimation
      acc_red += r;
      acc_ir += i;
      acc_count++;

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

    // small yield to allow other tasks to run
    vTaskDelay(shortDelay);
  } // end while s_taskRunning

  // cleanup: remove from WDT and delete self
  esp_task_wdt_delete(NULL);
  s_taskHandle = nullptr;
  vTaskDelete(NULL);
}

// getLatest: atomic-ish copy
bool Max30102Sensor::getLatest(int32_t &spo2, bool &spo2_valid, int32_t &hr, bool &hr_valid)
{
  spo2 = latest_spo2;
  hr = latest_hr;
  spo2_valid = latest_spo2_valid;
  hr_valid = latest_hr_valid;
  return true;
}

// enableSerialPrint: no-op in library (kept for API compatibility)
void Max30102Sensor::enableSerialPrint(bool en)
{
  (void)en;
  // NO-OP: printing should be done by application print/display task
}
