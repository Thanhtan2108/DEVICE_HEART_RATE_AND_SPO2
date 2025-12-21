#ifndef MAX30102_SENSOR_H
#define MAX30102_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <MAX30105.h>
#include <math.h>
#include "config.h"

/*
 * Max30102Sensor
 * ----------------
 * - Reads raw PPG (IR + RED) from MAX30102
 * - Computes Heart Rate (BPM) using peak detection
 * - Estimates SpO2 using simplified AC/DC ratio method
 *
 * WARNING:
 * - SpO2 calculation is NOT medically calibrated
 * - Intended for learning, research, and AI training only
 * - DO NOT use for medical diagnosis
 */

class Max30102Sensor {
public:
  Max30102Sensor();

  // Initialize sensor (Wire.begin must be called before)
  bool begin(TwoWire &wireInst, uint8_t addr = MAX30102_ADDR);

  // Call frequently from loop()
  void update();

  // Status / outputs
  bool  isFingerDetected() const { return _fingerDetected; }
  float getBPM() const { return _bpm; }
  float getSpO2() const { return _spo2; }

  // Raw PPG access (for AI logging)
  uint32_t getLastIR()  const { return _irBuf[(_bufIdx - 1 + BUF_MAX) % BUF_MAX]; }
  uint32_t getLastRed() const { return _redBuf[(_bufIdx - 1 + BUF_MAX) % BUF_MAX]; }

  // Power control
  void powerOn();
  void powerOff();
  void reset();

  // Tuning
  void setIRThreshold(uint32_t thr) { _irThreshold = thr; }
  void setSampleRate(int sr);

private:
  // Hardware
  MAX30105 particle;
  TwoWire *_wire;
  uint8_t _addr;

  // State
  bool  _fingerDetected;
  float _bpm;
  float _spo2;

  // Sampling
  int _sampleRate; // Hz

  // Circular buffers (raw PPG)
  static const int BUF_MAX = 600; // ~6 seconds @ 100Hz
  uint32_t _irBuf[BUF_MAX];
  uint32_t _redBuf[BUF_MAX];
  int _bufIdx;
  uint32_t _sampleCount;

  // EMA filtering
  float _emaLong;   // slow DC baseline
  float _emaShort;  // fast AC smoothing

  // Peak detection
  int _prev2, _prev1, _curr;
  uint32_t _lastPeakSample;
  uint32_t _peakIntervals[8];
  int _peakIdx;
  int _peakCountStored;

  // Parameters
  uint32_t _irThreshold;
  int _minBeatIntervalSamples;
  int _maxBeatIntervalSamples;

  // Internal helpers
  void pushSample(uint32_t ir, uint32_t red);
  int  computeFiltered(int raw);
  bool detectPeak(int filteredValue);
  void recordPeak(uint32_t sampleIndex);
  float computeBPMFromIntervals() const;

  // SpO2 estimation (simplified, non-calibrated)
  float computeSpO2Simple();
};

#endif // MAX30102_SENSOR_H
