#ifndef MAX30102_SENSOR_H
#define MAX30102_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <MAX30105.h>
#include <math.h>
#include "config.h"

class Max30102Sensor {
public:
  Max30102Sensor();

  // Initialize with TwoWire instance and 7-bit address (default 0x57)
  bool begin(TwoWire &wireInst, uint8_t addr = MAX30102_ADDR);

  // Call frequently from loop()
  void update();

  // Accessors
  bool isFingerDetected() const { return _fingerDetected; }
  float getBPM() const { return _bpm; }
  float getSpO2() const { return _spo2; }

  // Power control
  void powerOn();
  void powerOff();
  void reset();

  // Tune parameters
  void setIRThreshold(uint32_t thr) { _irThreshold = thr; }
  void setSampleRate(int sr) { _sampleRate = sr; }

private:
  MAX30105 particle;
  TwoWire *_wire;
  uint8_t _addr;

  // Running state
  bool _fingerDetected;
  float _bpm;
  float _spo2;

  // Filtering / peak detection
  static const int SAMPLE_RATE = 100;      // expected sensor sample rate (Hz)
  static const int BUF_MAX = 600;         // circular buffer size (samples)
  uint32_t _irBuf[BUF_MAX];
  uint32_t _redBuf[BUF_MAX];
  int _bufIdx;
  unsigned long _sampleCount;             // monotonic sample counter

  // EMA for DC removal
  float _emaLong;     // slow EMA for DC
  float _emaShort;    // short EMA for smoothing

  // peak detection variables
  int _prev2;         // filtered[n-2]
  int _prev1;         // filtered[n-1]
  int _curr;          // filtered[n]
  uint32_t _lastPeakSample;   // sample index of last peak
  uint32_t _peakIntervals[8]; // circular intervals (in samples)
  int _peakIdx;
  int _peakCountStored;

  // thresholds and params
  uint32_t _irThreshold; // detection threshold (raw IR)
  int _minBeatIntervalSamples; // min interval between beats in samples (~0.3s)
  int _maxBeatIntervalSamples; // max interval (~2s)

  // helpers
  void pushSample(uint32_t ir, uint32_t red);
  int computeFiltered(int raw);
  bool detectPeak(int filteredValue);
  void recordPeak(uint32_t sampleIndex);
  float computeBPMFromIntervals() const;
  float computeSpO2Simple(); // simple ratio method
  int _sampleRate;
};

#endif // MAX30102_SENSOR_H
