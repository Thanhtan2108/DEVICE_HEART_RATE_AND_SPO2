#include "Max30102Sensor.h"

Max30102Sensor::Max30102Sensor() {
  _wire = nullptr;
  _addr = 0x57;

  _fingerDetected = false;
  _bpm = 0.0f;
  _spo2 = 0.0f;

  _bufIdx = 0;
  _sampleCount = 0;

  _emaLong = 0.0f;
  _emaShort = 0.0f;

  _prev2 = _prev1 = _curr = 0;
  _lastPeakSample = 0;
  _peakIdx = 0;
  _peakCountStored = 0;

  _sampleRate = 100;   // default Hz
  _irThreshold = 8000; // coarse finger detection

  // Physiologically valid heart rate range: ~30–200 BPM
  _minBeatIntervalSamples = (int)(0.30f * _sampleRate);
  _maxBeatIntervalSamples = (int)(2.00f * _sampleRate);
}

bool Max30102Sensor::begin(TwoWire &wireInst, uint8_t addr) {
  _wire = &wireInst;
  _addr = addr;

  if (!particle.begin(*_wire, _addr)) {
    Serial.println("❌ MAX30102 not found");
    return false;
  }

  Serial.println("✅ MAX30102 detected");

  // Recommended configuration for PPG + SpO2
  byte ledBrightness = 0x3F;     // ~12–15 mA
  byte sampleAverage = 4;
  byte ledMode = 2;              // RED + IR
  int sampleRate = _sampleRate;  // Hz
  int pulseWidth = 411;          // 18-bit resolution
  int adcRange = 4096;

  particle.setup(ledBrightness, sampleAverage, ledMode,
                 sampleRate, pulseWidth, adcRange);

  particle.enableDIETEMPRDY();
  particle.enableFIFORollover();

  _wire->setClock(100000); // safer for ESP32
  delay(100);

  long initIR = particle.getIR();
  if (initIR > 0) {
    _emaLong = initIR;
    _emaShort = initIR;
  }

  return true;
}

void Max30102Sensor::powerOn() {
  particle.wakeUp();
}

void Max30102Sensor::powerOff() {
  particle.shutDown();
}

void Max30102Sensor::reset() {
  _fingerDetected = false;
  _bpm = 0.0f;
  _spo2 = 0.0f;

  _bufIdx = 0;
  _sampleCount = 0;

  _emaLong = 0.0f;
  _emaShort = 0.0f;

  _prev2 = _prev1 = _curr = 0;
  _lastPeakSample = 0;
  _peakIdx = 0;
  _peakCountStored = 0;
}

void Max30102Sensor::setSampleRate(int sr) {
  if (sr < 25) sr = 25;
  if (sr > 400) sr = 400;

  _sampleRate = sr;
  _minBeatIntervalSamples = (int)(0.30f * _sampleRate);
  _maxBeatIntervalSamples = (int)(2.00f * _sampleRate);
}

void Max30102Sensor::pushSample(uint32_t ir, uint32_t red) {
  _irBuf[_bufIdx] = ir;
  _redBuf[_bufIdx] = red;
  _bufIdx = (_bufIdx + 1) % BUF_MAX;
  _sampleCount++;
}

int Max30102Sensor::computeFiltered(int raw) {
  float alphaLong  = 1.0f / (_sampleRate * 0.8f); // DC removal
  float alphaShort = 0.25f;                       // AC smoothing

  if (_emaLong == 0.0f)  _emaLong  = raw;
  if (_emaShort == 0.0f) _emaShort = raw;

  _emaLong  = (1.0f - alphaLong)  * _emaLong  + alphaLong  * raw;
  _emaShort = (1.0f - alphaShort) * _emaShort + alphaShort * raw;

  return (int)(_emaShort - _emaLong);
}

bool Max30102Sensor::detectPeak(int filteredValue) {
  _prev2 = _prev1;
  _prev1 = _curr;
  _curr  = filteredValue;

  if (_prev1 > _prev2 && _prev1 > _curr) {
    int amplitude = abs(_prev1);
    int threshold = max(100, amplitude / 3);

    if (_prev1 > threshold) {
      uint32_t dt = _sampleCount - _lastPeakSample;
      if (dt > (uint32_t)_minBeatIntervalSamples) {
        return true;
      }
    }
  }
  return false;
}

void Max30102Sensor::recordPeak(uint32_t sampleIndex) {
  if (_lastPeakSample > 0) {
    uint32_t interval = sampleIndex - _lastPeakSample;

    if (interval >= (uint32_t)_minBeatIntervalSamples &&
        interval <= (uint32_t)_maxBeatIntervalSamples) {

      _peakIntervals[_peakIdx] = interval;
      _peakIdx = (_peakIdx + 1) % 8;
      if (_peakCountStored < 8) _peakCountStored++;
    }
  }
  _lastPeakSample = sampleIndex;
}

float Max30102Sensor::computeBPMFromIntervals() const {
  if (_peakCountStored < 2) return 0.0f;

  uint32_t sum = 0;
  for (int i = 0; i < _peakCountStored; i++) {
    sum += _peakIntervals[i];
  }

  float avgSamples = (float)sum / _peakCountStored;
  if (avgSamples <= 0.0f) return 0.0f;

  return 60.0f * _sampleRate / avgSamples;
}

float Max30102Sensor::computeSpO2Simple() {
  int n = 150;
  if (_sampleCount < (uint32_t)n) n = _sampleCount;
  if (n < 30) return _spo2;

  int idx = (_bufIdx - 1 + BUF_MAX) % BUF_MAX;

  double meanIr = 0.0, meanRed = 0.0;
  for (int i = 0; i < n; i++) {
    meanIr  += _irBuf[(idx - i + BUF_MAX) % BUF_MAX];
    meanRed += _redBuf[(idx - i + BUF_MAX) % BUF_MAX];
  }
  meanIr /= n;
  meanRed /= n;

  double rmsIr = 0.0, rmsRed = 0.0;
  for (int i = 0; i < n; i++) {
    double vIr  = _irBuf[(idx - i + BUF_MAX) % BUF_MAX]  - meanIr;
    double vRed = _redBuf[(idx - i + BUF_MAX) % BUF_MAX] - meanRed;
    rmsIr  += vIr  * vIr;
    rmsRed += vRed * vRed;
  }
  rmsIr  = sqrt(rmsIr  / n);
  rmsRed = sqrt(rmsRed / n);

  if (rmsIr <= 1e-6 || meanIr <= 0 || meanRed <= 0) return _spo2;

  double R = (rmsRed / meanRed) / (rmsIr / meanIr);
  double spo2 = 104.0 - 17.0 * R;

  if (spo2 > 100.0) spo2 = 100.0;
  if (spo2 < 85.0)  spo2 = 85.0;

  return (float)spo2;
}

void Max30102Sensor::update() {
  while (particle.available()) {
    long ir  = particle.getIR();
    long red = particle.getRed();

    pushSample((uint32_t)ir, (uint32_t)red);

    int filtered = computeFiltered((int)ir);

    _fingerDetected = (ir > _irThreshold) && (abs(filtered) > 50);

    if (_fingerDetected && detectPeak(filtered)) {
      recordPeak(_sampleCount - 1);
      _bpm = computeBPMFromIntervals();
    }

    _spo2 = computeSpO2Simple();
  }
}
