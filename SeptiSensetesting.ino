// ============================================================
//  SeptiSense - ESP32 multi-sensor prototype
//  MLX90614 (temperature) + AD8232 (ECG) + MAX30102 (HR/SpO2)
//  EDUCATIONAL PROTOTYPE - NOT A CLINICALLY VALIDATED DEVICE
// ============================================================
//
//  Libraries (Arduino Library Manager):
//   - Adafruit MLX90614 Library (accept Adafruit BusIO dependency)
//   - SparkFun MAX3010x Pulse and Proximity Sensor Library
//  Built in with the ESP32 board package: WiFi, WebServer, Wire
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <math.h>
#include "MAX30105.h"          // SparkFun MAX3010x library (also drives MAX30102)
#include "heartRate.h"
#include "spo2_algorithm.h"   // keep this include LAST (it defines short macro names)

// ---- Data types (must be defined BEFORE any function so the Arduino
// ---- IDE's auto-generated prototypes can see them) ----
struct PulseReading {
  bool consistent, hrValid, spo2Valid, fingerPresent, saturated;
  float hr;
  int spo2;
  uint32_t irValue;
};

struct TelemetryRecord {
  uint32_t t_ms;
  bool mlxOK, tempOK, ledOn, maxOK, finger, hrOK, spo2OK, ecgOK, leadOff;
  float obj, amb, hr, ecgBpm;
  int spo2;
  uint8_t ecgQ;
};

struct EcgShared {
  int raw;
  bool leadOff, saturated, bpmValid;
  float bpm;
  uint8_t quality;
  int p2p, rrCount;
  uint32_t totalBeats, sinceBeatMs;
  float measuredHz;
  uint32_t maxJitterUs, missedSamples;
};

// 0 = readable Serial Monitor, 1 = numeric ECG only (Serial Plotter)
#define SERIAL_MODE 0


// ---------------- WiFi ----------------
const char* ssid     = "ESP32_LED_Control";
const char* password = "ledcontrol123";

// ---------------- Telemetry to the laptop backend ----------------
// The ESP32 keeps its own Wi-Fi network (above) for the phone website AND
// joins a second network (2.4 GHz only) so it can reach the laptop.
// Leave STA_SSID empty ("") to disable sending.
//   Windows "Mobile hotspot" on the laptop: the laptop is always 192.168.137.1
const char* STA_SSID    = "";                       // <-- laptop hotspot / router name
const char* STA_PASS    = "";                       // <-- its password
const char* BACKEND_URL = "http://192.168.137.1:8000/api/telemetry";   // <-- laptop IP shown by server.py
const char* DEVICE_ID   = "septisense-01";
const unsigned long TELEMETRY_INTERVAL_MS = 5000;

// ---------------- Pins ----------------
const int LED_PIN      = 26;
const int SDA_PIN      = 21;
const int SCL_PIN      = 22;
const int ECG_PIN      = 34;
const int LO_PLUS_PIN  = 32;
const int LO_MINUS_PIN = 33;

// ---------------- Temperature ----------------
const float LED_ON_TEMP_C  = 25.0;
const float LED_OFF_TEMP_C = 24.0;
const unsigned long TEMP_INTERVAL = 1000;

WebServer server(80);
Adafruit_MLX90614 mlx;
MAX30105 particleSensor;   // SparkFun class; works with the MAX30102

bool  temperatureSensorOK  = false;
bool  temperatureReadingOK = false;
float objectTempC  = NAN;
float ambientTempC = NAN;
bool  ledState = false;
int   tempFailCount = 0;
unsigned long previousTempMillis = 0;

// ---------------- MAX30102 (SparkFun MAX3010x library) ----------------
bool maxSensorOK = false;
int  maxPingFails = 0;
unsigned long lastMaxCheckMs = 0;

// LED brightness 0-255 (0x3F is about 12 mA).
// If the IR value hits ~262143 (saturated) LOWER this (e.g. 0x1F).
// If the finger signal is weak (IR below ~30000) RAISE it (e.g. 0x7F).
#define MAX_LED_BRIGHTNESS 0x3F

// Finger detection uses the raw IR reflection value (real finger detection).
const uint32_t FINGER_ON_IR  = 30000;   // above this = finger present
const uint32_t FINGER_OFF_IR = 20000;   // below this = finger removed
const uint32_t IR_SATURATED  = 250000;  // near the 18-bit limit (262143)

const unsigned long PULSE_TIMEOUT = 5000;     // no beat for 5 s -> invalid
const int   MIN_CONSISTENT = 2;               // 2 consistent intervals = 3 beats
const float BEAT_TOLERANCE = 0.35;            // 35% interval tolerance
const uint32_t MIN_BEAT_INTERVAL = 300;
const uint32_t MAX_BEAT_INTERVAL = 2000;

uint32_t latestIR = 0, latestRed = 0;
bool fingerPresent = false;

// beat / heart-rate state
unsigned long lastBeatMillis = 0;
uint32_t previousBeatInterval = 0;
int consistentBeatIntervals = 0;
uint32_t ivBuf[5];
int ivCount = 0, ivHead = 0;

// SpO2 state (SparkFun/Maxim algorithm: 100 samples at 25 Hz = 4 s)
const int SPO2_BUF = 100;
uint32_t irBuffer[SPO2_BUF];
uint32_t redBuffer[SPO2_BUF];
int bufCount = 0, newSamples = 0;
uint32_t decimIR = 0, decimRed = 0; int decimN = 0;
int32_t spo2Raw = 0, algHr = 0;
int8_t  spo2OK = 0, algHrOK = 0;
int spo2Hist[4]; int spo2HistCount = 0, spo2HistHead = 0;
unsigned long lastSpo2GoodMs = 0;

void resetPulseState() {
  lastBeatMillis = 0; previousBeatInterval = 0; consistentBeatIntervals = 0;
  ivCount = 0; ivHead = 0;
  bufCount = 0; newSamples = 0; decimIR = 0; decimRed = 0; decimN = 0;
  spo2HistCount = 0; spo2HistHead = 0; lastSpo2GoodMs = 0;
}

// Called whenever the beat detector finds a pulse peak.
void onBeat(unsigned long now) {
  if (lastBeatMillis != 0) {
    uint32_t interval = now - lastBeatMillis;
    if (interval < MIN_BEAT_INTERVAL) return;       // double trigger: ignore
    if (interval <= MAX_BEAT_INTERVAL) {
      bool consistentNow = true;
      if (previousBeatInterval != 0) {
        uint32_t diff = (interval > previousBeatInterval)
                          ? interval - previousBeatInterval
                          : previousBeatInterval - interval;
        consistentNow = (diff <= previousBeatInterval * BEAT_TOLERANCE);
      }
      if (previousBeatInterval == 0) {
        consistentBeatIntervals = 1;
      } else if (consistentNow) {
        consistentBeatIntervals++;
      } else {
        consistentBeatIntervals = 1;                // restart the average
        ivCount = 0; ivHead = 0;
      }
      ivBuf[ivHead] = interval; ivHead = (ivHead + 1) % 5;
      if (ivCount < 5) ivCount++;
      previousBeatInterval = interval;
    } else {
      consistentBeatIntervals = 0; previousBeatInterval = 0;
      ivCount = 0; ivHead = 0;
    }
  }
  lastBeatMillis = now;
}

void runSpo2() {
  maxim_heart_rate_and_oxygen_saturation(irBuffer, SPO2_BUF, redBuffer,
                                         &spo2Raw, &spo2OK, &algHr, &algHrOK);
  if (spo2OK && spo2Raw >= 70 && spo2Raw <= 100) {
    spo2Hist[spo2HistHead] = (int)spo2Raw;
    spo2HistHead = (spo2HistHead + 1) % 4;
    if (spo2HistCount < 4) spo2HistCount++;
    lastSpo2GoodMs = millis();
  }
}

void pushSpo2Sample(uint32_t ir, uint32_t red) {
  if (bufCount < SPO2_BUF) {
    irBuffer[bufCount] = ir; redBuffer[bufCount] = red; bufCount++;
    if (bufCount == SPO2_BUF) runSpo2();
  } else {
    memmove(irBuffer, irBuffer + 1, (SPO2_BUF - 1) * sizeof(uint32_t));
    memmove(redBuffer, redBuffer + 1, (SPO2_BUF - 1) * sizeof(uint32_t));
    irBuffer[SPO2_BUF - 1] = ir; redBuffer[SPO2_BUF - 1] = red;
    if (++newSamples >= 25) { newSamples = 0; runSpo2(); }   // once per second
  }
}

void processPulseSample(uint32_t red, uint32_t ir) {
  unsigned long now = millis();
  latestIR = ir; latestRed = red;

  if (!fingerPresent) {
    if (ir >= FINGER_ON_IR) { fingerPresent = true; resetPulseState(); }
    else return;
  } else if (ir < FINGER_OFF_IR) {
    fingerPresent = false; resetPulseState();
    return;
  }

  if (checkForBeat((int32_t)ir)) onBeat(now);

  // average 4 samples (100 Hz -> 25 Hz) for the SpO2 algorithm
  decimIR += ir; decimRed += red;
  if (++decimN >= 4) {
    pushSpo2Sample(decimIR / 4, decimRed / 4);
    decimIR = 0; decimRed = 0; decimN = 0;
  }
}

// Read everything waiting in the sensor FIFO. Call this often.
void servicePulseSensor() {
  if (!maxSensorOK) return;
  static unsigned long lastPollUs = 0;
  unsigned long nowUs = micros();
  if (nowUs - lastPollUs < 4000) return;           // poll about every 4 ms
  lastPollUs = nowUs;

  particleSensor.check();
  while (particleSensor.available()) {
    uint32_t red = particleSensor.getFIFORed();
    uint32_t ir  = particleSensor.getFIFOIR();
    particleSensor.nextSample();
    processPulseSample(red, ir);
  }
}

PulseReading evaluatePulse() {
  PulseReading r;
  unsigned long now = millis();
  r.fingerPresent = maxSensorOK && fingerPresent;
  r.irValue = maxSensorOK ? latestIR : 0;
  r.saturated = maxSensorOK && fingerPresent && latestIR >= IR_SATURATED;

  bool recent = lastBeatMillis != 0 && (now - lastBeatMillis) < PULSE_TIMEOUT;
  r.consistent = r.fingerPresent && !r.saturated && recent &&
                 consistentBeatIntervals >= MIN_CONSISTENT && ivCount >= MIN_CONSISTENT;

  r.hr = NAN;
  if (ivCount > 0) {
    float s = 0;
    for (int i = 0; i < ivCount; i++) s += ivBuf[i];
    r.hr = 60000.0f / (s / ivCount);
  }
  r.hrValid = r.consistent && isfinite(r.hr) && r.hr >= 40.0 && r.hr <= 200.0;

  bool spo2Fresh = spo2HistCount > 0 && (now - lastSpo2GoodMs) < 8000;
  r.spo2 = 0;
  if (spo2Fresh) {
    int s = 0;
    for (int i = 0; i < spo2HistCount; i++) s += spo2Hist[i];
    r.spo2 = (s + spo2HistCount / 2) / spo2HistCount;
  }
  r.spo2Valid = r.fingerPresent && !r.saturated && spo2Fresh && bufCount >= SPO2_BUF;
  return r;
}

// ============================================================
//  ECG  (runs in its own task at 250 Hz)
// ============================================================
const uint32_t ECG_PERIOD_US = 4000;          // 250 Hz
const int DISPLAY_SIZE = 300;                 // 1.2 s on the website
const int MWI_LEN = 30;                       // 120 ms integration window
const int RR_MAX = 8;

const float MIN_PEAK        = 120.0;          // minimum detector energy to count as a beat
const int   MIN_P2P         = 60;             // below this (ADC counts) = flat signal
const uint32_t REFRACTORY_MS = 300;
const int   MIN_RR_FOR_BPM  = 4;              // 5 beats
const float MAX_RR_CV       = 0.25;           // RR variation above this = unreliable
const int   LEADOFF_CONFIRM = 25;             // 100 ms
const int   LEADON_CONFIRM  = 50;             // 200 ms

enum EcgQuality : uint8_t {
  Q_LEADS_OFF = 0, Q_SATURATED, Q_FLAT, Q_LEARNING,
  Q_NO_BEATS, Q_COLLECTING, Q_NOISY, Q_OK
};

const char* qualityText(uint8_t q) {
  switch (q) {
    case Q_LEADS_OFF:  return "LEADS OFF - ECG NOT VALID";
    case Q_SATURATED:  return "SIGNAL SATURATED (0/4095) - CHECK ELECTRODES";
    case Q_FLAT:       return "SIGNAL TOO WEAK / FLAT";
    case Q_LEARNING:   return "CALIBRATING DETECTOR (~3 s)";
    case Q_NO_BEATS:   return "NO RECENT HEARTBEATS DETECTED";
    case Q_COLLECTING: return "COLLECTING BEATS...";
    case Q_NOISY:      return "IRREGULAR OR NOISY - BPM WITHHELD";
    case Q_OK:         return "EXPERIMENTAL BPM ESTIMATE (NOT VALIDATED)";
  }
  return "UNKNOWN";
}

EcgShared ecgShared = {};
int16_t ecgDisplay[DISPLAY_SIZE];
int ecgDispIndex = 0;
int ecgDispCount = 0;
portMUX_TYPE ecgMux = portMUX_INITIALIZER_UNLOCKED;

void ecgTask(void* parameter) {
  // ---- filter state ----
  float baseline = 0; bool baselineReady = false; int settle = 0;
  float maBuf[5] = {0, 0, 0, 0, 0}; int maIdx = 0; float maSum = 0;
  float f1 = 0, f2 = 0;
  float sqBuf[MWI_LEN]; for (int i = 0; i < MWI_LEN; i++) sqBuf[i] = 0;
  int sqIdx = 0; float sqSum = 0; uint32_t recalcCount = 0;
  float mwi = 0, mwiPrev1 = 0, mwiPrev2 = 0;

  // ---- detector state ----
  bool learning = true; uint32_t learnStart = 0;
  float learnMax = 0, learnSum = 0; uint32_t learnN = 0;
  float spk = 0, npk = 0, thr = MIN_PEAK, lastPeakVal = 0;
  uint32_t lastPeakMs = 0, totalBeats = 0;
  uint32_t rrBuf[RR_MAX]; int rrCount = 0, rrHead = 0, rejectStreak = 0;

  // ---- 2 s statistics (4 bins of 0.5 s) ----
  float binMin[4], binMax[4]; int binSat[4]; int bin = 0, binSamples = 0;
  for (int i = 0; i < 4; i++) { binMin[i] = 1e9; binMax[i] = -1e9; binSat[i] = 0; }

  // ---- lead-off ----
  int offCount = 0, onCount = 0; bool leadOff = true; bool needReset = true;
  bool satFlag = false;

  // ---- timing ----
  uint32_t lastUs = micros();
  uint32_t jitterMax = 0, jitterPub = 0, missed = 0;
  uint32_t secStartMs = millis(), secCount = 0; float hz = 0;
  int statusCounter = 0;

  TickType_t period = pdMS_TO_TICKS(4);
  if (period < 1) period = 1;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, period);

    uint32_t nowUs = micros();
    uint32_t nowMs = millis();
    uint32_t dt = nowUs - lastUs; lastUs = nowUs;
    uint32_t jit = (dt > ECG_PERIOD_US) ? dt - ECG_PERIOD_US : ECG_PERIOD_US - dt;
    if (jit > jitterMax) jitterMax = jit;
    if (dt > ECG_PERIOD_US * 3 / 2) missed++;

    // ---- read ADC (average of 4 reads to reduce noise) ----
    int sum = 0;
    for (int i = 0; i < 4; i++) sum += analogRead(ECG_PIN);
    int raw = sum / 4;

#if SERIAL_MODE == 1
    Serial.println(raw);
#endif

    // ---- lead-off with debounce ----
    bool lo = (digitalRead(LO_PLUS_PIN) == HIGH) || (digitalRead(LO_MINUS_PIN) == HIGH);
    if (lo) {
      onCount = 0; if (offCount < 1000) offCount++;
      if (offCount >= LEADOFF_CONFIRM && !leadOff) { leadOff = true; needReset = true; }
    } else {
      offCount = 0; if (onCount < 1000) onCount++;
      if (onCount >= LEADON_CONFIRM && leadOff) { leadOff = false; needReset = true; }
    }

    // ---- reset detector on lead state change ----
    if (needReset) {
      needReset = false;
      baselineReady = false; settle = 0;
      for (int i = 0; i < 5; i++) maBuf[i] = 0;
      maSum = 0; maIdx = 0; f1 = f2 = 0;
      for (int i = 0; i < MWI_LEN; i++) sqBuf[i] = 0;
      sqSum = 0; sqIdx = 0; mwi = mwiPrev1 = mwiPrev2 = 0;
      learning = true; learnStart = 0; learnMax = 0; learnSum = 0; learnN = 0;
      spk = 0; npk = 0; thr = MIN_PEAK; lastPeakVal = 0; lastPeakMs = 0;
      rrCount = 0; rrHead = 0; rejectStreak = 0;
      for (int i = 0; i < 4; i++) { binMin[i] = 1e9; binMax[i] = -1e9; binSat[i] = 0; }
      bin = 0; binSamples = 0; satFlag = false;
    }

    // ---- filtering ----
    if (!baselineReady) { baseline = raw; baselineReady = true; }
    float alpha = (settle < 250) ? 0.05f : 0.01f;
    if (settle < 1000) settle++;
    baseline += alpha * (raw - baseline);
    float hp = raw - baseline;                       // baseline removed

    maSum += hp - maBuf[maIdx]; maBuf[maIdx] = hp; maIdx = (maIdx + 1) % 5;
    float filt = maSum / 5.0f;                       // 5-pt average (notches 50 Hz)

    float d = filt - f2; f2 = f1; f1 = filt;         // derivative
    float sq = d * d;
    sqSum += sq - sqBuf[sqIdx]; sqBuf[sqIdx] = sq; sqIdx = (sqIdx + 1) % MWI_LEN;
    if (++recalcCount >= 1000) {                     // cancel float drift
      recalcCount = 0; sqSum = 0;
      for (int i = 0; i < MWI_LEN; i++) sqSum += sqBuf[i];
    }
    if (sqSum < 0) sqSum = 0;
    mwiPrev2 = mwiPrev1; mwiPrev1 = mwi; mwi = sqSum / MWI_LEN;

    // ---- window statistics ----
    if (filt < binMin[bin]) binMin[bin] = filt;
    if (filt > binMax[bin]) binMax[bin] = filt;
    if (raw <= 10 || raw >= 4085) binSat[bin]++;
    if (++binSamples >= 125) {
      bin = (bin + 1) % 4; binSamples = 0;
      binMin[bin] = 1e9; binMax[bin] = -1e9; binSat[bin] = 0;
    }

    // ---- detector learning phase ----
    if (!leadOff && learning && settle >= 250) {
      if (learnStart == 0) learnStart = nowMs;
      if (mwi > learnMax) learnMax = mwi;
      learnSum += mwi; learnN++;
      if (nowMs - learnStart >= 2000 && learnN > 0) {
        spk = 0.25f * learnMax;
        npk = 0.5f * (learnSum / learnN);
        thr = npk + 0.25f * (spk - npk);
        if (thr < MIN_PEAK) thr = MIN_PEAK;
        learning = false;
      }
    }

    // ---- R-peak detection ----
    if (!leadOff && !learning && !satFlag) {
      if (mwiPrev1 > mwiPrev2 && mwiPrev1 >= mwi) {   // local maximum
        float pk = mwiPrev1;
        uint32_t peakMs = nowMs - 4;
        if (pk > thr && pk > MIN_PEAK) {
          uint32_t rr = (lastPeakMs != 0) ? (peakMs - lastPeakMs) : 0;
          bool accept = true;
          if (lastPeakMs != 0) {
            if (rr < REFRACTORY_MS) accept = false;
            else if (rr < 360 && pk < 0.5f * lastPeakVal) accept = false;   // T-wave
          }
          if (accept) {
            spk = 0.125f * pk + 0.875f * spk;
            lastPeakVal = pk;
            if (lastPeakMs != 0) {
              if (rr >= 300 && rr <= 2000) {
                bool store = true;
                if (rrCount >= 3) {                  // compare with median
                  uint32_t tmp[RR_MAX];
                  for (int i = 0; i < rrCount; i++) tmp[i] = rrBuf[i];
                  for (int i = 1; i < rrCount; i++) {
                    uint32_t v = tmp[i]; int j = i - 1;
                    while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
                    tmp[j + 1] = v;
                  }
                  float med = tmp[rrCount / 2];
                  float dev = (rr > med) ? rr - med : med - rr;
                  if (dev > 0.30f * med) {
                    store = false;
                    if (++rejectStreak >= 3) { rrCount = 0; rrHead = 0; rejectStreak = 0; store = true; }
                  }
                }
                if (store) {
                  rrBuf[rrHead] = rr; rrHead = (rrHead + 1) % RR_MAX;
                  if (rrCount < RR_MAX) rrCount++;
                  rejectStreak = 0;
                }
              } else {
                rrCount = 0; rrHead = 0;             // gap too long/short: restart
              }
            }
            lastPeakMs = peakMs; totalBeats++;
          } else {
            npk = 0.125f * pk + 0.875f * npk;
          }
        } else {
          npk = 0.125f * pk + 0.875f * npk;
        }
        thr = npk + 0.25f * (spk - npk);
        if (thr < MIN_PEAK) thr = MIN_PEAK;
      }

      // missed beats: slowly lower the signal level so the threshold can recover
      uint32_t since = (lastPeakMs != 0) ? (nowMs - lastPeakMs) : 0;
      if (lastPeakMs != 0 && since > 2500) {
        spk *= 0.998f;
        if (spk < MIN_PEAK * 2) spk = MIN_PEAK * 2;
        thr = npk + 0.25f * (spk - npk);
        if (thr < MIN_PEAK) thr = MIN_PEAK;
      }
    }

    // ---- per-sample publish (waveform + raw value) ----
    float dv = filt;
    if (dv > 2000) dv = 2000;
    if (dv < -2000) dv = -2000;
    portENTER_CRITICAL(&ecgMux);
    ecgDisplay[ecgDispIndex] = (int16_t)dv;
    ecgDispIndex = (ecgDispIndex + 1) % DISPLAY_SIZE;
    if (ecgDispCount < DISPLAY_SIZE) ecgDispCount++;
    ecgShared.raw = raw;
    ecgShared.leadOff = leadOff;
    portEXIT_CRITICAL(&ecgMux);

    // ---- timing statistics (once per second) ----
    secCount++;
    if (nowMs - secStartMs >= 1000) {
      hz = secCount * 1000.0f / (nowMs - secStartMs);
      secCount = 0; secStartMs = nowMs;
      jitterPub = jitterMax; jitterMax = 0;
    }

    // ---- quality + BPM (every 25 samples = 100 ms) ----
    if (++statusCounter >= 25) {
      statusCounter = 0;

      float mx = -1e9, mn = 1e9; int sat = 0;
      for (int i = 0; i < 4; i++) {
        if (binMax[i] > mx) mx = binMax[i];
        if (binMin[i] < mn) mn = binMin[i];
        sat += binSat[i];
      }
      int p2p = (mx > mn) ? (int)(mx - mn) : 0;
      satFlag = (sat >= 10);

      float avgRR = 0, cv = 0;
      if (rrCount > 0) {
        for (int i = 0; i < rrCount; i++) avgRR += rrBuf[i];
        avgRR /= rrCount;
        if (rrCount >= 3) {
          float var = 0;
          for (int i = 0; i < rrCount; i++) { float e = rrBuf[i] - avgRR; var += e * e; }
          cv = sqrtf(var / rrCount) / avgRR;
        }
      }
      uint32_t sinceBeat = (lastPeakMs != 0) ? (nowMs - lastPeakMs) : 0xFFFFFFFF;
      float staleLimit = 2.5f * avgRR;
      if (staleLimit < 3000.0f) staleLimit = 3000.0f;

      uint8_t q; bool bpmOK = false; float bpm = 0;
      if (leadOff)                         q = Q_LEADS_OFF;
      else if (satFlag)                    q = Q_SATURATED;
      else if (learning)                   q = Q_LEARNING;
      else if (p2p < MIN_P2P)              q = Q_FLAT;
      else if (lastPeakMs == 0 || sinceBeat > staleLimit) q = Q_NO_BEATS;
      else if (rrCount < MIN_RR_FOR_BPM)   q = Q_COLLECTING;
      else if (cv > MAX_RR_CV)             q = Q_NOISY;
      else { q = Q_OK; bpmOK = true; bpm = 60000.0f / avgRR; }

      portENTER_CRITICAL(&ecgMux);
      ecgShared.saturated = satFlag;
      ecgShared.bpmValid = bpmOK;
      ecgShared.bpm = bpm;
      ecgShared.quality = q;
      ecgShared.p2p = p2p;
      ecgShared.rrCount = rrCount;
      ecgShared.totalBeats = totalBeats;
      ecgShared.sinceBeatMs = sinceBeat;
      ecgShared.measuredHz = hz;
      ecgShared.maxJitterUs = jitterPub;
      ecgShared.missedSamples = missed;
      portEXIT_CRITICAL(&ecgMux);
    }
  }
}

// Copies the shared ECG state (and optionally the waveform) safely.
int getEcgSnapshot(EcgShared& s, int16_t* disp) {
  int count;
  portENTER_CRITICAL(&ecgMux);
  s = ecgShared;
  count = ecgDispCount;
  if (disp != NULL) {
    int start = (ecgDispCount == DISPLAY_SIZE) ? ecgDispIndex : 0;
    for (int i = 0; i < count; i++) disp[i] = ecgDisplay[(start + i) % DISPLAY_SIZE];
  }
  portEXIT_CRITICAL(&ecgMux);
  return count;
}

// ============================================================
//  WEBSITE
// ============================================================
const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SeptiSense Patient Monitor</title>
<style>
body{font-family:Arial,sans-serif;text-align:center;background:#eef2f7;padding:25px 12px}
.card{max-width:520px;margin:auto;background:#fff;padding:25px 15px;border-radius:15px;box-shadow:0 4px 15px #0002}
.reading{background:#f1f5f9;padding:16px;margin:13px 0;border-radius:10px}
.value{font-size:30px;font-weight:bold;color:#2563eb}
.label{color:#555;font-size:15px}
.status{font-size:19px;font-weight:bold;margin:15px}
.on{color:#16a34a}.off{color:#dc2626}
.note{color:#666;font-size:13px}
canvas{width:100%;height:210px;background:#101d2b;border-radius:10px}
.ecg-status{font-size:15px;font-weight:bold;padding:12px}
</style>
</head>
<body>
<div class="card">
<h1>SeptiSense Monitor</h1>
<p>Live physiological monitoring</p>

<h2>Temperature Monitoring</h2>
<div class="reading"><p class="label">Object Surface Temperature</p>
<div class="value" id="objectTemp">-- &deg;C</div></div>
<div class="reading"><p class="label">Sensor Ambient Temperature</p>
<div class="value" id="ambientTemp">-- &deg;C</div></div>

<h2>Automatic LED Status</h2>
<div class="status" id="ledStatus">WAITING</div>
<p id="tempMessage">Connecting to sensor...</p>
<div class="reading"><p class="label">Temperature thresholds</p>
<p>LED ON: 25&deg;C or above</p><p>LED OFF: 24&deg;C or below</p></div>

<h2>ECG Monitoring - AD8232</h2>
<div class="reading">
<p class="label">ECG Waveform (filtered, baseline removed)</p>
<canvas id="ecgChart" width="1000" height="420"></canvas>
<p class="ecg-status" id="ecgStatus">Waiting for AD8232...</p>
</div>
<div class="reading"><p class="label">Latest raw ECG ADC (0 or 4095 = bad signal)</p>
<div class="value" id="ecgValue">--</div></div>
<div class="reading"><p class="label">ECG-derived Heart Rate (experimental)</p>
<div class="value" id="ecgBpm">-- BPM</div>
<p class="note" id="ecgDiag">--</p>
<p class="note">Experimental R-peak detector. Not medically validated.</p></div>

<h2>Pulse Oximeter - MAX30102</h2>
<div class="reading"><p class="label">Pulse Sensor Status</p>
<div class="ecg-status" id="pulseStatus">Initializing...</div>
<p class="note" id="pulseDiag">--</p></div>
<div class="reading"><p class="label">Optical Heart Rate</p>
<div class="value" id="maxHR">-- BPM</div></div>
<div class="reading"><p class="label">Estimated SpO2</p>
<div class="value" id="maxSpO2">-- %</div></div>
<div class="reading"><p class="label">Beat Consistency</p>
<div class="value" id="beatStatus">--</div></div>

<p class="note">MAX30102 estimates are experimental and not clinically validated.</p>
<p class="note">This prototype is for educational and research purposes only.</p>
</div>

<script>
const canvas=document.getElementById("ecgChart");
const ctx=canvas.getContext("2d");
function $(id){return document.getElementById(id);}

function drawECG(samples){
  const w=canvas.width,h=canvas.height;
  ctx.clearRect(0,0,w,h);
  ctx.strokeStyle="#294052";ctx.lineWidth=1;
  for(let x=0;x<=w;x+=50){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,h);ctx.stroke();}
  for(let y=0;y<=h;y+=42){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke();}
  if(!samples||samples.length<2)return;
  let min=Infinity,max=-Infinity;
  for(const v of samples){if(v<min)min=v;if(v>max)max=v;}
  const range=Math.max(max-min,100);
  const mid=(max+min)/2;
  const low=mid-range*0.55,high=mid+range*0.55;
  ctx.strokeStyle="#45e0a5";ctx.lineWidth=2.5;ctx.beginPath();
  samples.forEach((v,i)=>{
    const x=(i/(samples.length-1))*w;
    const y=h-((v-low)/(high-low))*h;
    if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);
  });
  ctx.stroke();
}

function blankAll(msg){
  $("objectTemp").textContent="-- \u00B0C";
  $("ambientTemp").textContent="-- \u00B0C";
  $("ledStatus").textContent="NO CONNECTION";$("ledStatus").className="status off";
  $("tempMessage").textContent=msg;
  $("ecgValue").textContent="--";$("ecgBpm").textContent="-- BPM";
  $("ecgStatus").textContent=msg;$("ecgStatus").style.color="#dc2626";
  $("ecgDiag").textContent="--";
  $("pulseStatus").textContent=msg;$("pulseStatus").style.color="#dc2626";
  $("maxHR").textContent="-- BPM";$("maxSpO2").textContent="-- %";
  $("beatStatus").textContent="--";
  drawECG(null);
}

async function updateDashboard(){
  try{
    const r=await fetch("/data?t="+Date.now(),{cache:"no-store"});
    if(!r.ok)throw new Error("Server error");
    const d=await r.json();

    // Temperature
    if(d.temperatureOK){
      $("objectTemp").textContent=Number(d.objectC).toFixed(2)+" \u00B0C";
      $("ambientTemp").textContent=Number(d.ambientC).toFixed(2)+" \u00B0C";
      const led=$("ledStatus");
      if(d.ledOn){led.textContent="LED ON";led.className="status on";}
      else{led.textContent="LED OFF";led.className="status off";}
      $("tempMessage").textContent="Automatic temperature control active.";
    }else{
      $("objectTemp").textContent="-- \u00B0C";
      $("ambientTemp").textContent="-- \u00B0C";
      $("ledStatus").textContent="SENSOR ERROR";$("ledStatus").className="status off";
      $("tempMessage").textContent="Check MLX90614.";
    }

    // ECG
    $("ecgValue").textContent=d.ecgValue;
    drawECG(d.ecgSamples);
    const es=$("ecgStatus");
    es.textContent=d.ecgStatus;
    es.style.color=(d.ecgQuality===7)?"#d97706":(d.ecgQuality>=3&&d.ecgQuality<=5)?"#d97706":"#dc2626";
    $("ecgBpm").textContent=d.ecgBpmValid?Number(d.ecgBpm).toFixed(0)+" BPM":"-- BPM";
    $("ecgDiag").textContent="p2p: "+d.ecgP2P+" | RR intervals: "+d.ecgRR+
      " | beats: "+d.ecgBeats+" | rate: "+Number(d.ecgHz).toFixed(0)+" Hz | max jitter: "+
      d.ecgJitterUs+" us | missed: "+d.ecgMissed;

    // MAX30102
    const ps=$("pulseStatus");
    $("beatStatus").textContent=d.consistentIntervals;
    $("pulseDiag").textContent="IR signal: "+d.maxIR+" (finger needs > 30000, saturated near 262143)";
    if(!d.maxSensorOK){
      ps.textContent="MAX30102 NOT DETECTED - CHECK WIRING";ps.style.color="#dc2626";
      $("maxHR").textContent="-- BPM";$("maxSpO2").textContent="-- %";
    }else if(!d.maxFinger){
      ps.textContent="NO FINGER DETECTED - PLACE FINGERTIP ON SENSOR";ps.style.color="#dc2626";
      $("maxHR").textContent="-- BPM";$("maxSpO2").textContent="-- %";
    }else if(d.maxSaturated){
      ps.textContent="SENSOR SATURATED - PRESS LESS / LOWER LED BRIGHTNESS";ps.style.color="#dc2626";
      $("maxHR").textContent="-- BPM";$("maxSpO2").textContent="-- %";
    }else{
      if(d.pulseSignalConsistent){
        ps.textContent=d.maxSpO2Valid?"PULSE SIGNAL CONSISTENT (EXPERIMENTAL)":"PULSE FOUND - SpO2 SETTLING (~5-10 s)";
      }else{
        ps.textContent="FINGER DETECTED - WAITING FOR STEADY PULSE, HOLD STILL";
      }
      ps.style.color="#d97706";
      $("maxHR").textContent=d.maxHRValid?Number(d.heartRate).toFixed(1)+" BPM":"-- BPM";
      $("maxSpO2").textContent=d.maxSpO2Valid?d.spo2+" %":"-- %";
    }
  }catch(e){
    blankAll("Cannot communicate with ESP32.");
  }
}
updateDashboard();
setInterval(updateDashboard,500);
</script>
</body>
</html>
)rawliteral";

// ============================================================
//  /data  JSON endpoint
// ============================================================
void handleData() {
  bool tempValid = temperatureSensorOK && temperatureReadingOK &&
                   isfinite(objectTempC) && isfinite(ambientTempC);

  EcgShared e;
  int16_t disp[DISPLAY_SIZE];
  int count = getEcgSnapshot(e, disp);

  PulseReading p = evaluatePulse();
  unsigned long now = millis();
  long msSinceBeat = (lastBeatMillis != 0) ? (long)(now - lastBeatMillis) : -1;

  String json;
  json.reserve(3600);
  json = "{";
  json += "\"temperatureOK\":"; json += tempValid ? "true" : "false";
  json += ",\"objectC\":";  json += tempValid ? String(objectTempC, 2) : "null";
  json += ",\"ambientC\":"; json += tempValid ? String(ambientTempC, 2) : "null";
  json += ",\"ledOn\":";    json += ledState ? "true" : "false";

  json += ",\"ecgValue\":"; json += String(e.raw);
  json += ",\"leadOff\":";  json += e.leadOff ? "true" : "false";
  json += ",\"ecgSaturated\":"; json += e.saturated ? "true" : "false";
  json += ",\"ecgQuality\":"; json += String((int)e.quality);
  json += ",\"ecgStatus\":\""; json += qualityText(e.quality); json += "\"";
  json += ",\"ecgBpmValid\":"; json += e.bpmValid ? "true" : "false";
  json += ",\"ecgBpm\":"; json += e.bpmValid ? String(e.bpm, 0) : "null";
  json += ",\"ecgP2P\":"; json += String(e.p2p);
  json += ",\"ecgRR\":"; json += String(e.rrCount);
  json += ",\"ecgBeats\":"; json += String((unsigned long)e.totalBeats);
  json += ",\"ecgHz\":"; json += String(e.measuredHz, 1);
  json += ",\"ecgJitterUs\":"; json += String((unsigned long)e.maxJitterUs);
  json += ",\"ecgMissed\":"; json += String((unsigned long)e.missedSamples);

  json += ",\"ecgSamples\":[";
  for (int i = 0; i < count; i++) {
    json += String((int)disp[i]);
    if (i < count - 1) json += ",";
  }
  json += "]";

  json += ",\"maxSensorOK\":"; json += maxSensorOK ? "true" : "false";
  json += ",\"pulseSignalConsistent\":"; json += p.consistent ? "true" : "false";
  json += ",\"consistentIntervals\":"; json += String(consistentBeatIntervals);
  json += ",\"msSinceBeat\":"; json += String(msSinceBeat);
  json += ",\"maxFinger\":"; json += p.fingerPresent ? "true" : "false";
  json += ",\"maxSaturated\":"; json += p.saturated ? "true" : "false";
  json += ",\"maxIR\":"; json += String((unsigned long)p.irValue);
  json += ",\"maxHRValid\":"; json += p.hrValid ? "true" : "false";
  json += ",\"maxSpO2Valid\":"; json += p.spo2Valid ? "true" : "false";
  json += ",\"heartRate\":"; json += p.hrValid ? String(p.hr, 1) : "null";
  json += ",\"spo2\":"; json += p.spo2Valid ? String(p.spo2) : "null";
  json += "}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

// ============================================================
//  Helpers
// ============================================================
void scanI2C() {
#if SERIAL_MODE == 0
  Serial.println("Scanning I2C bus (expect 0x57 MAX30102, 0x5A MLX90614)...");
  int found = 0;
  for (byte a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Found device at 0x");
      if (a < 16) Serial.print("0");
      Serial.println(a, HEX);
      found++;
    }
  }
  if (found == 0) Serial.println("  No I2C devices found - check SDA/SCL/power/GND.");
#endif
}

bool startPulseSensor() {
  bool ok = particleSensor.begin(Wire, I2C_SPEED_STANDARD, 0x57);
  if (ok) {
    // brightness, sampleAverage, ledMode (2 = Red+IR), sampleRate, pulseWidth, adcRange
    particleSensor.setup(MAX_LED_BRIGHTNESS, 1, 2, 100, 411, 4096);
  }
  Wire.setClock(100000);   // keep the shared bus at 100 kHz
  fingerPresent = false;
  resetPulseState();
  return ok;
}

void updateTemperature(unsigned long now) {
  if (now - previousTempMillis < TEMP_INTERVAL) return;
  previousTempMillis = now;

  if (!temperatureSensorOK) {                    // try to (re)connect
    temperatureSensorOK = mlx.begin(0x5A, &Wire);
    temperatureReadingOK = false;
    ledState = false;
    digitalWrite(LED_PIN, LOW);
    return;
  }

  float o = mlx.readObjectTempC();
  float a = mlx.readAmbientTempC();
  bool ok = isfinite(o) && isfinite(a) && o > -70 && o < 380 && a > -40 && a < 125;

  if (ok) {
    objectTempC = o; ambientTempC = a;
    temperatureReadingOK = true; tempFailCount = 0;
    if (objectTempC >= LED_ON_TEMP_C) ledState = true;
    else if (objectTempC <= LED_OFF_TEMP_C) ledState = false;   // hysteresis
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  } else {
    temperatureReadingOK = false;
    ledState = false;
    digitalWrite(LED_PIN, LOW);
    if (++tempFailCount >= 3) { temperatureSensorOK = false; tempFailCount = 0; }
  }
}

void checkPulseSensor(unsigned long now) {
  if (now - lastMaxCheckMs < 2000) return;
  lastMaxCheckMs = now;
  Wire.beginTransmission(0x57);
  bool present = (Wire.endTransmission() == 0);

  if (maxSensorOK) {
    if (!present) {
      if (++maxPingFails >= 2) { maxSensorOK = false; maxPingFails = 0; }
    } else maxPingFails = 0;
  } else if (present) {
    maxSensorOK = startPulseSensor();
  }
}

// ============================================================
//  TELEMETRY TO THE LAPTOP BACKEND  (runs in its own task)
// ============================================================
// Samples are queued every TELEMETRY_INTERVAL_MS. A separate task on core 0
// sends them over Wi-Fi, so a slow network can never disturb the sensors.
// If the laptop is unreachable, up to TELEM_QUEUE samples are kept and sent later.
#define TELEM_QUEUE 60
TelemetryRecord telemQueue[TELEM_QUEUE];
int telemTail = 0, telemCount = 0;
portMUX_TYPE telemMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t telemSentOK = 0, telemFailed = 0;
volatile int telemLastCode = 0;

void queueTelemetry(unsigned long now) {
  static unsigned long lastQueued = 0;
  if (now - lastQueued < TELEMETRY_INTERVAL_MS) return;
  lastQueued = now;

  EcgShared e;
  getEcgSnapshot(e, NULL);
  PulseReading p = evaluatePulse();

  TelemetryRecord r;
  r.t_ms = (uint32_t)now;
  r.mlxOK = temperatureSensorOK;
  r.tempOK = temperatureSensorOK && temperatureReadingOK &&
             isfinite(objectTempC) && isfinite(ambientTempC);
  r.obj = objectTempC; r.amb = ambientTempC; r.ledOn = ledState;
  r.maxOK = maxSensorOK; r.finger = p.fingerPresent;
  r.hrOK = p.hrValid; r.hr = p.hr;
  r.spo2OK = p.spo2Valid; r.spo2 = p.spo2;
  r.ecgOK = e.bpmValid; r.ecgBpm = e.bpm; r.ecgQ = e.quality; r.leadOff = e.leadOff;

  portENTER_CRITICAL(&telemMux);
  if (telemCount == TELEM_QUEUE) {                 // full: drop the oldest
    telemTail = (telemTail + 1) % TELEM_QUEUE;
    telemCount--;
  }
  telemQueue[(telemTail + telemCount) % TELEM_QUEUE] = r;
  telemCount++;
  portEXIT_CRITICAL(&telemMux);
}

static void appendNum(String& s, bool valid, float v, int decimals) {
  if (valid && isfinite(v)) s += String(v, decimals);
  else s += "null";
}

void telemetryTask(void* parameter) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (strlen(STA_SSID) == 0) continue;                    // telemetry disabled
    if (WiFi.status() != WL_CONNECTED) continue;            // auto-reconnect is on

    TelemetryRecord batch[10];
    int n = 0;
    portENTER_CRITICAL(&telemMux);
    n = (telemCount < 10) ? telemCount : 10;
    for (int i = 0; i < n; i++) batch[i] = telemQueue[(telemTail + i) % TELEM_QUEUE];
    portEXIT_CRITICAL(&telemMux);
    if (n == 0) continue;

    String body;
    body.reserve(1800);
    body = "{\"device_id\":\"";
    body += DEVICE_ID;
    body += "\",\"sent_ms\":";
    body += String((uint32_t)millis());
    body += ",\"records\":[";
    for (int i = 0; i < n; i++) {
      TelemetryRecord& r = batch[i];
      if (i) body += ",";
      body += "{\"t\":" + String(r.t_ms);
      body += ",\"mlx\":" + String(r.mlxOK ? 1 : 0);
      body += ",\"tOK\":" + String(r.tempOK ? 1 : 0);
      body += ",\"obj\":"; appendNum(body, r.tempOK, r.obj, 2);
      body += ",\"amb\":"; appendNum(body, r.tempOK, r.amb, 2);
      body += ",\"led\":" + String(r.ledOn ? 1 : 0);
      body += ",\"max\":" + String(r.maxOK ? 1 : 0);
      body += ",\"fin\":" + String(r.finger ? 1 : 0);
      body += ",\"hrOK\":" + String(r.hrOK ? 1 : 0);
      body += ",\"hr\":"; appendNum(body, r.hrOK, r.hr, 1);
      body += ",\"spo2OK\":" + String(r.spo2OK ? 1 : 0);
      body += ",\"spo2\":" + String(r.spo2OK ? r.spo2 : 0);
      body += ",\"ecgOK\":" + String(r.ecgOK ? 1 : 0);
      body += ",\"ecg\":"; appendNum(body, r.ecgOK, r.ecgBpm, 0);
      body += ",\"ecgQ\":" + String((int)r.ecgQ);
      body += ",\"lo\":" + String(r.leadOff ? 1 : 0);
      body += "}";
    }
    body += "]}";

    HTTPClient http;
    http.setConnectTimeout(2000);
    http.setTimeout(3000);
    http.begin(BACKEND_URL);
    http.addHeader("Content-Type", "application/json");
    int code = http.POST(body);
    http.end();
    telemLastCode = code;

    if (code == 200) {
      telemSentOK++;
      portENTER_CRITICAL(&telemMux);
      telemTail = (telemTail + n) % TELEM_QUEUE;
      telemCount -= n;
      if (telemCount < 0) telemCount = 0;
      portEXIT_CRITICAL(&telemMux);
    } else {
      telemFailed++;
    }
  }
}

void printSerialStatus() {
#if SERIAL_MODE == 0
  EcgShared e;
  getEcgSnapshot(e, NULL);
  PulseReading p = evaluatePulse();

  Serial.println();
  Serial.println("========== SEPTISENSE ==========");
  if (temperatureReadingOK) {
    Serial.print("Object temperature: ");  Serial.print(objectTempC, 2);  Serial.println(" C");
    Serial.print("Ambient temperature: "); Serial.print(ambientTempC, 2); Serial.println(" C");
  } else Serial.println("Temperature unavailable.");
  Serial.print("LED: "); Serial.println(ledState ? "ON" : "OFF");

  Serial.print("ECG raw ADC: "); Serial.println(e.raw);
  Serial.print("ECG leads: "); Serial.println(e.leadOff ? "LEADS OFF" : "CONNECTED");
  Serial.print("ECG status: "); Serial.println(qualityText(e.quality));
  if (e.bpmValid) { Serial.print("ECG BPM (experimental): "); Serial.println(e.bpm, 0); }
  else Serial.println("ECG BPM: -- (not reliable)");
  Serial.print("ECG p2p="); Serial.print(e.p2p);
  Serial.print(" RR="); Serial.print(e.rrCount);
  Serial.print(" beats="); Serial.print(e.totalBeats);
  Serial.print(" rate="); Serial.print(e.measuredHz, 1);
  Serial.print("Hz jitter="); Serial.print(e.maxJitterUs);
  Serial.print("us missed="); Serial.println(e.missedSamples);

  Serial.print("MAX30102: "); Serial.println(maxSensorOK ? "CONNECTED" : "NOT DETECTED");
  Serial.print("Finger: "); Serial.print(p.fingerPresent ? "YES" : "NO");
  Serial.print(" | IR value: "); Serial.print(p.irValue);
  Serial.println(p.saturated ? " (SATURATED)" : "");
  Serial.print("Consistent beat intervals: "); Serial.println(consistentBeatIntervals);
  if (p.hrValid) { Serial.print("Optical HR (experimental): "); Serial.print(p.hr, 1); Serial.println(" BPM"); }
  else Serial.println("Optical HR: UNAVAILABLE");
  if (p.spo2Valid) { Serial.print("Estimated SpO2: "); Serial.print(p.spo2); Serial.println(" %"); }
  else Serial.println("SpO2: UNAVAILABLE");
  Serial.print("Telemetry: ");
  if (strlen(STA_SSID) == 0) Serial.println("DISABLED (STA_SSID empty)");
  else {
    Serial.print(WiFi.status() == WL_CONNECTED ? "Wi-Fi OK" : "Wi-Fi NOT CONNECTED");
    Serial.print(" | sent batches: "); Serial.print(telemSentOK);
    Serial.print(" | failed: "); Serial.print(telemFailed);
    Serial.print(" | last HTTP code: "); Serial.print(telemLastCode);
    Serial.print(" | queued: "); Serial.println(telemCount);
  }
  Serial.println("===============================");
#endif
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

#if SERIAL_MODE == 0
  Serial.println();
  Serial.println("=================================");
  Serial.println(" SEPTISENSE MULTI-SENSOR MONITOR");
  Serial.println("=================================");
#endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Pull-ups: if a LO wire is loose it reads HIGH = "leads off" (safe default)
  pinMode(LO_PLUS_PIN, INPUT_PULLUP);
  pinMode(LO_MINUS_PIN, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(ECG_PIN, ADC_11db);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(50);   // never hang forever on a bad I2C bus

  scanI2C();

  temperatureSensorOK = mlx.begin(0x5A, &Wire);
  maxSensorOK = startPulseSensor();

#if SERIAL_MODE == 0
  Serial.println(temperatureSensorOK ? "MLX90614 detected." : "MLX90614 NOT detected.");
  Serial.println(maxSensorOK ? "MAX30102 initialized." : "MAX30102 NOT detected.");
#endif

  WiFi.mode(WIFI_AP_STA);
  if (!WiFi.softAP(ssid, password)) {
#if SERIAL_MODE == 0
    Serial.println("Wi-Fi failed!");
#endif
    while (true) delay(1000);
  }

#if SERIAL_MODE == 0
  Serial.print("Network: "); Serial.println(ssid);
  Serial.print("Website: http://"); Serial.println(WiFi.softAPIP());
#endif

  if (strlen(STA_SSID) > 0) {
    WiFi.setAutoReconnect(true);
    WiFi.begin(STA_SSID, STA_PASS);
#if SERIAL_MODE == 0
    Serial.print("Joining network for telemetry: "); Serial.println(STA_SSID);
#endif
  }

  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", webpage); });
  server.on("/data", HTTP_GET, handleData);
  server.begin();

  // ECG sampling task on core 1 (higher priority than loop())
  xTaskCreatePinnedToCore(ecgTask, "ecgTask", 6144, NULL, 3, NULL, 1);
  // Telemetry sender on core 0 (the Wi-Fi core), low priority
  xTaskCreatePinnedToCore(telemetryTask, "telemTask", 8192, NULL, 1, NULL, 0);

#if SERIAL_MODE == 0
  Serial.println("ECG task started (250 Hz).");
  Serial.println("SYSTEM READY.");
#endif
}

// ============================================================
//  LOOP
// ============================================================
unsigned long previousSerialMillis = 0;

void loop() {
  servicePulseSensor();      // must run very frequently
  server.handleClient();
  servicePulseSensor();

  unsigned long now = millis();
  updateTemperature(now);
  checkPulseSensor(now);
  queueTelemetry(now);

  if (lastBeatMillis != 0 && now - lastBeatMillis >= PULSE_TIMEOUT) {
    consistentBeatIntervals = 0;
    previousBeatInterval = 0;
  }

#if SERIAL_MODE == 0
  if (now - previousSerialMillis >= 1000) {
    previousSerialMillis = now;
    printSerialStatus();
  }
#endif

  delay(1);
}
