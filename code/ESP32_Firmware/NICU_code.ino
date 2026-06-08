// =============================================================
// ESP32 + AD8232 + MAX30102 + MCP9808
// ECG HR + SpO2 + BP Trend + Estimated BP + Temperature
// =============================================================

#include <Wire.h>
#include <Adafruit_MCP9808.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const char* ssid = "NICU";
const char* password = "BMEY2026";

WiFiServer server(80);

Adafruit_MPU6050 mpu;
// ================= PINS =================
#define ECG_PIN 34

#define SDA_PIN 25
#define SCL_PIN 32

#define PAT_AVG_SIZE 8
#define BUZZER_PIN 14

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool oledOK = false;



// ================= MPU6050 APNEA =================
const unsigned long MPU_SAMPLE_INTERVAL_MS = 50;
const unsigned long APNEA_TIME_MS = 10000;
const unsigned long MIN_BREATH_INTERVAL = 1000;
const unsigned long MAX_BREATH_INTERVAL = 8000;


bool mpuOK = false;

float mpuBaseline = 0.0;
const float MPU_ALPHA = 0.95;

const float THRESHOLD_HIGH = 0.06;
const float THRESHOLD_LOW  = 0.01;

bool breathStarted = false;
unsigned long lastBreathTime = 0;
unsigned long lastMPUReadTime = 0;

float breathingRate = 0.0;

bool apneaLatched = false;
bool apneaRecovering = false;
int breathCountSinceApnea = 0;

String apneaStatus = "NORMAL";


// ================= APNEA CONFIRMATION =================
bool chestMovementDetected = false;
bool apneaConfirmed = false;
bool apneaConfirmTimerStarted = false;

unsigned long apneaConfirmStartTime = 0;

const unsigned long APNEA_CONFIRM_TIME_MS = 15000;

// change limits
const int SPO2_DROP_LIMIT = 4;   // SpO2 drops by 3% or more
const int HR_CHANGE_LIMIT = 12;  // HR changes by 10 BPM or more

int apneaStartHR = 0;
int apneaStartSpO2 = 0;

String finalApneaStatus = "NORMAL";

// ================= MCP9808 =================
#define MCP9808_ADDR 0x18
Adafruit_MCP9808 tempSensor = Adafruit_MCP9808();

float bodyTempC = 0.0;
bool tempSensorOK = false;

// ================= MAX30102 REGISTERS =================
#define MAX30102_ADDR   0x57
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA   0x07
#define REG_FIFO_CONFIG 0x08
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA     0x0C
#define REG_LED2_PA     0x0D
#define REG_PART_ID     0xFF

#define FINGER_THRESHOLD 50000
#define PPG_REFRACTORY_MS 500





// ================= ECG SETTINGS =================
#define ECG_LO_PLUS_PIN 18
#define ECG_LO_MINUS_PIN 19

const int SAMPLE_DELAY_MS = 4;

const int MIN_RR_MS = 300;
const int MAX_RR_MS = 1500;

const int HR_BUFFER_SIZE = 10;
const int HR_DIFF_LIMIT = 5;
const int TREND_CONFIRM_COUNT = 6;

// ================= ECG VARIABLES =================
int rawECG = 0;

float lowPass = 0;
float baseline = 0;
float filteredECG = 0;
float ecgPositive = 0;

float peakLevel = 0;
float threshold = 50;

bool aboveThreshold = false;
bool ecgLeadsOK = false;

unsigned long lastPeakTime_ms = 0;
unsigned long lastECGPeakTime_ms = 0;


// ================= ECG HR =================
int currentHR = 0;
int avgHR = 0;

int hrBuffer[HR_BUFFER_SIZE];
int hrIndex = 0;
int hrCount = 0;

int lastAcceptedHR = 0;
int trendCounter = 0;

// ================= SPO2 TABLE =================
const uint8_t spo2_table[184] = {
95,95,95,96,96,96,97,97,97,97,97,98,98,98,98,98,99,99,99,99,
99,99,99,99,100,100,100,100,100,100,100,100,100,100,100,100,
100,100,100,100,100,100,100,100,99,99,99,99,99,99,99,99,
98,98,98,98,98,98,97,97,97,97,96,96,96,96,95,95,95,94,
94,94,93,93,93,92,92,92,91,91,90,90,89,89,89,88,88,87,
87,86,86,85,85,84,84,83,82,82,81,81,80,80,79,78,78,77,
76,76,75,74,74,73,72,72,71,70,69,69,68,67,66,66,65,64,
63,62,62,61,60,59,58,57,56,56,55,54,53,52,51,50,49,48,
47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,31,30,29,
28,27,26,25,23,22,21,20,19,17,16,15,14,12,11,10,9,7,6,5,
3,2,1
};

// ================= PPG FILTER =================
struct Channel {
  float hp_alpha = 0.95f;
  float hp_prev_x = 0;
  float hp_prev_y = 0;

  float ma_buf[4] = {0, 0, 0, 0};
  uint8_t ma_idx = 0;

  float ac_peak = 0;
  float dc_mean = 0;

  void seed(uint32_t raw) {
    hp_prev_x = raw;
    hp_prev_y = 0;
    dc_mean = raw;

    for (int i = 0; i < 4; i++) ma_buf[i] = 0;

    ma_idx = 0;
    ac_peak = 0;
  }

  void reset() {
    hp_prev_x = 0;
    hp_prev_y = 0;
    dc_mean = 0;

    for (int i = 0; i < 4; i++) ma_buf[i] = 0;

    ma_idx = 0;
    ac_peak = 0;
  }

  float dc_filter(uint32_t raw) {
    float x = raw;
    float y = hp_alpha * (hp_prev_y + x - hp_prev_x);

    hp_prev_x = x;
    hp_prev_y = y;

    dc_mean = 0.99f * dc_mean + 0.01f * x;

    float absY = abs(y);

    if (absY > ac_peak) ac_peak = absY;
    else ac_peak *= 0.998f;

    return y;
  }

  float ma_filter(float val) {
    ma_buf[ma_idx] = val;
    ma_idx = (ma_idx + 1) % 4;

    float s = 0;
    for (int i = 0; i < 4; i++) s += ma_buf[i];

    return s / 4.0f;
  }

  float avgAC() { return ac_peak; }
  float avgDC() { return dc_mean; }
};

// ================= PPG PEAK DETECTOR =================
struct PPGPeakDetector {
  float prev = 0;
  bool rising = false;
  bool initialized = false;

  float candidatePeak = 0;
  unsigned long candidatePeakTime = 0;
  unsigned long lastAcceptedPeakTime = 0;

  bool update(float x, unsigned long now, unsigned long &beatTime) {
    if (!initialized) {
      prev = x;
      initialized = true;
      return false;
    }

    float diff = x - prev;

    if (diff > 0) {
      rising = true;

      if (x > candidatePeak) {
        candidatePeak = x;
        candidatePeakTime = now;
      }
    }

    if (rising && diff < -2.0f) {
      rising = false;

      if (candidatePeak < 130) {
        candidatePeak = 0;
        prev = x;
        return false;
      }

      if (lastAcceptedPeakTime > 0 &&
          candidatePeakTime - lastAcceptedPeakTime < PPG_REFRACTORY_MS) {
        candidatePeak = 0;
        prev = x;
        return false;
      }

      beatTime = candidatePeakTime;
      lastAcceptedPeakTime = candidatePeakTime;

      candidatePeak = 0;
      prev = x;

      return true;
    }

    prev = x;
    return false;
  }

  void reset() {
    prev = 0;
    rising = false;
    initialized = false;
    candidatePeak = 0;
    candidatePeakTime = 0;
    lastAcceptedPeakTime = 0;
  }
};

Channel irCh;
Channel redCh;
PPGPeakDetector ppgDetector;

// mpu function ---------------//
bool initMPU6050() {
  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 not found.");
    return false;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("Calibrating MPU... keep sensor still.");

  for (int i = 0; i < 20; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    float totalAcc = sqrt(
      a.acceleration.x * a.acceleration.x +
      a.acceleration.y * a.acceleration.y +
      a.acceleration.z * a.acceleration.z
    );

    mpuBaseline = (i == 0) ? totalAcc :
                  (MPU_ALPHA * mpuBaseline + (1.0 - MPU_ALPHA) * totalAcc);

    delay(50);
  }

  lastBreathTime = millis();

  Serial.print("MPU baseline ready: ");
  Serial.println(mpuBaseline);

  return true;
}

void detectBreath(float breathSignal) {
  unsigned long now = millis();

  if (!breathStarted && breathSignal > THRESHOLD_HIGH) {
    breathStarted = true;
  }

  if (breathStarted && breathSignal < THRESHOLD_LOW) {
    breathStarted = false;

    unsigned long interval = now - lastBreathTime;

    if (interval >= MIN_BREATH_INTERVAL && interval <= MAX_BREATH_INTERVAL) {
      lastBreathTime = now;
      breathingRate = 60000.0 / interval;

      if (apneaLatched) {
        apneaRecovering = true;
        breathCountSinceApnea++;

        if (breathCountSinceApnea >= 2) {
          apneaLatched = false;
          apneaRecovering = false;
          breathCountSinceApnea = 0;
        }
      }
    }
  }
}

void checkApnea() {
  
  unsigned long now = millis();

  if (!apneaLatched && (now - lastBreathTime >= APNEA_TIME_MS)) {

    apneaLatched = true;
    apneaRecovering = false;
    breathCountSinceApnea = 0;

    breathingRate = 0.0;

    lastBreathTime = now;
  }

  if (apneaLatched && !apneaRecovering) {
    apneaStatus = "APNEA";
  } 
  else if (apneaRecovering) {
    apneaStatus = "RECOVERING";
  } 
  else {
    apneaStatus = "NORMAL";
  }
}

void processMPUApnea() {

  if (!mpuOK) return;

  unsigned long now = millis();


  if (now - lastMPUReadTime < MPU_SAMPLE_INTERVAL_MS) return;
  lastMPUReadTime = now;

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  float totalAcc = sqrt(
    accel.acceleration.x * accel.acceleration.x +
    accel.acceleration.y * accel.acceleration.y +
    accel.acceleration.z * accel.acceleration.z
  );

  mpuBaseline = MPU_ALPHA * mpuBaseline + (1.0 - MPU_ALPHA) * totalAcc;

  float breathSignal = totalAcc - mpuBaseline;
  chestMovementDetected = (breathSignal > THRESHOLD_HIGH);

  detectBreath(breathSignal);
  checkApnea();
}


// ================= MAX VARIABLES =================
uint32_t prevIR = 0;
uint32_t prevRed = 0;

bool fingerDown = false;

int SPO2 = 0;
int SPO2f = 0;
#define SPO2_AVG_SIZE 8

int spo2Buffer[SPO2_AVG_SIZE];
int spo2Index = 0;
int spo2Count = 0;
int SPO2_raw = 0;

unsigned long ppgBeatTime_ms = 0;

// ================= BP VARIABLES =================
int pat_ms = 0;
int lastPAT_ms = 0;

const int MIN_PAT_MS = 20;
const int MAX_PAT_MS = 300;

int estimatedSBP = 68;
int estimatedDBP = 38;

String bpTrend = "Unknown";
int patBuffer[PAT_AVG_SIZE];
int patIndex = 0;
int patCount = 0;

int avgPAT_ms = 0;
int previousAvgPAT_ms = 0;

const int PAT_STABLE_DEADBAND_MS = 12;  // increase if still unstable

// ================= TIMING =================
unsigned long lastPrintTime = 0;

// =============================================================
// I2C FUNCTIONS FOR MAX30102
// =============================================================
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(MAX30102_ADDR, 1);

  if (Wire.available()) return Wire.read();

  return 0;
}

bool readFIFO(uint32_t &red, uint32_t &ir) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(REG_FIFO_DATA);
  Wire.endTransmission(false);

  Wire.requestFrom(MAX30102_ADDR, 6);

  if (Wire.available() < 6) return false;

  red  = ((uint32_t)Wire.read() << 16);
  red |= ((uint32_t)Wire.read() << 8);
  red |=  (uint32_t)Wire.read();

  ir  = ((uint32_t)Wire.read() << 16);
  ir |= ((uint32_t)Wire.read() << 8);
  ir |=  (uint32_t)Wire.read();

  red &= 0x03FFFF;
  ir  &= 0x03FFFF;

  return true;
}

// =============================================================
// MCP9808 TEMPERATURE
// =============================================================
void processTemperature() {
  if (tempSensorOK) {
    bodyTempC = tempSensor.readTempC();
  }
}

// =============================================================
// MAX30102 INIT
// =============================================================
bool initMAX30102() {
  for (int i = 0; i < 15; i++) {
    uint8_t id = readReg(REG_PART_ID);

    Serial.print("MAX30102 PART_ID attempt ");
    Serial.print(i + 1);
    Serial.print(": 0x");
    Serial.println(id, HEX);

    if (id == 0x15) break;

    if (i == 14) return false;

    delay(200);
  }

  writeReg(REG_MODE_CONFIG, 0x40);
  delay(500);

  writeReg(REG_FIFO_WR_PTR, 0x00);
  writeReg(REG_OVF_COUNTER, 0x00);
  writeReg(REG_FIFO_RD_PTR, 0x00);

  writeReg(REG_FIFO_CONFIG, 0x4F);
  writeReg(REG_MODE_CONFIG, 0x03);
  writeReg(REG_SPO2_CONFIG, 0x27);

  writeReg(REG_LED1_PA, 0x24);
  writeReg(REG_LED2_PA, 0x24);

  return true;
}

// =============================================================
// ECG HR FILTER FUNCTIONS
// =============================================================
bool compareNewHR(int newHR) {
  if (lastAcceptedHR == 0) {
    lastAcceptedHR = newHR;
    trendCounter = 0;
    return true;
  }

  int difference = abs(newHR - lastAcceptedHR);

  if (difference <= HR_DIFF_LIMIT) {
    lastAcceptedHR = newHR;
    trendCounter = 0;
    return true;
  }

  trendCounter++;

  if (trendCounter >= TREND_CONFIRM_COUNT) {
    lastAcceptedHR = newHR;
    trendCounter = 0;
    return true;
  }

  return false;
}

void saveHRToBuffer(int newHR) {
  hrBuffer[hrIndex] = newHR;
  hrIndex = (hrIndex + 1) % HR_BUFFER_SIZE;

  if (hrCount < HR_BUFFER_SIZE) hrCount++;
}

int calculateAverageHR() {
  if (hrCount == 0) return 0;

  int sum = 0;

  for (int i = 0; i < hrCount; i++) {
    sum += hrBuffer[i];
  }

  return sum / hrCount;
}

// =============================================================
// ECG PROCESSING
// =============================================================
void processECG() {


  ecgLeadsOK = (digitalRead(ECG_LO_PLUS_PIN) == LOW &&
              digitalRead(ECG_LO_MINUS_PIN) == LOW);

  if (!ecgLeadsOK) {
    avgHR = 0;
    currentHR = 0;
    hrCount = 0;
    lastAcceptedHR = 0;
    aboveThreshold = false;
    lastPeakTime_ms = 0;
    lastECGPeakTime_ms = 0;
    return;
  }

  unsigned long now_ms = millis();

  rawECG = analogRead(ECG_PIN);

  lowPass = 0.85 * lowPass + 0.15 * rawECG;
  baseline = 0.995 * baseline + 0.005 * lowPass;

  filteredECG = lowPass - baseline;

  ecgPositive = filteredECG;
  if (ecgPositive < 0) ecgPositive = 0;

  if (ecgPositive > peakLevel) peakLevel = ecgPositive;
  else peakLevel *= 0.985;

  threshold = peakLevel * 0.55;

  if (threshold < 40) threshold = 40;

  bool currentAbove = ecgPositive > threshold;

  if (currentAbove && !aboveThreshold) {
    unsigned long rrInterval = now_ms - lastPeakTime_ms;

    if (rrInterval > MIN_RR_MS && rrInterval < MAX_RR_MS) {
      currentHR = 60000 / rrInterval;

      if (currentHR >= 40 && currentHR <= 200) {
        if (compareNewHR(currentHR)) {
          saveHRToBuffer(currentHR);
          avgHR = calculateAverageHR();
        }
      }
    }

    lastPeakTime_ms = now_ms;
    lastECGPeakTime_ms = now_ms;
  }

  aboveThreshold = currentAbove;
}

// =============================================================
// ARTIFACT REJECTION
// =============================================================
bool isArtifact(uint32_t cur, uint32_t prev) {
  if (prev == 0) return false;

  uint32_t diff = cur > prev ? cur - prev : prev - cur;

  return diff > 40000;
}

// =============================================================
// SPO2 CALCULATION
// =============================================================
int getAverageSpO2(int newSpO2) {
  spo2Buffer[spo2Index] = newSpO2;
  spo2Index = (spo2Index + 1) % SPO2_AVG_SIZE;

  if (spo2Count < SPO2_AVG_SIZE) spo2Count++;

  int sum = 0;
  for (int i = 0; i < spo2Count; i++) {
    sum += spo2Buffer[i];
  }

  return sum / spo2Count;
}

void computeSpO2() {
  float acRed = redCh.avgAC();
  float dcRed = redCh.avgDC();

  float acIR = irCh.avgAC();
  float dcIR = irCh.avgDC();

  if (dcRed <= 0 || dcIR <= 0 || acIR <= 0) return;

  long numerator = (long)((acRed * dcIR) / 64.0f);
  long denominator = (long)((dcRed * acIR) / 64.0f);

  if (denominator == 0) return;

  int RX100 = (int)((numerator * 100L) / denominator);

  SPO2f = (10400 - RX100 * 17 + 50) / 100;
  SPO2f = constrain(SPO2f, 0, 100);

  if (RX100 >= 0 && RX100 < 184) {
  SPO2_raw = spo2_table[RX100];

  if (SPO2_raw >= 70 && SPO2_raw <= 100) {
    SPO2 = getAverageSpO2(SPO2_raw);
  }
}
}

// =============================================================
// BP TREND + ESTIMATION
// =============================================================
int getAveragePAT(int newPAT) {
  patBuffer[patIndex] = newPAT;
  patIndex = (patIndex + 1) % PAT_AVG_SIZE;

  if (patCount < PAT_AVG_SIZE) patCount++;

  int sum = 0;
  for (int i = 0; i < patCount; i++) {
    sum += patBuffer[i];
  }

  return sum / patCount;
}
// =============================================================
void updateBPTrendAndEstimate() {
  if (pat_ms <= 0) {
    bpTrend = "Unknown";
    return;
  }

  avgPAT_ms = getAveragePAT(pat_ms);

  if (previousAvgPAT_ms == 0) {
    previousAvgPAT_ms = avgPAT_ms;
    bpTrend = "Stable";
    return;
  }

  int diff = avgPAT_ms - previousAvgPAT_ms;

  if (abs(diff) <= PAT_STABLE_DEADBAND_MS) {
    bpTrend = "Stable";
  } 
  else if (diff < -PAT_STABLE_DEADBAND_MS) {
    bpTrend = "BP increasing";
  } 
  else if (diff > PAT_STABLE_DEADBAND_MS) {
    bpTrend = "BP decreasing";
  }

  previousAvgPAT_ms = avgPAT_ms;

  int PAT_REF = 120;

  estimatedSBP = 68 + (PAT_REF - avgPAT_ms) / 4;
  estimatedDBP = 38 + (PAT_REF - avgPAT_ms) / 8;

  estimatedSBP = constrain(estimatedSBP, 40, 110);
  estimatedDBP = constrain(estimatedDBP, 20, 80);
}
// =============================================================
// RESET MAX DATA
// =============================================================
void resetMAXData() {
  irCh.reset();
  redCh.reset();
  ppgDetector.reset();

  SPO2 = 0;
  SPO2f = 0;

  prevIR = 0;
  prevRed = 0;

  ppgBeatTime_ms = 0;

  pat_ms = 0;
  lastPAT_ms = 0;
  bpTrend = "Unknown";

  fingerDown = false;

  Serial.println("-- Finger removed. Waiting... --");
}

// =============================================================
// MAX30102 PROCESSING
// =============================================================
void processMAX30102() {
  uint32_t irRaw = 0;
  uint32_t redRaw = 0;

  if (!readFIFO(redRaw, irRaw)) return;

  unsigned long now_ms = millis();

  if (irRaw < FINGER_THRESHOLD) {
    if (fingerDown) resetMAXData();
    return;
  }

  if (!fingerDown) {
    fingerDown = true;

    irCh.seed(irRaw);
    redCh.seed(redRaw);
    ppgDetector.reset();

    prevIR = irRaw;
    prevRed = redRaw;

    Serial.println("-- Finger detected. Measuring SpO2 and BP trend... --");
    return;
  }

  if (isArtifact(irRaw, prevIR) || isArtifact(redRaw, prevRed)) {
    prevIR = irRaw;
    prevRed = redRaw;
    return;
  }

  prevIR = irRaw;
  prevRed = redRaw;

  float irSignal = irCh.dc_filter(irRaw);
  float redSignal = redCh.dc_filter(redRaw);

  float irSmooth = irCh.ma_filter(irSignal);
  redCh.ma_filter(redSignal);

  unsigned long detectedBeatTime_ms = 0;

  bool ppgBeat = ppgDetector.update(irSmooth, now_ms, detectedBeatTime_ms);

  if (ppgBeat) {
    ppgBeatTime_ms = detectedBeatTime_ms;

    computeSpO2();

    if (lastECGPeakTime_ms > 0 && ppgBeatTime_ms > lastECGPeakTime_ms) {
      int tempPAT = ppgBeatTime_ms - lastECGPeakTime_ms;

      if (tempPAT >= MIN_PAT_MS && tempPAT <= MAX_PAT_MS) {
        pat_ms = tempPAT;
        updateBPTrendAndEstimate();
      }
    }
  }
}

// ---------------Apnea confirmation-----------------------------//

void clearApneaFlags() {
  apneaConfirmTimerStarted = false;
  apneaConfirmed = false;
  apneaStartHR = 0;
  apneaStartSpO2 = 0;
}

void confirmApneaUsingHRSpO2() {

  bool strongChestMovement = chestMovementDetected;

    if (strongChestMovement) {
      clearApneaFlags();
      apneaLatched = false;
      apneaRecovering = false;
      breathCountSinceApnea = 0;
      breathingRate = 0.0;
      lastBreathTime = millis();
      apneaStatus = "NORMAL";
      finalApneaStatus = "NORMAL";
      return;
    }
  bool recentBreathDetected = (millis() - lastBreathTime < 3000);

  if ((chestMovementDetected || recentBreathDetected) && apneaStatus != "APNEA") {
    clearApneaFlags();
    finalApneaStatus = "NORMAL";
    return;
  }

  if (apneaStatus == "APNEA") {

    if (!apneaConfirmTimerStarted) {
      apneaConfirmTimerStarted = true;
      apneaConfirmStartTime = millis();

      apneaStartHR = avgHR;
      apneaStartSpO2 = SPO2;

      apneaConfirmed = false;
      finalApneaStatus = "SUSPECTED APNEA";
    }

    int hrDiff = abs(avgHR - apneaStartHR);
    int spo2Drop = apneaStartSpO2 - SPO2;

    bool hrChanged = (avgHR > 0 &&
                      apneaStartHR > 0 &&
                      hrDiff >= HR_CHANGE_LIMIT);

    bool spo2Dropped = (SPO2 > 0 &&
                        apneaStartSpO2 > 0 &&
                        spo2Drop >= SPO2_DROP_LIMIT);

    // Confirm apnea only if HR changes.
    // SpO2 can support the decision, but cannot confirm alone.
    if (hrChanged &&
        (millis() - apneaConfirmStartTime >= APNEA_CONFIRM_TIME_MS)) {

      apneaConfirmed = true;
      finalApneaStatus = "CONFIRMED APNEA";
    } 
    else if (!apneaConfirmed) {
      finalApneaStatus = "SUSPECTED APNEA";
    }

    // Recovery condition while MPU still says apnea:
    // HR returned near start value AND SpO2 is not dropped anymore.
    if (apneaConfirmed &&
        hrDiff < HR_CHANGE_LIMIT &&
        !spo2Dropped) {

      apneaConfirmed = false;
      apneaConfirmTimerStarted = false;
      finalApneaStatus = "SUSPECTED APNEA";
    }

  } 
    
    else {

      bool recentBreathDetected = (millis() - lastBreathTime < 3000);

      if (chestMovementDetected || recentBreathDetected){

        clearApneaFlags();
        finalApneaStatus = "NORMAL";

      } 
      else {

        finalApneaStatus = apneaStatus;
      }
    }

}

/*
void initBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

void updateBuzzer(bool alarmActive) {
  static unsigned long lastToggle = 0;
  static bool buzzerState = false;

  if (!alarmActive) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerState = false;
    return;
  }

  if (millis() - lastToggle >= 300) {
    lastToggle = millis();
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
  }
}
*/

void initOLED() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found");
    oledOK = false;
    return;
  }

  oledOK = true;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Vital Monitor");
  display.println("Starting...");
  display.display();
}

void updateOLED() {
  static unsigned long lastOLEDUpdate = 0;

  if (!oledOK) return;
  if (millis() - lastOLEDUpdate < 1000) return;

  lastOLEDUpdate = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("T:");
  display.print(bodyTempC, 1);
  display.print("C HR:");

  if (ecgLeadsOK && avgHR > 0) {
    display.print(avgHR);
  } else {
    display.print("--");
  }

  display.setCursor(0, 10);
  display.print("SpO2:");
  if (SPO2 > 0) {
    display.print(SPO2);
    display.print("%");
  } else {
    display.print("--");
  }

  display.print(" BP:");

  if (pat_ms > 0) {
    display.print(estimatedSBP);
    display.print("/");
    display.print(estimatedDBP);
  } else {
    display.print("Unk");
  }

  display.setCursor(0, 20);
  display.print("Apnea:");
  display.print(finalApneaStatus.substring(0, 12));

  display.display();
}


// =============================================================
// SETUP
// =============================================================
void setup() {

  
  Serial.begin(115200);
  delay(2000);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());

  server.begin();

  analogReadResolution(12);
  pinMode(ECG_LO_PLUS_PIN, INPUT);
  pinMode(ECG_LO_MINUS_PIN, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

 // initBuzzer();
  initOLED();
  mpuOK = initMPU6050();

  if (!mpuOK) {
    Serial.println("MPU apnea detection disabled.");
  }

  if (!initMAX30102()) {
    Serial.println("ERROR: MAX30102 not found. Check wiring.");
    while (1) delay(1000);
  }

  if (!tempSensor.begin(MCP9808_ADDR)) {
    Serial.println("ERROR: MCP9808 not found. Check wiring.");
    tempSensorOK = false;
  } else {
    tempSensorOK = true;
    tempSensor.setResolution(3);
    Serial.println("MCP9808 initialized.");
  }

  

  Serial.println("System ready.");
}

// =============================================================
// =============app send function ==============================
String buildSensorJSON() {
  String json = "{";

  json += "\"temperature\":";
  json += String(bodyTempC, 1);
  json += ",";

  json += "\"heartRate\":";

if (!ecgLeadsOK || hrCount == 0)
{
  json += "-1";
}
else
{
  json += String(avgHR);
}

json += ",";

json += "\"spo2\":";

if (SPO2 <= 0)
{
  json += "-1";
}
else
{
  json += String(SPO2);
}

json += ",";

json += "\"bpTrend\":\"";

if (pat_ms > 0)
{
  json += bpTrend;
  json += ", ";
  json += String(estimatedSBP);
  json += "/";
  json += String(estimatedDBP);
  json += " mmHg";
}
else
{
  json += "Unknown";
}

json += "\",";

  json += "\"apneaStatus\":\"";
  json += finalApneaStatus;
  json += "\"";

  json += "}";

  return json;
}
// =============================================================
// LOOP
// =============================================================
void loop() {
  unsigned long now_ms = millis();

  processECG();
  processMAX30102();
  processMPUApnea();
  confirmApneaUsingHRSpO2();
  bool alarmActive =
  finalApneaStatus == "CONFIRMED APNEA" ||
  bodyTempC < 22.0 ||
  bodyTempC > 38.5 ||
  (ecgLeadsOK && avgHR > 0 && (avgHR < 40 || avgHR > 120)) ||
  (SPO2 > 0 && SPO2 < 90);

//updateBuzzer(alarmActive);
updateOLED();

  //-----------------------wifi handeling----------------
    // WiFi request handling
  WiFiClient client = server.available();

  if (client) {

    String request = client.readStringUntil('\r');
    client.flush();

    if (request.indexOf("GET /sensors") >= 0) {

      String json = buildSensorJSON();

      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: application/json");
      client.println("Access-Control-Allow-Origin: *");
      client.println("Connection: close");
      client.println();
      client.println(json);
    }

    client.stop();
  }
  //////////////////////////////////////////////////

  if (now_ms - lastPrintTime >= 1000) {
    lastPrintTime = now_ms;

    processTemperature();

    Serial.print("ECG HR: ");

    if (!ecgLeadsOK) {
      Serial.print("LEADS OFF");
    }
    else if (hrCount > 0) {
      Serial.print(avgHR);
      Serial.print(" BPM");
    }
    else {
      Serial.print("---");
    }

    Serial.print(" | SpO2: ");
    if (SPO2 > 0) {
      Serial.print(SPO2);
      Serial.print(" %");
    } else {
      Serial.print("---");
    }

    Serial.print(" | Temp: ");
    if (tempSensorOK) {
      Serial.print(bodyTempC, 2);
      Serial.print(" C");
    } else {
      Serial.print("---");
    }

    Serial.print(" | PAT: ");
    if (pat_ms > 0) {
      Serial.print(avgPAT_ms);
      Serial.print(" ms");
    } else {
      Serial.print("---");
    }

    Serial.print(" | BP Trend: ");
    Serial.print(bpTrend);

    Serial.print(" | Estimated BP: ");
    if (pat_ms > 0) {
      Serial.print(estimatedSBP);
      Serial.print("/");
      Serial.print(estimatedDBP);
      Serial.print(" mmHg");
    } else {
      Serial.print("---");
    }

    Serial.print(" | Resp Rate: ");

    if (!mpuOK) {
      Serial.print("MPU OFF");
    }
    else if (apneaStatus == "APNEA" || finalApneaStatus == "SUSPECTED APNEA") {
      Serial.print("---");
    }
    else if (breathingRate > 5 && breathingRate < 80) {
      Serial.print(breathingRate, 1);
      Serial.print(" br/min");
    }
    else {
      Serial.print("Detecting");
    }

    Serial.print(" | Apnea: ");

    if (!mpuOK) {
      Serial.print("MPU OFF");
    } else {
      Serial.print(finalApneaStatus);
    }

    Serial.println();
  }

  delay(SAMPLE_DELAY_MS);
}

