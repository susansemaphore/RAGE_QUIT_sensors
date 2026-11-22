#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;

// Default sensitivity scale factors (±2g and ±250 °/s)
const float ACCEL_SCALE = 16384.0f; // LSB/g
const float GYRO_SCALE  = 131.0f;   // LSB/(°/s)

// Timing intervals (ms)
const unsigned long CALIBRATION_MS       = 2000;
const unsigned long SAMPLE_INTERVAL_MS   = 50;
const unsigned long STREAM_INTERVAL_MS   = 100;
const unsigned long SETTLE_HOLD_MS       = 400; // Must stay in start zone this long to end attempt

// Motion & "back at start" thresholds (tune to your table's weight/noise)
const float ACCEL_START_DELTA_G = 0.08f; // Deviation from starting (ax, az) that counts as moving
const float ACCEL_HOME_TOL_G    = 0.05f; // Max (ax, az) difference from start to count as "back in start position"
const float GYRO_MOVE_DPS       = 8.0f;  // |gz| above this (deg/s) indicates active rotation
const float GYRO_STILL_DPS      = 4.0f;  // |gz| below this (deg/s) indicates rotation has stopped

// 3-sample moving average buffers
const uint8_t WINDOW_SIZE = 3;
float axBuf[WINDOW_SIZE] = {0.0f};
float azBuf[WINDOW_SIZE] = {0.0f};
float gzBuf[WINDOW_SIZE] = {0.0f};
uint8_t bufIndex = 0;
uint8_t sampleCount = 0;

float avgAx = 0.0f;
float avgAz = 0.0f;
float avgGz = 0.0f;

// Calibration baseline values (captured during first 2 seconds)
float axStart = 0.0f;
float azStart = 0.0f;
float gzBias  = 0.0f;

double calibAxSum = 0.0;
double calibAzSum = 0.0;
double calibGzSum = 0.0;
uint16_t calibSamples = 0;

enum GameState {
  STATE_CALIBRATING,
  STATE_WAITING_FOR_FLIP,
  STATE_FLIPPING,
  STATE_FINISHED
};

GameState state = STATE_CALIBRATING;
unsigned long bootMs = 0;
unsigned long lastSampleMs = 0;
unsigned long lastStreamMs = 0;
unsigned long homeCandidateSinceMs = 0;
bool trackingHomeCandidate = false;

float computeAverage(const float* buf, uint8_t count) {
  if (count == 0) return 0.0f;
  float sum = 0.0f;
  for (uint8_t i = 0; i < count; i++) {
    sum += buf[i];
  }
  return sum / count;
}

bool readRawAxAzGz(float& axOut, float& azOut, float& gzOut) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t BYTES_TO_READ = 14;
  Wire.requestFrom(MPU_ADDR, BYTES_TO_READ, (uint8_t)true);
  if (Wire.available() != BYTES_TO_READ) {
    return false;
  }

  int16_t rawAx = (Wire.read() << 8) | Wire.read(); // 0x3B–0x3C
  (void)Wire.read(); (void)Wire.read();              // 0x3D–0x3E (Accel Y skipped)
  int16_t rawAz = (Wire.read() << 8) | Wire.read(); // 0x3F–0x40

  (void)Wire.read(); (void)Wire.read();              // 0x41–0x42 (Temp skipped)

  (void)Wire.read(); (void)Wire.read();              // 0x43–0x44 (Gyro X skipped)
  (void)Wire.read(); (void)Wire.read();              // 0x45–0x46 (Gyro Y skipped)
  int16_t rawGz = (Wire.read() << 8) | Wire.read(); // 0x47–0x48

  axOut = rawAx / ACCEL_SCALE;
  azOut = rawAz / ACCEL_SCALE;
  gzOut = rawGz / GYRO_SCALE;
  return true;
}

void pushSample(float ax, float az, float gz) {
  axBuf[bufIndex] = ax;
  azBuf[bufIndex] = az;
  gzBuf[bufIndex] = gz;

  bufIndex = (bufIndex + 1) % WINDOW_SIZE;
  if (sampleCount < WINDOW_SIZE) {
    sampleCount++;
  }

  avgAx = computeAverage(axBuf, sampleCount);
  avgAz = computeAverage(azBuf, sampleCount);
  avgGz = computeAverage(gzBuf, sampleCount);
}

void setup() {
  Serial.begin(115200);
  unsigned long startWait = millis();
  while (!Serial && (millis() - startWait < 3000)) {
    ;
  }

  Wire.begin();

  // Wake MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  byte status = Wire.endTransmission();

  if (status != 0) {
    Serial.print("MPU6050 init failed, error: ");
    Serial.println(status);
    while (true) {
      delay(1000);
    }
  }

  bootMs = millis();
  lastSampleMs = bootMs;
  lastStreamMs = bootMs;

  Serial.println("STATUS:CALIBRATING (keep table at starting position for 2s)");
}

void loop() {
  unsigned long now = millis();

  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs += SAMPLE_INTERVAL_MS;

    float axRaw = 0.0f, azRaw = 0.0f, gzRaw = 0.0f;
    if (!readRawAxAzGz(axRaw, azRaw, gzRaw)) {
      return;
    }

    if (state == STATE_CALIBRATING) {
      calibAxSum += axRaw;
      calibAzSum += azRaw;
      calibGzSum += gzRaw;
      calibSamples++;

      if (now - bootMs >= CALIBRATION_MS && calibSamples > 0) {
        axStart = calibAxSum / calibSamples;
        azStart = calibAzSum / calibSamples;
        gzBias  = calibGzSum / calibSamples;

        for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
          axBuf[i] = axStart;
          azBuf[i] = azStart;
          gzBuf[i] = 0.0f;
        }
        bufIndex = 0;
        sampleCount = WINDOW_SIZE;
        avgAx = axStart;
        avgAz = azStart;
        avgGz = 0.0f;

        state = STATE_WAITING_FOR_FLIP;
        Serial.print("STATUS:READY | axStart=");
        Serial.print(axStart, 3);
        Serial.print(" azStart=");
        Serial.print(azStart, 3);
        Serial.print(" gzBias=");
        Serial.println(gzBias, 2);
      }
      return;
    }

    float gzCorrected = gzRaw - gzBias;
    pushSample(axRaw, azRaw, gzCorrected);

    float dAx = avgAx - axStart;
    float dAz = avgAz - azStart;
    float accelDelta = sqrt(dAx * dAx + dAz * dAz);
    bool isAtStart = (accelDelta <= ACCEL_HOME_TOL_G) && (fabs(avgGz) <= GYRO_STILL_DPS);
    bool isMoving  = (accelDelta >= ACCEL_START_DELTA_G) || (fabs(avgGz) >= GYRO_MOVE_DPS);

    if (state == STATE_WAITING_FOR_FLIP && isMoving) {
      state = STATE_FLIPPING;
      trackingHomeCandidate = false;
      lastStreamMs = now;
      Serial.println("STARTED ROUND");
    } else if (state == STATE_FLIPPING) {
      if (isAtStart) {
        if (!trackingHomeCandidate) {
          trackingHomeCandidate = true;
          homeCandidateSinceMs = now;
        } else if (now - homeCandidateSinceMs >= SETTLE_HOLD_MS) {
          state = STATE_FINISHED;
          Serial.println("ENDED ROUND");
        }
      } else {
        trackingHomeCandidate = false;
      }
    }
  }

  // Stream averaged values every 100 ms while the table is actively flipping
  if (state == STATE_FLIPPING && (now - lastStreamMs >= STREAM_INTERVAL_MS)) {
    lastStreamMs += STREAM_INTERVAL_MS;

    Serial.print("t_ms:");
    Serial.print(now - bootMs);
    Serial.print(",ax:");
    Serial.print(avgAx, 3);
    Serial.print(",az:");
    Serial.print(avgAz, 3);
    Serial.print(",gz:");
    Serial.println(avgGz, 2);
  }
}