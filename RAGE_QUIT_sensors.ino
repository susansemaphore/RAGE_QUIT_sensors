#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;

// Default sensitivity scale factors (±2g and ±250 °/s)
const float ACCEL_SCALE = 16384.0f; // LSB/g
const float GYRO_SCALE  = 131.0f;   // LSB/(°/s)

// Timing intervals (ms)
const unsigned long CALIBRATION_MS         = 2000;
const unsigned long SAMPLE_INTERVAL_MS     = 20;
const unsigned long SETTLE_HOLD_MS         = 400;
const unsigned long REARM_DELAY_MS         = 200;
const unsigned long TWITCH_MAX_DURATION_MS = 400;
const unsigned long HOLD_REQUIRED_MS       = 1000;

// Thresholds (tune to your tabletop mass and mounting)
const float ACCEL_HOME_TOL_G    = 0.05f;
const float GYRO_STILL_DPS      = 4.0f;

const float TWITCH_HOME_TOL_G   = 0.10f;

const float HOLD_ACCEL_MIN_G    = 0.12f;
const float HOLD_STABLE_TOL_G   = 0.08f;

const float FLIP_ACCEL_MIN_G    = 0.35f;

// 2-sample moving average buffers
const uint8_t WINDOW_SIZE = 2;
float axBuf[WINDOW_SIZE] = {0.0f};
float azBuf[WINDOW_SIZE] = {0.0f};
float gxBuf[WINDOW_SIZE] = {0.0f};
float gyBuf[WINDOW_SIZE] = {0.0f};
float gzBuf[WINDOW_SIZE] = {0.0f};
uint8_t bufIndex = 0;
uint8_t sampleCount = 0;

float avgAx = 0.0f;
float avgAz = 0.0f;
float avgGx = 0.0f;
float avgGy = 0.0f;
float avgGz = 0.0f;

// Calibration baseline values (first 2 seconds)
float axStart = 0.0f;
float azStart = 0.0f;
float gxBias  = 0.0f;
float gyBias  = 0.0f;
float gzBias  = 0.0f;

double calibAxSum = 0.0;
double calibAzSum = 0.0;
double calibGxSum = 0.0;
double calibGySum = 0.0;
double calibGzSum = 0.0;
uint16_t calibSamples = 0;

enum GameState {
  STATE_CALIBRATING,
  STATE_SCENARIO_SELECT,
  STATE_SENSING,
  STATE_TWITCH,
  STATE_HOLD,
  STATE_FLIPPING,
  STATE_COOLDOWN
};

GameState state = STATE_CALIBRATING;

unsigned long bootMs = 0;
unsigned long lastSampleMs = 0;
unsigned long endedRoundMs = 0;

// Sensing, twitch & hold tracking
unsigned long sensingStartMs = 0;
unsigned long lastTwitchMs = 0;
unsigned long startTimeMs = 0;

bool holdingAngle = false;
unsigned long holdStartMs = 0;
float holdBaseAx = 0.0f;
float holdBaseAz = 0.0f;

// Round return-to-start tracking
bool trackingHomeCandidate = false;
unsigned long homeCandidateSinceMs = 0;

float computeAverage(const float* buf, uint8_t count) {
  if (count == 0) return 0.0f;
  float sum = 0.0f;
  for (uint8_t i = 0; i < count; i++) {
    sum += buf[i];
  }
  return sum / count;
}

float maxAbsThree(float a, float b, float c) {
  float m = fabs(a);
  if (fabs(b) > m) m = fabs(b);
  if (fabs(c) > m) m = fabs(c);
  return m;
}

bool readRawSensors(float& axOut, float& azOut, float& gxOut, float& gyOut, float& gzOut) {
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

  int16_t rawGx = (Wire.read() << 8) | Wire.read(); // 0x43–0x44
  int16_t rawGy = (Wire.read() << 8) | Wire.read(); // 0x45–0x46
  int16_t rawGz = (Wire.read() << 8) | Wire.read(); // 0x47–0x48

  axOut = rawAx / ACCEL_SCALE;
  azOut = rawAz / ACCEL_SCALE;
  gxOut = rawGx / GYRO_SCALE;
  gyOut = rawGy / GYRO_SCALE;
  gzOut = rawGz / GYRO_SCALE;
  return true;
}

void pushSample(float ax, float az, float gx, float gy, float gz) {
  axBuf[bufIndex] = ax;
  azBuf[bufIndex] = az;
  gxBuf[bufIndex] = gx;
  gyBuf[bufIndex] = gy;
  gzBuf[bufIndex] = gz;

  bufIndex = (bufIndex + 1) % WINDOW_SIZE;
  if (sampleCount < WINDOW_SIZE) {
    sampleCount++;
  }

  avgAx = computeAverage(axBuf, sampleCount);
  avgAz = computeAverage(azBuf, sampleCount);
  avgGx = computeAverage(gxBuf, sampleCount);
  avgGy = computeAverage(gyBuf, sampleCount);
  avgGz = computeAverage(gzBuf, sampleCount);
}

void printRow(unsigned long now) {
  Serial.print(now - startTimeMs);
  Serial.print(',');
  Serial.print(avgAx, 3);
  Serial.print(',');
  Serial.print(avgAz, 3);
  Serial.print(',');
  Serial.print(avgGx, 2);
  Serial.print(',');
  Serial.print(avgGy, 2);
  Serial.print(',');
  Serial.println(avgGz, 2);
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

  Serial.println("t_ms,ax,az,gx,gy,gz");
  Serial.println("*** STATE: STATE_CALIBRATING ***");
}

void loop() {
  unsigned long now = millis();

  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs += SAMPLE_INTERVAL_MS;

    float axRaw = 0.0f, azRaw = 0.0f, gxRaw = 0.0f, gyRaw = 0.0f, gzRaw = 0.0f;
    if (!readRawSensors(axRaw, azRaw, gxRaw, gyRaw, gzRaw)) {
      return;
    }

    if (state == STATE_CALIBRATING) {
      calibAxSum += axRaw;
      calibAzSum += azRaw;
      calibGxSum += gxRaw;
      calibGySum += gyRaw;
      calibGzSum += gzRaw;
      calibSamples++;

      if (now - bootMs >= CALIBRATION_MS && calibSamples > 0) {
        axStart = calibAxSum / calibSamples;
        azStart = calibAzSum / calibSamples;
        gxBias  = calibGxSum / calibSamples;
        gyBias  = calibGySum / calibSamples;
        gzBias  = calibGzSum / calibSamples;

        for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
          axBuf[i] = axStart;
          azBuf[i] = azStart;
          gxBuf[i] = 0.0f;
          gyBuf[i] = 0.0f;
          gzBuf[i] = 0.0f;
        }
        bufIndex = 0;
        sampleCount = WINDOW_SIZE;
        avgAx = axStart;
        avgAz = azStart;
        avgGx = 0.0f;
        avgGy = 0.0f;
        avgGz = 0.0f;

        holdingAngle = false;
        state = STATE_SCENARIO_SELECT;
        Serial.println("*** STATE: STATE_SCENARIO_SELECT *** (TWITCH to scroll, HOLD 1s to select)");
      }
      return;
    }

    pushSample(axRaw, azRaw, gxRaw - gxBias, gyRaw - gyBias, gzRaw - gzBias);
    if (state == STATE_SENSING || state == STATE_FLIPPING) {
      printRow(now);
    }

    float dAx = avgAx - axStart;
    float dAz = avgAz - azStart;
    float accelDelta = sqrt(dAx * dAx + dAz * dAz);
    float peakGyroDps = maxAbsThree(avgGx, avgGy, avgGz);

    bool isAtStart = (accelDelta <= ACCEL_HOME_TOL_G) && (peakGyroDps <= GYRO_STILL_DPS);
    bool nearRest  = (accelDelta <= TWITCH_HOME_TOL_G) && (peakGyroDps <= GYRO_STILL_DPS);

    bool holdElevated = (accelDelta >= HOLD_ACCEL_MIN_G) && (accelDelta < FLIP_ACCEL_MIN_G);

    if (state == STATE_HOLD) {
      // Held selection stays active until the table is lowered back to rest
      if (nearRest) {
        state = STATE_SCENARIO_SELECT;
        Serial.println("STATUS:SCENARIO_SELECT (TWITCH to scroll, HOLD 1s to select)");
      }
      return;
    }

    bool canHold = (state == STATE_SCENARIO_SELECT || state == STATE_SENSING || state == STATE_TWITCH);
    if (canHold) {
      if (!holdingAngle) {
        if (holdElevated) {
          holdingAngle = true;
          holdStartMs = now;
          holdBaseAx = avgAx;
          holdBaseAz = avgAz;
        }
      } else if (!holdElevated) {
        holdingAngle = false;
      } else {
        float dHoldAx = avgAx - holdBaseAx;
        float dHoldAz = avgAz - holdBaseAz;
        bool inPlace = sqrt(dHoldAx * dHoldAx + dHoldAz * dHoldAz) <= HOLD_STABLE_TOL_G;

        // Table is still but has settled at a new angle: re-anchor and restart the timer.
        // Movement while the table is not still is ignored, so it can't cancel the hold.
        if (!inPlace && peakGyroDps <= GYRO_STILL_DPS) {
          holdBaseAx = avgAx;
          holdBaseAz = avgAz;
          holdStartMs = now;
        } else if (inPlace && now - holdStartMs >= HOLD_REQUIRED_MS) {
          holdingAngle = false;
          state = STATE_HOLD;
          Serial.println("*** STATE: STATE_HOLD ***");
          return;
        }
      }
    }

    if (state == STATE_SCENARIO_SELECT) {
      if (!nearRest) {
        state = STATE_SENSING;
        Serial.println("*** STATE: STATE_SENSING ***");
        sensingStartMs = now;
        startTimeMs = now;
      }
      return;
    }

    if (state == STATE_SENSING) {
      bool withinTwitchWindow = (now - sensingStartMs < TWITCH_MAX_DURATION_MS);

      // Back at rest inside the window is a TWITCH; after it, a non-holding table is a FLIP
      if (nearRest) {
        if (withinTwitchWindow) {
          state = STATE_TWITCH;
          lastTwitchMs = now;
          Serial.println("*** STATE: STATE_TWITCH ***");
        } else {
          state = STATE_SCENARIO_SELECT;
        }
        return;
      }

      if (!withinTwitchWindow && !holdingAngle) {
        state = STATE_FLIPPING;
        Serial.println("*** STATE: STATE_FLIPPING ***");
        startTimeMs = sensingStartMs;
        trackingHomeCandidate = false;
        Serial.println("STARTED ROUND");
      }
      return;
    }

    if (state == STATE_TWITCH) {
      // Wait for the table to settle back to rest before accepting new input
      if (nearRest && (now - lastTwitchMs >= REARM_DELAY_MS)) {
        state = STATE_SCENARIO_SELECT;
      }
      return;
    }

    if (state == STATE_FLIPPING) {
      if (isAtStart) {
        if (!trackingHomeCandidate) {
          trackingHomeCandidate = true;
          homeCandidateSinceMs = now;
        } else if (now - homeCandidateSinceMs >= SETTLE_HOLD_MS) {
          Serial.println("*** STATE: STATE_COOLDOWN ***");
          state = STATE_COOLDOWN;
          endedRoundMs = now;
          trackingHomeCandidate = false;
          Serial.println("ENDED ROUND");
        }
      } else {
        trackingHomeCandidate = false;
      }
    } else if (state == STATE_COOLDOWN) {
      if (now - endedRoundMs >= REARM_DELAY_MS) {
        holdingAngle = false;
        state = STATE_SCENARIO_SELECT;
        Serial.println("STATUS:SCENARIO_SELECT (TWITCH to scroll, HOLD 1s to select)");
      }
    }
  }

}