#include "SensorModule.h"

SensorModule::SensorModule()
: mpuHealthy(true),
  lastMPUUpdate(0),
  loopTimer(0),
  RateCalibrationRoll(0),
  RateCalibrationPitch(0),
  RateCalibrationYaw(0),
  q0(1.0f),
  q1(0.0f),
  q2(0.0f),
  q3(0.0f),
  beta(0.1f),
  dt(0.004f),
  doubleTapDetected(false),
  lastTapTime(0),
  tapLatch(false),
  headshakeDetected(false),
  lastYawRate(0.0f),
  maxPeakInHalfCycle(0.0f) {
    memset(strokeTimes, 0, sizeof(strokeTimes));
}

bool SensorModule::begin() {
  Wire.begin();
  Wire.setClock(400000);
  delay(250);

  if (!resetAndInitMPU()) {
    mpuHealthy = false;
    return false;
  }

  /* ---- Load head calibration ---- */
  if (!loadCalibration()) {
    Serial.println("⚠ No head calibration found");
  } else {
    Serial.println("✅ Head calibration loaded from EEPROM");

  /* -------- Print results -------- */
  Serial.println("\n--- Head Calibration Results (degrees) ---");

  Serial.print("Pitch Neutral : "); Serial.println(calib.pitchNeutral, 2);
  Serial.print("Pitch Up      : "); Serial.println(calib.pitchUp, 2);
  Serial.print("Pitch Down    : "); Serial.println(calib.pitchDown, 2);

  Serial.print("Roll Neutral  : "); Serial.println(calib.rollNeutral, 2);
  Serial.print("Roll Left     : "); Serial.println(calib.rollLeft, 2);
  Serial.print("Roll Right    : "); Serial.println(calib.rollRight, 2);

  Serial.println("------------------------------------------");

  }

  loopTimer = micros();
  return true;
}

/* ===================== CALIBRATION ===================== */
void SensorModule::calibrateGyro(uint16_t samples) {
  RateCalibrationRoll  = 0;
  RateCalibrationPitch = 0;
  RateCalibrationYaw   = 0;

  for (uint16_t i = 0; i < samples; i++) {
    readMPU();
    RateCalibrationRoll  += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw   += RateYaw;
    delay(1);
  }

  RateCalibrationRoll  /= samples;
  RateCalibrationPitch /= samples;
  RateCalibrationYaw   /= samples;
}

/* ===================== MADGWICK ===================== */
void SensorModule::setMadgwickParams(float beta_val, float dt_sec) {
  beta = beta_val;
  dt = dt_sec;
}

void SensorModule::madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in normalization)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient descent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    
    // Normalise step size
    recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalise quaternion
  recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

/* ===================== MPU INIT ===================== */
bool SensorModule::resetAndInitMPU() {
  Wire.end();
  delay(5);
  Wire.begin();
  Wire.setClock(400000);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03); // Set DLPF to mode 3 (44 Hz Accel bandwidth, was 10 Hz) to catch fast tap spikes
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  if (Wire.endTransmission() != 0) return false;

  return true;
}

/* ===================== READ MPU ===================== */
bool SensorModule::readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(MPU_ADDR, 6) != 6) return false;
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(MPU_ADDR, 6) != 6) return false;
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();

  RateRoll  = (float)gx / 65.5;
  RatePitch = (float)gy / 65.5;
  RateYaw   = (float)gz / 65.5;

  AccX = (float)ax / 4096.0;
  AccY = (float)ay / 4096.0;
  AccZ = (float)az / 4096.0;

  AngleRoll  = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.2958;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.2958;

  lastMPUUpdate = micros();
  return true;
}

/* ===================== UPDATE LOOP ===================== */
void SensorModule::update() {

  if (!readMPU()) {
    mpuHealthy = false;
  }

  if ((micros() - lastMPUUpdate) > MPU_TIMEOUT_US) {
    mpuHealthy = false;
  }

  if (!mpuHealthy) {
    if (resetAndInitMPU()) {
      mpuHealthy = true;
    }
    delay(5);
    return;
  }

  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;

  // Convert gyro to rad/s for Madgwick
  float gx_rad = RateRoll * 0.0174532925f;
  float gy_rad = RatePitch * 0.0174532925f;
  float gz_rad = RateYaw * 0.0174532925f;

  // Run Madgwick update
  madgwickUpdate(gx_rad, gy_rad, gz_rad, AccX, AccY, AccZ);

  // Extract Euler angles (roll, pitch) in degrees
  float sinp = -2.0f * (q1 * q3 - q0 * q2);
  sinp = constrain(sinp, -1.0f, 1.0f);
  pitchAngle = asin(sinp) * 57.2957795f;

  rollAngle = atan2(2.0f * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 57.2957795f;

  // Software double tap detection
  float accelMag = sqrt(AccX * AccX + AccY * AccY + AccZ * AccZ);
  float diff = fabs(accelMag - 1.0f);

  if (diff > 0.6f) { // Lower threshold for higher sensitivity (0.6g)
    if (!tapLatch) {
      tapLatch = true;
      uint32_t tapInterval = millis() - lastTapTime;
      Serial.print("Tap detected! Interval: ");
      Serial.println(tapInterval);
      if (tapInterval >= 80 && tapInterval <= 650) {
        doubleTapDetected = true;
        Serial.println("DOUBLE TAP TRIGGERED!");
      }
      lastTapTime = millis();
    }
  } else if (diff < 0.25f) { // Reset threshold
    tapLatch = false; 
  }

  // Headshake detection
  float currentYawRate = RateYaw; // degrees per second
  
  // Track peak in current half cycle
  if (fabs(currentYawRate) > fabs(maxPeakInHalfCycle)) {
    maxPeakInHalfCycle = currentYawRate;
  }

  // Zero-crossing detection (sign change)
  if ((currentYawRate > 0.0f && lastYawRate <= 0.0f) || (currentYawRate < 0.0f && lastYawRate >= 0.0f)) {
    // We crossed zero! Check if the peak of the half-cycle we just finished was large enough
    if (fabs(maxPeakInHalfCycle) > 150.0f) { // 150 deg/s is a fast shake
      uint32_t nowMs = millis();
      // Shift stroke times
      for (int i = 3; i > 0; i--) {
        strokeTimes[i] = strokeTimes[i-1];
      }
      strokeTimes[0] = nowMs;

      // Count strokes in the last 1000ms
      int activeStrokes = 0;
      for (int i = 0; i < 4; i++) {
        if (strokeTimes[i] > 0 && (nowMs - strokeTimes[i]) <= 1000) {
          activeStrokes++;
        }
      }

      if (activeStrokes >= 3) {
        headshakeDetected = true;
        // Reset stroke log to prevent double triggering
        memset(strokeTimes, 0, sizeof(strokeTimes));
        Serial.println("HEADSHAKE DETECTED!");
      }
    }
    // Reset peak for the next half cycle
    maxPeakInHalfCycle = 0.0f;
  }
  lastYawRate = currentYawRate;

  while (micros() - loopTimer < LOOP_PERIOD_US);
  loopTimer = micros();
}

/* ===================== GETTERS ===================== */

bool  SensorModule::healthy() const { return mpuHealthy; }

/* ---------- RAW GETTERS ---------- */

float SensorModule::roll() const {
  return rollAngle;
}

float SensorModule::pitch() const {
  return pitchAngle;
}

bool SensorModule::checkDoubleTap() {
  if (doubleTapDetected) {
    doubleTapDetected = false;
    return true;
  }
  return false;
}

bool SensorModule::checkHeadshake() {
  if (headshakeDetected) {
    headshakeDetected = false;
    return true;
  }
  return false;
}

float SensorModule::yawRate() const {
  return RateYaw;
}

#include <math.h>   // for isnan(), isfinite()

void SensorModule::sampleStillPose(float &pitchOut,
                                   float &rollOut,
                                   uint16_t samples) {
  float pSum = 0.0f;
  float rSum = 0.0f;
  uint16_t validSamples = 0;

  /* -------- Let Kalman settle -------- */
  for (uint16_t i = 0; i < 50; i++) {
    update();
  }

  /* -------- Collect valid samples -------- */
  uint16_t attempts = 0;
  const uint16_t MAX_ATTEMPTS = samples * 3;

  while (validSamples < samples && attempts < MAX_ATTEMPTS) {
    attempts++;
    update();

    float p = pitchAngle;
    float r = rollAngle;
    

    // Discard NaN or Inf values
    if (!isfinite(p) || !isfinite(r)) {
      continue;
    }

    pSum += p;
    rSum += r;
    validSamples++;
  }

  /* -------- Handle failure -------- */
  if (validSamples == 0) {
    Serial.println("❌ Calibration failed: no valid IMU samples");
    pitchOut = 0.0f;
    rollOut  = 0.0f;
    return;
  }

  /* -------- Average -------- */
  pitchOut = pSum / validSamples;
  rollOut  = rSum / validSamples;

  /* -------- Debug -------- */
  if (validSamples < samples) {
    Serial.print("⚠ Calibration used ");
    Serial.print(validSamples);
    Serial.print("/");
    Serial.print(samples);
    Serial.println(" valid samples");
  }
}

void SensorModule::calibrateHeadPose() {

  Serial.println("\n=== Head Pose Calibration ===");

auto waitStill = [this](const char *msg)
{
    while (Serial.available())
        Serial.read();

    Serial.println(msg);
    Serial.println("Press ENTER when steady...");

    uint32_t lastPrint = 0;
    while (true)
    {
        update();

        if (millis() - lastPrint >= 500)
        {
            lastPrint = millis();
            Serial.print("Current -> Pitch: ");
            Serial.print(pitchAngle, 1);
            Serial.print(" | Roll: ");
            Serial.println(rollAngle, 1);
        }

        if (Serial.available())
        {
            char c = Serial.read();

            if (c == '\n' || c == '\r')
                break;
        }
    }

    delay(300);
};

  float tmpPitch, tmpRoll;

  // 1. Neutral
  waitStill("1) Neutral head position");
  sampleStillPose(calib.pitchNeutral, calib.rollNeutral);
  calib.rollLeft = calib.rollNeutral - 40.0f;
  calib.rollRight = calib.rollNeutral + 40.0f;

  // 2. Pitch Up
  waitStill("2) MAX PITCH UP");
  sampleStillPose(calib.pitchUp, tmpRoll);

  // 3. Pitch Down
  waitStill("3) MAX PITCH DOWN");
  sampleStillPose(calib.pitchDown, tmpRoll);

  // 4. Hold Neutral 
  waitStill("4) Maintain Neutral head position");

  // Validate calibration (supports either orientation of MPU sensor as long as opposite directions deflection is clear)
  bool pitchValid = ((calib.pitchUp < calib.pitchNeutral && calib.pitchDown > calib.pitchNeutral) ||
                     (calib.pitchUp > calib.pitchNeutral && calib.pitchDown < calib.pitchNeutral)) &&
                    (fabs(calib.pitchUp - calib.pitchNeutral) > 2.0f) &&
                    (fabs(calib.pitchDown - calib.pitchNeutral) > 2.0f);

  bool rollValid = ((calib.rollLeft < calib.rollNeutral && calib.rollRight > calib.rollNeutral) ||
                    (calib.rollLeft > calib.rollNeutral && calib.rollRight < calib.rollNeutral)) &&
                   (fabs(calib.rollLeft - calib.rollNeutral) > 2.0f) &&
                   (fabs(calib.rollRight - calib.rollNeutral) > 2.0f);

  if (!pitchValid || !rollValid) {

    Serial.println("\n❌ Calibration failed!");
    Serial.println("\n--- Head Calibration Results (degrees) ---");

    Serial.print("Pitch Neutral : ");
    Serial.println(calib.pitchNeutral, 2);

    Serial.print("Pitch Up      : ");
    Serial.println(calib.pitchUp, 2);

    Serial.print("Pitch Down    : ");
    Serial.println(calib.pitchDown, 2);

    Serial.print("Roll Neutral  : ");
    Serial.println(calib.rollNeutral, 2);

    Serial.print("Roll Left     : ");
    Serial.println(calib.rollLeft, 2);

    Serial.print("Roll Right    : ");
    Serial.println(calib.rollRight, 2);

    Serial.println("Please repeat calibration.");
    return;
  }

  Serial.println("\n--- Head Calibration Results (degrees) ---");

  Serial.print("Pitch Neutral : ");
  Serial.println(calib.pitchNeutral, 2);

  Serial.print("Pitch Up      : ");
  Serial.println(calib.pitchUp, 2);

  Serial.print("Pitch Down    : ");
  Serial.println(calib.pitchDown, 2);

  Serial.print("Roll Neutral  : ");
  Serial.println(calib.rollNeutral, 2);

  Serial.print("Roll Left     : ");
  Serial.println(calib.rollLeft, 2);

  Serial.print("Roll Right    : ");
  Serial.println(calib.rollRight, 2);

  Serial.println("------------------------------------------");

  saveCalibration();

  Serial.println("✅ Head pose calibration saved.");
}


/* ---------- EEPROM METHODS ---------- */
bool SensorModule::loadCalibration() {
  uint8_t magic;
  EEPROM.get(SENSOR_EEPROM_ADDR, magic);

  if (magic != SENSOR_EEPROM_MAGIC)
    return false;

  EEPROM.get(SENSOR_EEPROM_ADDR + 1, calib);
  return true;
}

void SensorModule::saveCalibration() {
  EEPROM.put(SENSOR_EEPROM_ADDR, SENSOR_EEPROM_MAGIC);
  EEPROM.put(SENSOR_EEPROM_ADDR + 1, calib);
}



/* ---------- NORMALIZED GETTERS ---------- */
float SensorModule::pitchNorm() const {
  float p = pitchAngle - calib.pitchNeutral;
  bool inverted = (calib.pitchDown < calib.pitchNeutral);
  if (inverted) {
    p = -p;
  }

  if (p >= 0) {
    float range = fabs(calib.pitchDown - calib.pitchNeutral);
    if (range < 0.1f) return 0.0f;
    return constrain(p / range, 0.0f, 1.0f);
  } else {
    float range = fabs(calib.pitchNeutral - calib.pitchUp);
    if (range < 0.1f) return 0.0f;
    return constrain(p / range, -1.0f, 0.0f);
  }
}

float SensorModule::rollNorm() const {
  float r = rollAngle - calib.rollNeutral;
  bool inverted = (calib.rollRight < calib.rollNeutral);
  if (inverted) {
    r = -r;
  }

  if (r >= 0) {
    float range = fabs(calib.rollRight - calib.rollNeutral);
    if (range < 0.1f) return 0.0f;
    return constrain(r / range, 0.0f, 1.0f);
  } else {
    float range = fabs(calib.rollNeutral - calib.rollLeft);
    if (range < 0.1f) return 0.0f;
    return constrain(r / range, -1.0f, 0.0f);
  }
}


