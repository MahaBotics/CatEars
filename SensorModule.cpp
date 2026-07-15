#include "SensorModule.h"

SensorModule::SensorModule()
: mpuHealthy(true),
  lastMPUUpdate(0),
  loopTimer(0),
  RateCalibrationRoll(0),
  RateCalibrationPitch(0),
  RateCalibrationYaw(0),
  KalmanAngleRoll(0),
  KalmanAnglePitch(0),
  KalmanUncertaintyRoll(4),
  KalmanUncertaintyPitch(4),
  Q_angle(16),
  R_angle(9),
  dt(0.004) {}

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

/* ===================== KALMAN ===================== */
void SensorModule::setKalmanParams(float q_angle,
                                   float r_angle,
                                   float dt_sec) {
  Q_angle = q_angle;
  R_angle = r_angle;
  dt = dt_sec;
}

void SensorModule::kalman1D(float &state,
                            float &uncertainty,
                            float input,
                            float measurement) {
  state += dt * input;
  uncertainty += dt * dt * Q_angle;

  float K = uncertainty / (uncertainty + R_angle);
  state += K * (measurement - state);
  uncertainty *= (1 - K);
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
  Wire.write(0x05);
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

  kalman1D(KalmanAngleRoll,
           KalmanUncertaintyRoll,
           RateRoll,
           AngleRoll);
  rollAngle = KalmanAngleRoll;


  kalman1D(KalmanAnglePitch,
           KalmanUncertaintyPitch,
           RatePitch,
           AnglePitch);

  pitchAngle = KalmanAnglePitch;


  while (micros() - loopTimer < LOOP_PERIOD_US);
  loopTimer = micros();
}

/* ===================== GETTERS ===================== */

bool  SensorModule::healthy() const { return mpuHealthy; }

/* ---------- RAW GETTERS ---------- */

float SensorModule::roll() const {
  return KalmanAngleRoll;
}

float SensorModule::pitch() const {
  return KalmanAnglePitch;
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
    delay(4);
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

    delay(4);
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

auto waitStill = [](const char *msg)
{
    while (Serial.available())
        Serial.read();

    Serial.println(msg);
    Serial.println("Press ENTER when steady...");

    while (true)
    {
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
  

  // 2. Pitch Up
  waitStill("2) MAX PITCH UP");
  sampleStillPose(calib.pitchUp, tmpRoll);

  // 3. Pitch Down
  waitStill("3) MAX PITCH DOWN");
  sampleStillPose(calib.pitchDown, tmpRoll);

  // 4. Roll Left
  waitStill("4) MAX ROLL LEFT");
  sampleStillPose(tmpPitch, calib.rollLeft);

  // 5. Roll Right
  waitStill("5) MAX ROLL RIGHT");
  sampleStillPose(tmpPitch, calib.rollRight);

  // 6. Hold Neutral 
  waitStill("6) Maintain Neutral head position");

  // Validate calibration
  if (!(calib.pitchUp < calib.pitchNeutral &&
        calib.pitchDown > calib.pitchNeutral &&
        calib.rollLeft < calib.rollNeutral &&
        calib.rollRight > calib.rollNeutral)) {

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

  if (p >= 0) {

    float range = calib.pitchDown - calib.pitchNeutral;

    if (fabs(range) < 0.1f)
      return 0.0f;

    return constrain(p / range, 0.0f, 1.0f);
  }
  else {

    float range = calib.pitchNeutral - calib.pitchUp;

    if (fabs(range) < 0.1f)
      return 0.0f;

    return constrain(p / range, -1.0f, 0.0f);
  }
}


float SensorModule::rollNorm() const {
  float r = rollAngle - calib.rollNeutral;
  if (r > 0)
    return constrain(r / (calib.rollRight - calib.rollNeutral), 0.0f, 1.0f);
  else
    return constrain(r / (calib.rollNeutral - calib.rollLeft), -1.0f, 0.0f);
}


