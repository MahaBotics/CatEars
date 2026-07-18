#pragma once

#ifndef SENSOR_MODULE_H
#define SENSOR_MODULE_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <EEPROM.h>


/* ================= CONFIG ================= */
#define MPU_ADDR            0x68
#define MPU_TIMEOUT_US      20000UL
#define LOOP_PERIOD_US      4000
#define MAX_HEAD_ANGLE_DEG 30.0f
/* ========================================== */

#define EEPROM_MAGIC    0xA5
#define EEPROM_ADDR     0   // starting address


/* ================= EEPROM ================= */
#define SENSOR_EEPROM_MAGIC 0xB2
#define SENSOR_EEPROM_ADDR  200   // avoid actuator EEPROM

struct HeadCalibration {
  float pitchNeutral;
  float pitchUp;
  float pitchDown;
  float rollNeutral;
  float rollLeft;
  float rollRight;
};

class SensorModule {
public:
  SensorModule();

  bool begin();
  void update();

  // Raw filtered outputs
  float roll() const;
  float pitch() const;
  float yawRate() const;

  // Normalized outputs [-1, 1]
  float rollNorm() const;
  float pitchNorm() const;

  bool healthy() const;

  // Madgwick tuning parameters (beta = filter gain, dt_sec = loop rate in seconds)
  void setMadgwickParams(float beta_val, float dt_sec);

  /* ---------- calibration ---------- */
  void calibrateGyro(uint16_t samples=1000);
  void calibrateHeadPose();    

  bool loadCalibration();
  void saveCalibration();
  bool checkDoubleTap();
  bool checkHeadshake();

  void sampleStillPose(float &pitchOut, float &rollOut,
                     uint16_t samples = 200);



private:
  // MPU health
  bool mpuHealthy;
  uint32_t lastMPUUpdate;
  uint32_t loopTimer;

  // Raw rates
  float RateRoll, RatePitch, RateYaw;
  float RateCalibrationRoll;
  float RateCalibrationPitch;
  float RateCalibrationYaw;

  // Acc
  float AccX, AccY, AccZ;
  float AngleRoll, AnglePitch;

  // Madgwick Filter State
  float q0, q1, q2, q3; // Quaternion
  float beta;          // Algorithm gain
  float dt;

  // Double tap detection state
  bool doubleTapDetected;
  uint32_t lastTapTime;
  bool tapLatch;

  // Headshake detection state
  bool headshakeDetected;
  float lastYawRate;
  float maxPeakInHalfCycle;
  uint32_t strokeTimes[4];

  // Calibration params
  HeadCalibration calib;

  // Processed Angles
  float rollAngle;
  float pitchAngle;

  // Internal helpers
  bool resetAndInitMPU();
  bool readMPU();
  void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az);
  float normalizeAngle(float angle) const;
};

#endif
