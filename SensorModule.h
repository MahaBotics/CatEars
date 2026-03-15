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

  // Kalman tuning Q-> Prediction Model Uncertainity (gyro), R-> Sensor Model Uncertainity (accelerometer)
  void setKalmanParams(float q_angle,
                       float r_angle,
                       float dt_sec);

  /* ---------- calibration ---------- */
  void calibrateGyro(uint16_t samples=1000);
  void calibrateHeadPose();    

  bool loadCalibration();
  void saveCalibration();

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

  // Kalman
  float KalmanAngleRoll;
  float KalmanAnglePitch;
  float KalmanUncertaintyRoll;
  float KalmanUncertaintyPitch;

  float Q_angle;
  float R_angle;
  float dt;

  // Calibration params
  HeadCalibration calib;

  // Processed Angles
  float rollAngle;
  float pitchAngle;

  // Internal helpers
  bool resetAndInitMPU();
  bool readMPU();
  void kalman1D(float &state,
                float &uncertainty,
                float input,
                float measurement);
  float normalizeAngle(float angle) const;
};

#endif
