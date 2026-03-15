#pragma once

#ifndef ACTUATOR_MODULE_H
#define ACTUATOR_MODULE_H

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_PWMServoDriver.h>

/* ---------- CONFIG ---------- */
#define PCA_FREQ        50
#define SERVO_STEP      10
#define EEPROM_MAGIC    0xA5
#define EEPROM_ADDR     0   // starting address

/* ---------- ACTUATOR IDS ---------- */
enum ActuatorID {
  LEFT_EAR_YAW = 0,
  LEFT_EAR_PITCH,
  RIGHT_EAR_YAW,
  RIGHT_EAR_PITCH,
  NUM_ACTUATORS
};

/* ---------- DATA STRUCT ---------- */
struct Actuator {
  uint8_t  channel;
  uint16_t minPulse;
  uint16_t idlePulse;
  uint16_t maxPulse;
  uint16_t currentPulse;
  bool     inverted;
};

/* ---------- CLASS ---------- */
class ActuatorModule {
public:
  ActuatorModule(uint8_t i2cAddr = 0x40);

  void begin();
  void attach(uint8_t id, uint8_t channel, bool inverted = false,uint16_t idleGuess = 375);

  void calibrate(uint8_t id);
  void setNormalized(uint8_t id, float value);

  void saveToEEPROM();
  bool loadFromEEPROM();

private:
  Adafruit_PWMServoDriver pwm;
  Actuator actuators[NUM_ACTUATORS];
  const char* actuatorName(uint8_t id);

  uint16_t eepromBase(uint8_t id);
};

#endif
