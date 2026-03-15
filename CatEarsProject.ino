#include "SensorModule.h"
#include "ActuatorModule.h"
#include "StateManager.h"

/* ================== OBJECTS ================== */
SensorModule   imu;
ActuatorModule ears;
StateManager   state;

/* ================== SETUP ================== */
void setup() {
  Serial.begin(115200);
  delay(200);

  /* ---------- ACTUATORS ---------- */
  ears.begin();

  ears.attach(LEFT_EAR_YAW,    0, true);
  ears.attach(LEFT_EAR_PITCH,  1);
  ears.attach(RIGHT_EAR_YAW,   2);
  ears.attach(RIGHT_EAR_PITCH, 3, true);

  if (!ears.loadFromEEPROM()) {
    Serial.println("No actuator calibration found. Starting calibration...");
    ears.calibrate(LEFT_EAR_YAW);
    ears.calibrate(LEFT_EAR_PITCH);
    ears.calibrate(RIGHT_EAR_YAW);
    ears.calibrate(RIGHT_EAR_PITCH);
    ears.saveToEEPROM();
  } else {
    Serial.println("Actuator calibration loaded from EEPROM");
  }

  /* ---------- SENSOR ---------- */
  if (!imu.begin()) {
    Serial.println("❌ MPU init failed");
    while (1);
  }

  Serial.println("Calibrating gyro, Hold Still...");
  imu.calibrateGyro();
  Serial.println("Gyro calibration done");

  // if (!imu.loadCalibration()) {
    // imu.calibrateHeadPose();
  // }


  imu.setKalmanParams(
    0.001f,   // Q
    4.0f,     // R // (0.072deg)^2 = 0.005deg^2
    0.004f    // dt (250 Hz)
  );

  /* ---------- STATE MANAGER ---------- */
  state.begin();

  Serial.println("System ready");
  Serial.println("Type 'ch' for actuator calibration help");
}

/* ================== SERIAL COMMANDS ================== */
void handleSerialCommands() {
  if (!Serial.available()) return;

  char cmd = Serial.read();

  if (cmd == 'c') {
    while (!Serial.available());
    char arg = Serial.read();

    switch (arg) {
      case '0': ears.calibrate(LEFT_EAR_YAW);    break;
      case '1': ears.calibrate(LEFT_EAR_PITCH);  break;
      case '2': ears.calibrate(RIGHT_EAR_YAW);   break;
      case '3': ears.calibrate(RIGHT_EAR_PITCH); break;
      case 's':
        ears.saveToEEPROM();
        Serial.println("Calibration saved");
        break;
      case 'l':
        ears.loadFromEEPROM();
        Serial.println("Calibration loaded");
        break;
      default:
        Serial.println("\nCalibration Commands:");
        Serial.println("c0 = LEFT_EAR_YAW");
        Serial.println("c1 = LEFT_EAR_PITCH");
        Serial.println("c2 = RIGHT_EAR_YAW");
        Serial.println("c3 = RIGHT_EAR_PITCH");
        Serial.println("cs = Save EEPROM");
        Serial.println("cl = Load EEPROM");
        Serial.println("ch = Help");
        break;
    }
  }
}

/* ================== LOOP ================== */
void loop() {

  handleSerialCommands();

  imu.update();

  if (!imu.healthy()) {
    Serial.println("⚠ MPU unhealthy");
    delay(100);
    return;
  }

  /* ---------- BUILD ATTITUDE ---------- */
  Attitude att;
  att.roll    = imu.rollNorm();     // [-1, 1]
  att.pitch   = imu.pitchNorm();//bias    // [-1, 1]
  att.yawRate = 0;// TO DO : imu.yawRateNorm();  // [-1, 1]

  /* ---------- STATE MACHINE ---------- */
  state.update(att);

  /* ---------- ACTUATOR TARGETS ---------- */
  float act[NUM_ACTUATORS];
  state.getActuatorTargets(act);

  ears.setNormalized(LEFT_EAR_YAW,    act[LEFT_EAR_YAW]);
  ears.setNormalized(LEFT_EAR_PITCH,  act[LEFT_EAR_PITCH]);
  ears.setNormalized(RIGHT_EAR_YAW,   act[RIGHT_EAR_YAW]);
  ears.setNormalized(RIGHT_EAR_PITCH, act[RIGHT_EAR_PITCH]);

  // /* ---------- DEBUG ---------- */
  // Serial.print("Emotion: ");
  // Serial.print(state.currentEmotion());
  // Serial.print(" | RollN: ");
  // Serial.print(att.roll, 3);
  // Serial.print(" PitchN: ");
  // Serial.println(att.pitch, 3);

  delay(4); // ~250 Hz
}
