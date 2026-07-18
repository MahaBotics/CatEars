#include <avr/wdt.h>
#include "SensorModule.h"
#include "ActuatorModule.h"
#include "StateManager.h"

/* ================== OBJECTS ================== */
SensorModule   imu;
ActuatorModule ears;
StateManager   state;
uint32_t imuUnhealthyStartTime = 0;

/* ================== SETUP ================== */
void setup() {
  Serial.begin(115200);
  delay(200);

  /* ---------- ACTUATORS ---------- */
  ears.begin();

  ears.attach(LEFT_EAR_YAW,    0);
  ears.attach(LEFT_EAR_PITCH,  1);
  ears.attach(RIGHT_EAR_YAW,   2, true);
  ears.attach(RIGHT_EAR_PITCH, 3, true);

  Serial.println("Actuators initialized with default calibration.");

  /* ---------- SENSOR ---------- */
  if (!imu.begin()) {
    Serial.println("❌ MPU init failed");
    while (1);
  }

  Serial.println("Calibrating gyro, Hold Still...");
  imu.calibrateGyro();
  Serial.println("Gyro calibration done"); 

  imu.setMadgwickParams(
    0.45f,     // beta (filter gain)
    0.004f    // dt (250 Hz / 4ms loop period)
  );

  // // Uncomment to calibrate imu pose
  // imu.calibrateHeadPose();

  /* ---------- STATE MANAGER ---------- */
  state.begin();

  Serial.println("System ready");
  Serial.println("Type 'c' for actuator calibration help");

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
      



      case 'e':
        Serial.println("Exit.");
        return;

      default:
        Serial.println("\nCalibration Commands:");
        Serial.println("c0 = LEFT_EAR_YAW");
        Serial.println("c1 = LEFT_EAR_PITCH");
        Serial.println("c2 = RIGHT_EAR_YAW");
        Serial.println("c3 = RIGHT_EAR_PITCH");
        Serial.println("ce = Exit");
        Serial.println("ch = Help");
        break;
    }
  }

}

/* ================== LOOP ================== */
void loop() {

  // Uncomment for actuator Calibration
  // handleSerialCommands();

  imu.update();

  if (!imu.healthy()) {
    if (imuUnhealthyStartTime == 0) {
      imuUnhealthyStartTime = millis();
    }
    Serial.println("⚠ MPU unhealthy");
    if (millis() - imuUnhealthyStartTime > 1000) {
      Serial.println("IMU unhealthy for >1s. Resetting board...");
      delay(100);
      wdt_enable(WDTO_15MS);
      while (true);
    }
    delay(100);
    return;
  } else {
    imuUnhealthyStartTime = 0;
  }

  /* ---------- BUILD ATTITUDE ---------- */
  Attitude att;
  att.roll      = imu.rollNorm();     // [-1, 1]
  att.pitch     = imu.pitchNorm();//bias    // [-1, 1]
  att.yawRate   = 0;// TO DO : imu.yawRateNorm();  // [-1, 1]
  att.doubleTap = imu.checkDoubleTap();
  att.headshake = imu.checkHeadshake();

  /* ---------- STATE MACHINE ---------- */
  state.update(att);

  /* ---------- ACTUATOR TARGETS ---------- */
  float act[NUM_ACTUATORS];
  state.getActuatorTargets(act);

  ears.setNormalized(LEFT_EAR_YAW,    act[LEFT_EAR_YAW]);
  ears.setNormalized(LEFT_EAR_PITCH,  act[LEFT_EAR_PITCH]);
  ears.setNormalized(RIGHT_EAR_YAW,   act[RIGHT_EAR_YAW]);
  ears.setNormalized(RIGHT_EAR_PITCH, act[RIGHT_EAR_PITCH]);

  /* ---------- DEBUG ---------- */
  Serial.print("Emotion: ");
  Serial.print(state.currentEmotionName());
  Serial.print(" | Roll: ");
  Serial.print(imu.roll(), 1);
  Serial.print(" (");
  Serial.print(att.roll, 2);
  Serial.print(") | Pitch: ");
  Serial.print(imu.pitch(), 1);
  Serial.print(" (");
  Serial.print(att.pitch, 2);
  Serial.println(")");
}
