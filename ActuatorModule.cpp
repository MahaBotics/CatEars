#include "ActuatorModule.h"

// ---------- DEFAULT SERVO CALIBRATION ----------

// Servo 0
#define LEFT_EAR_YAW_MIN       315
#define LEFT_EAR_YAW_IDLE      475
#define LEFT_EAR_YAW_MAX       535

// Servo 1
#define LEFT_EAR_PITCH_MIN     275
#define LEFT_EAR_PITCH_IDLE    375
#define LEFT_EAR_PITCH_MAX     500

// Servo 2
#define RIGHT_EAR_YAW_MIN      115
#define RIGHT_EAR_YAW_IDLE     275
#define RIGHT_EAR_YAW_MAX      335

// Servo 3
#define RIGHT_EAR_PITCH_MIN    275
#define RIGHT_EAR_PITCH_IDLE   375
#define RIGHT_EAR_PITCH_MAX    500


/* ---------- CONSTRUCTOR ---------- */
ActuatorModule::ActuatorModule(uint8_t i2cAddr)
  : pwm(i2cAddr) {}

/* ---------- ACTUATOR NAME ---------- */
const char* ActuatorModule::actuatorName(uint8_t id) {
  switch (id) {
    case LEFT_EAR_YAW:    return "LEFT EAR YAW";
    case LEFT_EAR_PITCH:  return "LEFT EAR PITCH";
    case RIGHT_EAR_YAW:   return "RIGHT EAR YAW";
    case RIGHT_EAR_PITCH: return "RIGHT EAR PITCH";
    default:              return "UNKNOWN ACTUATOR";
  }
}


/* ---------- INIT ---------- */
void ActuatorModule::begin() {
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(PCA_FREQ);
  delay(10);
}

/* ---------- ATTACH ---------- */
void ActuatorModule::attach(uint8_t id, uint8_t channel, bool inverted, uint16_t idleGuess) {

  actuators[id].channel      = channel;
  actuators[id].inverted     = inverted;
  actuators[id].currentPulse = idleGuess;


  // Apply hardcoded calibration
  setDefaultCalibration(id);


  // Move servo to idle position
  pwm.setPWM(channel, 0, actuators[id].idlePulse);

}

/* ---------- CALIBRATION ---------- */
void ActuatorModule::calibrate(uint8_t id) {
  Actuator &a = actuators[id];
  uint16_t pulse = a.idlePulse;

  Serial.println();
  Serial.println("================================");
  Serial.print("Calibrating: ");
  Serial.println(actuatorName(id));
  Serial.println("================================");
  Serial.println("+ / - : move");
  Serial.println("m = MIN | i = IDLE | M = MAX");
  Serial.println("v = TOGGLE INVERSION");
  Serial.println("q = quit");

  pwm.setPWM(a.channel, 0, pulse);

  while (true) {
    if (!Serial.available()) continue;
    char c = Serial.read();

    if (c == '+') pulse += SERVO_STEP;
    if (c == '-') pulse -= SERVO_STEP;

    pulse = constrain(pulse, 100, 600);
    pwm.setPWM(a.channel, 0, pulse);
    a.currentPulse = pulse;

    if (c == 'm') { a.minPulse = pulse; Serial.println("MIN saved"); 
    Serial.print("Pulse: ");
    Serial.println(pulse);
    }

    if (c == 'i') { a.idlePulse = pulse; Serial.println("IDLE saved"); 
    Serial.print("Pulse: ");
    Serial.println(pulse);
    }

    if (c == 'M') { a.maxPulse = pulse; Serial.println("MAX saved"); 
    Serial.print("Pulse: ");
    Serial.println(pulse);
    }

    if (c == 'v') {a.inverted = !a.inverted; Serial.print("Inversion: ");
    Serial.println(a.inverted ? "ON" : "OFF");
    Serial.print("Pulse: ");
    Serial.println(pulse);
    }

    if (c == 'q') {
      Serial.println("Back to Calibration Menu");
      break;
    }

  }
}

/* ---------- DEFAULT CALIBRATION ---------- */
void ActuatorModule::setDefaultCalibration(uint8_t id) {

  switch(id) {

    case LEFT_EAR_YAW:
      actuators[id].minPulse  = LEFT_EAR_YAW_MIN;
      actuators[id].idlePulse = LEFT_EAR_YAW_IDLE;
      actuators[id].maxPulse  = LEFT_EAR_YAW_MAX;
      break;


    case LEFT_EAR_PITCH:
      actuators[id].minPulse  = LEFT_EAR_PITCH_MIN;
      actuators[id].idlePulse = LEFT_EAR_PITCH_IDLE;
      actuators[id].maxPulse  = LEFT_EAR_PITCH_MAX;
      break;


    case RIGHT_EAR_YAW:
      actuators[id].minPulse  = RIGHT_EAR_YAW_MIN;
      actuators[id].idlePulse = RIGHT_EAR_YAW_IDLE;
      actuators[id].maxPulse  = RIGHT_EAR_YAW_MAX;
      break;


    case RIGHT_EAR_PITCH:
      actuators[id].minPulse  = RIGHT_EAR_PITCH_MIN;
      actuators[id].idlePulse = RIGHT_EAR_PITCH_IDLE;
      actuators[id].maxPulse  = RIGHT_EAR_PITCH_MAX;
      break;
  }
}

/* ---------- NORMALIZED CONTROL ---------- */
void ActuatorModule::setNormalized(uint8_t id, float value) {
  Actuator &a = actuators[id];

  value = constrain(value, -1.0f, 1.0f);

  // Apply inversion
  if (a.inverted) {
    value = -value;
  }
  uint16_t target;

  if (value >= 0.0f) {
    target = a.idlePulse +
             value * ((int16_t)a.maxPulse - (int16_t)a.idlePulse);
  } else {
    target = a.idlePulse +
             value * ((int16_t)a.idlePulse - (int16_t)a.minPulse);
  }

  pwm.setPWM(a.channel, 0, target);
  a.currentPulse = target;
}
