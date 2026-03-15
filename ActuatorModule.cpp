#include "ActuatorModule.h"

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
  actuators[id].idlePulse    = idleGuess;
  actuators[id].currentPulse = idleGuess;
  actuators[id].inverted     = inverted;
  pwm.setPWM(channel, 0, idleGuess);
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

    if (c == 'm') { a.minPulse = pulse; Serial.println("MIN saved"); }
    if (c == 'i') { a.idlePulse = pulse; Serial.println("IDLE saved"); }
    if (c == 'M') { a.maxPulse = pulse; Serial.println("MAX saved"); }
    if (c == 'v') {a.inverted = !a.inverted; Serial.print("Inversion: ");
      Serial.println(a.inverted ? "ON" : "OFF");
    }
    if (c == 'q') break;

    // Serial.print("Pulse: ");
    // Serial.println(pulse);
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
             value * (a.maxPulse - a.idlePulse);
  } else {
    target = a.idlePulse +
             value * (a.idlePulse - a.minPulse);
  }

  pwm.setPWM(a.channel, 0, target);
  a.currentPulse = target;
}


/* ---------- EEPROM ADDRESSING ---------- */
uint16_t ActuatorModule::eepromBase(uint8_t id) {
  return EEPROM_ADDR + 1 + id * sizeof(Actuator);
}

/* ---------- SAVE ---------- */
void ActuatorModule::saveToEEPROM() {
  EEPROM.update(EEPROM_ADDR, EEPROM_MAGIC);

  for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
    EEPROM.put(eepromBase(i), actuators[i]);
  }

  Serial.println("Calibration saved to EEPROM");
}

/* ---------- LOAD ---------- */
bool ActuatorModule::loadFromEEPROM() {
  if (EEPROM.read(EEPROM_ADDR) != EEPROM_MAGIC) {
    Serial.println("EEPROM invalid");
    return false;
  }

  for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
    EEPROM.get(eepromBase(i), actuators[i]);
    pwm.setPWM(actuators[i].channel, 0, actuators[i].idlePulse);
  }

  Serial.println("Calibration loaded from EEPROM");
  return true;
}
