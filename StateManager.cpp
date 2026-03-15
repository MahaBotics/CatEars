#include "StateManager.h"

/* ================= CONSTRUCTOR ================= */
StateManager::StateManager() {
  current = EMO_IDLE;
  target  = EMO_IDLE;
  lastTransitionTime = 0;

  for (int i = 0; i < NUM_ACTUATORS; i++) {
    currentOutput[i] = 0.0f;
    targetOutput[i]  = 0.0f;
  }
}

void StateManager::begin() {
  emotions.setEmotion(EMO_IDLE);
}

/* ================= UPDATE LOOP ================= */
void StateManager::update(const Attitude& att) {

  Emotion next = decideEmotion(att);

  uint32_t now = millis();

  // --- State transition (rate-limited) ---
  if (next != current && (now - lastTransitionTime) > EMOTION_HOLD_TIME_MS) {
    current = next;
    emotions.setEmotion(current);
    lastTransitionTime = now;
  }

  // --- Compute target actuator pose ---
  emotions.compute(att, targetOutput);

  for (int i = 0; i < NUM_ACTUATORS; i++)
    currentOutput[i] = targetOutput[i];

  // // --- Smooth transition ---
  // smoothTransition();
}

/* ================= OUTPUT ======================= */
void StateManager::getActuatorTargets(float* out) {
  for (int i = 0; i < NUM_ACTUATORS; i++)
    out[i] = currentOutput[i];
}

Emotion StateManager::currentEmotion() const {
  return current;
}

/* ================= DECISION LOGIC ================ */
/*
  Attitude conventions:
  pitch  +1 = head up
  roll   +1 = right tilt
  yawRate large = sudden movement
*/
Emotion StateManager::decideEmotion(const Attitude& a) {

  // --- ALERT: sudden rotation ---
  if (fabs(a.yawRate) > 0.7f) {
    return EMO_ALERT;
  }

  // --- HAPPY: upright & stable ---
  if (a.pitch < 0.0f && fabs(a.roll) < 0.5f) {
    return EMO_HAPPY;
  }

  // --- CURIOUS: head tilt ---
  if (fabs(a.roll) > 0.8f) {
    return EMO_CURIOUS;
  }

  // --- SAD: drooping head ---
  if (a.pitch > 0.9f) {
    return EMO_SAD;
  }

  return EMO_IDLE;
}

/* ================= TRANSITION ==================== */
void StateManager::smoothTransition() {
  for (int i = 0; i < NUM_ACTUATORS; i++) {
    currentOutput[i] += TRANSITION_RATE *
                        (targetOutput[i] - currentOutput[i]);
  }
}
