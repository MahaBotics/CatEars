#include "StateManager.h"

StateManager::StateManager() {
  current = EMO_IDLE;
  target  = EMO_IDLE;
  lastTransitionTime = 0;
  happyStartTime = 0;
  sadStartTime = 0;

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
  smoothTransition();
}

/* ================= OUTPUT ======================= */
void StateManager::getActuatorTargets(float* out) {
  for (int i = 0; i < NUM_ACTUATORS; i++)
    out[i] = currentOutput[i];
}

Emotion StateManager::currentEmotion() const {
  return current;
}

const char* StateManager::currentEmotionName() const {
  switch (current) {
    case EMO_IDLE:     return "IDLE";
    case EMO_HAPPY:    return "HAPPY";
    case EMO_SAD:      return "SAD";
    case EMO_CURIOUS:  return "CURIOUS";
    case EMO_ALERT:    return "ALERT";
    default:           return "UNKNOWN";
  }
}

/* ================= DECISION LOGIC ================ */
/*
  Attitude conventions:
  pitch  +1 = head up
  roll   +1 = right tilt
  yawRate large = sudden movement
*/
Emotion StateManager::decideEmotion(const Attitude& a) {
  // Check double tap to transition to HAPPY state
  if (a.doubleTap && current != EMO_HAPPY) {
    happyStartTime = millis();
    return EMO_HAPPY;
  }

  // Check headshake to transition to SAD state
  if (a.headshake && current != EMO_SAD) {
    sadStartTime = millis();
    return EMO_SAD;
  }

  if (current == EMO_IDLE) {
    if (fabs(a.roll) > 0.8f) {
      return EMO_CURIOUS;
    }
    return EMO_IDLE;
  }
  else if (current == EMO_SAD) {
    // Return to IDLE after 5 seconds, or if head is tilted back
    if (millis() - sadStartTime > 5000) {
      return EMO_IDLE;
    }
    return EMO_SAD;
  }
  else if (current == EMO_CURIOUS) {
    if (fabs(a.roll) < 0.1f) {
      return EMO_IDLE;
    }
    return EMO_CURIOUS;
  }
  else if (current == EMO_HAPPY) {
    // Return to IDLE after 5 seconds
    if (millis() - happyStartTime > 5000) {
      return EMO_IDLE;
    }

    return EMO_HAPPY;
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
