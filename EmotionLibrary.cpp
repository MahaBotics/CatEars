#include "EmotionLibrary.h"

/* ================= CONSTRUCTOR ==================== */
EmotionLibrary::EmotionLibrary() {
  current = EMO_IDLE;

  gains.pitchGain   = 1.0f;
  gains.rollYawGain = 0.5f;
  gains.yawRateGain = 0.8f;
  gains.idleGain    = 0.3f;
}

/* ================= PUBLIC API ===================== */
void EmotionLibrary::setEmotion(Emotion e) {
  current = e;
}

Emotion EmotionLibrary::currentEmotion() const {
  return current;
}

void EmotionLibrary::setGains(const EmotionGains& g) {
  gains = g;
}

void EmotionLibrary::compute(const Attitude& att, float* out) {
  // reset output
  for (int i = 0; i < NUM_ACTUATORS; i++)
    out[i] = 0.0f;

  switch (current) {
    case EMO_IDLE:    idle(att, out);    break;
    case EMO_HAPPY:   happy(att, out);   break;
    case EMO_CURIOUS: curious(att, out); break;
    case EMO_SAD:     sad(att, out);     break;
    case EMO_ALERT:   alert(att, out);   break;
    default: break;
  }

  clamp(out);
}

/* ================= EMOTION LOGIC ================== */

// --- IDLE: subtle breathing ---
void EmotionLibrary::idle(const Attitude&, float* o) {
  float t = millis() * 0.001f;
  float breathe = gains.idleGain * sin(2*t);

  o[LEFT_EAR_PITCH]  += breathe;
  o[RIGHT_EAR_PITCH] += breathe;

  o[LEFT_EAR_YAW]  = 0.0f;
  o[RIGHT_EAR_YAW] = 0.0f;
}

// --- HAPPY: perked & forward ---
void EmotionLibrary::happy(const Attitude& att, float* o) {
  float perk = gains.pitchGain * -att.pitch;

  o[LEFT_EAR_PITCH]  += perk;
  o[RIGHT_EAR_PITCH] += perk;

  o[LEFT_EAR_YAW]  -= 0.3f;
  o[RIGHT_EAR_YAW] -= 0.3f;
}

// --- CURIOUS: asymmetric attention toward tilt direction ---
void EmotionLibrary::curious(const Attitude& att, float* o) {

  const float roll = att.roll;

  // Yaw follows head roll (attention bias)
  const float yawInfluence  = gains.rollYawGain * roll;

  // Pitch perk follows roll magnitude
  const float pitchInfluence = gains.pitchGain * roll;

  // Roll < 0 → head tilted left → right ear attends
  if (roll < 0.0f) {

    // Yaw: right ear forward, left ear back
    o[LEFT_EAR_YAW]  -= 0.3f;
    o[RIGHT_EAR_YAW] += yawInfluence - 0.3f;

    // Pitch: right ear perks up
    o[LEFT_EAR_PITCH]  -= 0.3f;
    o[RIGHT_EAR_PITCH] -= pitchInfluence;

  }
  // Roll > 0 → head tilted right → left ear attends
  else {

    // Yaw: left ear forward, right ear back
    o[LEFT_EAR_YAW]  += yawInfluence - 0.3f;
    o[RIGHT_EAR_YAW] += 0.3f;

    // Pitch: left ear perks up
    o[LEFT_EAR_PITCH]  += pitchInfluence;
    o[RIGHT_EAR_PITCH] -= 0.3f;
  }
}


// --- SAD: drooped & withdrawn ---
void EmotionLibrary::sad(const Attitude&, float* o) {
  o[LEFT_EAR_PITCH]  -= 0.3f;
  o[RIGHT_EAR_PITCH] -= 0.3f;

  o[LEFT_EAR_YAW]  -= 0.2f;
  o[RIGHT_EAR_YAW] -= 0.2f;
}

// --- ALERT: sharp twitch + perk ---
void EmotionLibrary::alert(const Attitude& att, float* o) {
  float twitch = gains.yawRateGain * att.yawRate;

  o[LEFT_EAR_YAW]  += twitch;
  o[RIGHT_EAR_YAW] += twitch;

  o[LEFT_EAR_PITCH]  += 0.5f;
  o[RIGHT_EAR_PITCH] += 0.5f;
}

/* ================= UTIL =========================== */
void EmotionLibrary::clamp(float* o) {
  for (int i = 0; i < NUM_ACTUATORS; i++) {
    o[i] = constrain(o[i], -1.0f, 1.0f);
  }
}
