#pragma once
#include <Arduino.h>
#include "ActuatorModule.h"

// /* ================= ACTUATOR INDEX ================= */
// enum ActuatorID {
//   LEFT_EAR_YAW = 0,
//   LEFT_EAR_PITCH,
//   RIGHT_EAR_YAW,
//   RIGHT_EAR_PITCH,
//   ACTUATOR_COUNT
// };

/* ================= ATTITUDE INPUT ================= */
// All values normalized to [-1, +1]
struct Attitude {
  float roll;     // left (-) to right (+)
  float pitch;    // down (-) to up (+)
  float yawRate;  // CCW (-) to CW (+)
};

/* ================= EMOTIONS ======================= */
enum Emotion {
  EMO_IDLE = 0,
  EMO_HAPPY,
  EMO_CURIOUS,
  EMO_SAD,
  EMO_ALERT,
  EMO_COUNT
};

/* ================= GAIN TUNING ==================== */
struct EmotionGains {
  float pitchGain;     // how much pitch affects perk
  float rollYawGain;   // roll → yaw asymmetry
  float yawRateGain;   // twitch / alert response
  float idleGain;      // idle motion strength
};

/* ================= API ============================ */
class EmotionLibrary {
public:
  EmotionLibrary();

  void setEmotion(Emotion e);
  Emotion currentEmotion() const;

  void setGains(const EmotionGains& g);

  // Compute actuator outputs (normalized [-1,1])
  void compute(const Attitude& att, float* actuatorOut);

private:
  Emotion current;
  EmotionGains gains;

  /* ---- emotion implementations ---- */
  void idle(const Attitude&, float*);
  void happy(const Attitude&, float*);
  void curious(const Attitude&, float*);
  void sad(const Attitude&, float*);
  void alert(const Attitude&, float*);

  inline void clamp(float*);
};
