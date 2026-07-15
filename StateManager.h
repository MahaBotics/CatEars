#pragma once
#include <Arduino.h>
#include "EmotionLibrary.h"

/* ================= CONFIG ================= */
#define EMOTION_HOLD_TIME_MS 600   // min time before switching again
#define TRANSITION_RATE     0.02f  // smoothing speed

/* ================= STATE MANAGER ================= */
class StateManager {
public:
  StateManager();

  void begin();

  // Call every loop
  void update(const Attitude& att);

  // Output actuator targets (normalized)
  void getActuatorTargets(float* out);

  Emotion currentEmotion() const;
  const char* currentEmotionName() const;

private:
  Emotion current;
  Emotion target;

  uint32_t lastTransitionTime;

  float currentOutput[NUM_ACTUATORS];
  float targetOutput[NUM_ACTUATORS];

  EmotionLibrary emotions;

  /* ----- logic ----- */
  Emotion decideEmotion(const Attitude& att);
  void smoothTransition();
};
