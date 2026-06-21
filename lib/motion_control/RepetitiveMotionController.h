#pragma once

#include <stdint.h>

struct RepetitiveMotionConfig {
  float start_deg = 0.0f;
  float end_deg = 180.0f;
  float start_to_end_rpm = 1.8f;
  float end_to_start_rpm = 1.8f;
  uint32_t dwell_at_start_ms = 1000;
  uint32_t dwell_at_end_ms = 1000;
};

class RepetitiveMotionController {
 public:
  using StartMoveFn = void (*)(float target_deg, float rpm);
  using IsMoveActiveFn = bool (*)();
  using StopMoveFn = void (*)();

  struct Commands {
    StartMoveFn start_move = nullptr;
    IsMoveActiveFn is_move_active = nullptr;
    StopMoveFn stop_move = nullptr;
  };

  enum class Phase { STOPPED, TO_END, DWELL_AT_END, TO_START, DWELL_AT_START };

  explicit RepetitiveMotionController(const Commands& commands);

  void setConfig(const RepetitiveMotionConfig& config) { config_ = config; }
  const RepetitiveMotionConfig& config() const { return config_; }

  void setRunning(bool running, uint32_t now_ms);
  void stop();
  void update(uint32_t now_ms);

  bool running() const { return running_; }
  Phase phase() const { return phase_; }
  const char* phaseText() const;

 private:
  void beginMove(float target_deg, float rpm, Phase phase);

  Commands commands_;
  RepetitiveMotionConfig config_;
  bool running_ = false;
  Phase phase_ = Phase::STOPPED;
  uint32_t phase_started_ms_ = 0;
};
