#pragma once

#include <Arduino.h>

#include "VelocityEstimator.h"

struct AdrcPositionSettings {
  // Ganhos ADRC. b0 usa a mesma unidade do exemplo original: PWM 8-bit.
  float control_bandwidth = 25.0f;   // wc
  float observer_bandwidth = 80.0f;  // wo
  float plant_gain = 250.0f;         // b0

  float max_target_rpm = 2.4f;
  float physical_max_rpm = 3.0f;
  float stop_window_deg = 1.2f;

  uint16_t accel_ramp_ms = 250;
  uint16_t decel_ramp_ms = 220;
  float kick_pwm_percent = 85.0f;
  uint16_t kick_ms = 180;
  uint16_t samples_to_stop = 3;
  uint16_t velocity_window_ms = 400;
  uint8_t velocity_num_samples = 8;
  float minimum_drive_pwm_percent = 24.0f;

  uint16_t stall_timeout_ms = 1500;
  float stall_velocity_deg_s = 2.0f;

};

class AdrcPositionController {
 public:
  enum class MoveDirection { Shortest, Clockwise, CounterClockwise };

  void setSettings(const AdrcPositionSettings& settings);
  const AdrcPositionSettings& settings() const { return settings_; }

  void startMove(float target_deg, float max_speed_rpm,
                 MoveDirection direction = MoveDirection::Shortest);
  void cancel();
  void primeAccumulatedAngle(float current_deg);
  float computeOutputPercent(float current_deg, uint32_t now_ms);

  bool isActive() const { return active_; }
  bool isKicking() const { return active_ && kicking_; }
  bool isCruising() const { return active_ && !kicking_; }
  bool isStalled() const { return stalled_; }
  float targetDeg() const { return target_deg_; }
  float maxSpeedRpm() const { return max_speed_rpm_; }
  float lastErrorDeg() const { return last_error_pos_deg_; }
  float commandedRpm() const { return commanded_rpm_; }
  float measuredRpm() const { return velocity_estimator_.getLastRpm(); }
  float measuredRpmRaw() const { return velocity_estimator_.getRawRpm(); }
  float lastVelocityError() const { return commanded_rpm_ - measuredRpmRaw(); }
  int pwmOutput() const { return last_pwm_output_percent_; }
  float accumulatedDeg() const { return current_accumulated_deg_; }
  float estimatedPositionDeg() const { return z1_; }
  float estimatedVelocityDegS() const { return z2_; }
  float estimatedDisturbance() const { return z3_; }

 private:
  static float normalize360(float deg);
  static float shortestDelta(float from_deg, float to_deg);
  void resetObserver(float position_deg, uint32_t now_ms);

  AdrcPositionSettings settings_;
  VelocityEstimator velocity_estimator_{400, 8};

  bool active_ = false;
  bool kicking_ = false;
  bool stalled_ = false;
  bool accumulated_initialized_ = false;
  bool observer_initialized_ = false;

  float target_deg_ = 0.0f;
  MoveDirection direction_ = MoveDirection::Shortest;
  float target_accumulated_deg_ = 0.0f;
  float current_accumulated_deg_ = 0.0f;
  float last_current_deg_normalized_ = 0.0f;
  float profiled_target_deg_ = 0.0f;
  float profile_velocity_deg_s_ = 0.0f;
  float max_speed_rpm_ = 0.0f;
  float last_error_pos_deg_ = 0.0f;
  float commanded_rpm_ = 0.0f;

  float z1_ = 0.0f;
  float z2_ = 0.0f;
  float z3_ = 0.0f;
  float last_output_pwm_ = 0.0f;
  int last_pwm_output_percent_ = 0;

  uint16_t samples_in_window_ = 0;
  uint32_t move_started_ms_ = 0;
  uint32_t last_compute_ms_ = 0;
  uint32_t stall_started_ms_ = 0;
};
