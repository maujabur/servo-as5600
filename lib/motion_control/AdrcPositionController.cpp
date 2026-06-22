#include "AdrcPositionController.h"

#include <math.h>

void AdrcPositionController::setSettings(const AdrcPositionSettings& settings) {
  settings_ = settings;
  velocity_estimator_ = VelocityEstimator(settings_.velocity_window_ms,
                                          settings_.velocity_num_samples);
}

void AdrcPositionController::startMove(float target_deg, float max_speed_rpm,
                                       MoveDirection direction) {
  (void)direction; // Esta aplicacao usa exclusivamente o menor caminho 0..360.
  target_deg_ = normalize360(target_deg);
  target_accumulated_deg_ = target_deg;
  max_speed_rpm_ = constrain(max_speed_rpm, 0.1f,
                             fminf(settings_.max_target_rpm,
                                   settings_.physical_max_rpm));
  active_ = true;
  kicking_ = settings_.kick_ms > 0;
  stalled_ = false;
  observer_initialized_ = false;
  samples_in_window_ = 0;
  stall_started_ms_ = 0;
  move_started_ms_ = millis();
  last_compute_ms_ = 0;
  commanded_rpm_ = 0.0f;
  profile_velocity_deg_s_ = 0.0f;
  last_error_pos_deg_ = 0.0f;
  last_output_pwm_ = 0.0f;
  last_pwm_output_percent_ = 0;
  velocity_estimator_.reset();
}

void AdrcPositionController::cancel() {
  active_ = false;
  kicking_ = false;
  stalled_ = false;
  observer_initialized_ = false;
  accumulated_initialized_ = false;
  samples_in_window_ = 0;
  stall_started_ms_ = 0;
  max_speed_rpm_ = 0.0f;
  commanded_rpm_ = 0.0f;
  profile_velocity_deg_s_ = 0.0f;
  last_error_pos_deg_ = 0.0f;
  last_output_pwm_ = 0.0f;
  last_pwm_output_percent_ = 0;
  velocity_estimator_.reset();
}

void AdrcPositionController::primeAccumulatedAngle(float current_deg) {
  const float normalized = normalize360(current_deg);
  current_accumulated_deg_ = normalized;
  last_current_deg_normalized_ = normalized;
  accumulated_initialized_ = true;
  target_accumulated_deg_ = current_accumulated_deg_ +
    shortestDelta(normalized, target_deg_);
  profiled_target_deg_ = current_accumulated_deg_;
  profile_velocity_deg_s_ = 0.0f;
  resetObserver(current_accumulated_deg_, millis());
}

void AdrcPositionController::resetObserver(float position_deg, uint32_t now_ms) {
  z1_ = position_deg;
  z2_ = 0.0f;
  z3_ = 0.0f;
  last_output_pwm_ = 0.0f;
  last_pwm_output_percent_ = 0;
  last_compute_ms_ = now_ms;
  observer_initialized_ = true;
}

float AdrcPositionController::computeOutputPercent(float current_deg,
                                                   uint32_t now_ms) {
  if (!active_ || stalled_) return 0.0f;

  const float normalized = normalize360(current_deg);
  velocity_estimator_.update(normalized, now_ms);

  if (!accumulated_initialized_) {
    primeAccumulatedAngle(normalized);
  } else {
    current_accumulated_deg_ +=
      shortestDelta(last_current_deg_normalized_, normalized);
    last_current_deg_normalized_ = normalized;
  }

  last_error_pos_deg_ = target_accumulated_deg_ - current_accumulated_deg_;
  if (fabsf(last_error_pos_deg_) <= settings_.stop_window_deg) {
    if (++samples_in_window_ >= settings_.samples_to_stop) {
      active_ = false;
      kicking_ = false;
      commanded_rpm_ = 0.0f;
      last_output_pwm_ = 0.0f;
      last_pwm_output_percent_ = 0;
      return 0.0f;
    }
  } else {
    samples_in_window_ = 0;
  }

  const float direction_sign = last_error_pos_deg_ >= 0.0f ? 1.0f : -1.0f;
  if (kicking_) {
    if (now_ms - move_started_ms_ < settings_.kick_ms) {
      float kick_pct = constrain(settings_.kick_pwm_percent, 0.0f, 100.0f);
      if (settings_.physical_max_rpm > 0.1f) {
        const float rpm_limited_kick = constrain(
          (max_speed_rpm_ / settings_.physical_max_rpm) * 135.0f, 8.0f, 100.0f);
        kick_pct = fminf(kick_pct, rpm_limited_kick);
      }
      last_pwm_output_percent_ = (int)roundf(direction_sign * kick_pct);
      last_output_pwm_ = direction_sign * kick_pct * 2.55f;
      return (float)last_pwm_output_percent_;
    }
    kicking_ = false;
    profiled_target_deg_ = current_accumulated_deg_;
    profile_velocity_deg_s_ = 0.0f;
    resetObserver(current_accumulated_deg_, now_ms);
    return 0.0f;
  }

  if (!observer_initialized_) resetObserver(current_accumulated_deg_, now_ms);
  if (last_compute_ms_ == 0) {
    last_compute_ms_ = now_ms;
    return 0.0f;
  }

  const uint32_t dt_ms = now_ms - last_compute_ms_;
  if (dt_ms < 2) return (float)last_pwm_output_percent_;
  last_compute_ms_ = now_ms;
  const float dt = (float)dt_ms / 1000.0f;
  if (dt > 0.05f) {
    resetObserver(current_accumulated_deg_, now_ms);
    return 0.0f;
  }

  // Perfil de referencia: a RPM configurada vira velocidade angular maxima.
  const float max_velocity_deg_s = max_speed_rpm_ * 6.0f;
  const float profile_error = target_accumulated_deg_ - profiled_target_deg_;
  const float accel_s = fmaxf((float)settings_.accel_ramp_ms / 1000.0f, 0.01f);
  const float decel_s = fmaxf((float)settings_.decel_ramp_ms / 1000.0f, 0.01f);
  const float accel_deg_s2 = max_velocity_deg_s / accel_s;
  const float decel_deg_s2 = max_velocity_deg_s / decel_s;
  const float desired_speed = fminf(
    max_velocity_deg_s,
    sqrtf(2.0f * decel_deg_s2 * fabsf(profile_error)));
  const float desired_velocity = profile_error >= 0.0f ? desired_speed : -desired_speed;
  const bool accelerating = fabsf(desired_velocity) > fabsf(profile_velocity_deg_s_);
  const float max_velocity_change =
    (accelerating ? accel_deg_s2 : decel_deg_s2) * dt;
  profile_velocity_deg_s_ += constrain(
    desired_velocity - profile_velocity_deg_s_,
    -max_velocity_change, max_velocity_change);
  float profile_step = profile_velocity_deg_s_ * dt;
  if (fabsf(profile_step) > fabsf(profile_error)) {
    profile_step = profile_error;
    profile_velocity_deg_s_ = 0.0f;
  }
  profiled_target_deg_ += profile_step;
  commanded_rpm_ = profile_velocity_deg_s_ / 6.0f;

  const float wo = fmaxf(settings_.observer_bandwidth, 0.1f);
  const float wc = fmaxf(settings_.control_bandwidth, 0.1f);
  const float b0 = fmaxf(settings_.plant_gain, 0.1f);
  const float beta1 = 3.0f * wo;
  const float beta2 = 3.0f * wo * wo;
  const float beta3 = wo * wo * wo;
  const float observer_error = z1_ - current_accumulated_deg_;

  z1_ += dt * (z2_ - beta1 * observer_error);
  z2_ += dt * (z3_ + b0 * last_output_pwm_ - beta2 * observer_error);
  z3_ += dt * (-beta3 * observer_error);

  const float kp = wc * wc;
  const float kd = 2.0f * wc;
  const float virtual_control =
    kp * (profiled_target_deg_ - z1_) - kd * z2_;
  float output_pwm = constrain((virtual_control - z3_) / b0, -255.0f, 255.0f);
  float output_percent = output_pwm * (100.0f / 255.0f);

  // Preserva o criterio de torque minimo ja ajustado para o motor rated 2 RPM.
  if (fabsf(commanded_rpm_) >= 0.35f &&
      fabsf(velocity_estimator_.getRawRpm()) < 0.08f) {
    const float base_min_pwm = settings_.physical_max_rpm > 0.1f
      ? fabsf(commanded_rpm_) * 100.0f / settings_.physical_max_rpm : 18.0f;
    const float min_pwm = constrain(base_min_pwm + 8.0f, 18.0f, 45.0f);
    if (fabsf(output_percent) < min_pwm) {
      output_percent = commanded_rpm_ >= 0.0f ? min_pwm : -min_pwm;
      output_pwm = output_percent * 2.55f;
    }
  }

  if (fabsf(output_pwm) >= 250.0f &&
      fabsf(z2_) < settings_.stall_velocity_deg_s) {
    if (stall_started_ms_ == 0) stall_started_ms_ = now_ms;
    if (now_ms - stall_started_ms_ >= settings_.stall_timeout_ms) {
      stalled_ = true;
      active_ = false;
      commanded_rpm_ = 0.0f;
      last_output_pwm_ = 0.0f;
      last_pwm_output_percent_ = 0;
      return 0.0f;
    }
  } else {
    stall_started_ms_ = 0;
  }

  output_percent = constrain(output_percent, -100.0f, 100.0f);
  last_output_pwm_ = output_percent * 2.55f;
  last_pwm_output_percent_ = (int)roundf(output_percent);
  return output_percent;
}

float AdrcPositionController::normalize360(float deg) {
  float value = fmodf(deg, 360.0f);
  if (value < 0.0f) value += 360.0f;
  return value;
}

float AdrcPositionController::shortestDelta(float from_deg, float to_deg) {
  float delta = normalize360(to_deg) - normalize360(from_deg);
  while (delta > 180.0f) delta -= 360.0f;
  while (delta < -180.0f) delta += 360.0f;
  return delta;
}
