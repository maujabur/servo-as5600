#include "RepetitiveMotionController.h"

RepetitiveMotionController::RepetitiveMotionController(const Commands& commands)
    : commands_(commands) {}

void RepetitiveMotionController::beginMove(float target_deg, float rpm, Phase phase) {
  if (!commands_.start_move) return;
  phase_ = phase;
  commands_.start_move(target_deg, rpm);
}

void RepetitiveMotionController::setRunning(bool running, uint32_t now_ms) {
  if (!running) {
    stop();
    return;
  }
  if (running_) return;

  running_ = true;
  phase_started_ms_ = now_ms;
  // Ao habilitar, sincroniza primeiro no ponto inicial. Assim o primeiro ciclo
  // completo sempre respeita: pausa inicial -> ida -> pausa final -> volta.
  beginMove(config_.start_deg, config_.end_to_start_rpm, Phase::TO_START);
}

void RepetitiveMotionController::stop() {
  running_ = false;
  phase_ = Phase::STOPPED;
  if (commands_.stop_move) commands_.stop_move();
}

void RepetitiveMotionController::update(uint32_t now_ms) {
  if (!running_ || !commands_.is_move_active) return;

  switch (phase_) {
    case Phase::TO_END:
      if (!commands_.is_move_active()) {
        phase_ = Phase::DWELL_AT_END;
        phase_started_ms_ = now_ms;
      }
      break;

    case Phase::DWELL_AT_END:
      if (now_ms - phase_started_ms_ >= config_.dwell_at_end_ms) {
        beginMove(config_.start_deg, config_.end_to_start_rpm, Phase::TO_START);
      }
      break;

    case Phase::TO_START:
      if (!commands_.is_move_active()) {
        phase_ = Phase::DWELL_AT_START;
        phase_started_ms_ = now_ms;
      }
      break;

    case Phase::DWELL_AT_START:
      if (now_ms - phase_started_ms_ >= config_.dwell_at_start_ms) {
        beginMove(config_.end_deg, config_.start_to_end_rpm, Phase::TO_END);
      }
      break;

    case Phase::STOPPED:
      break;
  }
}

const char* RepetitiveMotionController::phaseText() const {
  switch (phase_) {
    case Phase::STOPPED: return "STOPPED";
    case Phase::TO_END: return "TO_END";
    case Phase::DWELL_AT_END: return "DWELL_END";
    case Phase::TO_START: return "TO_START";
    case Phase::DWELL_AT_START: return "DWELL_START";
  }
  return "STOPPED";
}
