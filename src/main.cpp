#include <Arduino.h>

#include <As5600Sensor.h>
#include <CascadePositionController.h>

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ESP32-C3 Super Mini + L298N
// IN1/IN2 = motor A,  IN3/IN4 = motor B  (padrao deste projeto)
constexpr uint8_t  MOTOR_A_IN1            = 1;
constexpr uint8_t  MOTOR_A_IN2            = 2;
constexpr uint8_t  MOTOR_B_IN1            = 3;
constexpr uint8_t  MOTOR_B_IN2            = 4;

constexpr uint32_t SERIAL_BAUD            = 115200;
constexpr size_t   SERIAL_LINE_BUFFER     = 96;
constexpr uint32_t CONTROL_PERIOD_MS      = 10;
constexpr uint32_t SERIAL_STARTUP_WAIT_MS = 3000;
constexpr uint32_t MOVE_RPM_TELEMETRY_WINDOW_MS = 80;
constexpr uint32_t MOVE_DEBUG_LOG_PERIOD_MS = 2000;
constexpr uint32_t PWM_DEFAULT_FREQUENCY_HZ = 500;
constexpr uint32_t PWM_MIN_FREQUENCY_HZ     = 500;
constexpr uint32_t PWM_MAX_FREQUENCY_HZ     = 20000;
constexpr uint8_t  PWM_RESOLUTION_BITS      = 8;

constexpr uint8_t  I2C_SDA_PIN            = 5;
constexpr uint8_t  I2C_SCL_PIN            = 6;
constexpr float    DEFAULT_MOVE_MAX_RPM   = 1.8f;
constexpr float    DEFAULT_MAX_TARGET_RPM = 2.4f;
float              g_default_move_max_rpm = DEFAULT_MOVE_MAX_RPM;

// ─── enums ──────────────────────────────────────────────────────────────────

enum class MotorSelection { B_IN3_IN4, A_IN1_IN2, BOTH };

// Fases da maquina de controle
//
//   IDLE    motor parado, saida = 0
//           -> KICK quando |target| > tstop e kick_ms > 0
//           -> RUNNING quando |target| > tstop e kick_ms == 0
//
//   KICK    pulso de partida ativo por kick_ms
//           -> RUNNING ao terminar
//
//   RUNNING rampa normal; dead band remapeado na saida PWM
//           -> IDLE quando |current| cai ate <= tstop
//              (IDLE relanca KICK se target ainda estiver no lado oposto)
//
//   BRAKE   freio eletrico (IN1=IN2=HIGH) por brake_ms
//           -> IDLE ao terminar
//
// Inversao de direcao:
//   Em RUNNING, quando target muda de sinal, ramp_target = 0.
//   A rampa desacelera ate tstop -> IDLE -> KICK no novo sentido.
enum class DrivePhase { IDLE, KICK, RUNNING, BRAKE };

// ─── estado ─────────────────────────────────────────────────────────────────

struct MotorControlState {
  // Selecao de canal
  MotorSelection selected_motor = MotorSelection::B_IN3_IN4;

  // Alvo e valor atual da rampa (assinados, -100..100 %)
  float  target_percent  = 0.0f;
  float  current_percent = 0.0f;
  int8_t direction_sign  = 1;       // usado pelos comandos pwm e dir

  // Rampas
  uint16_t accel_ms    = 250;
  uint16_t decel_ms    = 80;
  float    accel_curve = 1.25f;     // gamma > 1 = mais progressivo no inicio
  float    decel_curve = 1.0f;

  // Dead band
  float threshold_start = 20.0f;   // PWM minimo para arrancar do zero (%)
  float threshold_stop  = 8.0f;    // abaixo disto considera parado (%)

  // Kick de partida / inversao
  float    kick_pct = 85.0f;       // potencia do pulso (%)
  uint16_t kick_ms  = 100;         // duracao (ms);  0 = desabilitado

  // Freio eletrico
  uint16_t brake_ms = 200;         // duracao (ms);  0 = coast imediato

  // Limitador global
  uint8_t power_limit_percent = 100;

  // Interface serial
  bool serial_echo_enabled = true;

  // Estado interno da maquina de fases
  DrivePhase drive_phase    = DrivePhase::IDLE;
  uint32_t   kick_start_ms  = 0;
  int8_t     kick_direction = 1;
  uint32_t   brake_start_ms = 0;

  uint32_t last_control_update_ms = 0;
};

MotorControlState g_state;

char   g_serial_line[SERIAL_LINE_BUFFER];
size_t g_serial_index = 0;
As5600Sensor            g_as5600;
CascadePositionController g_position_servo;
bool                    g_move_done_reported = false;
bool                    g_serial_prompt_pending = true;
bool                    g_move_tracking_active = false;
float                   g_move_peak_rpm_abs = 0.0f;
float                   g_move_peak_rpm_signed = 0.0f;
float                   g_move_last_rpm_signed = 0.0f;
float                   g_move_last_nonzero_rpm_signed = 0.0f;
bool                    g_move_prev_sample_valid = false;
float                   g_move_prev_deg = 0.0f;
uint32_t                g_move_prev_ms = 0;
uint32_t                g_move_start_ms = 0;
float                   g_move_rpm_sum = 0.0f;
uint32_t                g_move_rpm_samples = 0;
float                   g_move_total_abs_delta_deg = 0.0f;
float                   g_move_total_net_delta_deg = 0.0f;
float                   g_move_total_progress_deg = 0.0f;
float                   g_move_start_accumulated_deg = 0.0f;
bool                    g_move_start_accumulated_captured = false;
float                   g_move_rpm_window_delta_deg = 0.0f;
uint32_t                g_move_rpm_window_dt_ms = 0;
int16_t                 g_last_applied_pwm_signed = 0;
int16_t                 g_move_peak_pwm_pid_abs = 0;
int16_t                 g_move_peak_pwm_out_abs = 0;
float                   g_move_pwm_out_abs_sum = 0.0f;
uint32_t                g_move_pwm_out_samples = 0;
uint32_t                g_move_pwm_out_sat_samples = 0;
uint32_t                g_pwm_frequency_hz = PWM_DEFAULT_FREQUENCY_HZ;
uint32_t                g_move_debug_last_print_ms = 0;

// ─── AUTOTUNE STATE ──────────────────────────────────────────────────────
struct AutotuneState {
  bool active = false;
  uint32_t start_ms = 0;
  float initial_deg = 0.0f;
  float target_deg = 0.0f;
  float test_rpm = 0.0f;
  bool has_suggestion = false;
  float pos_kp = 0.0f;
  float pos_ki = 0.0f;
  float pos_kd = 0.0f;
  float vel_kp = 0.0f;
  float vel_ki = 0.0f;
  float vel_kd = 0.0f;
};
AutotuneState g_autotune;

// ─── utilitarios ────────────────────────────────────────────────────────────

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

float signedPercentToLimitedPercent(float pct, uint8_t limit) {
  return clampf(pct * ((float)limit / 100.0f), -100.0f, 100.0f);
}

uint8_t percentToPwm(float pct_abs) {
  return (uint8_t)roundf(clampf(pct_abs * 255.0f / 100.0f, 0.0f, 255.0f));
}

// Remapeia [tstop..100] -> [tstart..100].
// Abaixo de tstop retorna 0 (dead band: motor nao recebe tensao).
float applyDeadBandRemap(float abs_pct, float tstart, float tstop) {
  if (abs_pct <= tstop) return 0.0f;
  return tstart + (abs_pct - tstop) * (100.0f - tstart) / (100.0f - tstop);
}

float shortestAngleDeltaDeg(float from_deg, float to_deg) {
  float delta = to_deg - from_deg;
  while (delta > 180.0f) delta -= 360.0f;
  while (delta < -180.0f) delta += 360.0f;
  return delta;
}

bool setPwmFrequencyHz(uint32_t hz) {
  if (hz < PWM_MIN_FREQUENCY_HZ || hz > PWM_MAX_FREQUENCY_HZ) {
    return false;
  }
  analogWriteFrequency(hz);
  g_pwm_frequency_hz = hz;
  return true;
}

void updatePositionMoveControl() {
  if (!g_as5600.detected()) {
    g_move_tracking_active = false;
    g_move_prev_sample_valid = false;
    g_move_total_abs_delta_deg = 0.0f;
    g_move_total_net_delta_deg = 0.0f;
    g_move_total_progress_deg = 0.0f;
    g_move_start_accumulated_captured = false;
    g_move_rpm_window_delta_deg = 0.0f;
    g_move_rpm_window_dt_ms = 0;
    g_move_debug_last_print_ms = 0;
    return;
  }

  if (!g_position_servo.isActive()) {
    g_move_tracking_active = false;
    g_move_prev_sample_valid = false;
    g_move_total_abs_delta_deg = 0.0f;
    g_move_total_net_delta_deg = 0.0f;
    g_move_total_progress_deg = 0.0f;
    g_move_start_accumulated_captured = false;
    g_move_rpm_window_delta_deg = 0.0f;
    g_move_rpm_window_dt_ms = 0;
    g_move_debug_last_print_ms = 0;
    return;
  }

  if (!g_move_tracking_active) {
    g_move_tracking_active = true;
    g_move_peak_rpm_abs = 0.0f;
    g_move_peak_rpm_signed = 0.0f;
    g_move_last_rpm_signed = 0.0f;
    g_move_last_nonzero_rpm_signed = 0.0f;
    g_move_rpm_sum = 0.0f;
    g_move_rpm_samples = 0;
    g_move_total_abs_delta_deg = 0.0f;
    g_move_total_net_delta_deg = 0.0f;
    g_move_total_progress_deg = 0.0f;
    g_move_start_accumulated_captured = false;
    g_move_rpm_window_delta_deg = 0.0f;
    g_move_rpm_window_dt_ms = 0;
    g_move_peak_pwm_pid_abs = 0;
    g_move_peak_pwm_out_abs = 0;
    g_move_pwm_out_abs_sum = 0.0f;
    g_move_pwm_out_samples = 0;
    g_move_pwm_out_sat_samples = 0;
    g_move_prev_sample_valid = false;
    g_move_debug_last_print_ms = millis();
  }

  float current_deg = 0.0f;
  if (!g_as5600.readAngleDeg(&current_deg)) return;

  const uint32_t now_ms = millis();

  // Acumula deslocamento angular absoluto para diagnostico de movimentos curtos.
  if (g_move_prev_sample_valid) {
    const uint32_t dt_ms = now_ms - g_move_prev_ms;
    if (dt_ms > 0) {
      const float delta_deg = shortestAngleDeltaDeg(g_move_prev_deg, current_deg);
      // Para rpm_medio, considera deslocamento quase integral; rejeita apenas glitches grosseiros.
      if (fabsf(delta_deg) <= 20.0f) {
        g_move_total_abs_delta_deg += fabsf(delta_deg);
        g_move_total_net_delta_deg += delta_deg;
        const float cmd_rpm = g_position_servo.commandedRpm();
        const float sign = (cmd_rpm >= 0.0f) ? 1.0f : -1.0f;
        const float projected_progress = sign * delta_deg;
        if (projected_progress > 0.0f) {
          g_move_total_progress_deg += projected_progress;
        }
      }

      // Nao computa rpm_pico durante KICK para evitar inflar metricas de regime.
      if (!g_position_servo.isKicking()) {
        const CascadePositionSettings& cfg = g_position_servo.settings();
        // Para rpm_pico, usa filtro dinamico mais estrito.
        float max_delta_plausible_deg = 3.0f;
        if (cfg.physical_max_rpm > 0.1f) {
          const float rpm_guard = cfg.physical_max_rpm * 1.8f;
          max_delta_plausible_deg = (rpm_guard * 360.0f * (float)dt_ms) / 60000.0f + 0.8f;
        }
        // Pico de RPM por janela temporal para reduzir efeito de quantizacao.
        if (fabsf(delta_deg) <= max_delta_plausible_deg) {
          g_move_rpm_window_delta_deg += delta_deg;
          g_move_rpm_window_dt_ms += dt_ms;
        }
        if (g_move_rpm_window_dt_ms >= MOVE_RPM_TELEMETRY_WINDOW_MS) {
          const float rpm_window =
            (g_move_rpm_window_delta_deg * 60000.0f) / (360.0f * (float)g_move_rpm_window_dt_ms);
          const float abs_rpm_window = fabsf(rpm_window);
          float rpm_peak_limit = fmaxf(g_position_servo.maxSpeedRpm() * 2.20f, 1.0f);
          if (cfg.physical_max_rpm > 0.1f) {
            rpm_peak_limit = fmaxf(rpm_peak_limit, cfg.physical_max_rpm * 1.40f + 0.20f);
          }
          rpm_peak_limit = fminf(rpm_peak_limit, 30.0f);

          if (abs_rpm_window <= rpm_peak_limit) {
            if (abs_rpm_window > g_move_peak_rpm_abs) {
              g_move_peak_rpm_abs = abs_rpm_window;
              g_move_peak_rpm_signed = rpm_window;
            }
            if (abs_rpm_window > 0.2f) {
              g_move_last_nonzero_rpm_signed = rpm_window;
            }
          }

          g_move_rpm_window_delta_deg = 0.0f;
          g_move_rpm_window_dt_ms = 0;
        }
      } else {
        // Reinicia a janela para nao carregar amostras de kick para o pico de regime.
        g_move_rpm_window_delta_deg = 0.0f;
        g_move_rpm_window_dt_ms = 0;
      }
    }
  }
  g_move_prev_deg = current_deg;
  g_move_prev_ms = now_ms;
  g_move_prev_sample_valid = true;

  const bool was_active = g_position_servo.isActive();
  const float command_pct = g_position_servo.computeOutputPercent(current_deg, now_ms);
  const int16_t pwm_pid_abs = (int16_t)abs(g_position_servo.pwmOutput());
  if (pwm_pid_abs > g_move_peak_pwm_pid_abs) {
    g_move_peak_pwm_pid_abs = pwm_pid_abs;
  }

  // Captura posição acumulada inicial (odômetro) na primeira iteração do movimento.
  if (!g_move_start_accumulated_captured && g_position_servo.isActive()) {
    g_move_start_accumulated_deg = g_position_servo.accumulatedDeg();
    g_move_start_accumulated_captured = true;
  }

  if (g_position_servo.isActive()) {
    // Mantem RPM instantaneo para debug/diagnostico.
    g_move_last_rpm_signed = g_position_servo.measuredRpm();

    // Debug periodico sem liberar entrada serial: status reduzido a cada 10s.
    if (g_move_debug_last_print_ms == 0 || (now_ms - g_move_debug_last_print_ms) >= MOVE_DEBUG_LOG_PERIOD_MS) {
      g_move_debug_last_print_ms = now_ms;
      Serial.printf("DBG: move t=%u ms  rpm_inst=%.2f  rpm_raw=%.2f  rpm_cmd=%.2f  erro_rpm=%.2f  pwm=%d%%\n",
                    now_ms - g_move_start_ms,
                    g_position_servo.measuredRpm(),
                    g_position_servo.measuredRpmRaw(),
                    g_position_servo.commandedRpm(),
                    g_position_servo.lastVelocityError(),
                    g_position_servo.pwmOutput());
    }

    const float pwm_out_abs = (float)abs(g_last_applied_pwm_signed);
    g_move_pwm_out_abs_sum += pwm_out_abs;
    g_move_pwm_out_samples++;
    if (pwm_out_abs >= 250.0f) {
      g_move_pwm_out_sat_samples++;
    }
  }
  g_state.target_percent = command_pct;
  if (command_pct > 0.01f) g_state.direction_sign = 1;
  if (command_pct < -0.01f) g_state.direction_sign = -1;

  if (was_active && !g_position_servo.isActive() && !g_move_done_reported) {
    g_move_done_reported = true;
    g_move_tracking_active = false;
    g_move_prev_sample_valid = false;
    g_move_debug_last_print_ms = 0;
    g_serial_prompt_pending = true;
    const uint32_t elapsed_ms = millis() - g_move_start_ms;
    // rpm_medio calculado a partir do odômetro interno do controlador (mais preciso
    // que acumulação de deltas externos, que perde progressão durante fase STOPPING).
    const float total_displacement_deg = g_move_start_accumulated_captured
                                           ? fabsf(g_position_servo.accumulatedDeg() - g_move_start_accumulated_deg)
                                           : fabsf(g_move_total_net_delta_deg);
    const float rpm_medio = (elapsed_ms > 0)
                              ? (total_displacement_deg * 60000.0f) / (360.0f * (float)elapsed_ms)
                              : 0.0f;
    if (g_move_peak_rpm_abs < 0.05f && rpm_medio > 0.05f) {
      const float peak_sign = (g_move_last_nonzero_rpm_signed < 0.0f) ? -1.0f : 1.0f;
      g_move_peak_rpm_abs = rpm_medio;
      g_move_peak_rpm_signed = peak_sign * rpm_medio;
    }
    const float pwm_pid_pico_pct = clampf((float)g_move_peak_pwm_pid_abs, 0.0f, 100.0f);
    const float pwm_out_pico_pct = ((float)g_move_peak_pwm_out_abs * 100.0f) / 255.0f;
    const float pwm_out_medio_pct = (g_move_pwm_out_samples > 0)
                                       ? ((g_move_pwm_out_abs_sum / (float)g_move_pwm_out_samples) * 100.0f / 255.0f)
                                       : 0.0f;
    const float pwm_out_sat_pct = (g_move_pwm_out_samples > 0)
                                     ? ((float)g_move_pwm_out_sat_samples * 100.0f / (float)g_move_pwm_out_samples)
                                     : 0.0f;
    const float ang_ini = g_move_start_accumulated_captured ? g_move_start_accumulated_deg : 0.0f;
    const float desl_signed = g_move_start_accumulated_captured
                                ? (g_position_servo.accumulatedDeg() - g_move_start_accumulated_deg)
                                : g_move_total_net_delta_deg;
    const float desl_abs = fabsf(desl_signed);
    Serial.printf("OK: alvo atingido em %.2f deg (ang_ini=%.2f deg, desl=%.2f deg, desl_abs=%.2f deg, tempo=%u ms, rpm_pico=%.2f, rpm_medio=%.2f, pwm_pid_pico=%.1f%%, pwm_out_pico=%.1f%%, pwm_out_med=%.1f%%, pwm_sat=%.0f%%)\n",
                  current_deg, ang_ini, desl_signed, desl_abs, elapsed_ms, g_move_peak_rpm_signed, rpm_medio,
                  pwm_pid_pico_pct, pwm_out_pico_pct, pwm_out_medio_pct, pwm_out_sat_pct);
  }
}

// ─── saida para os pinos ────────────────────────────────────────────────────

void setMotorSignedPwm(int16_t signed_pwm, uint8_t in1, uint8_t in2) {
  const int16_t v = (int16_t)abs(signed_pwm);
  if (v == 0) {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  } else if (signed_pwm > 0) {
    analogWrite(in1, v);
    analogWrite(in2, 0);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, v);
  }
}

// L298N: IN1=IN2=HIGH -> curto-circuito na bobina -> freio regenerativo
void setBrakeChannelPins(uint8_t in1, uint8_t in2) {
  analogWrite(in1, 255);
  analogWrite(in2, 255);
}

void applyMotorOutput(int16_t signed_pwm) {
  g_last_applied_pwm_signed = signed_pwm;
  if (g_move_tracking_active) {
    const int16_t pwm_out_abs = (int16_t)abs(signed_pwm);
    if (pwm_out_abs > g_move_peak_pwm_out_abs) {
      g_move_peak_pwm_out_abs = pwm_out_abs;
    }
  }
  switch (g_state.selected_motor) {
    case MotorSelection::B_IN3_IN4:
      setMotorSignedPwm(signed_pwm, MOTOR_B_IN1, MOTOR_B_IN2);
      setMotorSignedPwm(0,          MOTOR_A_IN1, MOTOR_A_IN2);
      break;
    case MotorSelection::A_IN1_IN2:
      setMotorSignedPwm(signed_pwm, MOTOR_A_IN1, MOTOR_A_IN2);
      setMotorSignedPwm(0,          MOTOR_B_IN1, MOTOR_B_IN2);
      break;
    case MotorSelection::BOTH:
      setMotorSignedPwm(signed_pwm, MOTOR_A_IN1, MOTOR_A_IN2);
      setMotorSignedPwm(signed_pwm, MOTOR_B_IN1, MOTOR_B_IN2);
      break;
  }
}

void applyBrakeOutput() {
  switch (g_state.selected_motor) {
    case MotorSelection::B_IN3_IN4:
      setBrakeChannelPins(MOTOR_B_IN1, MOTOR_B_IN2);
      setMotorSignedPwm(0, MOTOR_A_IN1, MOTOR_A_IN2);
      break;
    case MotorSelection::A_IN1_IN2:
      setBrakeChannelPins(MOTOR_A_IN1, MOTOR_A_IN2);
      setMotorSignedPwm(0, MOTOR_B_IN1, MOTOR_B_IN2);
      break;
    case MotorSelection::BOTH:
      setBrakeChannelPins(MOTOR_A_IN1, MOTOR_A_IN2);
      setBrakeChannelPins(MOTOR_B_IN1, MOTOR_B_IN2);
      break;
  }
}

// ─── texto de estado ────────────────────────────────────────────────────────

const char* selectedMotorText() {
  switch (g_state.selected_motor) {
    case MotorSelection::B_IN3_IN4: return "IN3/IN4";
    case MotorSelection::A_IN1_IN2: return "IN1/IN2";
    case MotorSelection::BOTH:      return "BOTH";
  }
  return "IN3/IN4";
}

const char* drivePhaseText() {
  switch (g_state.drive_phase) {
    case DrivePhase::IDLE:    return "IDLE";
    case DrivePhase::KICK:    return "KICK";
    case DrivePhase::RUNNING: return "RUNNING";
    case DrivePhase::BRAKE:   return "BRAKE";
  }
  return "IDLE";
}

// ─── selecao de motor ───────────────────────────────────────────────────────

void selectMotor(MotorSelection motor) {
  g_state.selected_motor = motor;
}

bool parseMotorToken(const char* val, MotorSelection* out) {
  if (!val || !out) return false;
  if (!strcmp(val,"in3") || !strcmp(val,"in3/in4") || !strcmp(val,"b")) {
    *out = MotorSelection::B_IN3_IN4; return true;
  }
  if (!strcmp(val,"in1") || !strcmp(val,"in1/in2") || !strcmp(val,"a")) {
    *out = MotorSelection::A_IN1_IN2; return true;
  }
  if (!strcmp(val,"both") || !strcmp(val,"all")) {
    *out = MotorSelection::BOTH; return true;
  }
  return false;
}

void printMotorOk() {
  switch (g_state.selected_motor) {
    case MotorSelection::B_IN3_IN4: Serial.println("OK: motor IN3/IN4"); break;
    case MotorSelection::A_IN1_IN2: Serial.println("OK: motor IN1/IN2"); break;
    case MotorSelection::BOTH:      Serial.println("OK: motor BOTH");    break;
  }
}

// ─── interface serial ────────────────────────────────────────────────────────

void printPrompt() { Serial.print("> "); }
void printErrorAndPrompt(const char* message) {
  Serial.println(message);
  g_serial_prompt_pending = true;
}

// Comandos com mais de 1 char que NAO devem sofrer expansao compacta.
// Ex: "kp" comeca com 'k'; sem protecao viraria cmd="k" value="p".
bool isLongCommand(const char* cmd) {
  const char* longs[] = {
    "help","status","stop","pwm","set","rev","dir",
    "pwmf","pf",
    "motor","accel","decel","curve","limit","echo",
    "motora","motorb","motorboth",
    "tstart","tstop","kick","kickp","brake","brakems",
    "mm","kp","bm",
    "goto","go","move","cancelmove","qc",
    "cw","ccw",
    "inc","dec",
    "autotune","aut","at",
    "autotune_apply","autapply","ata",
    "pidpos_kp","pp","pidpos_ki","pi","pidpos_kd","pd",
    "pidvel_kp","vp","pidvel_ki","vi","pidvel_kd","vd",
    "poswin","pw","accel_pos","ap","decel_pos","dp",
    "kick_pwm","kpp","kick_rpm","krp","kick_ms","kms","samples_to_stop","sts",
    "maxrpm","mr","physrpm","pr",
    "drpm","default_rpm",
    nullptr
  };
  for (int i = 0; longs[i]; ++i) {
    if (!strcmp(cmd, longs[i])) return true;
  }
  return false;
}

void printStatus() {
  Serial.println("\n=== STATUS ===");
  Serial.printf("Motor: %-8s  Fase: %s\n",
                selectedMotorText(), drivePhaseText());
  Serial.printf("Target: %6.1f%%  Atual: %6.1f%%  Dir: %s\n",
                g_state.target_percent,
                g_state.current_percent,
                g_state.direction_sign > 0 ? "FWD" : "REV");
  Serial.println();
  Serial.printf("Rampas:  acc=%ums  dec=%ums\n",
                g_state.accel_ms, g_state.decel_ms);
  Serial.printf("Curvas:  acc=%.2f  dec=%.2f\n",
                g_state.accel_curve, g_state.decel_curve);
  Serial.println();
  Serial.printf("DeadBand: start=%.0f%%  stop=%.0f%%\n",
                g_state.threshold_start, g_state.threshold_stop);
  Serial.printf("Kick:     %.0f%% / %ums%s\n",
                g_state.kick_pct, g_state.kick_ms,
                g_state.kick_ms == 0 ? "  (desabilitado)" : "");
  Serial.printf("Freio:    %ums%s\n",
                g_state.brake_ms,
                g_state.brake_ms == 0 ? "  (coast)" : "");
  Serial.println();
  Serial.printf("Limite: %u%%   Echo: %s\n",
                g_state.power_limit_percent,
                g_state.serial_echo_enabled ? "ON" : "OFF");
  Serial.printf("PWM: freq=%u Hz  resolucao=%u bits\n",
                g_pwm_frequency_hz,
                PWM_RESOLUTION_BITS);
  Serial.printf("AS5600: %s (0x%02X)\n",
                g_as5600.detected() ? "detectado" : "nao detectado",
                g_as5600.address());

  if (g_position_servo.isActive()) {
    Serial.printf("Move: ON  alvo=%.2f deg  vmax=%.2f rpm  erro_pos=%.2f deg\n",
                  g_position_servo.targetDeg(),
                  g_position_servo.maxSpeedRpm(),
                  g_position_servo.lastErrorDeg());
    Serial.printf("  rpm_cmd=%.2f  rpm_real=%.2f  erro_rpm=%.2f  pwm=%d%%\n",
                  g_position_servo.commandedRpm(),
                  g_position_servo.measuredRpm(),
                  g_position_servo.lastVelocityError(),
                  g_position_servo.pwmOutput());
  } else {
    Serial.println("Move: OFF");
  }

  const CascadePositionSettings& s = g_position_servo.settings();
  Serial.printf("PosCtrl: rated=%.2f rpm  phys=%.2f rpm  stop_win=%.2f deg\n",
                s.max_target_rpm, s.physical_max_rpm, s.stop_window_deg);
  Serial.printf("PID_Pos: kp=%.3f ki=%.3f kd=%.3f\n",
                s.pos_pid.kp, s.pos_pid.ki, s.pos_pid.kd);
  Serial.printf("PID_Vel: kp=%.3f ki=%.3f kd=%.3f\n",
                s.vel_pid.kp, s.vel_pid.ki, s.vel_pid.kd);
  Serial.printf("Rampa: accel=%ums  decel=%ums\n", s.accel_ramp_ms, s.decel_ramp_ms);
  Serial.printf("Kick(pos): %.1f%% pwm / %ums   Histerese: %u samples\n",
                s.kick_pwm_percent, s.kick_ms, s.samples_to_stop);
  Serial.printf("Move default: %.2f rpm\n", g_default_move_max_rpm);
}

void printHelp() {
  Serial.println("\n  Ajuda / estado:");
  Serial.println("    help | h | ?              -> ajuda");
  Serial.println("    status | s                -> estado atual");
  Serial.println();
  Serial.println("  Controle:");
  Serial.println("    pwm | p <0..100>          -> PWM % usando direcao atual");
  Serial.println("    pwmf| pf [1000..20000]    -> freq PWM (DRV8733: 3k..12k tipico)");
  Serial.println("    set | v <-100..100>       -> velocidade assinada");
  Serial.println("    stop | x                  -> corta saida imediatamente");
  Serial.println("    brake | b                 -> freio eletrico imediato");
  Serial.println("    rev | r                   -> inverte direcao");
  Serial.println("    dir | d f|r               -> define direcao manual");
  Serial.println();
  Serial.println("  Canal do motor:");
  Serial.println("    motor | m in3|in1|both");
  Serial.println("    ma | motora               -> IN1/IN2");
  Serial.println("    mb | motorb               -> IN3/IN4 (padrao)");
  Serial.println("    mm | motorboth            -> ambos");
  Serial.println();
  Serial.println("  Rampas:");
  Serial.println("    accel | a <ms>            -> tempo aceleracao      [250ms]");
  Serial.println("    decel | z <ms>            -> tempo desaceleracao   [80ms]");
  Serial.println("    curve | c <acc> <dec>     -> curva gamma           [1.25 / 1.0]");
  Serial.println();
  Serial.println("  Dead band / kick / freio / limite:");
  Serial.println("    tstart | ts <0..50>       -> PWM minimo de partida [20%]");
  Serial.println("    tstop  | tp <0..50>       -> threshold de parada   [8%]");
  Serial.println("    kick   | k  <ms>          -> duracao kick (0=off)  [100ms]");
  Serial.println("    kickp  | kp <0..100>      -> potencia kick         [85%]");
  Serial.println("    brakems| bm <ms>          -> duracao freio (0=coast)[200ms]");
  Serial.println("    limit  | l  <0..100>      -> limite de potencia    [100%]");
  Serial.println();
  Serial.println("  Interface:");
  Serial.println("    echo | e on|off           -> eco serial");
  Serial.println("    g                         -> le posicao AS5600");
  Serial.println();
  Serial.println("  Posicionamento com cascata PID (servo real):");
  Serial.println("    q | goto <deg> [rpm] [short|cw|ccw]");
  Serial.println("                              -> vai ao alvo (0..360), rpm max e sentido opcionais");
  Serial.println("    cw | ccw <deg> [rpm]      -> atalhos com sentido fixo");
  Serial.println("    inc <ddeg> [rpm]          -> move incremental CW (negativo=CCW, ex.: inc 90 1.8)");
  Serial.println("    dec <ddeg> [rpm]          -> move incremental CCW (negativo=CW, ex.: dec 90 1.8)");
  Serial.println("                              -> suporta multiplas voltas: inc 450 1.8 (1 volta + 90 graus)");
  Serial.println("    qc | cancelmove           -> cancela movimento de posicao");
  Serial.println("    autotune | aut [step] [rpm] -> sugestao PID pos+vel (padrao: 30 deg, 1.8 rpm)");
  Serial.println("    autotune_apply | ata      -> aplica ultima sugestao de PID pos+vel");
  Serial.println();
  Serial.println("  PID de posicao (controla RPM desejado a partir do erro):");
  Serial.println("    pp | pidpos_kp <kp>       -> ganho proporcional [0.75]");
  Serial.println("    pi | pidpos_ki <ki>       -> ganho integral [0.04]");
  Serial.println("    pd | pidpos_kd <kd>       -> ganho derivativo [0.04]");
  Serial.println();
  Serial.println("  PID de velocidade (controla PWM para manter RPM real):");
  Serial.println("    vp | pidvel_kp <kp>       -> ganho proporcional [8.0]");
  Serial.println("    vi | pidvel_ki <ki>       -> ganho integral [1.2]");
  Serial.println("    vd | pidvel_kd <kd>       -> ganho derivativo [0.08]");
  Serial.println();
  Serial.println("  Configuracao de movimento:");
  Serial.println("    pw | poswin <deg>         -> janela de chegada [1.2]");
  Serial.println("    ap | accel_pos <ms>       -> tempo aceleracao [250ms]");
  Serial.println("    dp | decel_pos <ms>       -> tempo desaceleracao [220ms]");
  Serial.println("    kpp | kick_pwm <pct>      -> kick do posicionamento em PWM% [85]");
  Serial.println("    kms | kick_ms <ms>        -> duracao kick [180ms]");
  Serial.println("    sts | samples_to_stop <n> -> leituras para parar [3]");
  Serial.println("    drpm| default_rpm <rpm>   -> RPM padrao para q/inc/dec/autotune [1.8]");
  Serial.println("    mr  | maxrpm <rpm>        -> limite max de RPM do servo [2.4]");
  Serial.println("    pr  | physrpm <rpm>       -> limite fisico estimado do motor [2.0]");
  Serial.println();
  Serial.println("  Forma compacta (letra+valor, sem espaco):");
  Serial.println("    p35  v-40  a250  z80  l60  k100  df  ts20  eon");
}

// ─── alvo ───────────────────────────────────────────────────────────────────

void setTargetFromUnsignedPercent(float pct) {
  g_state.target_percent =
    clampf(pct, 0.0f, 100.0f) * (float)g_state.direction_sign;
}

// ─── parser de comandos ──────────────────────────────────────────────────────

void parseAndHandleCommand(char* line) {
  if (!line || !line[0]) return;

  for (size_t i = 0; line[i]; ++i)
    line[i] = (char)tolower((unsigned char)line[i]);

  char* ctx = nullptr;
  char* cmd = strtok_r(line, " \t", &ctx);
  if (!cmd) return;

  char inline_value[SERIAL_LINE_BUFFER] = {0};

  // Forma compacta: p35 -> cmd="p", inline_value="35"
  if (strlen(cmd) > 1 && !isLongCommand(cmd)) {
    if (strchr("psrvdmazclekx?", cmd[0])) {
      memcpy(inline_value, cmd + 1, sizeof(inline_value) - 1);
      cmd[1] = '\0';
    }
  }

  auto nextArg = [&]() -> char* {
    return inline_value[0] ? inline_value : strtok_r(nullptr, " \t", &ctx);
  };

  auto parseMoveDirection = [](const char* token, CascadePositionController::MoveDirection* direction) -> bool {
    if (!token || !direction) return false;
    if (!strcmp(token, "short") || !strcmp(token, "shortest") || !strcmp(token, "nearest")) {
      *direction = CascadePositionController::MoveDirection::Shortest;
      return true;
    }
    if (!strcmp(token, "cw") || !strcmp(token, "horario") || !strcmp(token, "cw+")) {
      *direction = CascadePositionController::MoveDirection::Clockwise;
      return true;
    }
    if (!strcmp(token, "ccw") || !strcmp(token, "anti") || !strcmp(token, "antihorario") || !strcmp(token, "ccw-")) {
      *direction = CascadePositionController::MoveDirection::CounterClockwise;
      return true;
    }
    return false;
  };

  auto parseFloatToken = [](const char* token, float* value) -> bool {
    if (!token || !value) return false;
    char* end = nullptr;
    const float parsed = strtof(token, &end);
    if (end == token || (end && *end != '\0')) return false;
    *value = parsed;
    return true;
  };

  // ── ajuda / estado ──────────────────────────────────────────────────────

  if (!strcmp(cmd,"help") || !strcmp(cmd,"h") || !strcmp(cmd,"?")) {
    printHelp(); return;
  }
  if (!strcmp(cmd,"status") || !strcmp(cmd,"s")) {
    printStatus(); return;
  }

  if (!strcmp(cmd,"g")) {
    if (!g_as5600.detected()) {
      Serial.println("ERRO: AS5600 nao detectado no I2C");
      return;
    }

    uint16_t raw_angle = 0;
    if (!g_as5600.readRawAngle(&raw_angle)) {
      Serial.println("ERRO: falha ao ler AS5600");
      return;
    }

    const float degrees = ((float)raw_angle * 360.0f) / 4096.0f;
    Serial.printf("AS5600: raw=%u  deg=%.2f\n", raw_angle, degrees);
    return;
  }

  if (!strcmp(cmd,"autotune") || !strcmp(cmd,"aut") || !strcmp(cmd,"at")) {
    if (!g_as5600.detected()) {
      printErrorAndPrompt("ERRO: AS5600 nao detectado no I2C");
      return;
    }
    if (g_position_servo.isActive()) {
      printErrorAndPrompt("ERRO: aguarde o movimento atual terminar");
      return;
    }
    if (g_autotune.active) {
      printErrorAndPrompt("ERRO: autotune ja em andamento");
      return;
    }

    char* step_token = nextArg();
    char* rpm_token  = strtok_r(nullptr, " \t", &ctx);
    float step_deg = 30.0f;
    float rpm = g_default_move_max_rpm;

    if (step_token) step_deg = clampf((float)atof(step_token), -180.0f, 180.0f);
    if (rpm_token)  rpm      = clampf((float)atof(rpm_token), 1.0f, g_position_servo.settings().max_target_rpm);

    if (fabsf(step_deg) < 1.0f) {
      printErrorAndPrompt("ERRO: step deve ter modulo >= 1.0 deg");
      return;
    }

    float current_deg = 0.0f;
    if (!g_as5600.readAngleDeg(&current_deg)) {
      printErrorAndPrompt("ERRO: falha ao ler AS5600");
      return;
    }

    auto wrap360 = [](float deg) {
      while (deg < 0.0f) deg += 360.0f;
      while (deg >= 360.0f) deg -= 360.0f;
      return deg;
      
    };

    g_autotune.initial_deg = current_deg;
    g_autotune.target_deg = wrap360(current_deg + step_deg);
    g_autotune.test_rpm = rpm;
    g_autotune.has_suggestion = false;
    g_autotune.start_ms = millis();
    g_autotune.active = true;

    Serial.printf("AUTOTUNE: armado (ini=%.2f deg, alvo=%.2f deg, rpm=%.2f)\n",
                  g_autotune.initial_deg, g_autotune.target_deg, rpm);
    return;
  }

  if (!strcmp(cmd,"autotune_apply") || !strcmp(cmd,"autapply") || !strcmp(cmd,"ata")) {
    if (g_autotune.active) {
      printErrorAndPrompt("ERRO: aguarde o autotune terminar");
      return;
    }
    if (!g_autotune.has_suggestion) {
      printErrorAndPrompt("ERRO: sem sugestao. Execute autotune primeiro");
      return;
    }

    CascadePositionSettings s = g_position_servo.settings();
    s.pos_pid.kp = clampf(g_autotune.pos_kp, 0.01f, 10.0f);
    s.pos_pid.ki = clampf(g_autotune.pos_ki, 0.0f, 5.0f);
    s.pos_pid.kd = clampf(g_autotune.pos_kd, 0.0f, 5.0f);
    s.vel_pid.kp = clampf(g_autotune.vel_kp, 0.1f, 50.0f);
    s.vel_pid.ki = clampf(g_autotune.vel_ki, 0.0f, 20.0f);
    s.vel_pid.kd = clampf(g_autotune.vel_kd, 0.0f, 10.0f);
    g_position_servo.setSettings(s);

    Serial.println("AUTOTUNE: sugestao aplicada");
    Serial.printf("  POS: kp=%.3f ki=%.3f kd=%.3f\n", s.pos_pid.kp, s.pos_pid.ki, s.pos_pid.kd);
    Serial.printf("  VEL: kp=%.3f ki=%.3f kd=%.3f\n", s.vel_pid.kp, s.vel_pid.ki, s.vel_pid.kd);
    return;
  }

  if (!strcmp(cmd,"q") || !strcmp(cmd,"goto") || !strcmp(cmd,"go")
      || !strcmp(cmd,"move") || !strcmp(cmd,"cw") || !strcmp(cmd,"ccw")) {
    if (!g_as5600.detected()) {
      printErrorAndPrompt("ERRO: AS5600 nao detectado no I2C");
      return;
    }

    char* deg_token = nextArg();
    char* extra1 = strtok_r(nullptr, " \t", &ctx);
    char* extra2 = strtok_r(nullptr, " \t", &ctx);
    if (!deg_token) {
      printErrorAndPrompt("ERRO: use q <deg> [rpm] [short|cw|ccw]");
      return;
    }

    const float target_deg = (float)atof(deg_token);
    const CascadePositionSettings& s = g_position_servo.settings();
    float vmax_rpm = g_default_move_max_rpm;
    CascadePositionController::MoveDirection direction = CascadePositionController::MoveDirection::Shortest;

    if (!strcmp(cmd, "cw")) direction = CascadePositionController::MoveDirection::Clockwise;
    if (!strcmp(cmd, "ccw")) direction = CascadePositionController::MoveDirection::CounterClockwise;

    auto consumeToken = [&](char* token) {
      if (!token) return;
      float parsed_value = 0.0f;
      if (parseMoveDirection(token, &direction)) return;
      if (parseFloatToken(token, &parsed_value)) vmax_rpm = parsed_value;
    };

    consumeToken(extra1);
    consumeToken(extra2);
    vmax_rpm = clampf(vmax_rpm, 0.0f, s.max_target_rpm);

    float current_deg = 0.0f;
    if (!g_as5600.readAngleDeg(&current_deg)) {
      printErrorAndPrompt("ERRO: falha ao ler AS5600");
      return;
    }

    g_position_servo.startMove(target_deg, vmax_rpm, direction);
    g_position_servo.primeAccumulatedAngle(current_deg);
    g_move_done_reported = false;
    g_move_start_ms = millis();
    const char* direction_text =
      direction == CascadePositionController::MoveDirection::Clockwise ? "cw" :
      direction == CascadePositionController::MoveDirection::CounterClockwise ? "ccw" : "short";
    Serial.printf("OK: move alvo=%.2f deg  vmax=%.2f rpm  dir=%s\n", target_deg, vmax_rpm, direction_text);
    return;
  }

  if (!strcmp(cmd,"inc")) {
    if (!g_as5600.detected()) {
      printErrorAndPrompt("ERRO: AS5600 nao detectado no I2C");
      return;
    }

    char* delta_token = nextArg();
    char* rpm_token = strtok_r(nullptr, " \t", &ctx);
    if (!delta_token) {
      printErrorAndPrompt("ERRO: use inc <ddeg> [rpm]");
      return;
    }

    float delta_deg = (float)atof(delta_token);
    const CascadePositionSettings& s = g_position_servo.settings();
    float vmax_rpm = g_default_move_max_rpm;

    if (rpm_token) {
      float parsed_rpm = 0.0f;
      if (parseFloatToken(rpm_token, &parsed_rpm)) {
        vmax_rpm = parsed_rpm;
      }
    }
    vmax_rpm = clampf(vmax_rpm, 0.0f, s.max_target_rpm);

    float current_deg = 0.0f;
    if (!g_as5600.readAngleDeg(&current_deg)) {
      printErrorAndPrompt("ERRO: falha ao ler AS5600");
      return;
    }

    // Delta positivo = CW (Clockwise), negativo = CCW (CounterClockwise)
    const float target_accumulated_deg = current_deg + delta_deg;
    const CascadePositionController::MoveDirection direction =
      delta_deg >= 0.0f ? CascadePositionController::MoveDirection::Clockwise
                        : CascadePositionController::MoveDirection::CounterClockwise;

    g_position_servo.startMove(target_accumulated_deg, vmax_rpm, direction);
    g_position_servo.primeAccumulatedAngle(current_deg);
    g_move_done_reported = false;
    g_move_start_ms = millis();

    const char* direction_text = delta_deg >= 0.0f ? "cw" : "ccw";
    Serial.printf("OK: move inc d=%.2f deg  alvo=%.2f deg  vmax=%.2f rpm  dir=%s\n",
                  delta_deg, target_accumulated_deg, vmax_rpm, direction_text);
    return;
  }

  if (!strcmp(cmd,"dec")) {
    if (!g_as5600.detected()) {
      printErrorAndPrompt("ERRO: AS5600 nao detectado no I2C");
      return;
    }

    char* delta_token = nextArg();
    char* rpm_token = strtok_r(nullptr, " \t", &ctx);
    if (!delta_token) {
      printErrorAndPrompt("ERRO: use dec <ddeg> [rpm]");
      return;
    }

    float delta_deg = (float)atof(delta_token);
    const CascadePositionSettings& s = g_position_servo.settings();
    float vmax_rpm = g_default_move_max_rpm;

    if (rpm_token) {
      float parsed_rpm = 0.0f;
      if (parseFloatToken(rpm_token, &parsed_rpm)) {
        vmax_rpm = parsed_rpm;
      }
    }
    vmax_rpm = clampf(vmax_rpm, 0.0f, s.max_target_rpm);

    float current_deg = 0.0f;
    if (!g_as5600.readAngleDeg(&current_deg)) {
      printErrorAndPrompt("ERRO: falha ao ler AS5600");
      return;
    }

    // Delta positivo = CCW (CounterClockwise), negativo = CW (Clockwise)
    const float target_accumulated_deg = current_deg - delta_deg;
    const CascadePositionController::MoveDirection direction =
      delta_deg >= 0.0f ? CascadePositionController::MoveDirection::CounterClockwise
                        : CascadePositionController::MoveDirection::Clockwise;

    g_position_servo.startMove(target_accumulated_deg, vmax_rpm, direction);
    g_position_servo.primeAccumulatedAngle(current_deg);
    g_move_done_reported = false;
    g_move_start_ms = millis();

    const char* direction_text = delta_deg >= 0.0f ? "ccw" : "cw";
    Serial.printf("OK: move dec d=%.2f deg  alvo=%.2f deg  vmax=%.2f rpm  dir=%s\n",
                  delta_deg, target_accumulated_deg, vmax_rpm, direction_text);
    return;
  }

  if (!strcmp(cmd,"qc") || !strcmp(cmd,"cancelmove")) {
    g_position_servo.cancel();
    g_state.target_percent = 0.0f;
    Serial.println("OK: movimento de posicao cancelado");
    return;
  }

  // ── PID de posição ──────────────────────────────────────────────────────

  if (!strcmp(cmd,"pp") || !strcmp(cmd,"pidpos_kp")) {
    char* val = nextArg();
    CascadePositionSettings s = g_position_servo.settings();
    if (!val) {
      Serial.printf("OK: pidpos_kp=%.3f\n", s.pos_pid.kp);
      return;
    }
    s.pos_pid.kp = clampf((float)atof(val), 0.01f, 10.0f);
    g_position_servo.setSettings(s);
    Serial.printf("OK: pidpos_kp=%.3f\n", s.pos_pid.kp);
    return;
  }

  if (!strcmp(cmd,"pi") || !strcmp(cmd,"pidpos_ki")) {
    char* val = nextArg();
    CascadePositionSettings s = g_position_servo.settings();
    if (!val) {
      Serial.printf("OK: pidpos_ki=%.3f\n", s.pos_pid.ki);
      return;
    }
    s.pos_pid.ki = clampf((float)atof(val), 0.0f, 5.0f);
    g_position_servo.setSettings(s);
    Serial.printf("OK: pidpos_ki=%.3f\n", s.pos_pid.ki);
    return;
  }

  if (!strcmp(cmd,"pd") || !strcmp(cmd,"pidpos_kd")) {
    char* val = nextArg();
    CascadePositionSettings s = g_position_servo.settings();
    if (!val) {
      Serial.printf("OK: pidpos_kd=%.3f\n", s.pos_pid.kd);
      return;
    }
    s.pos_pid.kd = clampf((float)atof(val), 0.0f, 5.0f);
    g_position_servo.setSettings(s);
    Serial.printf("OK: pidpos_kd=%.3f\n", s.pos_pid.kd);
    return;
  }

  // ── PID de velocidade ───────────────────────────────────────────────────

  if (!strcmp(cmd,"vp") || !strcmp(cmd,"pidvel_kp")) {
    char* val = nextArg();
    CascadePositionSettings s = g_position_servo.settings();
    if (!val) {
      Serial.printf("OK: pidvel_kp=%.3f\n", s.vel_pid.kp);
      return;
    }
    s.vel_pid.kp = clampf((float)atof(val), 0.1f, 50.0f);
    g_position_servo.setSettings(s);
    Serial.printf("OK: pidvel_kp=%.3f\n", s.vel_pid.kp);
    return;
  }

  if (!strcmp(cmd,"vi") || !strcmp(cmd,"pidvel_ki")) {
    char* val = nextArg();
    CascadePositionSettings s = g_position_servo.settings();
    if (!val) {
      Serial.printf("OK: pidvel_ki=%.3f\n", s.vel_pid.ki);
      return;
    }
    s.vel_pid.ki = clampf((float)atof(val), 0.0f, 20.0f);
    g_position_servo.setSettings(s);
    Serial.printf("OK: pidvel_ki=%.3f\n", s.vel_pid.ki);
    return;
  }

  if (!strcmp(cmd,"vd") || !strcmp(cmd,"pidvel_kd")) {
    char* val = nextArg();
    CascadePositionSettings s = g_position_servo.settings();
    if (!val) {
      Serial.printf("OK: pidvel_kd=%.3f\n", s.vel_pid.kd);
      return;
    }
    s.vel_pid.kd = clampf((float)atof(val), 0.0f, 10.0f);
    g_position_servo.setSettings(s);
    Serial.printf("OK: pidvel_kd=%.3f\n", s.vel_pid.kd);
    return;
  }

  // ── Configuração de movimento ───────────────────────────────────────────

  if (!strcmp(cmd,"pw") || !strcmp(cmd,"poswin")) {
    char* val = nextArg();
    CascadePositionSettings s = g_position_servo.settings();
    if (!val) {
      Serial.printf("OK: poswin=%.2f\n", s.stop_window_deg);
      return;
    }
    s.stop_window_deg = clampf((float)atof(val), 0.2f, 20.0f);
    g_position_servo.setSettings(s);
    Serial.printf("OK: poswin=%.2f\n", s.stop_window_deg);
    return;
  }

  if (!strcmp(cmd,"ap") || !strcmp(cmd,"accel_pos")) {
    char* val = nextArg();
    CascadePositionSettings s = g_position_servo.settings();
    if (!val) {
      Serial.printf("OK: accel_pos=%ums\n", s.accel_ramp_ms);
      return;
    }
    s.accel_ramp_ms = (uint16_t)constrain(atol(val), 50L, 2000L);
    g_position_servo.setSettings(s);
    Serial.printf("OK: accel_pos=%ums\n", s.accel_ramp_ms);
    return;
  }

  if (!strcmp(cmd,"dp") || !strcmp(cmd,"decel_pos")) {
    char* val = nextArg();
    CascadePositionSettings s = g_position_servo.settings();
    if (!val) {
      Serial.printf("OK: decel_pos=%ums\n", s.decel_ramp_ms);
      return;
    }
    s.decel_ramp_ms = (uint16_t)constrain(atol(val), 50L, 2000L);
    g_position_servo.setSettings(s);
    Serial.printf("OK: decel_pos=%ums\n", s.decel_ramp_ms);
    return;
  }

  if (!strcmp(cmd,"kpp") || !strcmp(cmd,"kick_pwm") || !strcmp(cmd,"krp") || !strcmp(cmd,"kick_rpm")) {
    char* val = nextArg();
    CascadePositionSettings s = g_position_servo.settings();
    if (!val) {
      Serial.printf("OK: kick_pwm=%.1f%%\n", s.kick_pwm_percent);
      return;
    }
    s.kick_pwm_percent = clampf((float)atof(val), 0.0f, 100.0f);
    g_position_servo.setSettings(s);
    Serial.printf("OK: kick_pwm=%.1f%%\n", s.kick_pwm_percent);
    return;
  }

  if (!strcmp(cmd,"kms") || !strcmp(cmd,"kick_ms")) {
    char* val = nextArg();
    CascadePositionSettings s = g_position_servo.settings();
    if (!val) {
      Serial.printf("OK: kick_ms=%ums\n", s.kick_ms);
      return;
    }
    s.kick_ms = (uint16_t)constrain(atol(val), 0L, 1000L);
    g_position_servo.setSettings(s);
    Serial.printf("OK: kick_ms=%ums\n", s.kick_ms);
    return;
  }

  if (!strcmp(cmd,"sts") || !strcmp(cmd,"samples_to_stop")) {
    char* val = nextArg();
    CascadePositionSettings s = g_position_servo.settings();
    if (!val) {
      Serial.printf("OK: samples_to_stop=%u\n", s.samples_to_stop);
      return;
    }
    s.samples_to_stop = (uint16_t)constrain(atol(val), 1L, 20L);
    g_position_servo.setSettings(s);
    Serial.printf("OK: samples_to_stop=%u\n", s.samples_to_stop);
    return;
  }

  if (!strcmp(cmd,"mr") || !strcmp(cmd,"maxrpm")) {
    char* val = nextArg();
    CascadePositionSettings s = g_position_servo.settings();
    if (!val) {
      Serial.printf("OK: maxrpm=%.2f\n", s.max_target_rpm);
      return;
    }
    const float new_max = clampf((float)atof(val), 1.0f, 200.0f);
    s.max_target_rpm = new_max;
    // Mantem PID de posicao coerente com o teto maximo de RPM.
    s.pos_pid.output_min = -new_max;
    s.pos_pid.output_max = new_max;
    g_position_servo.setSettings(s);
    Serial.printf("OK: maxrpm=%.2f  pidpos_out=[%.2f..%.2f]\n",
                  s.max_target_rpm, s.pos_pid.output_min, s.pos_pid.output_max);
    return;
  }

  if (!strcmp(cmd,"drpm") || !strcmp(cmd,"default_rpm")) {
    char* val = nextArg();
    const CascadePositionSettings& s = g_position_servo.settings();
    if (!val) {
      Serial.printf("OK: default_rpm=%.2f\n", g_default_move_max_rpm);
      return;
    }
    g_default_move_max_rpm = clampf((float)atof(val), 0.1f, s.max_target_rpm);
    Serial.printf("OK: default_rpm=%.2f\n", g_default_move_max_rpm);
    return;
  }

  if (!strcmp(cmd,"pr") || !strcmp(cmd,"physrpm")) {
    char* val = nextArg();
    CascadePositionSettings s = g_position_servo.settings();
    if (!val) {
      Serial.printf("OK: physrpm=%.2f\n", s.physical_max_rpm);
      return;
    }
    s.physical_max_rpm = clampf((float)atof(val), 1.0f, 100.0f);
    g_position_servo.setSettings(s);
    Serial.printf("OK: physrpm=%.2f\n", s.physical_max_rpm);
    return;
  }

  // ── selecao rapida de canal ─────────────────────────────────────────────

  if (!strcmp(cmd,"ma") || !strcmp(cmd,"motora")) {
    selectMotor(MotorSelection::A_IN1_IN2); printMotorOk(); return;
  }
  if (!strcmp(cmd,"mb") || !strcmp(cmd,"motorb")) {
    selectMotor(MotorSelection::B_IN3_IN4); printMotorOk(); return;
  }
  if (!strcmp(cmd,"mm") || !strcmp(cmd,"motorboth")) {
    selectMotor(MotorSelection::BOTH); printMotorOk(); return;
  }

  // ── controle ────────────────────────────────────────────────────────────

  if (!strcmp(cmd,"pwmf") || !strcmp(cmd,"pf")) {
    char* val = nextArg();
    if (!val) {
      Serial.printf("OK: pwmf=%u Hz (faixa %u..%u)\n",
                    g_pwm_frequency_hz,
                    PWM_MIN_FREQUENCY_HZ,
                    PWM_MAX_FREQUENCY_HZ);
      return;
    }
    const uint32_t freq = (uint32_t)atol(val);
    if (!setPwmFrequencyHz(freq)) {
      Serial.printf("ERRO: use pf <%u..%u>\n",
                    PWM_MIN_FREQUENCY_HZ,
                    PWM_MAX_FREQUENCY_HZ);
      return;
    }
    Serial.printf("OK: pwmf=%u Hz\n", g_pwm_frequency_hz);
    return;
  }

  if (!strcmp(cmd,"stop") || !strcmp(cmd,"x")) {
    g_position_servo.cancel();
    g_state.target_percent  = 0.0f;
    g_state.current_percent = 0.0f;
    g_state.drive_phase     = DrivePhase::IDLE;
    applyMotorOutput(0);
    Serial.println("OK: stop");
    return;
  }

  if (!strcmp(cmd,"brake") || !strcmp(cmd,"b")) {
    g_position_servo.cancel();
    g_state.target_percent  = 0.0f;
    g_state.current_percent = 0.0f;
    if (g_state.brake_ms > 0) {
      g_state.drive_phase    = DrivePhase::BRAKE;
      g_state.brake_start_ms = millis();
      Serial.printf("OK: brake %ums\n", g_state.brake_ms);
    } else {
      g_state.drive_phase = DrivePhase::IDLE;
      applyMotorOutput(0);
      Serial.println("OK: brake (coast)");
    }
    return;
  }

  if (!strcmp(cmd,"pwm") || !strcmp(cmd,"p")) {
    g_position_servo.cancel();
    char* val = nextArg();
    if (!val) { printErrorAndPrompt("ERRO: use p <0..100>"); return; }
    setTargetFromUnsignedPercent((float)atof(val));
    Serial.printf("OK: pwm %.1f%% (%s)\n",
                  fabsf(g_state.target_percent),
                  g_state.direction_sign > 0 ? "fwd" : "rev");
    return;
  }

  if (!strcmp(cmd,"set") || !strcmp(cmd,"v")) {
    g_position_servo.cancel();
    char* val = nextArg();
    if (!val) { printErrorAndPrompt("ERRO: use v <-100..100>"); return; }
    const float pct = clampf((float)atof(val), -100.0f, 100.0f);
    g_state.target_percent = pct;
    if      (pct > 0.0f) g_state.direction_sign =  1;
    else if (pct < 0.0f) g_state.direction_sign = -1;
    Serial.printf("OK: set %.1f%%\n", pct);
    return;
  }

  if (!strcmp(cmd,"rev") || !strcmp(cmd,"r")) {
    g_position_servo.cancel();
    g_state.direction_sign *= -1;
    g_state.target_percent  = -g_state.target_percent;
    Serial.printf("OK: rev -> %s\n",
                  g_state.direction_sign > 0 ? "fwd" : "rev");
    return;
  }

  if (!strcmp(cmd,"dir") || !strcmp(cmd,"d")) {
    g_position_servo.cancel();
    char* val = nextArg();
    if (!val) { printErrorAndPrompt("ERRO: use d f|r"); return; }
    if (!strcmp(val,"f") || !strcmp(val,"fwd") || !strcmp(val,"frente")) {
      g_state.direction_sign = 1;
      if (g_state.target_percent < 0.0f)
        g_state.target_percent = -g_state.target_percent;
      Serial.println("OK: dir fwd");
    } else if (!strcmp(val,"r") || !strcmp(val,"rev") || !strcmp(val,"re")) {
      g_state.direction_sign = -1;
      if (g_state.target_percent > 0.0f)
        g_state.target_percent = -g_state.target_percent;
      Serial.println("OK: dir rev");
    } else {
      printErrorAndPrompt("ERRO: use d f|r");
    }
    return;
  }

  // ── canal do motor ──────────────────────────────────────────────────────

  if (!strcmp(cmd,"motor") || !strcmp(cmd,"m")) {
    char* val = nextArg();
    if (!val) { printErrorAndPrompt("ERRO: use m in3|in1|both"); return; }
    MotorSelection sel;
    if (parseMotorToken(val, &sel)) { selectMotor(sel); printMotorOk(); }
    else printErrorAndPrompt("ERRO: use m in3|in1|both");
    return;
  }

  // ── rampas ──────────────────────────────────────────────────────────────

  if (!strcmp(cmd,"accel") || !strcmp(cmd,"a")) {
    char* val = nextArg();
    if (!val) { Serial.printf("OK: accel=%ums\n", g_state.accel_ms); return; }
    g_state.accel_ms = (uint16_t)constrain(atol(val), 1L, 5000L);
    Serial.printf("OK: accel=%ums\n", g_state.accel_ms);
    return;
  }

  if (!strcmp(cmd,"decel") || !strcmp(cmd,"z")) {
    char* val = nextArg();
    if (!val) { Serial.printf("OK: decel=%ums\n", g_state.decel_ms); return; }
    g_state.decel_ms = (uint16_t)constrain(atol(val), 1L, 5000L);
    Serial.printf("OK: decel=%ums\n", g_state.decel_ms);
    return;
  }

  if (!strcmp(cmd,"curve") || !strcmp(cmd,"c")) {
    char* acc = nextArg();
    char* dec = strtok_r(nullptr, " \t", &ctx);
    if (!acc || !dec) { printErrorAndPrompt("ERRO: use c <acc> <dec>"); return; }
    g_state.accel_curve = clampf((float)atof(acc), 0.2f, 3.0f);
    g_state.decel_curve = clampf((float)atof(dec), 0.2f, 3.0f);
    Serial.printf("OK: curve acc=%.2f dec=%.2f\n",
                  g_state.accel_curve, g_state.decel_curve);
    return;
  }

  // ── dead band / kick / freio / limite ────────────────────────────────────

  if (!strcmp(cmd,"tstart") || !strcmp(cmd,"ts")) {
    char* val = nextArg();
    if (!val) {
      Serial.printf("OK: tstart=%.0f%%\n", g_state.threshold_start);
      return;
    }
    g_state.threshold_start = clampf((float)atof(val), 1.0f, 50.0f);
    // Garante que tstop < tstart
    if (g_state.threshold_stop >= g_state.threshold_start)
      g_state.threshold_stop = g_state.threshold_start * 0.4f;
    Serial.printf("OK: tstart=%.0f%%  tstop=%.0f%%\n",
                  g_state.threshold_start, g_state.threshold_stop);
    return;
  }

  if (!strcmp(cmd,"tstop") || !strcmp(cmd,"tp")) {
    char* val = nextArg();
    if (!val) {
      Serial.printf("OK: tstop=%.0f%%\n", g_state.threshold_stop);
      return;
    }
    g_state.threshold_stop =
      clampf((float)atof(val), 0.0f, g_state.threshold_start - 1.0f);
    Serial.printf("OK: tstop=%.0f%%\n", g_state.threshold_stop);
    return;
  }

  if (!strcmp(cmd,"kick") || !strcmp(cmd,"k")) {
    char* val = nextArg();
    if (!val) { Serial.printf("OK: kick=%ums\n", g_state.kick_ms); return; }
    g_state.kick_ms = (uint16_t)constrain(atol(val), 0L, 2000L);
    Serial.printf("OK: kick=%ums%s\n", g_state.kick_ms,
                  g_state.kick_ms == 0 ? " (desabilitado)" : "");
    return;
  }

  if (!strcmp(cmd,"kickp") || !strcmp(cmd,"kp")) {
    char* val = nextArg();
    if (!val) { Serial.printf("OK: kickp=%.0f%%\n", g_state.kick_pct); return; }
    g_state.kick_pct = clampf((float)atof(val), 0.0f, 100.0f);
    Serial.printf("OK: kickp=%.0f%%\n", g_state.kick_pct);
    return;
  }

  if (!strcmp(cmd,"brakems") || !strcmp(cmd,"bm")) {
    char* val = nextArg();
    if (!val) { Serial.printf("OK: brakems=%ums\n", g_state.brake_ms); return; }
    g_state.brake_ms = (uint16_t)constrain(atol(val), 0L, 2000L);
    Serial.printf("OK: brakems=%ums%s\n", g_state.brake_ms,
                  g_state.brake_ms == 0 ? " (coast)" : "");
    return;
  }

  if (!strcmp(cmd,"limit") || !strcmp(cmd,"l")) {
    char* val = nextArg();
    if (!val) {
      Serial.printf("OK: limit=%u%%\n", g_state.power_limit_percent);
      return;
    }
    g_state.power_limit_percent = (uint8_t)constrain(atol(val), 0L, 100L);
    Serial.printf("OK: limit=%u%%\n", g_state.power_limit_percent);
    return;
  }

  // ── interface ───────────────────────────────────────────────────────────

  if (!strcmp(cmd,"echo") || !strcmp(cmd,"e")) {
    char* val = nextArg();
    if (!val) {
      Serial.printf("OK: echo %s\n",
                    g_state.serial_echo_enabled ? "on" : "off");
      return;
    }
    if (!strcmp(val,"on") || !strcmp(val,"1")) {
      g_state.serial_echo_enabled = true;
      Serial.println("OK: echo on");
    } else if (!strcmp(val,"off") || !strcmp(val,"0")) {
      g_state.serial_echo_enabled = false;
      Serial.println("OK: echo off");
    } else {
      printErrorAndPrompt("ERRO: use e on|off");
    }
    return;
  }

  printErrorAndPrompt("ERRO: comando desconhecido. Digite h");
}

// ─── input serial ───────────────────────────────────────────────────────────

void processSerialInput() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\b' || c == 127) {
      if (g_serial_index > 0) {
        g_serial_index--;
        if (g_state.serial_echo_enabled) Serial.print("\b \b");
      }
      continue;
    }

    if (c == '\n') {
      g_serial_line[g_serial_index] = '\0';
      if (g_state.serial_echo_enabled) Serial.println();
      parseAndHandleCommand(g_serial_line);
      g_serial_index = 0;
      g_serial_prompt_pending = true;
      continue;
    }

    if (g_serial_index < SERIAL_LINE_BUFFER - 1) {
      g_serial_line[g_serial_index++] = c;
      if (g_state.serial_echo_enabled) Serial.print(c);
    } else {
      g_serial_index = 0;
      printErrorAndPrompt("\nERRO: linha muito longa");
    }
  }
}

// ─── maquina de controle ────────────────────────────────────────────────────

void updateRampControl() {
  const uint32_t now = millis();

  if (g_state.last_control_update_ms == 0) {
    g_state.last_control_update_ms = now;
    return;
  }

  const uint32_t dt = now - g_state.last_control_update_ms;
  if (dt < CONTROL_PERIOD_MS) return;
  g_state.last_control_update_ms = now;

  const bool servo_active = g_position_servo.isActive();

  // No modo de posicionamento, usa diretamente a saida da cascata PID
  // (sem a rampa manual adicional desta maquina de fases).
  if (servo_active) {
    g_state.current_percent = g_state.target_percent;
    g_state.drive_phase = (fabsf(g_state.current_percent) > 0.01f)
                           ? DrivePhase::RUNNING
                           : DrivePhase::IDLE;

    float limited = signedPercentToLimitedPercent(
                      g_state.current_percent, g_state.power_limit_percent);
    if (fabsf(limited) <= 0.01f) {
      applyMotorOutput(0);
      return;
    }

    const uint8_t pwm = percentToPwm(fabsf(limited));
    const int16_t out = (limited >= 0.0f) ? (int16_t)pwm : -(int16_t)pwm;
    applyMotorOutput(out);
    return;
  }

  const bool kick_allowed = !servo_active && (g_state.kick_ms > 0);

  const float  tstart      = g_state.threshold_start;
  const float  tstop       = g_state.threshold_stop;
  const bool   target_zero = fabsf(g_state.target_percent) <= tstop;
  const int8_t target_sign = (g_state.target_percent >= 0.0f) ? 1 : -1;

  switch (g_state.drive_phase) {

    // ── IDLE ──────────────────────────────────────────────────────────────
    case DrivePhase::IDLE: {
      g_state.current_percent = 0.0f;
      if (!target_zero) {
        g_state.kick_direction = target_sign;
        if (kick_allowed) {
          g_state.drive_phase   = DrivePhase::KICK;
          g_state.kick_start_ms = now;
        } else {
          // Sem kick: entra em RUNNING logo acima do tstop
          g_state.current_percent = g_state.kick_direction * (tstop + 0.1f);
          g_state.drive_phase     = DrivePhase::RUNNING;
        }
      }
      applyMotorOutput(0);
      return;
    }

    // ── KICK ──────────────────────────────────────────────────────────────
    case DrivePhase::KICK: {
      if (!kick_allowed || now - g_state.kick_start_ms >= g_state.kick_ms) {
        // Kick concluido: inicia RUNNING a partir de tstart
        g_state.current_percent = g_state.kick_direction * tstart;
        g_state.drive_phase     = DrivePhase::RUNNING;
        // cai em RUNNING logo abaixo
      } else {
        // Pulso ativo
        const float kick_out = g_state.kick_direction * g_state.kick_pct;
        const float limited  = signedPercentToLimitedPercent(
                                 kick_out, g_state.power_limit_percent);
        const uint8_t pwm = percentToPwm(fabsf(limited));
        const int16_t out = (limited >= 0.0f) ? (int16_t)pwm : -(int16_t)pwm;
        applyMotorOutput(out);
        return;
      }
      [[fallthrough]];
    }

    // ── RUNNING ───────────────────────────────────────────────────────────
    case DrivePhase::RUNNING: {
      const int8_t current_sign = (g_state.current_percent > 0.0f) ? 1 : -1;
      const bool   reversed     = !target_zero && (target_sign != current_sign);

      // Se invertendo, rampa em direcao ao zero.
      // Ao chegar em tstop -> IDLE -> KICK no novo sentido.
      const float ramp_target = reversed ? 0.0f : g_state.target_percent;
      const float error       = ramp_target - g_state.current_percent;
      const float abs_error   = fabsf(error);

      if (abs_error < 0.01f) {
        g_state.current_percent = ramp_target;
      } else {
        const bool accelerating =
          fabsf(ramp_target) > fabsf(g_state.current_percent);
        const uint16_t ramp_ms =
          accelerating ? g_state.accel_ms : g_state.decel_ms;
        const float gamma =
          accelerating ? g_state.accel_curve : g_state.decel_curve;
        const float step =
          (100.0f * (float)dt / (float)ramp_ms) *
          (0.15f + 0.85f * powf(
            clampf(abs_error / 100.0f, 0.0f, 1.0f), gamma));

        if (abs_error <= step) {
          g_state.current_percent = ramp_target;
        } else {
          g_state.current_percent += (error > 0.0f) ? step : -step;
        }
      }

      // Caiu abaixo do tstop: vai para IDLE.
      // IDLE fara KICK no sentido do target se ainda houver alvo.
      if (fabsf(g_state.current_percent) <= tstop) {
        g_state.current_percent = 0.0f;
        g_state.drive_phase     = DrivePhase::IDLE;
        applyMotorOutput(0);
        return;
      }

      // Saida com remapeamento de dead band:
      // current [tstop..100] -> PWM [tstart..100]
      const float mapped  = applyDeadBandRemap(
                              fabsf(g_state.current_percent), tstart, tstop);
      const float limited = signedPercentToLimitedPercent(
                              mapped, g_state.power_limit_percent);
      const uint8_t pwm = percentToPwm(limited);
      const int16_t out =
        (g_state.current_percent >= 0.0f) ? (int16_t)pwm : -(int16_t)pwm;
      applyMotorOutput(out);
      return;
    }

    // ── BRAKE ─────────────────────────────────────────────────────────────
    case DrivePhase::BRAKE: {
      if (now - g_state.brake_start_ms >= g_state.brake_ms) {
        g_state.current_percent = 0.0f;
        g_state.drive_phase     = DrivePhase::IDLE;
        applyMotorOutput(0);
      } else {
        applyBrakeOutput();
      }
      return;
    }
  }
}

// ─── setup / loop ───────────────────────────────────────────────────────────

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(SERIAL_STARTUP_WAIT_MS);

  setPwmFrequencyHz(g_pwm_frequency_hz);
  analogWriteResolution(PWM_RESOLUTION_BITS);

  CascadePositionSettings pos_settings;
  
  // PID de posição: controla RPM desejado a partir do erro de posição
  pos_settings.pos_pid.kp = 0.75f;    // Mais firme para o motor lento fechar os ultimos graus
  pos_settings.pos_pid.ki = 0.04f;    // Pequena integral para remover erro residual
  pos_settings.pos_pid.kd = 0.04f;    // Amortecimento leve para nao frear demais
  pos_settings.pos_pid.output_min = -DEFAULT_MAX_TARGET_RPM;
  pos_settings.pos_pid.output_max = DEFAULT_MAX_TARGET_RPM;
  
  // PID de velocidade: controla PWM para manter RPM real = RPM cmd
  // Defaults ajustados pelos ultimos testes de bancada para reduzir overspeed.
  pos_settings.vel_pid.kp = 5.0f;
  pos_settings.vel_pid.ki = 0.4f;
  pos_settings.vel_pid.kd = 0.04f;
  pos_settings.vel_pid.output_min = -100.0f;
  pos_settings.vel_pid.output_max = 100.0f;
  pos_settings.vel_pid.integral_max = 25.0f;
  
  pos_settings.max_target_rpm = DEFAULT_MAX_TARGET_RPM;
  pos_settings.physical_max_rpm = 3.0f;
  pos_settings.stop_window_deg = 1.2f;
  pos_settings.accel_ramp_ms = 250;
  pos_settings.decel_ramp_ms = 220;
  pos_settings.kick_pwm_percent = 85.0f;
  pos_settings.kick_ms = 180;
  pos_settings.samples_to_stop = 3;
  pos_settings.velocity_window_ms = 220;  // Menos atraso na leitura de rpm para encerrar a parada antes
  pos_settings.velocity_num_samples = 6;
  
  g_position_servo.setSettings(pos_settings);

  g_as5600.begin(Wire, I2C_SDA_PIN, I2C_SCL_PIN);

  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  setMotorSignedPwm(0, MOTOR_A_IN1, MOTOR_A_IN2);
  setMotorSignedPwm(0, MOTOR_B_IN1, MOTOR_B_IN2);

  Serial.println("\n=== Motor PWM Tester ===");
  Serial.println("Placa: ESP32-C3 Super Mini  |  Motor padrao: IN3/IN4");
  Serial.printf("PWM: freq=%u Hz  resolucao=%u bits\n", g_pwm_frequency_hz, PWM_RESOLUTION_BITS);
  Serial.printf("I2C: SDA=%u SCL=%u\n", I2C_SDA_PIN, I2C_SCL_PIN);
  if (g_as5600.detected()) {
    Serial.printf("AS5600 detectado no endereco 0x%02X\n", g_as5600.address());
  } else {
    Serial.printf("AS5600 NAO detectado no endereco 0x%02X\n", g_as5600.address());
  }
  Serial.println("PosCtrl pronto (motor nominal 2 rpm)");
  printHelp();
  printStatus();
  g_serial_prompt_pending = true;
}

void loop() {
  // Não processa serial durante movimento para não interromper a cascata PID
  if (!g_position_servo.isActive()) {
    processSerialInput();
  }
  updatePositionMoveControl();
  updateRampControl();

  // ─── AUTOTUNE HANDLER ────────────────────────────────────────────────
  if (g_autotune.active) {
    static bool move_started = false;
    static uint32_t move_start_ms = 0;
    static float measured_deg[128];
    static uint32_t measured_time[128];
    static float measured_rpm_cmd[128];
    static float measured_rpm_real[128];
    static int sample_count = 0;

    float current_deg = 0.0f;
    if (!g_as5600.readAngleDeg(&current_deg)) {
      Serial.println("AUTOTUNE ERRO: falha ao ler AS5600");
      g_autotune.active = false;
      g_serial_prompt_pending = true;
      move_started = false;
      return;
    }

    if (!move_started) {
      // Start the step move
      g_position_servo.startMove(g_autotune.target_deg, g_autotune.test_rpm, CascadePositionController::MoveDirection::Shortest);
      g_move_done_reported = false;
      move_start_ms = millis();
      sample_count = 0;
      move_started = true;
      Serial.printf("AUTOTUNE: step %.2f -> %.2f deg (rpm=%.2f)\n",
                    g_autotune.initial_deg, g_autotune.target_deg, g_autotune.test_rpm);
    } else {
      // Record response every 10ms (up to 128 samples)
      if (sample_count < 128) {
        measured_deg[sample_count] = current_deg;
        measured_time[sample_count] = millis() - move_start_ms;
        measured_rpm_cmd[sample_count] = g_position_servo.commandedRpm();
        measured_rpm_real[sample_count] = g_position_servo.measuredRpm();
        sample_count++;
      }
      // When move is done, analyze response
      if (!g_position_servo.isActive()) {
        // Find step response characteristics
        float step_size = g_autotune.target_deg - g_autotune.initial_deg;
        float peak = g_autotune.initial_deg;
        float peak_time = 0;
        float settle_time = 0;
        float overshoot = 0;
        for (int i = 0; i < sample_count; ++i) {
          float val = measured_deg[i];
          if ((val - g_autotune.initial_deg) > (peak - g_autotune.initial_deg)) {
            peak = val;
            peak_time = measured_time[i];
          }
          // Settling: within 2% of step
          if (settle_time == 0 && fabsf(val - g_autotune.target_deg) < fabsf(step_size) * 0.02f) {
            settle_time = measured_time[i];
          }
        }
        overshoot = peak - g_autotune.target_deg;
        Serial.println("AUTOTUNE: resposta medida:");
        Serial.printf("  Step: %.2f deg\n", step_size);
        Serial.printf("  Pico: %.2f deg (%.0f ms)\n", peak, peak_time);
        Serial.printf("  Overshoot: %.2f deg\n", overshoot);
        Serial.printf("  Settling time: %.0f ms\n", settle_time);

        float avg_abs_rpm_err = 0.0f;
        float avg_abs_rpm_cmd = 0.0f;
        float peak_rpm_err = 0.0f;
        int rpm_samples = 0;
        int rpm_sign_flips = 0;
        int last_sign = 0;
        for (int i = 0; i < sample_count; ++i) {
          const float cmd_rpm = measured_rpm_cmd[i];
          const float real_rpm = measured_rpm_real[i];
          if (fabsf(cmd_rpm) < 0.5f) continue;
          const float err = cmd_rpm - real_rpm;
          const float abs_err = fabsf(err);
          avg_abs_rpm_err += abs_err;
          avg_abs_rpm_cmd += fabsf(cmd_rpm);
          if (abs_err > peak_rpm_err) peak_rpm_err = abs_err;
          const int sign = (err > 0.05f) ? 1 : ((err < -0.05f) ? -1 : 0);
          if (sign != 0) {
            if (last_sign != 0 && sign != last_sign) rpm_sign_flips++;
            last_sign = sign;
          }
          rpm_samples++;
        }
        if (rpm_samples > 0) {
          avg_abs_rpm_err /= (float)rpm_samples;
          avg_abs_rpm_cmd /= (float)rpm_samples;
        }

        // Sugestão de PID (Ziegler-Nichols step response, simplificado)
        float Kp = 0.0f, Ki = 0.0f, Kd = 0.0f;
        if (settle_time > 0 && fabsf(step_size) > 1.0f && fabsf(overshoot) > 0.05f) {
          float tau = settle_time / 1000.0f; // s
          float Ku = fabsf(step_size) / fabsf(overshoot);
          Kp = 0.6f * Ku;
          Ki = 1.2f * Ku / tau;
          Kd = 0.075f * Ku * tau;
        } else {
          Serial.println("AUTOTUNE: resposta sem overshoot suficiente para estimar ganhos por este metodo");
        }
        Serial.println("AUTOTUNE: sugestao PID POS (Ziegler-Nichols):");
        Serial.printf("  Kp=%.3f  Ki=%.3f  Kd=%.3f\n", Kp, Ki, Kd);
        Serial.println("Digite: pp <Kp>  pi <Ki>  pd <Kd> para aplicar.");

        const CascadePositionSettings current_settings = g_position_servo.settings();
        float vKp = current_settings.vel_pid.kp;
        float vKi = current_settings.vel_pid.ki;
        float vKd = current_settings.vel_pid.kd;
        if (rpm_samples > 0 && avg_abs_rpm_cmd > 0.1f) {
          float err_ratio = avg_abs_rpm_err / avg_abs_rpm_cmd;
          vKp = clampf(vKp * (1.0f + 1.4f * err_ratio), 0.1f, 50.0f);
          vKi = clampf(vKi * (1.0f + 0.8f * err_ratio), 0.0f, 20.0f);
          vKd = clampf(vKd + 0.05f * peak_rpm_err, 0.0f, 10.0f);
          if (rpm_sign_flips >= 4) {
            // Oscilação detectada: reduz P/I e aumenta D para amortecer.
            vKp = clampf(vKp * 0.8f, 0.1f, 50.0f);
            vKi = clampf(vKi * 0.7f, 0.0f, 20.0f);
            vKd = clampf(vKd * 1.25f, 0.0f, 10.0f);
          }
        }
        Serial.println("AUTOTUNE: sugestao PID VEL (heuristica por erro de RPM):");
        Serial.printf("  Kp=%.3f  Ki=%.3f  Kd=%.3f\n", vKp, vKi, vKd);
        Serial.printf("  diagnostico: err_medio=%.2f rpm  err_pico=%.2f rpm  oscilacoes=%d\n",
                      avg_abs_rpm_err, peak_rpm_err, rpm_sign_flips);
        Serial.println("Digite: vp <Kp>  vi <Ki>  vd <Kd> para aplicar.");
        Serial.println("Ou use: autotune_apply");

        g_autotune.pos_kp = Kp;
        g_autotune.pos_ki = Ki;
        g_autotune.pos_kd = Kd;
        g_autotune.vel_kp = vKp;
        g_autotune.vel_ki = vKi;
        g_autotune.vel_kd = vKd;
        g_autotune.has_suggestion = true;

        g_autotune.active = false;
        g_serial_prompt_pending = true;
        move_started = false;
      }
    }
    return;
  }

  if (!g_position_servo.isActive() && g_serial_prompt_pending && Serial.available() == 0) {
    printPrompt();
    g_serial_prompt_pending = false;
  }
}
