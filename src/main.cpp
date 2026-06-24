#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include <As5600Sensor.h>
#include <AdrcPositionController.h>
#include <RepetitiveMotionController.h>

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "repetitive_motion_web_page.h"
#include "control_settings_web_page.h"

#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#else
#define WIFI_STA_SSID ""
#define WIFI_STA_PASSWORD ""
#endif

#ifndef MOTOR_CONTROL_UNIT
#define MOTOR_CONTROL_UNIT 1
#endif

static_assert(MOTOR_CONTROL_UNIT >= 1 && MOTOR_CONTROL_UNIT <= 99,
              "MOTOR_CONTROL_UNIT deve estar entre 1 e 99");

constexpr uint8_t MOTOR_CONTROL_UNIT_NUMBER = MOTOR_CONTROL_UNIT;
char OTA_AP_SSID[20] = {0};
constexpr char OTA_AP_PASSWORD[] = "+5511981550110";
char OTA_HOSTNAME[28] = {0};
constexpr char OTA_PASSWORD[]    = "as5600-update";
constexpr uint32_t WIFI_STA_CONNECT_TIMEOUT_MS = 12000;
// Para seis unidades: 01/04 -> canal 1, 02/05 -> canal 6, 03/06 -> canal 11.
// Os tres canais nao se sobrepoem e recebem exatamente duas unidades cada.
constexpr uint8_t WIFI_AP_CHANNEL_SLOT = (MOTOR_CONTROL_UNIT_NUMBER - 1U) % 3U;
constexpr uint8_t WIFI_AP_CHANNEL = WIFI_AP_CHANNEL_SLOT == 0U ? 1U
                                    : WIFI_AP_CHANNEL_SLOT == 1U ? 6U : 11U;
constexpr uint8_t OTA_BUTTON_PIN = 7;
constexpr uint32_t OTA_BUTTON_HOLD_MS = 1500;

// ESP32-C3 Super Mini + L298N
// IN1/IN2 = motor A,  IN3/IN4 = motor B  (A é o padrao deste projeto)
constexpr uint8_t  MOTOR_A_IN1            = 1;
constexpr uint8_t  MOTOR_A_IN2            = 2;
constexpr uint8_t  MOTOR_B_IN1            = 3;
constexpr uint8_t  MOTOR_B_IN2            = 4;

constexpr uint32_t SERIAL_BAUD            = 115200;
constexpr size_t   SERIAL_LINE_BUFFER     = 96;
constexpr uint32_t CONTROL_PERIOD_MS      = 2;  // ADRC e saida da ponte a 500 Hz
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
  MotorSelection selected_motor = MotorSelection::BOTH;

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
  float    kick_pct = 50.0f;       // potencia do pulso (%)
  uint16_t kick_ms  = 50;         // duracao (ms);  0 = desabilitado

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
bool g_ota_mode_active = false;
bool g_ota_update_in_progress = false;
bool g_ota_button_pressed = false;
uint32_t g_ota_button_pressed_ms = 0;

char   g_serial_line[SERIAL_LINE_BUFFER];
size_t g_serial_index = 0;
As5600Sensor            g_as5600;
AdrcPositionController g_position_servo;
bool                    g_move_done_reported = false;
bool                    g_adrc_stall_fault = false;
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
int16_t                 g_move_peak_pwm_adrc_abs = 0;
int16_t                 g_move_peak_pwm_out_abs = 0;
float                   g_move_pwm_out_abs_sum = 0.0f;
uint32_t                g_move_pwm_out_samples = 0;
uint32_t                g_move_pwm_out_sat_samples = 0;
uint32_t                g_pwm_frequency_hz = PWM_DEFAULT_FREQUENCY_HZ;
uint32_t                g_move_debug_last_print_ms = 0;

// Adaptadores: a camada de movimento repetitivo conhece apenas estes comandos
// genericos, sem depender do AS5600 ou do controlador ADRC concreto.
void startAutomaticPositionMove(
  float target_deg, float rpm, RepetitiveMotionController::Direction direction);
bool isAutomaticPositionMoveActive();
void stopAutomaticPositionMove();
void applyMotorOutput(int16_t signed_pwm);
float clampf(float v, float lo, float hi);
bool setPwmFrequencyHz(uint32_t hz);

RepetitiveMotionController g_repetitive_motion({
  startAutomaticPositionMove,
  isAutomaticPositionMoveActive,
  stopAutomaticPositionMove
});

Preferences g_repetitive_preferences;
bool g_repetitive_preferences_ready = false;
bool g_repetitive_run_on_boot = false;
bool g_persisted_repetitive_running = false;
WebServer g_web_server(80);
bool g_web_server_started = false;

void loadRepetitiveMotionPreferences() {
  g_repetitive_preferences_ready =
    g_repetitive_preferences.begin("repeat_motion", false);
  if (!g_repetitive_preferences_ready) {
    Serial.println("AVISO: nao foi possivel abrir a configuracao persistente do ciclo");
    return;
  }

  AdrcPositionSettings adrc = g_position_servo.settings();
  adrc.control_bandwidth = clampf(g_repetitive_preferences.getFloat(
    "adrc_wc", adrc.control_bandwidth), 1.0f, 100.0f);
  adrc.observer_bandwidth = clampf(g_repetitive_preferences.getFloat(
    "adrc_wo", adrc.observer_bandwidth), 1.0f, 300.0f);
  adrc.plant_gain = clampf(g_repetitive_preferences.getFloat(
    "adrc_b0", adrc.plant_gain), 1.0f, 2000.0f);
  adrc.max_target_rpm = clampf(g_repetitive_preferences.getFloat(
    "max_rpm", adrc.max_target_rpm), 0.1f, 10.0f);
  adrc.physical_max_rpm = clampf(g_repetitive_preferences.getFloat(
    "phys_rpm", adrc.physical_max_rpm), 0.1f, 10.0f);
  adrc.max_target_rpm = fminf(adrc.max_target_rpm, adrc.physical_max_rpm);
  adrc.stop_window_deg = clampf(g_repetitive_preferences.getFloat(
    "stop_win", adrc.stop_window_deg), 0.2f, 20.0f);
  adrc.accel_ramp_ms = (uint16_t)constrain(
    g_repetitive_preferences.getUInt("accel_ms", adrc.accel_ramp_ms), 50U, 2000U);
  adrc.decel_ramp_ms = (uint16_t)constrain(
    g_repetitive_preferences.getUInt("decel_ms", adrc.decel_ramp_ms), 50U, 2000U);
  adrc.kick_pwm_percent = clampf(g_repetitive_preferences.getFloat(
    "kick_pct", adrc.kick_pwm_percent), 0.0f, 100.0f);
  adrc.kick_ms = (uint16_t)constrain(
    g_repetitive_preferences.getUInt("kick_ms", adrc.kick_ms), 0U, 1000U);
  adrc.samples_to_stop = (uint16_t)constrain(
    g_repetitive_preferences.getUInt("stop_samples", adrc.samples_to_stop), 1U, 20U);
  adrc.minimum_drive_pwm_percent = clampf(g_repetitive_preferences.getFloat(
    "min_pwm", adrc.minimum_drive_pwm_percent), 0.0f, 45.0f);
  adrc.stall_timeout_ms = (uint16_t)constrain(
    g_repetitive_preferences.getUInt("stall_ms", adrc.stall_timeout_ms), 100U, 10000U);
  adrc.stall_velocity_deg_s = clampf(g_repetitive_preferences.getFloat(
    "stall_vel", adrc.stall_velocity_deg_s), 0.1f, 20.0f);
  adrc.velocity_window_ms = (uint16_t)constrain(
    g_repetitive_preferences.getUInt("vel_win", adrc.velocity_window_ms), 20U, 1000U);
  adrc.velocity_num_samples = (uint8_t)constrain(
    g_repetitive_preferences.getUInt("vel_samples", adrc.velocity_num_samples), 2U, 20U);
  g_position_servo.setSettings(adrc);
  g_state.power_limit_percent = (uint8_t)constrain(
    g_repetitive_preferences.getUInt("power_limit", g_state.power_limit_percent), 0U, 100U);
  g_pwm_frequency_hz = constrain(
    g_repetitive_preferences.getUInt("pwm_hz", g_pwm_frequency_hz),
    PWM_MIN_FREQUENCY_HZ, PWM_MAX_FREQUENCY_HZ);

  RepetitiveMotionConfig c = g_repetitive_motion.config();
  c.start_deg = g_repetitive_preferences.getFloat("start_deg", c.start_deg);
  c.end_deg = g_repetitive_preferences.getFloat("end_deg", c.end_deg);
  const float max_rpm = g_position_servo.settings().max_target_rpm;
  c.start_to_end_rpm = clampf(
    g_repetitive_preferences.getFloat("rpm_out", c.start_to_end_rpm), 0.1f, max_rpm);
  c.end_to_start_rpm = clampf(
    g_repetitive_preferences.getFloat("rpm_back", c.end_to_start_rpm), 0.1f, max_rpm);
  c.dwell_at_start_ms = min(
    g_repetitive_preferences.getULong("wait_start", c.dwell_at_start_ms), 3600000UL);
  c.dwell_at_end_ms = min(
    g_repetitive_preferences.getULong("wait_end", c.dwell_at_end_ms), 3600000UL);
  g_repetitive_motion.setConfig(c);
  g_repetitive_run_on_boot = g_repetitive_preferences.getBool("running", false);
  g_persisted_repetitive_running = g_repetitive_run_on_boot;

}

void saveRepetitiveMotionConfig() {
  if (!g_repetitive_preferences_ready) return;
  const RepetitiveMotionConfig& c = g_repetitive_motion.config();
  g_repetitive_preferences.putFloat("start_deg", c.start_deg);
  g_repetitive_preferences.putFloat("end_deg", c.end_deg);
  g_repetitive_preferences.putFloat("rpm_out", c.start_to_end_rpm);
  g_repetitive_preferences.putFloat("rpm_back", c.end_to_start_rpm);
  g_repetitive_preferences.putULong("wait_start", c.dwell_at_start_ms);
  g_repetitive_preferences.putULong("wait_end", c.dwell_at_end_ms);
}

void saveAdrcSettings() {
  if (!g_repetitive_preferences_ready) return;
  const AdrcPositionSettings& s = g_position_servo.settings();
  g_repetitive_preferences.putFloat("adrc_wc", s.control_bandwidth);
  g_repetitive_preferences.putFloat("adrc_wo", s.observer_bandwidth);
  g_repetitive_preferences.putFloat("adrc_b0", s.plant_gain);
}

void saveControlSettings() {
  if (!g_repetitive_preferences_ready) return;
  const AdrcPositionSettings& s = g_position_servo.settings();
  g_repetitive_preferences.putFloat("adrc_wc", s.control_bandwidth);
  g_repetitive_preferences.putFloat("adrc_wo", s.observer_bandwidth);
  g_repetitive_preferences.putFloat("adrc_b0", s.plant_gain);
  g_repetitive_preferences.putFloat("max_rpm", s.max_target_rpm);
  g_repetitive_preferences.putFloat("phys_rpm", s.physical_max_rpm);
  g_repetitive_preferences.putFloat("stop_win", s.stop_window_deg);
  g_repetitive_preferences.putUInt("accel_ms", s.accel_ramp_ms);
  g_repetitive_preferences.putUInt("decel_ms", s.decel_ramp_ms);
  g_repetitive_preferences.putFloat("kick_pct", s.kick_pwm_percent);
  g_repetitive_preferences.putUInt("kick_ms", s.kick_ms);
  g_repetitive_preferences.putUInt("stop_samples", s.samples_to_stop);
  g_repetitive_preferences.putFloat("min_pwm", s.minimum_drive_pwm_percent);
  g_repetitive_preferences.putUInt("stall_ms", s.stall_timeout_ms);
  g_repetitive_preferences.putFloat("stall_vel", s.stall_velocity_deg_s);
  g_repetitive_preferences.putUInt("vel_win", s.velocity_window_ms);
  g_repetitive_preferences.putUInt("vel_samples", s.velocity_num_samples);
  g_repetitive_preferences.putUInt("power_limit", g_state.power_limit_percent);
  g_repetitive_preferences.putUInt("pwm_hz", g_pwm_frequency_hz);
}

void setRepetitiveRunning(bool running, bool persist = true) {
  g_repetitive_motion.setRunning(running, millis());
  if (persist && g_repetitive_preferences_ready &&
      running != g_persisted_repetitive_running) {
    g_repetitive_preferences.putBool("running", running);
    g_persisted_repetitive_running = running;
  }
}

bool parseWebNumber(const char* name, float* value) {
  if (!value || !g_web_server.hasArg(name)) return false;
  const String text = g_web_server.arg(name);
  char* end = nullptr;
  const float parsed = strtof(text.c_str(), &end);
  if (end == text.c_str() || !end || *end != '\0' || !isfinite(parsed)) return false;
  *value = parsed;
  return true;
}

void sendWebStatus(int status_code = 200) {
  const RepetitiveMotionConfig& c = g_repetitive_motion.config();
  float angle_deg = 0.0f;
  const bool sensor_ok = g_as5600.detected() && g_as5600.readAngleDeg(&angle_deg);
  String json;
  json.reserve(320);
  json += F("{\"unit\":");
  json += String(MOTOR_CONTROL_UNIT_NUMBER);
  json += F(",\"running\":");
  json += g_repetitive_motion.running() ? F("true") : F("false");
  json += F(",\"moveActive\":");
  json += g_position_servo.isActive() ? F("true") : F("false");
  json += F(",\"phase\":\"");
  json += g_repetitive_motion.phaseText();
  json += F("\",\"sensor\":");
  json += sensor_ok ? F("true") : F("false");
  json += F(",\"angle\":"); json += String(angle_deg, 2);
  json += F(",\"start\":"); json += String(c.start_deg, 2);
  json += F(",\"end\":"); json += String(c.end_deg, 2);
  json += F(",\"rpmOut\":"); json += String(c.start_to_end_rpm, 2);
  json += F(",\"rpmBack\":"); json += String(c.end_to_start_rpm, 2);
  json += F(",\"waitStart\":"); json += String(c.dwell_at_start_ms);
  json += F(",\"waitEnd\":"); json += String(c.dwell_at_end_ms);
  json += F(",\"maxRpm\":"); json += String(g_position_servo.settings().max_target_rpm, 2);
  json += F(",\"otaBusy\":");
  json += g_ota_update_in_progress ? F("true") : F("false");
  json += F(",\"stall\":");
  json += g_adrc_stall_fault ? F("true") : F("false");
  json += '}';
  g_web_server.send(status_code, "application/json", json);
}

void sendWebError(int status_code, const char* message) {
  String json = F("{\"error\":\"");
  json += message;
  json += F("\"}");
  g_web_server.send(status_code, "application/json", json);
}

void sendWebSettings() {
  const AdrcPositionSettings& s = g_position_servo.settings();
  String json;
  json.reserve(360);
  json += F("{\"canEdit\":");
  json += (!g_repetitive_motion.running() && !g_position_servo.isActive() &&
           !g_ota_update_in_progress) ? F("true") : F("false");
  json += F(",\"wc\":"); json += String(s.control_bandwidth, 2);
  json += F(",\"wo\":"); json += String(s.observer_bandwidth, 2);
  json += F(",\"b0\":"); json += String(s.plant_gain, 2);
  json += F(",\"maxRpm\":"); json += String(s.max_target_rpm, 2);
  json += F(",\"physRpm\":"); json += String(s.physical_max_rpm, 2);
  json += F(",\"powerLimit\":"); json += String(g_state.power_limit_percent);
  json += F(",\"pwmHz\":"); json += String(g_pwm_frequency_hz);
  json += F(",\"stopWindow\":"); json += String(s.stop_window_deg, 2);
  json += F(",\"stopSamples\":"); json += String(s.samples_to_stop);
  json += F(",\"accelMs\":"); json += String(s.accel_ramp_ms);
  json += F(",\"decelMs\":"); json += String(s.decel_ramp_ms);
  json += F(",\"kickPct\":"); json += String(s.kick_pwm_percent, 1);
  json += F(",\"kickMs\":"); json += String(s.kick_ms);
  json += F(",\"minPwm\":"); json += String(s.minimum_drive_pwm_percent, 1);
  json += F(",\"stallMs\":"); json += String(s.stall_timeout_ms);
  json += F(",\"stallVel\":"); json += String(s.stall_velocity_deg_s, 2);
  json += F(",\"velWindow\":"); json += String(s.velocity_window_ms);
  json += F(",\"velSamples\":"); json += String(s.velocity_num_samples);
  json += '}';
  g_web_server.send(200, "application/json", json);
}

void setupWebControl() {
  if (g_web_server_started) return;

  g_web_server.on("/", HTTP_GET, []() {
    g_web_server.send_P(200, "text/html; charset=utf-8", REPETITIVE_MOTION_WEB_PAGE);
  });
  g_web_server.on("/settings", HTTP_GET, []() {
    g_web_server.send_P(200, "text/html; charset=utf-8", CONTROL_SETTINGS_WEB_PAGE);
  });
  g_web_server.on("/api/settings", HTTP_GET, []() { sendWebSettings(); });
  g_web_server.on("/api/settings", HTTP_POST, []() {
    if (g_repetitive_motion.running() || g_position_servo.isActive() ||
        g_ota_update_in_progress) {
      sendWebError(409, "Pare o motor antes de alterar os ajustes");
      return;
    }
    float wc, wo, b0, max_rpm, phys_rpm, power_limit, pwm_hz;
    float stop_window, stop_samples, accel_ms, decel_ms, kick_pct, kick_ms;
    float min_pwm, stall_ms, stall_vel, vel_window, vel_samples;
    if (!parseWebNumber("wc", &wc) || !parseWebNumber("wo", &wo) ||
        !parseWebNumber("b0", &b0) || !parseWebNumber("maxRpm", &max_rpm) ||
        !parseWebNumber("physRpm", &phys_rpm) ||
        !parseWebNumber("powerLimit", &power_limit) ||
        !parseWebNumber("pwmHz", &pwm_hz) ||
        !parseWebNumber("stopWindow", &stop_window) ||
        !parseWebNumber("stopSamples", &stop_samples) ||
        !parseWebNumber("accelMs", &accel_ms) ||
        !parseWebNumber("decelMs", &decel_ms) ||
        !parseWebNumber("kickPct", &kick_pct) ||
        !parseWebNumber("kickMs", &kick_ms) ||
        !parseWebNumber("minPwm", &min_pwm) ||
        !parseWebNumber("stallMs", &stall_ms) ||
        !parseWebNumber("stallVel", &stall_vel) ||
        !parseWebNumber("velWindow", &vel_window) ||
        !parseWebNumber("velSamples", &vel_samples)) {
      sendWebError(400, "Parametros invalidos ou incompletos");
      return;
    }
    if (wc < 1.0f || wc > 100.0f || wo < 1.0f || wo > 300.0f ||
        b0 < 1.0f || b0 > 2000.0f || max_rpm < 0.1f || max_rpm > 10.0f ||
        phys_rpm < 0.1f || phys_rpm > 10.0f || max_rpm > phys_rpm ||
        power_limit < 0.0f || power_limit > 100.0f ||
        pwm_hz < PWM_MIN_FREQUENCY_HZ || pwm_hz > PWM_MAX_FREQUENCY_HZ ||
        stop_window < 0.2f || stop_window > 20.0f ||
        stop_samples < 1.0f || stop_samples > 20.0f ||
        accel_ms < 50.0f || accel_ms > 2000.0f ||
        decel_ms < 50.0f || decel_ms > 2000.0f ||
        kick_pct < 0.0f || kick_pct > 100.0f ||
        kick_ms < 0.0f || kick_ms > 1000.0f ||
        min_pwm < 0.0f || min_pwm > 45.0f ||
        stall_ms < 100.0f || stall_ms > 10000.0f ||
        stall_vel < 0.1f || stall_vel > 20.0f ||
        vel_window < 20.0f || vel_window > 1000.0f ||
        vel_samples < 2.0f || vel_samples > 20.0f) {
      sendWebError(422, "Valor fora da faixa ou RPM maxima acima da RPM fisica");
      return;
    }

    AdrcPositionSettings s = g_position_servo.settings();
    s.control_bandwidth = wc;
    s.observer_bandwidth = wo;
    s.plant_gain = b0;
    s.max_target_rpm = max_rpm;
    s.physical_max_rpm = phys_rpm;
    s.stop_window_deg = stop_window;
    s.samples_to_stop = (uint16_t)lroundf(stop_samples);
    s.accel_ramp_ms = (uint16_t)lroundf(accel_ms);
    s.decel_ramp_ms = (uint16_t)lroundf(decel_ms);
    s.kick_pwm_percent = kick_pct;
    s.kick_ms = (uint16_t)lroundf(kick_ms);
    s.minimum_drive_pwm_percent = min_pwm;
    s.stall_timeout_ms = (uint16_t)lroundf(stall_ms);
    s.stall_velocity_deg_s = stall_vel;
    s.velocity_window_ms = (uint16_t)lroundf(vel_window);
    s.velocity_num_samples = (uint8_t)lroundf(vel_samples);
    g_position_servo.setSettings(s);
    g_state.power_limit_percent = (uint8_t)lroundf(power_limit);
    if (!setPwmFrequencyHz((uint32_t)lroundf(pwm_hz))) {
      sendWebError(500, "Falha ao aplicar frequencia PWM");
      return;
    }

    RepetitiveMotionConfig c = g_repetitive_motion.config();
    c.start_to_end_rpm = fminf(c.start_to_end_rpm, max_rpm);
    c.end_to_start_rpm = fminf(c.end_to_start_rpm, max_rpm);
    g_repetitive_motion.setConfig(c);
    saveRepetitiveMotionConfig();
    saveControlSettings();
    sendWebSettings();
  });
  g_web_server.on("/api/status", HTTP_GET, []() { sendWebStatus(); });
  g_web_server.on("/api/run", HTTP_POST, []() {
    if (!g_web_server.hasArg("running")) {
      sendWebError(400, "Parametro running ausente");
      return;
    }
    const bool requested = g_web_server.arg("running") == "1";
    if (requested && g_ota_update_in_progress) {
      sendWebError(409, "Atualizacao OTA em andamento");
      return;
    }
    if (requested && !g_as5600.detected()) {
      sendWebError(409, "AS5600 nao detectado");
      return;
    }
    if (requested) g_adrc_stall_fault = false;
    setRepetitiveRunning(requested);
    sendWebStatus();
  });
  g_web_server.on("/api/adjust", HTTP_POST, []() {
    if (g_repetitive_motion.running() || g_position_servo.isActive()) {
      sendWebError(409, "Ajuste permitido somente com o motor parado");
      return;
    }
    if (g_ota_update_in_progress) {
      sendWebError(409, "Atualizacao OTA em andamento");
      return;
    }
    if (!g_as5600.detected()) {
      sendWebError(409, "AS5600 nao detectado");
      return;
    }
    if (!g_web_server.hasArg("target")) {
      sendWebError(400, "Destino ausente");
      return;
    }
    const RepetitiveMotionConfig& c = g_repetitive_motion.config();
    g_adrc_stall_fault = false;
    const String target = g_web_server.arg("target");
    if (target == "start") {
      startAutomaticPositionMove(c.start_deg, c.end_to_start_rpm,
                                  RepetitiveMotionController::Direction::ByNumericComparison);
    } else if (target == "end") {
      startAutomaticPositionMove(c.end_deg, c.start_to_end_rpm,
                                  RepetitiveMotionController::Direction::ByNumericComparison);
    } else {
      sendWebError(400, "Destino invalido");
      return;
    }
    sendWebStatus();
  });
  g_web_server.on("/api/config", HTTP_POST, []() {
    float start = 0.0f, end = 0.0f, rpm_out = 0.0f, rpm_back = 0.0f;
    float wait_start = 0.0f, wait_end = 0.0f;
    if (!parseWebNumber("start", &start) || !parseWebNumber("end", &end) ||
        !parseWebNumber("rpmOut", &rpm_out) || !parseWebNumber("rpmBack", &rpm_back) ||
        !parseWebNumber("waitStart", &wait_start) || !parseWebNumber("waitEnd", &wait_end)) {
      sendWebError(400, "Parametros invalidos ou incompletos");
      return;
    }
    const float max_rpm = g_position_servo.settings().max_target_rpm;
    if (start < 0.0f || start >= 360.0f || end < 0.0f || end >= 360.0f ||
        rpm_out < 0.1f || rpm_out > max_rpm || rpm_back < 0.1f || rpm_back > max_rpm ||
        wait_start < 0.0f || wait_start > 3600000.0f ||
        wait_end < 0.0f || wait_end > 3600000.0f) {
      sendWebError(422, "Valor fora da faixa permitida");
      return;
    }
    RepetitiveMotionConfig c = g_repetitive_motion.config();
    c.start_deg = start;
    c.end_deg = end;
    c.start_to_end_rpm = rpm_out;
    c.end_to_start_rpm = rpm_back;
    c.dwell_at_start_ms = (uint32_t)lroundf(wait_start);
    c.dwell_at_end_ms = (uint32_t)lroundf(wait_end);
    g_repetitive_motion.setConfig(c);
    saveRepetitiveMotionConfig();
    sendWebStatus();
  });
  g_web_server.onNotFound([]() {
    g_web_server.sendHeader("Location", "/", true);
    g_web_server.send(302, "text/plain", "");
  });
  g_web_server.begin();
  g_web_server_started = true;
  Serial.println("WEB: painel disponivel em http://192.168.4.1/");
}

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

void setPwmOutputFrequency(uint8_t pin, uint32_t hz) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  analogWriteFrequency(pin, hz);
#else
  (void)pin;
  analogWriteFrequency(hz);
#endif
}

void setPwmOutputResolution(uint8_t pin, uint8_t bits) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  analogWriteResolution(pin, bits);
#else
  (void)pin;
  analogWriteResolution(bits);
#endif
}

void configurePwmOutputs(uint32_t hz, uint8_t bits) {
  const uint8_t pins[] = {MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_B_IN1, MOTOR_B_IN2};
  for (uint8_t pin : pins) {
    setPwmOutputFrequency(pin, hz);
    setPwmOutputResolution(pin, bits);
  }
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
  const uint8_t pins[] = {MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_B_IN1, MOTOR_B_IN2};
  for (uint8_t pin : pins) {
    setPwmOutputFrequency(pin, hz);
  }
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
    g_move_peak_pwm_adrc_abs = 0;
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
        const AdrcPositionSettings& cfg = g_position_servo.settings();
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
  if (g_position_servo.isStalled()) {
    g_adrc_stall_fault = true;
    setRepetitiveRunning(false);
    Serial.println("ERRO ADRC: stall detectado; motor e ciclo desativados");
    return;
  }
  const int16_t pwm_adrc_abs = (int16_t)abs(g_position_servo.pwmOutput());
  if (pwm_adrc_abs > g_move_peak_pwm_adrc_abs) {
    g_move_peak_pwm_adrc_abs = pwm_adrc_abs;
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
    const float pwm_adrc_pico_pct = clampf((float)g_move_peak_pwm_adrc_abs, 0.0f, 100.0f);
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
    Serial.printf("OK: alvo atingido em %.2f deg (ang_ini=%.2f deg, desl=%.2f deg, desl_abs=%.2f deg, tempo=%u ms, rpm_pico=%.2f, rpm_medio=%.2f, pwm_adrc_pico=%.1f%%, pwm_out_pico=%.1f%%, pwm_out_med=%.1f%%, pwm_sat=%.0f%%)\n",
                  current_deg, ang_ini, desl_signed, desl_abs, elapsed_ms, g_move_peak_rpm_signed, rpm_medio,
                  pwm_adrc_pico_pct, pwm_out_pico_pct, pwm_out_medio_pct, pwm_out_sat_pct);
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

void stopMotorForOta() {
  g_repetitive_motion.stop();
  g_move_tracking_active = false;
  g_state.target_percent = 0.0f;
  g_state.current_percent = 0.0f;
  g_state.drive_phase = DrivePhase::IDLE;
  applyMotorOutput(0);
}

bool setupOtaAndWebServices() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    g_ota_update_in_progress = true;
    stopMotorForOta();
    Serial.println("OTA: atualizacao iniciada; motores desativados");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA: atualizacao concluida; reiniciando");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    const unsigned int percent = total == 0 ? 0 : (progress * 100U) / total;
    Serial.printf("\rOTA: %u%%", percent);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("\nOTA: erro %u; reinicie a placa antes de operar o motor\n",
                  (unsigned int)error);
  });
  ArduinoOTA.begin();
  setupWebControl();

  return true;
}

bool setupOtaAccessPoint() {
  snprintf(OTA_AP_SSID, sizeof(OTA_AP_SSID), "Motor-Control-%02u",
           MOTOR_CONTROL_UNIT_NUMBER);
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(OTA_AP_SSID, OTA_AP_PASSWORD, WIFI_AP_CHANNEL)) {
    Serial.println("OTA: falha ao criar ponto de acesso");
    return false;
  }

  // HT40 pode fazer alguns scanners mostrarem o canal central (por exemplo 9
  // para um AP cujo canal primario e 11). Fixamos HT20 e reafirmamos o canal.
  if (!WiFi.softAPbandwidth(WIFI_BW_HT20)) {
    Serial.println("OTA/WEB: falha ao fixar largura WiFi em 20 MHz");
    WiFi.softAPdisconnect(true);
    return false;
  }
  const esp_err_t channel_result =
    esp_wifi_set_channel(WIFI_AP_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (channel_result != ESP_OK) {
    Serial.printf("OTA/WEB: falha ao fixar canal WiFi: %s\n",
                  esp_err_to_name(channel_result));
    WiFi.softAPdisconnect(true);
    return false;
  }

  uint8_t actual_channel = 0;
  wifi_second_chan_t actual_secondary = WIFI_SECOND_CHAN_NONE;
  wifi_bandwidth_t actual_bandwidth = WIFI_BW_HT20;
  const esp_err_t channel_read_result =
    esp_wifi_get_channel(&actual_channel, &actual_secondary);
  const esp_err_t bandwidth_read_result =
    esp_wifi_get_bandwidth(WIFI_IF_AP, &actual_bandwidth);
  if (channel_read_result != ESP_OK || bandwidth_read_result != ESP_OK ||
      actual_channel != WIFI_AP_CHANNEL || actual_secondary != WIFI_SECOND_CHAN_NONE ||
      actual_bandwidth != WIFI_BW_HT20) {
    Serial.printf("OTA/WEB: configuracao WiFi divergente (canal=%u secundario=%d largura=%d)\n",
                  actual_channel, (int)actual_secondary, (int)actual_bandwidth);
    WiFi.softAPdisconnect(true);
    return false;
  }

  setupOtaAndWebServices();

  Serial.printf("OTA/WEB AP: SSID=%s  canal=%u  largura=20MHz  IP=%s  OTA=3232 WEB=80\n",
                OTA_AP_SSID, actual_channel, WiFi.softAPIP().toString().c_str());
  return true;
}

bool setupStationOrAccessPoint() {
  snprintf(OTA_AP_SSID, sizeof(OTA_AP_SSID), "Motor-Control-%02u",
           MOTOR_CONTROL_UNIT_NUMBER);
  snprintf(OTA_HOSTNAME, sizeof(OTA_HOSTNAME), "as5600-motor-%02u",
           MOTOR_CONTROL_UNIT_NUMBER);

  if (strlen(WIFI_STA_SSID) > 0) {
    Serial.printf("WiFi: conectando a %s", WIFI_STA_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(OTA_HOSTNAME);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASSWORD);
    const uint32_t started_ms = millis();
    while (WiFi.status() != WL_CONNECTED &&
           millis() - started_ms < WIFI_STA_CONNECT_TIMEOUT_MS) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      setupOtaAndWebServices();
      Serial.printf("OTA/WEB STA: SSID=%s  IP=%s  OTA=3232 WEB=80\n",
                    WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
      return true;
    }

    Serial.println("WiFi: rede nao encontrada; iniciando AP de contingencia");
    WiFi.disconnect(true);
    delay(100);
  } else {
    Serial.println("WiFi: credenciais STA ausentes; iniciando AP de contingencia");
  }

  return setupOtaAccessPoint();
}

void startAutomaticPositionMove(
    float target_deg, float rpm, RepetitiveMotionController::Direction direction) {
  float current_deg = 0.0f;
  if (!g_as5600.detected() || !g_as5600.readAngleDeg(&current_deg)) {
    setRepetitiveRunning(false);
    Serial.println("ERRO: ciclo automatico parado; falha ao ler AS5600");
    return;
  }

  const float limited_rpm = clampf(rpm, 0.1f, g_position_servo.settings().max_target_rpm);
  AdrcPositionController::MoveDirection adrc_direction;
  if (direction == RepetitiveMotionController::Direction::ByNumericComparison) {
    adrc_direction = current_deg < target_deg
      ? AdrcPositionController::MoveDirection::Clockwise
      : AdrcPositionController::MoveDirection::CounterClockwise;
  } else {
    adrc_direction = direction == RepetitiveMotionController::Direction::Increasing
      ? AdrcPositionController::MoveDirection::Clockwise
      : AdrcPositionController::MoveDirection::CounterClockwise;
  }
  g_position_servo.startMove(target_deg, limited_rpm, adrc_direction);
  g_position_servo.primeAccumulatedAngle(current_deg);
  g_move_done_reported = false;
  g_move_start_ms = millis();
}

bool isAutomaticPositionMoveActive() {
  return g_position_servo.isActive();
}

void stopAutomaticPositionMove() {
  g_position_servo.cancel();
  g_state.target_percent = 0.0f;
  g_state.current_percent = 0.0f;
  g_state.drive_phase = DrivePhase::IDLE;
  applyMotorOutput(0);
}

void handleOtaMaintenanceMode() {
  if (g_ota_mode_active) {
    ArduinoOTA.handle();
    if (g_web_server_started) g_web_server.handleClient();
    return;
  }

  const bool button_pressed = digitalRead(OTA_BUTTON_PIN) == LOW;
  if (!button_pressed) {
    g_ota_button_pressed = false;
    return;
  }

  const uint32_t now = millis();
  if (!g_ota_button_pressed) {
    g_ota_button_pressed = true;
    g_ota_button_pressed_ms = now;
    return;
  }

  if (now - g_ota_button_pressed_ms < OTA_BUTTON_HOLD_MS) {
    return;
  }

  g_ota_mode_active = true;
  Serial.println("OTA/WEB: ativando AP; controle do motor permanece disponivel");
  if (!setupOtaAccessPoint()) {
    Serial.println("OTA/WEB: falha ao ativar AP");
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
    "poswin","pw","accel_pos","ap","decel_pos","dp",
    "kick_pwm","kpp","kick_rpm","krp","kick_ms","kms","samples_to_stop","sts",
    "maxrpm","mr","physrpm","pr",
    "drpm","default_rpm",
    "adrc_wc","awc","adrc_wo","awo","adrc_b0","ab0",
    "run","running","cycle","cstart","cend","rpmout","rpmback",
    "waitstart","waitend",
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

  const AdrcPositionSettings& s = g_position_servo.settings();
  Serial.printf("ADRC: rated=%.2f rpm  phys=%.2f rpm  stop_win=%.2f deg\n",
                s.max_target_rpm, s.physical_max_rpm, s.stop_window_deg);
  Serial.printf("ADRC: wc=%.2f  wo=%.2f  b0=%.2f\n",
                s.control_bandwidth, s.observer_bandwidth, s.plant_gain);
  Serial.printf("Rampa: accel=%ums  decel=%ums\n", s.accel_ramp_ms, s.decel_ramp_ms);
  Serial.printf("Kick(pos): %.1f%% pwm / %ums   Histerese: %u samples\n",
                s.kick_pwm_percent, s.kick_ms, s.samples_to_stop);
  Serial.printf("Move default: %.2f rpm\n", g_default_move_max_rpm);
  const RepetitiveMotionConfig& cycle = g_repetitive_motion.config();
  Serial.printf("Ciclo: running=%s  fase=%s\n",
                g_repetitive_motion.running() ? "ON" : "OFF",
                g_repetitive_motion.phaseText());
  Serial.printf("  inicio=%.2f deg  fim=%.2f deg  rpm ida=%.2f  rpm volta=%.2f\n",
                cycle.start_deg, cycle.end_deg,
                cycle.start_to_end_rpm, cycle.end_to_start_rpm);
  Serial.printf("  espera inicio=%u ms  espera fim=%u ms\n",
                cycle.dwell_at_start_ms, cycle.dwell_at_end_ms);
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
  Serial.println("  Posicionamento com ADRC:");
  Serial.println("    q | goto <deg> [rpm] [short|cw|ccw]");
  Serial.println("                              -> vai ao alvo (0..360), rpm max e sentido opcionais");
  Serial.println("    cw | ccw <deg> [rpm]      -> atalhos com sentido fixo");
  Serial.println("    inc <ddeg> [rpm]          -> move incremental CW (negativo=CCW, ex.: inc 90 1.8)");
  Serial.println("    dec <ddeg> [rpm]          -> move incremental CCW (negativo=CW, ex.: dec 90 1.8)");
  Serial.println("                              -> suporta multiplas voltas: inc 450 1.8 (1 volta + 90 graus)");
  Serial.println("    qc | cancelmove           -> cancela movimento de posicao");
  Serial.println();
  Serial.println("  Movimento repetitivo:");
  Serial.println("    run | running on|off      -> liga/desliga o ciclo (off para o motor)");
  Serial.println("    cycle <ini> <fim> <rpm_ida> <rpm_volta> <espera_ini_ms> <espera_fim_ms>");
  Serial.println("    cstart | cend <deg>       -> configura os pontos");
  Serial.println("    rpmout | rpmback <rpm>    -> RPM inicio->fim / fim->inicio");
  Serial.println("    waitstart | waitend <ms>  -> pausa no inicio / no fim");
  Serial.println("  Controle ADRC:");
  Serial.println("    awc | adrc_wc <valor>     -> banda do controlador [30]");
  Serial.println("    awo | adrc_wo <valor>     -> banda do observador [100]");
  Serial.println("    ab0 | adrc_b0 <valor>     -> ganho estimado da planta [250]");
  Serial.println();
  Serial.println("  Configuracao de movimento:");
  Serial.println("    pw | poswin <deg>         -> janela de chegada [1.2]");
  Serial.println("    ap | accel_pos <ms>       -> tempo aceleracao [250ms]");
  Serial.println("    dp | decel_pos <ms>       -> tempo desaceleracao [220ms]");
  Serial.println("    kpp | kick_pwm <pct>      -> kick do posicionamento em PWM% [85]");
  Serial.println("    kms | kick_ms <ms>        -> duracao kick [180ms]");
  Serial.println("    sts | samples_to_stop <n> -> leituras para parar [3]");
  Serial.println("    drpm| default_rpm <rpm>   -> RPM padrao para q/inc/dec [1.8]");
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

  auto parseMoveDirection = [](const char* token, AdrcPositionController::MoveDirection* direction) -> bool {
    if (!token || !direction) return false;
    if (!strcmp(token, "short") || !strcmp(token, "shortest") || !strcmp(token, "nearest")) {
      *direction = AdrcPositionController::MoveDirection::Shortest;
      return true;
    }
    if (!strcmp(token, "cw") || !strcmp(token, "horario") || !strcmp(token, "cw+")) {
      *direction = AdrcPositionController::MoveDirection::Clockwise;
      return true;
    }
    if (!strcmp(token, "ccw") || !strcmp(token, "anti") || !strcmp(token, "antihorario") || !strcmp(token, "ccw-")) {
      *direction = AdrcPositionController::MoveDirection::CounterClockwise;
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

  // ── ciclo automatico ───────────────────────────────────────────────────

  if (!strcmp(cmd,"run") || !strcmp(cmd,"running")) {
    char* val = nextArg();
    if (!val) {
      Serial.printf("OK: running=%s  fase=%s\n",
                    g_repetitive_motion.running() ? "on" : "off",
                    g_repetitive_motion.phaseText());
      return;
    }
    if (!strcmp(val,"on") || !strcmp(val,"1") || !strcmp(val,"true")) {
      if (!g_as5600.detected()) {
        printErrorAndPrompt("ERRO: AS5600 nao detectado no I2C");
        return;
      }
      setRepetitiveRunning(true);
      Serial.printf("OK: running=%s\n", g_repetitive_motion.running() ? "on" : "off");
    } else if (!strcmp(val,"off") || !strcmp(val,"0") || !strcmp(val,"false")) {
      setRepetitiveRunning(false);
      Serial.println("OK: running=off; motor parado");
    } else {
      printErrorAndPrompt("ERRO: use run on|off");
    }
    return;
  }

  if (!strcmp(cmd,"cycle")) {
    char* tokens[6];
    tokens[0] = nextArg();
    for (uint8_t i = 1; i < 6; ++i) tokens[i] = strtok_r(nullptr, " \t", &ctx);
    if (!tokens[0]) {
      const RepetitiveMotionConfig& c = g_repetitive_motion.config();
      Serial.printf("OK: cycle %.2f %.2f %.2f %.2f %u %u\n",
                    c.start_deg, c.end_deg, c.start_to_end_rpm, c.end_to_start_rpm,
                    c.dwell_at_start_ms, c.dwell_at_end_ms);
      return;
    }
    for (uint8_t i = 1; i < 6; ++i) {
      if (!tokens[i]) {
        printErrorAndPrompt("ERRO: use cycle <ini> <fim> <rpm_ida> <rpm_volta> <espera_ini_ms> <espera_fim_ms>");
        return;
      }
    }
    RepetitiveMotionConfig c = g_repetitive_motion.config();
    c.start_deg = (float)atof(tokens[0]);
    c.end_deg = (float)atof(tokens[1]);
    const float max_rpm = g_position_servo.settings().max_target_rpm;
    c.start_to_end_rpm = clampf((float)atof(tokens[2]), 0.1f, max_rpm);
    c.end_to_start_rpm = clampf((float)atof(tokens[3]), 0.1f, max_rpm);
    c.dwell_at_start_ms = (uint32_t)constrain(atol(tokens[4]), 0L, 3600000L);
    c.dwell_at_end_ms = (uint32_t)constrain(atol(tokens[5]), 0L, 3600000L);
    g_repetitive_motion.setConfig(c);
    saveRepetitiveMotionConfig();
    Serial.println("OK: ciclo configurado");
    return;
  }

  if (!strcmp(cmd,"cstart") || !strcmp(cmd,"cend") ||
      !strcmp(cmd,"rpmout") || !strcmp(cmd,"rpmback") ||
      !strcmp(cmd,"waitstart") || !strcmp(cmd,"waitend")) {
    char* val = nextArg();
    RepetitiveMotionConfig c = g_repetitive_motion.config();
    if (!val) {
      if (!strcmp(cmd,"cstart")) Serial.printf("OK: cstart=%.2f deg\n", c.start_deg);
      else if (!strcmp(cmd,"cend")) Serial.printf("OK: cend=%.2f deg\n", c.end_deg);
      else if (!strcmp(cmd,"rpmout")) Serial.printf("OK: rpmout=%.2f rpm\n", c.start_to_end_rpm);
      else if (!strcmp(cmd,"rpmback")) Serial.printf("OK: rpmback=%.2f rpm\n", c.end_to_start_rpm);
      else if (!strcmp(cmd,"waitstart")) Serial.printf("OK: waitstart=%u ms\n", c.dwell_at_start_ms);
      else Serial.printf("OK: waitend=%u ms\n", c.dwell_at_end_ms);
      return;
    }
    if (!strcmp(cmd,"cstart")) c.start_deg = (float)atof(val);
    else if (!strcmp(cmd,"cend")) c.end_deg = (float)atof(val);
    else if (!strcmp(cmd,"rpmout")) c.start_to_end_rpm = clampf((float)atof(val), 0.1f, g_position_servo.settings().max_target_rpm);
    else if (!strcmp(cmd,"rpmback")) c.end_to_start_rpm = clampf((float)atof(val), 0.1f, g_position_servo.settings().max_target_rpm);
    else if (!strcmp(cmd,"waitstart")) c.dwell_at_start_ms = (uint32_t)constrain(atol(val), 0L, 3600000L);
    else c.dwell_at_end_ms = (uint32_t)constrain(atol(val), 0L, 3600000L);
    g_repetitive_motion.setConfig(c);
    saveRepetitiveMotionConfig();
    Serial.println("OK: ciclo atualizado");
    return;
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

  if (!strcmp(cmd,"awc") || !strcmp(cmd,"adrc_wc") ||
      !strcmp(cmd,"awo") || !strcmp(cmd,"adrc_wo") ||
      !strcmp(cmd,"ab0") || !strcmp(cmd,"adrc_b0")) {
    char* val = nextArg();
    AdrcPositionSettings s = g_position_servo.settings();
    float* setting = !strcmp(cmd,"awc") || !strcmp(cmd,"adrc_wc")
                       ? &s.control_bandwidth
                       : (!strcmp(cmd,"awo") || !strcmp(cmd,"adrc_wo")
                            ? &s.observer_bandwidth : &s.plant_gain);
    const char* name = !strcmp(cmd,"awc") || !strcmp(cmd,"adrc_wc")
                         ? "wc" : ((!strcmp(cmd,"awo") || !strcmp(cmd,"adrc_wo")) ? "wo" : "b0");
    if (!val) {
      Serial.printf("OK: ADRC %s=%.3f\n", name, *setting);
      return;
    }
    if (g_position_servo.isActive() || g_repetitive_motion.running()) {
      printErrorAndPrompt("ERRO: pare o motor antes de alterar ADRC");
      return;
    }
    float parsed = 0.0f;
    if (!parseFloatToken(val, &parsed) || parsed <= 0.0f) {
      printErrorAndPrompt("ERRO: ganho ADRC deve ser positivo");
      return;
    }
    if (setting == &s.control_bandwidth) parsed = clampf(parsed, 1.0f, 100.0f);
    else if (setting == &s.observer_bandwidth) parsed = clampf(parsed, 1.0f, 300.0f);
    else parsed = clampf(parsed, 1.0f, 2000.0f);
    *setting = parsed;
    g_position_servo.setSettings(s);
    saveAdrcSettings();
    Serial.printf("OK: ADRC %s=%.3f (persistente)\n", name, parsed);
    return;
  }

  if (!strcmp(cmd,"q") || !strcmp(cmd,"goto") || !strcmp(cmd,"go")
      || !strcmp(cmd,"move") || !strcmp(cmd,"cw") || !strcmp(cmd,"ccw")) {
    if (!g_as5600.detected()) {
      printErrorAndPrompt("ERRO: AS5600 nao detectado no I2C");
      return;
    }
    setRepetitiveRunning(false);

    char* deg_token = nextArg();
    char* extra1 = strtok_r(nullptr, " \t", &ctx);
    char* extra2 = strtok_r(nullptr, " \t", &ctx);
    if (!deg_token) {
      printErrorAndPrompt("ERRO: use q <deg> [rpm] [short|cw|ccw]");
      return;
    }

    const float target_deg = (float)atof(deg_token);
    const AdrcPositionSettings& s = g_position_servo.settings();
    float vmax_rpm = g_default_move_max_rpm;
    AdrcPositionController::MoveDirection direction = AdrcPositionController::MoveDirection::Shortest;

    if (!strcmp(cmd, "cw")) direction = AdrcPositionController::MoveDirection::Clockwise;
    if (!strcmp(cmd, "ccw")) direction = AdrcPositionController::MoveDirection::CounterClockwise;

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
      direction == AdrcPositionController::MoveDirection::Clockwise ? "cw" :
      direction == AdrcPositionController::MoveDirection::CounterClockwise ? "ccw" : "short";
    Serial.printf("OK: move alvo=%.2f deg  vmax=%.2f rpm  dir=%s\n", target_deg, vmax_rpm, direction_text);
    return;
  }

  if (!strcmp(cmd,"inc")) {
    if (!g_as5600.detected()) {
      printErrorAndPrompt("ERRO: AS5600 nao detectado no I2C");
      return;
    }
    setRepetitiveRunning(false);

    char* delta_token = nextArg();
    char* rpm_token = strtok_r(nullptr, " \t", &ctx);
    if (!delta_token) {
      printErrorAndPrompt("ERRO: use inc <ddeg> [rpm]");
      return;
    }

    float delta_deg = (float)atof(delta_token);
    const AdrcPositionSettings& s = g_position_servo.settings();
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
    const AdrcPositionController::MoveDirection direction =
      delta_deg >= 0.0f ? AdrcPositionController::MoveDirection::Clockwise
                        : AdrcPositionController::MoveDirection::CounterClockwise;

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
    setRepetitiveRunning(false);

    char* delta_token = nextArg();
    char* rpm_token = strtok_r(nullptr, " \t", &ctx);
    if (!delta_token) {
      printErrorAndPrompt("ERRO: use dec <ddeg> [rpm]");
      return;
    }

    float delta_deg = (float)atof(delta_token);
    const AdrcPositionSettings& s = g_position_servo.settings();
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
    const AdrcPositionController::MoveDirection direction =
      delta_deg >= 0.0f ? AdrcPositionController::MoveDirection::CounterClockwise
                        : AdrcPositionController::MoveDirection::Clockwise;

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
    setRepetitiveRunning(false);
    Serial.println("OK: movimento/ciclo cancelado");
    return;
  }

  // ── Configuração de movimento ───────────────────────────────────────────

  if (!strcmp(cmd,"pw") || !strcmp(cmd,"poswin")) {
    char* val = nextArg();
    AdrcPositionSettings s = g_position_servo.settings();
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
    AdrcPositionSettings s = g_position_servo.settings();
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
    AdrcPositionSettings s = g_position_servo.settings();
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
    AdrcPositionSettings s = g_position_servo.settings();
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
    AdrcPositionSettings s = g_position_servo.settings();
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
    AdrcPositionSettings s = g_position_servo.settings();
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
    AdrcPositionSettings s = g_position_servo.settings();
    if (!val) {
      Serial.printf("OK: maxrpm=%.2f\n", s.max_target_rpm);
      return;
    }
    const float new_max = clampf((float)atof(val), 1.0f, 200.0f);
    s.max_target_rpm = new_max;
    g_position_servo.setSettings(s);
    Serial.printf("OK: maxrpm=%.2f\n", s.max_target_rpm);
    return;
  }

  if (!strcmp(cmd,"drpm") || !strcmp(cmd,"default_rpm")) {
    char* val = nextArg();
    const AdrcPositionSettings& s = g_position_servo.settings();
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
    AdrcPositionSettings s = g_position_servo.settings();
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
    setRepetitiveRunning(false);
    g_state.target_percent  = 0.0f;
    g_state.current_percent = 0.0f;
    g_state.drive_phase     = DrivePhase::IDLE;
    applyMotorOutput(0);
    Serial.println("OK: stop");
    return;
  }

  if (!strcmp(cmd,"brake") || !strcmp(cmd,"b")) {
    setRepetitiveRunning(false);
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
    setRepetitiveRunning(false);
    char* val = nextArg();
    if (!val) { printErrorAndPrompt("ERRO: use p <0..100>"); return; }
    setTargetFromUnsignedPercent((float)atof(val));
    Serial.printf("OK: pwm %.1f%% (%s)\n",
                  fabsf(g_state.target_percent),
                  g_state.direction_sign > 0 ? "fwd" : "rev");
    return;
  }

  if (!strcmp(cmd,"set") || !strcmp(cmd,"v")) {
    setRepetitiveRunning(false);
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
    setRepetitiveRunning(false);
    g_state.direction_sign *= -1;
    g_state.target_percent  = -g_state.target_percent;
    Serial.printf("OK: rev -> %s\n",
                  g_state.direction_sign > 0 ? "fwd" : "rev");
    return;
  }

  if (!strcmp(cmd,"dir") || !strcmp(cmd,"d")) {
    setRepetitiveRunning(false);
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

  // No modo de posicionamento, usa diretamente a saida calculada pelo ADRC
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
  configurePwmOutputs(g_pwm_frequency_hz, PWM_RESOLUTION_BITS);

  AdrcPositionSettings pos_settings;

  // ADRC: valores iniciais extraidos do prototipo V2.5. A escala de b0 segue
  // PWM 8-bit internamente, embora a ponte H continue recebendo percentual.
  pos_settings.control_bandwidth = 25.0f;
  pos_settings.observer_bandwidth = 80.0f;
  pos_settings.plant_gain = 250.0f;
  
  pos_settings.max_target_rpm = DEFAULT_MAX_TARGET_RPM;
  pos_settings.physical_max_rpm = 3.0f;
  pos_settings.stop_window_deg = 1.0f;
  pos_settings.accel_ramp_ms = 250;
  pos_settings.decel_ramp_ms = 220;
  pos_settings.kick_pwm_percent = 85.0f;
  pos_settings.kick_ms = 180;
  pos_settings.samples_to_stop = 3;
  pos_settings.velocity_window_ms = 400;
  pos_settings.velocity_num_samples = 8;
  pos_settings.minimum_drive_pwm_percent = 24.0f;
  
  g_position_servo.setSettings(pos_settings);
  loadRepetitiveMotionPreferences();
  setPwmFrequencyHz(g_pwm_frequency_hz);
  configurePwmOutputs(g_pwm_frequency_hz, PWM_RESOLUTION_BITS);

  g_as5600.begin(Wire, I2C_SDA_PIN, I2C_SCL_PIN);

  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(OTA_BUTTON_PIN, INPUT_PULLUP);

  setMotorSignedPwm(0, MOTOR_A_IN1, MOTOR_A_IN2);
  setMotorSignedPwm(0, MOTOR_B_IN1, MOTOR_B_IN2);

  WiFi.mode(WIFI_OFF);

  // Tenta a rede local primeiro. Se ela nao estiver disponivel, cria o AP de
  // contingencia. Somente uma transferencia OTA interrompe o motor.
  g_ota_mode_active = setupStationOrAccessPoint();

  Serial.println("\n=== Motor PWM Tester ===");
  Serial.println("Placa: ESP32-C3 Super Mini  |  Motor padrao: IN3/IN4");
  Serial.printf("PWM: freq=%u Hz  resolucao=%u bits\n", g_pwm_frequency_hz, PWM_RESOLUTION_BITS);
  Serial.printf("I2C: SDA=%u SCL=%u\n", I2C_SDA_PIN, I2C_SCL_PIN);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi: conectado a %s  painel=http://%s/\n",
                  WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
  } else {
    Serial.printf("WiFi: AP de contingencia  SSID=%s  canal=%u  painel=http://192.168.4.1/\n",
                  OTA_AP_SSID, WIFI_AP_CHANNEL);
  }
  if (g_as5600.detected()) {
    Serial.printf("AS5600 detectado no endereco 0x%02X\n", g_as5600.address());
  } else {
    Serial.printf("AS5600 NAO detectado no endereco 0x%02X\n", g_as5600.address());
  }
  Serial.println("ADRC pronto (motor nominal 2 rpm)");

  if (g_repetitive_run_on_boot) {
    if (g_as5600.detected()) {
      Serial.println("Ciclo persistente: running=on; iniciando homing no ponto inicial");
      setRepetitiveRunning(true, false);
    } else {
      Serial.println("ERRO: ciclo salvo como running=on, mas AS5600 nao foi detectado");
    }
  }

  Serial.println("Serial: comandos desativados; interface disponivel apenas via web");
  g_serial_prompt_pending = false;
}

void loop() {
  handleOtaMaintenanceMode();

  // Mantem a serial exclusivamente para logs; qualquer entrada e descartada.
  while (Serial.available() > 0) Serial.read();
  g_repetitive_motion.update(millis());
  updatePositionMoveControl();
  updateRampControl();

}
