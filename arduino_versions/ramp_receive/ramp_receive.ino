//----------------------------------------------------------------------
// ESP32-S2 Joystick Receiver (ESP-NOW + L298N Motor Control) - VERSÃO RAMPA
// Arquivo separado para uso no Arduino IDE
//
// ⚡ VERSÃO COM RAMPAS ASSIMÉTRICAS
// Implementa controle suave dos motores com:
// - Aceleração suave (rampa de ~200ms) - evita trancos
// - Frenagem rápida (rampa de ~50ms) - resposta rápida
//
// As rampas são aplicadas independentemente a cada motor
//----------------------------------------------------------------------

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

//----------------------------------------------------------------------
// CONFIGURAÇÕES DO RECEPTOR
//----------------------------------------------------------------------

// Range de saída normalizada do joystick (para PWM 8-bit)
#define FATOR_DE_POTENCIA 0.75
#define AXIS_MAX_OUTPUT 255 * FATOR_DE_POTENCIA
#define AXIS_MIN_OUTPUT -255 * FATOR_DE_POTENCIA
#define AXIS_CENTER_OUTPUT 0

// Definições dos pinos para ESP32-S2 Mini (RECEPTOR)
#define LED_PIN 15          // LED interno para status

// LED ativo em HIGH no ESP32-S2 Mini usado neste projeto
#define LED_ON  HIGH
#define LED_OFF LOW

// Definições dos pinos para L298N (receptor)
#define MOTOR_RIGHT_IN1   11  // IN1 - PWM+Direção motor direito
#define MOTOR_RIGHT_IN2    9  // IN2 - PWM+Direção motor direito
#define MOTOR_LEFT_IN1     7  // IN3 - PWM+Direção motor esquerdo
#define MOTOR_LEFT_IN2     5  // IN4 - PWM+Direção motor esquerdo

//----------------------------------------------------------------------
// CONFIGURAÇÕES DE RAMPA
//----------------------------------------------------------------------

// Tempo em ms para atingir velocidade máxima durante aceleração
#define ACCELERATION_TIME 500  // Aceleração lenta (sem trancos)

// Tempo em ms para parar completamente durante frenagem
#define DECELERATION_TIME 150   // Frenagem rápida (resposta rápida)

//----------------------------------------------------------------------
// ESTRUTURAS DE DADOS
//----------------------------------------------------------------------

// Estrutura de dados para receber
typedef struct {
  int16_t joystick_x;     // Valor X do joystick (0-4095)
  int16_t joystick_y;     // Valor Y do joystick (0-4095) 
  bool button_pressed;    // Estado do botão
  uint32_t timestamp;     // Timestamp para debugging
} JoystickData;

// Configuração de calibração de eixo
typedef struct {
  int16_t center;           // Centro detectado
  int16_t min_value;        // Mínimo útil
  int16_t max_value;        // Máximo útil
  int16_t center_deadzone;  // Zona morta centro
  int16_t edge_deadzone;    // Zona morta extremos
} AxisConfig;

// Dados normalizados do joystick
typedef struct {
  int16_t x;              // -255 a +255
  int16_t y;              // -255 a +255
  bool button;            // true/false
  uint32_t timestamp;     // Timestamp
} NormalizedJoystick;

// Regiões do joystick
enum JoystickRegion {
  CENTER,       // Dentro da deadzone
  LEFT,         // X < center - deadzone  
  RIGHT,        // X > center + deadzone
  UP,           // Y > center + deadzone
  DOWN,         // Y < center - deadzone
  UP_LEFT,      // Combinações diagonais
  UP_RIGHT,
  DOWN_LEFT, 
  DOWN_RIGHT
};

// Comandos para os motores
typedef struct {
  int16_t left_motor;     // -255 a +255 (negativo = trás, positivo = frente)
  int16_t right_motor;    // -255 a +255 (negativo = trás, positivo = frente)
  bool enable;            // true = motores ativos
  uint32_t timestamp;     // Para timeout
} MotorCommands;

// Estado de rampa para cada motor
typedef struct {
  int16_t current_speed;      // Velocidade atual após rampa
  int16_t target_speed;       // Velocidade alvo
  unsigned long start_time;   // Quando começou a rampa atual (0 = parado)
  unsigned long ramp_duration;// Duração da rampa (ACCELERATION_TIME ou DECELERATION_TIME)
} MotorRampState;

//----------------------------------------------------------------------
// VARIÁVEIS GLOBAIS
//----------------------------------------------------------------------

// Variáveis do receptor
JoystickData receivedData;
NormalizedJoystick normalizedData;
MotorCommands motorCommands;
unsigned long lastReceiveTime = 0;
const unsigned long RECEIVE_TIMEOUT = 1000; // Timeout de 1 segundo
bool dataReceived = false;
int packetsReceived = 0;
bool espnowReady = false;

// Estados das rampas para cada motor
MotorRampState leftMotorRamp = {0, 0, 0, 0};
MotorRampState rightMotorRamp = {0, 0, 0, 0};

// Configuração de calibração dos eixos (valores iniciais)
AxisConfig x_axis_config = {
  .center = 2650,           // Centro teórico
  .min_value = 0,           // Mínimo ADC
  .max_value = 4095,        // Máximo ADC
  .center_deadzone = 300,   // Zona morta centro
  .edge_deadzone = 50       // Zona morta extremos
};

AxisConfig y_axis_config = {
  .center = 2650,
  .min_value = 0,
  .max_value = 4095,
  .center_deadzone = 300,
  .edge_deadzone = 50
};

//----------------------------------------------------------------------
// FUNÇÕES DE APOIO
//----------------------------------------------------------------------

// Função para mapear eixo do joystick
int16_t mapAxisInt(int16_t raw, AxisConfig config) {
  // Zona morta do centro
  if (abs(raw - config.center) <= config.center_deadzone) {
    return AXIS_CENTER_OUTPUT;
  }
  
  if (raw > config.center) {
    // Lado positivo
    int16_t useful_min = config.center + config.center_deadzone;
    int16_t useful_max = config.max_value - config.edge_deadzone;
    
    if (raw >= useful_max) return AXIS_MAX_OUTPUT;
    return map(raw, useful_min, useful_max, AXIS_CENTER_OUTPUT, AXIS_MAX_OUTPUT);
  } else {
    // Lado negativo
    int16_t useful_max = config.center - config.center_deadzone;
    int16_t useful_min = config.min_value + config.edge_deadzone;
    
    if (raw <= useful_min) return AXIS_MIN_OUTPUT;
    return map(raw, useful_min, useful_max, AXIS_MIN_OUTPUT, AXIS_CENTER_OUTPUT);
  }
}

// Função para normalizar dados do joystick
void normalizeJoystickData(JoystickData* raw_data, NormalizedJoystick* normalized) {
  normalized->x = mapAxisInt(raw_data->joystick_x, x_axis_config);
  normalized->y = mapAxisInt(raw_data->joystick_y, y_axis_config);
  normalized->button = raw_data->button_pressed;
  normalized->timestamp = raw_data->timestamp;
}

// Função para detectar região do joystick
JoystickRegion getJoystickRegion(NormalizedJoystick* data) {
  const int16_t threshold = 30;  // Threshold para considerar movimento
  
  bool x_center = (abs(data->x) <= threshold);
  bool y_center = (abs(data->y) <= threshold);
  
  if (x_center && y_center) return CENTER;
  
  if (!x_center && !y_center) {
    // Diagonais
    if (data->x > 0 && data->y > 0) return UP_RIGHT;
    if (data->x > 0 && data->y < 0) return DOWN_RIGHT;
    if (data->x < 0 && data->y > 0) return UP_LEFT;
    if (data->x < 0 && data->y < 0) return DOWN_LEFT;
  }
  
  // Direções puras
  if (data->x > 0) return RIGHT;
  if (data->x < 0) return LEFT;
  if (data->y > 0) return UP;
  if (data->y < 0) return DOWN;
  
  return CENTER;
}

// Função para converter região em string
const char* regionToString(JoystickRegion region) {
  switch(region) {
    case CENTER: return "CENTER";
    case LEFT: return "LEFT";
    case RIGHT: return "RIGHT";
    case UP: return "UP";
    case DOWN: return "DOWN";
    case UP_LEFT: return "UP_LEFT";
    case UP_RIGHT: return "UP_RIGHT";
    case DOWN_LEFT: return "DOWN_LEFT";
    case DOWN_RIGHT: return "DOWN_RIGHT";
    default: return "UNKNOWN";
  }
}

// Função para calcular magnitude do joystick
uint8_t calculateMagnitude(NormalizedJoystick* data) {
  // Calcular distância euclidiana do centro
  float magnitude = sqrt((float)(data->x * data->x) + (float)(data->y * data->y));
  
  // Limitar a 255 (máximo PWM)
  return (magnitude > 255) ? 255 : (uint8_t)magnitude;
}

// Função para mapear região do joystick para comandos de motor (Tank Drive)
void mapJoystickToMotors(NormalizedJoystick* joystick, MotorCommands* motors) {
  JoystickRegion region = getJoystickRegion(joystick);
  uint8_t speed = calculateMagnitude(joystick);
  
  // Inicializar comandos
  motors->left_motor = 0;
  motors->right_motor = 0;
  motors->enable = true;
  motors->timestamp = joystick->timestamp;
  
  // Mapear região para comandos de motor
  switch(region) {
    case CENTER:
      // Parado
      motors->enable = false;
      break;
      
    case UP:
      // Ambos motores para frente
      motors->left_motor = speed;
      motors->right_motor = speed;
      break;
      
    case DOWN:
      // Ambos motores para trás
      motors->left_motor = -speed;
      motors->right_motor = -speed;
      break;
      
    case LEFT:
      // Giro no lugar para esquerda (esquerdo trás + direito frente)
      motors->left_motor = -speed;
      motors->right_motor = speed;
      break;
      
    case RIGHT:
      // Giro no lugar para direita (esquerdo frente + direito trás)
      motors->left_motor = speed;
      motors->right_motor = -speed;
      break;
      
    case UP_LEFT:
      // Curva suave esquerda (apenas motor direito frente)
      motors->left_motor = 0;
      motors->right_motor = speed;
      break;
      
    case UP_RIGHT:
      // Curva suave direita (apenas motor esquerdo frente)
      motors->left_motor = speed;
      motors->right_motor = 0;
      break;
      
    case DOWN_LEFT:
      // Curva suave esquerda em ré (apenas motor direito trás)
      motors->left_motor = 0;
      motors->right_motor = -speed;
      break;
      
    case DOWN_RIGHT:
      // Curva suave direita em ré (apenas motor esquerdo trás)
      motors->left_motor = -speed;
      motors->right_motor = 0;
      break;
  }
}

//----------------------------------------------------------------------
// FUNÇÕES DE CONTROLE COM RAMPAS
//----------------------------------------------------------------------

// Função para aplicar rampa assimétrica a um motor
// Aceleração lenta (ACCELERATION_TIME) vs Frenagem rápida (DECELERATION_TIME)
int16_t applyMotorRamp(MotorRampState* ramp, int16_t target_speed, unsigned long current_time) {
  // Se o alvo mudou, iniciar nova rampa
  if (target_speed != ramp->target_speed) {
    ramp->target_speed = target_speed;
    ramp->start_time = current_time;
    
    // Determinar tempo de rampa baseado se é aceleração ou frenagem
    // Aceleração: se a magnitude aumenta
    // Frenagem: se a magnitude diminui (ou muda de direção)
    if (abs(target_speed) > abs(ramp->current_speed)) {
      ramp->ramp_duration = ACCELERATION_TIME;
    } else {
      ramp->ramp_duration = DECELERATION_TIME;
    }
  }
  
  // Se velocidade alvo já foi atingida, retornar alvo
  if (ramp->current_speed == ramp->target_speed) {
    return ramp->current_speed;
  }
  
  // Calcular tempo decorrido desde início da rampa
  unsigned long elapsed = current_time - ramp->start_time;
  
  // Se passou todo o tempo de rampa, atingiu o alvo
  if (elapsed >= ramp->ramp_duration) {
    ramp->current_speed = ramp->target_speed;
    return ramp->current_speed;
  }
  
  // Interpolação linear: progress de 0 a 1
  // speed = current + (target - current) * (elapsed / duration)
  int32_t speed_change = (int32_t)ramp->target_speed - (int32_t)ramp->current_speed;
  int32_t interpolated = ((int32_t)ramp->current_speed * 1000) + 
                         (speed_change * (int32_t)elapsed * 1000) / (int32_t)ramp->ramp_duration;
  
  // Converter de volta para int16_t (dividir por 1000 para desfazer escala)
  int16_t new_speed = (int16_t)(interpolated / 1000);
  
  // Proteção contra overshooting
  if (speed_change > 0) {
    // Aceleração: não ultrapassar alvo
    ramp->current_speed = (new_speed > ramp->target_speed) ? ramp->target_speed : new_speed;
  } else {
    // Frenagem: não passar do alvo
    ramp->current_speed = (new_speed < ramp->target_speed) ? ramp->target_speed : new_speed;
  }
  
  return ramp->current_speed;
}

// Função para controlar motor individual L298N com rampa (ENA/ENB jumpeados)
void setMotorWithRamp(int16_t target_speed, uint8_t in1_pin, uint8_t in2_pin, 
                      MotorRampState* ramp, unsigned long current_time) {
  // Aplicar rampa para obter velocidade atual
  int16_t ramped_speed = applyMotorRamp(ramp, target_speed, current_time);
  
  if (ramped_speed == 0) {
    // Motor parado
    analogWrite(in1_pin, 0);
    analogWrite(in2_pin, 0);
  } else if (ramped_speed > 0) {
    // Motor para frente: IN1 = PWM, IN2 = LOW
    analogWrite(in1_pin, abs(ramped_speed));
    analogWrite(in2_pin, 0);
  } else {
    // Motor para trás: IN1 = LOW, IN2 = PWM
    analogWrite(in1_pin, 0);
    analogWrite(in2_pin, abs(ramped_speed));
  }
}

// Função para controlar ambos os motores L298N com rampas
void controlMotorsWithRamps(MotorCommands* commands, unsigned long current_time) {
  if (commands->enable) {
    setMotorWithRamp(commands->left_motor, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, 
                     &leftMotorRamp, current_time);
    setMotorWithRamp(commands->right_motor, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, 
                     &rightMotorRamp, current_time);
  } else {
    // Parar ambos os motores (usando rampas)
    setMotorWithRamp(0, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, &leftMotorRamp, current_time);
    setMotorWithRamp(0, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, &rightMotorRamp, current_time);
  }
}

//----------------------------------------------------------------------
// FUNÇÕES DE CALLBACK ESP-NOW
//----------------------------------------------------------------------

// Callback quando dados são recebidos (receptor)
void OnDataReceived(const esp_now_recv_info *recv_info, const uint8_t *data, int len) {
  unsigned long current_time = millis();
  
  // Verificar se o tamanho dos dados está correto
  if (len == sizeof(JoystickData)) {
    memcpy(&receivedData, data, sizeof(JoystickData));
    lastReceiveTime = current_time;
    dataReceived = true;
    packetsReceived++;
    
    // Log no Serial com MAC do transmissor (acessar via recv_info->src_addr)
    Serial.printf("[%d] Recebido de %02X:%02X:%02X:%02X:%02X:%02X:\n",
                  packetsReceived,
                  recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2], 
                  recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
    
    // Normalizar dados
    normalizeJoystickData(&receivedData, &normalizedData);
    
    // Detectar região
    JoystickRegion region = getJoystickRegion(&normalizedData);
    
    // Mapear para comandos de motor
    mapJoystickToMotors(&normalizedData, &motorCommands);
    
    // Controlar motores COM RAMPAS (antes do delay do LED para não corromper o timestamp)
    controlMotorsWithRamps(&motorCommands, current_time);
    
    // Piscar LED para indicar recebimento (após controle dos motores)
    digitalWrite(LED_PIN, LED_ON);
    delay(10);
    digitalWrite(LED_PIN, LED_OFF);
    
    // Exibir dados raw e normalizados
    Serial.printf("  RAW    -> X:%4d Y:%4d BTN:%s\n",
                  receivedData.joystick_x,
                  receivedData.joystick_y,
                  receivedData.button_pressed ? "PRESS" : "FREE");
    
    Serial.printf("  MAPPED -> X:%4d Y:%4d BTN:%s REGION:%s\n",
                  normalizedData.x,
                  normalizedData.y,
                  normalizedData.button ? "PRESS" : "FREE",
                  regionToString(region));
    
    Serial.printf("  MOTORS -> L_TARGET:%4d L_ACTUAL:%4d | R_TARGET:%4d R_ACTUAL:%4d SPEED:%d\n",
                  motorCommands.left_motor, leftMotorRamp.current_speed,
                  motorCommands.right_motor, rightMotorRamp.current_speed,
                  calculateMagnitude(&normalizedData));
    
    Serial.println();
  } else {
    Serial.printf("Erro: Tamanho incorreto recebido: %d bytes\n", len);
  }
}

//----------------------------------------------------------------------
// FUNÇÕES DE INICIALIZAÇÃO
//----------------------------------------------------------------------

// Função para inicializar ESP-NOW (receptor)
bool initESPNow_RX() {
  WiFi.disconnect();
  
  // Configuração de RF equilibrada (temperatura menor)
  WiFi.setSleep(WIFI_PS_MIN_MODEM);      // Power save habilitado
  
  // Configurar canal fixo para maior estabilidade
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  
  // Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao inicializar ESP-NOW");
    return false;
  }
  
  // Registrar callback de recebimento
  esp_now_register_recv_cb(OnDataReceived);
  
  Serial.println("ESP-NOW inicializado com sucesso (RX)");
  Serial.println("Configuração de RF aplicada:");
  Serial.println("- Potência TX: padrão do sistema");
  Serial.println("- Modem sleep: MIN_MODEM");
  Serial.println("- Canal: 1 (fixo)");
  Serial.println("Aguardando dados do transmissor...");
  return true;
}

//----------------------------------------------------------------------
// SETUP E LOOP PRINCIPAL
//----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(2000);  // Delay para conseguir ver o MAC address
  Serial.println("=== ESP32-S2 Joystick Receptor (VERSÃO RAMPA) ===");
  Serial.println("⚡ RAMPAS ASSIMÉTRICAS ATIVADAS");
  Serial.printf("   Aceleração: %dms | Frenagem: %dms\n", ACCELERATION_TIME, DECELERATION_TIME);
  
  // Configurar WiFi para obter MAC address
  WiFi.mode(WIFI_STA);
  
  // Exibir MAC address de forma destacada
  Serial.println("\n*** CONFIGURAÇÃO DE PAREAMENTO ***");
  Serial.println("MAC Address deste RECEPTOR:");
  Serial.println(WiFi.macAddress());
  Serial.println("\nConfigure este MAC no TRANSMISSOR:");
  Serial.println("uint8_t receiverMAC[] = {0x" + 
                 WiFi.macAddress().substring(0,2) + ", 0x" +
                 WiFi.macAddress().substring(3,5) + ", 0x" +
                 WiFi.macAddress().substring(6,8) + ", 0x" +
                 WiFi.macAddress().substring(9,11) + ", 0x" +
                 WiFi.macAddress().substring(12,14) + ", 0x" +
                 WiFi.macAddress().substring(15,17) + "};");
  Serial.println("*********************************\n");
  
  // Configurar pinos
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF); // LED desligado inicialmente
  
  // Configurar pinos dos motores L298N (apenas IN1-IN4, ENA/ENB jumpeados)
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  
  // Parar motores inicialmente (sem rampa)
  analogWrite(MOTOR_LEFT_IN1, 0);
  analogWrite(MOTOR_LEFT_IN2, 0);
  analogWrite(MOTOR_RIGHT_IN1, 0);
  analogWrite(MOTOR_RIGHT_IN2, 0);
  
  // Inicializar estados das rampas
  leftMotorRamp.current_speed = 0;
  leftMotorRamp.target_speed = 0;
  leftMotorRamp.start_time = 0;
  leftMotorRamp.ramp_duration = 0;
  
  rightMotorRamp.current_speed = 0;
  rightMotorRamp.target_speed = 0;
  rightMotorRamp.start_time = 0;
  rightMotorRamp.ramp_duration = 0;
  
  // Inicializar ESP-NOW para recepção
  espnowReady = initESPNow_RX();
  
  if (espnowReady) {
    Serial.println("Receptor pronto!");
    // Piscar LED 3 vezes para indicar inicialização OK
    for(int i = 0; i < 3; i++) {
      digitalWrite(LED_PIN, LED_ON);  // Liga LED
      delay(200);
      digitalWrite(LED_PIN, LED_OFF); // Desliga LED
      delay(200);
    }
  } else {
    Serial.println("Falha na inicialização!");
    // LED fixo indica erro (ligado)
    digitalWrite(LED_PIN, LED_ON);
  }
}

void loop() {
  if (!espnowReady) {
    delay(1000);
    return;
  }
  
  unsigned long currentTime = millis();
  
  // Verificar se há timeout na recepção
  if (dataReceived && (currentTime - lastReceiveTime > RECEIVE_TIMEOUT)) {
    Serial.println("[TIMEOUT] Sem dados há mais de 1 segundo - PARANDO MOTORES");
    // Parar motores por segurança (com rampa para frenagem suave)
    MotorCommands stop_commands = {0, 0, false, currentTime};
    controlMotorsWithRamps(&stop_commands, currentTime);
    dataReceived = false;
  }
  
  // Atualizar rampas continuamente (mesmo sem novo pacote recebido)
  // Garante que a interpolação seja suave entre callbacks do ESP-NOW
  if (dataReceived) {
    controlMotorsWithRamps(&motorCommands, currentTime);
  }

  // Mostrar status a cada 5 segundos
  static unsigned long lastStatusTime = 0;
  if (currentTime - lastStatusTime >= 5000) {
    Serial.printf("[STATUS] Pacotes: %d | Último: %lu ms | Config X(c:%d dz:%d) Y(c:%d dz:%d)\n", 
                  packetsReceived, 
                  dataReceived ? (currentTime - lastReceiveTime) : 0,
                  x_axis_config.center, x_axis_config.center_deadzone,
                  y_axis_config.center, y_axis_config.center_deadzone);
    Serial.printf("[RAMPAS] Aceleração: %dms | Frenagem: %dms\n", ACCELERATION_TIME, DECELERATION_TIME);
    lastStatusTime = currentTime;
  }
  
  delay(10); // Delay reduzido para atualizar rampas com mais frequência
}
