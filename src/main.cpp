#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define TX_MODE 0 // transmitter mode
#define RX_MODE 1 // receiver mode

#define MODE TX_MODE  // Mudando para modo receptor

// Inicialização rápida: 1 = sem delay/piscadas de boot, 0 = comportamento normal
#define FAST_STARTUP 0

//----------------------------------------------------------------------
// esp-now analog joystick with 1 button transmitter and receiver
//----------------------------------------------------------------------

// Range de saída normalizada do joystick (para PWM 8-bit)
#define FATOR_DE_POTENCIA 0.75
#define AXIS_MAX_OUTPUT 255 * FATOR_DE_POTENCIA
#define AXIS_MIN_OUTPUT -255 * FATOR_DE_POTENCIA
#define AXIS_CENTER_OUTPUT 0

// Definições dos pinos para ESP32-S2 Mini
#define JOYSTICK_X_PIN 7    // Pino analógico X do joystick (otimizado para conexão física)
#define JOYSTICK_Y_PIN 9    // Pino analógico Y do joystick (otimizado para conexão física)
#define JOYSTICK_BTN_PIN 5  // Botão do joystick (com pull-up interno)
#define LED_PIN 15           // LED interno para status

// Para usar o joystick com os pinos de conexão para "baixo" os sinais marcados como "VRX" e "VRY" são conectados de forma cruzada (VRX -> Y e VRY -> X)
// LED ativo em HIGH no ESP32-S2 Mini usado neste projeto
#define LED_ON  HIGH
#define LED_OFF LOW

// Definições dos pinos para L298N (receptor)
#define MOTOR_RIGHT_IN1   11   // IN1 - PWM+Direção motor direito
#define MOTOR_RIGHT_IN2    9   // IN2 - PWM+Direção motor direito
#define MOTOR_LEFT_IN1     7   // IN3 - PWM+Direção motor esquerdo
#define MOTOR_LEFT_IN2     5   // IN4 - PWM+Direção motor esquerdo

// MAC Address do receptor (substitua pelo MAC real do seu receptor ESP32)
// Para descobrir o MAC, use WiFi.macAddress() no receptor
uint8_t receiverMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast - use durante testes iniciais, mas não recomendado para uso final
//uint8_t receiverMAC[] = {0x90, 0xE5, 0xB1, 0x8E, 0x83, 0x4E};

// Estrutura de dados para enviar
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

JoystickData joystickData;
esp_now_peer_info_t peerInfo;

// Variáveis de controle
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 50; // Enviar a cada 50ms (20Hz)
bool espnowReady = false;

// Variáveis do receptor
JoystickData receivedData;
NormalizedJoystick normalizedData;
MotorCommands motorCommands;
unsigned long lastReceiveTime = 0;
const unsigned long RECEIVE_TIMEOUT = 1000; // Timeout de 1 segundo
bool dataReceived = false;
int packetsReceived = 0;

// Configuração de calibração dos eixos (valores iniciais)
AxisConfig x_axis_config = {
  .center = 2660,           // Centro teórico
  .min_value = 0,           // Mínimo ADC
  .max_value = 4095,        // Máximo ADC
  .center_deadzone = 200,   // Zona morta centro
  .edge_deadzone = 50       // Zona morta extremos
};

AxisConfig y_axis_config = {
  .center = 2620,
  .min_value = 0,
  .max_value = 4095,
  .center_deadzone = 200,
  .edge_deadzone = 50
};

//----------------------------------------------------------------------
// supporting functions
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

// Função para controlar motor individual L298N (ENA/ENB jumpeados)
void setMotor(int16_t speed, uint8_t in1_pin, uint8_t in2_pin) {
  if (speed == 0) {
    // Motor parado
    analogWrite(in1_pin, 0);
    analogWrite(in2_pin, 0);
  } else if (speed > 0) {
    // Motor para frente: IN1 = PWM, IN2 = LOW
    analogWrite(in1_pin, abs(speed));
    analogWrite(in2_pin, 0);
  } else {
    // Motor para trás: IN1 = LOW, IN2 = PWM
    analogWrite(in1_pin, 0);
    analogWrite(in2_pin, abs(speed));
  }
}

// Função para controlar ambos os motores L298N
void controlMotors(MotorCommands* commands) {
  if (commands->enable) {
    setMotor(commands->left_motor, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2);
    setMotor(commands->right_motor, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2);
  } else {
    // Parar ambos os motores
    setMotor(0, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2);
    setMotor(0, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2);
  }
}

// Callback quando dados são enviados (transmissor)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
#else
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
  static int success_count = 0;
  static int fail_count = 0;
  
  if (status == ESP_NOW_SEND_SUCCESS) {
    success_count++;
  } else {
    fail_count++;
    Serial.printf("[ERRO] Falha no envio! Success:%d Fail:%d\n", success_count, fail_count);
  }
  
  // Piscar LED para indicar envio (lógica invertida)
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

// Callback quando dados são recebidos (receptor)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
void OnDataReceived(const esp_now_recv_info *recv_info, const uint8_t *data, int len) {
#else
void OnDataReceived(const uint8_t *mac_addr, const uint8_t *data, int len) {
#endif
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  const uint8_t *src_mac = recv_info->src_addr;
  #else
  const uint8_t *src_mac = mac_addr;
  #endif

  // Verificar se o tamanho dos dados está correto
  if (len == sizeof(JoystickData)) {
    memcpy(&receivedData, data, sizeof(JoystickData));
    lastReceiveTime = millis();
    dataReceived = true;
    packetsReceived++;
    
    // Piscar LED para indicar recebimento
    digitalWrite(LED_PIN, LED_ON);
    delay(50);
    digitalWrite(LED_PIN, LED_OFF);
    
    // Log no Serial com MAC do transmissor
    Serial.printf("[%d] Recebido de %02X:%02X:%02X:%02X:%02X:%02X:\n",
                  packetsReceived,
                  src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5]);
    
    // Normalizar dados
    normalizeJoystickData(&receivedData, &normalizedData);
    
    // Detectar região
    JoystickRegion region = getJoystickRegion(&normalizedData);
    
    // Mapear para comandos de motor
    mapJoystickToMotors(&normalizedData, &motorCommands);
    
    // Controlar motores fisicamente
    controlMotors(&motorCommands);
    
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
    
    Serial.printf("  MOTORS -> L:%4d R:%4d EN:%s SPEED:%d\n",
                  motorCommands.left_motor,
                  motorCommands.right_motor,
                  motorCommands.enable ? "YES" : "NO",
                  calculateMagnitude(&normalizedData));
    
    Serial.println();
  } else {
    Serial.printf("Erro: Tamanho incorreto recebido: %d bytes\n", len);
  }
}

// Função para inicializar ESP-NOW (transmissor)
bool initESPNow_TX() {
  // WiFi já configurado em tx_setup()
  WiFi.disconnect();
  
  // Configuração de RF equilibrada (temperatura menor)
  WiFi.setSleep(WIFI_PS_MIN_MODEM);      // Power save habilitado
  
  // Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao inicializar ESP-NOW");
    return false;
  }
  
  // Registrar callback de envio
  esp_now_register_send_cb(OnDataSent);
  
  // Registrar peer (receptor) com configurações otimizadas
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 1;          // Canal fixo para maior estabilidade
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;  // Interface STA
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Falha ao adicionar peer");
    return false;
  }
  
  Serial.println("ESP-NOW inicializado com sucesso (TX)");
  Serial.println("Configuração de RF aplicada:");
  Serial.println("- Potência TX: padrão do sistema");
  Serial.println("- Modem sleep: MIN_MODEM");
  Serial.println("- Canal: 1 (fixo)");
  return true;
}

// Função para inicializar ESP-NOW (receptor)
bool initESPNow_RX() {
  // WiFi já configurado em rx_setup()
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

// Função para ler dados do joystick
void readJoystick() {
  joystickData.joystick_x = analogRead(JOYSTICK_X_PIN);
  joystickData.joystick_y = analogRead(JOYSTICK_Y_PIN);
  joystickData.button_pressed = !digitalRead(JOYSTICK_BTN_PIN); // Pull-up, então inverte
  joystickData.timestamp = millis();
}

// Função para enviar dados via ESP-NOW
void sendJoystickData() {
  esp_err_t result = esp_now_send(receiverMAC, (uint8_t*) &joystickData, sizeof(joystickData));
  
  if (result != ESP_OK) {
    Serial.println("Erro ao enviar dados");
  }
}

//----------------------------------------------------------------------
// Transmitter section
//----------------------------------------------------------------------
void tx_setup() {
  Serial.begin(115200);
  #if !FAST_STARTUP
  delay(2000);  // Delay para conseguir ver o MAC address
  #endif
  Serial.println("=== ESP32-S2 Joystick Transmitter (Otimizado) ===");
  
  // Configurar WiFi para obter MAC address
  WiFi.mode(WIFI_STA);
  
  // Exibir MAC address de forma destacada
  Serial.println("\n*** CONFIGURAÇÃO DE PAREAMENTO ***");
  Serial.println("MAC Address deste TRANSMISSOR:");
  Serial.println(WiFi.macAddress());
  Serial.println("\nCopie este MAC e configure no RECEPTOR:");
  Serial.println("uint8_t receiverMAC[] = {0x" + 
                 WiFi.macAddress().substring(0,2) + ", 0x" +
                 WiFi.macAddress().substring(3,5) + ", 0x" +
                 WiFi.macAddress().substring(6,8) + ", 0x" +
                 WiFi.macAddress().substring(9,11) + ", 0x" +
                 WiFi.macAddress().substring(12,14) + ", 0x" +
                 WiFi.macAddress().substring(15,17) + "};");
  Serial.println("*********************************\n");
  
  // Configurar pinos
  pinMode(JOYSTICK_BTN_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF); // LED desligado inicialmente
  
  // Configurar resolução ADC para 12 bits (0-4095)
  analogReadResolution(12);
  
  // Inicializar ESP-NOW
  espnowReady = initESPNow_TX();
  
  if (espnowReady) {
    Serial.println("Transmissor pronto!");
    #if !FAST_STARTUP
    // Piscar LED 3 vezes para indicar inicialização OK
    for(int i = 0; i < 3; i++) {
      digitalWrite(LED_PIN, LED_ON);  // Liga LED
      delay(200);
      digitalWrite(LED_PIN, LED_OFF); // Desliga LED
      delay(200);
    }
    #endif
  } else {
    Serial.println("Falha na inicialização!");
    // LED fixo indica erro (ligado)
    digitalWrite(LED_PIN, LED_ON);
  }
}

//----------------------------------------------------------------------
void tx_loop() {
  if (!espnowReady) {
    delay(1000);
    return;
  }
  
  unsigned long currentTime = millis();
  
  // Enviar dados a cada SEND_INTERVAL ms
  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    readJoystick();
    sendJoystickData();
    lastSendTime = currentTime;
    
    // Debug no Serial Monitor
    Serial.printf("X:%d Y:%d BTN:%s | ", 
                  joystickData.joystick_x, 
                  joystickData.joystick_y, 
                  joystickData.button_pressed ? "PRESS" : "FREE");
    Serial.println();
  }
  
  delay(10); // Pequeno delay para não sobrecarregar
}

//----------------------------------------------------------------------
// Receiver section
//----------------------------------------------------------------------
void rx_setup() {
  Serial.begin(115200);
  #if !FAST_STARTUP
  delay(2000);  // Delay para conseguir ver o MAC address
  #endif
  Serial.println("=== ESP32-S2 Joystick Receptor (Otimizado) ===");
  
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
  
  // Parar motores inicialmente
  setMotor(0, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2);
  setMotor(0, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2);
  
  // Inicializar ESP-NOW para recepção
  espnowReady = initESPNow_RX();
  
  if (espnowReady) {
    Serial.println("Receptor pronto!");
    #if !FAST_STARTUP
    // Piscar LED 3 vezes para indicar inicialização OK
    for(int i = 0; i < 3; i++) {
      digitalWrite(LED_PIN, LED_ON);  // Liga LED
      delay(200);
      digitalWrite(LED_PIN, LED_OFF); // Desliga LED
      delay(200);
    }
    #endif
  } else {
    Serial.println("Falha na inicialização!");
    // LED fixo indica erro (ligado)
    digitalWrite(LED_PIN, LED_ON);
  }
}

void rx_loop() {
  if (!espnowReady) {
    delay(1000);
    return;
  }
  
  unsigned long currentTime = millis();
  
  // Verificar se há timeout na recepção
  if (dataReceived && (currentTime - lastReceiveTime > RECEIVE_TIMEOUT)) {
    Serial.println("[TIMEOUT] Sem dados há mais de 1 segundo - PARANDO MOTORES");
    // Parar motores por segurança
    setMotor(0, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2);
    setMotor(0, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2);
    dataReceived = false;
  }
  
  // Mostrar status a cada 5 segundos
  static unsigned long lastStatusTime = 0;
  if (currentTime - lastStatusTime >= 5000) {
    Serial.printf("[STATUS] Pacotes: %d | Último: %lu ms | Config X(c:%d dz:%d) Y(c:%d dz:%d)\n", 
                  packetsReceived, 
                  dataReceived ? (currentTime - lastReceiveTime) : 0,
                  x_axis_config.center, x_axis_config.center_deadzone,
                  y_axis_config.center, y_axis_config.center_deadzone);
    lastStatusTime = currentTime;
  }
  
  delay(100); // Pequeno delay
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------
void setup()
{
#if MODE == TX_MODE
  tx_setup();
#elif MODE == RX_MODE
  rx_setup();
#endif
}

void loop()
{
#if MODE == TX_MODE
  tx_loop();
#elif MODE == RX_MODE
  rx_loop();
#endif
}
