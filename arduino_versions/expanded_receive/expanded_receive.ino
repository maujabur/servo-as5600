//----------------------------------------------------------------------
// ESP32-S2 Expanded Joystick Receiver (ESP-NOW + Controles Avançados)
// Versão expandida para múltiplos sensores e controles
//----------------------------------------------------------------------

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

//----------------------------------------------------------------------
// CONFIGURAÇÕES DO RECEPTOR EXPANDIDO
//----------------------------------------------------------------------

// Pinos do receptor expandido para ESP32-S2
#define LED_PIN 15          // LED interno para status

// Inicialização rápida: 1 = sem delay/piscadas de boot, 0 = comportamento normal
#define FAST_STARTUP 0

// Motores principais (L298N)
#define MOTOR_RIGHT_IN1   11  // Motor direito
#define MOTOR_RIGHT_IN2    9
#define MOTOR_LEFT_IN1     7  // Motor esquerdo
#define MOTOR_LEFT_IN2     5

// Controles adicionais
#define SERVO_PIN        6    // Servo motor (controle de câmera/braço)
#define BUZZER_PIN       8    // Buzzer
#define LED_LIGHT_PIN    10   // LEDs de iluminação
#define RELAY_PIN        13   // Relé geral

// LED ativo em HIGH no ESP32-S2 Mini usado neste projeto
#define LED_ON  HIGH
#define LED_OFF LOW

//----------------------------------------------------------------------
// ESTRUTURAS DE DADOS EXPANDIDAS (DEVE SER IGUAL AO TRANSMISSOR!)
//----------------------------------------------------------------------

typedef struct {
  // Joystick principal
  int16_t joystick1_x;        
  int16_t joystick1_y;        
  bool joystick1_button;      
  
  // Joystick secundário
  int16_t joystick2_x;        
  int16_t joystick2_y;        
  
  // Controles analógicos
  int16_t potentiometer;      
  int16_t light_sensor;       
  
  // Botões digitais
  struct {
    bool button1 : 1;         // Modo turbo
    bool button2 : 1;         // Controle de luzes
    bool button3 : 1;         // Buzzer
    bool switch1 : 1;         // Chave geral
    bool reserved : 4;        
  } buttons;
  
  uint32_t timestamp;         
} ExpandedJoystickData;

// Estrutura para comandos expandidos
typedef struct {
  // Motores principais
  int16_t left_motor;
  int16_t right_motor;
  
  // Controles adicionais
  int16_t servo_position;     // Posição do servo (0-180)
  bool lights_on;             // Estado das luzes
  bool buzzer_on;             // Estado do buzzer
  bool relay_on;              // Estado do relé
  
  // Modificadores de velocidade
  float speed_multiplier;     // Multiplicador baseado no potenciômetro
  bool turbo_mode;            // Modo turbo
  bool auto_lights;           // Luzes automáticas baseadas no sensor
  
  uint32_t timestamp;
} ExpandedMotorCommands;

//----------------------------------------------------------------------
// VARIÁVEIS GLOBAIS
//----------------------------------------------------------------------

ExpandedJoystickData receivedData;
ExpandedMotorCommands motorCommands;
unsigned long lastReceiveTime = 0;
const unsigned long RECEIVE_TIMEOUT = 1000;
bool dataReceived = false;
int packetsReceived = 0;
bool espnowReady = false;

// Estados dos controles
bool systemEnabled = false;     // Sistema geral ligado/desligado
bool turboMode = false;
bool autoLights = true;
unsigned long lastBuzzerTime = 0;

//----------------------------------------------------------------------
// FUNÇÕES DE MAPEAMENTO EXPANDIDAS
//----------------------------------------------------------------------

// Mapear joystick principal para motores (igual ao sistema básico)
void mapJoystick1ToMotors(ExpandedJoystickData* data, ExpandedMotorCommands* commands) {
  // Normalizar valores do joystick 1 (simplificado para exemplo)
  int16_t x = map(data->joystick1_x, 0, 4095, -255, 255);
  int16_t y = map(data->joystick1_y, 0, 4095, -255, 255);
  
  // Tank drive básico
  commands->left_motor = constrain(y + x, -255, 255);
  commands->right_motor = constrain(y - x, -255, 255);
}

// Mapear joystick secundário para servo
void mapJoystick2ToServo(ExpandedJoystickData* data, ExpandedMotorCommands* commands) {
  // Mapear eixo X do joystick 2 para posição do servo (0-180 graus)
  commands->servo_position = map(data->joystick2_x, 0, 4095, 0, 180);
}

// Processar controles analógicos
void processAnalogControls(ExpandedJoystickData* data, ExpandedMotorCommands* commands) {
  // Potenciômetro controla velocidade geral (0.2x a 1.0x)
  commands->speed_multiplier = map(data->potentiometer, 0, 4095, 20, 100) / 100.0;
  
  // Sensor de luz para controle automático das luzes
  if (autoLights) {
    int lightLevel = map(data->light_sensor, 0, 4095, 0, 100);
    commands->lights_on = (lightLevel < 30); // Ligar luzes se escuro
  }
}

// Processar botões digitais
void processDigitalButtons(ExpandedJoystickData* data, ExpandedMotorCommands* commands) {
  // Chave geral
  systemEnabled = data->buttons.switch1;
  commands->relay_on = systemEnabled;
  
  // Botão 1: Modo turbo
  static bool lastButton1 = false;
  if (data->buttons.button1 && !lastButton1) {
    turboMode = !turboMode;
    commands->turbo_mode = turboMode;
    Serial.printf("[TURBO] Modo: %s\n", turboMode ? "ATIVADO" : "DESATIVADO");
  }
  lastButton1 = data->buttons.button1;
  
  // Botão 2: Controle manual de luzes
  static bool lastButton2 = false;
  if (data->buttons.button2 && !lastButton2) {
    autoLights = !autoLights;
    if (!autoLights) {
      commands->lights_on = !commands->lights_on; // Toggle manual
    }
    Serial.printf("[LUZES] Modo: %s\n", autoLights ? "AUTO" : "MANUAL");
  }
  lastButton2 = data->buttons.button2;
  
  // Botão 3: Buzzer
  commands->buzzer_on = data->buttons.button3;
  
  // Botão do joystick: Parada de emergência
  if (data->joystick1_button) {
    commands->left_motor = 0;
    commands->right_motor = 0;
    Serial.println("[EMERGÊNCIA] Motores parados!");
  }
}

//----------------------------------------------------------------------
// FUNÇÕES DE CONTROLE FÍSICO
//----------------------------------------------------------------------

// Controlar motor individual
void setMotor(int16_t speed, uint8_t in1_pin, uint8_t in2_pin) {
  if (speed == 0) {
    analogWrite(in1_pin, 0);
    analogWrite(in2_pin, 0);
  } else if (speed > 0) {
    analogWrite(in1_pin, abs(speed));
    analogWrite(in2_pin, 0);
  } else {
    analogWrite(in1_pin, 0);
    analogWrite(in2_pin, abs(speed));
  }
}

// Controlar servo (simulação com PWM)
void setServo(int position) {
  // Converter posição (0-180) para duty cycle PWM (aproximação)
  int dutyCycle = map(position, 0, 180, 26, 128); // ~1ms a 2ms em 8-bit PWM
  analogWrite(SERVO_PIN, dutyCycle);
}

// Executar todos os comandos físicos
void executeCommands(ExpandedMotorCommands* commands) {
  if (!systemEnabled) {
    // Sistema desligado - parar tudo
    setMotor(0, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2);
    setMotor(0, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2);
    digitalWrite(LED_LIGHT_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(RELAY_PIN, LOW);
    return;
  }
  
  // Aplicar multiplicador de velocidade e modo turbo
  float finalMultiplier = commands->speed_multiplier;
  if (commands->turbo_mode) {
    finalMultiplier *= 1.5; // Turbo adiciona 50%
  }
  
  int16_t leftSpeed = commands->left_motor * finalMultiplier;
  int16_t rightSpeed = commands->right_motor * finalMultiplier;
  
  // Limitar valores
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Controlar motores
  setMotor(leftSpeed, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2);
  setMotor(rightSpeed, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2);
  
  // Controlar servo
  setServo(commands->servo_position);
  
  // Controlar LEDs de iluminação
  digitalWrite(LED_LIGHT_PIN, commands->lights_on ? HIGH : LOW);
  
  // Controlar buzzer
  digitalWrite(BUZZER_PIN, commands->buzzer_on ? HIGH : LOW);
  
  // Controlar relé
  digitalWrite(RELAY_PIN, commands->relay_on ? HIGH : LOW);
}

//----------------------------------------------------------------------
// FUNÇÕES DE CALLBACK ESP-NOW
//----------------------------------------------------------------------

// Callback quando dados são recebidos
void OnDataReceived(const esp_now_recv_info *recv_info, const uint8_t *data, int len) {
  if (len == sizeof(ExpandedJoystickData)) {
    memcpy(&receivedData, data, sizeof(ExpandedJoystickData));
    lastReceiveTime = millis();
    dataReceived = true;
    packetsReceived++;
    
    // Piscar LED de status
    digitalWrite(LED_PIN, LED_ON);
    delay(10);
    digitalWrite(LED_PIN, LED_OFF);
    
    // Processar dados recebidos
    mapJoystick1ToMotors(&receivedData, &motorCommands);
    mapJoystick2ToServo(&receivedData, &motorCommands);
    processAnalogControls(&receivedData, &motorCommands);
    processDigitalButtons(&receivedData, &motorCommands);
    
    // Executar comandos físicos
    executeCommands(&motorCommands);
    
    // Log detalhado
    Serial.printf("[%d] J1(%d,%d,%s) J2(%d,%d) POT:%.2f LIGHT:%s BTNS(%s%s%s%s)\n",
                  packetsReceived,
                  receivedData.joystick1_x, receivedData.joystick1_y,
                  receivedData.joystick1_button ? "P" : "F",
                  receivedData.joystick2_x, receivedData.joystick2_y,
                  motorCommands.speed_multiplier,
                  motorCommands.lights_on ? "ON" : "OFF",
                  receivedData.buttons.button1 ? "T" : "-",
                  receivedData.buttons.button2 ? "L" : "-", 
                  receivedData.buttons.button3 ? "B" : "-",
                  receivedData.buttons.switch1 ? "S" : "-");
                  
    Serial.printf("     MOTORS(L:%d R:%d) SERVO:%d SYS:%s TURBO:%s\n",
                  (int)(motorCommands.left_motor * motorCommands.speed_multiplier),
                  (int)(motorCommands.right_motor * motorCommands.speed_multiplier),
                  motorCommands.servo_position,
                  systemEnabled ? "ON" : "OFF",
                  turboMode ? "ON" : "OFF");
  }
}

//----------------------------------------------------------------------
// SETUP E LOOP
//----------------------------------------------------------------------

bool initESPNow_RX() {
  WiFi.disconnect();
  WiFi.setSleep(WIFI_PS_MIN_MODEM);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao inicializar ESP-NOW expandido");
    return false;
  }
  
  esp_now_register_recv_cb(OnDataReceived);
  Serial.println("ESP-NOW expandido inicializado com sucesso!");
  Serial.println("RF: potência padrão + modem sleep MIN_MODEM");
  return true;
}

void setup() {
  Serial.begin(115200);
  #if !FAST_STARTUP
  delay(2000);
  #endif
  Serial.println("=== ESP32-S2 Expanded Joystick Receiver ===");
  
  WiFi.mode(WIFI_STA);
  Serial.println("MAC Address do RECEPTOR EXPANDIDO:");
  Serial.println(WiFi.macAddress());
  
  // Configurar pinos de saída
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_LIGHT_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  
  // Estado inicial
  digitalWrite(LED_PIN, LED_OFF);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_LIGHT_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  
  // Parar motores
  setMotor(0, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2);
  setMotor(0, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2);
  
  espnowReady = initESPNow_RX();
  
  if (espnowReady) {
    Serial.println("Receptor expandido pronto!");
    Serial.println("Controles disponíveis:");
    Serial.println("- Joystick 1: Motores principais");
    Serial.println("- Joystick 2: Controle de servo"); 
    Serial.println("- Potenciômetro: Velocidade geral");
    Serial.println("- Sensor luz: Controle automático de LEDs");
    Serial.println("- Botão 1: Modo turbo");
    Serial.println("- Botão 2: Toggle luzes auto/manual");
    Serial.println("- Botão 3: Buzzer");
    Serial.println("- Switch: Liga/desliga geral");
    
    #if !FAST_STARTUP
    // Sequência de inicialização
    for(int i = 0; i < 3; i++) {
      digitalWrite(LED_PIN, LED_ON);
      digitalWrite(LED_LIGHT_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LED_OFF);
      digitalWrite(LED_LIGHT_PIN, LOW);
      delay(200);
    }
    #endif
  }
}

void loop() {
  if (!espnowReady) {
    delay(1000);
    return;
  }
  
  unsigned long currentTime = millis();
  
  // Verificar timeout
  if (dataReceived && (currentTime - lastReceiveTime > RECEIVE_TIMEOUT)) {
    Serial.println("[TIMEOUT] Sem dados - PARANDO SISTEMA");
    setMotor(0, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2);
    setMotor(0, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2);
    digitalWrite(BUZZER_PIN, LOW);
    dataReceived = false;
  }
  
  // Status periódico
  static unsigned long lastStatusTime = 0;
  if (currentTime - lastStatusTime >= 5000) {
    Serial.printf("[STATUS] Pacotes:%d SYS:%s AUTO_LUZ:%s TURBO:%s\n", 
                  packetsReceived,
                  systemEnabled ? "ON" : "OFF",
                  autoLights ? "ON" : "OFF", 
                  turboMode ? "ON" : "OFF");
    lastStatusTime = currentTime;
  }
  
  delay(50);
}