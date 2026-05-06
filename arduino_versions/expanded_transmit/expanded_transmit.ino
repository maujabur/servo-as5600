//----------------------------------------------------------------------
// ESP32-C3 Expanded Joystick Transmitter (ESP-NOW)
// Versão expandida com múltiplos sensores e botões
//----------------------------------------------------------------------

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

//----------------------------------------------------------------------
// CONFIGURAÇÕES DO TRANSMISSOR EXPANDIDO
//----------------------------------------------------------------------

// Definições dos pinos para ESP32-C3 Super Mini (TRANSMISSOR EXPANDIDO)
#define JOYSTICK_X_PIN 3     // Pino analógico X do joystick principal
#define JOYSTICK_Y_PIN 4     // Pino analógico Y do joystick principal
#define JOYSTICK_BTN_PIN 2   // Botão do joystick principal

// Sensores adicionais
#define JOYSTICK2_X_PIN 0    // Segundo joystick (eixo X)
#define JOYSTICK2_Y_PIN 1    // Segundo joystick (eixo Y)
#define POTENTIOMETER_PIN 5  // Potenciômetro para velocidade
#define LIGHT_SENSOR_PIN 6   // Sensor de luz (para controle automático)

// Botões adicionais
#define BUTTON1_PIN 7        // Botão 1 (modo turbo)
#define BUTTON2_PIN 9        // Botão 2 (luzes)
#define BUTTON3_PIN 10       // Botão 3 (buzzer)
#define SWITCH1_PIN 20       // Chave liga/desliga geral

#define LED_PIN 8           // LED interno para status

// LED com lógica invertida
#define LED_ON  LOW
#define LED_OFF HIGH

// MAC Address do receptor
//uint8_t receiverMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast por enquanto
uint8_t receiverMAC[] = {0x20, 0x6E, 0xF1, 0x6D, 0x9F, 0xBC}; // ALTERE AQUI!

//----------------------------------------------------------------------
// ESTRUTURAS DE DADOS EXPANDIDAS
//----------------------------------------------------------------------

// Estrutura expandida para enviar múltiplos sensores
typedef struct {
  // Joystick principal
  int16_t joystick1_x;        // Valor X joystick principal (0-4095)
  int16_t joystick1_y;        // Valor Y joystick principal (0-4095) 
  bool joystick1_button;      // Estado do botão joystick principal
  
  // Joystick secundário (para controle de câmera, braço robótico, etc.)
  int16_t joystick2_x;        // Valor X joystick secundário (0-4095)
  int16_t joystick2_y;        // Valor Y joystick secundário (0-4095)
  
  // Controles analógicos
  int16_t potentiometer;      // Potenciômetro de velocidade (0-4095)
  int16_t light_sensor;       // Sensor de luz ambiente (0-4095)
  
  // Botões digitais (bitfield para economizar espaço)
  struct {
    bool button1 : 1;         // Botão modo turbo
    bool button2 : 1;         // Botão luzes
    bool button3 : 1;         // Botão buzzer
    bool switch1 : 1;         // Chave geral
    bool reserved : 4;        // Espaço para mais 4 botões
  } buttons;
  
  uint32_t timestamp;         // Timestamp para debugging
} ExpandedJoystickData;

//----------------------------------------------------------------------
// VARIÁVEIS GLOBAIS
//----------------------------------------------------------------------

ExpandedJoystickData joystickData;
esp_now_peer_info_t peerInfo;

// Variáveis de controle
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 50; // Enviar a cada 50ms (20Hz)
bool espnowReady = false;

// Variáveis para debouncing dos botões
struct ButtonState {
  bool last_state;
  unsigned long last_change;
  const unsigned long debounce_time = 50; // 50ms debounce
};

ButtonState button_states[5]; // Para 5 botões

//----------------------------------------------------------------------
// FUNÇÕES DE CALLBACK ESP-NOW
//----------------------------------------------------------------------

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  static int success_count = 0;
  static int fail_count = 0;
  
  if (status == ESP_NOW_SEND_SUCCESS) {
    success_count++;
  } else {
    fail_count++;
    Serial.printf("[ERRO] Falha no envio! Success:%d Fail:%d\n", success_count, fail_count);
  }
  
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

//----------------------------------------------------------------------
// FUNÇÕES DE INICIALIZAÇÃO
//----------------------------------------------------------------------

bool initESPNow_TX() {
  WiFi.disconnect();
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setSleep(WIFI_PS_NONE);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao inicializar ESP-NOW");
    return false;
  }
  
  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Falha ao adicionar peer");
    return false;
  }
  
  Serial.println("ESP-NOW expandido inicializado com sucesso!");
  return true;
}

//----------------------------------------------------------------------
// FUNÇÕES DE CONTROLE DOS SENSORES
//----------------------------------------------------------------------

// Função para ler botão com debouncing
bool readButtonDebounced(int pin, int button_index) {
  bool current_reading = !digitalRead(pin); // Pull-up, então inverte
  unsigned long current_time = millis();
  
  if (current_reading != button_states[button_index].last_state) {
    button_states[button_index].last_change = current_time;
  }
  
  if ((current_time - button_states[button_index].last_change) > button_states[button_index].debounce_time) {
    button_states[button_index].last_state = current_reading;
    return current_reading;
  }
  
  return button_states[button_index].last_state;
}

// Função para ler todos os sensores
void readAllSensors() {
  // Joysticks analógicos
  joystickData.joystick1_x = analogRead(JOYSTICK_X_PIN);
  joystickData.joystick1_y = analogRead(JOYSTICK_Y_PIN);
  joystickData.joystick1_button = !digitalRead(JOYSTICK_BTN_PIN);
  
  joystickData.joystick2_x = analogRead(JOYSTICK2_X_PIN);
  joystickData.joystick2_y = analogRead(JOYSTICK2_Y_PIN);
  
  // Sensores analógicos adicionais
  joystickData.potentiometer = analogRead(POTENTIOMETER_PIN);
  joystickData.light_sensor = analogRead(LIGHT_SENSOR_PIN);
  
  // Botões com debouncing
  joystickData.buttons.button1 = readButtonDebounced(BUTTON1_PIN, 0);
  joystickData.buttons.button2 = readButtonDebounced(BUTTON2_PIN, 1);
  joystickData.buttons.button3 = readButtonDebounced(BUTTON3_PIN, 2);
  joystickData.buttons.switch1 = readButtonDebounced(SWITCH1_PIN, 3);
  
  joystickData.timestamp = millis();
}

// Função para enviar dados via ESP-NOW
void sendJoystickData() {
  esp_err_t result = esp_now_send(receiverMAC, (uint8_t*) &joystickData, sizeof(joystickData));
  
  if (result != ESP_OK) {
    Serial.println("Erro ao enviar dados expandidos");
  }
}

//----------------------------------------------------------------------
// SETUP E LOOP PRINCIPAL
//----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== ESP32-C3 Expanded Joystick Transmitter ===");
  
  WiFi.mode(WIFI_STA);
  
  Serial.println("\n*** TRANSMISSOR EXPANDIDO ***");
  Serial.println("MAC Address:");
  Serial.println(WiFi.macAddress());
  Serial.println("Sensores configurados:");
  Serial.println("- 2x Joysticks analógicos");
  Serial.println("- 1x Potenciômetro");
  Serial.println("- 1x Sensor de luz");
  Serial.println("- 4x Botões digitais");
  Serial.println("*****************************\n");
  
  // Configurar pinos
  pinMode(JOYSTICK_BTN_PIN, INPUT_PULLUP);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
  pinMode(SWITCH1_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);
  
  // Configurar resolução ADC para 12 bits
  analogReadResolution(12);
  
  // Inicializar estados dos botões
  for(int i = 0; i < 5; i++) {
    button_states[i].last_state = false;
    button_states[i].last_change = 0;
  }
  
  espnowReady = initESPNow_TX();
  
  if (espnowReady) {
    Serial.println("Transmissor expandido pronto!");
    for(int i = 0; i < 5; i++) {
      digitalWrite(LED_PIN, LED_ON);
      delay(100);
      digitalWrite(LED_PIN, LED_OFF);
      delay(100);
    }
  } else {
    Serial.println("Falha na inicialização!");
    digitalWrite(LED_PIN, LED_ON);
  }
}

void loop() {
  if (!espnowReady) {
    delay(1000);
    return;
  }
  
  unsigned long currentTime = millis();
  
  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    readAllSensors();
    sendJoystickData();
    lastSendTime = currentTime;
    
    // Debug expandido no Serial Monitor
    Serial.printf("J1(%d,%d,%s) J2(%d,%d) POT:%d LIGHT:%d BTN(%s%s%s%s)\n", 
                  joystickData.joystick1_x, joystickData.joystick1_y, 
                  joystickData.joystick1_button ? "P" : "F",
                  joystickData.joystick2_x, joystickData.joystick2_y,
                  joystickData.potentiometer, joystickData.light_sensor,
                  joystickData.buttons.button1 ? "1" : "-",
                  joystickData.buttons.button2 ? "2" : "-",
                  joystickData.buttons.button3 ? "3" : "-",
                  joystickData.buttons.switch1 ? "S" : "-");
  }
  
  delay(10);
}