//----------------------------------------------------------------------
// ESP32-S2 Joystick Transmitter (ESP-NOW)
// Arquivo separado para uso no Arduino IDE
//----------------------------------------------------------------------

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

//----------------------------------------------------------------------
// CONFIGURAÇÕES DO TRANSMISSOR
//----------------------------------------------------------------------

// Range de saída normalizada do joystick (para PWM 8-bit)
#define FATOR_DE_POTENCIA 0.75
#define AXIS_MAX_OUTPUT 255 * FATOR_DE_POTENCIA
#define AXIS_MIN_OUTPUT -255 * FATOR_DE_POTENCIA
#define AXIS_CENTER_OUTPUT 0

// Definições dos pinos para ESP32-S2 Mini (TRANSMISSOR)
#define JOYSTICK_X_PIN 7    // Pino analógico X do joystick
#define JOYSTICK_Y_PIN 9    // Pino analógico Y do joystick
#define JOYSTICK_BTN_PIN 5  // Botão do joystick (com pull-up interno)
#define LED_PIN 15          // LED interno para status

// Se montar o módulo joystick com pinos "para baixo", conectar cruzado:
// VRX -> eixo Y e VRY -> eixo X.

// LED ativo em HIGH no ESP32-S2 Mini usado neste projeto
#define LED_ON  HIGH
#define LED_OFF LOW

// MAC Address do receptor (substitua pelo MAC real do seu receptor ESP32)
// Para descobrir o MAC, execute o código no receptor e veja o Serial Monitor
// Mantenha esta opção de broadcast para testes iniciais.
uint8_t receiverMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//uint8_t receiverMAC[] = {0x90, 0xE5, 0xB1, 0x8E, 0x83, 0x4E}; // ALTERE AQUI!

//----------------------------------------------------------------------
// ESTRUTURAS DE DADOS
//----------------------------------------------------------------------

// Estrutura de dados para enviar
typedef struct {
  int16_t joystick_x;     // Valor X do joystick (0-4095)
  int16_t joystick_y;     // Valor Y do joystick (0-4095) 
  bool button_pressed;    // Estado do botão
  uint32_t timestamp;     // Timestamp para debugging
} JoystickData;

//----------------------------------------------------------------------
// VARIÁVEIS GLOBAIS
//----------------------------------------------------------------------

JoystickData joystickData;
esp_now_peer_info_t peerInfo;

// Variáveis de controle
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 50; // Enviar a cada 50ms (20Hz)
bool espnowReady = false;

//----------------------------------------------------------------------
// FUNÇÕES DE CALLBACK ESP-NOW
//----------------------------------------------------------------------

// Callback quando dados são enviados (transmissor)
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
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

//----------------------------------------------------------------------
// FUNÇÕES DE INICIALIZAÇÃO
//----------------------------------------------------------------------

// Função para inicializar ESP-NOW (transmissor)
bool initESPNow_TX() {
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

//----------------------------------------------------------------------
// FUNÇÕES DE CONTROLE DO JOYSTICK
//----------------------------------------------------------------------

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
// SETUP E LOOP PRINCIPAL
//----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(2000);  // Delay para conseguir ver o MAC address
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