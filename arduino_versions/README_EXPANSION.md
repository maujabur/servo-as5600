# Expansão do Sistema ESP32 Joystick

## 🤔 Qual Versão Escolher?

**Use o Sistema Básico se:**
- ⭐ Está começando com ESP32/robótica
- 🎯 Quer um projeto simples e funcional
- 📦 Tem apenas 1 joystick analógico
- 🚗 Precisa apenas de controle básico de motores

**Use o Sistema Expandido se:**
- 🚀 Quer funcionalidades avançadas
- 🎮 Tem múltiplos sensores/controles
- 🤖 Está construindo um robô complexo
- 💡 Precisa de controles auxiliares (servo, LEDs, buzzer)

## 🎯 Duas Versões Disponíveis

### 📁 Sistema Básico (`transmit.ino` / `receive.ino`) 
**Para iniciantes ou projetos simples:**
- 1 joystick analógico (2 eixos + botão)
- Controle básico de motores L298N (tank drive)
- Botão do joystick apenas enviado, sem funcionalidade específica
- Ideal para: robôs simples, aprendizado, testes básicos

### 🚀 Sistema Expandido (`expanded_transmit.ino` / `expanded_receive.ino`)
**Para projetos avançados com múltiplos sensores:**
- Todos os recursos do sistema básico +
- 2º joystick, potenciômetro, sensor de luz
- 4 botões digitais com funcionalidades específicas
- Controles adicionais: servo, buzzer, LEDs, relé
- Modo turbo, parada de emergência, luzes automáticas

## ✅ Funcionalidades do Sistema Expandido

### Hardware Adicional Suportado
```
Sensores Analógicos:
├── Joystick Principal (GPIO 7,9,5) - Motores principais
├── Joystick Secundário (GPIO 6,8) - Controle servo/câmera
├── Potenciômetro (GPIO 10) - Controle de velocidade
└── Sensor de Luz (GPIO 11) - Luzes automáticas

Controles Digitais:
├── Botão 1 (GPIO 12) - Modo turbo
├── Botão 2 (GPIO 13) - Controle luzes auto/manual
├── Botão 3 (GPIO 14) - Buzzer/sirene
└── Switch (GPIO 16) - Liga/desliga geral

Saídas do Receptor:
├── Motores L298N (GPIO 11,9,7,5) - Tank drive
├── Servo Motor (GPIO 6) - Câmera/braço robótico
├── LEDs Iluminação (GPIO 10) - Faróis
├── Buzzer (GPIO 8) - Sirene
└── Relé (GPIO 13) - Controle geral 5V
```

### Funcionalidades Expandidas

#### Controles de Velocidade (Apenas Sistema Expandido)
- **Potenciômetro**: Velocidade base de 20% a 100%
- **Modo Turbo**: Adiciona +50% à velocidade (botão 1)
- **Parada Emergência**: Botão do joystick principal

#### Sistema de Iluminação (Apenas Sistema Expandido)
- **Modo Automático**: Baseado no sensor de luz
- **Modo Manual**: Controle por botão 2
- **Indicador**: LEDs ligam automaticamente quando escuro

#### Controles Auxiliares (Apenas Sistema Expandido)
- **Servo Motor**: Controlado pelo joystick secundário (eixo X)
- **Buzzer**: Ativado pelo botão 3
- **Relé Geral**: Controlado pela chave principal

#### Sistema Básico vs Expandido
| Funcionalidade | Sistema Básico | Sistema Expandido |
|---|---|---|
| Joystick principal | ✅ Tank drive | ✅ Tank drive |
| Botão do joystick | ⚠️ Enviado sem função | ✅ Parada emergência |
| Joystick secundário | ❌ | ✅ Controle servo |
| Potenciômetro velocidade | ❌ | ✅ 20-100% |
| Sensor de luz | ❌ | ✅ Luzes automáticas |
| Botões extras | ❌ | ✅ 4 botões funcionais |
| Modo turbo | ❌ | ✅ +50% velocidade |
| Controles auxiliares | ❌ | ✅ Servo, buzzer, LEDs, relé |

## 📊 Estrutura de Dados Expandida

```cpp
// Estrutura que comporta múltiplos sensores
typedef struct {
  // Joystick principal (movimento)
  int16_t joystick1_x, joystick1_y;
  bool joystick1_button;
  
  // Joystick secundário (câmera/auxiliar)  
  int16_t joystick2_x, joystick2_y;
  
  // Sensores analógicos
  int16_t potentiometer;    // Velocidade 0-100%
  int16_t light_sensor;     // Nível de luz
  
  // Botões (compactado em bitfield)
  struct {
    bool button1 : 1;       // Turbo
    bool button2 : 1;       // Luzes
    bool button3 : 1;       // Buzzer  
    bool switch1 : 1;       // Geral
  } buttons;
  
  uint32_t timestamp;
} ExpandedJoystickData;
```

## 🔧 Como Implementar Expansões

### 1. Para Adicionar Novos Sensores Analógicos

**No Transmissor:**
```cpp
// Adicionar pino
#define NEW_SENSOR_PIN 21

// Na estrutura de dados
int16_t new_sensor;

// Na função de leitura
joystickData.new_sensor = analogRead(NEW_SENSOR_PIN);
```

**No Receptor:**
```cpp
// Processar o novo sensor
void processNewSensor(ExpandedJoystickData* data) {
  int sensorValue = map(data->new_sensor, 0, 4095, 0, 255);
  // Usar o valor para controlar algo...
}
```

### 2. Para Adicionar Novos Botões

**No Transmissor (bitfield):**
```cpp
// Na estrutura
struct {
  bool button1 : 1;
  bool button2 : 1; 
  bool button3 : 1;
  bool new_button : 1;  // <-- Adicionar aqui
} buttons;

// Na leitura
joystickData.buttons.new_button = !digitalRead(NEW_BUTTON_PIN);
```

**No Receptor:**
```cpp
// Processar novo botão
static bool lastNewButton = false;
if (data->buttons.new_button && !lastNewButton) {
  // Ação no pressionar
  Serial.println("Novo botão pressionado!");
}
lastNewButton = data->buttons.new_button;
```

### 3. Para Adicionar Novos Atuadores

**No Receptor:**
```cpp
#define NEW_OUTPUT_PIN 11

void setup() {
  pinMode(NEW_OUTPUT_PIN, OUTPUT);
}

// Na função de controle
void executeCommands(ExpandedMotorCommands* commands) {
  digitalWrite(NEW_OUTPUT_PIN, commands->new_output_state);
}
```

## 📋 Casos de Uso Práticos

### Robô de Limpeza
- **Joystick 1**: Movimento do robô
- **Joystick 2**: Controle da escova rotativa  
- **Potenciômetro**: Velocidade de sucção
- **Sensor luz**: Detecção de sujeira
- **Botões**: Liga/desliga, modo turbo, retorno à base

### Robô de Exploração  
- **Joystick 1**: Movimento das rodas
- **Joystick 2**: Controle de câmera pan/tilt
- **Potenciômetro**: Velocidade geral
- **Sensor luz**: Controle automático do flash
- **Botões**: Foto, vídeo, modo noturno

### Drone Terrestre
- **Joystick 1**: Movimento direcional
- **Joystick 2**: Controle de altitude (com rotor)
- **Potenciômetro**: Sensibilidade de controle
- **Sensor luz**: Luzes de navegação
- **Botões**: Decolagem, pouso, modo acrobacia

## ⚙️ Configuração e Uso

### Conexões de Hardware (Ambas as Versões)

#### Alimentação através do L298N
```
Borne de parafuso do L298N:
+12V → Duas baterias lipo em série (7.4V-8.4V)
GND  → GND geral do circuito  
+5V  → Alimentação +5V do ESP32

⚠️ CRÍTICO: NÃO ligue as baterias direto ao ESP32!
           Isto pode danificá-lo permanentemente.
           
✅ USE: O regulador de tensão interno do L298N
        (borne +5V → ESP32)
```

**Jumper do Regulador:**  
Mantenha o jumper próximo ao borne **SEMPRE LIGADO** para ativar o regulador de 5V.

**Documentação Técnica:**
- **L298N**: Consulte o datasheet do módulo L298N para especificações completas, diagramas de pinout e características elétricas
- **ESP32-S2 Mini**: Para detalhes sobre alimentação, consumo e especificações técnicas, consulte o manual do módulo ESP32-S2 Mini

### Observação do Módulo Joystick

Se o módulo joystick for montado com os pinos "para baixo", conecte cruzado no joystick principal:
- VRX -> eixo Y
- VRY -> eixo X

### MAC de Broadcast para Testes

No `expanded_transmit.ino`, mantenha disponível a linha comentada de broadcast para fases iniciais de validação:

```cpp
//uint8_t receiverMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
```

---

### Sistema Básico
**Para projetos simples com 1 joystick:**
1. Upload do `receive.ino` no receptor
2. Upload do `transmit.ino` no transmissor  
3. Configure os MACs de pareamento (exibidos no Serial Monitor)
4. Teste movimentação dos motores
5. **Nota**: Botão é enviado mas não tem funcionalidade específica

### Sistema Expandido
**Para projetos avançados com múltiplos sensores:**
1. Upload do `expanded_receive.ino` no receptor
2. Upload do `expanded_transmit.ino` no transmissor
3. Conecte sensores e atuadores adicionais conforme pinout
4. Configure MACs de pareamento
5. Teste todas as funcionalidades no Serial Monitor

## 🛡️ Considerações de Segurança

### Sistema Básico
- **Timeout**: Sistema para automaticamente sem sinal (1 segundo)
- **Proteção de Tensão**: Use FATOR_DE_POTENCIA para motores 5V

### Sistema Expandido (Funcionalidades Adiciais)
- **Parada de Emergência**: Botão do joystick principal  
- **Chave Geral**: Switch principal desliga todo o sistema
- **Debouncing**: Botões têm filtro anti-bounce de 50ms
- **Sistema Hierárquico**: Switch geral > parada emergência > controles individuais

## 🎯 Próximos Passos

Para expandir ainda mais:
1. **Encoder nos motores** - Controle de velocidade preciso
2. **IMU/Giroscópio** - Estabilização automática  
3. **GPS** - Navegação autônoma
4. **Câmera com streaming** - Controle remoto visual
5. **Sensores de distância** - Evitar colisões
6. **Bateria com telemetria** - Monitoramento de energia