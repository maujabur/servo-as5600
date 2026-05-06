# Sistema Básico - ESP32 Joystick

## 📄 Arquivos

- **`transmit.ino`**: Código do transmissor (lê joystick, envia via ESP-NOW)
- **`receive.ino`**: Código do receptor (recebe dados, controla motores)

## ⚡ Funcionalidades

### Hardware Necessário
- 2x ESP32-C3 Super Mini
- 1x Joystick analógico (2 eixos + botão)
- 1x Driver L298N 
- 2x Motores DC

### Controles Disponíveis
- **Joystick X/Y**: Controle tank drive dos motores
- **Botão**: Enviado via ESP-NOW (sem funcionalidade específica)
- **LED de status**: Indica comunicação ESP-NOW

### Características
- ✅ Controle de motores em 9 direções (incluindo diagonais)
- ✅ Normalização e deadzone do joystick
- ✅ Timeout de segurança (para motores em 1 segundo)
- ✅ Mapeamento automático de regiões do joystick
- ✅ Debug detalhado no Serial Monitor
- ✅ Otimizações de RF para estabilidade
- ✅ Proteção de tensão com FATOR_DE_POTENCIA

## 🔧 Configuração

### Pinos do Transmissor (ESP32-C3)
```
GPIO 3: Joystick X (analógico)
GPIO 4: Joystick Y (analógico)  
GPIO 2: Botão joystick (digital, pull-up)
GPIO 8: LED status (invertido: LOW=liga)
```

### Pinos do Receptor (ESP32-C3)
```
GPIO 1: Motor direito IN1 (PWM)
GPIO 2: Motor direito IN2 (PWM)
GPIO 3: Motor esquerdo IN1 (PWM)  
GPIO 4: Motor esquerdo IN2 (PWM)
GPIO 8: LED status (invertido: LOW=liga)
```

### L298N (ENA/ENB com jumpers)
```
IN1 ← GPIO 3 (PWM+Direção motor esquerdo)
IN2 ← GPIO 4 (PWM+Direção motor esquerdo)
IN3 ← GPIO 1 (PWM+Direção motor direito)
IN4 ← GPIO 2 (PWM+Direção motor direito)
ENA: Jumper (sempre HIGH)
ENB: Jumper (sempre HIGH)
```

## 🚀 Como Usar

1. **Configure os MACs**: 
   - Execute o receptor primeiro para ver seu MAC
   - Copie o MAC e cole no transmissor (linha `receiverMAC[]`)

2. **Upload dos códigos**:
   - `transmit.ino` → ESP32 transmissor
   - `receive.ino` → ESP32 receptor

3. **Teste no Serial Monitor** (115200 baud):
   - **Transmissor**: Mostra valores lidos do joystick
   - **Receptor**: Mostra dados recebidos e comandos dos motores

4. **Conecte os motores** conforme pinout do L298N

## 🎮 Controles

### Mapeamento do Joystick
```
    ↖ UP_LEFT    ↑ UP      ↗ UP_RIGHT
    ←  LEFT      ⊕ CENTER  → RIGHT  
    ↙ DOWN_LEFT  ↓ DOWN    ↘ DOWN_RIGHT
```

### Comportamento dos Motores
- **UP**: Ambos motores frente
- **DOWN**: Ambos motores trás  
- **LEFT**: Motor esquerdo trás + direito frente (giro)
- **RIGHT**: Motor esquerdo frente + direito trás (giro)
- **Diagonais**: Apenas um motor ativo (curvas suaves)
- **CENTER**: Motores parados

## ⚠️ Segurança

- **Timeout**: Motores param se não receber dados por 1 segundo
- **FATOR_DE_POTENCIA 0.75**: Protege motores 5V de sobretensão
- **Deadzone**: Evita movimento involuntário no centro
- **LED de status**: Indica comunicação ativa

## 🔄 Para Funcionalidades Avançadas

Se precisar de mais controles (servo, LEDs, buzzer, múltiplos joysticks), use o **Sistema Expandido**:
- `expanded_transmit.ino`
- `expanded_receive.ino` 
- Veja `README_EXPANSION.md` para detalhes

## 📊 Dados Transmitidos

```cpp
typedef struct {
  int16_t joystick_x;     // 0-4095 (12-bit ADC)
  int16_t joystick_y;     // 0-4095 (12-bit ADC)
  bool button_pressed;    // true/false
  uint32_t timestamp;     // millis() para debug
} JoystickData;
```

**Frequência**: 20 Hz (pacote a cada 50ms)  
**Tamanho**: 9 bytes por pacote