# Sistema Básico - ESP32-S2 Joystick

## 📄 Arquivos

- **`transmit.ino`**: Código do transmissor (lê joystick, envia via ESP-NOW)
- **`receive.ino`**: Código do receptor (recebe dados, controla motores)

## ⚡ Funcionalidades

### Hardware Necessário
- 2x ESP32-S2 Mini
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

### Pinos do Transmissor (ESP32-S2)
```
GPIO 7: Joystick X (analógico)
GPIO 9: Joystick Y (analógico)
GPIO 5: Botão joystick (digital, pull-up)
GPIO 15: LED status (ativo em HIGH)
```

Observação do módulo joystick:
- Se os pinos do módulo ficarem "para baixo" na montagem, conecte cruzado: VRX -> Y e VRY -> X.

### Pinos do Receptor (ESP32-S2)
```
GPIO 12: Motor direito IN1 (PWM)
GPIO 11: Motor direito IN2 (PWM)
GPIO 9: Motor esquerdo IN3 (PWM)
GPIO 7: Motor esquerdo IN4 (PWM)
GPIO 15: LED status (ativo em HIGH)
```

### L298N (ENA/ENB com jumpers)
```
IN1 ← GPIO 12 (PWM+Direção motor direito)
IN2 ← GPIO 11 (PWM+Direção motor direito)
IN3 ← GPIO 9 (PWM+Direção motor esquerdo)
IN4 ← GPIO 7 (PWM+Direção motor esquerdo)
ENA: Jumper (sempre HIGH)
ENB: Jumper (sempre HIGH)
```

### Alimentação do Sistema
```
Borne de parafuso do L298N:
+12V → Duas baterias lipo em série (7.4V-8.4V)
GND  → GND geral do circuito
+5V  → Alimentação +5V do ESP32

⚠️ NUNCA ligue as baterias direto ao ESP32!
✅ USE o regulador interno do L298N (borne +5V → ESP32)
```

**Jumper do Regulador:**  
Mantenha o jumper próximo ao borne de alimentação **LIGADO** para ativar o regulador de 5V.

**Documentação Técnica:**
- **L298N**: Consulte o datasheet do módulo L298N para especificações completas, diagramas de pinout e características elétricas
- **ESP32-S2 Mini**: Para informações detalhadas sobre alimentação, consumo e especificações técnicas, consulte o manual do módulo ESP32-S2 Mini

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

## 📡 MAC de Broadcast para Testes

No `transmit.ino`, mantenha a linha de broadcast comentada para testes iniciais:

```cpp
//uint8_t receiverMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
```