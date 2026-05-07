# ESP32-S2 Joystick Control System - Arduino IDE

Sistema de controle remoto usando ESP32-S2 Mini e ESP-NOW, com versões para diferentes níveis de complexidade.

## 🎯 Duas Versões Disponíveis

### 📁 Sistema Básico (`transmit.ino` + `receive.ino`)
**Para iniciantes e projetos simples**
- 1 joystick analógico (X, Y, botão)
- Controle tank drive de motores
- Sistema de timeout e proteção
- **📖 Guia**: [README_BASICO.md](README_BASICO.md)

### 🚀 Sistema Expandido (`expanded_transmit.ino` + `expanded_receive.ino`)  
**Para projetos avançados**
- Todos os recursos do básico +
- 2º joystick, potenciômetro, sensor luz
- 4 botões funcionais (turbo, luzes, buzzer, liga/desliga)
- Controles auxiliares (servo, LEDs, relé)
- **📖 Guia**: [README_EXPANSION.md](README_EXPANSION.md)

---

## 📁 Estrutura de Pastas

### Sistema Básico
#### **`transmit/`** - Transmissor Básico
- **Arquivo**: `transmit.ino` 
- **Hardware**: ESP32-S2 Mini + 1 joystick analógico
- **Pinos**: GPIO 7,9,5 (X,Y,botão) + GPIO 15 (LED)

#### **`receive/`** - Receptor Básico  
- **Arquivo**: `receive.ino`
- **Hardware**: ESP32-S2 Mini + L298N + 2 motores DC
- **Pinos**: GPIO 12,11,9,7 (motores) + GPIO 15 (LED)

### Sistema Expandido
#### **`expanded_transmit/`** - Transmissor Expandido
- **Arquivo**: `expanded_transmit.ino`
- **Hardware**: ESP32-S2 Mini + múltiplos sensores
- **Sensores**: 2 joysticks, potenciômetro, sensor luz, 4 botões

#### **`expanded_receive/`** - Receptor Expandido
- **Arquivo**: `expanded_receive.ino` 
- **Hardware**: ESP32-S2 Mini + L298N + controles auxiliares
- **Controles**: Motores, servo, buzzer, LEDs, relé

---

## 🚀 Início Rápido

### Escolha sua versão primeiro:
- **🟢 Iniciante?** Use o Sistema Básico
- **🟡 Avançado?** Use o Sistema Expandido

### Para ambas as versões:

#### **1. Configure o Receptor**
1. Upload do código receptor (`receive.ino` ou `expanded_receive.ino`)
2. Abra Serial Monitor (115200 baud)  
3. **Copie o MAC Address** exibido

#### **2. Configure o Transmissor**
1. No código transmissor, altere:
   ```cpp
   uint8_t receiverMAC[] = {0x20, 0x6E, 0xF1, 0x6D, 0x9F, 0xBC}; // ALTERE!
   ```
2. Upload do código transmissor

#### **3. Teste no Serial Monitor**
- **Transmissor**: Valores lidos dos sensores
- **Receptor**: Dados recebidos + comandos dos motores

---

## 🔌 Conexões de Hardware (Ambas as Versões)

### Alimentação via L298N (Recomendado)
```
Borne de parafuso do L298N:
+12V → Duas baterias lipo em série (7.4V-8.4V)
GND  → GND geral do circuito
+5V  → Alimentação +5V do ESP32

⚠️ NUNCA ligue as baterias direto ao ESP32!
✅ USE o regulador interno do L298N (borne +5V → ESP32)
```

**Jumper do Regulador:**  
Mantenha o jumper próximo ao borne **SEMPRE LIGADO** para ativar o regulador de 5V.

**Documentação Técnica:**
- **L298N**: Consulte o datasheet do módulo L298N para especificações completas e diagramas
- **ESP32-S2 Mini**: Para informações detalhadas sobre alimentação e especificações técnicas, consulte o manual do módulo ESP32-S2 Mini

---

## ⚙️ Configurações Importantes

### Proteção de Motores
Para motores 5V com baterias lítio (8.4V), altere:
```cpp
#define FATOR_DE_POTENCIA 0.6  // 60% = proteção contra sobretensão
```

### Configuração Arduino IDE
- **Placa**: "ESP32S2 Dev Module"
- **Serial**: 115200 baud
- **Bibliotecas**: Apenas ESP32 core (WiFi e ESP-NOW inclusos)

---

## 📊 Comparação das Versões

| Característica | Sistema Básico | Sistema Expandido |
|---|:---:|:---:|
| **Arquivos** | `transmit.ino`<br>`receive.ino` | `expanded_transmit.ino`<br>`expanded_receive.ino` |
| **Complexidade** | 🟢 Simples | 🟡 Intermediária |
| **Sensores** | 1 joystick | 2 joysticks + potenciômetro + LDR |
| **Botões** | 1 (sem função) | 5 (c/ funcionalidades) |
| **Controles** | Apenas motores | Motores + servo + buzzer + LEDs |
| **Tempo montagem** | 30 min | 2 horas |
| **Ideal para** | Aprendizado | Robô completo |

---

## 🛡️ Segurança

**Ambas as versões:**
- ⏰ Timeout: Para motores sem sinal (1s)
- 🔋 Proteção tensão: FATOR_DE_POTENCIA  
- 📡 RF otimizado: Máxima estabilidade

**Sistema expandido adiciona:**
- 🛑 Parada de emergência (botão joystick)
- 🔌 Chave geral (desliga tudo)
- 🔘 Debouncing automático

---

## 💡 Qual Sistema Escolher?

### Use o **Sistema Básico** se:
- ⭐ É seu primeiro projeto ESP32
- 🎯 Quer algo funcional rapidamente  
- 📦 Tem apenas 1 joystick
- 🚗 Precisa de controle simples

### Use o **Sistema Expandido** se:
- 🚀 Quer todas as funcionalidades
- 🎮 Tem múltiplos sensores
- 🤖 Está fazendo robô complexo
- 💡 Precisa de controles auxiliares

---

## 📞 Suporte

1. **📖 Leia o README específico** da sua versão
2. **🔍 Verifique Serial Monitor** para debug
3. **🔌 Confirme conexões** conforme pinout
4. **📡 Teste ESP-NOW** isoladamente

**Proteção via software:**
```cpp
#define FATOR_DE_POTENCIA 0.6   // Motor 5V + 2S lítio = SEGURO
```
Com 60%, a tensão efetiva fica: 8.4V × 0.6 = **5.0V** ✅

### **Calibração do Joystick:**
No arquivo `receive/receive.ino`, ajuste se necessário:
```cpp
AxisConfig x_axis_config = {
  .center = 2266,           // Valor central X
  .center_deadzone = 200,   // Zona morta (menor = mais sensível)
};

AxisConfig y_axis_config = {
  .center = 2221,           // Valor central Y  
  .center_deadzone = 200,   // Zona morta
};
```

## 🛠️ **Conexões de Hardware:**

### **Transmissor (Joystick):**
```
Joystick → ESP32-S2
GND      → GND  
+5V      → 3.3V
VRX      → GPIO 7
VRY      → GPIO 9
SW       → GPIO 5
```

### **Receptor (Motores):**
```
L298N → ESP32-S2
IN1   → GPIO 12 (motor direito)
IN2   → GPIO 11 (motor direito)
IN3   → GPIO 9 (motor esquerdo)
IN4   → GPIO 7 (motor esquerdo)
VCC   → 5V
GND   → GND

⚠️ IMPORTANTE: ENA e ENB devem estar jumpeados (sempre HIGH)
```

## 📡 **Características:**

- **Comunicação**: ESP-NOW (baixa latência)
- **Alcance**: ~200m linha de vista
- **Taxa de atualização**: 20Hz (50ms)
- **Potência RF**: Máxima (19.5dBm)
- **Controle**: Tank drive com 9 regiões (incluindo diagonais)
- **Segurança**: Timeout de 1 segundo para parar motores

## 🔍 **Debugging:**

- **LED piscando**: Dados sendo enviados/recebidos
- **LED fixo**: Erro na inicialização
- **3 piscadas no boot**: Inicialização OK
- **Serial Monitor**: Debug detalhado dos comandos

Estes arquivos são **independentes** do PlatformIO e funcionam diretamente no Arduino IDE! 🎯

## Nota de Pareamento

Nos transmissores (`transmit.ino` e `expanded_transmit.ino`), a opção de MAC de broadcast (`FF:FF:FF:FF:FF:FF`) permanece comentada para testes iniciais e não deve ser removida.

---

## 📂 **Estrutura de Pastas Arduino IDE:**

```
arduino_versions/
├── README.md              # Este arquivo de instruções
├── transmit/              # Pasta do sketch transmissor
│   └── transmit.ino      # Código do transmissor
└── receive/              # Pasta do sketch receptor  
    └── receive.ino       # Código do receptor
```

**⚠️ IMPORTANTE**: No Arduino IDE, cada sketch (.ino) deve estar em sua própria pasta com o mesmo nome do arquivo. Esta é a convenção padrão do Arduino IDE.