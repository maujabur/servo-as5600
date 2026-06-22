#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Preferences.h>

// --- Pins (ESP32-S3 Configuration) ---
const int motorIN1 = 1;  // Direction / PWM 1
const int motorIN2 = 2;  // Direction / PWM 2
const int motorENA = 4; // Enable Pin
const int inputPin = 5; // PWM or Analog Input

// I2C Pins (Default for standard ESP32-S3 are usually SDA=8, SCL=9. Change if needed via Wire.begin(SDA, SCL))

// --- Hardware PWM (LEDC) Configuration ---
const int pwmFreq = 20000;   // 20kHz to eliminate motor whine
const int pwmResolution = 8; // 8-bit (0-255)

// --- ADRC & Kinematics Variables ---
float wc, wo, b0; // Bandwidth_c, Bandwidth_o, System Gain
float tolerance, maxDeg, gearRatio;
float targetDeg = 0.0, currentDeg = 0.0;
bool servoEnabled = true;
int controlMode = 0; // 0: Web, 1: PWM, 2: Analog

// ESO (Extended State Observer) States
float z1 = 0.0; // Estimated Position
float z2 = 0.0; // Estimated Velocity
float z3 = 0.0; // Estimated Disturbance (Friction/Weight)
float lastOutput = 0.0;

// --- FreeRTOS & Time Variables ---
TaskHandle_t controlTaskHandle;
unsigned long lastLoopTime = 0;

// --- Motion Profiling & Safety ---
float maxVelocity = 300.0; // Max degrees per second allowed
float currentProfiledTarget = 0.0; 
unsigned long stallStartTime = 0;
bool isStalled = false;

// --- Smoothing/Filtering ---
float filterAlpha = 0.15; 

// --- Encoder Variables ---
int lastRaw = 0;
long totalRaw = 0;
long homeOffset = 0;
volatile unsigned long pulseStart = 0;
volatile int pulseWidth = 0;

Preferences prefs; 
AsyncWebServer server(80);

// --- Interrupt for External PWM ---
void IRAM_ATTR handlePWM() {
  if (digitalRead(inputPin) == HIGH) pulseStart = micros();
  else pulseWidth = micros() - pulseStart;
}

// --- Sensor Update ---
void updateEncoder() {
  Wire.beginTransmission(0x36);
  Wire.write(0x0E); // AS5600 Angle Register
  if (Wire.endTransmission() != 0) return; // Prevent bus hang if sensor disconnects

  Wire.requestFrom(0x36, 2);
  if (Wire.available() >= 2) {
    int raw = (Wire.read() << 8) | Wire.read();
    int diff = raw - lastRaw;
    
    if (diff > 2048) diff -= 4096;
    if (diff < -2048) diff += 4096;
    
    totalRaw += diff;
    lastRaw = raw;

    currentDeg = ((totalRaw - homeOffset) * (360.0 / 4096.0)) * gearRatio;
  }
}

void resetController() {
  isStalled = false;
  stallStartTime = 0;
  
  // Reset ADRC states to current reality to prevent violent jumps
  z1 = currentDeg;
  z2 = 0.0;
  z3 = 0.0;
  lastOutput = 0.0;
  
  targetDeg = currentDeg;
  currentProfiledTarget = currentDeg;
}

// --- Motor Output via Hardware PWM (Core v3.0.0+) ---
void driveMotor(float output) {
  if (!servoEnabled || isStalled) {
    ledcWrite(motorIN1, 0);
    ledcWrite(motorIN2, 0);
    lastOutput = 0.0;
    return;
  }

  // Deadzone to prevent jitter
  if (abs(currentProfiledTarget - currentDeg) < tolerance) {
    ledcWrite(motorIN1, 0);
    ledcWrite(motorIN2, 0);
    lastOutput = 0.0;
    return;
  }

  int speed = constrain(abs((int)output), 45, 255); 
  
  if (output > 0) {
    ledcWrite(motorIN1, speed);
    ledcWrite(motorIN2, 0);
    lastOutput = speed;
  } else {
    ledcWrite(motorIN1, 0);
    ledcWrite(motorIN2, speed);
    lastOutput = -speed;
  }
}

// --- Time-Based ADRC Core ---
void runADRC(float dt) {
  if (dt <= 0.0f || dt > 0.05f) return; // Prevent dt explosion

  // 1. Observer Gains
  float beta1 = 3.0f * wo;
  float beta2 = 3.0f * (wo * wo);
  float beta3 = (wo * wo * wo);

  // 2. Extended State Observer (ESO) Update
  float error_eso = z1 - currentDeg;
  
  z1 += dt * (z2 - beta1 * error_eso);
  z2 += dt * (z3 + (b0 * lastOutput) - beta2 * error_eso);
  z3 += dt * (-beta3 * error_eso); 

  // 3. Control Law (Virtual PD)
  // We track the 'currentProfiledTarget' instead of abrupt 'targetDeg' for smooth motion
  float kp = wc * wc;
  float kd = 2.0f * wc;
  
  float u0 = kp * (currentProfiledTarget - z1) - kd * z2;

  // 4. Disturbance Rejection
  float u = (u0 - z3) / b0;

  // Constrain to Arduino's 8-bit range (-255 to 255)
  float safeOutput = constrain(u, -255.0f, 255.0f);

  // --- Stall Protection ---
  // If output is maxed but estimated velocity (z2) is near zero
  if (abs(safeOutput) >= 250 && abs(z2) < 2.0) {
    if (stallStartTime == 0) stallStartTime = millis();
    else if (millis() - stallStartTime > 1500) { // 1.5 seconds of stall
      isStalled = true;
      Serial.println("STALL DETECTED! Motor Disabled.");
    }
  } else {
    stallStartTime = 0; 
  }
  
  driveMotor(safeOutput);
}

// --- Core 1 RTOS Task (Dedicated Control Loop) ---
void controlLoopTask(void * pvParameters) {
  lastLoopTime = micros();
  
  for(;;) {
    unsigned long now = micros();
    float dt = (now - lastLoopTime) / 1000000.0; // Delta time in seconds
    lastLoopTime = now;

    updateEncoder();

    // 1. Process Input
    float rawInput = targetDeg;
    if (controlMode == 1 && pulseWidth > 800 && pulseWidth < 2200) {
      rawInput = map(pulseWidth, 1000, 2000, 0, maxDeg);
    } else if (controlMode == 2) {
      int analogVal = 0;
      for(int i=0; i<4; i++) analogVal += analogRead(inputPin); 
      rawInput = map(analogVal / 4, 0, 4095, 0, maxDeg);
    }

    if (controlMode != 0) {
      targetDeg = (filterAlpha * rawInput) + ((1.0 - filterAlpha) * targetDeg);
    }

    // 2. Motion Profiling (Velocity Limiting)
    float maxChange = maxVelocity * dt;
    if (targetDeg > currentProfiledTarget + maxChange) {
      currentProfiledTarget += maxChange;
    } else if (targetDeg < currentProfiledTarget - maxChange) {
      currentProfiledTarget -= maxChange;
    } else {
      currentProfiledTarget = targetDeg;
    }

    // 3. Execute Control
    if (!isStalled) {
        runADRC(dt);
    }

    vTaskDelay(2 / portTICK_PERIOD_MS); // Run at ~500Hz
  }
}

// --- HTML Dashboard ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><title>Pro ADRC Servo</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { font-family: sans-serif; text-align: center; background: #1a1a1a; color: white; padding: 20px; }
  .card { background: #2d2d2d; padding: 20px; border-radius: 15px; display: inline-block; width: 400px; border: 1px solid #444; max-width: 100%; box-sizing: border-box; }
  .btn { padding: 12px; margin: 5px; cursor: pointer; border: none; border-radius: 8px; font-weight: bold; width: 45%; }
  .on { background: #2ecc71; color: white; }
  .save { background: #3498db; color: white; width: 93%; }
  .danger { background: #e74c3c; color: white; width: 93%; display: none; }
  input, select { width: 85%; padding: 10px; margin: 10px 0; border-radius: 5px; background: #444; color: white; border: none; }
  .row { display: flex; justify-content: space-around; align-items: center; }
  .row input { width: 60px; }
  .alert { background: #e74c3c; padding: 10px; border-radius: 5px; font-weight: bold; display: none; margin-bottom: 10px; }
  .stats { display: flex; justify-content: space-between; background: #111; padding: 10px; border-radius: 8px; margin-bottom: 15px; }
  .stat-box { width: 30%; }
  .stat-val { font-size: 1.5em; font-weight: bold; }
  .err-val { color: #e74c3c; }
</style></head>
<body>
  <div class="card">
    <h2>Smart Servo V2 (ADRC)</h2>
    <div id="stallAlert" class="alert">⚠️ MOTOR STALLED</div>
    
    <div class="stats">
      <div class="stat-box"><div style="font-size:12px; color:#aaa;">Target&deg;</div><div class="stat-val" id="tgt">0.0</div></div>
      <div class="stat-box"><div style="font-size:12px; color:#aaa;">Current&deg;</div><div class="stat-val" id="pos" style="color:#2ecc71;">0.0</div></div>
      <div class="stat-box"><div style="font-size:12px; color:#aaa;">Dist (z3)</div><div class="stat-val err-val" id="dist">0.0</div></div>
    </div>

    <button id="resetBtn" class="btn danger" onclick="resetStall()">CLEAR FAULT</button>
    <hr>
    <h4>Control & Limits</h4>
    <select id="mode">
      <option value="0">Web Dashboard</option>
      <option value="1">External PWM</option>
      <option value="2">Analog Signal</option>
    </select>
    <div class="row"> Ratio: <input type="text" id="ratio"> MaxLimits: <input type="text" id="mDeg"> </div>
    <hr>
    <h4>ADRC Tuning</h4>
    <div class="row"> &#969;c (Stiffness):<input type="text" id="wc"> &#969;o (Observer):<input type="text" id="wo"> </div>
    <div class="row"> b0 (Motor Gain):<input type="text" id="b0"> Tol&deg;:<input type="text" id="t"> </div>
    
    <button class="btn save" onclick="saveSettings()">APPLY & SAVE</button>
    <hr>
    <button class="btn on" onclick="fetch('/sethome')">SET ZERO</button>
    <input type="number" id="moveVal" placeholder="Degrees" style="width: 120px;">
    <button class="btn on" style="width:93%; background:#9b59b6" onclick="move()">WEB MOVE</button>
  </div>
<script>
  function move() { fetch('/move?val=' + document.getElementById('moveVal').value); }
  function resetStall() { 
    fetch('/resetstall'); 
    document.getElementById('stallAlert').style.display = 'none'; 
    document.getElementById('resetBtn').style.display = 'none'; 
  }
  function saveSettings() {
    const wc = document.getElementById('wc').value, wo = document.getElementById('wo').value, b0 = document.getElementById('b0').value;
    const t = document.getElementById('t').value, m = document.getElementById('mode').value;
    const max = document.getElementById('mDeg').value, r = document.getElementById('ratio').value;
    fetch(`/save?wc=${wc}&wo=${wo}&b0=${b0}&t=${t}&m=${m}&max=${max}&r=${r}`).then(() => alert("Saved!"));
  }
  fetch('/getparams').then(r => r.json()).then(data => {
    document.getElementById('wc').value = data.wc; document.getElementById('wo').value = data.wo;
    document.getElementById('b0').value = data.b0; document.getElementById('t').value = data.t;
    document.getElementById('mode').value = data.m; document.getElementById('mDeg').value = data.max;
    document.getElementById('ratio').value = data.r;
  });
  setInterval(() => { 
    fetch('/status').then(r => r.json()).then(data => { 
      document.getElementById('pos').innerText = data.pos; 
      document.getElementById('tgt').innerText = data.tgt;
      document.getElementById('dist').innerText = data.dist;
      if(data.stall) {
        document.getElementById('stallAlert').style.display = 'block';
        document.getElementById('resetBtn').style.display = 'inline-block';
      }
    }); 
  }, 250);
</script></body></html>)rawliteral";

void setup() {
  Serial.begin(115200);
  Wire.begin(5,6); // Depending on your S3 board, you may need Wire.begin(8, 9);
  Wire.setClock(1000000); // 1MHz I2C for faster encoder reading
  
  // Hardware Enable
  pinMode(motorENA, OUTPUT); 
  digitalWrite(motorENA, HIGH);
  pinMode(inputPin, INPUT);

  // Configure ESP32 LEDC (Hardware PWM) - Core 3.0.0+
  ledcAttach(motorIN1, pwmFreq, pwmResolution);
  ledcAttach(motorIN2, pwmFreq, pwmResolution);

  // Load Saved Data
  prefs.begin("servo-data", false);
  homeOffset = prefs.getLong("offset", 0);
  wc = prefs.getFloat("wc", 30.0);   
  wo = prefs.getFloat("wo", 100.0);
  b0 = prefs.getFloat("b0", 250.0);
  tolerance = prefs.getFloat("tol", 1.0);
  maxDeg = prefs.getFloat("maxDeg", 180.0);
  gearRatio = prefs.getFloat("ratio", 1.0);
  controlMode = prefs.getInt("mode", 0);

  // INITIAL ENCODER READ
  Wire.beginTransmission(0x36);
  Wire.write(0x0E);
  Wire.endTransmission();
  Wire.requestFrom(0x36, 2);
  if (Wire.available() >= 2) {
    lastRaw = (Wire.read() << 8) | Wire.read();
    totalRaw = lastRaw; 
  }
  updateEncoder();
  targetDeg = currentDeg; 
  currentProfiledTarget = currentDeg;
  
  // Init Observer
  z1 = currentDeg;
  z2 = 0.0;
  z3 = 0.0;

  if (controlMode == 1) attachInterrupt(digitalPinToInterrupt(inputPin), handlePWM, CHANGE);

  // Start FreeRTOS Task on Core 1
  xTaskCreatePinnedToCore(
    controlLoopTask,   // Task function
    "ADRC_Loop",       // Name
    4096,              // Stack size
    NULL,              // Parameters
    1,                 // Priority
    &controlTaskHandle,// Handle
    1                  // Core Number (0 is WiFi, 1 is App)
  );

  WiFi.softAP("SmartServo_ADRC", "");

  // API Routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *f){ f->send_P(200, "text/html", index_html); });
  
  server.on("/save", [](AsyncWebServerRequest *f){
    wc = f->getParam("wc")->value().toFloat();
    wo = f->getParam("wo")->value().toFloat();
    b0 = f->getParam("b0")->value().toFloat();
    tolerance = f->getParam("t")->value().toFloat();
    maxDeg = f->getParam("max")->value().toFloat();
    gearRatio = f->getParam("r")->value().toFloat();
    int newMode = f->getParam("m")->value().toInt();

    if (newMode == 1 && controlMode != 1) attachInterrupt(digitalPinToInterrupt(inputPin), handlePWM, CHANGE);
    else if (newMode != 1 && controlMode == 1) detachInterrupt(digitalPinToInterrupt(inputPin));
    
    controlMode = newMode;
    resetController();

    prefs.putFloat("wc", wc); prefs.putFloat("wo", wo); prefs.putFloat("b0", b0);
    prefs.putFloat("tol", tolerance); prefs.putInt("mode", controlMode);
    prefs.putFloat("maxDeg", maxDeg); prefs.putFloat("ratio", gearRatio);
    f->send(200);
  });

  server.on("/move", [](AsyncWebServerRequest *f){ targetDeg = f->getParam("val")->value().toFloat(); f->send(200); });
  server.on("/resetstall", [](AsyncWebServerRequest *f){ resetController(); f->send(200); });
  
  server.on("/getparams", [](AsyncWebServerRequest *f){
    String json = "{\"wc\":"+String(wc)+",\"wo\":"+String(wo)+",\"b0\":"+String(b0)+",\"t\":"+String(tolerance)+",\"m\":"+String(controlMode)+",\"max\":"+String(maxDeg)+",\"r\":"+String(gearRatio)+"}";
    f->send(200, "application/json", json);
  });
  
  server.on("/status", [](AsyncWebServerRequest *f){ 
    String json = "{\"pos\":\"" + String(currentDeg, 1) + "\",\"tgt\":\"" + String(targetDeg, 1) + "\",\"dist\":\"" + String(z3, 1) + "\",\"stall\":" + (isStalled ? "true" : "false") + "}";
    f->send(200, "application/json", json);
  });
  
  server.on("/sethome", [](AsyncWebServerRequest *f){ 
    homeOffset = totalRaw; 
    targetDeg = 0; currentProfiledTarget = 0; currentDeg = 0;
    z1 = 0; z2 = 0; z3 = 0; // Reset Observer
    prefs.putLong("offset", homeOffset); 
    f->send(200); 
  });

  server.begin();
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS); 
}