#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#endif

// WiFi credentials
const char *ssid = "Huynh Thi Thu";
const char *password = "huynhthithu789";

// WebServer on port 80
WebServer server(80);
// WebSocket server on port 81
WebSocketsServer webSocket(81);

// LED Flash control
bool flashState = false;
#if defined(LED_GPIO_NUM)
#define FLASH_PIN LED_GPIO_NUM
#else
#define FLASH_PIN 4 // Default flash pin for AI-Thinker
#endif

// Camera streaming state
bool streamingEnabled = true;

// ========== ROBOT ARM CONTROL SYSTEM ==========
// Áp dụng từ armrobot.ino với cải tiến cho ESP32-CAM

//Tao struct ServoPins gom cac thong so cua Servo nhu: chan servo, ten servo, goc khoi dau cua servo
struct ServoPins
{
  Servo servo;
  int servoPin;
  String servoName;
  int initialPosition;  
};

//Su dung struct de khai bao 4 servo (pins được chọn tương thích với ESP32-CAM)
std::vector<ServoPins> servoPins = 
{
  { Servo(), 2 , "Base", 90},      // GPIO2 - Safe for ESP32-CAM
  { Servo(), 12 , "Shoulder", 90}, // GPIO12 - Safe for ESP32-CAM  
  { Servo(), 15 , "Elbow", 90},    // GPIO15 - Safe for ESP32-CAM
  { Servo(), 13 , "Gripper", 90},  // GPIO13 - Safe for ESP32-CAM
};

//Struct de luu tru cac buoc khi Recording (chi so servo, cac gia tri trong do)
struct RecordedStep
{
  int servoIndex;
  int value;
  int delayInStep;
};

std::vector<RecordedStep> recordedSteps;

//Khai bao cac bien check qua trinh record dang dien ra hay dang play 
bool recordSteps = false;
bool playRecordedSteps = false;
unsigned long previousTimeInMilli = millis();

// Servo positions tracking
struct RobotArmPosition
{
  int base = 90;
  int shoulder = 90;
  int elbow = 90;
  int gripper = 90;
};

RobotArmPosition currentPosition;

// Servo control settings
int servoSpeed = 15;    // Speed delay in ms
bool autoMode = false;

// Auto mode pattern
struct AutoPattern
{
  RobotArmPosition positions[4];
  int stepCount;
  int currentStep;
  unsigned long lastStepTime;
};

AutoPattern autoPattern;

// ========== HTML INTERFACE ==========
// Giao diện kết hợp cả camera và robot arm control
const char* htmlHomePage PROGMEM = R"HTMLHOMEPAGE(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32-CAM Robot Arm Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    
    body { 
      font-family: 'Arial', sans-serif; 
      background: linear-gradient(135deg, #1a1a2e, #16213e, #0f3460);
      color: white;
      min-height: 100vh;
      padding: 10px;
    }
    
    .container {
      max-width: 1200px;
      margin: 0 auto;
      display: grid;
      grid-template-columns: 1fr 400px;
      gap: 20px;
      min-height: 100vh;
    }
    
    .left-panel {
      display: flex;
      flex-direction: column;
      gap: 20px;
    }
    
    .right-panel {
      background: rgba(255,255,255,0.1);
      border-radius: 15px;
      padding: 20px;
      backdrop-filter: blur(10px);
      border: 1px solid rgba(255,255,255,0.2);
      height: fit-content;
    }
    
    h1 {
      color: #00d4ff;
      text-align: center;
      margin-bottom: 20px;
      font-size: 2.5em;
      text-shadow: 0 0 20px rgba(0, 212, 255, 0.5);
    }
    
    h2 {
      color: #ff6b35;
      margin-bottom: 15px;
      font-size: 1.5em;
      border-bottom: 2px solid #ff6b35;
      padding-bottom: 5px;
    }
    
    /* Camera Section */
    .camera-section {
      background: rgba(0,0,0,0.3);
      border-radius: 15px;
      padding: 20px;
      border: 2px solid #00d4ff;
    }
    
    .stream-container { 
      margin: 20px 0; 
      position: relative;
      border: 2px solid #00d4ff; 
      border-radius: 10px;
      overflow: hidden;
      box-shadow: 0 8px 32px rgba(0, 212, 255, 0.3);
      background: #000;
    }
    
    #stream { 
      width: 100%; 
      height: auto; 
      display: block;
      image-rendering: -webkit-optimize-contrast;
      image-rendering: crisp-edges;
    }
    
    .fps-counter {
      position: absolute;
      top: 10px;
      right: 10px;
      background: rgba(0,0,0,0.8);
      color: #00d4ff;
      padding: 5px 10px;
      border-radius: 5px;
      font-family: monospace;
      font-size: 12px;
    }
    
    .status {
      margin: 10px 0;
      padding: 8px 16px;
      border-radius: 20px;
      font-weight: bold;
      display: inline-block;
      text-align: center;
    }
    
    .status.connected {
      background: linear-gradient(45deg, #4CAF50, #45a049);
      box-shadow: 0 4px 15px rgba(76, 175, 80, 0.4);
    }
    
    .status.disconnected {
      background: linear-gradient(45deg, #f44336, #d32f2f);
      box-shadow: 0 4px 15px rgba(244, 67, 54, 0.4);
    }
    
    button { 
      padding: 12px 20px; 
      font-size: 14px; 
      margin: 5px;
      border: none;
      border-radius: 8px;
      cursor: pointer;
      transition: all 0.3s;
      font-weight: bold;
      box-shadow: 0 4px 15px rgba(0,0,0,0.2);
    }
    
    button:hover {
      transform: translateY(-2px);
      box-shadow: 0 6px 20px rgba(0,0,0,0.3);
    }
    
    .camera-btn {
      background: linear-gradient(45deg, #4CAF50, #45a049);
      color: white;
    }
    
    .flash-btn {
      background: linear-gradient(45deg, #FF9800, #F57C00);
      color: white;
    }
    
    /* Robot Arm Control Section */
    .arm-control {
      background: rgba(255,255,255,0.05);
      border-radius: 10px;
      padding: 15px;
      margin: 10px 0;
      border: 1px solid rgba(255,255,255,0.1);
    }
    
    .servo-row {
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin: 15px 0;
      padding: 10px;
      background: rgba(0,0,0,0.2);
      border-radius: 8px;
    }
    
    .servo-label {
      font-weight: bold;
      color: #00d4ff;
      min-width: 80px;
      font-size: 14px;
    }
    
    .slider {
      -webkit-appearance: none;
      flex: 1;
      margin: 0 15px;
      height: 8px;
      border-radius: 4px;
      background: rgba(255,255,255,0.2);
      outline: none;
      transition: all 0.3s;
    }
    
    .slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 24px;
      height: 24px;
      border-radius: 50%;
      background: linear-gradient(45deg, #ff6b35, #f7931e);
      cursor: pointer;
      box-shadow: 0 0 15px rgba(255, 107, 53, 0.6);
      transition: all 0.3s;
    }
    
    .slider::-webkit-slider-thumb:hover {
      transform: scale(1.2);
      box-shadow: 0 0 20px rgba(255, 107, 53, 0.8);
    }
    
    .servo-value {
      background: rgba(0,0,0,0.3);
      padding: 5px 10px;
      border-radius: 5px;
      font-family: monospace;
      color: #ff6b35;
      min-width: 50px;
      text-align: center;
      font-size: 14px;
      font-weight: bold;
    }
    
    .control-buttons {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin: 15px 0;
    }
    
    .record-btn {
      background: linear-gradient(45deg, #f44336, #d32f2f);
      color: white;
      font-size: 12px;
      padding: 8px;
    }
    
    .record-btn.active {
      background: linear-gradient(45deg, #4CAF50, #388E3C);
    }
    
    .play-btn {
      background: linear-gradient(45deg, #2196F3, #1976D2);
      color: white;
      font-size: 12px;
      padding: 8px;
    }
    
    .play-btn.active {
      background: linear-gradient(45deg, #4CAF50, #388E3C);
    }
    
    .preset-buttons {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      margin: 15px 0;
    }
    
    .preset-btn {
      background: linear-gradient(45deg, #9c27b0, #7b1fa2);
      color: white;
      padding: 8px;
      font-size: 11px;
    }
    
    .speed-control {
      margin: 15px 0;
      text-align: center;
    }
    
    .speed-control label {
      display: block;
      margin-bottom: 8px;
      font-size: 12px;
      color: #00d4ff;
    }
    
    .auto-btn {
      background: linear-gradient(45deg, #607d8b, #455a64);
      color: white;
      width: 100%;
      margin: 10px 0;
    }
    
    .auto-btn.active {
      background: linear-gradient(45deg, #4CAF50, #388E3C);
    }
    
    .info-panel {
      background: rgba(0,0,0,0.3);
      border-radius: 8px;
      padding: 10px;
      margin: 15px 0;
      font-size: 11px;
      opacity: 0.8;
    }
    
    /* Responsive */
    @media (max-width: 1024px) {
      .container {
        grid-template-columns: 1fr;
        gap: 15px;
      }
      
      .right-panel {
        order: -1;
      }
    }
    
    @media (max-width: 768px) {
      .servo-row {
        flex-direction: column;
        align-items: stretch;
        gap: 8px;
      }
      
      .slider {
        margin: 8px 0;
      }
      
      .control-buttons {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>
<body class="noselect">
  <div class="container">
    <div class="left-panel">
      <h1>🤖 ESP32-CAM Robot Arm</h1>
      
      <!-- Camera Section -->
      <div class="camera-section">
        <h2>📷 Live Camera Feed</h2>
        <div id="status" class="status disconnected">Disconnected</div>
        
        <div class="stream-container">
          <div class="fps-counter" id="fpsCounter">FPS: 0</div>
          <img id="stream" src="" alt="Loading stream...">
        </div>
        
        <div style="text-align: center;">
          <button id="streamBtn" class="camera-btn" onclick="toggleStream()">Stop Stream</button>
          <button id="flashBtn" class="flash-btn" onclick="toggleFlash()">Flash ON</button>
        </div>
        
        <div style="text-align: center; margin-top: 15px; font-size: 12px; opacity: 0.7;">
          <div>Resolution: 320x240 | Quality: Optimized</div>
          <div id="streamInfo">Stream: Active</div>
        </div>
      </div>
    </div>
    
    <div class="right-panel">
      <h2>🦾 Robot Arm Control</h2>
      
      <!-- Servo Controls -->
      <div class="arm-control">
        <div class="servo-row">
          <span class="servo-label">Base:</span>
          <input type="range" min="0" max="180" value="90" class="slider" id="Base" oninput='sendServoCommand("Base", this.value)'>
          <span class="servo-value" id="BaseValue">90°</span>
        </div>
        
        <div class="servo-row">
          <span class="servo-label">Shoulder:</span>
          <input type="range" min="0" max="180" value="90" class="slider" id="Shoulder" oninput='sendServoCommand("Shoulder", this.value)'>
          <span class="servo-value" id="ShoulderValue">90°</span>
        </div>
        
        <div class="servo-row">
          <span class="servo-label">Elbow:</span>
          <input type="range" min="0" max="180" value="90" class="slider" id="Elbow" oninput='sendServoCommand("Elbow", this.value)'>
          <span class="servo-value" id="ElbowValue">90°</span>
        </div>
        
        <div class="servo-row">
          <span class="servo-label">Gripper:</span>
          <input type="range" min="0" max="180" value="90" class="slider" id="Gripper" oninput='sendServoCommand("Gripper", this.value)'>
          <span class="servo-value" id="GripperValue">90°</span>
        </div>
      </div>
      
      <!-- Control Buttons -->
      <div class="control-buttons">
        <button id="recordBtn" class="record-btn" onclick="toggleRecord()">Record OFF</button>
        <button id="playBtn" class="play-btn" onclick="togglePlay()">Play OFF</button>
      </div>
      
      <!-- Preset Positions -->
      <div class="preset-buttons">
        <button class="preset-btn" onclick="setPreset('home')">🏠 Home</button>
        <button class="preset-btn" onclick="setPreset('pickup')">📦 Pick Up</button>
        <button class="preset-btn" onclick="setPreset('reach')">🤏 Reach</button>
        <button class="preset-btn" onclick="setPreset('wave')">👋 Wave</button>
      </div>
      
      <!-- Speed Control -->
      <div class="speed-control">
        <label>Movement Speed:</label>
        <input type="range" min="1" max="50" value="15" class="slider" id="speedSlider" oninput="updateSpeed(this.value)">
        <div style="font-size: 12px; margin-top: 5px;" id="speedValue">Speed: 15ms</div>
      </div>
      
      <!-- Auto Mode -->
      <button id="autoBtn" class="auto-btn" onclick="toggleAuto()">Auto Mode</button>
      
      <!-- Info Panel -->
      <div class="info-panel">
        <div><strong>📊 Robot Status:</strong></div>
        <div id="armStatus">All servos: Ready</div>
        <div style="margin-top: 8px;"><strong>🎮 Controls:</strong></div>
        <div>• Move sliders to control servos</div>
        <div>• Record/Play sequences</div>
        <div>• Use presets for quick positions</div>
      </div>
    </div>
  </div>
  
  <script>
    // WebSocket connection
    const ws = new WebSocket('ws://' + window.location.hostname + ':81/');
    let streamEnabled = true;
    let flashEnabled = false;
    let autoMode = false;
    let recordMode = false;
    let playMode = false;
    let frameCount = 0;
    let lastTime = Date.now();
    
    // WebSocket event handlers
    ws.onopen = function(event) {
      console.log('WebSocket connected');
      document.getElementById('status').textContent = 'Connected';
      document.getElementById('status').className = 'status connected';
    };
    
    ws.onclose = function(event) {
      console.log('WebSocket disconnected');
      document.getElementById('status').textContent = 'Disconnected';
      document.getElementById('status').className = 'status disconnected';
      // Auto reconnect
      setTimeout(() => {
        window.location.reload();
      }, 3000);
    };
    
    ws.onmessage = function(event) {
      console.log('Server:', event.data);
      try {
        const data = JSON.parse(event.data);
        if (data.type === 'servo_update') {
          updateServoDisplay(data.servo, data.value);
        } else if (data.type === 'arm_status') {
          document.getElementById('armStatus').textContent = data.message;
        }
      } catch (e) {
        // Handle non-JSON messages (from armrobot protocol)
        const parts = event.data.split(',');
        if (parts.length === 2) {
          const servo = parts[0];
          const value = parts[1];
          updateServoDisplay(servo, parseInt(value));
        }
      }
    };
    
    // Servo control functions
    function sendServoCommand(servo, value) {
      document.getElementById(servo + 'Value').textContent = value + '°';
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(servo + ',' + value);
      }
    }
    
    function updateServoDisplay(servo, value) {
      const slider = document.getElementById(servo);
      const display = document.getElementById(servo + 'Value');
      if (slider && display) {
        slider.value = value;
        display.textContent = value + '°';
      }
    }
    
    // Record/Play functions
    function toggleRecord() {
      recordMode = !recordMode;
      const btn = document.getElementById('recordBtn');
      
      if (recordMode) {
        btn.textContent = 'Record ON';
        btn.classList.add('active');
        enableDisableControls(false);
      } else {
        btn.textContent = 'Record OFF';
        btn.classList.remove('active');
        enableDisableControls(true);
      }
      
      if (ws.readyState === WebSocket.OPEN) {
        ws.send('Record,' + (recordMode ? '1' : '0'));
      }
    }
    
    function togglePlay() {
      playMode = !playMode;
      const btn = document.getElementById('playBtn');
      
      if (playMode) {
        btn.textContent = 'Play ON';
        btn.classList.add('active');
        enableDisableControls(false);
      } else {
        btn.textContent = 'Play OFF';
        btn.classList.remove('active');
        enableDisableControls(true);
      }
      
      if (ws.readyState === WebSocket.OPEN) {
        ws.send('Play,' + (playMode ? '1' : '0'));
      }
    }
    
    function enableDisableControls(enabled) {
      const controls = ['Base', 'Shoulder', 'Elbow', 'Gripper'];
      controls.forEach(control => {
        document.getElementById(control).disabled = !enabled;
        document.getElementById(control).style.pointerEvents = enabled ? 'auto' : 'none';
      });
      
      if (playMode) {
        document.getElementById('recordBtn').style.pointerEvents = 'none';
      } else if (recordMode) {
        document.getElementById('playBtn').style.pointerEvents = 'none';
      } else {
        document.getElementById('recordBtn').style.pointerEvents = 'auto';
        document.getElementById('playBtn').style.pointerEvents = 'auto';
      }
    }
    
    // Preset positions
    function setPreset(preset) {
      let positions = {};
      
      switch(preset) {
        case 'home':
          positions = {Base: 90, Shoulder: 90, Elbow: 90, Gripper: 90};
          break;
        case 'pickup':
          positions = {Base: 90, Shoulder: 130, Elbow: 60, Gripper: 0};
          break;
        case 'reach':
          positions = {Base: 45, Shoulder: 45, Elbow: 90, Gripper: 90};
          break;
        case 'wave':
          positions = {Base: 135, Shoulder: 45, Elbow: 100, Gripper: 90};
          break;
      }
      
      Object.keys(positions).forEach(servo => {
        sendServoCommand(servo, positions[servo]);
      });
    }
    
    // Speed control
    function updateSpeed(value) {
      document.getElementById('speedValue').textContent = 'Speed: ' + value + 'ms';
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({type: 'speed', value: parseInt(value)}));
      }
    }
    
    // Auto mode
    function toggleAuto() {
      autoMode = !autoMode;
      const btn = document.getElementById('autoBtn');
      
      if (autoMode) {
        btn.textContent = 'Stop Auto';
        btn.classList.add('active');
      } else {
        btn.textContent = 'Auto Mode';
        btn.classList.remove('active');
      }
      
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({type: 'auto', value: autoMode}));
      }
    }
    
    // Camera functions
    function updateFPS() {
      const now = Date.now();
      const timeDiff = now - lastTime;
      if (timeDiff >= 1000) {
        const fps = Math.round((frameCount * 1000) / timeDiff);
        document.getElementById('fpsCounter').textContent = 'FPS: ' + fps;
        frameCount = 0;
        lastTime = now;
      }
    }
    
    function updateStream() {
      if (!streamEnabled) {
        document.getElementById('stream').src = '';
        document.getElementById('stream').style.display = 'none';
        document.getElementById('streamInfo').textContent = 'Stream: Stopped';
        return;
      }
      
      const streamImg = document.getElementById('stream');
      const timestamp = Date.now();
      const newSrc = '/stream?t=' + timestamp;
      
      const img = new Image();
      img.onload = function() {
        streamImg.src = newSrc;
        streamImg.style.display = 'block';
        frameCount++;
        updateFPS();
        setTimeout(updateStream, 33); // ~30 FPS
      };
      
      img.onerror = function() {
        setTimeout(updateStream, 100);
      };
      
      img.src = newSrc;
      document.getElementById('streamInfo').textContent = 'Stream: Active';
    }
    
    function toggleStream() {
      streamEnabled = !streamEnabled;
      const btn = document.getElementById('streamBtn');
      
      if (streamEnabled) {
        btn.textContent = 'Stop Stream';
        updateStream();
      } else {
        btn.textContent = 'Start Stream';
        updateStream();
      }
      
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({type: 'stream', value: streamEnabled}));
      }
    }
    
    function toggleFlash() {
      flashEnabled = !flashEnabled;
      const btn = document.getElementById('flashBtn');
      
      if (flashEnabled) {
        btn.textContent = 'Flash OFF';
        btn.style.background = 'linear-gradient(45deg, #FFC107, #FF8F00)';
      } else {
        btn.textContent = 'Flash ON';
        btn.style.background = 'linear-gradient(45deg, #FF9800, #F57C00)';
      }
      
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({type: 'flash', value: flashEnabled}));
      }
    }
    
    // Initialize
    document.addEventListener('DOMContentLoaded', function() {
      updateStream();
    });
    
    // Handle page visibility
    document.addEventListener('visibilitychange', function() {
      if (document.hidden) {
        streamEnabled = false;
      } else if (document.getElementById('streamBtn').textContent.includes('Stop')) {
        streamEnabled = true;
        updateStream();
      }
    });
    
    // Prevent context menu and selection
    document.addEventListener('contextmenu', e => e.preventDefault());
    document.addEventListener('selectstart', e => e.preventDefault());
  </script>
</body>    
</html>
)HTMLHOMEPAGE";

// ========== FUNCTIONS ==========

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("🚀 ESP32-CAM Robot Arm Starting...");

  // Initialize servos first
  initializeServos();
  
  // Initialize auto pattern
  initializeAutoPattern();

  // Initialize camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA; // QVGA (320x240) for higher FPS
  config.pixel_format = PIXFORMAT_JPEG;
  config.jpeg_quality = 15; // Lower quality for faster streaming
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Camera sensor configuration
  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV2640_PID)
  {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_framesize(s, FRAMESIZE_QVGA);
    s->set_quality(s, 15);
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_special_effect(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 0);
    s->set_ae_level(s, 0);
    s->set_aec_value(s, 300);
    s->set_gain_ctrl(s, 1);
    s->set_agc_gain(s, 0);
    s->set_gainceiling(s, (gainceiling_t)0);
    s->set_bpc(s, 0);
    s->set_wpc(s, 1);
    s->set_raw_gma(s, 1);
    s->set_lenc(s, 1);
    s->set_dcw(s, 1);
    s->set_colorbar(s, 0);
  }

  // Initialize LED Flash
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(FLASH_PIN, LOW);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  // Web Server Routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/stream", HTTP_GET, handleStream);
  server.on("/arm-status", HTTP_GET, handleArmStatus);
  server.on("/info", HTTP_GET, handleInfo);
  server.onNotFound(handleNotFound);

  // Start servers
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Serial.println("🚀 ESP32-CAM Robot Arm Ready!");
  Serial.printf("📱 Web Interface: http://%s\n", WiFi.localIP().toString().c_str());
  Serial.printf("🦾 Robot Arm Control: Servos initialized\n");
  Serial.printf("📊 System Info: http://%s/info\n", WiFi.localIP().toString().c_str());
}

// ========== SERVO CONTROL FUNCTIONS ==========

void initializeServos()
{
  Serial.println("🦾 Initializing Robot Arm Servos...");

  for (int i = 0; i < servoPins.size(); i++)
  {
    Serial.printf("  Attaching servo %s to pin %d...", 
                  servoPins[i].servoName.c_str(), servoPins[i].servoPin);
    
    servoPins[i].servo.attach(servoPins[i].servoPin);
    delay(50); // Give servo time to attach
    
    // Verify attachment
    if (servoPins[i].servo.attached()) {
      Serial.println(" ✓");
    } else {
      Serial.println(" ✗ FAILED!");
      continue;
    }
    
    // Apply mapping for Shoulder servo during initialization
    int initialValue = servoPins[i].initialPosition;
    if (i == 1) { // Shoulder servo - apply inverse mapping like armrobot
      initialValue = mapShoulderAngle(servoPins[i].initialPosition);
      Serial.printf("    %s: UI %d° -> Servo %d° (with mapping)\n", 
                   servoPins[i].servoName.c_str(), 
                   servoPins[i].initialPosition, 
                   initialValue);
    }
    
    servoPins[i].servo.write(initialValue);
    delay(100); // Give servo time to move
    
    // Update current position
    if (i == 0) currentPosition.base = servoPins[i].initialPosition;
    else if (i == 1) currentPosition.shoulder = servoPins[i].initialPosition;
    else if (i == 2) currentPosition.elbow = servoPins[i].initialPosition;
    else if (i == 3) currentPosition.gripper = servoPins[i].initialPosition;
    
    Serial.printf("    %s: Initialized at UI position %d°\n", 
                  servoPins[i].servoName.c_str(), servoPins[i].initialPosition);
  }
  
  Serial.println("✅ Servo initialization complete.");
}

void initializeAutoPattern()
{
  // Define 4 positions for auto mode pattern
  autoPattern.stepCount = 4;
  autoPattern.currentStep = 0;
  autoPattern.lastStepTime = 0;
  
  // Position 1: Home
  autoPattern.positions[0] = {90, 90, 90, 90};
  // Position 2: Reach
  autoPattern.positions[1] = {45, 45, 90, 90};
  // Position 3: Pick
  autoPattern.positions[2] = {90, 130, 60, 0};
  // Position 4: Wave
  autoPattern.positions[3] = {135, 45, 100, 90};
}

// Servo mapping function - exactly like armrobot.ino
int mapShoulderAngle(int uiAngle) {
  // Convert from UI (0-180°) to actual servo (180-0°) - inverted
  int servoAngle = map(uiAngle, 0, 180, 180, 0);
  Serial.printf("Shoulder mapping: UI %d° -> Servo %d° (inverted)\n", uiAngle, servoAngle);
  return servoAngle;
}

// Write servo values with recording capability - from armrobot.ino
void writeServoValues(int servoIndex, int value)
{
  if (servoIndex < 0 || servoIndex >= servoPins.size()) {
    Serial.printf("ERROR: Invalid servo index %d\n", servoIndex);
    return;
  }

  if (recordSteps)
  {
    RecordedStep recordedStep;       
    if (recordedSteps.size() == 0) // First record initial position of all servos
    {
      for (int i = 0; i < servoPins.size(); i++)
      {
        recordedStep.servoIndex = i; 
        recordedStep.value = servoPins[i].servo.read(); 
        recordedStep.delayInStep = 0;
        recordedSteps.push_back(recordedStep);         
      }      
    }
    unsigned long currentTime = millis();
    recordedStep.servoIndex = servoIndex; 
    recordedStep.value = value; 
    recordedStep.delayInStep = currentTime - previousTimeInMilli;
    recordedSteps.push_back(recordedStep);  
    previousTimeInMilli = currentTime;         
  }
  
  // Apply mapping for Shoulder servo (index 1)
  int servoValue = value;
  if (servoIndex == 1) { // Shoulder servo
    servoValue = mapShoulderAngle(value);
  }
  
  Serial.printf("writeServoValues: Servo %s (index %d) - UI: %d° -> Servo: %d°\n", 
                servoPins[servoIndex].servoName.c_str(), servoIndex, value, servoValue);
  
  // Write to servo
  servoPins[servoIndex].servo.write(servoValue);
  
  // Update current position tracking
  if (servoIndex == 0) currentPosition.base = value;
  else if (servoIndex == 1) currentPosition.shoulder = value;
  else if (servoIndex == 2) currentPosition.elbow = value;
  else if (servoIndex == 3) currentPosition.gripper = value;
}

// Smooth servo movement
void moveServo(Servo &servo, int current, int target, int speed)
{
  if (current == target) return;
  
  int step = (current < target) ? 1 : -1;
  for (int pos = current; pos != target; pos += step)
  {
    servo.write(pos);
    delay(speed);
  }
  servo.write(target); // Ensure final position
}

// Move to specific position
void moveToPosition(RobotArmPosition target)
{
  moveServo(servoPins[0].servo, currentPosition.base, target.base, servoSpeed);
  moveServo(servoPins[1].servo, currentPosition.shoulder, target.shoulder, servoSpeed);
  moveServo(servoPins[2].servo, currentPosition.elbow, target.elbow, servoSpeed);
  moveServo(servoPins[3].servo, currentPosition.gripper, target.gripper, servoSpeed);

  currentPosition = target;
}

// Play recorded steps - from armrobot.ino
void playRecordedRobotArmSteps()
{
  if (recordedSteps.size() == 0)
  {
    return;
  }
  
  // Move to initial position slowly. First 4 steps are initial position    
  for (int i = 0; i < 4 && playRecordedSteps; i++)
  {
    RecordedStep &recordedStep = recordedSteps[i];
    int currentServoPosition = servoPins[recordedStep.servoIndex].servo.read();
    
    // Apply mapping for target position if Shoulder servo
    int targetPosition = recordedStep.value;
    if (recordedStep.servoIndex == 1) { // Shoulder servo
      targetPosition = mapShoulderAngle(recordedStep.value);
    }
    
    while (currentServoPosition != targetPosition && playRecordedSteps)  
    {
      currentServoPosition = (currentServoPosition > targetPosition ? currentServoPosition - 1 : currentServoPosition + 1); 
      servoPins[recordedStep.servoIndex].servo.write(currentServoPosition);
      String message = servoPins[recordedStep.servoIndex].servoName + "," + String(recordedStep.value);
      webSocket.broadcastTXT(message);
      delay(50);
    }
  }
  
  delay(2000); // Delay before starting the actual steps
  
  for (int i = 4; i < recordedSteps.size() && playRecordedSteps; i++)
  {
    RecordedStep &recordedStep = recordedSteps[i];
    delay(recordedStep.delayInStep);
    
    // Apply mapping for servo value if Shoulder servo
    int servoValue = recordedStep.value;
    if (recordedStep.servoIndex == 1) { // Shoulder servo
      servoValue = mapShoulderAngle(recordedStep.value);
    }
    
    servoPins[recordedStep.servoIndex].servo.write(servoValue);
    String message = servoPins[recordedStep.servoIndex].servoName + "," + String(recordedStep.value);
    webSocket.broadcastTXT(message);
  }
}

// Send current robot arm state - from armrobot.ino
void sendCurrentRobotArmState(uint8_t num)
{
  for (int i = 0; i < servoPins.size(); i++)
  {
    int currentValue = servoPins[i].servo.read();
    
    // If Shoulder servo, convert from servo value (180-0°) to UI value (0-180°)
    if (i == 1) { // Shoulder servo
      currentValue = map(currentValue, 180, 0, 0, 180);
      Serial.printf("Send state - Shoulder: Servo %d° -> UI %d° (inverted)\n", servoPins[i].servo.read(), currentValue);
    }
    
    String message = servoPins[i].servoName + "," + String(currentValue);
    webSocket.sendTXT(num, message);
  }
  webSocket.sendTXT(num, String("Record,") + (recordSteps ? "ON" : "OFF"));
  webSocket.sendTXT(num, String("Play,") + (playRecordedSteps ? "ON" : "OFF"));
}

// ========== WEB SERVER HANDLERS ==========

void handleRoot() 
{
  server.send(200, "text/html", htmlHomePage);
}

void handleStream()
{
  if (!streamingEnabled) {
    server.send(503, "text/plain", "Streaming disabled");
    return;
  }
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    server.send(500, "text/plain", "Camera Error");
    return;
  }
  
  server.sendHeader("Content-Type", "image/jpeg");
  server.sendHeader("Content-Length", String(fb->len));
  server.sendHeader("Cache-Control", "no-cache, no-store, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "Thu, 01 Jan 1970 00:00:00 GMT");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
  
  esp_camera_fb_return(fb);
}

void handleArmStatus()
{
  DynamicJsonDocument doc(512);
  doc["base"] = currentPosition.base;
  doc["shoulder"] = currentPosition.shoulder;
  doc["elbow"] = currentPosition.elbow;
  doc["gripper"] = currentPosition.gripper;
  doc["speed"] = servoSpeed;
  doc["autoMode"] = autoMode;
  doc["recording"] = recordSteps;
  doc["playing"] = playRecordedSteps;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleInfo()
{
  String info = "ESP32-CAM Robot Arm Status:\n";
  info += "Free Heap: " + String(ESP.getFreeHeap()) + " bytes\n";
  info += "WiFi RSSI: " + String(WiFi.RSSI()) + " dBm\n";
  info += "Streaming: " + String(streamingEnabled ? "ON" : "OFF") + "\n";
  info += "Flash: " + String(flashState ? "ON" : "OFF") + "\n";
  info += "Robot Arm:\n";
  info += "  Base: " + String(currentPosition.base) + "°\n";
  info += "  Shoulder: " + String(currentPosition.shoulder) + "°\n";
  info += "  Elbow: " + String(currentPosition.elbow) + "°\n";
  info += "  Gripper: " + String(currentPosition.gripper) + "°\n";
  info += "  Speed: " + String(servoSpeed) + "ms\n";
  info += "  Auto Mode: " + String(autoMode ? "ON" : "OFF") + "\n";
  info += "  Recording: " + String(recordSteps ? "ON" : "OFF") + "\n";
  info += "  Playing: " + String(playRecordedSteps ? "ON" : "OFF") + "\n";
  server.send(200, "text/plain", info);
}

void handleNotFound() 
{
  server.send(404, "text/plain", "File Not found");
}

// ========== WEBSOCKET EVENT HANDLER ==========

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
    case WStype_DISCONNECTED:
      Serial.printf("WebSocket client #%u disconnected\n", num);
      break;
      
    case WStype_CONNECTED:
      Serial.printf("WebSocket client #%u connected\n", num);
      sendCurrentRobotArmState(num);
      break;
      
    case WStype_TEXT:
      if (length > 0)
      {
        String message = String((char*)payload);
        Serial.printf("Received: %s\n", message.c_str());
        
        // Try JSON first (new protocol)
        DynamicJsonDocument doc(512);
        if (deserializeJson(doc, message) == DeserializationError::Ok) {
          handleJsonMessage(doc, num);
        }
        // Fall back to armrobot protocol (comma-separated)
        else {
          handleArmrobotMessage(message, num);
        }
      }
      break;
      
    default:
      break;
  }
}

void handleJsonMessage(DynamicJsonDocument& doc, uint8_t num)
{
  String messageType = doc["type"];
  
  if (messageType == "stream")
  {
    streamingEnabled = doc["value"];
    Serial.printf("Streaming %s\n", streamingEnabled ? "ENABLED" : "DISABLED");
  }
  else if (messageType == "flash")
  {
    flashState = doc["value"];
    digitalWrite(FLASH_PIN, flashState ? HIGH : LOW);
    Serial.printf("Flash %s\n", flashState ? "ON" : "OFF");
  }
  else if (messageType == "speed")
  {
    servoSpeed = doc["value"];
    Serial.printf("Servo speed set to: %d ms\n", servoSpeed);
  }
  else if (messageType == "auto")
  {
    autoMode = doc["value"];
    autoPattern.currentStep = 0;
    autoPattern.lastStepTime = millis();
    Serial.printf("Auto mode: %s\n", autoMode ? "ON" : "OFF");
  }
  
  // Send acknowledgment
  webSocket.sendTXT(num, "{\"status\":\"ok\"}");
}

void handleArmrobotMessage(String message, uint8_t num)
{
  int commaIndex = message.indexOf(',');
  if (commaIndex != -1)
  {
    String key = message.substring(0, commaIndex);
    String value = message.substring(commaIndex + 1);
    int valueInt = value.toInt();
    
    Serial.printf("Armrobot Protocol - Key [%s] Value[%s]\n", key.c_str(), value.c_str());
    
    if (key == "Record")
    {
      recordSteps = valueInt;
      if (recordSteps)
      {
        recordedSteps.clear();
        previousTimeInMilli = millis();
      }
      Serial.printf("Recording: %s\n", recordSteps ? "ON" : "OFF");
    }
    else if (key == "Play")
    {
      playRecordedSteps = valueInt;
      Serial.printf("Playing: %s\n", playRecordedSteps ? "ON" : "OFF");
    }
    else if (key == "Base")
    {
      writeServoValues(0, valueInt);
    }
    else if (key == "Shoulder")
    {
      writeServoValues(1, valueInt);
    }
    else if (key == "Elbow")
    {
      writeServoValues(2, valueInt);
    }
    else if (key == "Gripper")
    {
      writeServoValues(3, valueInt);
    }
  }
}

// ========== MAIN LOOP ==========

void loop()
{
  server.handleClient();
  webSocket.loop();

  // Auto mode handling
  if (autoMode)
  {
    unsigned long currentTime = millis();
    if (currentTime - autoPattern.lastStepTime >= 3000) // 3-second interval
    {
      moveToPosition(autoPattern.positions[autoPattern.currentStep]);
      
      // Broadcast position to all clients
      for (int i = 0; i < servoPins.size(); i++) {
        String message = servoPins[i].servoName + "," + String(
          (i == 0) ? currentPosition.base :
          (i == 1) ? currentPosition.shoulder :
          (i == 2) ? currentPosition.elbow :
          currentPosition.gripper
        );
        webSocket.broadcastTXT(message);
      }
      
      autoPattern.currentStep = (autoPattern.currentStep + 1) % autoPattern.stepCount;
      autoPattern.lastStepTime = currentTime;
      
      Serial.printf("Auto mode: Step %d completed\n", autoPattern.currentStep);
    }
  }

  // Play recorded steps
  if (playRecordedSteps)
  { 
    playRecordedRobotArmSteps();
  }

  // Small delay for system stability
  delay(1);
}