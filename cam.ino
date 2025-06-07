#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27

#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22
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

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Initialize camera với cấu hình tối ưu cho streaming
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
  config.frame_size = FRAMESIZE_QVGA;  // Giảm xuống QVGA (320x240) để tăng FPS
  config.pixel_format = PIXFORMAT_JPEG;
  config.jpeg_quality = 15;  // Giảm chất lượng để tăng tốc độ
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;  // Luôn lấy frame mới nhất

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Cấu hình camera sensor để tối ưu FPS
  sensor_t * s = esp_camera_sensor_get();
  if (s->id.PID == OV2640_PID) {
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

  // Connect to WiFi với cấu hình tối ưu
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // Tăng công suất WiFi

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  // Web Server Routes
  server.on("/", HTTP_GET, []() {
    String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <title>ESP32-CAM Live Stream</title>
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <style>
        body { 
          font-family: Arial; 
          text-align: center; 
          margin: 0;
          padding: 20px;
          background-color: #1a1a1a;
          color: white;
        }
        .container {
          max-width: 800px;
          margin: 0 auto;
        }
        h1 {
          color: #4CAF50;
          margin-bottom: 10px;
        }
        button { 
          padding: 12px 24px; 
          font-size: 16px; 
          margin: 5px;
          border: none;
          border-radius: 5px;
          cursor: pointer;
          transition: all 0.3s;
          font-weight: bold;
        }
        #streamBtn {
          background-color: #4CAF50;
          color: white;
        }
        #streamBtn:hover {
          background-color: #45a049;
          transform: scale(1.05);
        }
        #flashBtn {
          background-color: #FF9800;
          color: white;
        }
        #flashBtn:hover {
          background-color: #F57C00;
          transform: scale(1.05);
        }
        .stream-container { 
          margin: 20px auto; 
          border: 2px solid #4CAF50; 
          border-radius: 10px;
          overflow: hidden;
          box-shadow: 0 4px 20px rgba(76, 175, 80, 0.3);
          background: #000;
          position: relative;
        }
        #stream { 
          width: 100%; 
          height: auto; 
          display: block;
          image-rendering: -webkit-optimize-contrast;
          image-rendering: crisp-edges;
        }
        .status {
          margin: 10px 0;
          padding: 8px 16px;
          border-radius: 20px;
          font-weight: bold;
          display: inline-block;
        }
        .status.connected {
          background-color: #4CAF50;
          color: white;
        }
        .status.disconnected {
          background-color: #f44336;
          color: white;
        }
        .fps-counter {
          position: absolute;
          top: 10px;
          right: 10px;
          background: rgba(0,0,0,0.7);
          color: #4CAF50;
          padding: 5px 10px;
          border-radius: 5px;
          font-family: monospace;
          font-size: 14px;
        }
        .controls {
          margin: 20px 0;
        }
      </style>
    </head>
    <body>
      <div class="container">
        <h1> ESP32-CAM Live Stream</h1>
        
        <div id="status" class="status disconnected">Disconnected</div>
        
        <div class="stream-container">
          <div class="fps-counter" id="fpsCounter">FPS: 0</div>
          <img id="stream" src="" alt="Loading stream...">
        </div>
        
        <div class="controls">
          <button id="streamBtn" onclick="toggleStream()">Stop Stream</button>
          <button id="flashBtn" onclick="toggleFlash()">Flash ON</button>
        </div>
        
        <div style="margin-top: 20px; font-size: 14px; opacity: 0.7;">
          <div>Resolution: 320x240 | Quality: Optimized for Speed</div>
          <div id="streamInfo">Stream: Active</div>
        </div>
      </div>
      
      <script>
        const ws = new WebSocket('ws://' + window.location.hostname + ':81/');
        let streamEnabled = true;
        let flashEnabled = false;
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
        };
        
        ws.onmessage = function(event) {
          console.log('Server:', event.data);
        };
        
        // FPS Counter
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
        
        // Optimized stream update
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
          
          // Preload new image
          const img = new Image();
          img.onload = function() {
            streamImg.src = newSrc;
            streamImg.style.display = 'block';
            frameCount++;
            updateFPS();
            
            // Schedule next update immediately
            setTimeout(updateStream, 33); // ~30 FPS
          };
          
          img.onerror = function() {
            console.log('Stream error, retrying...');
            setTimeout(updateStream, 100); // Retry faster on error
          };
          
          img.src = newSrc;
          document.getElementById('streamInfo').textContent = 'Stream: Active';
        }
        
        // Toggle camera stream
        function toggleStream() {
          streamEnabled = !streamEnabled;
          const btn = document.getElementById('streamBtn');
          
          if (streamEnabled) {
            btn.textContent = 'Stop Stream';
            btn.style.backgroundColor = '#f44336';
            updateStream();
          } else {
            btn.textContent = 'Start Stream';
            btn.style.backgroundColor = '#4CAF50';
            updateStream();
          }
          
          if (ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({type: 'stream', value: streamEnabled}));
          }
        }
        
        // Toggle flash
        function toggleFlash() {
          flashEnabled = !flashEnabled;
          const btn = document.getElementById('flashBtn');
          
          if (flashEnabled) {
            btn.textContent = 'Flash OFF';
            btn.style.backgroundColor = '#FFC107';
          } else {
            btn.textContent = 'Flash ON';
            btn.style.backgroundColor = '#FF9800';
          }
          
          if (ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({type: 'flash', value: flashEnabled}));
          }
        }
        
        // Initialize
        document.addEventListener('DOMContentLoaded', function() {
          updateStream();
        });
        
        // Handle page visibility for performance
        document.addEventListener('visibilitychange', function() {
          if (document.hidden) {
            streamEnabled = false;
          } else if (document.getElementById('streamBtn').textContent.includes('Stop')) {
            streamEnabled = true;
            updateStream();
          }
        });
      </script>
    </body>
    </html>
    )rawliteral";
    server.send(200, "text/html", html);
  });

  // Optimized stream endpoint
  server.on("/stream", HTTP_GET, []() {
    if (!streamingEnabled) {
      server.send(503, "text/plain", "Streaming disabled");
      return;
    }
    
    unsigned long startTime = millis();
    
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      server.send(500, "text/plain", "Camera Error");
      return;
    }
    
    // Optimized headers for fast streaming
    server.sendHeader("Content-Type", "image/jpeg");
    server.sendHeader("Content-Length", String(fb->len));
    server.sendHeader("Cache-Control", "no-cache, no-store, max-age=0");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "Thu, 01 Jan 1970 00:00:00 GMT");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    
    // Send image data directly
    server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
    
    esp_camera_fb_return(fb);
    
    unsigned long processingTime = millis() - startTime;
    Serial.printf("Frame sent: %zu bytes in %lu ms\n", fb->len, processingTime);
  });

  // Performance info endpoint
  server.on("/info", HTTP_GET, []() {
    String info = "ESP32-CAM Status:\n";
    info += "Free Heap: " + String(ESP.getFreeHeap()) + " bytes\n";
    info += "WiFi RSSI: " + String(WiFi.RSSI()) + " dBm\n";
    info += "Streaming: " + String(streamingEnabled ? "ON" : "OFF") + "\n";
    info += "Flash: " + String(flashState ? "ON" : "OFF") + "\n";
    server.send(200, "text/plain", info);
  });

  // Start servers
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Serial.println("🚀 ESP32-CAM Ready!");
  Serial.printf("📱 Web Interface: http://%s\n", WiFi.localIP().toString().c_str());
  Serial.printf("📊 System Info: http://%s/info\n", WiFi.localIP().toString().c_str());
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Client disconnected\n", num);
      break;
      
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] New client: %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      }
      break;
      
    case WStype_TEXT:
      {
        String message = (char*)payload;
        Serial.printf("[%u] Command: %s\n", num, message.c_str());
        
        DynamicJsonDocument doc(256);
        if (deserializeJson(doc, message) == DeserializationError::Ok) {
          
          if (doc["type"] == "stream") {
            streamingEnabled = doc["value"];
            Serial.printf("Streaming %s\n", streamingEnabled ? "ENABLED" : "DISABLED");
          }
          else if (doc["type"] == "flash") {
            flashState = doc["value"];
            digitalWrite(FLASH_PIN, flashState ? HIGH : LOW);
            Serial.printf("Flash %s\n", flashState ? "ON" : "OFF");
          }
          
          // Send acknowledgment
          webSocket.sendTXT(num, "{\"status\":\"ok\"}");
        }
      }
      break;
      
    default:
      break;
  }
}

void loop() {
  server.handleClient();
  webSocket.loop();
  
  // Minimal delay for optimal performance
  delay(1);
}