#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

// Tao struct ServoPins gom cac thong so cua Servo nhu: chan servo, ten servo, goc khoi dau cua servo
struct ServoPins
{
  Servo servo;
  int servoPin;
  String servoName;
  int initialPosition;
};

// Su dung struct vua tao ben tren de khai bao 4 servo
std::vector<ServoPins> servoPins =
    {
        {Servo(), 13, "Base", 90},
        {Servo(), 12, "Shoulder", 90},
        {Servo(), 14, "Elbow", 90},
        {Servo(), 27, "Gripper", 85}, // Tối ưu: 85° = lực nắm mạnh hơn (71-105°)
};

// Struct de luu tru cac buoc khi Recording (chi so servo, cac gia tri trong do)
struct RecordedStep
{
  int servoIndex;
  int value;
  int delayInStep;
};

std::vector<RecordedStep> recordedSteps;

// Khai bao cac bien check qua trinh record dang dien ra hay dang play
bool recordSteps = false;
bool playRecordedSteps = false;

unsigned long previousTimeInMilli = millis();

// Định nghĩa chân GPIO cho module L298N điều khiển xe 3 bánh
// L298N có các chân: IN1, IN2, IN3, IN4, ENA, ENB
// Sử dụng các chân GPIO đã kiểm chứng hoạt động tốt trên ESP32
// Định nghĩa chân GPIO cho module L298N điều khiển xe 3 bánh
// Chỉ sử dụng IN1, IN2, IN3, IN4 - KHÔNG dùng ENA, ENB để tránh chip nóng
#define IN1 22 // Chân IN1 của L298N (Motor A - Bánh trái)
#define IN2 23 // Chân IN2 của L298N (Motor A - Bánh trái)
#define IN3 5  // Chân IN3 của L298N (Motor B - Bánh phải)
#define IN4 18 // Chân IN4 của L298N (Motor B - Bánh phải)
// #define ENA 19 // ĐÃ TẮT - Nối ENA trực tiếp với VCC trên L298N
// #define ENB 4  // ĐÃ TẮT - Nối ENB trực tiếp với VCC trên L298N

/*
Sơ đồ kết nối L298N (CẬP NHẬT - KHÔNG dùng PWM):
ESP32     ->   L298N
GPIO 22   ->   IN1
GPIO 23   ->   IN2
GPIO 5    ->   IN3
GPIO 18   ->   IN4
VCC (5V)  ->   ENA (nối trực tiếp, không qua ESP32)
VCC (5V)  ->   ENB (nối trực tiếp, không qua ESP32)

L298N     ->   Motor
OUT1      ->   Motor Trái (+)
OUT2      ->   Motor Trái (-)
OUT3      ->   Motor Phải (+)
OUT4      ->   Motor Phải (-)

ƯU ĐIỂM: Tránh ESP32 nóng, không cần PWM, đơn giản hơn
NHƯỢC ĐIỂM: Motor chạy full speed, không điều chỉnh được tốc độ

Lý do thay đổi:
- GPIO 25, 26: Có thể không hoạt động trên một số board ESP32 cụ thể
- GPIO 16, 17: Là các GPIO OUTPUT ổn định, ít xung đột
- GPIO 2: Có thể xung đột với LED builtin hoặc boot sequence
- GPIO 19: GPIO OUTPUT an toàn, hỗ trợ PWM tốt

Các GPIO an toàn cho OUTPUT:
- GPIO 16, 17, 19, 21, 22, 23: Ổn định cho digital output
- GPIO 5, 18: Đã test hoạt động tốt
*/

// Biến điều khiển xe
int currentSpeed = 150; // Tốc độ hiện tại (0-255)
bool carMoving = false; // Trạng thái xe đang di chuyển

// Biến cân bằng tốc độ cho từng motor (điều chỉnh nếu một motor chạy chậm hơn)
float motorA_SpeedFactor = 1.0; // Hệ số tốc độ Motor A (OUT1,2) - 1.0 = 100% FULL SPEED
float motorB_SpeedFactor = 0.9; // Hệ số tốc độ Motor B (OUT3,4) - 0.9 = 90% SPEED
// Ví dụ: Nếu Motor A chậm, set motorA_SpeedFactor = 1.1 (110%)
//        Nếu Motor B chậm, set motorB_SpeedFactor = 1.1 (110%)

// PWM mềm cho Motor B (OUT3,4) để chạy chậm 90%
unsigned long lastPwmCycle = 0;
const unsigned long pwmPeriod = 20;                           // Chu kỳ PWM 20ms
bool motorB_PwmState = false;                                 // Trạng thái PWM của motor B
bool enablePWM = false;                                       // Biến kiểm soát khi nào cho phép PWM hoạt động
unsigned long motorB_OnTime = pwmPeriod * motorB_SpeedFactor; // Thời gian bật = 20ms * 0.9 = 18ms
unsigned long motorB_OffTime = pwmPeriod - motorB_OnTime;     // Thời gian tắt = 2ms

// Biến lưu trạng thái IN3, IN4 khi PWM
int motorB_IN3_State = LOW;
int motorB_IN4_State = LOW;

// Dat ten wifi va mat khau
const char *ssid = "Huynh Thi Thu";
const char *password = "huynhthithu789";

// Tao may chu web tai server 80
WebServer server(80);

// Su dung Websocket de kiem soat input cua Robot
WebSocketsServer webSocket = WebSocketsServer(81);

// Tao trang HTML cho ung dung dieu khien canh tay robot va luu tru trong trang HTML
// Neu co bat ki thanh truot (slider) duoc di chuyen tren dien thoai, no se gui vi tri thanh truot den ESP32, su dung Websocket
// Neu co button nao duoc bam, value se la 0 hoac 1
const char *htmlHomePage PROGMEM = R"HTMLHOMEPAGE(
<!DOCTYPE html>
<html>  <head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <title>Robot Arm Control - Team NTD</title>
    <style>

    input[type=button]
    {
      background-color:red;color:white;border-radius:30px;width:100%;height:40px;font-size:20px;text-align:center;
    }
        
    .noselect {
      -webkit-touch-callout: none; /* iOS Safari */
        -webkit-user-select: none; /* Safari */
         -khtml-user-select: none; /* Konqueror HTML */
           -moz-user-select: none; /* Firefox */
            -ms-user-select: none; /* Internet Explorer/Edge */
                user-select: none; /* Non-prefixed version, currently
                                      supported by Chrome and Opera */
    }

    .slidecontainer {
      width: 100%;
    }

    .slider {
      -webkit-appearance: none;
      width: 100%;
      height: 20px;
      border-radius: 5px;
      background: #d3d3d3;
      outline: none;
      opacity: 0.7;
      -webkit-transition: .2s;
      transition: opacity .2s;
    }

    .slider:hover {
      opacity: 1;
    }
  
    .slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 40px;
      height: 40px;
      border-radius: 50%;
      background: red;
      cursor: pointer;
    }    
    .slider::-moz-range-thumb {
      width: 40px;
      height: 40px;
      border-radius: 50%;
      background: red;
      cursor: pointer;
    }
    
    #ws-status {
      margin: 10px auto;
      width: 250px;
      font-weight: bold;
      font-size: 14px;
      color: white;
      border-radius: 8px;
      background: #888;
      padding: 8px 12px;
      transition: background 0.3s;
      text-align: center;
      box-shadow: 0 2px 4px rgba(0,0,0,0.2);
    }
    .ws-connected { background: #27ae60 !important; }
    .ws-connecting { background: #f39c12 !important; }
    .ws-disconnected { background: #e74c3c !important; }
    .ws-error { background: #c0392b !important; }

    </style>
  
  </head>  <body class="noselect" align="center" style="background-color:white">
     
    <div id="ws-status" class="ws-connecting">WebSocket: Đang kết nối...</div>
    <h1 style="color: teal;text-align:center;">Robot Arm Control</h1>
    <h2 style="color: teal;text-align:center;">Team NTD</h2>
    
    <table id="mainTable" style="width:400px;margin:auto;table-layout:fixed" CELLSPACING=10>
      <tr/><tr/>
      <tr/><tr/>      <tr>
        <td style="text-align:left;font-size:25px"><b>Gripper:</b></td>
        <td colspan=2>         <div class="slidecontainer">
            <input type="range" min="71" max="105" value="85" class="slider" id="Gripper" oninput='sendButtonInput("Gripper",value)'>
          </div>
        </td>
      </tr>
      <tr>
        <td style="text-align:left;font-size:20px"><b>Quick:</b></td>
        <td><input type="button" value="OPEN" onclick='sendButtonInput("QuickOpen","1")' style="background-color:orange;width:100%;height:35px;font-size:16px;border-radius:15px;"></td>
        <td><input type="button" value="CLOSE" onclick='sendButtonInput("QuickClose","1")' style="background-color:orange;width:100%;height:35px;font-size:16px;border-radius:15px;"></td>
      </tr>
      <tr/><tr/>
      <tr>
        <td style="text-align:left;font-size:25px"><b>Elbow:</b></td>
        <td colspan=2>
         <div class="slidecontainer">
            <input type="range" min="0" max="180" value="90" class="slider" id="Elbow" oninput='sendButtonInput("Elbow",value)'>
          </div>
        </td>
      </tr> 
      <tr/><tr/>      
      <tr>        <td style="text-align:left;font-size:25px"><b>Shoulder:</b></td>
        <td colspan=2>
         <div class="slidecontainer">
            <input type="range" min="38" max="156" value="90" class="slider" id="Shoulder" oninput='sendButtonInput("Shoulder",value)'>
          </div>
        </td>
      </tr>  
      <tr/><tr/>      
      <tr>
        <td style="text-align:left;font-size:25px"><b>Base:</b></td>
        <td colspan=2>
         <div class="slidecontainer">
            <input type="range" min="0" max="180" value="90" class="slider" id="Base" oninput='sendButtonInput("Base",value)'>
          </div>
        </td>
      </tr> 
      <tr/><tr/> 
      <tr>
        <td style="text-align:left;font-size:25px"><b>Record:</b></td>
        <td><input type="button" id="Record" value="OFF" ontouchend='onclickButton(this)'></td>
        <td></td>
      </tr>
      <tr/><tr/> 
      <tr>
        <td style="text-align:left;font-size:25px"><b>Play:</b></td>
        <td><input type="button" id="Play" value="OFF" ontouchend='onclickButton(this)'></td>
        <td></td>      </tr>      
    </table>
    
    <!-- Bảng điều khiển xe 3 bánh -->
    <h2 style="color: #2c3e50;text-align:center;margin-top:30px;">Điều khiển xe 3 bánh</h2>
    <table id="carTable" style="width:350px;margin:auto;table-layout:fixed;margin-top:20px;" CELLSPACING=10>
      <tr>
        <td></td>
        <td style="text-align:center;">
          <input type="button" value="TIẾN" onmousedown='sendCarCommand("FORWARD")' onmouseup='sendCarCommand("STOP")' ontouchstart='sendCarCommand("FORWARD")' ontouchend='sendCarCommand("STOP")' style="background-color:#27ae60;width:100%;height:50px;font-size:18px;font-weight:bold;border-radius:10px;color:white;">
        </td>
        <td></td>
      </tr>
      <tr>
        <td style="text-align:center;">
          <input type="button" value="TRÁI" onmousedown='sendCarCommand("LEFT")' onmouseup='sendCarCommand("STOP")' ontouchstart='sendCarCommand("LEFT")' ontouchend='sendCarCommand("STOP")' style="background-color:#3498db;width:100%;height:50px;font-size:18px;font-weight:bold;border-radius:10px;color:white;">
        </td>
        <td style="text-align:center;">
          <input type="button" value="DỪNG" onclick='sendCarCommand("STOP")' style="background-color:#e74c3c;width:100%;height:50px;font-size:18px;font-weight:bold;border-radius:10px;color:white;">
        </td>
        <td style="text-align:center;">
          <input type="button" value="PHẢI" onmousedown='sendCarCommand("RIGHT")' onmouseup='sendCarCommand("STOP")' ontouchstart='sendCarCommand("RIGHT")' ontouchend='sendCarCommand("STOP")' style="background-color:#3498db;width:100%;height:50px;font-size:18px;font-weight:bold;border-radius:10px;color:white;">
        </td>
      </tr>
      <tr>
        <td></td>
        <td style="text-align:center;">
          <input type="button" value="LÙI" onmousedown='sendCarCommand("BACKWARD")' onmouseup='sendCarCommand("STOP")' ontouchstart='sendCarCommand("BACKWARD")' ontouchend='sendCarCommand("STOP")' style="background-color:#e67e22;width:100%;height:50px;font-size:18px;font-weight:bold;border-radius:10px;color:white;">
        </td>
        <td></td>
      </tr>
      <tr>
        <td colspan="3" style="text-align:center;padding-top:15px;">
          <label style="font-size:16px;font-weight:bold;color:#2c3e50;">Tốc độ: </label>
          <input type="range" min="0" max="255" value="150" id="speed" style="width:200px;" oninput='updateSpeed(this.value)'>
          <span id="speedValue" style="font-size:16px;font-weight:bold;color:#e74c3c;">150</span>
        </td>
      </tr>
    </table><script>
      var webSocketRobotArmInputUrl = "ws:\/\/" + window.location.hostname + ":81";      
      var websocketRobotArmInput;
      
      function setWebSocketStatus(statusText, className) {
        var statusElement = document.getElementById('ws-status');
        if (statusElement) {
          statusElement.textContent = statusText;
          statusElement.className = className;
        }
      }
        function initRobotArmInputWebSocket() 
      {
        setWebSocketStatus('WebSocket: Đang kết nối...', 'ws-connecting');
        
        websocketRobotArmInput = new WebSocket(webSocketRobotArmInputUrl);
          websocketRobotArmInput.onopen = function(event){
          setWebSocketStatus('WebSocket: Đã kết nối', 'ws-connected');
          console.log('WebSocket connected successfully');
        };
        
        websocketRobotArmInput.onclose = function(event){
          setWebSocketStatus('WebSocket: Mất kết nối - Đang thử lại...', 'ws-disconnected');
          console.log('WebSocket disconnected, attempting to reconnect...');
          setTimeout(initRobotArmInputWebSocket, 2000);
        };
        
        websocketRobotArmInput.onerror = function(event){
          setWebSocketStatus('WebSocket: Lỗi kết nối', 'ws-error');
          console.log('WebSocket error:', event);
        };
        
        websocketRobotArmInput.onmessage = function(event)
        {
          var keyValue = event.data.split(",");
          var button = document.getElementById(keyValue[0]);
          if (button) {
            button.value = keyValue[1];
            if (button.id == "Record" || button.id == "Play")
            {
              button.style.backgroundColor = (button.value == "ON" ? "green" : "red");  
              enableDisableButtonsSliders(button);
            }
          }
        };      }
      
      // Hàm gửi lệnh điều khiển xe
      function sendCarCommand(command) {
        var speed = document.getElementById('speed').value;
        var data = "CAR," + command + "," + speed;
        console.log('Sending car command:', data);
        websocketRobotArmInput.send(data);
      }
      
      // Hàm cập nhật tốc độ
      function updateSpeed(value) {
        document.getElementById('speedValue').textContent = value;
        // Gửi tốc độ mới nếu xe đang di chuyển
        console.log('Speed updated to:', value);
      }
      
      function sendButtonInput(key, value) 
      {
        var data = key + "," + value;
        websocketRobotArmInput.send(data);
      }

      
      function onclickButton(button) 
      {
        button.value = (button.value == "ON") ? "OFF" : "ON" ;        
        button.style.backgroundColor = (button.value == "ON" ? "green" : "red");          
        var value = (button.value == "ON") ? 1 : 0 ;
        sendButtonInput(button.id, value);
        enableDisableButtonsSliders(button);
      }
      
      function enableDisableButtonsSliders(button)
      {
        if(button.id == "Play")
        {
          var disabled = "auto";
          if (button.value == "ON")
          {
            disabled = "none";            
          }
          document.getElementById("Gripper").style.pointerEvents = disabled;
          document.getElementById("Elbow").style.pointerEvents = disabled;          
          document.getElementById("Shoulder").style.pointerEvents = disabled;          
          document.getElementById("Base").style.pointerEvents = disabled; 
          document.getElementById("Record").style.pointerEvents = disabled;
        }
        if(button.id == "Record")
        {
          var disabled = "auto";
          if (button.value == "ON")
          {
            disabled = "none";            
          }
          document.getElementById("Play").style.pointerEvents = disabled;
        }        
      }
           
      window.onload = initRobotArmInputWebSocket;
      document.getElementById("mainTable").addEventListener("touchend", function(event){
        event.preventDefault()
      });      
    </script>
  </body>    
</html>
)HTMLHOMEPAGE";

// Function prototypes (KHAI BÁO HÀM TRƯỚC KHI SỬ DỤNG)
void sendCurrentRobotArmState(uint8_t num);
void writeServoValues(int servoIndex, int value);
void quickGripperAction(String action);
void ultraFastGripperMove(int targetAngle);
int constrainGripperAngle(int angle);
int mapShoulderAngle(int uiAngle);
void optimizedGripperControl(int targetAngle);
void adaptiveGripperControl(int targetAngle, String speedMode = "FAST");
void playRecordedRobotArmSteps();
void setupMotors();
void controlCar(String command, int speed);
void stopCar();
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void updateSoftPWM(); // Hàm cập nhật PWM mềm cho Motor B

// Ham handleRoot se gui trang HTML khi chung ta nhap dia chi IP tren trinh duyet Web
void handleRoot()
{
  server.send(200, "text/html; charset=utf-8", htmlHomePage);
}

void handleNotFound()
{
  server.send(404, "text/plain", "File Not found");
}

// Websocket event se duoc goi khi ESP32 nhan duoc bat ki lenh nao tu dien thoai
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.printf("🔌 [WEBSOCKET] Client #%u disconnected\n", num);
    break;
  case WStype_CONNECTED:
    Serial.printf("🔌 [WEBSOCKET] Client #%u connected from %s\n", num, webSocket.remoteIP(num).toString().c_str());
    sendCurrentRobotArmState(num);
    break;
  case WStype_TEXT:
    Serial.printf("📨 [WEBSOCKET] Received from client #%u: %s (length: %d)\n",
                  num, String((char *)payload).c_str(), length);
    if (length > 0)
    {
      String message = String((char *)payload);
      Serial.printf("📝 [WEBSOCKET] Parsed message: '%s'\n", message.c_str());

      int commaIndex = message.indexOf(',');
      if (commaIndex != -1)
      {
        String key = message.substring(0, commaIndex);
        String value = message.substring(commaIndex + 1);
        int valueInt = value.toInt();

        Serial.printf("🔑 [WEBSOCKET] Key: '%s', Value: '%s' (int: %d)\n", key.c_str(), value.c_str(), valueInt);

        if (key == "Record")
        {
          recordSteps = valueInt;
          if (recordSteps)
          {
            recordedSteps.clear();
            previousTimeInMilli = millis();
          }
        }
        else if (key == "Play")
        {
          playRecordedSteps = valueInt;
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
        else if (key == "QuickOpen")
        {
          quickGripperAction("QUICK_OPEN");
          // Cập nhật slider về vị trí mở
          webSocket.broadcastTXT("Gripper,71");
        }
        else if (key == "QuickClose")
        {
          quickGripperAction("QUICK_CLOSE");
          // Cập nhật slider về vị trí đóng
          webSocket.broadcastTXT("Gripper,105");
        }
        else if (key == "CAR")
        {
          // Xử lý lệnh điều khiển xe: CAR,COMMAND,SPEED
          Serial.printf("🚗 [DEBUG] Processing CAR command: %s\n", message.c_str());

          // value chứa "COMMAND,SPEED", cần tách ra
          int commaInValue = value.indexOf(',');
          String carCommand;
          int speed = 150; // Tốc độ mặc định

          if (commaInValue != -1)
          {
            carCommand = value.substring(0, commaInValue);
            String speedStr = value.substring(commaInValue + 1);
            speed = speedStr.toInt();
            Serial.printf("🚗 [DEBUG] Parsed - Command: '%s', Speed: %d\n", carCommand.c_str(), speed);
          }
          else
          {
            carCommand = value; // Nếu không có speed, chỉ có command
            Serial.printf("🚗 [DEBUG] No speed in command, using default: %d\n", speed);
          }

          Serial.printf("🚗 [DEBUG] Calling controlCar('%s', %d)\n", carCommand.c_str(), speed);
          controlCar(carCommand, speed);
          Serial.printf("✅ [DEBUG] Car control completed: %s at speed %d\n", carCommand.c_str(), speed);
        }
        else if (key == "TEST")
        {
          // ⚠️ CẢNH BÁO: Các lệnh test có thể làm motor chạy!
          // Chỉ sử dụng khi cần debug, không phải tự động
          Serial.printf("⚠️ TEST command received: %s\n", value.c_str());

          // Có thể comment các dòng dưới để tắt hoàn toàn test
          if (value == "PINS")
          {
            testAllPins();
          }
          else if (value == "MOTOR_A")
          {
            testMotorA(150);
          }
          else if (value == "MOTOR_B")
          {
            testMotorB(150);
          }
        }
      }
      else
      {
        Serial.printf("❌ [WEBSOCKET] Invalid message format (no comma): '%s'\n", message.c_str());
      }
    }
    else
    {
      Serial.println("❌ [WEBSOCKET] Empty message received");
    }
    break;
  default:
    Serial.printf("🔍 [WEBSOCKET] Unknown event type: %d from client #%u\n", type, num);
    break;
  }
}

// Ham gui cac gia tri hien tai cho servo va cac nut cho dien thoai
void sendCurrentRobotArmState(uint8_t num)
{
  for (int i = 0; i < servoPins.size(); i++)
  {
    int currentValue = servoPins[i].servo.read(); // Nếu là servo Shoulder, chuyển đổi từ servo value về UI value theo mapping mới
    if (i == 1)
    { // Shoulder servo - mapping ngược: Servo 142° -> UI 38°, Servo 24° -> UI 156°
      int servoValue = servoPins[i].servo.read();
      currentValue = map(servoValue, 142, 24, 38, 156);
      currentValue = constrain(currentValue, 38, 156); // Giới hạn trong vùng UI hợp lệ
      Serial.printf("Send state - Shoulder: Servo %d° -> UI %d°\n", servoValue, currentValue);
    }

    String message = servoPins[i].servoName + "," + String(currentValue);
    webSocket.sendTXT(num, message);
  }
  webSocket.sendTXT(num, String("Record,") + (recordSteps ? "ON" : "OFF"));
  webSocket.sendTXT(num, String("Play,") + (playRecordedSteps ? "ON" : "OFF"));
}

// Ham mapping cho servo Shoulder: UI 38° -> Servo 142°, UI 156° -> Servo 24°
int mapShoulderAngle(int uiAngle)
{
  // Chuyển đổi từ UI (38-156°) sang servo thực tế (142-24°) theo yêu cầu mới
  // Giới hạn vùng di chuyển an toàn cho shoulder
  int constrainedUI = constrain(uiAngle, 38, 156);
  int servoAngle = map(constrainedUI, 38, 156, 142, 24);

  Serial.printf("Shoulder mapping: UI %d° (constrained: %d°) -> Servo %d°\n", uiAngle, constrainedUI, servoAngle);
  return servoAngle;
}

// Ham gioi han goc cho Gripper trong khoang 71-105 do (an toan va luc nam manh)
int constrainGripperAngle(int angle)
{
  if (angle < 71)
  {
    Serial.printf("Gripper angle %d° too low, adjusted to 71° (safe minimum)\n", angle);
    return 71; // An toàn hơn, tránh servo bị kẹt
  }
  if (angle > 105)
  {
    Serial.printf("Gripper angle %d° too high, adjusted to 105° (safe maximum)\n", angle);
    return 105; // Tăng lên 105° để tăng lực đóng
  }
  return angle;
}

// ========== TỐI ƯU SERVO GRIPPER - PHIÊN BẢN NHANH ==========

// Hàm điều khiển gripper với tốc độ cao và độ chính xác
void optimizedGripperControl(int targetAngle)
{
  int currentAngle = servoPins[3].servo.read();
  int safeAngle = constrainGripperAngle(targetAngle);

  Serial.printf("🚀 Fast Gripper Control: %d° → %d° (constrained: %d°)\n",
                currentAngle, targetAngle, safeAngle);

  int distance = abs(safeAngle - currentAngle);

  // Nếu khoảng cách nhỏ (≤3°), di chuyển trực tiếp không delay
  if (distance <= 3)
  {
    servoPins[3].servo.write(safeAngle);
    delay(20); // Delay tối thiểu để servo ổn định
    Serial.printf("⚡ Direct positioning: %d°\n", safeAngle);
    return;
  }

  // Cho khoảng cách lớn hơn, sử dụng acceleration curve
  int step = (safeAngle > currentAngle) ? 1 : -1;
  int moveDelay;

  while (currentAngle != safeAngle)
  {
    currentAngle += step;
    servoPins[3].servo.write(currentAngle);

    // Acceleration curve: nhanh ở giữa, chậm ở đầu/cuối
    int remaining = abs(safeAngle - currentAngle);
    if (remaining <= 2)
    {
      moveDelay = 8; // Chậm lại ở cuối để chính xác
    }
    else if (remaining >= distance - 2)
    {
      moveDelay = 8; // Chậm ở đầu để khởi động mượt
    }
    else
    {
      moveDelay = 3; // Tốc độ cao ở giữa
    }

    delay(moveDelay);
  }

  // Giữ vị trí cuối ngắn hơn
  delay(25);
  servoPins[3].servo.write(safeAngle);

  Serial.printf("✅ Fast gripper positioned at %d° (improved speed)\n", safeAngle);
}

// Hàm di chuyển gripper siêu nhanh cho các thao tác cơ bản
void ultraFastGripperMove(int targetAngle)
{
  int safeAngle = constrainGripperAngle(targetAngle);

  // Di chuyển trực tiếp không có delay trung gian
  servoPins[3].servo.write(safeAngle);

  // Chỉ delay tối thiểu để servo nhận lệnh
  delay(15);

  Serial.printf("⚡⚡ Ultra fast gripper move to %d°\n", safeAngle);
}

// Hàm nhanh để mở/đóng gripper với nhiều chế độ tốc độ
void quickGripperAction(String action)
{
  if (action == "OPEN")
  {
    ultraFastGripperMove(71); // Mở hoàn toàn siêu nhanh
    Serial.println("🔓 Gripper OPENED ultra fast (71°)");
  }
  else if (action == "CLOSE")
  {
    optimizedGripperControl(105); // Đóng với lực tối đa 105°
    Serial.println("🔒 Gripper CLOSED fast with maximum force (105°)");
  }
  else if (action == "GRIP")
  {
    optimizedGripperControl(85); // Nắm vừa phải
    Serial.println("🤏 Gripper GRIPPING fast (85°)");
  }
  else if (action == "QUICK_OPEN")
  {
    ultraFastGripperMove(71); // Mở siêu nhanh
    Serial.println("⚡ Gripper QUICK OPEN (71°)");
  }
  else if (action == "QUICK_CLOSE")
  {
    ultraFastGripperMove(105); // Đóng siêu nhanh 105°
    Serial.println("⚡ Gripper QUICK CLOSE (105°)");
  }
  else if (action == "INSTANT_GRIP")
  {
    ultraFastGripperMove(85); // Nắm siêu nhanh
    Serial.println("⚡ Gripper INSTANT GRIP (85°)");
  }
}

// Hàm đặc biệt cho việc di chuyển gripper với tốc độ thích ứng
void adaptiveGripperControl(int targetAngle, String speedMode)
{
  int currentAngle = servoPins[3].servo.read();
  int safeAngle = constrainGripperAngle(targetAngle);
  int distance = abs(safeAngle - currentAngle);

  Serial.printf("🎯 Adaptive Gripper (%s): %d° → %d°\n",
                speedMode.c_str(), currentAngle, safeAngle);

  if (speedMode == "ULTRA")
  {
    ultraFastGripperMove(safeAngle);
  }
  else if (speedMode == "INSTANT" || distance <= 5)
  {
    // Cho khoảng cách ngắn, di chuyển trực tiếp
    servoPins[3].servo.write(safeAngle);
    delay(12);
  }
  else
  {
    // Sử dụng mode nhanh tiêu chuẩn
    optimizedGripperControl(safeAngle);
  }
}

// ========== KẾT THÚC TỐI ƯU GRIPPER NHANH ==========/

// ham ghi gia tri cung nhu record lai cac buoc va cung ghi no vao servo
void writeServoValues(int servoIndex, int value)
{
  if (recordSteps)
  {
    RecordedStep recordedStep;
    if (recordedSteps.size() == 0) // We will first record initial position of all servos.
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
  } // Áp dụng mapping cho servo Shoulder (index 1)
  if (servoIndex == 1)
  { // Shoulder servo
    int servoValue = mapShoulderAngle(value);
    servoPins[servoIndex].servo.write(servoValue);
    Serial.printf("writeServoValues: Servo %s (index %d) - UI: %d° -> Servo: %d°\n",
                  servoPins[servoIndex].servoName.c_str(), servoIndex, value, servoValue);
  } // Tối ưu đặc biệt cho Gripper (index 3) - sử dụng hàm điều khiển nhanh
  else if (servoIndex == 3)
  { // Gripper servo - sử dụng hàm tối ưu tốc độ cao
    // Sử dụng adaptive control để tự động chọn tốc độ phù hợp
    adaptiveGripperControl(value, "FAST");
    Serial.printf("writeServoValues: Gripper fast control - UI: %d°\n", value);
  }
  // Các servo khác (Base, Elbow)
  else
  {
    servoPins[servoIndex].servo.write(value);
    Serial.printf("writeServoValues: Servo %s (index %d) - %d°\n",
                  servoPins[servoIndex].servoName.c_str(), servoIndex, value);
  }
}

// Phat lai tung buoc da duoc record va di chuyen servo tuong ung
void playRecordedRobotArmSteps()
{
  if (recordedSteps.size() == 0)
  {
    return;
  } // This is to move servo to initial position slowly. First 4 steps are initial position
  for (int i = 0; i < 4 && playRecordedSteps; i++)
  {
    RecordedStep &recordedStep = recordedSteps[i];
    int currentServoPosition = servoPins[recordedStep.servoIndex].servo.read(); // Áp dụng mapping cho target position nếu là servo Shoulder và giới hạn cho Gripper
    int targetPosition = recordedStep.value;
    if (recordedStep.servoIndex == 1)
    { // Shoulder servo
      targetPosition = mapShoulderAngle(recordedStep.value);
    }
    else if (recordedStep.servoIndex == 3)
    { // Gripper servo
      targetPosition = constrainGripperAngle(recordedStep.value);
    }
    while (currentServoPosition != targetPosition && playRecordedSteps)
    {
      currentServoPosition = (currentServoPosition > targetPosition ? currentServoPosition - 1 : currentServoPosition + 1);

      // Sử dụng điều khiển nhanh cho gripper trong playback
      if (recordedStep.servoIndex == 3)
      {
        // Cho gripper, sử dụng ultra fast move
        ultraFastGripperMove(currentServoPosition);
      }
      else
      {
        servoPins[recordedStep.servoIndex].servo.write(currentServoPosition);
      }

      String message = servoPins[recordedStep.servoIndex].servoName + "," + String(recordedStep.value); // Gửi UI value, không phải servo value
      webSocket.broadcastTXT(message);

      // Delay ngắn hơn cho gripper để tăng tốc độ playback
      if (recordedStep.servoIndex == 3)
      {
        delay(20); // Gripper nhanh hơn
      }
      else
      {
        delay(50); // Các servo khác giữ delay tiêu chuẩn
      }
    }
  }
  delay(2000); // Delay before starting the actual steps.
  for (int i = 4; i < recordedSteps.size() && playRecordedSteps; i++)
  {
    RecordedStep &recordedStep = recordedSteps[i];
    delay(recordedStep.delayInStep);

    // Áp dụng mapping cho servo value nếu là servo Shoulder
    int servoValue = recordedStep.value;
    if (recordedStep.servoIndex == 1)
    { // Shoulder servo
      servoValue = mapShoulderAngle(recordedStep.value);
    } // Áp dụng giới hạn góc cho Gripper và sử dụng điều khiển nhanh
    if (recordedStep.servoIndex == 3)
    { // Gripper servo - sử dụng ultra fast control trong playback
      servoValue = constrainGripperAngle(recordedStep.value);
      ultraFastGripperMove(servoValue);
    }
    else
    {
      servoPins[recordedStep.servoIndex].servo.write(servoValue);
    }
    String message = servoPins[recordedStep.servoIndex].servoName + "," + String(recordedStep.value); // Gửi UI value
    webSocket.broadcastTXT(message);
    // Trong khi di chuyen servo, no cung gui vi tri den dien thoai. Cac thanh truot cung se di chuyen tuong ung
  }
}

// ========== ĐIỀU KHIỂN XE 3 BÁNH ==========

// Hàm khởi tạo các chân L298N
void setupMotors()
{
  // Cấu hình các chân INPUT cho L298N
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // KHÔNG sử dụng ENA, ENB để tránh chip nóng và hư ESP32
  // pinMode(ENA, OUTPUT);  // ĐÃ TẮT
  // pinMode(ENB, OUTPUT);  // ĐÃ TẮT

  // Nối ENA và ENB trực tiếp với VCC (5V) trên L298N để motor luôn bật

  stopCar(); // Đảm bảo xe dừng khi khởi động

  // Debug: In thông tin chân GPIO
  Serial.println("🚗 L298N Motor Driver initialized - PWM DISABLED for safety");
  Serial.printf("📌 GPIO Pin Configuration:\n");
  Serial.printf("   IN1: GPIO %d\n", IN1);
  Serial.printf("   IN2: GPIO %d\n", IN2);
  Serial.printf("   IN3: GPIO %d\n", IN3);
  Serial.printf("   IN4: GPIO %d\n", IN4);
  Serial.printf("   ENA: Nối trực tiếp VCC (không dùng ESP32)\n");
  Serial.printf("   ENB: Nối trực tiếp VCC (không dùng ESP32)\n");
  Serial.println("✅ Motor pins configured - PWM disabled for chip safety");
}

// Hàm điều khiển xe chính
void controlCar(String command, int speed)
{
  Serial.printf("🚗 [CONTROL] controlCar() called: command='%s', speed=%d\n", command.c_str(), speed);
  Serial.printf("🚗 [CONTROL] GPIO before: IN1=%d, IN2=%d, IN3=%d, IN4=%d\n",
                digitalRead(IN1), digitalRead(IN2), digitalRead(IN3), digitalRead(IN4));

  currentSpeed = constrain(speed, 0, 255);
  if (speed != currentSpeed)
  {
    Serial.printf("🚗 [CONTROL] Speed constrained from %d to %d\n", speed, currentSpeed);
  }

  if (command == "FORWARD")
  {
    Serial.println("🚗 [CONTROL] Executing FORWARD");
    moveForward(currentSpeed);
  }
  else if (command == "BACKWARD")
  {
    Serial.println("🚗 [CONTROL] Executing BACKWARD");
    moveBackward(currentSpeed);
  }
  else if (command == "LEFT")
  {
    Serial.println("🚗 [CONTROL] Executing LEFT");
    turnLeft(currentSpeed);
  }
  else if (command == "RIGHT")
  {
    Serial.println("🚗 [CONTROL] Executing RIGHT");
    turnRight(currentSpeed);
  }
  else if (command == "STOP")
  {
    Serial.println("🚗 [CONTROL] Executing STOP");
    stopCar();
  }
  else
  {
    Serial.printf("❌ [CONTROL] Unknown command: '%s'\n", command.c_str());
  }

  Serial.printf("🚗 [CONTROL] GPIO after: IN1=%d, IN2=%d, IN3=%d, IN4=%d\n",
                digitalRead(IN1), digitalRead(IN2), digitalRead(IN3), digitalRead(IN4));
}

// Dừng xe
void stopCar()
{
  Serial.println("🛑 [STOP] stopCar() called");
  // Tất cả chân IN của L298N = LOW để dừng motor
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  // KHÔNG sử dụng PWM - ENA/ENB nối trực tiếp VCC
  // analogWrite(ENA, 0); // ĐÃ TẮT
  // analogWrite(ENB, 0); // ĐÃ TẮT
  // Reset PWM mềm
  enablePWM = false;
  motorB_PwmState = false;
  motorB_IN3_State = LOW;
  motorB_IN4_State = LOW;

  carMoving = false;
  Serial.printf("🛑 [STOP] Car STOPPED - GPIO: IN1=%d, IN2=%d, IN3=%d, IN4=%d\n",
                digitalRead(IN1), digitalRead(IN2), digitalRead(IN3), digitalRead(IN4));
}

// Di chuyển tiến
void moveForward(int speed)
{
  // Motor A (OUT1,2) - Motor TRÁI chạy FULL SPEED
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Motor B (OUT3,4) - Motor PHẢI sẽ được PWM mềm để chạy 50%
  // Lưu trạng thái mong muốn cho PWM mềm
  motorB_IN3_State = LOW; // Motor phải tiến
  motorB_IN4_State = HIGH;
  // Khởi động PWM mềm cho motor B
  lastPwmCycle = millis();
  motorB_PwmState = true;
  enablePWM = true; // Cho phép PWM hoạt động

  carMoving = true;
  Serial.printf("⬆️ [FORWARD] Motor A (OUT1,2) FULL SPEED, Motor B (OUT3,4) PWM 90%%\n");
  Serial.printf("📍 GPIO Status: IN1=%d, IN2=%d, IN3=%d, IN4=%d\n",
                digitalRead(IN1), digitalRead(IN2), digitalRead(IN3), digitalRead(IN4));
}

// Di chuyển lùi
void moveBackward(int speed)
{
  Serial.printf("⬇️ [BACKWARD] moveBackward() called\n");

  // Motor A (OUT1,2) - Motor TRÁI lùi FULL SPEED
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // Motor B (OUT3,4) - Motor PHẢI lùi sẽ được PWM mềm để chạy 90%
  // Lưu trạng thái mong muốn cho PWM mềm
  motorB_IN3_State = HIGH; // Motor phải lùi  motorB_IN4_State = LOW;

  // Khởi động PWM mềm cho motor B
  lastPwmCycle = millis();
  motorB_PwmState = true;
  enablePWM = true; // Cho phép PWM hoạt động

  carMoving = true;
  Serial.printf("⬇️ [BACKWARD] Motor A (OUT1,2) FULL SPEED, Motor B (OUT3,4) PWM 90%%\n");
  Serial.printf("📍 GPIO Status: IN1=%d, IN2=%d, IN3=%d, IN4=%d\n",
                digitalRead(IN1), digitalRead(IN2), digitalRead(IN3), digitalRead(IN4));
}

// Rẽ trái (motor phải quay, motor trái dừng hoặc chậm)
void turnLeft(int speed)
{
  Serial.printf("⬅️ [LEFT] turnLeft() called\n");

  // Tắt PWM mềm khi rẽ
  enablePWM = false;
  motorB_PwmState = false;

  // L298N: Motor A (trái) dừng - IN1=LOW, IN2=LOW
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  // L298N: Motor B (phải) tiến - IN3=LOW, IN4=HIGH
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  carMoving = true;
  Serial.printf("⬅️ Car TURN LEFT - Motor A STOP, Motor B FULL SPEED\n");
  Serial.printf("📍 GPIO Status: IN1=%d, IN2=%d, IN3=%d, IN4=%d\n",
                digitalRead(IN1), digitalRead(IN2), digitalRead(IN3), digitalRead(IN4));
}

// Rẽ phải (motor trái quay, motor phải dừng hoặc chậm)
void turnRight(int speed)
{
  Serial.printf("➡️ [RIGHT] turnRight() called\n");

  // Tắt PWM mềm khi rẽ
  enablePWM = false;
  motorB_PwmState = false;

  // L298N: Motor A (trái) tiến - IN1=HIGH, IN2=LOW
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // L298N: Motor B (phải) dừng - IN3=LOW, IN4=LOW
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  carMoving = true;
  Serial.printf("➡️ Car TURN RIGHT - Motor A FULL SPEED, Motor B STOP\n");
  Serial.printf("📍 GPIO Status: IN1=%d, IN2=%d, IN3=%d, IN4=%d\n",
                digitalRead(IN1), digitalRead(IN2), digitalRead(IN3), digitalRead(IN4));
}

// ========== KẾT THÚC ĐIỀU KHIỂN XE ==========

// Gan chan servo vao cac doi tuong, dat vi tri ban dau cho servo
void setUpPinModes()
{
  for (int i = 0; i < servoPins.size(); i++)
  {
    servoPins[i].servo.attach(servoPins[i].servoPin); // Áp dụng mapping cho servo Shoulder và giới hạn cho Gripper khi khởi tạo
    int initialValue = servoPins[i].initialPosition;
    if (i == 1)
    { // Shoulder servo
      initialValue = mapShoulderAngle(servoPins[i].initialPosition);
      Serial.printf("Servo %s: UI %d° -> Servo %d° (initial position)\n",
                    servoPins[i].servoName.c_str(),
                    servoPins[i].initialPosition,
                    initialValue);
    }
    else if (i == 3)
    { // Gripper servo
      initialValue = constrainGripperAngle(servoPins[i].initialPosition);
      Serial.printf("Servo %s: UI %d° -> Servo %d° (gripper constrained)\n",
                    servoPins[i].servoName.c_str(),
                    servoPins[i].initialPosition,
                    initialValue);
    }

    servoPins[i].servo.write(initialValue);
  }
}

void setup(void)
{
  setUpPinModes(); // Goi ham dat vi tri ban dau cho servo
  setupMotors();   // Khởi tạo các chân motor cho xe 3 bánh
  Serial.begin(115200);

  // Test GPIO pins đã TẮT để tránh motor chạy tự động
  // delay(1000);
  // testAllPins();

  WiFi.begin(ssid, password); // Ket noi den mang WiFi nha
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, handleRoot); // Thiet lap chuc nang call back tren Root va tren NotFound
  server.onNotFound(handleNotFound);

  webSocket.begin();                 // Bat dau WebSocket
  webSocket.onEvent(webSocketEvent); // Gan websocket event
  server.begin();                    // Bat dau phat Wifi
  Serial.println("HTTP server started");
  Serial.println("WebSocket server started on port 81");

  // Test motors đã được TẮT để tránh chạy tự động
  // Serial.println("🔧 [SETUP] Testing NEW GPIO pins in 5 seconds...");
  // delay(5000);
  // testIN1_IN2_Only();
  // delay(2000);
  // testMotorA_NewGPIO();
  // delay(2000);
  // testMotorB(150);

  Serial.println("✅ [SETUP] System ready - Auto tests DISABLED!");
}

void loop()
{
  static unsigned long lastHeartbeat = 0;
  static unsigned long heartbeatInterval = 30000; // 30 giây

  // Heartbeat log mỗi 30 giây
  unsigned long currentTime = millis();
  if (currentTime - lastHeartbeat > heartbeatInterval)
  {
    Serial.printf("💓 [HEARTBEAT] Uptime: %lu ms, WiFi: %s, Car: %s\n",
                  currentTime,
                  WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected",
                  carMoving ? "Moving" : "Stopped");
    lastHeartbeat = currentTime;
  }
  server.handleClient(); // Xu ly cac client HTTP
  webSocket.loop();      // Xu ly WebSocket

  // Cập nhật PWM mềm cho Motor B (OUT3,4) khi xe đang chạy
  if (carMoving)
  {
    updateSoftPWM();
  }

  // Chỉ phát lại cánh tay robot khi người dùng bấm nút "Play" trên web interface
  // KHÔNG tự động chạy - chỉ chạy khi playRecordedSteps = true từ WebSocket
  if (playRecordedSteps)
  {
    Serial.println("🎬 [LOOP] Playing recorded steps...");
    playRecordedRobotArmSteps();
  }
}

// ========== HÀM DEBUG VÀ TEST MOTOR ==========

// Hàm test motor A (trái)
void testMotorA(int speed)
{
  Serial.printf("🔧 Testing Motor A (OUT1,2) - ENA nối trực tiếp với VCC\n");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // analogWrite(ENA, speed); // ĐÃ BỎ - ENA nối trực tiếp với VCC
  delay(2000); // Chạy 2 giây
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  // analogWrite(ENA, 0); // ĐÃ BỎ - ENA nối trực tiếp với VCC
  Serial.println("🔧 Motor A test completed");
}

// Hàm test motor B (phải)
void testMotorB(int speed)
{
  Serial.printf("🔧 Testing Motor B (OUT3,4) - ENB nối trực tiếp với VCC\n");
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  // analogWrite(ENB, speed); // ĐÃ BỎ - ENB nối trực tiếp với VCC
  delay(2000); // Chạy 2 giây
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  // analogWrite(ENB, 0); // ĐÃ BỎ - ENB nối trực tiếp với VCC
  Serial.println("🔧 Motor B test completed");
}

// Hàm test toàn bộ chân GPIO
void testAllPins()
{
  Serial.println("🔍 Testing all GPIO pins...");

  // Test từng chân IN
  digitalWrite(IN1, HIGH);
  delay(500);
  Serial.printf("IN1 (GPIO %d): %s\n", IN1, digitalRead(IN1) ? "HIGH ✅" : "LOW ❌");
  digitalWrite(IN1, LOW);

  digitalWrite(IN2, HIGH);
  delay(500);
  Serial.printf("IN2 (GPIO %d): %s\n", IN2, digitalRead(IN2) ? "HIGH ✅" : "LOW ❌");
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  delay(500);
  Serial.printf("IN3 (GPIO %d): %s\n", IN3, digitalRead(IN3) ? "HIGH ✅" : "LOW ❌");
  digitalWrite(IN3, LOW);

  digitalWrite(IN4, HIGH);
  delay(500);
  Serial.printf("IN4 (GPIO %d): %s\n", IN4, digitalRead(IN4) ? "HIGH ✅" : "LOW ❌");
  digitalWrite(IN4, LOW);
  // Test PWM pins - ĐÃ BỎ VÌ ENA/ENB NỐI TRỰC TIẾP VỚI VCC
  // analogWrite(ENA, 128); // ĐÃ BỎ - ENA nối trực tiếp với VCC
  // delay(500);
  // Serial.printf("ENA (GPIO %d): PWM 128 ✅\n", ENA);
  // analogWrite(ENA, 0);

  // analogWrite(ENB, 128); // ĐÃ BỎ - ENB nối trực tiếp với VCC
  delay(500); // delay(500);
  // Serial.printf("ENB (GPIO %d): PWM 128 ✅\n", ENB);
  // analogWrite(ENB, 0);

  Serial.println("🔍 GPIO pin test completed (ENA/ENB bypassed)");
}

// ========== KẾT THÚC DEBUG ==========/

// ========== HÀM TEST GPIO RIÊNG LẺ ==========

// Test nhanh chỉ IN1, IN2 với GPIO mới
void testIN1_IN2_Only()
{
  Serial.println("🔧 [TEST] Testing IN1, IN2 with NEW GPIO (16, 17)");

  // Test IN1 (GPIO 16)
  Serial.println("Testing IN1 (GPIO 16):");
  digitalWrite(IN1, HIGH);
  delay(1000);
  Serial.printf("IN1 state: %d\n", digitalRead(IN1));
  digitalWrite(IN1, LOW);
  delay(1000);
  Serial.printf("IN1 state: %d\n", digitalRead(IN1));

  // Test IN2 (GPIO 17)
  Serial.println("Testing IN2 (GPIO 17):");
  digitalWrite(IN2, HIGH);
  delay(1000);
  Serial.printf("IN2 state: %d\n", digitalRead(IN2));
  digitalWrite(IN2, LOW);
  delay(1000);
  Serial.printf("IN2 state: %d\n", digitalRead(IN2));

  Serial.println("🔧 IN1, IN2 GPIO test completed");
}

// Test Motor A với GPIO mới
void testMotorA_NewGPIO()
{
  Serial.println("🔧 [TEST] Testing Motor A with NEW GPIO (22, 23) - ENA/ENB nối VCC");
  // Motor A tiến
  Serial.println("Motor A FORWARD: IN1=HIGH, IN2=LOW (ENA/ENB nối VCC)");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); // Tắt motor B
  digitalWrite(IN4, LOW);
  // analogWrite(ENA, 255); // ĐÃ BỎ - ENA nối trực tiếp với VCC
  // analogWrite(ENB, 0); // ĐÃ BỎ - ENB nối trực tiếp với VCC

  Serial.printf("GPIO Status: IN1=%d, IN2=%d (ENA/ENB bypassed)\n", digitalRead(IN1), digitalRead(IN2));
  delay(3000);

  // Dừng
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  // analogWrite(ENA, 0); // ĐÃ BỎ - ENA nối trực tiếp với VCC
  Serial.println("Motor A stopped");

  delay(1000);
  // Motor A lùi
  Serial.println("Motor A BACKWARD: IN1=LOW, IN2=HIGH (ENA nối VCC)");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  // analogWrite(ENA, 255); // ĐÃ BỎ - ENA nối trực tiếp với VCC
  Serial.printf("GPIO Status: IN1=%d, IN2=%d (ENA bypassed)\n", digitalRead(IN1), digitalRead(IN2));
  delay(3000);

  // Dừng
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  // analogWrite(ENA, 0); // ĐÃ BỎ - ENA nối trực tiếp với VCC
  Serial.println("🔧 Motor A test with new GPIO completed");
  delay(1000);
  Serial.println("===== End testMotorA_NewGPIO =====");
}

// ========== HÀM PWM MỀM CHO MOTOR B ==========
// Hàm cập nhật PWM mềm cho Motor B (OUT3,4) để chạy 90% tốc độ
void updateSoftPWM()
{
  // Chỉ hoạt động khi PWM được phép
  if (!enablePWM)
  {
    return; // Thoát ngay nếu PWM bị tắt
  }

  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastPwmCycle;

  if (motorB_PwmState) // Đang trong chu kỳ BẬT
  {
    if (elapsedTime >= motorB_OnTime) // Hết thời gian bật
    {
      // Tắt motor B
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      motorB_PwmState = false;
      lastPwmCycle = currentTime;
    }
    else // Vẫn trong thời gian bật
    {
      // Giữ motor B chạy theo hướng đã set
      digitalWrite(IN3, motorB_IN3_State);
      digitalWrite(IN4, motorB_IN4_State);
    }
  }
  else // Đang trong chu kỳ TẮT
  {
    if (elapsedTime >= motorB_OffTime) // Hết thời gian tắt
    {
      // Bật lại motor B
      digitalWrite(IN3, motorB_IN3_State);
      digitalWrite(IN4, motorB_IN4_State);
      motorB_PwmState = true;
      lastPwmCycle = currentTime;

      // Cập nhật thời gian ON/OFF cho chu kỳ mới
      motorB_OnTime = pwmPeriod * motorB_SpeedFactor;
      motorB_OffTime = pwmPeriod - motorB_OnTime;
    }
    else // Vẫn trong thời gian tắt
    {
      // Giữ motor B tắt
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }
  }
}