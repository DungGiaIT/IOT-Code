# CarAramRobot - 3-Wheel Car and Robot Arm Control with ESP32

## 1. Introduction

This project uses an ESP32 to control a 3-wheel car integrated with a robot arm. The system supports remote control via WiFi using a web interface (WebSocket), allowing users to control both the car and the robot arm from a phone or computer.

## 2. Hardware Diagram

- **Microcontroller:** ESP32
- **Motor driver:** L298N
- **DC motors:** 2 motors (left/right)
- **Servos:** 4 servos for the robot arm (Base, Shoulder, Elbow, Gripper)
- **Power:** ESP32 uses a separate power supply; motors and servos use a dedicated 5V supply to avoid voltage drops.

### GPIO Pin Mapping:
| ESP32 | L298N | Function           |
|-------|-------|--------------------|
| 22    | IN1   | Motor A (left)     |
| 23    | IN2   | Motor A (left)     |
| 5     | IN3   | Motor B (right)    |
| 18    | IN4   | Motor B (right)    |
| 13    | Servo | Base               |
| 12    | Servo | Shoulder           |
| 14    | Servo | Elbow              |
| 27    | Servo | Gripper            |

## 3. Operating Principle

### a. Receiving Control Commands

- The ESP32 runs a web server and a WebSocket server.
- Users access the web interface and send commands to control the car or robot arm.
- Commands are sent via WebSocket as strings, e.g., `CAR,FORWARD,150` or `Base,120`.

### b. Car Motor Control

- Car control commands are parsed and converted into signals for the IN1, IN2, IN3, IN4 pins of the L298N.
- The car can move forward, backward, turn left, turn right, or stop.
- To protect the ESP32, ENA/ENB are connected directly to VCC, not controlled by hardware PWM.

### c. Robot Arm Control

- Four servos are controlled via GPIO pins.
- Commands from the web interface adjust the angle of each servo (base rotation, shoulder lift, elbow bend, gripper open/close).
- There is a record/playback function for movement sequences.

### d. Power Supply

![image](https://github.com/user-attachments/assets/67bc043c-d969-446f-934c-32e9a3db46a5)

## 4. Usage

1. **Upload the code to the ESP32 using Arduino IDE.**
2. **Connect the ESP32 to WiFi (edit SSID and password in the code).**
3. **Open the Serial Monitor to get the IP address.**
4. **Access the IP address in your browser to open the control interface.**
5. **Use the buttons to control the car and the robot arm.**

## 5. Technical Notes

- Do not use hardware PWM for the motors to avoid overheating the chip; only use digital output.
- If a motor does not run, check the wiring, power supply, or try changing the GPIO pin.
- If the ESP32 resets when the servos operate, provide a separate power supply for the servos.

## 6. Authors & Contact

- **Team:** Team NTD
- **Contact:** dung.nguyen230208@vnuk.edu.vn

---

**Main file:** `car.ino`  
**Camera file:** `cam.ino`
