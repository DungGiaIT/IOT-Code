# CarAramRobot - Điều Khiển Xe 3 Bánh và Cánh Tay Robot bằng ESP32

## 1. Giới thiệu

Dự án này sử dụng ESP32 để điều khiển một xe 3 bánh tích hợp cánh tay robot. Hệ thống hỗ trợ điều khiển qua WiFi bằng giao diện web (WebSocket), cho phép người dùng điều khiển cả xe và cánh tay robot từ xa trên điện thoại hoặc máy tính.

## 2. Sơ đồ phần cứng

- **Vi điều khiển:** ESP32
- **Driver động cơ:** L298N
- **Động cơ DC:** 2 động cơ (trái/phải)
- **Servo:** 4 servo cho cánh tay robot (Base, Shoulder, Elbow, Gripper)
- **Nguồn:** ESP32 dùng nguồn riêng, động cơ và servo dùng nguồn 5V riêng để tránh sụt áp

### Kết nối chân GPIO:
| ESP32 | L298N | Chức năng         |
|-------|-------|-------------------|
| 22    | IN1   | Motor A (trái)    |
| 23    | IN2   | Motor A (trái)    |
| 5     | IN3   | Motor B (phải)    |
| 18    | IN4   | Motor B (phải)    |
| 13    | Servo | Base              |
| 12    | Servo | Shoulder          |
| 14    | Servo | Elbow             |
| 27    | Servo | Gripper           |

## 3. Nguyên lý hoạt động

### a. Nhận lệnh điều khiển

- ESP32 tạo một web server và WebSocket server.
- Người dùng truy cập giao diện web, gửi lệnh điều khiển xe hoặc cánh tay robot.
- Lệnh gửi qua WebSocket dạng chuỗi, ví dụ: `CAR,FORWARD,150` hoặc `Base,120`.

### b. Điều khiển động cơ xe

- Lệnh điều khiển xe được phân tích và chuyển thành tín hiệu điều khiển các chân IN1, IN2, IN3, IN4 của L298N.
- Xe có thể tiến, lùi, rẽ trái, rẽ phải hoặc dừng lại.
- Để bảo vệ ESP32, ENA/ENB được nối trực tiếp VCC, không dùng PWM phần cứng.

### c. Điều khiển cánh tay robot

- 4 servo được điều khiển qua các chân GPIO.
- Lệnh từ web sẽ điều chỉnh góc từng servo (xoay đế, nâng vai, gập khuỷu, mở/đóng gắp).
- Có chức năng record/play lại các bước chuyển động.

### d. Cấp nguồn
![image](https://github.com/user-attachments/assets/67bc043c-d969-446f-934c-32e9a3db46a5)


## 4. Cách sử dụng

1. **Nạp code cho ESP32 bằng Arduino IDE.**
2. **Kết nối ESP32 với WiFi (SSID và password chỉnh trong code).**
3. **Mở Serial Monitor để lấy địa chỉ IP.**
4. **Truy cập địa chỉ IP trên trình duyệt để mở giao diện điều khiển.**
5. **Sử dụng các nút để điều khiển xe và cánh tay robot.**

## 5. Một số lưu ý kỹ thuật

- Không dùng PWM phần cứng cho động cơ để tránh nóng chip, chỉ dùng digital output.
- Nếu một motor không chạy, kiểm tra lại dây nối, nguồn, hoặc thử đổi chân GPIO.
- Nếu ESP32 bị reset khi servo hoạt động, cần cấp nguồn riêng cho servo.

## 6. Tác giả & liên hệ

- **Nhóm thực hiện:** Team NTD
- **Liên hệ:** dung.nguyen230208@vnuk.edu.vn

---

**File chính:** `car.ino`  
**File Cam:** `cam.ino`
