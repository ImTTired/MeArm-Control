
#include <Servo.h>

// ========== 舵机对象 ==========
Servo baseServo;
Servo leftServo;
Servo rightServo;
Servo clawServo;

// ========== 舵机引脚 ==========
const int BASE_PIN = 9;
const int LEFT_PIN = 8;
const int RIGHT_PIN = 7;
const int CLAW_PIN = 6;

// ========== 摇杆引脚 ==========
const int BASE_JOY = A0;
const int LEFT_JOY = A1;
const int RIGHT_JOY = A3;
const int CLAW_JOY = A2;

// ========== 当前指令角度 ==========
int baseAngle = 90;
int leftAngle = 90;
int rightAngle = 90;
int clawAngle = 30;

// ========== 临时安全限位 ==========
const int BASE_MIN = 85;
const int BASE_MAX = 95;

const int LEFT_MIN = 85;
const int LEFT_MAX = 95;

const int RIGHT_MIN = 85;
const int RIGHT_MAX = 95;

const int CLAW_MIN = 30;
const int CLAW_MAX = 45;

// ========== 摇杆死区 ==========
const int DEAD_LOW = 450;
const int DEAD_HIGH = 570;

// ========== 速度参数 ==========
unsigned long updateInterval = 100;
unsigned long lastUpdate = 0;

// ========== 初始化 ==========
void setup() {
  Serial.begin(9600);

  baseServo.write(baseAngle);
  leftServo.write(leftAngle);
  rightServo.write(rightAngle);
  clawServo.write(clawAngle);

  baseServo.attach(BASE_PIN);
  leftServo.attach(LEFT_PIN);
  rightServo.attach(RIGHT_PIN);
  clawServo.attach(CLAW_PIN);

  Serial.println("MeArm Control V1.1 Ready");
  Serial.println("Commands: O S H L");
}

// ========== 摇杆关节控制 ==========
void updateJoint(Servo &servo,
                 int joyPin,
                 int &angle,
                 int minAngle,
                 int maxAngle) {

  int value = analogRead(joyPin);
  int direction = 0;

  if (value < DEAD_LOW) {
    direction = -1;
  }
  else if (value > DEAD_HIGH) {
    direction = 1;
  }

  if (direction != 0) {
    angle = constrain(angle + direction,
                      minAngle, maxAngle);
    servo.write(angle);
  }
}

// ========== 电脑串口控制 ==========
void handleSerial() {

  while (Serial.available() > 0) {

    char cmd = Serial.read();

    switch (cmd) {

      case 'O':
      case 'o':
        clawAngle = CLAW_MIN;
        clawServo.write(clawAngle);
        Serial.println("Gripper OPEN");
        break;

      case 'S':
      case 's':
        clawAngle = CLAW_MAX;
        clawServo.write(clawAngle);
        Serial.println("Gripper CLOSE");
        break;

      case 'H':
      case 'h':
        updateInterval = 50;
        Serial.println("Speed HIGH");
        break;

      case 'L':
      case 'l':
        updateInterval = 200;
        Serial.println("Speed LOW");
        break;

      default:
        // 忽略换行及未知字符
        break;
    }
  }
}

// ========== 主循环 ==========
void loop() {

  handleSerial();

  unsigned long now = millis();

  if (now - lastUpdate >= updateInterval) {

    lastUpdate = now;

    updateJoint(baseServo, BASE_JOY,
                baseAngle, BASE_MIN, BASE_MAX);

    updateJoint(leftServo, LEFT_JOY,
                leftAngle, LEFT_MIN, LEFT_MAX);

    updateJoint(rightServo, RIGHT_JOY,
                rightAngle, RIGHT_MIN, RIGHT_MAX);

    updateJoint(clawServo, CLAW_JOY,
                clawAngle, CLAW_MIN, CLAW_MAX);
  }
}
