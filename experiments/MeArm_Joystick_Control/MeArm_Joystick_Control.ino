
#include <Servo.h>

// 四个舵机
Servo baseServo;
Servo leftServo;
Servo rightServo;
Servo clawServo;

// 舵机信号引脚
const int BASE_PIN  = 9;
const int LEFT_PIN  = 8;
const int RIGHT_PIN = 7;
const int CLAW_PIN  = 6;

// 摇杆模拟引脚
const int BASE_JOY  = A0;
const int LEFT_JOY  = A1;
const int RIGHT_JOY = A3;
const int CLAW_JOY  = A2;

// 当前目标角度
int baseAngle  = 90;
int leftAngle  = 90;
int rightAngle = 90;
int clawAngle  = 30;

// 初始测试限位
const int BASE_MIN  = 85, BASE_MAX  = 95;
const int LEFT_MIN  = 85, LEFT_MAX  = 95;
const int RIGHT_MIN = 85, RIGHT_MAX = 95;
const int CLAW_MIN  = 30, CLAW_MAX  = 45;

// 摇杆死区
const int DEAD_LOW  = 450;
const int DEAD_HIGH = 570;

// 每次更新间隔：毫秒
const unsigned long UPDATE_INTERVAL = 100;
unsigned long lastUpdate = 0;

void setup() {
  Serial.begin(9600);

  // 预设目标角度
  baseServo.write(baseAngle);
  leftServo.write(leftAngle);
  rightServo.write(rightAngle);
  clawServo.write(clawAngle);

  // 连接舵机信号
  baseServo.attach(BASE_PIN);
  leftServo.attach(LEFT_PIN);
  rightServo.attach(RIGHT_PIN);
  clawServo.attach(CLAW_PIN);

  Serial.println("MeArm Joystick Control Ready");
}

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

void loop() {
  unsigned long now = millis();

  if (now - lastUpdate >= UPDATE_INTERVAL) {
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
