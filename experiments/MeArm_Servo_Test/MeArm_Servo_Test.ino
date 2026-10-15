
#include <Servo.h>

Servo baseServo;

const int BASE_PIN = 7;       // 右臂舵机 D7
const int BASE_START = 90;    // 初始参考角度
const int TEST_MIN = 85;      // 测试下限
const int TEST_MAX = 95;      // 测试上限   // 暂定测试上限
int currentAngle = BASE_START;

void setup() {
  Serial.begin(9600);

  // 先设置目标脉宽，再附加舵机
  baseServo.write(BASE_START);
  baseServo.attach(BASE_PIN);

  Serial.println("MeArm Base Servo Test");
  Serial.println("Check power and mechanical clearance.");
  Serial.println("Commands: +, -, ?");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == '+') {
      moveBase(currentAngle + 1);
    }
    else if (cmd == '-') {
      moveBase(currentAngle - 1);
    }
    else if (cmd == '?') {
      Serial.print("Current command angle: ");
      Serial.println(currentAngle);
    }
  }
}

void moveBase(int targetAngle) {
  targetAngle = constrain(
    targetAngle, TEST_MIN, TEST_MAX
  );

  if (targetAngle == currentAngle) {
    return;
  }

  currentAngle = targetAngle;
  baseServo.write(currentAngle);

  Serial.print("Base target angle: ");
  Serial.println(currentAngle);
}
