
#include <Servo.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

// 关节编号
enum {
  BASE = 0,
  LEFT = 1,
  RIGHT = 2,
  CLAW = 3,
  JOINT_COUNT = 4
};

// 四个舵机
Servo servos[JOINT_COUNT];

// V3 扩展板引脚
const byte servoPins[4] = {9, 8, 7, 6};
const byte joyPins[4] = {A0, A1, A3, A2};

// 当前命令角度与目标角度
int currentAngle[4] = {90, 90, 90, 30};
int targetAngle[4]  = {90, 90, 90, 30};

// 当前已经测试过的角度区间
const int minAngle[4] = {85, 85, 85, 30};
const int maxAngle[4] = {95, 95, 95, 45};

// 摇杆死区
const int DEAD_LOW = 450;
const int DEAD_HIGH = 570;

// 控制模式
enum ControlMode {
  JOYSTICK_MODE,
  SERIAL_MODE
};

ControlMode mode = JOYSTICK_MODE;

// 运动速度
unsigned long updateInterval = 100;
unsigned long lastUpdate = 0;

// 串口命令缓存
char commandBuffer[48];
byte commandLength = 0;

// 每个关节每次只变化 1 度
void moveOneStep(byte joint) {
  if (currentAngle[joint] < targetAngle[joint]) {
    currentAngle[joint]++;
  }
  else if (currentAngle[joint] > targetAngle[joint]) {
    currentAngle[joint]--;
  }

  servos[joint].write(currentAngle[joint]);
}

// 读取摇杆，产生关节目标角度
void updateJoystick() {
  for (byte i = 0; i < JOINT_COUNT; i++) {
    int value = analogRead(joyPins[i]);

    if (value < DEAD_LOW) {
      targetAngle[i] = currentAngle[i] - 1;
    }
    else if (value > DEAD_HIGH) {
      targetAngle[i] = currentAngle[i] + 1;
    }
    else {
      targetAngle[i] = currentAngle[i];
    }

    targetAngle[i] = constrain(
      targetAngle[i],
      minAngle[i],
      maxAngle[i]
    );
  }
}

// 三舵机协同目标
void setXYZ(int x, int y, int z) {
  if (x < minAngle[BASE] || x > maxAngle[BASE] ||
      y < minAngle[LEFT] || y > maxAngle[LEFT] ||
      z < minAngle[RIGHT] || z > maxAngle[RIGHT]) {

    Serial.println("ERROR: Angle out of range");
    return;
  }

  mode = SERIAL_MODE;

  targetAngle[BASE] = x;
  targetAngle[LEFT] = y;
  targetAngle[RIGHT] = z;

  Serial.print("XYZ target: ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(z);
}

// 处理完整的一条串口命令
void executeCommand(char *cmd) {
  int x, y, z;
  int consumed = 0;

  if (strlen(cmd) == 1) {
    char c = toupper((unsigned char)cmd[0]);

    switch (c) {
      case 'O':
        targetAngle[CLAW] = minAngle[CLAW];
        Serial.println("Gripper OPEN");
        return;

      case 'S':
        targetAngle[CLAW] = maxAngle[CLAW];
        Serial.println("Gripper CLOSE");
        return;

      case 'H':
        updateInterval = 50;
        Serial.println("Speed HIGH");
        return;

      case 'L':
        updateInterval = 200;
        Serial.println("Speed LOW");
        return;

      case 'J':
        mode = JOYSTICK_MODE;
        Serial.println("Mode JOYSTICK");
        return;

      case 'M':
        mode = SERIAL_MODE;
        for (byte i = 0; i < JOINT_COUNT; i++) {
          targetAngle[i] = currentAngle[i];
        }
        Serial.println("Mode SERIAL");
        return;
    }
  }

  // 格式：x90,y90,z90
  if (sscanf(cmd, "x%d,y%d,z%d%n",
             &x, &y, &z, &consumed) == 3 &&
      consumed == (int)strlen(cmd)) {
    setXYZ(x, y, z);
    return;
  }

  Serial.println("ERROR: Unknown command");
}

// 非阻塞串口逐字符接收
void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        executeCommand(commandBuffer);
        commandLength = 0;
      }
    }
    else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    }
    else {
      commandLength = 0;
      Serial.println("ERROR: Command too long");
    }
  }
}

void setup() {
  Serial.begin(9600);

  // 先预设目标，再 attach
  for (byte i = 0; i < JOINT_COUNT; i++) {
    servos[i].write(currentAngle[i]);
    servos[i].attach(servoPins[i]);
  }

  Serial.println("MeArm Control V1.2 Ready");
  Serial.println("Commands: O S H L J M");
  Serial.println("XYZ example: x90,y90,z90");
}

void loop() {
  handleSerial();

  unsigned long now = millis();

  if (now - lastUpdate >= updateInterval) {
    lastUpdate = now;

    if (mode == JOYSTICK_MODE) {
      updateJoystick();
    }

    for (byte i = 0; i < JOINT_COUNT; i++) {
      moveOneStep(i);
    }
  }
}
