
#include <Arduino.h>
#include <Servo.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

const byte N = 4;
const byte BASE = 0;
const byte LEFT = 1;
const byte RIGHT = 2;
const byte CLAW = 3;

Servo servos[N];

const byte servoPins[N] = {9, 8, 7, 6};
const byte joyPins[N] = {A0, A1, A3, A2};

const int minAngle[N] = {0, 45, 0, 30};
const int maxAngle[N] = {180, 160, 180, 100};

int currentAngle[N] = {90, 90, 90, 30};
int targetAngle[N]  = {90, 90, 90, 30};

const int CLAW_OPEN = 30;
const int CLAW_CLOSE = 84;

unsigned long stepInterval = 100;
unsigned long lastStep = 0;

enum ControlMode {
  JOYSTICK_MODE,
  SERIAL_MODE,
  AUTO_MODE
};

ControlMode mode = JOYSTICK_MODE;

struct Pose {
  int base;
  int left;
  int right;
  int claw;
};

// A 的前三组姿态来自实测。
// 其余位置是候选值，必须先空载验证。
const Pose sequenceA[] = {
  {60, 130, 86, 30},   // 0 APPROACH
  {60, 130, 43, 30},   // 1 PICK
  {60, 130, 43, 84},   // 2 GRASP
  {60, 130, 86, 84},   // 3 LIFT
  {100, 130, 86, 84},  // 4 TRANSFER
  {100, 130, 55, 84},  // 5 PLACE
  {100, 130, 55, 30},  // 6 RELEASE
  {100, 130, 86, 30},  // 7 RETRACT
  {90, 90, 90, 30}     // 8 HOME
};

const byte SEQUENCE_LENGTH =
  sizeof(sequenceA) / sizeof(sequenceA[0]);

const char *stepNames[] = {
  "APPROACH", "PICK", "GRASP",
  "LIFT", "TRANSFER", "PLACE",
  "RELEASE", "RETRACT", "HOME"
};

byte autoStep = 0;
bool waiting = false;
unsigned long waitStart = 0;

char buffer[48];
byte bufferIndex = 0;

bool validAngle(byte joint, int angle) {
  return angle >= minAngle[joint] &&
         angle <= maxAngle[joint];
}

void printPose() {
  Serial.print("POSE: ");
  for (byte i = 0; i < N; i++) {
    Serial.print(currentAngle[i]);
    if (i < N - 1) Serial.print(",");
  }
  Serial.println();
}

void setPose(const Pose &p) {
  targetAngle[BASE] = p.base;
  targetAngle[LEFT] = p.left;
  targetAngle[RIGHT] = p.right;
  targetAngle[CLAW] = p.claw;
}

bool reachedTarget() {
  for (byte i = 0; i < N; i++) {
    if (currentAngle[i] != targetAngle[i])
      return false;
  }
  return true;
}

void beginAutoStep() {
  if (autoStep >= SEQUENCE_LENGTH) {
    mode = SERIAL_MODE;
    Serial.println("AUTO A COMPLETE");
    printPose();
    return;
  }

  setPose(sequenceA[autoStep]);

  Serial.print("STEP ");
  Serial.print(autoStep);
  Serial.print(": ");
  Serial.println(stepNames[autoStep]);

  waiting = false;
}

void startAutoA() {
  mode = AUTO_MODE;
  autoStep = 0;

  // 自动流程使用更慢的步进速度
  stepInterval = 100;

  Serial.println("AUTO A START");
  beginAutoStep();
}

void stopAuto() {
  for (byte i = 0; i < N; i++) {
    targetAngle[i] = currentAngle[i];
  }

  mode = SERIAL_MODE;
  waiting = false;

  Serial.println("AUTO STOPPED");
  printPose();
}

void updateAuto() {
  if (mode != AUTO_MODE) return;
  if (!reachedTarget()) return;

  if (!waiting) {
    waiting = true;
    waitStart = millis();
    return;
  }

  // 夹紧和松开之后多等待一会儿
  unsigned long waitTime =
    (autoStep == 2 || autoStep == 6) ? 800 : 400;

  if (millis() - waitStart < waitTime)
    return;

  autoStep++;
  beginAutoStep();
}

void updateServos() {
  if (millis() - lastStep < stepInterval)
    return;

  lastStep = millis();

  for (byte i = 0; i < N; i++) {
    if (currentAngle[i] < targetAngle[i])
      currentAngle[i]++;
    else if (currentAngle[i] > targetAngle[i])
      currentAngle[i]--;

    servos[i].write(currentAngle[i]);
  }
}

void updateJoystick() {
  if (mode != JOYSTICK_MODE) return;
  if (!reachedTarget()) return;

  for (byte i = 0; i < N; i++) {
    int value = analogRead(joyPins[i]);
    int change = 0;

    if (value > 570) change = 1;
    if (value < 450) change = -1;

    int next = currentAngle[i] + change;

    if (validAngle(i, next))
      targetAngle[i] = next;
  }
}

void processCommand(char *cmd) {
  if (strcmp(cmd, "!") == 0) {
    stopAuto();
    return;
  }

  if (mode == AUTO_MODE) {
    Serial.println("AUTO RUNNING; send ! to stop");
    return;
  }

  if (strlen(cmd) == 1) {
    char c = toupper((unsigned char)cmd[0]);

    switch (c) {
      case 'P':
        printPose();
        return;

      case 'J':
        mode = JOYSTICK_MODE;
        Serial.println("JOYSTICK MODE");
        return;

      case 'M':
        mode = SERIAL_MODE;
        Serial.println("SERIAL MODE");
        return;

      case 'O':
        targetAngle[CLAW] = CLAW_OPEN;
        Serial.println("CLAW OPEN");
        return;

      case 'S':
        targetAngle[CLAW] = CLAW_CLOSE;
        Serial.println("CLAW CLOSE");
        return;

      case 'H':
        stepInterval = 50;
        Serial.println("HIGH SPEED");
        return;

      case 'L':
        stepInterval = 200;
        Serial.println("LOW SPEED");
        return;

      case 'A':
        startAutoA();
        return;
    }
  }

  int x, y, z, consumed = 0;

  if (sscanf(cmd, "x%d,y%d,z%d%n",
             &x, &y, &z, &consumed) == 3 &&
      consumed == (int)strlen(cmd)) {

    if (validAngle(BASE, x) &&
        validAngle(LEFT, y) &&
        validAngle(RIGHT, z)) {

      mode = SERIAL_MODE;

      targetAngle[BASE] = x;
      targetAngle[LEFT] = y;
      targetAngle[RIGHT] = z;

      Serial.println("XYZ TARGET ACCEPTED");
    } else {
      Serial.println("ANGLE OUT OF RANGE");
    }

    return;
  }

  Serial.println("UNKNOWN COMMAND");
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    // 单字符紧急中止，不必等待换行
    if (c == '!') {
      bufferIndex = 0;
      buffer[0] = '\0';
      stopAuto();
      continue;
    }

    if (c == '\n') {
      buffer[bufferIndex] = '\0';

      if (bufferIndex > 0)
        processCommand(buffer);

      bufferIndex = 0;
    } else if (c != '\r') {
      if (bufferIndex < sizeof(buffer) - 1) {
        buffer[bufferIndex++] = c;
      } else {
        bufferIndex = 0;
        Serial.println("COMMAND TOO LONG");
      }
    }
  }
}

void setup() {
  Serial.begin(9600);

  for (byte i = 0; i < N; i++) {
    servos[i].write(currentAngle[i]);
    servos[i].attach(servoPins[i]);
  }

  Serial.println("MeArm Control V1.4 Ready");
  Serial.println("Commands: A ! P J M O S H L");
  Serial.println("XYZ: x90,y90,z90");
  printPose();
}

void loop() {
  readSerial();

  updateServos();

  updateJoystick();

  updateAuto();
}
