// MeArm Task 3 V1.7: V1.6 Fast + four-button remote (buttons 1 and 4)
// CAUTION: B/C positions and C grip are proposed, not hardware-tested.
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

// ========= Hardware =========

const byte N = 4;

const byte BASE  = 0;
const byte LEFT  = 1;
const byte RIGHT = 2;
const byte CLAW  = 3;

Servo servos[N];

// Remote UART: UNO D10 RX <- remote TX; D11 TX -> remote RX. Common GND.
SoftwareSerial remoteSerial(10, 11);
const unsigned long REMOTE_DEBOUNCE_MS = 250;
unsigned long lastRemoteButtonTime = 0;
char lastRemoteButton = 0;
bool repeatA = false;

const byte servoPins[N] = {9, 8, 7, 6};
const byte joyPins[N]   = {A0, A1, A3, A2};

const int minAngle[N] = {0, 45, 0, 30};
const int maxAngle[N] = {180, 160, 180, 100};

const float initialPose[N] = {90, 90, 90, 30};

const int CLAW_OPEN  = 30;
const int CLAW_CLOSE = 94; // generic S command (A); B auto uses 100

// ========= Motion Parameters =========

// Largest moving joint's average speed, deg/s
float motionSpeed = 15.0f;

// Motion interpolation update interval
const unsigned long UPDATE_MS = 20;

// Minimum duration of each movement
const unsigned long MIN_DURATION_MS = 350;

// Joystick movement polling interval
const unsigned long JOYSTICK_MS = 100;

unsigned long lastJoystick = 0;

// ========= Motion Engine =========

float currentAngle[N];
float startAngle[N];
float targetAngle[N];

unsigned long motionStart = 0;
unsigned long motionDuration = 0;

bool moving = false;

enum ControlMode {
  JOYSTICK_MODE,
  SERIAL_MODE,
  AUTO_MODE
};

ControlMode mode = JOYSTICK_MODE;

// ========= Pose Definition =========

struct Pose {
  int base;
  int left;
  int right;
  int claw;
};

// A: measured pickup angles; B/C: proposed station angles.
// Verify all B/C poses EMPTY before gripping any object.
// Layout: A at base 60, B at 90, C at 120 deg;
// right-side destinations use base 100, 130, 160 deg.
const Pose sequenceA[] = {
  {60,130,86,30},   // APPROACH
  {60,130,43,30},   // PICK
  {60,130,43,94},   // GRASP: user-confirmed closing target
  {60,130,86,94},   // LIFT
  {100,130,86,94},  // TRANSFER
  {100,130,55,94},  // PLACE
  {100,130,55,30},  // RELEASE
  {100,130,86,30},  // RETRACT
  {90,90,90,30}     // HOME
};
const Pose sequenceB[] = {
  {90,130,86,30},
  {90,130,43,30},
  {90,130,43,100},  // B gripper close
  {90,130,86,100},
  {130,130,86,100},
  {130,130,55,100},
  {130,130,55,30},
  {130,130,86,30},
  {90,90,90,30}
};
const Pose sequenceC[] = {
  {120,130,86,30},
  {120,130,43,30},
  {120,130,43,94},  // C TEMPORARY: must validate suitable grip
  {120,130,86,94},
  {160,130,86,94},
  {160,130,55,94},
  {160,130,55,30},
  {160,130,86,30},
  {90,90,90,30}
};
const Pose *activeSequence = sequenceA;
char activeObject = 'A';

const char *stepNames[] = {
  "APPROACH",
  "PICK",
  "GRASP",
  "LIFT",
  "TRANSFER",
  "PLACE",
  "RELEASE",
  "RETRACT",
  "HOME"
};

const Pose HOME_POSE = {90, 90, 90, 30};

const byte SEQUENCE_LENGTH =
    sizeof(sequenceA) / sizeof(sequenceA[0]);

byte autoStep = 0;
bool waiting = false;
unsigned long waitStart = 0;

// ========= Serial Buffer =========

char buffer[48];
byte bufferIndex = 0;

// ========= Utility =========

bool validAngle(byte joint, float angle) {
  return angle >= minAngle[joint] &&
         angle <= maxAngle[joint];
}

void printPose() {
  Serial.print("POSE: ");

  for (byte i = 0; i < N; i++) {
    Serial.print(currentAngle[i], 1);

    if (i < N - 1) {
      Serial.print(",");
    }
  }

  Serial.println();
}

void writeServoAngle(byte joint, float angle) {
  // Match Arduino Servo default angle mapping:
  // 0 deg -> 544 us, 180 deg -> 2400 us.
  // Fractional angle commands avoid unnecessary
  // whole-degree quantization.
  float pulse = 544.0f +
                angle * (1856.0f / 180.0f);

  int us = (int)(pulse + 0.5f);
  servos[joint].writeMicroseconds(us);
}

// Quintic smoothstep
float smoothstep5(float t) {
  if (t <= 0.0f) return 0.0f;
  if (t >= 1.0f) return 1.0f;

  return t * t * t *
         (10.0f + t * (-15.0f + 6.0f * t));
}

void sampleMotion(unsigned long now) {
  if (!moving) return;

  float t = (float)(now - motionStart) /
            (float)motionDuration;

  if (t >= 1.0f) t = 1.0f;

  float s = smoothstep5(t);

  for (byte i = 0; i < N; i++) {
    currentAngle[i] =
      startAngle[i] +
      (targetAngle[i] - startAngle[i]) * s;
  }

  if (t >= 1.0f) {
    moving = false;
  }
}

void updateMotion() {
  if (!moving) return;

  unsigned long now = millis();

  sampleMotion(now);

  // Servo library keeps generating pulses between
  // updates. We refresh target positions frequently.
  static unsigned long lastWrite = 0;

  if (now - lastWrite >= UPDATE_MS || !moving) {
    lastWrite = now;

    for (byte i = 0; i < N; i++) {
      writeServoAngle(i, currentAngle[i]);
    }
  }
}

bool startMotion(const float next[N]) {
  for (byte i = 0; i < N; i++) {
    if (!validAngle(i, next[i])) {
      Serial.println("ANGLE OUT OF RANGE");
      return false;
    }
  }

  // Preserve continuity if a new command arrives
  // while the previous motion is still running.
  sampleMotion(millis());

  float maxDistance = 0.0f;

  for (byte i = 0; i < N; i++) {
    startAngle[i] = currentAngle[i];
    targetAngle[i] = next[i];

    float distance =
      fabsf(targetAngle[i] - startAngle[i]);

    if (distance > maxDistance) {
      maxDistance = distance;
    }
  }

  if (maxDistance < 0.01f) {
    moving = false;

    for (byte i = 0; i < N; i++) {
      currentAngle[i] = targetAngle[i];
      writeServoAngle(i, currentAngle[i]);
    }

    return true;
  }

  float durationSeconds =
    maxDistance / motionSpeed;

  motionDuration =
    (unsigned long)(durationSeconds * 1000.0f);

  if (motionDuration < MIN_DURATION_MS) {
    motionDuration = MIN_DURATION_MS;
  }

  motionStart = millis();
  moving = true;

  return true;
}

bool startPose(const Pose &p) {
  float next[N] = {
    (float)p.base,
    (float)p.left,
    (float)p.right,
    (float)p.claw
  };

  return startMotion(next);
}

// ========= Auto Sequence =========

void beginAutoStep() {
  if (autoStep >= SEQUENCE_LENGTH) {
    if (repeatA && activeObject == 'A') {
      // A cycle finished. Start the next cycle at the next HOME endpoint.
      autoStep = 0;
      waiting = false;
      Serial.println("REMOTE LOOP A: NEXT CYCLE");
      // No recursion: this branch starts exactly one new pose.
      if (!startPose(activeSequence[autoStep])) {
        repeatA = false;
        mode = SERIAL_MODE;
        Serial.println("REMOTE LOOP FAILED");
      } else {
        Serial.println("STEP 0: APPROACH");
      }
      return;
    }
    mode = SERIAL_MODE;
    Serial.print("AUTO ");
    Serial.print(activeObject);
    Serial.println(" COMPLETE");
    printPose();
    return;
  }

  Serial.print("STEP ");
  Serial.print(autoStep);
  Serial.print(": ");
  Serial.println(stepNames[autoStep]);

  if (!startPose(activeSequence[autoStep])) {
    mode = SERIAL_MODE;
    Serial.println("AUTO FAILED: INVALID POSE");
    return;
  }

  waiting = false;
}

void startAuto(char objectCode) {
  if (objectCode == 'A') activeSequence = sequenceA;
  else if (objectCode == 'B') activeSequence = sequenceB;
  else if (objectCode == 'C') activeSequence = sequenceC;
  else return;
  activeObject = objectCode;
  mode = AUTO_MODE;
  autoStep = 0;
  waiting = false;

  Serial.print("AUTO ");
  Serial.print(activeObject);
  Serial.println(" START");

  beginAutoStep();
}

void updateAuto() {
  if (mode != AUTO_MODE) return;
  if (moving) return;

  if (!waiting) {
    waiting = true;
    waitStart = millis();
    return;
  }

  unsigned long waitTime = 400;

  if (autoStep == 2 || autoStep == 6) {
    waitTime = 800;
  }

  if (millis() - waitStart < waitTime) {
    return;
  }

  autoStep++;
  beginAutoStep();
}

void stopAuto() {
  sampleMotion(millis());

  moving = false;

  for (byte i = 0; i < N; i++) {
    targetAngle[i] = currentAngle[i];
    writeServoAngle(i, currentAngle[i]);
  }

  mode = SERIAL_MODE;
  waiting = false;
  repeatA = false;

  Serial.println("STOPPED");
  printPose();
}

// ========= Remote Button Commands =========

void remoteHome() {
  repeatA = false;
  // Interrupt the current commanded trajectory before scheduling HOME.
  stopAuto();
  if (startPose(HOME_POSE)) {
    mode = SERIAL_MODE;
    Serial.println("REMOTE 4: HOME TARGET SET");
  }
}

void handleRemoteButton(char c) {
  if (c < '1' || c > '4') return;
  unsigned long now = millis();
  if (c == lastRemoteButton &&
      now - lastRemoteButtonTime < REMOTE_DEBOUNCE_MS) {
    return;
  }
  lastRemoteButton = c;
  lastRemoteButtonTime = now;

  switch (c) {
    case '1':
      if (repeatA) {
        // A second press cancels the ongoing cycle, not just future repeats.
        stopAuto();
        Serial.println("REMOTE 1: LOOP A OFF");
      } else {
        // If other actions are running, stop first.
        if (moving || mode == AUTO_MODE) stopAuto();
        repeatA = true;
        Serial.println("REMOTE 1: LOOP A ON");
        startAuto('A');
      }
      break;
    case '2':
      Serial.println("REMOTE 2: RECORD NOT IMPLEMENTED YET");
      break;
    case '3':
      Serial.println("REMOTE 3: PLAYBACK NOT IMPLEMENTED YET");
      break;
    case '4':
      remoteHome();
      break;
  }
}

void readRemote() {
  // Remote firmware sends raw button characters 1/2/3/4.
  // Newline is not required; CR/LF is ignored.
  while (remoteSerial.available()) {
    char c = (char)remoteSerial.read();
    handleRemoteButton(c);
  }
}

// ========= Joystick ========= 

void updateJoystick() {
  if (mode != JOYSTICK_MODE) return;
  if (moving) return;

  unsigned long now = millis();

  if (now - lastJoystick < JOYSTICK_MS) {
    return;
  }

  lastJoystick = now;

  float next[N];
  bool changed = false;

  for (byte i = 0; i < N; i++) {
    next[i] = currentAngle[i];

    int value = analogRead(joyPins[i]);
    int change = 0;

    if (value > 570) change = 1;
    if (value < 450) change = -1;

    float candidate = next[i] + change;

    if (change != 0 &&
        validAngle(i, candidate)) {
      next[i] = candidate;
      changed = true;
    }
  }

  if (changed) {
    startMotion(next);
  }
}

// ========= Commands =========

void processCommand(char *cmd) {
  if (strcmp(cmd, "!") == 0) {
    stopAuto();
    return;
  }

  if (mode == AUTO_MODE) {
    // Permit status queries during automatic movement.
    if (strcmp(cmd, "P") == 0 ||
        strcmp(cmd, "p") == 0) {
      printPose();
    } else {
      Serial.println("AUTO RUNNING; ! TO STOP");
    }
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

      case 'O': {
        float next[N];

        for (byte i = 0; i < N; i++) {
          next[i] = currentAngle[i];
        }

        next[CLAW] = CLAW_OPEN;

        startMotion(next);
        Serial.println("CLAW OPEN");
        return;
      }

      case 'S': {
        float next[N];

        for (byte i = 0; i < N; i++) {
          next[i] = currentAngle[i];
        }

        next[CLAW] = CLAW_CLOSE;

        startMotion(next);
        Serial.println("CLAW CLOSE");
        return;
      }

      case 'H':
        motionSpeed = 25.0f;
        Serial.println("SPEED: HIGH 25 deg/s");
        return;

      case 'L':
        motionSpeed = 8.0f;
        Serial.println("SPEED: LOW 8 deg/s");
        return;

      case 'A':
      case 'B':
      case 'C':
        repeatA = false;
        startAuto(c);
        return;
    }
  }

  int x, y, z;
  int consumed = 0;

  if (sscanf(cmd, "x%d,y%d,z%d%n",
             &x, &y, &z, &consumed) == 3 &&
      consumed == (int)strlen(cmd)) {

    float next[N] = {
      (float)x,
      (float)y,
      (float)z,
      currentAngle[CLAW]
    };

    if (startMotion(next)) {
      mode = SERIAL_MODE;
      Serial.println("XYZ TARGET ACCEPTED");
    }

    return;
  }

  Serial.println("UNKNOWN COMMAND");
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    // Emergency software stop without newline.
    if (c == '!') {
      bufferIndex = 0;
      buffer[0] = '\0';
      stopAuto();
      continue;
    }

    if (c == '\n') {
      buffer[bufferIndex] = '\0';

      if (bufferIndex > 0) {
        processCommand(buffer);
      }

      bufferIndex = 0;
    }
    else if (c != '\r') {
      if (bufferIndex < sizeof(buffer) - 1) {
        buffer[bufferIndex++] = c;
      } else {
        bufferIndex = 0;
        Serial.println("COMMAND TOO LONG");
      }
    }
  }
}

// ========= Setup & Loop =========

void setup() {
  Serial.begin(9600);
  remoteSerial.begin(9600);

  for (byte i = 0; i < N; i++) {
    currentAngle[i] = initialPose[i];
    startAngle[i] = initialPose[i];
    targetAngle[i] = initialPose[i];

    servos[i].write((int)initialPose[i]);
    servos[i].attach(servoPins[i]);
  }

  Serial.println("MeArm Task 3 V1.7 Ready (V1.6 Fast core)");
  Serial.println("Remote 1: toggle LOOP A, 4: HOME; 2/3 pending");
  Serial.println("Commands: A B C ! P J M O S H L");
  Serial.println("XYZ: x90,y90,z90");

  printPose();
}

void loop() {
  readRemote();
  readSerial();
  updateMotion();
  updateJoystick();
  updateAuto();
}
