// MeArm Task 5 V2.0 - Pen calibration and five-point teach-in
// This is a calibration-only program: it DOES NOT draw automatically.
// UNO pins are the same as the user-tested V1.6: servos D9,D8,D7,D6;
// joysticks A0,A1,A3,A2. The remote controller is not used in this version.
#include <Arduino.h>
#include <Servo.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

const byte JOINTS = 4;
const byte SERVO_PINS[JOINTS] = {9, 8, 7, 6};
const byte JOY_PINS[JOINTS] = {A0, A1, A3, A2};
const byte MIN_DEG[JOINTS] = {0, 45, 0, 30};
const byte MAX_DEG[JOINTS] = {180, 160, 180, 100};
const byte HOME[JOINTS] = {90, 90, 90, 30};

Servo motors[JOINTS];
int pose[JOINTS] = {90, 90, 90, 30};
const byte MAX_POINTS = 5;
byte points[MAX_POINTS][JOINTS];
byte pointCount = 0;
unsigned long lastStepMs = 0;
unsigned long stepMs = 110;  // slow manual positioning, only when joystick held
char command[36];
byte cmdLen = 0;
bool manualEnabled = true;

void printPose() {
  Serial.print(F("POSE: "));
  for (byte i = 0; i < JOINTS; ++i) {
    Serial.print(pose[i]);
    if (i < JOINTS - 1) Serial.print(',');
  }
  Serial.println();
}

void printPoints() {
  Serial.print(F("POINTS: "));
  Serial.println(pointCount);
  for (byte p = 0; p < pointCount; ++p) {
    Serial.print(F("P")); Serial.print(p + 1); Serial.print(F(": "));
    for (byte i = 0; i < JOINTS; ++i) {
      Serial.print(points[p][i]);
      if (i < JOINTS - 1) Serial.print(',');
    }
    Serial.println();
  }
}

void recordPoint() {
  if (pointCount >= MAX_POINTS) {
    Serial.println(F("FULL: use X to clear saved points"));
    return;
  }
  // Joystick movement is incremental and writes the same pose[] commands;
  // it must be released before recording a point.
  for (byte i = 0; i < JOINTS; ++i) points[pointCount][i] = (byte)pose[i];
  ++pointCount;
  Serial.print(F("SAVED P")); Serial.println(pointCount);
  printPose();
}

void handleCommand(char *cmd) {
  if (strlen(cmd) != 1) { Serial.println(F("UNKNOWN")); return; }
  char c = toupper((unsigned char)cmd[0]);
  switch (c) {
    case 'P': printPose(); break;
    case 'K': recordPoint(); break;
    case 'D': printPoints(); break;
    case 'X': pointCount = 0; Serial.println(F("CLEARED")); break;
    case 'J': manualEnabled = true; Serial.println(F("JOYSTICK ON")); break;
    case 'M': manualEnabled = false; Serial.println(F("JOYSTICK OFF")); break;
    case 'L': stepMs = 160; Serial.println(F("SLOW MANUAL")); break;
    case 'H': stepMs = 80; Serial.println(F("FAST MANUAL")); break;
    case '!': manualEnabled = false; Serial.println(F("MANUAL DISABLED")); break;
    default: Serial.println(F("UNKNOWN")); break;
  }
}

void readUsbSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '!') { manualEnabled = false; cmdLen = 0; Serial.println(F("MANUAL DISABLED")); continue; }
    if (c == '\n') {
      command[cmdLen] = '\0';
      if (cmdLen) handleCommand(command);
      cmdLen = 0;
    } else if (c != '\r') {
      if (cmdLen < sizeof(command) - 1) command[cmdLen++] = c;
      else { cmdLen = 0; Serial.println(F("COMMAND OVERFLOW")); }
    }
  }
}

void updateJoystick() {
  if (!manualEnabled) return;
  unsigned long now = millis();
  if (now - lastStepMs < stepMs) return;
  lastStepMs = now;
  for (byte i = 0; i < JOINTS; ++i) {
    int raw = analogRead(JOY_PINS[i]);
    int increment = (raw > 570) ? 1 : ((raw < 450) ? -1 : 0);
    int next = pose[i] + increment;
    if (increment && next >= MIN_DEG[i] && next <= MAX_DEG[i]) {
      pose[i] = next;
      motors[i].write(pose[i]);
    }
  }
}

void setup() {
  Serial.begin(9600);
  // As in tested V1.6, servos receive their assumed startup setpoints
  // on attach. Power servos only with arm supported in compatible pose.
  for (byte i = 0; i < JOINTS; ++i) {
    motors[i].write(HOME[i]);
    motors[i].attach(SERVO_PINS[i]);
  }
  Serial.println(F("MeArm Task 5 V2.0 - TEACH ONLY"));
  Serial.println(F("P pose | K save point | D dump | X clear"));
  Serial.println(F("J joystick on | M off | L/H manual speed"));
  Serial.println(F("Max 5 points. NO AUTO DRAWING."));
  printPose();
}

void loop() {
  readUsbSerial();
  updateJoystick();
}
