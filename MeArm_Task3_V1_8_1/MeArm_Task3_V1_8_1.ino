// MeArm Task 3 V1.8.1: V1.7 + joystick motion recording and playback
// CAUTION: B/C positions and C grip are proposed, not hardware-tested.
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <avr/pgmspace.h>
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

// ========= Motion Recorder (UNO SRAM-conscious) =========
// Each frame uses 4 bytes. Up to 140 frames * 120 ms = about 16.8 s.
const byte MAX_RECORD_FRAMES = 140;
const unsigned long RECORD_INTERVAL_MS = 120;
const unsigned long PLAYBACK_UPDATE_MS = 20;
const float REPLAY_START_TOLERANCE = 12.0f; // degrees per joint

struct RecordedFrame {
  uint8_t angle[4];
};
RecordedFrame recordedFrames[MAX_RECORD_FRAMES];
byte recordedCount = 0;  // byte supports 0..200
bool recording = false;
bool playingBack = false;
bool playbackPriming = false;
unsigned long lastRecordTime = 0;
unsigned long playbackStartTime = 0;
unsigned long lastPlaybackWrite = 0;


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
const Pose sequenceA[] PROGMEM = {
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
const Pose sequenceB[] PROGMEM = {
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
const Pose sequenceC[] PROGMEM = {
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

// Step names are printed from flash to avoid SRAM allocations.
void printStepName(byte step) {
  switch (step) {
    case 0: Serial.println(F("APPROACH")); break;
    case 1: Serial.println(F("PICK")); break;
    case 2: Serial.println(F("GRASP")); break;
    case 3: Serial.println(F("LIFT")); break;
    case 4: Serial.println(F("TRANSFER")); break;
    case 5: Serial.println(F("PLACE")); break;
    case 6: Serial.println(F("RELEASE")); break;
    case 7: Serial.println(F("RETRACT")); break;
    case 8: Serial.println(F("HOME")); break;
  }
}

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
  Serial.print(F("POSE: "));

  for (byte i = 0; i < N; i++) {
    Serial.print(currentAngle[i], 1);

    if (i < N - 1) {
      Serial.print(F(","));
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
      Serial.println(F("ANGLE OUT OF RANGE"));
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
      Serial.println(F("REMOTE LOOP A: NEXT CYCLE"));
      // No recursion: this branch starts exactly one new pose.
      Pose nextPose;
      memcpy_P(&nextPose, &activeSequence[autoStep], sizeof(Pose));
      if (!startPose(nextPose)) {
        repeatA = false;
        mode = SERIAL_MODE;
        Serial.println(F("REMOTE LOOP FAILED"));
      } else {
        Serial.println(F("STEP 0: APPROACH"));
      }
      return;
    }
    mode = SERIAL_MODE;
    Serial.print(F("AUTO "));
    Serial.print(activeObject);
    Serial.println(F(" COMPLETE"));
    printPose();
    return;
  }

  Serial.print(F("STEP "));
  Serial.print(autoStep);
  Serial.print(F(": "));
  printStepName(autoStep);

  Pose nextPose;
  memcpy_P(&nextPose, &activeSequence[autoStep], sizeof(Pose));
  if (!startPose(nextPose)) {
    mode = SERIAL_MODE;
    Serial.println(F("AUTO FAILED: INVALID POSE"));
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

  Serial.print(F("AUTO "));
  Serial.print(activeObject);
  Serial.println(F(" START"));

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

void cancelRecorder() {
  if (recording) {
    recording = false;
    Serial.println(F("RECORD CANCELLED"));
  }
  if (playingBack) {
    playingBack = false;
    playbackPriming = false;
    Serial.println(F("PLAYBACK STOPPED"));
  }
}

void stopAuto() {
  cancelRecorder();
  sampleMotion(millis());

  moving = false;

  for (byte i = 0; i < N; i++) {
    targetAngle[i] = currentAngle[i];
    writeServoAngle(i, currentAngle[i]);
  }

  mode = SERIAL_MODE;
  waiting = false;
  repeatA = false;

  Serial.println(F("STOPPED"));
  printPose();
}

// ========= Recording & Playback =========
// These are commanded angles, not encoder measurements.
// RAM-only: recordings disappear after reset or power loss.

void captureFrame() {
  if (recordedCount >= MAX_RECORD_FRAMES) return;
  RecordedFrame &frame = recordedFrames[recordedCount++];
  for (byte j = 0; j < N; j++) {
    float a = currentAngle[j];
    if (a < 0.0f) a = 0.0f;
    if (a > 180.0f) a = 180.0f;
    frame.angle[j] = (uint8_t)(a + 0.5f);
  }
}

void finishRecording() {
  if (!recording) return;
  sampleMotion(millis());
  // Include endpoint if interval hasn't elapsed.
  if (recordedCount < MAX_RECORD_FRAMES) captureFrame();
  recording = false;
  Serial.print(F("RECORD SAVED FRAMES: "));
  Serial.println(recordedCount);
}

void beginRecording() {
  if (mode == AUTO_MODE || playingBack) {
    Serial.println(F("RECORD REJECTED: BUSY"));
    return;
  }
  // Enter joystick control. Start only when previous serial motion ends.
  if (moving) {
    Serial.println(F("RECORD REJECTED: MOTION IN PROGRESS"));
    return;
  }
  mode = JOYSTICK_MODE;
  recordedCount = 0;
  recording = true;
  captureFrame();
  lastRecordTime = millis();
  Serial.println(F("RECORD START: MOVE JOYSTICKS; PRESS 2 TO SAVE"));
  Serial.println(F("Maximum about 16.8 seconds (140 frames)"));
}

void updateRecording() {
  if (!recording) return;
  unsigned long now = millis();
  if (now - lastRecordTime < RECORD_INTERVAL_MS) return;
  sampleMotion(now);
  captureFrame();
  lastRecordTime = now;
  if (recordedCount >= MAX_RECORD_FRAMES) {
    recording = false;
    Serial.println(F("RECORD FULL: AUTO SAVED 140 FRAMES"));
  }
}

void finishPlayback() {
  playingBack = false;
  playbackPriming = false;
  Serial.println(F("PLAYBACK COMPLETE"));
  printPose();
}

void beginPlayback() {
  if (recording || mode == AUTO_MODE || moving) {
    Serial.println(F("PLAYBACK REJECTED: BUSY"));
    return;
  }
  if (recordedCount < 2) {
    Serial.println(F("PLAYBACK REJECTED: NO RECORDING"));
    return;
  }
  // Ensure playback starts near the taught starting pose.
  for (byte j = 0; j < N; j++) {
    if (fabsf(currentAngle[j] - recordedFrames[0].angle[j]) >
        REPLAY_START_TOLERANCE) {
      Serial.println(F("PLAYBACK REJECTED: RETURN NEAR RECORD START"));
      return;
    }
  }
  float first[N];
  for (byte j = 0; j < N; j++) first[j] = recordedFrames[0].angle[j];
  mode = SERIAL_MODE; // prevent joystick from modifying commanded trajectory
  playingBack = true;
  playbackPriming = true;
  if (!startMotion(first)) {
    playingBack = false;
    playbackPriming = false;
    return;
  }
  Serial.print(F("PLAYBACK START FRAMES: "));
  Serial.println(recordedCount);
}

void updatePlayback() {
  if (!playingBack) return;
  unsigned long now = millis();
  if (playbackPriming) {
    if (moving) return;
    playbackPriming = false;
    playbackStartTime = now;
    lastPlaybackWrite = 0;
  }
  // Recorded samples are linearly interpolated at 20ms intervals.
  // Do not launch a new eased startMotion() at every recorded frame.
  unsigned long elapsed = now - playbackStartTime;
  unsigned long maxTime = ((unsigned long)recordedCount - 1UL) *
                          RECORD_INTERVAL_MS;
  if (elapsed >= maxTime) {
    for (byte j = 0; j < N; j++) {
      float a = recordedFrames[recordedCount - 1].angle[j];
      currentAngle[j] = targetAngle[j] = a;
      writeServoAngle(j, a);
    }
    finishPlayback();
    return;
  }
  if (now - lastPlaybackWrite < PLAYBACK_UPDATE_MS) return;
  lastPlaybackWrite = now;
  unsigned int k = elapsed / RECORD_INTERVAL_MS;
  float t = (float)(elapsed % RECORD_INTERVAL_MS) /
            (float)RECORD_INTERVAL_MS;
  for (byte j = 0; j < N; j++) {
    float a = recordedFrames[k].angle[j];
    float b = recordedFrames[k + 1].angle[j];
    float angle = a + (b - a) * t;
    currentAngle[j] = targetAngle[j] = angle;
    writeServoAngle(j, angle);
  }
}

// ========= Remote Button Commands =========

void remoteHome() {
  repeatA = false;
  // Interrupt the current commanded trajectory before scheduling HOME.
  stopAuto();
  if (startPose(HOME_POSE)) {
    mode = SERIAL_MODE;
    Serial.println(F("REMOTE 4: HOME TARGET SET"));
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
      if (recording || playingBack) {
        Serial.println(F("REMOTE 1 REJECTED: RECORD/PLAYBACK ACTIVE"));
        break;
      }
      if (repeatA) {
        // A second press cancels the ongoing cycle, not just future repeats.
        stopAuto();
        Serial.println(F("REMOTE 1: LOOP A OFF"));
      } else {
        // If other actions are running, stop first.
        if (moving || mode == AUTO_MODE) stopAuto();
        repeatA = true;
        Serial.println(F("REMOTE 1: LOOP A ON"));
        startAuto('A');
      }
      break;
    case '2':
      if (recording) finishRecording();
      else beginRecording();
      break;
    case '3':
      if (playingBack) stopAuto();
      else beginPlayback();
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

  if (recording || playingBack) {
    if (strcmp(cmd, "P") == 0 || strcmp(cmd, "p") == 0) {
      printPose();
    } else {
      Serial.println(F("BUSY RECORDING/PLAYBACK; SEND ! TO CANCEL"));
    }
    return;
  }

  if (mode == AUTO_MODE) {
    // Permit status queries during automatic movement.
    if (strcmp(cmd, "P") == 0 ||
        strcmp(cmd, "p") == 0) {
      printPose();
    } else {
      Serial.println(F("AUTO RUNNING; ! TO STOP"));
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
        Serial.println(F("JOYSTICK MODE"));
        return;

      case 'M':
        mode = SERIAL_MODE;
        Serial.println(F("SERIAL MODE"));
        return;

      case 'O': {
        float next[N];

        for (byte i = 0; i < N; i++) {
          next[i] = currentAngle[i];
        }

        next[CLAW] = CLAW_OPEN;

        startMotion(next);
        Serial.println(F("CLAW OPEN"));
        return;
      }

      case 'S': {
        float next[N];

        for (byte i = 0; i < N; i++) {
          next[i] = currentAngle[i];
        }

        next[CLAW] = CLAW_CLOSE;

        startMotion(next);
        Serial.println(F("CLAW CLOSE"));
        return;
      }

      case 'H':
        motionSpeed = 25.0f;
        Serial.println(F("SPEED: HIGH 25 deg/s"));
        return;

      case 'L':
        motionSpeed = 8.0f;
        Serial.println(F("SPEED: LOW 8 deg/s"));
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
      Serial.println(F("XYZ TARGET ACCEPTED"));
    }

    return;
  }

  Serial.println(F("UNKNOWN COMMAND"));
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
        Serial.println(F("COMMAND TOO LONG"));
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

  Serial.println(F("MeArm Task 3 V1.8.1 Ready (V1.6 Fast core)"));
  Serial.println(F("Remote 1: LOOP A; 2: RECORD; 3: PLAY; 4: HOME"));
  Serial.println(F("Commands: A B C ! P J M O S H L"));
  Serial.println(F("XYZ: x90,y90,z90"));

  printPose();
}

void loop() {
  readRemote();
  readSerial();
  updateMotion();
  updateJoystick();
  updateRecording();
  updatePlayback();
  updateAuto();
}
