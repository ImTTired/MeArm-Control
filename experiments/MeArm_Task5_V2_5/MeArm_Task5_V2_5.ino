// MeArm Task 5 V2.5 - Wider and centered five-point W trajectory
// Arduino UNO: D9 Base, D8 Left, D7 Right, D6 Claw
// Joystick: A0 Base, A1 Left, A3 Right, A2 Claw
// IMPORTANT: Joint-space interpolation is NOT guaranteed to draw a straight line
// on paper. The points below are candidate poses, NOT calibrated paper contact.
#include <Arduino.h>
#include <Servo.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

const byte NJ = 4;
const byte SERVO_PINS[NJ] = {9, 8, 7, 6};
const byte JOY_PINS[NJ] = {A0, A1, A3, A2};
const byte MIN_DEG[NJ] = {0, 45, 0, 30};
const byte MAX_DEG[NJ] = {180, 160, 180, 100};
Servo servo[NJ];
float angleNow[NJ] = {90, 90, 90, 30};

// Expanded CANDIDATE poses (~2x joint-angle spread from V2.3).
// These are NOT verified A4 paper coordinates or a Cartesian straight-line path.
// Larger left/right sweep, centered symmetrically around P3.
// Base increase moves toward the right in the previously tested A transfer.
// These angles are candidates: verify all five poses and in-between paths unloaded.
// Keep Left/Right from V2.4: vertical pen-tip motion was already observed.
// DO NOT assume the resulting trajectory remains on the paper.
const byte BASE_CENTER = 52;
const byte BASE_HALF_SPAN = 28;  // +/-28 deg; V2.4 was 36 deg end-to-end

const byte W_POINTS[5][NJ] = {
  {BASE_CENTER + BASE_HALF_SPAN, 122, 33, 100}, // P1: Base 80
  {BASE_CENTER + BASE_HALF_SPAN/2, 134, 49, 100}, // P2: Base 66
  {BASE_CENTER,                  128, 39, 100}, // P3: Base 52
  {BASE_CENTER - BASE_HALF_SPAN/2, 140, 54, 100}, // P4: Base 38
  {BASE_CENTER - BASE_HALF_SPAN, 130, 43, 100}  // P5: Base 24
};

// Segments are interpolated in JOINT SPACE as an initial demonstration.
// Reduce this duration only after successful unloaded testing.
const unsigned long SEGMENT_MS = 3500UL;
const unsigned long UPDATE_MS = 20UL;
const unsigned long JOYSTICK_MS = 100UL;
const unsigned long APPROACH_MS = 5500UL;

bool joystickEnabled = true;
bool playing = false;
bool paused = false;

byte segmentIndex = 0;
unsigned long segmentStart = 0;
unsigned long lastWrite = 0;
unsigned long lastJoy = 0;
unsigned long pausedAt = 0;
float segmentFrom[NJ];
float segmentTo[NJ];
char input[32];
byte inputLength = 0;

int rounded(float a) { return (int)(a + 0.5f); }

byte pointAngle(byte p, byte j) {
  return W_POINTS[p][j];
}

void printPose() {
  Serial.print(F("POSE: "));
  for (byte j = 0; j < NJ; ++j) {
    Serial.print(angleNow[j], 1);
    if (j < NJ - 1) Serial.print(',');
  }
  Serial.println();
}

void listPoints() {
  Serial.println(F("POINTS: PRESET (CANDIDATE)"));
  for (byte p = 0; p < 5; ++p) {
    Serial.print(F("P")); Serial.print(p+1); Serial.print(F(": "));
    for (byte j=0; j<NJ; ++j) {
      Serial.print(pointAngle(p,j));
      if (j< NJ-1) Serial.print(',');
    }
    Serial.println();
  }
}

bool validPoint(byte p) {
  for (byte j = 0; j < NJ; ++j) {
    if (pointAngle(p,j) < MIN_DEG[j] || pointAngle(p,j) > MAX_DEG[j]) return false;
  }
  return true;
}

void outputPose() {
  for (byte j=0; j<NJ; ++j) servo[j].write(rounded(angleNow[j]));
}

float smooth(float t) {
  if (t<=0) return 0;
  if (t>=1) return 1;
  return t*t*t*(10.0f + t*(-15.0f+6.0f*t));
}

void startSegment(byte index, unsigned long now) {
  segmentIndex = index;
  segmentStart = now;
  lastWrite = now - UPDATE_MS;
  for (byte j=0; j<NJ; ++j) {
    segmentFrom[j] = angleNow[j];
    segmentTo[j] = (index == 255) ? pointAngle(0,j) : pointAngle(index+1,j);
  }
  if (index == 255) {
    Serial.println(F("MOVING TO P1"));
  } else {
    Serial.print(F("SEGMENT P")); Serial.print(index+1);
    Serial.print(F(" -> P")); Serial.println(index+2);
  }
}

void updatePlayback() {
  if (!playing || paused) return;
  unsigned long now=millis();
  unsigned long duration = (segmentIndex == 255) ? APPROACH_MS : SEGMENT_MS;
  float t=(float)(now-segmentStart)/(float)duration;
  if (t>1.0f) t=1.0f;
  float k=smooth(t);
  for(byte j=0;j<NJ;++j)
    angleNow[j]=segmentFrom[j]+(segmentTo[j]-segmentFrom[j])*k;
  if (now-lastWrite>=UPDATE_MS || t>=1.0f) {
    outputPose();
    lastWrite=now;
  }
  if (t>=1.0f) {
    if (segmentIndex == 255) {
      startSegment(0,now);
      return;
    }
    if (segmentIndex>=3) {
      playing=false; paused=false;
      Serial.println(F("TRACE COMPLETE"));
      printPose();
    } else startSegment(segmentIndex+1,now);
  }
}

void cancel() {
  playing=false;
  paused=false;
  joystickEnabled=false;
  // Keep current commanded position; this is a SOFTWARE stop, not a hard E-stop.
  outputPose();
  Serial.println(F("CANCELLED; JOYSTICK OFF"));
}

void startTrace() {
  if (playing) { Serial.println(F("ALREADY RUNNING")); return; }
  for (byte p=0; p<5; ++p) {
    if (!validPoint(p)) { Serial.println(F("INVALID POINT")); return; }
  }
  // Move to P1 from the current *commanded* pose, then P1->P5.
  // There is no physical position feedback or pen-lift mechanism.
  joystickEnabled = false;
  playing = true;
  paused = false;
  startSegment(255, millis());
  Serial.println(F("TRACE START (joint-space prototype)"));
}

void command(char *s) {
  if (strlen(s)!=1) { Serial.println(F("UNKNOWN")); return; }
  char c=toupper((unsigned char)s[0]);
  if(c=='!') { cancel(); return; }
  if(c=='P') { printPose(); return; }
  if(c=='D') { listPoints(); return; }
  if(c=='C') { cancel(); return; }
  if(c=='T') { startTrace(); return; }
  if(c=='U') {
    if(playing) {
      if (!paused) { paused=true; pausedAt=millis(); Serial.println(F("PAUSED")); }
      else { segmentStart += millis()-pausedAt; paused=false; Serial.println(F("RESUMED")); }
    } else Serial.println(F("NOT RUNNING"));
    return;
  }
  if(c=='M') { joystickEnabled=false; Serial.println(F("JOYSTICK OFF")); return; }
  if(c=='J') {
    if(playing) { Serial.println(F("CANCEL FIRST")); return; }
    joystickEnabled=true; Serial.println(F("JOYSTICK ON")); return;
  }
  Serial.println(F("UNKNOWN"));
}

void readUsb() {
  while (Serial.available()) {
    char c=(char)Serial.read();
    if(c=='!') { inputLength=0; cancel(); continue; }
    if(c=='\n') {
      input[inputLength]='\0';
      if(inputLength) command(input);
      inputLength=0;
    } else if(c!='\r') {
      if(inputLength<sizeof(input)-1) input[inputLength++]=c;
      else { inputLength=0; Serial.println(F("OVERFLOW")); }
    }
  }
}

void updateJoystick() {
  if (!joystickEnabled || playing) return;
  unsigned long now=millis();
  if(now-lastJoy<JOYSTICK_MS) return;
  lastJoy=now;
  for(byte j=0;j<NJ;++j) {
    int adc=analogRead(JOY_PINS[j]);
    int delta=(adc>570)?1:((adc<450)?-1:0);
    int next=rounded(angleNow[j])+delta;
    if(delta && next>=MIN_DEG[j] && next<=MAX_DEG[j]) angleNow[j]=next;
  }
  outputPose();
}

void setup() {
  Serial.begin(9600);
  for(byte j=0;j<NJ;++j) {
    servo[j].write(rounded(angleNow[j]));
    servo[j].attach(SERVO_PINS[j]);
  }
  Serial.println(F("MeArm Task5 V2.5 - WIDER CENTERED W TRACE"));
  Serial.println(F("P pose, D preset points, J/M manual"));
  Serial.println(F("T move to P1 then P5; U pause/resume; C/! cancel"));
  Serial.println(F("TEST WITHOUT PAPER CONTACT; VERIFY EACH POSE / PATH"));
  listPoints();
}

void loop() {
  readUsb();
  updateJoystick();
  updatePlayback();
}
