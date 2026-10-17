/* MeArm Task 5 V2.7 - UNO - preset V + 5-point teach/replay + controls
 * Servos: D9 Base, D8 Left, D7 Right, D6 Claw.
 * Joysticks: A0 Base, A1 Left, A3 Right, A2 Claw.
 * Second Arduino: TX -> THIS UNO D10 (SoftwareSerial RX), common GND.
 * Button ASCII: 1 preset V/start or cancel running; 2 save point;
 * 3 replay taught points; 4 pause/resume. Serial C always cancels.
 * IMPORTANT: Joint-angle interpolation (including curve mode) DOES NOT
 * guarantee a straight or planar pen-tip path. No tool-tip calibration here.
 * First test in mid-air; serial cancel is NOT an emergency power cutoff.
 */
#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <ctype.h>
#include <string.h>

const byte N = 4, NP = 5;
const byte SERVO_PIN[N] = {9,8,7,6};
const byte JOY_PIN[N] = {A0,A1,A3,A2};
const byte LO[N] = {0,45,0,30};
const byte HI[N] = {180,160,180,100};
const byte HOME[N] = {90,90,90,30};
// WIDE V: five samples along two branches; P3 is the bottom vertex.
// Approximate joint-space V only; pen tip will not necessarily draw straight lines.
// Keep claw at task-2 B grip angle (100 deg).
const byte PRESET[NP][N] = {
  {80,122,33,100}, {66,131,44,100}, {52,140,54,100},
  {38,131,44,100}, {24,122,33,100}
};
Servo servos[N];
SoftwareSerial remote(10,11); // RX, TX; D11 TX is unused
float pos[N] = {90,90,90,30};
byte taught[NP][N];
byte taughtCount=0;
bool joystickOn=true;
bool running=false, paused=false, curve=false;
bool useTaught=false;
byte leg=0;                 // 0: approach to P1; 1..4: P1->P2...P4->P5
unsigned long legStart=0, pausedAt=0, lastServo=0, lastJoy=0;
float fromPose[N];
char usb[32]; byte usbLen=0;
const unsigned long SERVO_PERIOD=20, JOY_PERIOD=100;
const unsigned long APPROACH_MS=3000, LINE_MS=1800;
const unsigned long CURVE_MS=2200;

byte point(byte i, byte j) {return useTaught ? taught[i][j] : PRESET[i][j];}
int asInt(float a) {return (int)(a+0.5f);}
void writeAll() {
  for(byte j=0;j<N;j++) servos[j].write(constrain(asInt(pos[j]),(int)LO[j],(int)HI[j]));
}
void posePrint() {
  Serial.print(F("POSE: "));
  for(byte j=0;j<N;j++) { Serial.print(pos[j],1); if(j<3) Serial.print(','); }
  Serial.println();
}
void pointsPrint() {
  Serial.println(F("PRESET V:"));
  for(byte i=0;i<NP;i++) {
    Serial.print(F("V"));Serial.print(i+1);Serial.print(F(": "));
    for(byte j=0;j<N;j++) {Serial.print(PRESET[i][j]);if(j<3)Serial.print(',');}
    Serial.println();
  }
  Serial.print(F("TAUGHT: "));Serial.println(taughtCount);
  for(byte i=0;i<taughtCount;i++) {
    Serial.print(F("P"));Serial.print(i+1);Serial.print(F(": "));
    for(byte j=0;j<N;j++) {Serial.print(taught[i][j]);if(j<3)Serial.print(',');}
    Serial.println();
  }
}
float ease5(float x) {
  if(x<=0)return 0;
  if(x>=1)return 1;
  return x*x*x*(10+x*(-15+6*x));
}
// Uniform cubic Catmull-Rom in JOINT space, not a geometrical pen-tip curve.
float catmull(float a,float b,float c,float d,float t) {
  return 0.5f*((2*b)+(-a+c)*t+(2*a-5*b+4*c-d)*t*t+(-a+3*b-3*c+d)*t*t*t);
}
void cancelRun() {
  running=false;paused=false;curve=false;joystickOn=false;
  writeAll();
  Serial.println(F("CANCELLED (hold current command; joystick OFF)"));
}
void togglePause() {
  if(!running){Serial.println(F("NOT RUNNING"));return;}
  if(!paused){ paused=true;pausedAt=millis();Serial.println(F("PAUSED")); }
  else {legStart+=(unsigned long)(millis()-pausedAt); paused=false;Serial.println(F("RESUMED"));}
}
void announceLeg() {
  if(leg==0) Serial.println(F("APPROACH P1 (not a pen-up move)"));
  else {Serial.print(F("SEGMENT P"));Serial.print(leg);Serial.print(F(" -> P"));Serial.println(leg+1);}
}
void initLeg(byte next) {
  leg=next;legStart=millis();lastServo=legStart-SERVO_PERIOD;
  for(byte j=0;j<N;j++)fromPose[j]=pos[j];
  announceLeg();
}
void beginRun(bool taughtMode,bool curveMode) {
  if(running) {Serial.println(F("ALREADY RUNNING"));return;}
  if(taughtMode && taughtCount!=5){Serial.println(F("NEED EXACTLY 5 TAUGHT POINTS"));return;}
  useTaught=taughtMode;curve=curveMode;joystickOn=false;
  for(byte i=0;i<NP;i++)for(byte j=0;j<N;j++){
    byte a=point(i,j);if(a<LO[j]||a>HI[j]){Serial.println(F("INVALID POINT ANGLE"));return;}
  }
  running=true;paused=false;
  initLeg(0);
  Serial.println(curve ? F("RUNNING 5-POINT JOINT CURVE") :
             (taughtMode ? F("RUNNING TAUGHT JOINT POLYLINE") : F("RUNNING PRESET V")));
}
void savePoint() {
  if(running){Serial.println(F("CANCEL FIRST"));return;}
  if(taughtCount>=NP){Serial.println(F("5 POINTS FULL; X TO CLEAR"));return;}
  byte idx=taughtCount;
  for(byte j=0;j<N;j++)taught[idx][j]=(byte)constrain(asInt(pos[j]),(int)LO[j],(int)HI[j]);
  taughtCount++;
  Serial.print(F("SAVED P"));Serial.println(taughtCount);posePrint();
}
void serviceMotion() {
  if(!running||paused)return;
  unsigned long now=millis();
  if(now-lastServo<SERVO_PERIOD)return;
  lastServo=now;
  unsigned long duration=(leg==0)?APPROACH_MS:(curve?CURVE_MS:LINE_MS);
  float t=(float)(now-legStart)/(float)duration;
  if(t>1)t=1;
  float u=ease5(t);
  for(byte j=0;j<N;j++){
    float target;
    if(leg==0)target=fromPose[j]+(point(0,j)-fromPose[j])*u;
    else if(curve) {
      byte k=leg-1;
      float a=point(k==0?0:k-1,j), b=point(k,j);
      float c=point(k+1,j), d=point(k+2>=NP?NP-1:k+2,j);
      target=catmull(a,b,c,d,u);
    } else target=fromPose[j]+(point(leg,j)-fromPose[j])*u;
    pos[j]=constrain(target,(float)LO[j],(float)HI[j]);
  }
  writeAll();
  if(t>=1) {
    // Snap endpoints to recorded point; avoid rounding accumulation.
    for(byte j=0;j<N;j++)pos[j]=point(leg==0?0:leg,j);
    writeAll();
    if(leg==4) {
      running=false;paused=false;
      Serial.println(F("TRACE COMPLETE"));posePrint();
    } else initLeg(leg+1);
  }
}
void dispatch(char c) {
  c=toupper((unsigned char)c);
  if(c=='P'){posePrint();return;}
  if(c=='D'){pointsPrint();return;}
  if(c=='C'||c=='!'){cancelRun();return;}
  if(c=='U'){togglePause();return;}
  if(c=='T'){beginRun(false,false);return;}
  if(c=='R'){beginRun(true,false);return;}
  if(c=='G'){beginRun(true,true);return;}
  if(c=='K'){savePoint();return;}
  if(c=='X') {
    if(running){Serial.println(F("CANCEL FIRST"));return;}
    taughtCount=0;Serial.println(F("TAUGHT POINTS CLEARED"));return;
  }
  if(c=='J') {
    if(running){Serial.println(F("CANCEL FIRST"));return;}
    joystickOn=true;Serial.println(F("JOYSTICK ON"));return;
  }
  if(c=='M'){joystickOn=false;Serial.println(F("JOYSTICK OFF"));return;}
  Serial.println(F("UNKNOWN COMMAND"));
}
void serviceUsb() {
  while(Serial.available()) {
    char c=(char)Serial.read();
    if(c=='!'){usbLen=0;dispatch('!');continue;}
    if(c=='\r'||c=='\n') {
      if(usbLen){usb[usbLen]=0;
        if(usbLen==1)dispatch(usb[0]);else Serial.println(F("SEND ONE LETTER"));
        usbLen=0;
      }
    } else if(usbLen<sizeof(usb)-1)usb[usbLen++]=c;
    else {usbLen=0;Serial.println(F("INPUT OVERFLOW"));}
  }
}
void serviceButtons() {
  while(remote.available()) {
    char c=(char)remote.read();
    if(c=='1') {
      if(running)cancelRun();else beginRun(false,false);
    } else if(c=='2')savePoint();
    else if(c=='3')beginRun(true,false);
    else if(c=='4')togglePause();
  }
}
void serviceJoy() {
  if(!joystickOn||running)return;
  unsigned long now=millis();if(now-lastJoy<JOY_PERIOD)return;
  lastJoy=now;
  bool changed=false;
  for(byte j=0;j<N;j++){
    int a=analogRead(JOY_PIN[j]);
    int d=(a>570)?1:((a<450)?-1:0);
    if(d){int next=asInt(pos[j])+d;
      if(next>=LO[j]&&next<=HI[j]){pos[j]=next;changed=true;}}
  }
  if(changed)writeAll();
}
void setup() {
  Serial.begin(9600);remote.begin(9600);
  for(byte j=0;j<N;j++){servos[j].write(HOME[j]);servos[j].attach(SERVO_PIN[j]);}
  Serial.println(F("MeArm TASK5 V2.7 | Uno | 9600"));
  Serial.println(F("T preset V; R taught 5-point polyline; G taught joint curve"));
  Serial.println(F("K save; X clear; D list; P pose; U pause/resume; C cancel"));
  Serial.println(F("J joystick ON; M joystick OFF"));
  Serial.println(F("BUTTONS: 1 preset V/cancel, 2 save, 3 replay, 4 pause/resume"));
  Serial.println(F("WARN: NO Cartesian IK / PEN-UP / PLANE CALIBRATION"));
  Serial.println(F("FIRST TEST WITH PEN CLEAR OF PAPER; FASTER MOTION"));
}
void loop(){serviceUsb();serviceButtons();serviceJoy();serviceMotion();}
