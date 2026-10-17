
#include <SoftwareSerial.h>

SoftwareSerial remoteSerial(10, 11);

void setup() {
  Serial.begin(9600);
  remoteSerial.begin(9600);

  Serial.println("MeArm Task 3 Link Test");
  Serial.println("Press buttons 1, 2, 3, 4");
}

void loop() {
  while (remoteSerial.available()) {
    char c = remoteSerial.read();

    Serial.print("Received ASCII: ");
    Serial.print((int)(unsigned char)c);

    Serial.print("  Character: ");

    if (c >= 32 && c <= 126) {
      Serial.println(c);
    } else {
      Serial.println("(control character)");
    }
  }
}
