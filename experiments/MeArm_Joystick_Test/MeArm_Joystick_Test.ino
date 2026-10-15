void setup()
{
  Serial.begin(9600);

  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
}

void loop()
{
  Serial.print("A0=");
  Serial.print(analogRead(A0));

  Serial.print("  A1=");
  Serial.print(analogRead(A1));

  Serial.print("  A2=");
  Serial.print(analogRead(A2));

  Serial.print("  A3=");
  Serial.print(analogRead(A3));

  Serial.print("  D4=");
  Serial.print(digitalRead(4));

  Serial.print("  D5=");
  Serial.println(digitalRead(5));

  delay(100);
}