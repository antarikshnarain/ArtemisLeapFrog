void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(analogRead(A0));
  Serial.print(",");
  Serial.println(analogRead(A1));
  delay(1);
}
