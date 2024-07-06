void setup() {
  pinMode(1, INPUT);
}

void loop() {
  int val = analogRead(1);
  Serial.println(val);
  delay(100);
}
