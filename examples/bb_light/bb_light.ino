void setup() {
  Serial.begin(115200);
  pinMode(9, INPUT);
  pinMode(7, INPUT);
}

void loop() {
  int val0 = analogRead(9);
  int val1 = analogRead(7);
  Serial.print(val0);
  Serial.print(",");
  Serial.println(val1);
}
