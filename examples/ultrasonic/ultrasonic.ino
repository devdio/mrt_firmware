#include <BbBoard.h>

int trig = P0;
int echo = P1;

void setup()
{
  pinMode(trig, OUTPUT); //trig
  pinMode(echo, INPUT);  //echo

  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  delay(200);
  Serial.begin(9600);
}

void loop()
{
  
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH);
  int distance = duration * 0.017;
  distance = constrain(distance, 0, 250);  // 0 ~ 250cm
  Serial.println(distance);
}
