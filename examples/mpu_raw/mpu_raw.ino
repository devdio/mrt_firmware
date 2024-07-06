#include <BbBoard.h>
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

void setup()
{ 
  Serial.begin(9600);

  Wire.begin(14, 21);
  byte status = mpu.begin();
  mpu.calcOffsets(true, true); // gyro and accelero
  delay(200);
}


void loop()
{
  mpu.update(); 

  float ax = mpu.getAngleX();
  float ay = mpu.getAngleY();
  float az = mpu.getAngleZ();
  // ax = -255;
  // ay = -254;
  // az = 1023;

  Serial.print("ax=");
  Serial.print(ax);
  Serial.print(",ay=");
  Serial.print(ay);
  Serial.print(",az=");
  Serial.println(az);
  delay(500);
}
