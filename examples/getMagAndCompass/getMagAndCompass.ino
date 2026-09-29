#include "DFRobot_9DofODO.h"

DFRobot_9DofODO_I2C odo;

void setup()
{
  Serial.begin(115200);
  while (!odo.begin()) {
    delay(1000);
  }
}

void loop()
{
  int16_t mag[3];
  float compass;
  if (odo.getGeomagneticData(mag) == 0 && odo.getCompassDegree(&compass) == 0) {
    Serial.print(F("mag uT: "));
    Serial.print(mag[0]);
    Serial.print(F(", "));
    Serial.print(mag[1]);
    Serial.print(F(", "));
    Serial.println(mag[2]);
    Serial.print(F("compass: "));
    Serial.println(compass);
  }
  delay(200);
}
