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
  float roll;
  float pitch;
  float yaw;
  float compass;
  if (odo.getEuler(&roll, &pitch, &yaw) == 0 && odo.getCompassDegree(&compass) == 0) {
    Serial.print(F("R/P/Y: "));
    Serial.print(roll);
    Serial.print(F(" / "));
    Serial.print(pitch);
    Serial.print(F(" / "));
    Serial.print(yaw);
    Serial.print(F("  compass: "));
    Serial.println(compass);
  }
  delay(200);
}
