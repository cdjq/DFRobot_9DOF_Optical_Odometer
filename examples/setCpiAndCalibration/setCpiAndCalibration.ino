#include "DFRobot_9DofODO.h"

DFRobot_9DofODO_I2C odo;

void setup()
{
  Serial.begin(115200);
  while (!odo.begin()) {
    delay(1000);
  }
  odo.setCpi(50);
  if (odo.setCalibrationScale(1.0f, 1.0f) == 0) {
    Serial.println(F("cal scale saved"));
  }
}

void loop()
{
  delay(1000);
}
