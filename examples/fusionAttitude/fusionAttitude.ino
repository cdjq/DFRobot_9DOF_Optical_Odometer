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
  float q[4];
  float gravity[3];
  float bias[3];
  if (odo.getEuler(&roll, &pitch, &yaw) == 0 && odo.getCompassDegree(&compass) == 0 &&
      odo.fusionAhrsGetQuaternion(q) == 0 && odo.fusionAhrsGetGravity(gravity) == 0 &&
      odo.fusionBiasGetOffset(bias) == 0) {
    Serial.print(F("R/P/Y: "));
    Serial.print(roll);
    Serial.print(F(" / "));
    Serial.print(pitch);
    Serial.print(F(" / "));
    Serial.println(yaw);
    Serial.print(F("quat wxyz: "));
    Serial.print(q[0], 4);
    Serial.print(F(" "));
    Serial.print(q[1], 4);
    Serial.print(F(" "));
    Serial.print(q[2], 4);
    Serial.print(F(" "));
    Serial.println(q[3], 4);
    Serial.print(F("gravity g: "));
    Serial.print(gravity[0], 3);
    Serial.print(F(" "));
    Serial.print(gravity[1], 3);
    Serial.print(F(" "));
    Serial.println(gravity[2], 3);
    Serial.print(F("gyro bias dps: "));
    Serial.print(bias[0], 3);
    Serial.print(F(" "));
    Serial.print(bias[1], 3);
    Serial.print(F(" "));
    Serial.println(bias[2], 3);
    Serial.print(F("compass: "));
    Serial.println(compass);
  }
  delay(200);
}
