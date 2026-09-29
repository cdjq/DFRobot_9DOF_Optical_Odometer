#include "DFRobot_9DofODO.h"

DFRobot_9DofODO_I2C odo;

void setup()
{
  Serial.begin(115200);
  while (!odo.begin()) {
    Serial.println(F("Waiting SEN0776..."));
    delay(1000);
  }
  Serial.println(F("SEN0776 ready"));
}

void loop()
{
  float tx;
  float ty;
  float roll;
  float pitch;
  float yaw;
  int16_t acc[3];
  int16_t gyr[3];

  if (odo.getTotalDisplacement(&tx, &ty) == 0 && odo.getEuler(&roll, &pitch, &yaw) == 0 &&
      odo.get6dofData(acc, gyr) == 0) {
    Serial.print(F("disp cm: "));
    Serial.print(tx);
    Serial.print(F(", "));
    Serial.println(ty);
    Serial.print(F("euler deg: "));
    Serial.print(roll);
    Serial.print(F(", "));
    Serial.print(pitch);
    Serial.print(F(", "));
    Serial.println(yaw);
  }
  delay(200);
}
