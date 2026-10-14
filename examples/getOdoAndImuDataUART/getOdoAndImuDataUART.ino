#include "DFRobot_9DofODO.h"

#if !defined(HAVE_HWSERIAL1)
#error This example needs a board with Serial1. Use getOdoAndImuData on UNO.
#endif

DFRobot_9DofODO_UART odo(&Serial1, 9600, 0x11);

void setup()
{
  Serial.begin(115200);
  while (!odo.begin()) {
    Serial.println(F("Waiting SEN0776 UART..."));
    delay(1000);
  }
  odo.resetTotalDisplacement();
}

void loop()
{
  float tx;
  float ty;
  float roll;
  float pitch;
  float yaw;
  if (odo.getTotalDisplacement(&tx, &ty) == 0 && odo.getEuler(&roll, &pitch, &yaw) == 0) {
    Serial.print(F("disp cm: "));
    Serial.print(tx, 4);
    Serial.print(F(", "));
    Serial.println(ty, 4);
    Serial.print(F("euler: "));
    Serial.print(roll);
    Serial.print(F(", "));
    Serial.print(pitch);
    Serial.print(F(", "));
    Serial.println(yaw);
  }
  delay(200);
}
