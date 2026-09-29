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
  /* 模块侧累计位移在 RAM 中；UNO 复位而模块仍供电时会保留旧值，示例先清零便于观测。 */
  if (odo.resetTotalDisplacement() == 0) {
    Serial.println(F("total displacement reset"));
  }
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
  uint8_t squal;

  if (odo.getTotalDisplacement(&tx, &ty) == 0 && odo.getEuler(&roll, &pitch, &yaw) == 0 &&
      odo.get6dofData(acc, gyr) == 0 && odo.readSqual(&squal) == 0) {
    Serial.print(F("disp cm: "));
    Serial.print(tx, 4);
    Serial.print(F(", "));
    Serial.println(ty, 4);
    Serial.print(F("squal: "));
    Serial.println(squal);
    Serial.print(F("euler deg: "));
    Serial.print(roll);
    Serial.print(F(", "));
    Serial.print(pitch);
    Serial.print(F(", "));
    Serial.println(yaw);
  }
  delay(200);
}
