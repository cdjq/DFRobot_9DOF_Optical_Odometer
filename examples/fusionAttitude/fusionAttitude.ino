/*!
 * @file fusionAttitude.ino
 * @brief 读取 Fusion 姿态：欧拉角、四元数、重力向量、陀螺零偏和罗盘航向。
 * @details 横滚、俯仰由重力倾角得到，平放时接近 0°。航向 0° = 丝印 +X 朝北。
 *          四元数顺序为 w、x、y、z。重力单位 g。陀螺零偏单位 °/s，模块静止约 3 秒后开始估计，
 *          并在变化足够大时写入 Flash。
 * @n 接线（I2C，地址 0x11）：UNO SDA=A4/SCL=A5；Mega2560 SDA=20/SCL=21；ESP32 SDA=GPIO21/SCL=GPIO22。
 * @copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      wxzed(xiao.wu@dfrobot.com)
 * @version     V1.0.0
 * @date        2026-10-14
 * @url         https://github.com/DFRobot/DFRobot_9DOF_Optical_Odometer
 */
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
    Serial.print(F("roll = "));
    Serial.print(roll, 2);
    Serial.print(F(" deg, pitch = "));
    Serial.print(pitch, 2);
    Serial.print(F(" deg, yaw = "));
    Serial.print(yaw, 2);
    Serial.print(F(" deg  |  compass = "));
    Serial.print(compass, 2);
    Serial.println(F(" deg"));
    Serial.print(F("quat  w = "));
    Serial.print(q[0], 4);
    Serial.print(F(", x = "));
    Serial.print(q[1], 4);
    Serial.print(F(", y = "));
    Serial.print(q[2], 4);
    Serial.print(F(", z = "));
    Serial.println(q[3], 4);
    Serial.print(F("gravity  X = "));
    Serial.print(gravity[0], 3);
    Serial.print(F(" g, Y = "));
    Serial.print(gravity[1], 3);
    Serial.print(F(" g, Z = "));
    Serial.print(gravity[2], 3);
    Serial.println(F(" g"));
    Serial.print(F("gyro bias  X = "));
    Serial.print(bias[0], 3);
    Serial.print(F(" dps, Y = "));
    Serial.print(bias[1], 3);
    Serial.print(F(" dps, Z = "));
    Serial.print(bias[2], 3);
    Serial.println(F(" dps"));
  }
  delay(200);
}
