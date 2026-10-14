/*!
 * @file getMagAndCompass.ino
 * @brief 读取丝印坐标系下的三轴地磁与倾斜补偿罗盘航向。
 * @details 地磁单位为 µT（整数）。固件已重映射：BMM 的 X→模块 Y，Y→模块 X，Z 取反。
 *          罗盘 0° 表示丝印 +X 指向磁北，90° 为东，270° 为西。
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
  int16_t mag[3];
  float compass;
  if (odo.getGeomagneticData(mag) == 0 && odo.getCompassDegree(&compass) == 0) {
    Serial.print(F("mag  X = "));
    Serial.print(mag[0]);
    Serial.print(F(" uT, Y = "));
    Serial.print(mag[1]);
    Serial.print(F(" uT, Z = "));
    Serial.print(mag[2]);
    Serial.print(F(" uT  |  compass = "));
    Serial.print(compass, 2);
    Serial.println(F(" deg"));
  }
  delay(200);
}
