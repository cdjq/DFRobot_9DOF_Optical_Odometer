/*!
 * @file bmm350Calibration.ino
 * @brief 在丝印坐标系下估计硬铁偏移，并连同单位软铁矩阵写入模块 Flash。
 * @details 上电后缓慢转动模块约 20 秒，尽量覆盖各方向。示例用地磁最小/最大值的中点作为硬铁偏移，
 *          软铁先写单位阵。坐标系与 getGeomagneticData 相同，单位 µT。
 * @n 这只消除恒定偏移，不能校正软铁畸变。写成功后，后续地磁与罗盘读数都使用该校准。
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
  Serial.println(F("Rotate the module for 20 s"));
}

void loop()
{
  static float mn[3] = {1e6f, 1e6f, 1e6f};
  static float mx[3] = {-1e6f, -1e6f, -1e6f};
  static uint32_t t0 = 0;
  static bool saved = false;
  int16_t mag[3];
  float compass;
  uint8_t i;

  if (t0 == 0) {
    t0 = millis();
  }
  if (odo.getGeomagneticData(mag) != 0) {
    delay(100);
    return;
  }
  /* 记录 20 秒内三轴地磁的最小、最大值。 */
  for (i = 0; i < 3; i++) {
    float v = (float)mag[i];
    if (v < mn[i]) {
      mn[i] = v;
    }
    if (v > mx[i]) {
      mx[i] = v;
    }
  }
  odo.getCompassDegree(&compass);
  Serial.print(F("mag  X = "));
  Serial.print(mag[0]);
  Serial.print(F(" uT, Y = "));
  Serial.print(mag[1]);
  Serial.print(F(" uT, Z = "));
  Serial.print(mag[2]);
  Serial.print(F(" uT  |  compass = "));
  Serial.print(compass, 2);
  Serial.println(F(" deg"));

  if (!saved && (millis() - t0) > 20000u) {
    float hard[3];
    /* 软铁先用单位阵，只保存硬铁中点。 */
    float soft[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    hard[0] = 0.5f * (mx[0] + mn[0]);
    hard[1] = 0.5f * (mx[1] + mn[1]);
    hard[2] = 0.5f * (mx[2] + mn[2]);
    if (odo.setMagCalibration(hard, soft) == 0) {
      Serial.print(F("hard iron saved  X = "));
      Serial.print(hard[0], 1);
      Serial.print(F(" uT, Y = "));
      Serial.print(hard[1], 1);
      Serial.print(F(" uT, Z = "));
      Serial.print(hard[2], 1);
      Serial.println(F(" uT"));
      saved = true;
    }
  }
  delay(100);
}
