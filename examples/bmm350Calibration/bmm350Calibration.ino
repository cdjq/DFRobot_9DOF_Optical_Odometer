/**
 * 在丝印坐标系下做硬铁校准：上电后缓慢转动模块约 20 秒，覆盖各方向。
 * 示例用这段时间的地磁最小/最大值估计硬铁偏移，软铁先写单位阵，并保存到模块 Flash。
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
  Serial.print(mag[0]);
  Serial.print(F(","));
  Serial.print(mag[1]);
  Serial.print(F(","));
  Serial.print(mag[2]);
  Serial.print(F(" compass "));
  Serial.println(compass);

  if (!saved && (millis() - t0) > 20000u) {
    float hard[3];
    float soft[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    hard[0] = 0.5f * (mx[0] + mn[0]);
    hard[1] = 0.5f * (mx[1] + mn[1]);
    hard[2] = 0.5f * (mx[2] + mn[2]);
    if (odo.setMagCalibration(hard, soft) == 0) {
      Serial.print(F("hard iron saved "));
      Serial.print(hard[0]);
      Serial.print(F(" "));
      Serial.print(hard[1]);
      Serial.print(F(" "));
      Serial.println(hard[2]);
      saved = true;
    }
  }
  delay(100);
}
