/**
 * BMM350 软/硬铁校准需在主机侧采集数据后写回固件 holding 区；
 * 完整流程请参考 sources/DFRobot_BMM350/examples/calibration。
 * 本示例仅演示读取校准后地磁与罗盘角。
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
  odo.getGeomagneticData(mag);
  odo.getCompassDegree(&compass);
  delay(500);
}
