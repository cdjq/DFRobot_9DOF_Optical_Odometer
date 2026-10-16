# DFRobot_9DOF_Optical_Odometer

- [中文版](./README_CN.md)

Host library for the Gravity 9-DOF optical odometer. The module reads optical flow, the 6-axis IMU and the magnetometer, and runs Fusion on board. The host reads the same data over I2C or UART.

## Product

SKU: SEN0776 (Gravity)

## Table of Contents

- [Summary](#summary)
- [Installation](#installation)
- [Methods](#methods)
- [Compatibility](#compatibility)
- [History](#history)
- [Credits](#credits)

## Summary

Displacement is in centimeters and is lost at power-off. `begin()` succeeds only when the product id is 0x0776. Use the compass for heading, and read it while the module is flat. yaw stops using the magnetometer once roll or pitch exceeds about 55°.

- I2C: mode switch open. Address 0x11 when the addr switch is closed, 0x12 when it is open.
- UART: mode switch closed. 9600 8N1, unit id equal to the I2C address. Depends on DFRobot_RTU.
- The switches are read once at power-on and cannot be changed in software. The host port is ready about 1 second after power-on. Power-cycle after changing a switch.
- Gravity wire order: red = VCC, black = GND, blue = SDA (module TX in UART), yellow = SCL (module RX in UART). VCC is 3.3 V or 5 V. Cross the UART wires. The Mega example uses Serial3: TX3=14 to module yellow/RX, RX3=15 to module blue/TX. UNO has no Serial1; use I2C.

## Installation

Install [DFRobot_RTU](https://github.com/DFRobot/DFRobot_RTU) first. Download this library into the Arduino `libraries` directory, then open a sketch in `examples`.

## Methods

```cpp
  /**
   * @brief Open the bus and check product id 0x0776
   * @n Valid only for the interface selected by the mode switch. It fails during the first second; the examples retry
   * @return true on success, false when there is no reply or the id does not match
   */
  bool begin(void);

  /**
   * @brief Read the PAA5160E1-Q product id
   * @n Normally 0x004F
   * @return true on success, false on failure
   */
  bool getPAA_ID(uint16_t &productId);

  /**
   * @brief Read the PAA5160E1-Q revision. The value is in the low 8 bits
   * @return true on success, false on failure
   */
  bool getPAA_RevisionID(uint16_t &revisionId);

  /**
   * @brief Read the accumulated X/Y displacement in centimeters
   * @n It keeps accumulating while powered and is lost at power-off. This call does not clear it
   * @return 0 on success, -1 on failure
   */
  int getTotalDisplacement(float *totalX_cm, float *totalY_cm);

  /**
   * @brief Clear the accumulated displacement
   * @n The calibration scale and CPI are unchanged
   * @return 0 on success, -1 on failure
   */
  int resetTotalDisplacement(void);

  /**
   * @brief Set the X/Y displacement scale. Stored in flash
   * @n Scale = actual distance / module reading, limited to 0.5~2.0. Stored centimeters are recomputed
   * @return 0 on success, -1 when out of range or on failure
   */
  int setCalibrationScale(float scaleX, float scaleY);

  /**
   * @brief Set the optical-flow CPI step. Stored in flash
   * @n Steps 1~200. Counts per inch = step × 100. Accumulated displacement is recomputed
   * @return 0 on success, -1 when the argument is invalid or on failure
   */
  int setCpi(uint16_t cpi);

  /**
   * @brief Read SQUAL, raw value 0~255
   * @n Feature points = squal × 4. A low value means the surface is a poor tracking target
   * @return 0 on success, -1 on failure
   */
  int readSqual(uint8_t *squal);

  /**
   * @brief Set optical-flow orientation: swap XY, invert X, invert Y
   * @n Not stored in flash. Swap is applied before invert
   * @return true when written, false on failure
   */
  bool setOrientation(bool swapXY, bool invertX, bool invertY);

  /**
   * @brief Read raw accelerometer and gyroscope LSBs
   * @n Power-on defaults are ±8 g and ±2000 °/s. Axes are remapped to the silkscreen
   * @return 0 on success, -1 on failure
   */
  int get6dofData(int16_t *accel, int16_t *gyro);

  /**
   * @brief Read the three-axis magnetic field in µT
   * @n Already remapped to the silkscreen. After calibration this is the calibrated field
   * @return 0 on success, -1 on failure
   */
  int getGeomagneticData(int16_t *mag);

  /**
   * @brief Read Fusion Euler angles in degrees
   * @n roll and pitch are near 0° when the module is flat. yaw is not a heading above about 55° of tilt
   * @return 0 on success, -1 on failure
   */
  int getEuler(float *roll, float *pitch, float *yaw);

  /**
   * @brief Read the compass heading, 0~360°. This is the heading to use
   * @n 0° means silkscreen +X points north, 90° means east. Read it while the module is flat
   * @return 0 on success, -1 on failure
   */
  int getCompassDegree(float *compassDeg);

  /**
   * @brief Set the BMI323 mode
   * @n 0 sleep, 1 low power, 2 normal (default), 3 high performance
   * @return true when written, false on failure
   */
  bool setBMI_Mode(eSensorMode_t mode);

  /**
   * @brief Soft-reset the BMI323 and apply the current mode and ranges again
   * @return true when written, false on failure
   */
  bool reset_BMI(void);

  /**
   * @brief Set the accelerometer range
   * @n 0 = ±2 g, 1 = ±4 g, 2 = ±8 g (default), 3 = ±16 g
   * @return true when written, false on failure
   */
  bool setAccelRange(eAccelRange_t range);

  /**
   * @brief Set the gyroscope range
   * @n 0 = ±125, 1 = ±250, 2 = ±500, 3 = ±1000, 4 = ±2000 °/s (default)
   * @return true when written, false on failure
   */
  bool setGyroRange(eGyroRange_t range);

  /**
   * @brief Set the BMM350 power mode
   * @n 0 suspend, 1 normal (default), 2 forced, 3 forced fast
   * @return true when written, false on failure
   */
  bool setBMM_Mode(uint8_t opMode);

  /**
   * @brief Set the BMM350 accuracy preset and output rate
   * @n presetMode: 1 low power, 2 regular, 4 enhanced, 3 high accuracy. rate 6 is 25 Hz
   * @return true when written, false on failure
   */
  bool setBMM_PresetMode(uint8_t presetMode, uint8_t rate);

  /**
   * @brief Change only the BMM350 output rate
   * @return true when written, false on failure
   */
  bool setBMM_Rate(uint8_t rate);

  /**
   * @brief Enable or disable BMM350 X/Y/Z measurement. Non-zero enables an axis
   * @n All three axes are enabled at power-on
   * @return true when written, false on failure
   */
  bool setBMM_MeasurementXYZ(uint8_t enX, uint8_t enY, uint8_t enZ);

  /**
   * @brief Read whether the BMM350 has a new sample
   * @return true when a new sample is ready, false otherwise
   */
  bool getDataReadyState(void);

  /**
   * @brief Read the BMM350 power mode. Values match setBMM_Mode
   * @return true on success, false on failure
   */
  bool getBMM_Mode(uint16_t *mode);

  /**
   * @brief Read the BMM350 output-rate enum
   * @return true on success, false on failure
   */
  bool getBMM_Rate(uint8_t *rate);

  /**
   * @brief Read the attitude quaternion, order w, x, y, z
   * @return 0 on success, -1 on failure
   */
  int fusionAhrsGetQuaternion(float q[4]);

  /**
   * @brief Read the gravity vector in g
   * @return 0 on success, -1 on failure
   */
  int fusionAhrsGetGravity(float gravity[3]);

  /**
   * @brief Read gravity-free linear acceleration in the sensor frame, in g
   * @return 0 on success, -1 on failure
   */
  int fusionAhrsGetLinearAcceleration(float linearAccel[3]);

  /**
   * @brief Read gravity-free linear acceleration in the north-west-up frame, in g
   * @return 0 on success, -1 on failure
   */
  int fusionAhrsGetEarthAcceleration(float earthAccel[3]);

  /**
   * @brief Read the gyro bias in °/s
   * @n Estimation starts after about 3 seconds of stillness. A change above 0.05 °/s is stored after 120 seconds
   * @return 0 on success, -1 on failure
   */
  int fusionBiasGetOffset(float gyroBias[3]);

  /**
   * @brief Store the hard-iron offset (µT) and the 3×3 soft-iron matrix in flash
   * @n Hard iron is subtracted first, then the soft-iron matrix is applied. Defaults are zero and identity
   * @return 0 when stored, -1 on failure
   */
  int setMagCalibration(const float hardIron[3], const float softIron[3][3]);

  /**
   * @brief Reserved. Empty. The mode comes from the mode switch
   */
  void setHostModeSwitch(void);

  /**
   * @brief Reserved. Empty. The address comes from the addr switch
   */
  void setI2cAddressSwitch(void);

  /**
   * @brief I2C constructor. i2cAddr defaults to 0x11; pass 0x12 when the addr switch is open
   * @n Does not start the bus. Call begin() from setup()
   */
  explicit DFRobot_9DofODO_I2C(TwoWire *wire = &Wire, uint8_t i2cAddr = 0x11);
  bool begin(void);

  /**
   * @brief UART constructor. Baud is fixed at 9600. unitId defaults to 0x11
   * @n Defaults to &Serial1 when the board has it. The Mega example passes &Serial3. Frame format is 8N1
   */
  explicit DFRobot_9DofODO_UART(HardwareSerial *serial = &Serial1, uint32_t baudRate = 9600, uint8_t unitId = 0x11);
  bool begin(void);
```

## Compatibility

| MCU | Work Well | Work Wrong | Untested | Remarks |
| --- | :--: | :----: | :----: | --- |
| Arduino UNO |  |  | √ | I2C only, no Serial1 |
| Arduino Mega2560 | √ |  |  | I2C and UART |
| Arduino Leonardo |  |  | √ |  |
| ESP32 |  |  | √ |  |

## History

- 2026/10/14 - V1.0.0

## Credits

Written by wxzed(xiao.wu@dfrobot.com), 2026. (Welcome to our [website](https://www.dfrobot.com/))
