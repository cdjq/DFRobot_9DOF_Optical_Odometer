# DFRobot_9DOF_Optical_Odometer
- [中文版](./README_CN.md)

Arduino host library for **SEN0776** Gravity 9-DOF optical odometer: optical flow displacement, BMI323 6-axis IMU, BMM350 magnetometer, and on-board Fusion attitude over **I2C** (default address **0x11**).

## Installation

Download or clone into your Arduino `libraries` folder, or use the Library Manager when published.

## Connection (I2C)

| Module | Arduino UNO |
|--------|-------------|
| SDA    | A4          |
| SCL    | A5          |
| VCC    | 5V          |
| GND    | GND         |

Open an example under `examples/`, set serial monitor to **115200**.

## Examples

| Sketch | Purpose |
|--------|---------|
| getOdoAndImuData | Displacement, Euler angles, 6-axis raw |
| getMagAndCompass | Magnetometer and compass heading |
| setCpiAndCalibration | CPI and scale calibration |
| bmm350Calibration | Placeholder for BMM350 soft-iron flow |
| fusionAttitude | Fusion-related reads via product registers |

## License

MIT — see product documentation.
