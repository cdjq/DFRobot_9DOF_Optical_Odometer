/*!
 * @file DFRobot_9DofODO.h
 * @brief Gravity：九轴光学里程计（SEN0776）的 Arduino 主机库。
 * @details 模块内部由 CS32 读取 PAA5160 位移、BMI323、BMM350 并做 Fusion。主机通过 I2C 或
 *          UART（MODBUS RTU，DFRobot General RTU v1.0）访问同一套 16 位大端寄存器。
 * @n 默认 I2C 地址与 UART 从站地址均为 0x11；UART 固定 9600 8N1。本版固件不读 mode/addr 拨码。
 * @copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      wxzed(xiao.wu@dfrobot.com)
 * @version     V1.0.0
 * @date        2026-10-14
 * @url         https://github.com/DFRobot/DFRobot_9DOF_Optical_Odometer
 */

#ifndef __DFROBOT_9DOF_ODO_H  ///< 头文件包含保护
#define __DFROBOT_9DOF_ODO_H  ///< 头文件包含保护

#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_RTU.h>

/** @brief 产品 PID，输入寄存器 0x0000 */
#define DFROBOT_9DOF_PID 0x0776u  ///< SEN0776 产品号
/** @brief 默认 7 位 I2C 地址，也是 UART 从站地址 */
#define DFROBOT_9DOF_DEFAULT_I2C_ADDR 0x11u  ///< 默认从机地址 0x11

/**
 * @enum eSensorMode_t
 * @brief BMI323 工作模式，写入 holding 0x0107。
 */
typedef enum {
  eSleepMode = 0,             /**< 睡眠：加速度与陀螺停止输出 */
  eLowPowerMode = 1,          /**< 低功耗：降低采样率 */
  eNormalMode = 2,            /**< 正常模式，上电默认 */
  eHighPerformanceMode = 3    /**< 高性能：最高采样率 */
} eSensorMode_t;

/**
 * @enum eAccelRange_t
 * @brief 加速度量程，写入 holding 0x0108。数值与 BMI3_ACC_RANGE_* 一致。
 */
typedef enum {
  eAccelRange2G = 0,   /**< ±2 g */
  eAccelRange4G = 1,   /**< ±4 g */
  eAccelRange8G = 2,   /**< ±8 g，上电默认 */
  eAccelRange16G = 3   /**< ±16 g */
} eAccelRange_t;

/**
 * @enum eGyroRange_t
 * @brief 陀螺仪量程，写入 holding 0x0109。数值与 BMI3_GYR_RANGE_* 一致。
 */
typedef enum {
  eGyroRange125DPS = 0,    /**< ±125 °/s */
  eGyroRange250DPS = 1,    /**< ±250 °/s */
  eGyroRange500DPS = 2,    /**< ±500 °/s */
  eGyroRange1000DPS = 3,   /**< ±1000 °/s */
  eGyroRange2000DPS = 4    /**< ±2000 °/s，上电默认 */
} eGyroRange_t;

/*!
 * @class DFRobot_9DofODO_I2C
 * @brief I2C 模式下的九轴光学里程计对象。
 */
class DFRobot_9DofODO_I2C {
public:
  /*!
   * @fn DFRobot_9DofODO_I2C
   * @brief 构造函数，绑定 I2C 总线与模块从机地址。
   * @param wire I2C 总线对象，默认 &Wire
   * @param i2cAddr 模块 7 位地址，默认 0x11
   * @n 本接口不打开总线，须在 setup() 中调用 begin()。
   * @return 无
   */
  explicit DFRobot_9DofODO_I2C(TwoWire *wire = &Wire, uint8_t i2cAddr = DFROBOT_9DOF_DEFAULT_I2C_ADDR);

  /*!
   * @fn begin
   * @brief 初始化 I2C 并读取 PID，确认通讯正常。
   * @return bool true=读到产品号 0x0776；false=无设备或身份不符
   * @n 只适用于模块当前为 I2C 主机口的情况；UART 模式请用 DFRobot_9DofODO_UART。
   */
  bool begin(void);

  /*!
   * @fn getPAA_ID
   * @brief 读取 PAA5160E1-Q 芯片产品号。
   * @param productId 输出 16 位产品号
   * @return bool true=读取成功
   * @n 数据在输入寄存器 0x0006，正常应为 0x004F。
   */
  bool getPAA_ID(uint16_t &productId);

  /*!
   * @fn getPAA_RevisionID
   * @brief 读取 PAA5160E1-Q 芯片版本号。
   * @param revisionId 输出修订号，有效值在低 8 位
   * @return bool true=读取成功
   * @n 数据在输入寄存器 0x0007。
   */
  bool getPAA_RevisionID(uint16_t &revisionId);

  /*!
   * @fn getTotalDisplacement
   * @brief 获取模块持续累加的 X/Y 总位移。
   * @param totalX_cm 输出 X 轴总位移，单位 cm，不可为 NULL
   * @param totalY_cm 输出 Y 轴总位移，单位 cm，不可为 NULL
   * @return int 0=成功；-1=未 begin、空指针或通讯失败
   * @n 1. 模块不断电时数值持续累加，断电后丢失；校准系数和 CPI 保存在 Flash；
   * @n 2. 本接口不清零累加器。模块内部用整数 counts 累加，再按 CPI 与校准系数换成厘米。
   */
  int getTotalDisplacement(float *totalX_cm, float *totalY_cm);

  /*!
   * @fn resetTotalDisplacement
   * @brief 清零模块软件侧累加的总位移。
   * @return int 0=成功；-1=未 begin 或通讯失败
   * @n 只清 XY 位移累计，校准系数和 CPI 不受影响。
   */
  int resetTotalDisplacement(void);

  /*!
   * @fn setCalibrationScale
   * @brief 设置 X/Y 位移校准比例，掉电保存。
   * @param scaleX X 轴校准比例
   * @param scaleY Y 轴校准比例
   * @return int 0=写入成功；-1=未 begin、超出 0.5~2.0 或通讯失败
   * @n 1. 系数含义为「实际距离 / 模块读数」。例如实际 100 cm、模块输出 109 cm，则取 100.0/109.0；
   * @n 2. 允许范围 0.5~2.0，超出由固件拒绝；
   * @n 3. 写入 Flash 后，已累计的厘米数会按新系数重算。
   */
  int setCalibrationScale(float scaleX, float scaleY);

  /*!
   * @fn setCpi
   * @brief 设置光流 CPI 挡位，掉电保存。
   * @param cpi 挡位 1~200
   * @return int 0=设置成功；-1=未 begin、参数非法或通讯失败
   * @n 1. counts/英寸 = 挡位 × 100。setCpi(20) 为 2000，与输入寄存器 0x0011 的回读一致；
   * @n 2. 修改后已累计位移按新挡位重算；
   * @n 3. 高挡位分辨更细、噪声更大；低挡位噪声小，但微小位移会丢失。
   */
  int setCpi(uint16_t cpi);

  /*!
   * @fn readSqual
   * @brief 读取 SQUAL 表面特征质量，判断当前表面是否适合光流跟踪。
   * @param squal 输出原始寄存器值 0~255
   * @return int 0=读取成功；-1=空指针或通讯失败
   * @n 有效特征点数 = squal × 4。数值偏低表示表面特征不足，位移可信度下降。
   */
  int readSqual(uint8_t *squal);

  /*!
   * @fn setOrientation
   * @brief 设置光流坐标方向（交换 XY、翻转 X/Y），不掉电保存。
   * @param swapXY true=交换 X、Y；false=不交换
   * @param invertX true=翻转 X；false=不翻转
   * @param invertY true=翻转 Y；false=不翻转
   * @return bool true=配置已写入
   * @n 1. 固件把这三位写进 PAA5160 Orientation 寄存器 0x5B，芯片随后输出的 delta 已是变换后的；
   * @n 2. 同时开启交换和翻转时，芯片先交换 XY，再翻转方向；
   * @n 3. 重新上电后回到默认方向（不交换、不翻转）。
   */
  bool setOrientation(bool swapXY, bool invertX, bool invertY);

  /*!
   * @fn get6dofData
   * @brief 读取 BMI323 加速度与陀螺仪原始值。
   * @param accel 长度 3 的数组，单位为当前量程下的原始 LSB，不可为 NULL
   * @param gyro 长度 3 的数组，单位为当前量程下的原始 LSB，不可为 NULL
   * @return int 0=成功；-1=空指针或通讯失败
   * @n 上电默认 ±8 g、±2000 °/s。改量程后 LSB 含义随之变化，轴方向已按模块丝印重映射。
   */
  int get6dofData(int16_t *accel, int16_t *gyro);

  /*!
   * @fn getGeomagneticData
   * @brief 读取 BMM350 三轴地磁。
   * @param mag 长度 3 的数组，单位 µT 的整数，不可为 NULL
   * @return int 0=成功；-1=空指针或通讯失败
   * @n 固件已做丝印重映射：BMM 的 X→模块 Y，Y→模块 X，Z 取反。若写过 setMagCalibration，这里是校准后的值。
   */
  int getGeomagneticData(int16_t *mag);

  /*!
   * @fn getEuler
   * @brief 读取 Fusion 欧拉角，坐标系为模块丝印。
   * @param roll 绕丝印 +X 的横滚，单位度，不可为 NULL
   * @param pitch 绕丝印 +Y 的俯仰，单位度，不可为 NULL
   * @param yaw 航向 0~360°，不可为 NULL
   * @return int 0=成功；-1=空指针或通讯失败
   * @n 横滚、俯仰由重力倾角计算，平放时接近 0°。航向 0° 表示丝印 +X 指向磁北，90° 为东，270° 为西。
   */
  int getEuler(float *roll, float *pitch, float *yaw);

  /*!
   * @fn getCompassDegree
   * @brief 读取倾斜补偿后的罗盘航向。
   * @param compassDeg 输出 0~360°，不可为 NULL
   * @return int 0=成功；-1=空指针或通讯失败
   * @n 与 getEuler 的 yaw 同源：0°=丝印 +X 朝北。磁力计无效时该值不会更新为罗盘角。
   */
  int getCompassDegree(float *compassDeg);

  /*!
   * @fn setBMI_Mode
   * @brief 设置 BMI323 工作模式。
   * @param mode 见 eSensorMode_t
   * @return bool true=命令已写入
   * @n 睡眠停止加速度和陀螺输出；低功耗降低采样；正常与高性能提高采样。上电默认正常模式。
   */
  bool setBMI_Mode(eSensorMode_t mode);

  /*!
   * @fn reset_BMI
   * @brief 软复位 BMI323，并按当前模式与量程重新配置。
   * @return bool true=复位命令已写入
   */
  bool reset_BMI(void);

  /*!
   * @fn setAccelRange
   * @brief 设置加速度计量程。
   * @param range 见 eAccelRange_t
   * @return bool true=命令已写入
   * @n 量程越大，同样 LSB 对应的加速度越大。get6dofData 返回的仍是原始 LSB。
   */
  bool setAccelRange(eAccelRange_t range);

  /*!
   * @fn setGyroRange
   * @brief 设置陀螺仪量程。
   * @param range 见 eGyroRange_t
   * @return bool true=命令已写入
   */
  bool setGyroRange(eGyroRange_t range);

  /*!
   * @fn setBMM_Mode
   * @brief 设置 BMM350 电源模式。
   * @param opMode Bosch 电源模式值：0 挂起，1 常规，2 强制，3 强制快速
   * @return bool true=命令已写入
   * @n 常规模式持续输出地磁。挂起时电流最低，不转换数据。上电默认常规模式。
   */
  bool setBMM_Mode(uint8_t opMode);

  /*!
   * @fn setBMM_PresetMode
   * @brief 设置 BMM350 预设精度，并同时写入采样率。
   * @param presetMode 1=低功耗，2=普通，4=增强，3=高精度
   * @param rate ODR 枚举。6=25 Hz，为上电默认；数值与 BMM350_DATA_RATE_* 一致
   * @return bool true=两条命令都已写入
   * @n 预设决定片内平均次数：低功耗不平均，普通 2 次，增强 4 次，高精度 8 次。
   */
  bool setBMM_PresetMode(uint8_t presetMode, uint8_t rate);

  /*!
   * @fn setBMM_Rate
   * @brief 只修改 BMM350 采样率，不改预设。
   * @param rate ODR 枚举，与 setBMM_PresetMode 的 rate 相同
   * @return bool true=命令已写入
   */
  bool setBMM_Rate(uint8_t rate);

  /*!
   * @fn setBMM_MeasurementXYZ
   * @brief 使能或关闭 BMM350 的 X/Y/Z 测量。
   * @param enX 非 0=测量 X
   * @param enY 非 0=测量 Y
   * @param enZ 非 0=测量 Z
   * @return bool true=命令已写入
   * @n 关闭的轴数据不可信。上电默认三轴都测量。
   */
  bool setBMM_MeasurementXYZ(uint8_t enX, uint8_t enY, uint8_t enZ);

  /*!
   * @fn getDataReadyState
   * @brief 读取 BMM350 数据就绪标志。
   * @return bool true=有新样本；false=未就绪或通讯失败
   */
  bool getDataReadyState(void);

  /*!
   * @fn getBMM_Mode
   * @brief 读取模块回读的 BMM350 电源模式。
   * @param mode 输出模式值，含义同 setBMM_Mode
   * @return bool true=读取成功
   */
  bool getBMM_Mode(uint16_t *mode);

  /*!
   * @fn getBMM_Rate
   * @brief 读取模块回读的 BMM350 采样率枚举。
   * @param rate 输出 ODR 枚举
   * @return bool true=读取成功
   */
  bool getBMM_Rate(uint8_t *rate);

  /*!
   * @fn fusionAhrsGetQuaternion
   * @brief 读取当前姿态四元数。
   * @param q 长度 4，顺序为 w、x、y、z
   * @return int 0=成功；-1=空指针或通讯失败
   */
  int fusionAhrsGetQuaternion(float q[4]);

  /*!
   * @fn fusionAhrsGetGravity
   * @brief 读取传感器坐标系下的重力向量。
   * @param gravity 长度 3，单位 g
   * @return int 0=成功；-1=空指针或通讯失败
   */
  int fusionAhrsGetGravity(float gravity[3]);

  /*!
   * @fn fusionAhrsGetLinearAcceleration
   * @brief 读取传感器坐标系下已去除重力的线性加速度。
   * @param linearAccel 长度 3，单位 g
   * @return int 0=成功；-1=空指针或通讯失败
   */
  int fusionAhrsGetLinearAcceleration(float linearAccel[3]);

  /*!
   * @fn fusionAhrsGetEarthAcceleration
   * @brief 读取地球坐标系（NWU）下已去除重力的线性加速度。
   * @param earthAccel 长度 3，单位 g
   * @return int 0=成功；-1=空指针或通讯失败
   */
  int fusionAhrsGetEarthAcceleration(float earthAccel[3]);

  /*!
   * @fn fusionBiasGetOffset
   * @brief 读取陀螺仪运行时估计的零偏。
   * @param gyroBias 长度 3，单位 °/s
   * @return int 0=成功；-1=空指针或通讯失败
   * @n 模块静止约 3 秒后开始估计。变化超过 0.05 °/s 且间隔不少于 120 秒时写入 Flash，下次上电恢复。
   */
  int fusionBiasGetOffset(float gyroBias[3]);

  /*!
   * @fn setMagCalibration
   * @brief 写入硬铁与软铁校准，并保存到模块 Flash。
   * @param hardIron 长度 3，硬铁偏移，单位 µT，坐标系与 getGeomagneticData 相同
   * @param softIron 3×3 软铁矩阵，行优先
   * @return int 0=已保存；-1=空指针或通讯失败
   * @n 校准作用在丝印坐标系：先减硬铁，再乘软铁。未写过时软铁为单位阵、硬铁为 0。
   */
  int setMagCalibration(const float hardIron[3], const float softIron[3][3]);

  /*!
   * @fn setHostModeSwitch
   * @brief 预留的主机口切换入口。本版为空实现。
   * @n mode 脚与 SWD 复用，本版固件不在运行中切换 I2C/UART。主机按接线选择 I2C 或 UART 类。
   */
  void setHostModeSwitch(void) {}

  /*!
   * @fn setI2cAddressSwitch
   * @brief 预留的 I2C 地址切换入口。本版为空实现。
   * @n addr 脚与 SWD 复用，本版固件固定地址 0x11。
   */
  void setI2cAddressSwitch(void) {}

private:
  TwoWire *_wire;
  uint8_t _addr;
  bool _ready;

  bool readReg16(uint16_t reg, uint16_t *value);
  bool writeReg16(uint16_t reg, uint16_t value);
  bool readFloat32(uint16_t regHi, float *out);
  bool writeFloat32(uint16_t regHi, float value);
};

/*!
 * @class DFRobot_9DofODO_UART
 * @brief UART（MODBUS RTU）模式下的九轴光学里程计对象，传输层复用 DFRobot_RTU。
 */
class DFRobot_9DofODO_UART : public DFRobot_RTU {
public:
  /*!
   * @fn DFRobot_9DofODO_UART
   * @brief 构造函数，绑定硬件串口、波特率与从站地址。
   * @param serial UART 对象。带 Serial1 的板子默认 &Serial1；没有 Serial1 的板子必须由用户传入
   * @param baudRate 波特率，模块固件固定 9600，此参数只做兼容
   * @param unitId MODBUS 从站地址，默认 0x11
   * @n 串口格式固定 8N1。CRC、组帧和超时由 DFRobot_RTU 完成。
   * @return 无
   */
#if defined(HAVE_HWSERIAL1)
  explicit DFRobot_9DofODO_UART(HardwareSerial *serial = &Serial1,
                                uint32_t baudRate = 9600,
                                uint8_t unitId = DFROBOT_9DOF_DEFAULT_I2C_ADDR);
#else
  explicit DFRobot_9DofODO_UART(HardwareSerial *serial,
                                uint32_t baudRate = 9600,
                                uint8_t unitId = DFROBOT_9DOF_DEFAULT_I2C_ADDR);
#endif

  /*!
   * @fn begin
   * @brief 打开串口并读取 PID，确认通讯正常。
   * @return bool true=读到产品号 0x0776；false=无应答或身份不符
   * @n 只适用于模块当前为 UART 主机口的情况。波特率固定 9600。
   */
  bool begin(void);

  /*!
   * @fn getPAA_ID
   * @brief 读取 PAA5160E1-Q 芯片产品号。
   * @param productId 输出 16 位产品号
   * @return bool true=读取成功
   * @n 输入寄存器 0x0006，正常应为 0x004F。语义与 I2C 类相同。
   */
  bool getPAA_ID(uint16_t &productId);

  /*!
   * @fn getPAA_RevisionID
   * @brief 读取 PAA5160E1-Q 芯片版本号。
   * @param revisionId 输出修订号，有效值在低 8 位
   * @return bool true=读取成功
   */
  bool getPAA_RevisionID(uint16_t &revisionId);

  /*!
   * @fn getTotalDisplacement
   * @brief 获取 X/Y 总位移，单位 cm。
   * @param totalX_cm 输出 X，不可为 NULL
   * @param totalY_cm 输出 Y，不可为 NULL
   * @return int 0=成功；-1=失败
   * @n 不清零累加器。说明同 I2C 类 getTotalDisplacement。
   */
  int getTotalDisplacement(float *totalX_cm, float *totalY_cm);

  /*!
   * @fn resetTotalDisplacement
   * @brief 清零总位移，不影响校准系数和 CPI。
   * @return int 0=成功；-1=失败
   */
  int resetTotalDisplacement(void);

  /*!
   * @fn setCalibrationScale
   * @brief 设置 X/Y 校准比例 0.5~2.0，掉电保存，并重算已累计位移。
   * @param scaleX X 轴比例
   * @param scaleY Y 轴比例
   * @return int 0=成功；-1=参数非法或通讯失败
   */
  int setCalibrationScale(float scaleX, float scaleY);

  /*!
   * @fn setCpi
   * @brief 设置 CPI 挡位 1~200，掉电保存。setCpi(20)=2000 counts/英寸。
   * @param cpi 挡位
   * @return int 0=成功；-1=参数非法或通讯失败
   */
  int setCpi(uint16_t cpi);

  /*!
   * @fn readSqual
   * @brief 读取 SQUAL 原始值 0~255。有效特征点数 = 值 × 4。
   * @param squal 输出
   * @return int 0=成功；-1=失败
   */
  int readSqual(uint8_t *squal);

  /*!
   * @fn setOrientation
   * @brief 设置光流方向，不掉电保存。语义同 I2C 类 setOrientation。
   * @param swapXY true=交换 XY
   * @param invertX true=翻转 X
   * @param invertY true=翻转 Y
   * @return bool true=已写入
   */
  bool setOrientation(bool swapXY, bool invertX, bool invertY);

  /*!
   * @fn get6dofData
   * @brief 读取 BMI323 原始加速度与陀螺仪 LSB。
   * @param accel 长度 3，不可为 NULL
   * @param gyro 长度 3，不可为 NULL
   * @return int 0=成功；-1=失败
   */
  int get6dofData(int16_t *accel, int16_t *gyro);

  /*!
   * @fn getGeomagneticData
   * @brief 读取丝印坐标系下的地磁，单位 µT（整数）。
   * @param mag 长度 3，不可为 NULL
   * @return int 0=成功；-1=失败
   */
  int getGeomagneticData(int16_t *mag);

  /*!
   * @fn getEuler
   * @brief 读取丝印系 roll/pitch/yaw，单位度。yaw 为 0~360°，0=丝印 +X 朝北。
   * @param roll 横滚，不可为 NULL
   * @param pitch 俯仰，不可为 NULL
   * @param yaw 航向，不可为 NULL
   * @return int 0=成功；-1=失败
   */
  int getEuler(float *roll, float *pitch, float *yaw);

  /*!
   * @fn getCompassDegree
   * @brief 读取罗盘航向 0~360°，与 yaw 同源。
   * @param compassDeg 输出，不可为 NULL
   * @return int 0=成功；-1=失败
   */
  int getCompassDegree(float *compassDeg);

  /*!
   * @fn setBMI_Mode
   * @brief 设置 BMI323 工作模式，见 eSensorMode_t。
   * @param mode 模式
   * @return bool true=已写入
   */
  bool setBMI_Mode(eSensorMode_t mode);

  /*!
   * @fn reset_BMI
   * @brief 软复位 BMI323 并按当前模式、量程重配。
   * @return bool true=已写入
   */
  bool reset_BMI(void);

  /*!
   * @fn setAccelRange
   * @brief 设置加速度量程，见 eAccelRange_t。
   * @param range 量程
   * @return bool true=已写入
   */
  bool setAccelRange(eAccelRange_t range);

  /*!
   * @fn setGyroRange
   * @brief 设置陀螺仪量程，见 eGyroRange_t。
   * @param range 量程
   * @return bool true=已写入
   */
  bool setGyroRange(eGyroRange_t range);

  /*!
   * @fn setBMM_Mode
   * @brief 设置 BMM350 电源模式。0 挂起，1 常规，2 强制，3 强制快速。
   * @param opMode 模式值
   * @return bool true=已写入
   */
  bool setBMM_Mode(uint8_t opMode);

  /*!
   * @fn setBMM_PresetMode
   * @brief 设置 BMM350 预设与采样率。预设 1/2/4/3 对应低功耗、普通、增强、高精度。
   * @param presetMode 预设
   * @param rate ODR 枚举，6=25 Hz
   * @return bool true=已写入
   */
  bool setBMM_PresetMode(uint8_t presetMode, uint8_t rate);

  /*!
   * @fn setBMM_Rate
   * @brief 只修改 BMM350 采样率。
   * @param rate ODR 枚举
   * @return bool true=已写入
   */
  bool setBMM_Rate(uint8_t rate);

  /*!
   * @fn setBMM_MeasurementXYZ
   * @brief 开关 BMM350 三轴测量。非 0 为使能。
   * @param enX X 轴
   * @param enY Y 轴
   * @param enZ Z 轴
   * @return bool true=已写入
   */
  bool setBMM_MeasurementXYZ(uint8_t enX, uint8_t enY, uint8_t enZ);

  /*!
   * @fn getDataReadyState
   * @brief 读取 BMM350 是否有新样本。
   * @return bool true=数据就绪
   */
  bool getDataReadyState(void);

  /*!
   * @fn getBMM_Mode
   * @brief 回读 BMM350 电源模式。
   * @param mode 输出
   * @return bool true=成功
   */
  bool getBMM_Mode(uint16_t *mode);

  /*!
   * @fn getBMM_Rate
   * @brief 回读 BMM350 采样率枚举。
   * @param rate 输出
   * @return bool true=成功
   */
  bool getBMM_Rate(uint8_t *rate);

  /*!
   * @fn fusionAhrsGetQuaternion
   * @brief 读取姿态四元数，顺序 w、x、y、z。
   * @param q 长度 4
   * @return int 0=成功；-1=失败
   */
  int fusionAhrsGetQuaternion(float q[4]);

  /*!
   * @fn fusionAhrsGetGravity
   * @brief 读取传感器系重力，单位 g。
   * @param gravity 长度 3
   * @return int 0=成功；-1=失败
   */
  int fusionAhrsGetGravity(float gravity[3]);

  /*!
   * @fn fusionAhrsGetLinearAcceleration
   * @brief 读取传感器系线性加速度（已去重力），单位 g。
   * @param linearAccel 长度 3
   * @return int 0=成功；-1=失败
   */
  int fusionAhrsGetLinearAcceleration(float linearAccel[3]);

  /*!
   * @fn fusionAhrsGetEarthAcceleration
   * @brief 读取地球系（NWU）线性加速度（已去重力），单位 g。
   * @param earthAccel 长度 3
   * @return int 0=成功；-1=失败
   */
  int fusionAhrsGetEarthAcceleration(float earthAccel[3]);

  /*!
   * @fn fusionBiasGetOffset
   * @brief 读取陀螺零偏，单位 °/s。说明同 I2C 类。
   * @param gyroBias 长度 3
   * @return int 0=成功；-1=失败
   */
  int fusionBiasGetOffset(float gyroBias[3]);

  /*!
   * @fn setMagCalibration
   * @brief 写入硬铁（µT）与 3×3 软铁并保存到 Flash。坐标系与 getGeomagneticData 相同。
   * @param hardIron 长度 3
   * @param softIron 3×3，行优先
   * @return int 0=成功；-1=失败
   */
  int setMagCalibration(const float hardIron[3], const float softIron[3][3]);

  /*!
   * @fn setHostModeSwitch
   * @brief 预留。本版为空实现，不切换 I2C/UART。
   */
  void setHostModeSwitch(void) {}

  /*!
   * @fn setI2cAddressSwitch
   * @brief 预留。本版为空实现，地址固定 0x11。
   */
  void setI2cAddressSwitch(void) {}

private:
  HardwareSerial *_serial;
  uint32_t _baud;
  uint8_t _addr;
  bool _ready;

  bool readReg16Input(uint16_t reg, uint16_t *value);
  bool writeReg16Holding(uint16_t reg, uint16_t value);
  bool readFloat32Input(uint16_t regHi, float *out);
  bool writeFloat32Holding(uint16_t regHi, float value);
};

#endif
