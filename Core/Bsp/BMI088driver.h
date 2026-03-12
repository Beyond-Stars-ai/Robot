#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

#include <stdint.h>
#include "main.h"  // 包含 HAL 库等基础定义

// 温度计算参数（来自 BMI088 数据手册）
#define BMI088_TEMP_FACTOR 0.125f   // 温度分辨率：0.125°C/LSB
#define BMI088_TEMP_OFFSET 23.0f    // 温度偏移：0 LSB 对应 23°C

// 配置寄存器数量（用于初始化校验）
#define BMI088_WRITE_ACCEL_REG_NUM 6   // 加速度计需配置的寄存器个数
#define BMI088_WRITE_GYRO_REG_NUM  6   // 陀螺仪需配置的寄存器个数

// 数据就绪状态位（用于多传感器同步）
#define BMI088_GYRO_DATA_READY_BIT        0   // 陀螺仪数据就绪标志位
#define BMI088_ACCEL_DATA_READY_BIT       1   // 加速度计数据就绪标志位
#define BMI088_ACCEL_TEMP_DATA_READY_BIT  2   // 加速度计温度数据就绪标志位

// 延时参数（单位：毫秒 / 微秒）
#define BMI088_LONG_DELAY_TIME        80      // 软件复位后所需长延时（ms）
#define BMI088_COM_WAIT_SENSOR_TIME   150     // 寄存器写入后等待传感器响应时间（us）

// I²C 地址（实际使用 SPI，此处可能为遗留定义或备用）
// 注意：BMI088 通常通过 SPI 通信，I²C 地址在此驱动中未使用
#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)  // 加速度计 I²C 地址（7-bit 左移1位）
#define BMI088_GYRO_IIC_ADDRESSE  (0x68 << 1)  // 陀螺仪 I²C 地址

// 量程选择（取消注释以切换量程，当前启用 3G / 2000°/s）
// 注意：灵敏度（SEN）必须与量程匹配！
#define BMI088_ACCEL_RANGE_3G
// #define BMI088_ACCEL_RANGE_6G
// #define BMI088_ACCEL_RANGE_12G
// #define BMI088_ACCEL_RANGE_24G

#define BMI088_GYRO_RANGE_2000
// #define BMI088_GYRO_RANGE_1000
// #define BMI088_GYRO_RANGE_500
// #define BMI088_GYRO_RANGE_250
// #define BMI088_GYRO_RANGE_125

 
// 灵敏度系数（单位转换：LSB → 物理单位）
// 根据所选量程自动使用对应的 SEN 值
// - 加速度单位：m/s² （注意：部分系统用 g，需确认）
// - 角速度单位：rad/s （若需 °/s，需额外转换）
 

// 加速度计灵敏度（m/s² per LSB）
#define BMI088_ACCEL_3G_SEN   0.0008974358974f      // ±3g 量程
#define BMI088_ACCEL_6G_SEN   0.00179443359375f     // ±6g
#define BMI088_ACCEL_12G_SEN  0.0035888671875f      // ±12g
#define BMI088_ACCEL_24G_SEN  0.007177734375f       // ±24g

// 陀螺仪灵敏度（rad/s per LSB）
#define BMI088_GYRO_2000_SEN  0.00106526443603169529841533860381f   // ±2000°/s
#define BMI088_GYRO_1000_SEN  0.00053263221801584764920766930190693f // ±1000°/s
#define BMI088_GYRO_500_SEN   0.00026631610900792382460383465095346f // ±500°/s
#define BMI088_GYRO_250_SEN   0.00013315805450396191230191732547673f // ±250°/s
#define BMI088_GYRO_125_SEN   0.000066579027251980956150958662738366f // ±125°/s

 
// 原始数据结构体（直接从传感器读取的16位整型数据）
// 使用 __packed 防止编译器对齐填充，确保内存布局紧凑
 
typedef __packed struct BMI088_RAW_DATA
{
    uint8_t status;        // 状态字节（如数据就绪标志）
    int16_t accel[3];      // 加速度原始值 [X, Y, Z]
    int16_t temp;          // 温度原始值
    int16_t gyro[3];       // 陀螺仪原始值 [X, Y, Z]
} bmi088_raw_data_t;

 
// 物理数据结构体（转换后的浮点型物理量）
 
typedef struct BMI088_REAL_DATA
{
    uint8_t status;        // 状态字节
    float accel[3];        // 加速度 (m/s²)
    float temp;            // 温度 (°C)
    float gyro[3];         // 角速度 (rad/s)
    float time;            // 时间戳（秒，可由 HAL_GetTick() 转换）
} bmi088_real_data_t;

 
// 错误码定义（用于初始化返回值）
// 高位用于标识模块，低位用于具体错误
 
enum
{
    BMI088_NO_ERROR = 0x00,                             // 无错误

    // 加速度计配置错误
    BMI088_ACC_PWR_CTRL_ERROR      = 0x01,
    BMI088_ACC_PWR_CONF_ERROR      = 0x02,
    BMI088_ACC_CONF_ERROR          = 0x03,
    BMI088_ACC_SELF_TEST_ERROR     = 0x04,
    BMI088_ACC_RANGE_ERROR         = 0x05,
    BMI088_INT1_IO_CTRL_ERROR      = 0x06,
    BMI088_INT_MAP_DATA_ERROR      = 0x07,

    // 陀螺仪配置错误
    BMI088_GYRO_RANGE_ERROR        = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR    = 0x09,
    BMI088_GYRO_LPM1_ERROR         = 0x0A,
    BMI088_GYRO_CTRL_ERROR         = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR  = 0x0D,

    // 自检错误（高位标志）
    BMI088_SELF_TEST_ACCEL_ERROR   = 0x80,  // 加速度计自检失败
    BMI088_SELF_TEST_GYRO_ERROR    = 0x40,  // 陀螺仪自检失败

    // 严重错误
    BMI088_NO_SENSOR               = 0xFF,  // 未检测到传感器（ID 不匹配）
};

 
// 公共函数声明
 

/**
 * @brief 初始化 BMI088（加速度计 + 陀螺仪）
 * @return 错误码（0 表示成功，非0表示具体错误）
 */
uint8_t BMI088_init(void);

/**
 * @brief 单独初始化加速度计
 * @return 错误码
 */
unsigned char bmi088_accel_init(void);

/**
 * @brief 单独初始化陀螺仪
 * @return 错误码
 */
unsigned char bmi088_gyro_init(void);

/**
 * @brief 读取 BMI088 传感器数据
 * @param gyro[3]     输出：陀螺仪角速度（单位由 BMI088_GYRO_xxx_SEN 决定，通常为 rad/s）
 * @param accel[3]    输出：加速度（单位由 BMI088_ACCEL_xxx_SEN 决定，通常为 m/s²）
 * @param temperate   输出：温度（°C）
 * 
 * @note 此函数内部已处理片选、SPI 通信、数据解析
 */
void BMI088_read(float gyro[3], float accel[3], float *temperate);

#endif // BMI088DRIVER_H
