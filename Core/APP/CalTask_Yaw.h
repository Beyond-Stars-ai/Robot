#ifndef __CALTASK_YAW_H
#define __CALTASK_YAW_H

#include <stdint.h>

//=========================== 配置参数 ===========================//

// 采样周期：1ms（在CalTask中1ms调用一次）
#define CALTASK_YAW_SAMPLE_MS       1

// 差值计算间隔：20ms（保留20ms前的数据做差）
#define CALTASK_YAW_INTERVAL_MS     20

// 缓冲区大小（需要容纳21个数据点：当前 + 20ms前）
#define CALTASK_YAW_BUFFER_SIZE     (CALTASK_YAW_INTERVAL_MS + 1)

//=========================== 数据结构 ===========================//
//
// 类似BMI088_Init_typedef的设计风格
// 输入：当前Yaw角度 + 时间戳
// 输出：变化量 + 角速度 + 有效标志
//
//===========================

// 输入结构体（由调用者填充）
typedef struct {
    float yaw_now;          // 当前Yaw角度（度，-180~180）
    uint32_t timestamp;     // 当前时间戳（ms，可选）
} CalTask_YawInput_t;

// 输出结构体（计算结果）
typedef struct {
    // 原始数据
    float yaw_now;          // 当前Yaw值
    float yaw_prev;         // 1ms前的Yaw值
    float yaw_20ms_ago;     // 20ms前的Yaw值（内部使用）
    
    // 变化量输出（核心）
    float delta_1ms;        // 1ms间隔变化量（后-前）
    float delta_20ms;       // 20ms间隔变化量（后-前），这是主要输出
    
    // 派生值
    float angular_velocity; // 角速度（度/秒）
    
    // 状态
    uint8_t data_valid;     // 数据有效标志（1=已积累20ms数据）
    uint32_t timestamp;     // 时间戳
    uint32_t delta_time;    // 计算间隔（20ms）
    
} CalTask_YawOutput_t;

//=========================== 接口函数 ===========================//

/**
 * @brief 计算Yaw变化量（在1ms周期的CalTask中调用）
 * @param input 输入结构体指针（填充yaw_now和timestamp）
 * @return 输出结构体（包含变化量和状态）
 * 
 * @note 使用方式：
 *       1. 在1ms周期任务中调用
 *       2. 填充input->yaw_now为当前Yaw角度
 *       3. 调用CalTask_Yaw_Calculate(&input)
 *       4. 通过获取接口或返回值读取delta_20ms
 * 
 * @example
 *       CalTask_YawInput_t input = {0};
 *       input.yaw_now = g_imu_yaw;  // 当前IMU读数
 *       CalTask_Yaw_Calculate(&input);
 *       float delta = CalTask_Yaw_GetDelta20ms();  // 获取20ms变化量
 */
CalTask_YawOutput_t CalTask_Yaw_Calculate(CalTask_YawInput_t *input);

/**
 * @brief 获取20ms间隔的变化量（后-前）
 * @return 20ms内的Yaw变化量（度）
 * 
 * @note 这是主要输出接口，供给Chassis_Follow使用
 *       正值表示顺时针转动，负值表示逆时针转动
 */
float CalTask_Yaw_GetDelta20ms(void);

/**
 * @brief 获取角速度（度/秒）
 * @return 当前估算的角速度
 */
float CalTask_Yaw_GetAngularVelocity(void);

/**
 * @brief 获取1ms间隔的即时变化量
 * @return 1ms内的Yaw变化量（度）
 */
float CalTask_Yaw_GetDelta1ms(void);

/**
 * @brief 检查数据是否有效（已积累20ms数据）
 * @return 1=有效，0=无效（刚启动未满20ms）
 */
uint8_t CalTask_Yaw_IsDataValid(void);

/**
 * @brief 获取完整的输出结构体
 * @return 输出结构体指针
 */
const CalTask_YawOutput_t* CalTask_Yaw_GetOutput(void);

/**
 * @brief 重置模块（清零缓冲区）
 */
void CalTask_Yaw_Reset(void);

#endif // __CALTASK_YAW_H
