#ifndef __GIMBAL_POSECALC_H
#define __GIMBAL_POSECALC_H

#include <stdint.h>
#include <math.h>
#include "BMI088.h"

//=========================== 机械偏移配置 ===========================//
// 根据实际机械结构调整！
// #define BIGYAW_ANGLE_OFFSET     186.0f        // 大Yaw机械中值偏移（度）
#define BIGYAW_ANGLE_OFFSET     8.0f        // 大Yaw机械中值偏移（度）
// 注意：SmallYaw 在哨兵模式下锁定编码器值，不使用角度偏移

//=========================== 数据类型 ===========================//

typedef struct {
    float Yaw;          // 绝对Yaw角度（相对于地面，-180~180）
    float Pitch;        // 绝对Pitch角度
    float Roll;         // 绝对Roll角度
    float Temp;         // 温度
    
    // 原始数据（调试用）
    float Raw_Yaw;      // 原始Yaw
    int64_t r;          // 转过圈数（编码器跨圈计数）
} Gimbal_Absolute_Angle_t;

//=========================== 外部变量 ===========================//

extern Gimbal_Absolute_Angle_t BigYaw_Absolute;     // 大Yaw绝对角度（用于控制）
extern Gimbal_Absolute_Angle_t SmallYaw_Absolute;   // 小Yaw绝对角度（调试用，哨兵模式下不用于控制）
extern float Chassis_IMU_Yaw;                       // 底盘IMU Yaw（取反后）

//=========================== 接口函数 ===========================//

/**
 * @brief 初始化姿态计算
 */
void Gimbal_PoseCalc_Init(void);

/**
 * @brief 姿态计算主函数（每1ms调用）
 * @note  核心公式：
 *        BigYaw绝对角度 = -(底盘IMU_Yaw + 电机编码器角度 + 偏移)
 *        SmallYaw绝对角度 = BigYaw绝对角度 + 电机相对角度 + 偏移
 */
void Gimbal_PoseCalc_Update(void);

/**
 * @brief 编码器值转角度（度）
 * @param encoder 编码器值 (0-8192)
 * @return 角度 (-180~180)
 */
static inline float Encoder_To_Angle(int16_t encoder)
{
    float angle = encoder / 8192.0f * 360.0f;
    if (angle > 180.0f) angle -= 360.0f;
    else if (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 角度归一化到-180~180
 */
static inline float Angle_Normalize(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 计算角度差（处理环绕）
 */
static inline float Angle_Diff(float target, float current)
{
    float diff = target - current;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

#endif // __GIMBAL_POSECALC_H
