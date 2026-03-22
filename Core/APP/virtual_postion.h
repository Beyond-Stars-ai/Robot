#ifndef __VIRTUAL_POSTION_H
#define __VIRTUAL_POSTION_H

#include <stdint.h>
#include <stdbool.h>
#include "Gimbal_PoseCalc.h"

//=========================== 外部变量 ===========================//

extern int16_t origin_BigYaw_count;
extern int16_t origin_SmallYaw_count;

//=========================== 配置参数 ===========================//

#define VIRTUAL_RC_SENS             0.0007f     // 遥控器角度控制灵敏度（度/单位）
#define VIRTUAL_RC_DEADZONE         50.0f       // 遥控器死区

//=========================== 数据结构 ===========================//

typedef struct {
    // 目标角度（蓝图式直接控制）
    float target_angle_yaw;         // 目标绝对角度（度，-180~180）
    
    // 输出
    float target_small;             // SmallYaw目标编码器（锁定中值）
    float target_big;               // BigYaw目标编码器
    
    // 误差（调试用）
    float error_small;
    float error_big;
    
} Virtual_Yaw_State_t;

//=========================== 接口函数 ===========================//

/**
 * @brief 初始化
 */
void Virtual_Yaw_Init(void);

/**
 * @brief 更新目标（支持底盘补偿）
 * @param rc_value 遥控器值
 * @param chassis_delta 底盘补偿变化量（编码器值/1ms，CalTask计算）
 * @param real_small SmallYaw实际编码
 * @param real_big BigYaw实际编码
 */
void Virtual_Yaw_Update(int16_t rc_value, float chassis_delta,
                        float real_small, float real_big);

/**
 * @brief 直接设置目标角度（用于自瞄等）
 * @param angle 目标角度（度，-180~180）
 */
void Virtual_Yaw_SetTargetAngle(float angle);

/**
 * @brief 获取目标编码值
 */
float Virtual_Yaw_GetTarget_Small(void);
float Virtual_Yaw_GetTarget_Big(void);

/**
 * @brief 获取误差
 */
float Virtual_Yaw_GetError_Small(void);
float Virtual_Yaw_GetError_Big(void);

/**
 * @brief 获取目标角度
 */
float Virtual_Yaw_GetTargetAngle(void);

/**
 * @brief 重置
 */
void Virtual_Yaw_Reset(void);

#endif // __VIRTUAL_POSTION_H
