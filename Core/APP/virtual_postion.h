#ifndef __VIRTUAL_POSTION_H
#define __VIRTUAL_POSTION_H

#include <stdint.h>
#include <stdbool.h>

//=========================== 外部变量 ===========================//

extern int16_t origin_BigYaw_count;
extern int16_t origin_SmallYaw_count;

//=========================== 配置参数 ===========================//

#define VIRTUAL_RC_CHANNEL          2
#define VIRTUAL_RC_SENS             0.15f
#define VIRTUAL_RC_DEADZONE         50.0f

// 编码器参数
#define ENCODER_MAX                 8192.0f
#define ENCODER_HALF                4096.0f

// SmallYaw限幅（40%量程）
#define SMALL_YAW_LIMIT             3276.8f

// 底盘跟随增益：chassis_delta -> 虚拟RC值的倍数
// 1ms平均分策略：CalTask把10ms delta除以10，每1ms给一份
// 所以gain保持6.0，10ms累积效果与原来相同，但更平滑
// 如果仍感觉太灵敏，请尝试减小到 3.0f 或 4.0f
#define CHASSIS_FOLLOW_GAIN         3.0f  // 建议先试试6.0，如果太灵敏改3.0


//=========================== 数据结构 ===========================//

typedef struct {
    // 配置
    int16_t origin_big;
    int16_t origin_small;
    
    // 输入混合
    int16_t rc_real;                // 真实遥控器值
    int16_t rc_chassis;             // 底盘补偿虚拟RC值
    int16_t rc_total;               // 总RC值 = rc_real + rc_chassis
    
    // 虚拟坐标（神龙摆尾）
    float virtual_coord;
    float small_part;
    float big_yaw_vel;
    
    // 实际值
    float real_small;
    float real_big;
    
    // 目标值
    float target_small;
    float target_big;
    
    // 误差
    float error_small;
    float error_big;
    
} Virtual_Yaw_State_t;

//=========================== 接口函数 ===========================//

/**
 * @brief 初始化
 */
void Virtual_Yaw_Init(void);

/**
 * @brief 更新虚拟坐标（保留神龙摆尾）
 * 
 * 核心：底盘补偿delta直接转换为虚拟RC值
 *       virtual_rc = -chassis_delta * CHASSIS_FOLLOW_GAIN
 * 
 * @param rc_value          真实遥控器值（ch[2]）
 * @param chassis_delta     底盘转动变化量（编码器值/20ms）
 *                          来自CalTask：chassis_delta = -delta_yaw * 22.756f
 * @param real_small        SmallYaw实际编码
 * @param real_big          BigYaw实际编码
 */
void Virtual_Yaw_Update(int16_t rc_value, float chassis_delta,
                        float real_small, float real_big);

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
 * @brief 获取完整状态
 */
const Virtual_Yaw_State_t* Virtual_Yaw_GetState(void);

/**
 * @brief 重置
 */
void Virtual_Yaw_Reset(void);

/**
 * @brief 设置底盘跟随增益（调试用）
 */
void Virtual_Yaw_SetChassisGain(float gain);

#endif // __VIRTUAL_POSTION_H
