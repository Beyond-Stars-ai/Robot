#include "virtual_postion.h"
#include <math.h>
#include <stdlib.h>

//=========================== 内部变量 ===========================//

static Virtual_Yaw_State_t g_state = {0};
static float g_chassis_gain = CHASSIS_FOLLOW_GAIN;

// 调试用
int16_t virtual_big;
int16_t virtual_small;
float virtual_coord_debug;
int16_t rc_chassis_debug;

//=========================== 工具函数 ===========================//

static inline float limit_value(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static float normalize_encoder(float value)
{
    while (value < 0.0f) value += ENCODER_MAX;
    while (value >= ENCODER_MAX) value -= ENCODER_MAX;
    return value;
}

static float encoder_delta(float target, float current)
{
    float diff = target - current;
    if (diff > ENCODER_HALF) {
        diff -= ENCODER_MAX;
    } else if (diff < -ENCODER_HALF) {
        diff += ENCODER_MAX;
    }
    return diff;
}

//=========================== 初始化 ===========================//

void Virtual_Yaw_Init(void)
{
    g_state.origin_big = origin_BigYaw_count;
    g_state.origin_small = origin_SmallYaw_count;
    
    g_state.rc_real = 0;
    g_state.rc_chassis = 0;
    g_state.rc_total = 0;
    
    g_state.virtual_coord = 0.0f;
    g_state.small_part = 0.0f;
    g_state.big_yaw_vel = 0.0f;
    
    g_state.real_small = 0.0f;
    g_state.real_big = 0.0f;
    
    g_state.target_small = (float)origin_SmallYaw_count;
    g_state.target_big = (float)origin_BigYaw_count;
    
    g_state.error_small = 0.0f;
    g_state.error_big = 0.0f;
    
    g_chassis_gain = CHASSIS_FOLLOW_GAIN;
}

//=========================== 核心更新函数 ===========================//

void Virtual_Yaw_Update(int16_t rc_value, float chassis_delta,
                        float real_small, float real_big)
{
    //---------- 1. 记录实际编码 ----------
    g_state.real_small = real_small;
    g_state.real_big = real_big;
    g_state.rc_real = rc_value;
    
    //---------- 2. 【核心】底盘delta -> 虚拟RC值 ----------
    // 简单直接：底盘右转（delta为正），虚拟RC为负（左补偿）
    // virtual_rc = -chassis_delta * gain
    // 
    // 例如：chassis_delta = 30（底盘右转30编码器值/20ms）
    //       virtual_rc = -30 * 3 = -90
    //       相当于遥控器向左打90，云台跟着左转
    
    float virtual_rc_f = -chassis_delta * g_chassis_gain;
    
    // 限幅保护（避免电机暴走）
    virtual_rc_f = limit_value(virtual_rc_f, -500.0f, 500.0f);
    g_state.rc_chassis = (int16_t)virtual_rc_f;
    
    //---------- 3. 混合输入 ----------
    g_state.rc_total = g_state.rc_real + g_state.rc_chassis;
    
    //---------- 4. 【保持不变】神龙摆尾算法 ----------
    float delta = 0.0f;
    if (abs(g_state.rc_total) >= (int16_t)VIRTUAL_RC_DEADZONE) {
        delta = VIRTUAL_RC_SENS * (float)g_state.rc_total;
    }
    
    // 累加到虚拟坐标
    g_state.virtual_coord = normalize_encoder(g_state.virtual_coord + delta);
    
    // 输入作用于SmallYaw
    g_state.small_part += delta;
    
    // 神龙摆尾：非线性弹性模型
    const float K_P = 0.05f;
    const float K_Q = 0.0001f;
    const float SMOOTH_COEFF = 0.15f;
    
    float error = g_state.small_part;
    float desired_vel = (K_P * error) + 
                        (K_Q * error * error * (error > 0 ? 1.0f : -1.0f));
    
    // 速度平滑
    g_state.big_yaw_vel += (desired_vel - g_state.big_yaw_vel) * SMOOTH_COEFF;
    
    // SmallYaw减去速度
    g_state.small_part -= g_state.big_yaw_vel;
    
    // SmallYaw限位
    g_state.small_part = limit_value(g_state.small_part, 
                                     -SMALL_YAW_LIMIT, SMALL_YAW_LIMIT);
    
    //---------- 5. 计算BigYaw承担部分 ----------
    float big_part = encoder_delta(g_state.virtual_coord, g_state.small_part);
    
    //---------- 6. 计算目标编码 ----------
    g_state.target_small = normalize_encoder(
        (float)g_state.origin_small - g_state.small_part);
    
    g_state.target_big = normalize_encoder(
        (float)g_state.origin_big + big_part);
    
    //---------- 7. 计算误差 ----------
    g_state.error_small = encoder_delta(g_state.target_small, real_small);
    g_state.error_big = encoder_delta(g_state.target_big, real_big);
    
    //---------- 8. 调试变量 ----------
    virtual_small = (int16_t)g_state.target_small;
    virtual_big = (int16_t)g_state.target_big;
    virtual_coord_debug = g_state.virtual_coord;
    rc_chassis_debug = g_state.rc_chassis;
}

//=========================== 输出接口 ===========================//

float Virtual_Yaw_GetTarget_Small(void)
{
    return g_state.target_small;
}

float Virtual_Yaw_GetTarget_Big(void)
{
    return g_state.target_big;
}

float Virtual_Yaw_GetError_Small(void)
{
    return g_state.error_small;
}

float Virtual_Yaw_GetError_Big(void)
{
    return g_state.error_big;
}

const Virtual_Yaw_State_t* Virtual_Yaw_GetState(void)
{
    return &g_state;
}

void Virtual_Yaw_Reset(void)
{
    g_state.virtual_coord = 0.0f;
    g_state.small_part = 0.0f;
    g_state.big_yaw_vel = 0.0f;
    g_state.rc_chassis = 0;
    g_state.rc_total = 0;
    
    g_state.target_small = (float)g_state.origin_small;
    g_state.target_big = (float)g_state.origin_big;
}

void Virtual_Yaw_SetChassisGain(float gain)
{
    g_chassis_gain = gain;
}
