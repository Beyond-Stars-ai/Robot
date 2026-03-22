#include "virtual_postion.h"
#include <math.h>
#include <stdlib.h>

//=========================== 内部变量 ===========================//

static Virtual_Yaw_State_t g_state = {0};

// 调试用
int16_t virtual_big;
int16_t virtual_small;
float target_angle_debug;

//=========================== 工具函数 ===========================//

static float normalize_encoder(float value)
{
    while (value < 0.0f) value += 8192.0f;
    while (value >= 8192.0f) value -= 8192.0f;
    return value;
}

//=========================== 初始化 ===========================//

void Virtual_Yaw_Init(void)
{
    g_state.target_angle_yaw = 0.0f;
    
    g_state.target_small = (float)origin_SmallYaw_count;  // 锁定中值
    g_state.target_big = (float)origin_BigYaw_count;
    
    g_state.error_small = 0.0f;
    g_state.error_big = 0.0f;
}

//=========================== 核心更新函数 ===========================//

void Virtual_Yaw_Update(int16_t rc_value, float real_small, float real_big)
{
    (void)real_small;     // 未使用
    (void)real_big;       // 未使用（使用BigYaw_Absolute绝对角度）
    
    //---------- 1. 遥控器输入控制目标角度 ----------
    if (abs(rc_value) >= (int16_t)VIRTUAL_RC_DEADZONE) {
        // 遥控器控制目标角度增量
        g_state.target_angle_yaw += (float)rc_value * VIRTUAL_RC_SENS;
        
        // 归一化到-180~180
        while (g_state.target_angle_yaw > 180.0f) g_state.target_angle_yaw -= 360.0f;
        while (g_state.target_angle_yaw < -180.0f) g_state.target_angle_yaw += 360.0f;
    }
    
    //---------- 2. SmallYaw：锁定中值，不参与控制 ----------
    g_state.target_small = (float)origin_SmallYaw_count;
    g_state.error_small = 0.0f;  // 无误差（直接锁定）
    
    //---------- 3. BigYaw：目标编码器值计算 ----------
    // 从姿态计算获取当前绝对角度，计算误差
    extern Gimbal_Absolute_Angle_t BigYaw_Absolute;
    
    // 角度误差
    float angle_error = g_state.target_angle_yaw - BigYaw_Absolute.Yaw;
    
    // 处理环绕
    while (angle_error > 180.0f) angle_error -= 360.0f;
    while (angle_error < -180.0f) angle_error += 360.0f;
    
    g_state.error_big = angle_error;
    
    // 转换为编码器值：角度误差 * (8192/360) = 角度误差 * 22.756
    const float ANGLE_TO_ENC = 22.756f;
    float enc_offset = angle_error * ANGLE_TO_ENC;
    
    // 目标编码器 = 中值 + 编码器偏移
    g_state.target_big = normalize_encoder((float)origin_BigYaw_count + enc_offset);
    
    //---------- 4. 更新调试变量 ----------
    virtual_small = (int16_t)g_state.target_small;
    virtual_big = (int16_t)g_state.target_big;
    target_angle_debug = g_state.target_angle_yaw;
}

//=========================== 直接设置目标角度 ===========================//

void Virtual_Yaw_SetTargetAngle(float angle)
{
    // 归一化
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    
    g_state.target_angle_yaw = angle;
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

float Virtual_Yaw_GetTargetAngle(void)
{
    return g_state.target_angle_yaw;
}

void Virtual_Yaw_Reset(void)
{
    g_state.target_angle_yaw = 0.0f;
    g_state.target_small = (float)origin_SmallYaw_count;
    g_state.target_big = (float)origin_BigYaw_count;
}
