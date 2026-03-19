#include "Chassis_Follow.h"
#include <math.h>
#include <stdlib.h>

//=========================== 内部状态 ===========================//

static ChassisFollow_State_t g_state = {0};

//=========================== 工具函数 ===========================//

static float normalize_angle_180(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float normalize_encoder(float angle)
{
    while (angle < 0.0f) angle += 8192.0f;
    while (angle >= 8192.0f) angle -= 8192.0f;
    return angle;
}

//=========================== 核心解析函数 ===========================//
//
// 功能：解析底盘转动，输出BigYaw补偿值
// 原理：类似BMI088_GetData的数据处理方式
//       输入底盘Yaw数据，输出编码器补偿值
//
//===========================

ChassisFollow_State_t ChassisFollow_Parse(ChassisFollow_State_t *state)
{
    static uint8_t s_inited = 0;
    static float s_origin_yaw = 0.0f;
    static uint32_t last_time = 0;
    
    uint32_t current_time = HAL_GetTick();
    state->delta_time = current_time - last_time;
    last_time = current_time;
    
    //---------- 1. 初始化检查 ----------
    if (!s_inited && state->chassis_yaw_valid) {
        s_origin_yaw = state->chassis_yaw;
        s_inited = 1;
        state->is_ready = 1;
    }
    
    //---------- 2. 计算底盘相对角度 ----------
    if (s_inited) {
        float rel = state->chassis_yaw - s_origin_yaw;
        state->chassis_relative = normalize_angle_180(rel);
        
        //---------- 3. 计算编码器补偿值 ----------
        // 底盘转动时，BigYaw需要反向补偿以保持云台绝对朝向
        // 补偿值 = -底盘相对角度 * 编码器转换系数
        float compensation = -state->chassis_relative * CHASSIS_FOLLOW_ENCODER_SCALE;
        state->compensation_encoder = compensation;
        
        //---------- 4. 计算目标编码器值（机械中值 + 补偿）----------
        state->target_encoder = normalize_encoder(
            (float)state->origin_encoder + compensation);
    } else {
        state->chassis_relative = 0.0f;
        state->compensation_encoder = 0.0f;
        state->target_encoder = (float)state->origin_encoder;
    }
    
    return *state;
}

//=========================== 便捷接口 ===========================//

void ChassisFollow_Init(ChassisFollow_State_t *state, int16_t origin_encoder)
{
    state->origin_encoder = origin_encoder;
    state->chassis_yaw = 0.0f;
    state->chassis_yaw_valid = 0;
    state->chassis_relative = 0.0f;
    state->compensation_encoder = 0.0f;
    state->target_encoder = (float)origin_encoder;
    state->is_ready = 0;
    state->delta_time = 0;
}

float ChassisFollow_GetCompensation(const ChassisFollow_State_t *state)
{
    // 返回补偿值（可与virtual_position输出相加）
    return state->compensation_encoder;
}

float ChassisFollow_GetTarget(const ChassisFollow_State_t *state)
{
    // 返回绝对目标值（机械中值 + 补偿）
    return state->target_encoder;
}

float ChassisFollow_GetRelativeAngle(const ChassisFollow_State_t *state)
{
    return state->chassis_relative;
}

uint8_t ChassisFollow_IsReady(const ChassisFollow_State_t *state)
{
    return state->is_ready;
}
