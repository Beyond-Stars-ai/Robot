#include "Chassis_Follow.h"
#include <math.h>
#include <stdlib.h>

//=========================== 内部状态 ===========================//

static Chassis_Follow_State_t g_state = {0};

//=========================== 工具函数 ===========================//

static float normalize_encoder(float angle)
{
    while (angle < 0.0f) angle += 8192.0f;
    while (angle >= 8192.0f) angle -= 8192.0f;
    return angle;
}

//=========================== 核心更新函数 ===========================//
//
// 功能：接收CalTask的delta，积分累加得到总变化，输出编码器补偿值
// 原理：
//   - 每次接收delta（20ms的yaw变化量）
//   - 积分累加到total_angle（类似virtual_position的virtual_coord）
//   - 转换为编码器值，作为big_yaw的补偿
//
// 注意：此模块只影响BigYaw，SmallYaw由virtual_position独立控制
//===========================

Chassis_Follow_State_t Chassis_Follow_Update(float delta_yaw)
{
    //---------- 1. 保存输入 ----------
    g_state.delta_yaw = delta_yaw;
    
    //---------- 2. 积分累加（核心）----------
    // total_angle是底盘累计转动的角度（相对于开机时刻）
    g_state.total_angle += delta_yaw;
    
    // 归一化到[-180, 180)防止数值溢出
    if (g_state.total_angle >= 180.0f) {
        g_state.total_angle -= 360.0f;
    } else if (g_state.total_angle < -180.0f) {
        g_state.total_angle += 360.0f;
    }
    
    //---------- 3. 转换为编码器值 ----------
    // 底盘转动时，BigYaw需要反向补偿以保持云台绝对朝向
    // 补偿值 = -total_angle * 编码器转换系数
    g_state.compensation_encoder = -g_state.total_angle * CHASSIS_FOLLOW_ENCODER_SCALE;
    
    //---------- 4. 计算绝对目标值 ----------
    // 机械中值 + 补偿 = BigYaw应该去的位置
    g_state.target_big = normalize_encoder(
        (float)g_state.origin_encoder + g_state.compensation_encoder);
    
    //---------- 5. SmallYaw不受影响 ----------
    // SmallYaw由virtual_position独立控制，此处保持为0
    g_state.target_small = 0.0f;
    
    //---------- 6. 更新状态 ----------
    g_state.is_ready = 1;
    
    return g_state;
}

//=========================== 初始化与接口 ===========================//

void Chassis_Follow_Init(int16_t origin_encoder)
{
    g_state.origin_encoder = origin_encoder;
    g_state.total_angle = 0.0f;
    g_state.delta_yaw = 0.0f;
    g_state.compensation_encoder = 0.0f;
    g_state.target_big = (float)origin_encoder;
    g_state.target_small = 0.0f;
    g_state.is_ready = 0;
}

float Chassis_Follow_GetTarget_Big(void)
{
    // 返回BigYaw补偿值（与Virtual_Yaw_GetTarget_Big()相加）
    return g_state.compensation_encoder;
}

float Chassis_Follow_GetTarget_Small(void)
{
    // SmallYaw不受底盘跟随影响，返回0
    return 0.0f;
}

float Chassis_Follow_GetTotalAngle(void)
{
    return g_state.total_angle;
}

float Chassis_Follow_GetDelta(void)
{
    return g_state.delta_yaw;
}

uint8_t Chassis_Follow_IsReady(void)
{
    return g_state.is_ready;
}

const Chassis_Follow_State_t* Chassis_Follow_GetState(void)
{
    return &g_state;
}

void Chassis_Follow_Reset(void)
{
    g_state.total_angle = 0.0f;
    g_state.delta_yaw = 0.0f;
    g_state.compensation_encoder = 0.0f;
    g_state.target_big = (float)g_state.origin_encoder;
    g_state.is_ready = 0;
}
