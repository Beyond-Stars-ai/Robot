#include "virtual_postion.h"
#include <math.h>

//=========================== 内部变量 ===========================//

static Virtual_Yaw_State_t g_state = {0};

//=========================== 工具函数 ===========================//

static float limit_value(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static float normalize_angle(float angle)
{
    while (angle < 0.0f) angle += 8192.0f;
    while (angle >= 8192.0f) angle -= 8192.0f;
    return angle;
}

//=========================== 初始化 ===========================//

void Virtual_Yaw_Init(void)
{
    // 虚拟坐标从0开始（对应机械中值）
    g_state.virtual_coord = 0.0f;
    
    // 初始化目标为机械中值
    g_state.target_small = (float)origin_SmallYaw_count;
    g_state.target_big = (float)origin_BigYaw_count;
    
    // 清零其他
    g_state.real_small_now = 0.0f;
    g_state.real_big_now = 0.0f;
    g_state.error_small = 0.0f;
    g_state.error_big = 0.0f;
}

//=========================== 更新（核心）===========================//

void Virtual_Yaw_Update(int16_t rc_value, float real_small, float real_big)
{
    //---------- 1. 记录实际编码 ----------
    g_state.real_small_now = real_small;
    g_state.real_big_now = real_big;
    
    //---------- 2. 更新虚拟坐标（遥控器控制）----------
    // rc_value已偏移（中位=0），向右打杆为正
    float delta = VIRTUAL_RC_SENS * (float)rc_value;
    g_state.virtual_coord += delta;
    
    // 限幅
    g_state.virtual_coord = limit_value(g_state.virtual_coord, -VIRTUAL_LIMIT, VIRTUAL_LIMIT);
    
    //---------- 3. 虚拟坐标 → 目标编码 ----------
    // SmallYaw: 向右转编码减小，目标 = origin - virtual
    g_state.target_small = (float)origin_SmallYaw_count - g_state.virtual_coord;
    g_state.target_small = normalize_angle(g_state.target_small);
    
    // BigYaw: 向右转编码增加，目标 = origin + virtual
    g_state.target_big = (float)origin_BigYaw_count + g_state.virtual_coord;
    g_state.target_big = normalize_angle(g_state.target_big);
    
    //---------- 4. 计算误差 ----------
    // 处理循环误差（0/8191跳变）
    float err_s = g_state.target_small - real_small;
    if (err_s > 4096.0f) err_s -= 8192.0f;
    else if (err_s < -4096.0f) err_s += 8192.0f;
    g_state.error_small = err_s;
    
    float err_b = g_state.target_big - real_big;
    if (err_b > 4096.0f) err_b -= 8192.0f;
    else if (err_b < -4096.0f) err_b += 8192.0f;
    g_state.error_big = err_b;
}

//=========================== 获取目标（供控制层使用）===========================//

float Virtual_Yaw_GetTarget_Small(void)
{
    return g_state.target_small;
}

float Virtual_Yaw_GetTarget_Big(void)
{
    return g_state.target_big;
}

//=========================== 获取完整状态 ===========================//

void Virtual_Yaw_GetState(Virtual_Yaw_State_t *state)
{
    if (state != NULL) {
        *state = g_state;
    }
}
