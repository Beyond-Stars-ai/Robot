#include "virtual_postion.h"
#include <math.h>
#include <stdlib.h>

//=========================== 内部变量 ===========================//

static Virtual_Yaw_State_t g_state = {0};

// 供外部观察（调试用，只读）
int16_t virtual_big;
int16_t virtual_small;

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
    g_state.target_big   = (float)origin_BigYaw_count;

    // 清零其他
    g_state.real_small_now = 0.0f;
    g_state.real_big_now   = 0.0f;
    g_state.error_small    = 0.0f;
    g_state.error_big      = 0.0f;
}

//=========================== 更新（核心）===========================//
//
// 控制逻辑：BigYaw 连续比例跟随 SmallYaw
//
// 【方向约定】（如与实际相反，修改对应符号即可）
//   SmallYaw: 向右转 → real_small 编码值减小
//             → virtual_coord 为正时 target_small = origin - virtual
//   BigYaw  : 跟随 SmallYaw 实际偏角
//             → target_big = origin + small_offset
//             → 若实际跟随方向相反，将 "+" 改为 "-"
//
void Virtual_Yaw_Update(int16_t rc_value, float real_small, float real_big)
{
    //---------- 1. 记录实际编码 ----------
    g_state.real_small_now = real_small;
    g_state.real_big_now   = real_big;

    //---------- 2. 遥控器积分控制 SmallYaw 虚拟目标 ----------
    float delta = 0.0f;
    if (abs(rc_value) >= (int16_t)VIRTUAL_RC_DEADZONE) {
        delta = VIRTUAL_RC_SENS * (float)rc_value;
    }

    g_state.virtual_coord += delta;
    g_state.virtual_coord = limit_value(g_state.virtual_coord,
                                        -VIRTUAL_LIMIT, VIRTUAL_LIMIT);

    // SmallYaw 目标 = 机械中值 - 虚拟坐标（向右打杆为正，编码减小）
    g_state.target_small = normalize_angle(
        (float)origin_SmallYaw_count - g_state.virtual_coord);

    virtual_small = (int16_t)g_state.target_small;  // 调试观察

    //---------- 3. BigYaw 连续比例跟随 SmallYaw ----------
    // 计算 SmallYaw 实际偏角（相对机械中值，带符号，处理 0/8191 跳变）
    float small_offset = real_small - (float)origin_SmallYaw_count;
    if (small_offset >  4096.0f) small_offset -= 8192.0f;
    if (small_offset < -4096.0f) small_offset += 8192.0f;

    // BigYaw 目标 = 机械中值 + SmallYaw 实际偏角
    // 含义：SmallYaw 偏多少，BigYaw 就跟多少，使 SmallYaw 始终趋向归中
    // 【注意】如 BigYaw 跟随方向与 SmallYaw 偏角方向相反，将 "+" 改为 "-"
    g_state.target_big = normalize_angle(
        (float)origin_BigYaw_count - small_offset);

    virtual_big = (int16_t)g_state.target_big;  // 调试观察

    //---------- 4. 计算误差（带环绕处理）----------
    float err_s = g_state.target_small - real_small;
    if (err_s >  4096.0f) err_s -= 8192.0f;
    if (err_s < -4096.0f) err_s += 8192.0f;
    g_state.error_small = err_s;

    float err_b = g_state.target_big - real_big;
    if (err_b >  4096.0f) err_b -= 8192.0f;
    if (err_b < -4096.0f) err_b += 8192.0f;
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

