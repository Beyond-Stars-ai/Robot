#include "virtual_postion.h"
#include <math.h>
#include <stdlib.h>

//=========================== 内部变量 ===========================//

static Virtual_Yaw_State_t g_state = {0};

// 调试用
int16_t virtual_big;
int16_t virtual_small;
float virtual_coord_debug;

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
    
    g_state.rc_input = 0;
    g_state.chassis_virtual_rc = 0;
    g_state.total_rc_input = 0;
    
    g_state.virtual_coord = 0.0f;
    g_state.small_part = 0.0f;
    g_state.big_yaw_vel = 0.0f;
    
    g_state.real_small = 0.0f;
    g_state.real_big = 0.0f;
    
    g_state.target_small = (float)origin_SmallYaw_count;
    g_state.target_big = (float)origin_BigYaw_count;
    
    g_state.error_small = 0.0f;
    g_state.error_big = 0.0f;
    
    g_state.last_chassis_compensation = 0.0f;
}

//=========================== 核心更新函数 ===========================//

void Virtual_Yaw_Update(int16_t rc_value, float chassis_compensation,
                        float real_small, float real_big)
{
    //---------- 1. 记录实际编码 ----------
    g_state.real_small = real_small;
    g_state.real_big = real_big;
    g_state.rc_input = rc_value;
    
    //---------- 2. 【核心改进】底盘补偿 → 虚拟RC值 ----------
    // 计算需要的补偿速度（编码器值/控制周期）
    // 10ms控制周期，要抵消 chassis_compensation，需要多少RC值？
    
    // 方法：计算底盘补偿的变化率，转换为等效RC值
    float comp_delta = chassis_compensation - g_state.last_chassis_compensation;
    
    // 处理环绕（比如从10变到8182，实际只变了-20）
    if (comp_delta > ENCODER_HALF) comp_delta -= ENCODER_MAX;
    if (comp_delta < -ENCODER_HALF) comp_delta += ENCODER_MAX;
    
    // 将编码器变化量转换为RC值
    // RC值 * VIRTUAL_RC_SENS = 编码器变化量
    // 所以：RC值 = 编码器变化量 / VIRTUAL_RC_SENS
    if (fabsf(comp_delta) > 1.0f) {
        // 有底盘转动，计算需要的虚拟RC值
        // 注意：这里是负号！底盘右转（comp_delta为正），需要左补偿（RC为负）
        float virtual_rc = -comp_delta / VIRTUAL_RC_SENS;
        
        // 限幅保护（避免电机暴走）
        g_state.chassis_virtual_rc = (int16_t)limit_value(virtual_rc, -660.0f, 660.0f);
    } else {
        g_state.chassis_virtual_rc = 0;
    }
    
    g_state.last_chassis_compensation = chassis_compensation;
    
    //---------- 3. 混合输入（真实RC + 虚拟RC）----------
    g_state.total_rc_input = g_state.rc_input + g_state.chassis_virtual_rc;
    
    //---------- 4. 【保持不变】神龙摆尾算法 ----------
    float delta = 0.0f;
    if (abs(g_state.total_rc_input) >= (int16_t)VIRTUAL_RC_DEADZONE) {
        delta = VIRTUAL_RC_SENS * (float)g_state.total_rc_input;
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
    
    // 速度平滑（保留摆尾效果）
    g_state.big_yaw_vel += (desired_vel - g_state.big_yaw_vel) * SMOOTH_COEFF;
    
    // SmallYaw减去速度，BigYaw承担
    g_state.small_part -= g_state.big_yaw_vel;
    
    // SmallYaw限位保护
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
    
    //---------- 8. 更新调试变量 ----------
    virtual_small = (int16_t)g_state.target_small;
    virtual_big = (int16_t)g_state.target_big;
    virtual_coord_debug = g_state.virtual_coord;
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
    g_state.chassis_virtual_rc = 0;
    g_state.total_rc_input = 0;
    g_state.last_chassis_compensation = 0.0f;
    
    g_state.target_small = (float)g_state.origin_small;
    g_state.target_big = (float)g_state.origin_big;
}
