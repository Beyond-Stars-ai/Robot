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
    g_state.small_part    = 0.0f;
    g_state.big_yaw_vel   = 0.0f;

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
// 控制逻辑：接力模型（SmallYaw 先动，到达限位后退避并由 BigYaw 接手）
//
//   阶段一：SmallYaw 未触及限位 (is_returning == 0)
//           → 遥控器的输入 delta 直接累加到 small_part
//           → 当 small_part 达到 SMALL_LIMIT 时，切换到阶段二
//
//   阶段二：SmallYaw 回中阶段 (is_returning == 1)
//           → small_part 按 DECAY_RATE 比例缩小（缓缓回中）
//           → 当 small_part 接近 0 时，回到阶段一，重新获得移动能力
//
//   最终效果：BigYaw 承接了 (virtual_coord - small_part)，
//             在稳态下 small_part 为 0，即 SmallYaw 归中，BigYaw 全权负责朝向。
//
void Virtual_Yaw_Update(int16_t rc_value, float real_small, float real_big)
{
    // SmallYaw 最大偏转限幅（8192 * 0.2 = 1638.4）
    const float SMALL_LIMIT = 1638.4f; 
    // 衰减系数（控制回中速度），0.98 表示每拍缩小 2%
    const float DECAY_RATE  = 0.98f;

    //---------- 1. 记录实际编码 ----------
    g_state.real_small_now = real_small;
    g_state.real_big_now   = real_big;

    //---------- 2. 遥控器积分（总期望朝向总量）----------
    float delta = 0.0f;
    if (abs(rc_value) >= (int16_t)VIRTUAL_RC_DEADZONE) {
        delta = VIRTUAL_RC_SENS * (float)rc_value;
    }
    g_state.virtual_coord += delta;
    g_state.virtual_coord = limit_value(g_state.virtual_coord,
                                        -VIRTUAL_LIMIT, VIRTUAL_LIMIT);

    //---------- 3. 处理“神龙摆尾”跟随逻辑 ----------
    //
    // 控制策略：非线性弹性模型 + 速度平滑
    // 1. 遥控器输入改变 SmallYaw 角度
    // 2. 根据 SmallYaw 的偏角大小计算一个“期望跟随速度”
    //    - 偏角越大，跟随速度越快（二次方特性）
    // 3. 对该速度进行低通滤波，模拟“惯性”和“摆尾”的柔顺感

    // 遥控器输入作用于小头
    g_state.small_part += delta;

    // A. 计算期望跟随速度 (Desired Velocity)
    // 基础系数：小范围随动速度
    const float K_P = 0.05f; 
    // 非线性系数：大范围甩头速度
    const float K_Q = 0.0001f; 
    
    float error = g_state.small_part;
    float desired_vel = (K_P * error) + (K_Q * error * error * (error > 0 ? 1.0f : -1.0f));

    // B. 速度平滑滤波 (Velocity Smoothing / The Tail Effect)
    // 系数越小，尾巴越“肉”，加速感越强；系数越大，跟随越死板
    const float SMOOTH_COEFF = 0.15f; 
    g_state.big_yaw_vel += (desired_vel - g_state.big_yaw_vel) * SMOOTH_COEFF;

    // C. 应用位移
    // 小 Yaw 减去速度，大 Yaw 加上速度
    g_state.small_part -= g_state.big_yaw_vel;

    // 物理保护：防止小 Yaw 超出机械限位
    if (fabsf(g_state.small_part) > SMALL_LIMIT) {
        float over = fabsf(g_state.small_part) - SMALL_LIMIT;
        // 超限部分强制推给大 Yaw
        g_state.small_part = limit_value(g_state.small_part, -SMALL_LIMIT, SMALL_LIMIT);
        // 这里可以根据需要微调（超限补偿）
    }

    // BigYaw 承担总量中除去 SmallYaw 贡献后的剩余部分
    float big_part = g_state.virtual_coord - g_state.small_part;

    //---------- 4. 计算目标编码 ----------
    // SmallYaw：机械中值 - small_part（向右减小）
    g_state.target_small = normalize_angle(
        (float)origin_SmallYaw_count - g_state.small_part);

    virtual_small = (int16_t)g_state.target_small;

    // BigYaw：机械中值 + big_part
    // 【注意】如 BigYaw 方向相反，将 big_part 前的 "+" 改为 "-"
    g_state.target_big = normalize_angle(
        (float)origin_BigYaw_count + big_part);

    virtual_big = (int16_t)g_state.target_big;

    //---------- 5. 计算误差（带环绕处理）----------
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

