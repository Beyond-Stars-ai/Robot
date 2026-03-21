#include "CalTask_Yaw.h"
#include <math.h>

//=========================== 内部状态 ===========================//

static CalTask_YawData_t s_yaw_data = {0};
static uint8_t s_inited = 0;

//=========================== 工具函数 ===========================//

/**
 * @brief 计算角度差值（处理±180°跨越）
 */
static float angle_diff(float new_angle, float old_angle)
{
    float diff = new_angle - old_angle;
    // 处理跨越±180°的情况
    if (diff > 180.0f) {
        diff -= 360.0f;
    } else if (diff < -180.0f) {
        diff += 360.0f;
    }
    return diff;
}

//=========================== 核心计算函数 ===========================//

CalTask_YawData_t CalTask_Yaw_Update(float yaw_now)
{
    //---------- 0. 首次初始化 ----------
    if (!s_inited) {
        s_yaw_data.yaw_prev = yaw_now;
        s_yaw_data.yaw_now = yaw_now;
        s_yaw_data.delta_yaw = 0.0f;
        s_yaw_data.angular_velocity = 0.0f;
        s_yaw_data.data_valid = 0;
        s_inited = 1;
        return s_yaw_data;
    }
    
    //---------- 1. 保存历史值 ----------
    s_yaw_data.yaw_prev = s_yaw_data.yaw_now;
    s_yaw_data.yaw_now = yaw_now;
    
    //---------- 2. 计算变化量（后 - 前）----------
    s_yaw_data.delta_yaw = angle_diff(s_yaw_data.yaw_now, s_yaw_data.yaw_prev);
    
    //---------- 3. 计算角速度（度/秒）----------
    // 10ms = 0.01s，乘以100 = 每秒变化
    s_yaw_data.angular_velocity = s_yaw_data.delta_yaw * 100.0f;
    
    //---------- 4. 标记数据有效 ----------
    s_yaw_data.data_valid = 1;
    
    return s_yaw_data;
}

//=========================== 便捷接口 ===========================//

float CalTask_Yaw_GetDelta(void)
{
    return s_yaw_data.delta_yaw;
}

float CalTask_Yaw_GetAngularVelocity(void)
{
    return s_yaw_data.angular_velocity;
}

float CalTask_Yaw_GetNow(void)
{
    return s_yaw_data.yaw_now;
}

float CalTask_Yaw_GetPrev(void)
{
    return s_yaw_data.yaw_prev;
}

uint8_t CalTask_Yaw_IsValid(void)
{
    return s_yaw_data.data_valid;
}

const CalTask_YawData_t* CalTask_Yaw_GetData(void)
{
    return &s_yaw_data;
}

void CalTask_Yaw_Reset(void)
{
    s_yaw_data.yaw_prev = 0.0f;
    s_yaw_data.yaw_now = 0.0f;
    s_yaw_data.delta_yaw = 0.0f;
    s_yaw_data.angular_velocity = 0.0f;
    s_yaw_data.data_valid = 0;
    s_inited = 0;
}
