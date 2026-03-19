#include "CalTask_Yaw.h"
#include <string.h>

//=========================== 内部缓冲区 ===========================//

// 环形缓冲区：保存最近20ms + 当前的数据（共21个采样点）
static float s_yaw_buffer[CALTASK_YAW_BUFFER_SIZE] = {0};
static uint32_t s_buffer_index = 0;
static uint32_t s_sample_count = 0;

// 输出状态
static CalTask_YawOutput_t s_output = {0};

// 初始化标志
static uint8_t s_inited = 0;

//=========================== 内部工具函数 ===========================//

/**
 * @brief 写入新数据到环形缓冲区
 */
static void buffer_write(float yaw)
{
    s_yaw_buffer[s_buffer_index] = yaw;
    s_buffer_index = (s_buffer_index + 1) % CALTASK_YAW_BUFFER_SIZE;
    if (s_sample_count < CALTASK_YAW_BUFFER_SIZE) {
        s_sample_count++;
    }
}

/**
 * @brief 获取N个采样点前的数据
 */
static float buffer_get_ago(uint32_t samples_ago)
{
    if (samples_ago >= s_sample_count) {
        return s_yaw_buffer[(s_buffer_index + CALTASK_YAW_BUFFER_SIZE - 1) % CALTASK_YAW_BUFFER_SIZE];
    }
    // 计算目标索引
    int32_t target = (int32_t)s_buffer_index - (int32_t)samples_ago - 1;
    if (target < 0) {
        target += CALTASK_YAW_BUFFER_SIZE;
    }
    return s_yaw_buffer[target];
}

/**
 * @brief 角度差值计算（处理跨±180°的情况）
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

CalTask_YawOutput_t CalTask_Yaw_Calculate(CalTask_YawInput_t *input)
{
    //---------- 0. 首次初始化 ----------
    if (!s_inited) {
        memset(s_yaw_buffer, 0, sizeof(s_yaw_buffer));
        s_buffer_index = 0;
        s_sample_count = 0;
        memset(&s_output, 0, sizeof(s_output));
        s_inited = 1;
    }
    
    //---------- 1. 写入当前数据到缓冲区 ----------
    buffer_write(input->yaw_now);
    
    //---------- 2. 保存当前值和前一次值 ----------
    s_output.yaw_now = input->yaw_now;
    s_output.yaw_prev = buffer_get_ago(1);  // 1ms前的值
    
    //---------- 3. 计算即时变化率（1ms间隔）----------
    s_output.delta_1ms = angle_diff(s_output.yaw_now, s_output.yaw_prev);
    
    //---------- 4. 计算20ms间隔的变化量（核心功能）----------
    if (s_sample_count > CALTASK_YAW_INTERVAL_MS) {
        // 获取20ms前的数据
        float yaw_20ms_ago = buffer_get_ago(CALTASK_YAW_INTERVAL_MS);
        
        // 计算20ms内的总变化量
        s_output.delta_20ms = angle_diff(s_output.yaw_now, yaw_20ms_ago);
        
        // 计算平均角速度（度/秒）
        // delta_20ms 是20ms内的角度变化，乘以50得到每秒变化
        s_output.angular_velocity = s_output.delta_20ms * 50.0f;  // °/s
        
        // 数据有效标志
        s_output.data_valid = 1;
    } else {
        // 数据不足20ms，标记为无效
        s_output.delta_20ms = 0.0f;
        s_output.angular_velocity = 0.0f;
        s_output.data_valid = 0;
    }
    
    //---------- 5. 更新时间戳 ----------
    s_output.timestamp = input->timestamp;
    s_output.delta_time = CALTASK_YAW_INTERVAL_MS;  // 20ms
    
    return s_output;
}

//=========================== 便捷接口 ===========================//

float CalTask_Yaw_GetDelta20ms(void)
{
    return s_output.delta_20ms;
}

float CalTask_Yaw_GetAngularVelocity(void)
{
    return s_output.angular_velocity;
}

float CalTask_Yaw_GetDelta1ms(void)
{
    return s_output.delta_1ms;
}

uint8_t CalTask_Yaw_IsDataValid(void)
{
    return s_output.data_valid;
}

const CalTask_YawOutput_t* CalTask_Yaw_GetOutput(void)
{
    return &s_output;
}

void CalTask_Yaw_Reset(void)
{
    memset(s_yaw_buffer, 0, sizeof(s_yaw_buffer));
    s_buffer_index = 0;
    s_sample_count = 0;
    memset(&s_output, 0, sizeof(s_output));
    s_inited = 1;
}
