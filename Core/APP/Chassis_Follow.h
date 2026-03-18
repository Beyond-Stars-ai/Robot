#ifndef __CHASSIS_FOLLOW_H
#define __CHASSIS_FOLLOW_H

#include <stdint.h>
#include "main.h"  // for HAL_GetTick

//=========================== 配置参数 ===========================//

// 角度转编码器系数 (8192 / 360 = 22.756)
#define CHASSIS_FOLLOW_ENCODER_SCALE    22.756f

//=========================== 数据结构 ===========================//
//
// 类似virtual_position的设计风格
// 输入：底盘Yaw数据
// 输出：BigYaw补偿值或目标值
//
//===========================

typedef struct {
    //========== 配置参数（初始化时设置）==========
    int16_t origin_encoder;         // BigYaw机械中值编码器值
    
    //========== 输入数据（由外部填充）==========
    float chassis_yaw;              // 底盘当前Yaw角度（度，-180~180）
    uint8_t chassis_yaw_valid;      // 底盘数据有效标志
    
    //========== 输出结果（解析后填充）==========
    float chassis_relative;         // 底盘相对开机时的转动角度
    float compensation_encoder;     // BigYaw补偿编码器值（可与virtual_position相加）
    float target_encoder;           // BigYaw绝对目标编码器值（机械中值+补偿）
    
    //========== 状态观察 ==========
    uint8_t is_ready;               // 是否已初始化完成
    uint32_t delta_time;            // 本次调用时间间隔(ms)
    
} ChassisFollow_State_t;

//=========================== 接口函数 ===========================//

/**
 * @brief 解析底盘Yaw数据，输出BigYaw补偿/目标值
 * @param state 状态结构体指针（输入输出）
 * @return 更新后的状态结构体
 * 
 * @note 使用方式：
 *       1. 初始化：ChassisFollow_Init(&state, origin_BigYaw_count)
 *       2. 填充输入：chassis_yaw, chassis_yaw_valid
 *       3. 调用解析：ChassisFollow_Parse(&state)
 *       4. 读取输出：compensation_encoder 或 target_encoder
 */
ChassisFollow_State_t ChassisFollow_Parse(ChassisFollow_State_t *state);

/**
 * @brief 初始化底盘跟随模块
 * @param state 状态结构体指针
 * @param origin_encoder BigYaw机械中值编码器值
 */
void ChassisFollow_Init(ChassisFollow_State_t *state, int16_t origin_encoder);

/**
 * @brief 获取BigYaw补偿编码器值（用于与virtual_position输出相加）
 * @param state 状态结构体指针
 * @return 补偿编码器值
 * 
 * @note 典型用法：
 *       float target = Virtual_Yaw_GetTarget_Big() + ChassisFollow_GetCompensation(&state);
 */
float ChassisFollow_GetCompensation(const ChassisFollow_State_t *state);

/**
 * @brief 获取BigYaw绝对目标编码器值（机械中值 + 补偿）
 * @param state 状态结构体指针
 * @return 目标编码器值
 */
float ChassisFollow_GetTarget(const ChassisFollow_State_t *state);

/**
 * @brief 获取底盘相对转动角度
 * @param state 状态结构体指针
 * @return 底盘相对开机时的转动角度（度）
 */
float ChassisFollow_GetRelativeAngle(const ChassisFollow_State_t *state);

/**
 * @brief 检查模块是否已准备好
 * @param state 状态结构体指针
 * @return 1=已准备好，0=未准备好
 */
uint8_t ChassisFollow_IsReady(const ChassisFollow_State_t *state);

#endif // __CHASSIS_FOLLOW_H
