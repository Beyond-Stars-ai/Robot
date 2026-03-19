#ifndef __CHASSIS_FOLLOW_H
#define __CHASSIS_FOLLOW_H

#include <stdint.h>

//=========================== 配置参数 ===========================//

// 角度转编码器系数 (8192 / 360 = 22.756)
#define CHASSIS_FOLLOW_ENCODER_SCALE    22.756f

//=========================== 数据结构 ===========================//
//
// 类似virtual_position的设计风格
// 输入：CalTask提供的delta_yaw（20ms间隔的yaw变化量）
// 输出：BigYaw补偿编码器值（从实际反馈中减去）
//
// 【架构变更说明】
//   旧方案：补偿目标值 target_big = Virtual_Target + Chassis_Compensation
//   新方案：补偿反馈值 real_big_compensated = Real_Encoder - Chassis_Compensation
//
// 【数学等价】error = (V + C) - R  ==  V - (R - C)
//
// 【新方案优势】
//   1. Virtual_Yaw 保持单一职责：只需关注绝对朝向闭环
//   2. 便于单独开关底盘跟随（只需注释掉补偿那一行）
//   3. 目标值保持语义清晰：就是期望的绝对朝向
//
//===========================

typedef struct {
    //========== 配置参数（初始化时设置）==========
    int16_t origin_encoder;         // BigYaw机械中值编码器值
    
    //========== 输入数据（由CalTask提供）==========
    float delta_yaw;                // 20ms间隔的yaw变化量（度）
    
    //========== 内部状态（自动维护）==========
    float total_angle;              // 累计转动角度（相对于开机时刻，-180~180）
    
    //========== 输出结果（计算后填充）==========
    float compensation_encoder;     // BigYaw补偿编码器值（与virtual_position相加）
    float target_big;               // BigYaw绝对目标编码器值（机械中值+补偿）
    float target_small;             // SmallYaw目标（始终为0，不受底盘跟随影响）
    
    //========== 状态标志 ==========
    uint8_t is_ready;               // 是否已准备好
    
} Chassis_Follow_State_t;

//=========================== 接口函数 ===========================//

/**
 * @brief 初始化底盘跟随模块
 * @param origin_encoder BigYaw机械中值编码器值
 * 
 * @note 在系统初始化时调用一次
 */
void Chassis_Follow_Init(int16_t origin_encoder);

/**
 * @brief 更新底盘跟随状态（在CalTask中调用）
 * @param delta_yaw 20ms间隔的yaw变化量（度），来自CalTask_Yaw_GetDelta()
 * @return 更新后的状态结构体
 * 
 * @note 调用周期：20ms（与CalTask同步）
 * @note 核心逻辑：积分delta_yaw得到total_angle，转换为编码器补偿值
 * 
 * @example
 *       void StartCalTask(void *argument)
 *       {
 *           Chassis_Follow_Init(origin_BigYaw_count);
 *           
 *           for (;;)
 *           {
 *               float yaw = Can_BMI088_Data.Yaw;
 *               CalTask_Yaw_Update(yaw);
 *               
 *               float delta = CalTask_Yaw_GetDelta();
 *               Chassis_Follow_Update(delta);  // 更新底盘跟随
 *               
 *               osDelay(20);
 *           }
 *       }
 */
Chassis_Follow_State_t Chassis_Follow_Update(float delta_yaw);

/**
 * @brief 获取BigYaw补偿编码器值（用于反馈补偿）
 * @return 补偿编码器值（应从实际编码器中减去）
 * 
 * @note 【使用方式变更】
 *       旧：float target_big = Virtual_Yaw_GetTarget_Big() + Chassis_Follow_GetTarget_Big();
 *       新：float real_big_compensated = real_big - Chassis_Follow_GetTarget_Big();
 * 
 * @note 数学等价性：
 *       补偿目标：error = (V + C) - R
 *       补偿反馈：error = V - (R - C)
 *       两者相等，但后者更利于模块解耦
 */
float Chassis_Follow_GetTarget_Big(void);

/**
 * @brief 获取SmallYaw目标值
 * @return 始终返回0（SmallYaw由virtual_position独立控制）
 * 
 * @note 为了保持接口一致性，实际SmallYaw控制使用Virtual_Yaw_GetTarget_Small()
 */
float Chassis_Follow_GetTarget_Small(void);

/**
 * @brief 获取累计转动角度
 * @return 底盘累计转动的角度（相对于开机时刻，-180~180）
 */
float Chassis_Follow_GetTotalAngle(void);

/**
 * @brief 获取当前delta值
 * @return 本次更新的delta_yaw
 */
float Chassis_Follow_GetDelta(void);

/**
 * @brief 检查模块是否已准备好
 * @return 1=已准备好，0=未准备好
 */
uint8_t Chassis_Follow_IsReady(void);

/**
 * @brief 获取完整状态结构体
 * @return 状态结构体指针
 */
const Chassis_Follow_State_t* Chassis_Follow_GetState(void);

/**
 * @brief 重置模块（清零累计角度）
 */
void Chassis_Follow_Reset(void);

#endif // __CHASSIS_FOLLOW_H
