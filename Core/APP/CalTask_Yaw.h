#ifndef __CALTASK_YAW_H
#define __CALTASK_YAW_H

#include <stdint.h>

//=========================== 数据结构 ===========================//
//
// CalTask专用：10ms周期计算Yaw变化量
// 输入：当前Yaw角度
// 输出：10ms间隔的变化量（后-前）
//
//===========================

typedef struct {
    float yaw_now;              // 当前Yaw值（本次）
    float yaw_prev;             // 上一次Yaw值（10ms前）
    float delta_yaw;            // 变化量 = yaw_now - yaw_prev（处理±180°跨越）
    float angular_velocity;     // 角速度（度/秒）
    uint8_t data_valid;         // 数据有效标志（1=已计算过差值）
} CalTask_YawData_t;

//=========================== 接口函数 ===========================//

/**
 * @brief 更新Yaw数据并计算变化量（在10ms周期的CalTask中调用）
 * @param yaw_now 当前Yaw角度（度，-180~180）
 * @return 更新后的数据结构
 * 
 * @note 调用周期：10ms（与CalTask同步）
 * @note 首次调用会记录初始值，不计算差值
 * 
 * @example
 *       void StartCalTask(void *argument)
 *       {
 *           for (;;)
 *           {
 *               // 读取IMU或编码器Yaw值
 *               float yaw = Get_Yaw_Angle();
 *               
 *               // 计算变化量
 *               CalTask_Yaw_Update(yaw);
 *               
 *               // 传递给Chassis_Follow（后续步骤）
 *               // ChassisFollow_UpdateDelta(CalTask_Yaw_GetDelta());
 *               
 *               osDelay(10);
 *           }
 *       }
 */
CalTask_YawData_t CalTask_Yaw_Update(float yaw_now);

/**
 * @brief 获取Yaw变化量（10ms间隔）
 * @return 变化量（度），正值表示顺时针，负值表示逆时针
 */
float CalTask_Yaw_GetDelta(void);

/**
 * @brief 获取角速度
 * @return 角速度（度/秒）
 */
float CalTask_Yaw_GetAngularVelocity(void);

/**
 * @brief 获取当前Yaw值
 * @return 当前Yaw角度
 */
float CalTask_Yaw_GetNow(void);

/**
 * @brief 获取上一次Yaw值（10ms前）
 * @return 上一次Yaw角度
 */
float CalTask_Yaw_GetPrev(void);

/**
 * @brief 检查数据是否有效
 * @return 1=有效（已计算过至少一次差值），0=无效（首次运行）
 */
uint8_t CalTask_Yaw_IsValid(void);

/**
 * @brief 获取完整数据结构
 * @return 数据结构指针
 */
const CalTask_YawData_t* CalTask_Yaw_GetData(void);

/**
 * @brief 重置模块
 */
void CalTask_Yaw_Reset(void);

#endif // __CALTASK_YAW_H
