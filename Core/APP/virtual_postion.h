#ifndef __VIRTUAL_POSTION_H
#define __VIRTUAL_POSTION_H

#include <stdint.h>

//=========================== 外部变量 ===========================//

// 机械中值（定义在freertos.c）
extern int16_t origin_BigYaw_count;
extern int16_t origin_SmallYaw_count;

//=========================== 配置 ===========================//

#define VIRTUAL_RC_CHANNEL      2       // 遥控器通道ch[2]
#define VIRTUAL_RC_SENS         0.15f    // 虚拟坐标灵敏度
#define VIRTUAL_RC_DEADZONE     50.0f   // 遥控器死区（防止微小抖动）
// #define VIRTUAL_LIMIT           4000.0f // 虚拟坐标限幅（SmallYaw最大偏角对应编码值）

//=========================== 虚拟坐标数据结构 ===========================//

typedef struct {
    // 虚拟坐标（核心）
    float virtual_coord;            // 总虚拟坐标（目标朝向总量）
    float small_part;               // 小Yaw当前承担的偏量（持久化状态）
    float big_yaw_vel;              // BigYaw 实时速度分量（用于平滑跟随，产生“摆尾”感）
    
    // 实际编码（输入）
    float real_small_now;           // SmallYaw当前实际编码
    float real_big_now;             // BigYaw当前实际编码
    
    // 目标编码（输出，供控制层使用）
    float target_small;             // SmallYaw目标编码（计算结果）
    float target_big;               // BigYaw目标编码（计算结果）
    
    // 状态（供观察）
    float error_small;              // SmallYaw目标-当前误差
    float error_big;                // BigYaw目标-当前误差
    
} Virtual_Yaw_State_t;

//=========================== 接口 ===========================//

// 初始化（开机时调用一次）
void Virtual_Yaw_Init(void);

// 更新（控制周期调用）
// 输入：遥控器值、当前实际编码
// 输出：更新后的目标编码
void Virtual_Yaw_Update(int16_t rc_value, float real_small, float real_big);

// 获取目标（控制层调用）
float Virtual_Yaw_GetTarget_Small(void);
float Virtual_Yaw_GetTarget_Big(void);

#endif // __VIRTUAL_POSTION_H
