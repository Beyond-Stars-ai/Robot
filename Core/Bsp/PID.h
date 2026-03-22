#ifndef __PID_H__
#define __PID_H__

#include <math.h>

typedef struct
{
    float Need_Value; // 预期值
    float Now_Value;  // 当前值
    float OUT;        // 总输出
    float OUT_Min;    // 输出限幅下限
    float OUT_Max;    // 输出限幅上限

    float Ek;      // 本次误差
    float Ek_Last; // 上一次误差
    float Ek_Sum;  // 误差积分
    float Ek_Sum_Max;
    float Ek_Sum_Min;

    float Kp; // Kp
    float Ki; // Ki
    float Kd; // Kd

    float Ek_Del; // 误差差分
    float Ek_Min; // 误差为0阈值下限
    float Ek_Max; // 误差为0阈值上限

    float P_OUT; // 比例输出
    float I_OUT; // 积分输出
    float D_OUT; // 微分输出

} PID_PositionInitTypedef; // 位置式PID参数结构体

typedef struct
{
    float Need_Value; // 预期值
    float Now_Value;  // 当前值

    float Ek;       // 本次误差
    float Ek_Last;  // 上一次误差
    float Ek_Last2; // 上两次误差
    float Ek_Min;   // 误差为0阈值下限
    float Ek_Max;   // 误差为0阈值上限

    float Kp; // Kp
    float Ki; // Ki
    float Kd; // Kd

    float P_OUT;              // 比例输出
    float I_OUT;              // 积分输出
    float D_OUT;              // 微分输出
    float OUT;                // 总输出
    float OUT_Min;            // 输出限幅下限
    float OUT_Max;            // 输出限幅上限
} PID_IncrementalInitTypedef; // 增量式PID参数结构体

void PID_PositionStructureInit(PID_PositionInitTypedef *PID_InitStructure, float NeedValue);             // 位置式PID初始化结构体
void PID_PositionSetParameter(PID_PositionInitTypedef *PID_InitStructure, float kp, float ki, float kd); // 位置式PID设置参数
void PID_PositionSetEkRange(PID_PositionInitTypedef *PID_InitStructure, float ek_low, float ek_up);      // 位置式PID设置误差为0阈值
void PID_PositionSetOUTRange(PID_PositionInitTypedef *PID_InitStructure, float out_low, float out_up);   // 位置式PID设置输出限幅
void PID_PositionSetNeedValue(PID_PositionInitTypedef *PID_InitStructure, float NeedValue);
void PID_PositionClean(PID_PositionInitTypedef *PID_InitStructure);                // 位置式PID清理
void PID_PositionCalc(PID_PositionInitTypedef *PID_InitStructure, float NowValue); // 位置式PID计算

void PID_PositionCalc_Encoder(PID_PositionInitTypedef *PID_InitStructure, float NowValue);
void PID_PositionCalc_IMU(PID_PositionInitTypedef *PID_InitStructure, float NowValue);

void PID_IncrementalStructureInit(PID_IncrementalInitTypedef *PID_InitStructure, float NeedValue);             // 增量式PID初始化结构体
void PID_IncrementalSetParameter(PID_IncrementalInitTypedef *PID_InitStructure, float kp, float ki, float kd); // 增量式PID设置参数
void PID_IncrementalSetEkRange(PID_IncrementalInitTypedef *PID_InitStructure, float ek_low, float ek_up);      // 增量式PID设置误差为0阈值
void PID_IncrementalSetOUTRange(PID_IncrementalInitTypedef *PID_InitStructure, float out_low, float out_up);   // 增量式PID设置输出限幅
void PID_IncrementalCalc(PID_IncrementalInitTypedef *PID_InitStructure, float NowValue);                       // 增量式PID计算

#endif
