#include "Gimbal_Control.h"
#include "can.h"
#include <math.h>

//=========================== PID定义 ===========================//

PID_PositionInitTypedef Pitch_PositionPID;
PID_PositionInitTypedef Pitch_SpeedPID;

//=========================== 初始化 ===========================//

void Gimbal_Pitch_Init(void)
{
  PID_PositionStructureInit (&Pitch_PositionPID,4074);        //外环位置环
  PID_PositionSetParameter  (&Pitch_PositionPID,0.5,0,0);
  PID_PositionSetOUTRange   (&Pitch_PositionPID,-400,400);

  PID_PositionStructureInit (&Pitch_SpeedPID,0);              //内环速度环
  PID_PositionSetParameter  (&Pitch_SpeedPID,50,0,0);
  PID_PositionSetOUTRange   (&Pitch_SpeedPID,-20000,20000);
  PID_PositionSetEkRange    (&Pitch_SpeedPID, -3.0f, 3.0f);
}

void Gimbal_Control_Init(void)
{
    Gimbal_Pitch_Init();
}

//=========================== Pitch控制（回退到原版）===========================//

void Gimbal_Pitch_Control(void)
{
    // ============更新位置目标（仅打杆时）============
        Pitch_PositionPID.Need_Value -= 0.01f * global_rc_control.rc.ch[3];

        // 限幅 [0, 4848]
        if (Pitch_PositionPID.Need_Value > 4500.0f)
            Pitch_PositionPID.Need_Value = 4500.0f;
        else if (Pitch_PositionPID.Need_Value < 3300.0f)
            Pitch_PositionPID.Need_Value = 3300.0f;

    // ============位置环计算=========================
    PID_PositionCalc(&Pitch_PositionPID, Can1_M6020_MotorStatus[1].Position);


    // ============速度环计算=========================
    PID_PositionSetNeedValue(&Pitch_SpeedPID, Pitch_PositionPID.OUT);//
    PID_PositionCalc(&Pitch_SpeedPID, Can1_M6020_MotorStatus[1].Speed);

    // ============发送输出===========================
    Motor_6020_Voltage1(0, (int16_t)Pitch_SpeedPID.OUT, 0, 0, &hcan1);
}

//=========================== 总控制循环 ===========================//

void Gimbal_Control_Loop(void)
{
    Gimbal_Pitch_Control();
    // Yaw轴后续添加
}

//=========================== PID调试接口 ===========================//

void Gimbal_Pitch_SetPosPID(float kp, float ki, float kd)
{
    PID_PositionSetParameter(&Pitch_PositionPID, kp, ki, kd);
}

void Gimbal_Pitch_SetSpeedPID(float kp, float ki, float kd)
{
    PID_PositionSetParameter(&Pitch_SpeedPID, kp, ki, kd);
}
