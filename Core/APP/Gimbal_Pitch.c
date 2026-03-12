#include "Gimbal_Pitch.h"
#define PITCH_MID 2424          //小yaw轴中位值
#define PITCH_LEFT 1650          //小yaw轴左侧最大偏移
#define PITCH_RIGHT 1650          //小yaw轴右侧最大偏移（顺时针减小）

PID_PositionInitTypedef Pitch_PositionPID;
PID_PositionInitTypedef Pitch_SpeedPID;
extern M6020_Motor Can1_M6020_MotorStatus[7];//GM6020电机状态数组
extern M6020_Motor Can2_M6020_MotorStatus[7];//GM6020电机状态数组
// extern RC_ctrl_t *local_rc_ctrl;
extern RC_ctrl_t global_rc_control; // 全局遥控器数据



void Gimbal_Pitch_Init(void)
{
	PID_PositionStructureInit (&Pitch_PositionPID,4074);        //外环位置环
  PID_PositionSetParameter  (&Pitch_PositionPID,0.5,0,0);
  PID_PositionSetOUTRange   (&Pitch_PositionPID,-400,400);
  // PID_PositionSetNeedValueRange(&Pitch_PositionPID,4848,0);

	PID_PositionStructureInit (&Pitch_SpeedPID,0);              //内环速度环
  PID_PositionSetParameter  (&Pitch_SpeedPID,50,0,0);
  PID_PositionSetOUTRange   (&Pitch_SpeedPID,-20000,20000);
  PID_PositionSetEkRange    (&Pitch_SpeedPID, -3.0f, 3.0f);
}

// void Gimbal_Pitch_Control(void)
// {
//     // ============更新位置目标（仅打杆时）============
//         Pitch_PositionPID.Need_Value -= 0.01f * local_rc_ctrl->rc.ch[3];

//         // 限幅 [0, 4848]
//         if (Pitch_PositionPID.Need_Value > 4500.0f)
//             Pitch_PositionPID.Need_Value = 4500.0f;
//         else if (Pitch_PositionPID.Need_Value < 3300.0f)
//             Pitch_PositionPID.Need_Value = 3300.0f;

//     // ============位置环计算=========================
//     PID_PositionCalc(&Pitch_PositionPID, Can1_M6020_MotorStatus[1].Position);


//     // ============速度环计算=========================
//     PID_PositionSetNeedValue(&Pitch_SpeedPID, Pitch_PositionPID.OUT);//
//     PID_PositionCalc(&Pitch_SpeedPID, Can1_M6020_MotorStatus[1].Speed);

//     // ============发送输出===========================
//     Motor_6020_Voltage1(0, (int16_t)Pitch_SpeedPID.OUT, 0, 0, &hcan1);
// }


