#include "Gimbal_Trigger.h"

PID_PositionInitTypedef Trigger_SpeedPID;
PID_PositionInitTypedef Trigger_PositionPID;
extern M2006_Motor Can1_M2006_MotorStatus[8];//M2006电机状态数组
extern M2006_Motor Can2_M2006_MotorStatus[8];//M2006电机状态数组
// extern RC_ctrl_t *local_rc_ctrl;
extern RC_ctrl_t global_rc_control;
extern uint8_t Gimbal_Shoot_Flag;

void Gimbal_Trigger_Init()
{
  PID_PositionStructureInit (&Trigger_SpeedPID,0);              //拨弹盘速度环
  PID_PositionSetParameter  (&Trigger_SpeedPID,20,0,0);
  PID_PositionSetOUTRange   (&Trigger_SpeedPID,-10000,10000);
  PID_PositionSetEkRange    (&Trigger_SpeedPID, -3.0f, 3.0f);
}


void Gimbal_Trigger_Control()
{
		if(global_rc_control.rc.s[1] == 0x02 && Gimbal_Shoot_Flag )
		{
			PID_PositionSetNeedValue(&Trigger_SpeedPID, 2000);
		}
		else if(global_rc_control.rc.s[1] == 0x01)
		{
			PID_PositionSetNeedValue(&Trigger_SpeedPID, -2000);
		}
		else
		{
			PID_PositionSetNeedValue(&Trigger_SpeedPID, 0);
			PID_PositionClean(&Trigger_SpeedPID);	//位置式PID清理
		}
		PID_PositionCalc(&Trigger_SpeedPID, Can1_M2006_MotorStatus[6].RotorSpeed);
		Motor_2006_Current2(0,0,(int16_t)Trigger_SpeedPID.OUT,0,&hcan1);
}


