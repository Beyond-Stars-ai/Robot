#include "Gimbal_Yaw_Small.h"
#define SMALLYAW_MID 2424          //小yaw轴中位值
#define SMALLYAW_LEFT 1650          //小yaw轴左侧最大偏移
#define SMALLYAW_RIGHT 1650          //小yaw轴右侧最大偏移（顺时针减小）

PID_PositionInitTypedef SmallYaw_GyroscopePID;
PID_PositionInitTypedef SmallYaw_PositionPID;
PID_PositionInitTypedef SmallYaw_SpeedPID;
extern BMI088_Init_typedef BMI088_Data;
extern BMI088_Init_typedef Can_BMI088_Data;
extern BMI088_Init_typedef BigYaw_BMI088_Data;
extern BMI088_Init_typedef SmallYaw_BMI088_Data;
extern M6020_Motor Can1_M6020_MotorStatus[7];//GM6020电机状态数组
extern M6020_Motor Can2_M6020_MotorStatus[7];//GM6020电机状态数组
// extern RC_ctrl_t *local_rc_ctrl;

void Gimbal_YawSmall_Init(void)
{	
   PID_PositionStructureInit (&SmallYaw_GyroscopePID,0);        //外环小yaw角度环
   PID_PositionSetParameter  (&SmallYaw_GyroscopePID,40,0,0);
   PID_PositionSetOUTRange   (&SmallYaw_GyroscopePID,-20000,20000);

	PID_PositionStructureInit (&SmallYaw_SpeedPID,0);              //内环速度环
   PID_PositionSetParameter  (&SmallYaw_SpeedPID,100,0,0);
   PID_PositionSetOUTRange   (&SmallYaw_SpeedPID,-20000,20000);
   PID_PositionSetEkRange    (&SmallYaw_SpeedPID, -3.0f, 3.0f);
}

// void Gimbal_YawSmall_Control(void)
// {
// 	// ============获取目标角度============
// 	gyro_needvalue  -= 0.0007 * local_rc_ctrl->rc.ch[2]; //-= 0.0007 * local_rc_ctrl->rc.ch[2];// = 0;
// 	if(gyro_needvalue > 180) gyro_needvalue -=360 ;
// 	else if(gyro_needvalue < -180) gyro_needvalue +=360 ;
	
// 	adjustAngle3(gyro_needvalue , SmallYaw_BMI088_Data.Yaw , &SmallYaw_GyroscopePID.Need_Value);
	
// //	SmallYaw_GyroscopePID.Need_Value  -= 0.0007 * local_rc_ctrl->rc.ch[2]; //-= 0.0007 * local_rc_ctrl->rc.ch[2];// = 0;
// //	if(SmallYaw_GyroscopePID.Need_Value > 180) SmallYaw_GyroscopePID.Need_Value -=360 ;
// //	else if(SmallYaw_GyroscopePID.Need_Value < -180) SmallYaw_GyroscopePID.Need_Value +=360 ;
// 	// ============角度环计算============
// 	PID_PositionCalc_IMU(&SmallYaw_GyroscopePID, SmallYaw_BMI088_Data.Yaw);
// 	// ============ 速度环计算 =========================
// 	PID_PositionSetNeedValue(&SmallYaw_SpeedPID, SmallYaw_GyroscopePID.OUT);//SmallYaw_GyroscopePID.OUT
// 	PID_PositionCalc(&SmallYaw_SpeedPID, Can2_M6020_MotorStatus[1].Speed);
	
// 	//====新加的
// 	// 【相对限位】基于开机位置x的限位区域
// 	// 右限位区：[x-74, x+8]，强制输出+400（向右）
// 	// 左限位区：[x-154, x-74]，强制输出-400（向左）
// 	float angle = Can2_M6020_MotorStatus[1].ANgle;
// 	if(angle > smallyaw_x - 74 && angle < smallyaw_x + 8) {
// 		SmallYaw_SpeedPID.OUT = 400;
// 	}
// 	else if(angle > smallyaw_x - 154 && angle < smallyaw_x - 74) {
// 		SmallYaw_SpeedPID.OUT = -400;
// 	}
// 	//====
// 	// ============ 发送输出 ===========================
// //    Motor_6020_Voltage1			(0, (int16_t)SmallYaw_SpeedPID.OUT, 0, 0, &hcan2);
// }

