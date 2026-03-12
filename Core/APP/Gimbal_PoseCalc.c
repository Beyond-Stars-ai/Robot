#include "Gimbal_PoseCalc.h"
#include "BMI088.h"
#include "Motor.h"
#include "bsp_can.h"

extern M3508_Motor Can1_M3508_MotorStatus[8];//M3508电机状态数组
extern M3508_Motor Can2_M3508_MotorStatus[8];//M3508电机状态数组
extern M6020_Motor Can1_M6020_MotorStatus[7];//GM6020电机状态数组
extern M6020_Motor Can2_M6020_MotorStatus[7];//GM6020电机状态数组
extern M2006_Motor Can1_M2006_MotorStatus[8];//M2006电机状态数组
extern M2006_Motor Can2_M2006_MotorStatus[8];//M2006电机状态数组

BMI088_Init_typedef BMI088_Data;					//自己的陀螺仪数据
BMI088_Init_typedef Can_BMI088_Data;			//can总线收的陀螺仪数据
BMI088_Init_typedef BigYaw_BMI088_Data;		//大yaw轴解算的陀螺仪数据
BMI088_Init_typedef SmallYaw_BMI088_Data;	//小yaw轴解算的陀螺仪数据

#define BIGYAW_OFFSET 186.0f
#define SMALLYAW_OFFSET 106.5f



void Gimbal_PoseCalc(void)
{
  //获取陀螺仪数据
  BMI088_GetData(&Can_BMI088_Data);
  
  //计算姿态
	Can_BMI088_Data.Yaw = -Can_BMI088_Data.Yaw;
	
  BigYaw_BMI088_Data.Yaw = -(Can_BMI088_Data.Yaw + (Can2_M6020_MotorStatus[0].ANgle )); // + BIGYAW_OFFSET
	if(BigYaw_BMI088_Data.Yaw > 180)				BigYaw_BMI088_Data.Yaw -=360;
	else if(BigYaw_BMI088_Data.Yaw < -180)	BigYaw_BMI088_Data.Yaw +=360;
  
  SmallYaw_BMI088_Data.Yaw = BigYaw_BMI088_Data.Yaw + (Can2_M6020_MotorStatus[1].ANgle ); // - SMALLYAW_OFFSET
	if(SmallYaw_BMI088_Data.Yaw > 180)				SmallYaw_BMI088_Data.Yaw -=360;
	else if(SmallYaw_BMI088_Data.Yaw < -180)	SmallYaw_BMI088_Data.Yaw +=360;
}
  