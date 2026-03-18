#include "PID.h"
#define Motor_ECD_MAX 8192.0f
#define IMU_Circle_MAX 360.0f

/*
 *函数简介:位置式PID初始化结构体
 *参数说明:位置式PID参数结构体
 *参数说明:预期值
 *返回类型:无
 *备注:无
 */
void PID_PositionStructureInit(PID_PositionInitTypedef* pid, float NeedValue)
{
    pid->Need_Value = NeedValue;
    pid->Now_Value = 0;
    pid->Ek = 0;
    pid->Ek_Last = 0;
    pid->Ek_Sum = 0;
    pid->Ek_Del = 0;

    // PID 参数
    pid->Kp = 0;
    pid->Ki = 0;
    pid->Kd = 0;
		// 默认小死区	
    pid->Ek_Min = -0;   
    pid->Ek_Max = 0;
		// 输出限幅
    pid->OUT_Min = -65535.0f;  // 用户后续应覆盖
    pid->OUT_Max =  65535.0f;

    // 积分限幅
    pid->Ek_Sum_Min = -65535.0f;
    pid->Ek_Sum_Max =  65535.0f;

    // 输出清零
    pid->P_OUT = 0;
    pid->I_OUT = 0;
    pid->D_OUT = 0;
    pid->OUT = 0;
}

/*
 *函数简介:位置式PID设置参数
 *参数说明:位置式PID参数结构体
 *参数说明:单精度浮点型Kp
 *参数说明:单精度浮点型Ki
 *参数说明:单精度浮点型Kd
 *返回类型:无
 *备注:无
 */
void PID_PositionSetParameter(PID_PositionInitTypedef* PID_InitStructure,float kp,float ki,float kd)
{
	PID_InitStructure->Kp=kp;
	PID_InitStructure->Ki=ki;
	PID_InitStructure->Kd=kd;
}

/*
 *函数简介:位置式PID设置误差为0阈值
 *参数说明:位置式PID参数结构体
 *参数说明:误差为0阈值下限
 *参数说明:误差为0阈值上限
 *返回类型:无
 *备注:无
 */
void PID_PositionSetEkRange(PID_PositionInitTypedef* PID_InitStructure,float Ek_Min,float Ek_Max)
{
	PID_InitStructure->Ek_Min=Ek_Min;
	PID_InitStructure->Ek_Max=Ek_Max;
}

/*
 *函数简介:位置式PID设置输出限幅
 *参数说明:位置式PID参数结构体
 *参数说明:输出限幅下限
 *参数说明:输出限幅上限
 *返回类型:无
 *备注:无
 */
void PID_PositionSetOUTRange(PID_PositionInitTypedef* PID_InitStructure,float OUT_Min,float OUT_Max)
{
	PID_InitStructure->OUT_Min=OUT_Min;
	PID_InitStructure->OUT_Max=OUT_Max;
}

void PID_PositionSetNeedValue(PID_PositionInitTypedef* PID_InitStructure,float NeedValue)
{
	PID_InitStructure->Need_Value=NeedValue;
}

/*
 *函数简介:位置式PID清理
 *参数说明:位置式PID参数结构体
 *返回类型:无
 *备注:使Ek和Sum为0
 */
void PID_PositionClean(PID_PositionInitTypedef* PID_InitStructure)
{
	PID_InitStructure->Ek=0;
	PID_InitStructure->Ek_Sum=0;
}

/*
 *函数简介:位置式PID计算
 *参数说明:位置式PID参数结构体
 *参数说明:当前值
 *返回类型:无
 *备注:OUT=POUT+IOUT+DOUT=Kp*Ek+Ki*ΣEk+Kd*(Ek-Ek_Last)
 *备注:计算结果保存在位置式PID参数结构体中
 */
void PID_PositionCalc(PID_PositionInitTypedef* PID_InitStructure,float NowValue)
{	
	float eksum_speed = 1;//变速积分参数
	//===============更新基本数据===============//
	PID_InitStructure->Now_Value 	= NowValue;
	PID_InitStructure->Ek_Last 		= PID_InitStructure->Ek;
	PID_InitStructure->Ek 				= PID_InitStructure->Need_Value - PID_InitStructure->Now_Value;
	PID_InitStructure->Ek_Del 		= PID_InitStructure->Ek - PID_InitStructure->Ek_Last;//误差差分
	//===============误差死区===============//
	if(PID_InitStructure->Ek_Min < PID_InitStructure->Ek && PID_InitStructure->Ek < PID_InitStructure->Ek_Max)//误差为0检测
	{
		PID_InitStructure->Ek = 0;
	}
	//===============变速积分===============//
	eksum_speed = 1/( 1 * fabs(PID_InitStructure->Ek)+1);
	PID_InitStructure->Ek_Sum += eksum_speed*PID_InitStructure->Ek;
//	PID_InitStructure->Ek_Sum += PID_InitStructure->Ek;
	//===============误差积分限幅===============//
	if(PID_InitStructure->Ek_Sum > PID_InitStructure->Ek_Sum_Max)
		PID_InitStructure->Ek_Sum = PID_InitStructure->Ek_Sum_Max;
	if(PID_InitStructure->Ek_Sum < PID_InitStructure->Ek_Sum_Min)
		PID_InitStructure->Ek_Sum = PID_InitStructure->Ek_Sum_Min;
	//===============计算pid各项输出及总输出===============//
	PID_InitStructure->P_OUT = PID_InitStructure->Kp * PID_InitStructure->Ek;
	PID_InitStructure->I_OUT = PID_InitStructure->Ki * PID_InitStructure->Ek_Sum;
	PID_InitStructure->D_OUT = PID_InitStructure->Kd * PID_InitStructure->Ek_Del;
	PID_InitStructure->OUT	 = PID_InitStructure->P_OUT + PID_InitStructure->I_OUT + PID_InitStructure->D_OUT;
	//===============输出限幅===============//
	if(PID_InitStructure->OUT < PID_InitStructure->OUT_Min)//输出限幅
		PID_InitStructure->OUT = PID_InitStructure->OUT_Min;
	if(PID_InitStructure->OUT > PID_InitStructure->OUT_Max)
		PID_InitStructure->OUT = PID_InitStructure->OUT_Max;
}

void PID_PositionCalc_Encoder(PID_PositionInitTypedef* PID_InitStructure,float NowValue)
{	
	//===============更新基本数据===============//
	PID_InitStructure->Now_Value = NowValue;
	PID_InitStructure->Ek_Last = PID_InitStructure->Ek;
  float err = PID_InitStructure->Need_Value - PID_InitStructure->Now_Value;
  if (err > Motor_ECD_MAX/2) {          // 8192 / 2 = 4096
      err -= Motor_ECD_MAX;
  } else if (err < -Motor_ECD_MAX/2) {
      err += Motor_ECD_MAX;
  }
	PID_InitStructure->Ek = err;
	PID_InitStructure->Ek_Del = PID_InitStructure->Ek - PID_InitStructure->Ek_Last;//误差差分

	//===============误差死区===============//
	if(PID_InitStructure->Ek_Min < PID_InitStructure->Ek && PID_InitStructure->Ek < PID_InitStructure->Ek_Max)//误差为0检测
	{
		PID_InitStructure->Ek = 0;
	}	
	PID_InitStructure->Ek_Sum += PID_InitStructure->Ek;
	//===============误差积分限幅===============//
	if(PID_InitStructure->Ek_Sum > PID_InitStructure->Ek_Sum_Max)
		PID_InitStructure->Ek_Sum = PID_InitStructure->Ek_Sum_Max;
	if(PID_InitStructure->Ek_Sum < PID_InitStructure->Ek_Sum_Min)
		PID_InitStructure->Ek_Sum = PID_InitStructure->Ek_Sum_Min;
	//===============计算pid各项输出及总输出===============//
	PID_InitStructure->P_OUT = PID_InitStructure->Kp * PID_InitStructure->Ek;
	PID_InitStructure->I_OUT = PID_InitStructure->Ki * PID_InitStructure->Ek_Sum;
	PID_InitStructure->D_OUT = PID_InitStructure->Kd * PID_InitStructure->Ek_Del;
	PID_InitStructure->OUT	 = PID_InitStructure->P_OUT + PID_InitStructure->I_OUT + PID_InitStructure->D_OUT;
	
	//===============输出限幅===============//
	if(PID_InitStructure->OUT < PID_InitStructure->OUT_Min)//输出限幅
		PID_InitStructure->OUT = PID_InitStructure->OUT_Min;
	if(PID_InitStructure->OUT > PID_InitStructure->OUT_Max)
		PID_InitStructure->OUT = PID_InitStructure->OUT_Max;
}


void PID_PositionCalc_IMU(PID_PositionInitTypedef* PID_InitStructure,float NowValue)
{	
	//===============更新基本数据===============//
	PID_InitStructure->Now_Value = NowValue;
	PID_InitStructure->Ek_Last = PID_InitStructure->Ek;
  float err = PID_InitStructure->Need_Value - PID_InitStructure->Now_Value;
  if (err > IMU_Circle_MAX/2) {          // 8192 / 2 = 4096
      err -= IMU_Circle_MAX;
  } else if (err < -IMU_Circle_MAX/2) {
      err += IMU_Circle_MAX;
  }
	PID_InitStructure->Ek = err;
	PID_InitStructure->Ek_Del = PID_InitStructure->Ek - PID_InitStructure->Ek_Last;//误差差分

	//===============误差死区===============//
	if(PID_InitStructure->Ek_Min < PID_InitStructure->Ek && PID_InitStructure->Ek < PID_InitStructure->Ek_Max)//误差为0检测
	{
		PID_InitStructure->Ek = 0;
	}	
	PID_InitStructure->Ek_Sum += PID_InitStructure->Ek;
	//===============误差积分限幅===============//
	if(PID_InitStructure->Ek_Sum > PID_InitStructure->Ek_Sum_Max)
		PID_InitStructure->Ek_Sum = PID_InitStructure->Ek_Sum_Max;
	if(PID_InitStructure->Ek_Sum < PID_InitStructure->Ek_Sum_Min)
		PID_InitStructure->Ek_Sum = PID_InitStructure->Ek_Sum_Min;
	//===============计算pid各项输出及总输出===============//
	PID_InitStructure->P_OUT = PID_InitStructure->Kp * PID_InitStructure->Ek;
	PID_InitStructure->I_OUT = PID_InitStructure->Ki * PID_InitStructure->Ek_Sum;
	PID_InitStructure->D_OUT = PID_InitStructure->Kd * PID_InitStructure->Ek_Del;
	PID_InitStructure->OUT	 = PID_InitStructure->P_OUT + PID_InitStructure->I_OUT + PID_InitStructure->D_OUT;
	
	//===============输出限幅===============//
	if(PID_InitStructure->OUT < PID_InitStructure->OUT_Min)//输出限幅
		PID_InitStructure->OUT = PID_InitStructure->OUT_Min;
	if(PID_InitStructure->OUT > PID_InitStructure->OUT_Max)
		PID_InitStructure->OUT = PID_InitStructure->OUT_Max;
}


/*
 *函数简介:增量式PID初始化结构体
 *参数说明:增量式PID参数结构体
 *参数说明:预期值
 *返回类型:无
 *备注:无
 */
void PID_IncrementalStructureInit(PID_IncrementalInitTypedef* PID_InitStructure,float NeedValue)
{
	PID_InitStructure->Need_Value=NeedValue;
	PID_InitStructure->Ek=0;
	PID_InitStructure->Ek_Last=0;
	PID_InitStructure->Ek_Min=0;
	PID_InitStructure->Ek_Max=0;
	PID_InitStructure->Kp=0;
	PID_InitStructure->Ki=0;
	PID_InitStructure->Kd=0;
	PID_InitStructure->OUT_Min=-65535;
	PID_InitStructure->OUT_Max=65535;
}

/*
 *函数简介:增量式PID设置参数
 *参数说明:增量式PID参数结构体
 *参数说明:单精度浮点型Kp
 *参数说明:单精度浮点型Ki
 *参数说明:单精度浮点型Kd
 *返回类型:无
 *备注:无
 */
void PID_IncrementalSetParameter(PID_IncrementalInitTypedef* PID_InitStructure,float kp,float ki,float kd)
{
	PID_InitStructure->Kp=kp;
	PID_InitStructure->Ki=ki;
	PID_InitStructure->Kd=kd;
}

/*
 *函数简介:增量式PID设置误差为0阈值
 *参数说明:增量式PID参数结构体
 *参数说明:误差为0阈值下限
 *参数说明:误差为0阈值上限
 *返回类型:无
 *备注:无
 */
void PID_IncrementalSetEkRange(PID_IncrementalInitTypedef* PID_InitStructure,float Ek_Min,float Ek_Max)
{
	PID_InitStructure->Ek_Min=Ek_Min;
	PID_InitStructure->Ek_Max=Ek_Max;
}

/*
 *函数简介:增量式PID设置输出限幅
 *参数说明:增量式PID参数结构体
 *参数说明:输出限幅下限
 *参数说明:输出限幅上限
 *返回类型:无
 *备注:无
 */
void PID_IncrementalSetOUTRange(PID_IncrementalInitTypedef* PID_InitStructure,float OUT_Min,float OUT_Max)
{
	PID_InitStructure->OUT_Min=OUT_Min;
	PID_InitStructure->OUT_Max=OUT_Max;
}

/*
 *函数简介:增量式PID计算
 *参数说明:增量式PID参数结构体
 *参数说明:当前值
 *返回类型:无
 *备注:OUT=POUT+IOUT+DOUT=Kp*ΔEk+Ki*ΣΔEk+Kd*(ΔEk-ΔEk_Last)=Kp*(Ek-Ek_Last)+Ki*Ek+Kd*(Ek-2*Ek_Last+Ek_Last2)
 *备注:计算结果保存在增量式PID参数结构体中
 */
void PID_IncrementalCalc(PID_IncrementalInitTypedef* PID_InitStructure,float NowValue)
{
	PID_InitStructure->Now_Value	=NowValue;
	PID_InitStructure->Ek_Last2		=PID_InitStructure->Ek_Last;
	PID_InitStructure->Ek_Last		=PID_InitStructure->Ek;
	PID_InitStructure->Ek					=PID_InitStructure->Need_Value-PID_InitStructure->Now_Value;
	
	if(PID_InitStructure->Ek_Min < PID_InitStructure->Ek && PID_InitStructure->Ek < PID_InitStructure->Ek_Max)//误差为0检测
		PID_InitStructure->Ek				=0;

	PID_InitStructure->P_OUT	=PID_InitStructure->Kp * (PID_InitStructure->Ek - PID_InitStructure->Ek_Last);
	PID_InitStructure->I_OUT	=PID_InitStructure->Ki * PID_InitStructure->Ek;
	PID_InitStructure->D_OUT	=PID_InitStructure->Kd * (PID_InitStructure->Ek-2*PID_InitStructure->Ek_Last+PID_InitStructure->Ek_Last2);
	PID_InitStructure->OUT		=PID_InitStructure->P_OUT + PID_InitStructure->I_OUT+PID_InitStructure->D_OUT;
	
	if(PID_InitStructure->OUT<PID_InitStructure->OUT_Min)//输出限幅
		PID_InitStructure->OUT=PID_InitStructure->OUT_Min;
	if(PID_InitStructure->OUT>PID_InitStructure->OUT_Max)
		PID_InitStructure->OUT=PID_InitStructure->OUT_Max;
}
