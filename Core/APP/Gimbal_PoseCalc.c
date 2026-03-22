#include "Gimbal_PoseCalc.h"
#include "BMI088.h"
#include "Motor.h"

//=========================== 全局变量定义 ===========================//

Gimbal_Absolute_Angle_t BigYaw_Absolute = {0};
Gimbal_Absolute_Angle_t SmallYaw_Absolute = {0};
float Chassis_IMU_Yaw = 0.0f;

// 外部引用
extern BMI088_Init_typedef Can_BMI088_Data;  // 底盘IMU数据（通过CAN接收）
extern M6020_Motor Can2_M6020_MotorStatus[7]; // 电机状态

//=========================== 初始化 ===========================//

void Gimbal_PoseCalc_Init(void)
{
    BigYaw_Absolute.Yaw = 0.0f;
    BigYaw_Absolute.Pitch = 0.0f;
    BigYaw_Absolute.Roll = 0.0f;
    BigYaw_Absolute.r = 0;
    
    SmallYaw_Absolute.Yaw = 0.0f;
    SmallYaw_Absolute.Pitch = 0.0f;
    SmallYaw_Absolute.Roll = 0.0f;
    SmallYaw_Absolute.r = 0;
    
    Chassis_IMU_Yaw = 0.0f;
}

//=========================== 核心姿态计算 ===========================//

void Gimbal_PoseCalc_Update(void)
{
    //---------- 1. 获取底盘IMU数据（通过CAN从底盘接收）----------
    // 注意：Can_BMI088_Data需要在其他地方定义和更新
    float chassis_yaw = Can_BMI088_Data.Yaw;
    
    // 取反（坐标系对齐）
    Chassis_IMU_Yaw = -chassis_yaw;
    
    //---------- 2. 获取BigYaw电机编码器角度（SmallYaw无需计算）----------
    float bigyaw_motor_angle = Encoder_To_Angle(Can2_M6020_MotorStatus[0].Angle);
    // 注意：SmallYaw 在哨兵模式下锁定编码器，不需要角度计算
    
    //---------- 3. 计算大Yaw绝对角度（蓝图核心公式）----------
    // BigYaw绝对角度 = -(底盘IMU_Yaw + 电机编码器角度 + 偏移)
    float bigyaw_abs = -(Chassis_IMU_Yaw + bigyaw_motor_angle + BIGYAW_ANGLE_OFFSET);
    
    // 归一化到-180~180
    BigYaw_Absolute.Yaw = Angle_Normalize(bigyaw_abs);
    BigYaw_Absolute.Raw_Yaw = bigyaw_abs;
    
    // 计算圈数（用于调试）
    static float last_bigyaw = 0;
    float diff = BigYaw_Absolute.Yaw - last_bigyaw;
    if (diff > 180.0f) BigYaw_Absolute.r--;
    else if (diff < -180.0f) BigYaw_Absolute.r++;
    last_bigyaw = BigYaw_Absolute.Yaw;
}
