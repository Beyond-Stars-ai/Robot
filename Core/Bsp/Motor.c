#include "Motor.h"

M3508_Motor Can1_M3508_MotorStatus[8]; // M3508电机状态数组
M3508_Motor Can2_M3508_MotorStatus[8]; // M3508电机状态数组
M6020_Motor Can1_M6020_MotorStatus[7]; // GM6020电机状态数组
M6020_Motor Can2_M6020_MotorStatus[7]; // GM6020电机状态数组
M2006_Motor Can1_M2006_MotorStatus[8]; // M2006电机状态数组
M2006_Motor Can2_M2006_MotorStatus[8]; // M2006电机状态数组
#define MOTOR_ENCODER_ECD_MAX 8192.0f
#define MOTOR_ENCODER_HALF_ECD_MAX 4096.0f

void Motor_3508_Current1(int16_t device1, int16_t device2,
                         int16_t device3, int16_t device4,
                         CAN_HandleTypeDef *hcan) // CANID:0x200 电机ID:1-4 (-16384,16384)
{
    CAN_TxHeaderTypeDef Tx_Message;
    uint8_t Can_Send_Data[8];
    uint32_t send_mail_box;

    Tx_Message.StdId = 0x200;
    Tx_Message.IDE = CAN_ID_STD;
    Tx_Message.RTR = CAN_RTR_DATA;
    Tx_Message.DLC = 0x08;
    Can_Send_Data[0] = device1 >> 8;
    Can_Send_Data[1] = device1;
    Can_Send_Data[2] = device2 >> 8;
    Can_Send_Data[3] = device2;
    Can_Send_Data[4] = device3 >> 8;
    Can_Send_Data[5] = device3;
    Can_Send_Data[6] = device4 >> 8;
    Can_Send_Data[7] = device4;

    HAL_CAN_AddTxMessage(hcan, &Tx_Message, Can_Send_Data, &send_mail_box);
}

void Motor_3508_Current2(int16_t device1, int16_t device2,
                         int16_t device3, int16_t device4,
                         CAN_HandleTypeDef *hcan) // CANID:0x1FF 电机ID:5-8 (-16384,16384)
{
    CAN_TxHeaderTypeDef Tx_Message;
    uint8_t Can_Send_Data[8];
    uint32_t send_mail_box;

    Tx_Message.StdId = 0x1FF;
    Tx_Message.IDE = CAN_ID_STD;
    Tx_Message.RTR = CAN_RTR_DATA;
    Tx_Message.DLC = 0x08;
    Can_Send_Data[0] = device1 >> 8;
    Can_Send_Data[1] = device1;
    Can_Send_Data[2] = device2 >> 8;
    Can_Send_Data[3] = device2;
    Can_Send_Data[4] = device3 >> 8;
    Can_Send_Data[5] = device3;
    Can_Send_Data[6] = device4 >> 8;
    Can_Send_Data[7] = device4;

    HAL_CAN_AddTxMessage(hcan, &Tx_Message, Can_Send_Data, &send_mail_box);
}

void CAN1_M3508_DataProcess(M3508_ID ID, uint8_t *Data)
{
    uint16_t M3508_RotorNowAngle = (uint16_t)((((uint16_t)Data[0]) << 8) | Data[1]); // 本次转子机械角度原始数据
    if (M3508_RotorNowAngle - Can1_M3508_MotorStatus[ID - 0x201].RawRotorAngle > 4000 && Can1_M3508_MotorStatus[ID - 0x201].First_Flag == 1)
        Can1_M3508_MotorStatus[ID - 0x201].Rotor_r--; // 本次转子机械角度原始数据和上次转子机械角度原始数据出现跃变
    else if (Can1_M3508_MotorStatus[ID - 0x201].RawRotorAngle - M3508_RotorNowAngle > 4000 && Can1_M3508_MotorStatus[ID - 0x201].First_Flag == 1)
        Can1_M3508_MotorStatus[ID - 0x201].Rotor_r++;
    else if (Can1_M3508_MotorStatus[ID - 0x201].First_Flag != 1)
        Can1_M3508_MotorStatus[ID - 0x201].First_Flag = 1;

    Can1_M3508_MotorStatus[ID - 0x201].RawRotorAngle = M3508_RotorNowAngle;                                                                                 // 转子机械角度原始数据
    Can1_M3508_MotorStatus[ID - 0x201].RotorAngle = M3508_RotorNowAngle * 0.0439453125f;                                                                    //=M3508_RotorNowAngle/8192.0f*360.0f;//转子机械角度
    Can1_M3508_MotorStatus[ID - 0x201].RawRotorPosition = 8192 * Can1_M3508_MotorStatus[ID - 0x201].Rotor_r + M3508_RotorNowAngle;                          // 转子角度位置原始数据
    Can1_M3508_MotorStatus[ID - 0x201].RotorPosition = 360.0f * Can1_M3508_MotorStatus[ID - 0x201].Rotor_r + Can1_M3508_MotorStatus[ID - 0x201].RotorAngle; // 转子角度位置
    Can1_M3508_MotorStatus[ID - 0x201].RotorSpeed = (int16_t)((((uint16_t)Data[2]) << 8) | Data[3]);                                                        // 转子转速原始数据

    Can1_M3508_MotorStatus[ID - 0x201].ShaftPosition = Can1_M3508_MotorStatus[ID - 0x201].RotorPosition * 0.0520746310219994f; //=Can1_M3508_MotorStatus[ID-0x201].RotorPosition/M3508_ReductionRatio;//转轴角度位置
    Can1_M3508_MotorStatus[ID - 0x201].Shaft_r = (int64_t)(Can1_M3508_MotorStatus[ID - 0x201].ShaftPosition) / 360;
    if (Can1_M3508_MotorStatus[ID - 0x201].ShaftPosition < 0 && Can1_M3508_MotorStatus[ID - 0x201].ShaftPosition - 360.0f * Can1_M3508_MotorStatus[ID - 0x201].Shaft_r < 0)
        Can1_M3508_MotorStatus[ID - 0x201].Shaft_r--;                                                                                                       // 获取转轴圈数
    Can1_M3508_MotorStatus[ID - 0x201].ShaftAngle = Can1_M3508_MotorStatus[ID - 0x201].ShaftPosition - 360.0f * Can1_M3508_MotorStatus[ID - 0x201].Shaft_r; // 转轴机械角度
    Can1_M3508_MotorStatus[ID - 0x201].ShaftSpeed = Can1_M3508_MotorStatus[ID - 0x201].RotorSpeed * 0.0520746310219994f;                                    //=Can1_M3508_MotorStatus[ID-0x201].RotorSpeed/M3508_ReductionRatio;//转轴转速

    Can1_M3508_MotorStatus[ID - 0x201].RawCurrent = (int16_t)((((uint16_t)Data[4]) << 8) | Data[5]);              // 转矩电流原始数据
    Can1_M3508_MotorStatus[ID - 0x201].Current = Can1_M3508_MotorStatus[ID - 0x201].RawCurrent * 0.001220703125f; //=Can1_M3508_MotorStatus[ID-0x201].RawCurrent/16384.0f*20.0f;//转矩电流

    Can1_M3508_MotorStatus[ID - 0x201].Power = Can1_M3508_MotorStatus[ID - 0x201].ShaftSpeed * Can1_M3508_MotorStatus[ID - 0x201].Current * 0.031413612565445f; //=Can1_M3508_MotorStatus[ID-0x201].ShaftSpeed*Can1_M3508_MotorStatus[ID-0x201].Current*M3508_TorqueConstant/9.55f;//电机功率(功率P(kW)=转轴转速v(RPM)*转矩T(N·m)/9550,转矩T=转矩电流*转矩常数)
    if (Can1_M3508_MotorStatus[ID - 0x201].Power < 0)
        Can1_M3508_MotorStatus[ID - 0x201].Power *= -1;       // 功率去负数化
    Can1_M3508_MotorStatus[ID - 0x201].Temperature = Data[6]; // 电机温度
}
void CAN2_M3508_DataProcess(M3508_ID ID, uint8_t *Data)
{
    uint16_t M3508_RotorNowAngle = (uint16_t)((((uint16_t)Data[0]) << 8) | Data[1]); // 本次转子机械角度原始数据
    if (M3508_RotorNowAngle - Can2_M3508_MotorStatus[ID - 0x201].RawRotorAngle > 4000 && Can2_M3508_MotorStatus[ID - 0x201].First_Flag == 1)
        Can2_M3508_MotorStatus[ID - 0x201].Rotor_r--; // 本次转子机械角度原始数据和上次转子机械角度原始数据出现跃变
    else if (Can2_M3508_MotorStatus[ID - 0x201].RawRotorAngle - M3508_RotorNowAngle > 4000 && Can2_M3508_MotorStatus[ID - 0x201].First_Flag == 1)
        Can2_M3508_MotorStatus[ID - 0x201].Rotor_r++;
    else if (Can2_M3508_MotorStatus[ID - 0x201].First_Flag != 1)
        Can2_M3508_MotorStatus[ID - 0x201].First_Flag = 1;

    Can2_M3508_MotorStatus[ID - 0x201].RawRotorAngle = M3508_RotorNowAngle;                                                                                 // 转子机械角度原始数据
    Can2_M3508_MotorStatus[ID - 0x201].RotorAngle = M3508_RotorNowAngle * 0.0439453125f;                                                                    //=M3508_RotorNowAngle/8192.0f*360.0f;//转子机械角度
    Can2_M3508_MotorStatus[ID - 0x201].RawRotorPosition = 8192 * Can2_M3508_MotorStatus[ID - 0x201].Rotor_r + M3508_RotorNowAngle;                          // 转子角度位置原始数据
    Can2_M3508_MotorStatus[ID - 0x201].RotorPosition = 360.0f * Can2_M3508_MotorStatus[ID - 0x201].Rotor_r + Can2_M3508_MotorStatus[ID - 0x201].RotorAngle; // 转子角度位置
    Can2_M3508_MotorStatus[ID - 0x201].RotorSpeed = (int16_t)((((uint16_t)Data[2]) << 8) | Data[3]);                                                        // 转子转速原始数据

    Can2_M3508_MotorStatus[ID - 0x201].ShaftPosition = Can2_M3508_MotorStatus[ID - 0x201].RotorPosition * 0.0520746310219994f; //=Can2_M3508_MotorStatus[ID-0x201].RotorPosition/M3508_ReductionRatio;//转轴角度位置
    Can2_M3508_MotorStatus[ID - 0x201].Shaft_r = (int64_t)(Can2_M3508_MotorStatus[ID - 0x201].ShaftPosition) / 360;
    if (Can2_M3508_MotorStatus[ID - 0x201].ShaftPosition < 0 && Can2_M3508_MotorStatus[ID - 0x201].ShaftPosition - 360.0f * Can2_M3508_MotorStatus[ID - 0x201].Shaft_r < 0)
        Can2_M3508_MotorStatus[ID - 0x201].Shaft_r--;                                                                                                       // 获取转轴圈数
    Can2_M3508_MotorStatus[ID - 0x201].ShaftAngle = Can2_M3508_MotorStatus[ID - 0x201].ShaftPosition - 360.0f * Can2_M3508_MotorStatus[ID - 0x201].Shaft_r; // 转轴机械角度
    Can2_M3508_MotorStatus[ID - 0x201].ShaftSpeed = Can2_M3508_MotorStatus[ID - 0x201].RotorSpeed * 0.0520746310219994f;                                    //=Can2_M3508_MotorStatus[ID-0x201].RotorSpeed/M3508_ReductionRatio;//转轴转速

    Can2_M3508_MotorStatus[ID - 0x201].RawCurrent = (int16_t)((((uint16_t)Data[4]) << 8) | Data[5]);              // 转矩电流原始数据
    Can2_M3508_MotorStatus[ID - 0x201].Current = Can2_M3508_MotorStatus[ID - 0x201].RawCurrent * 0.001220703125f; //=Can2_M3508_MotorStatus[ID-0x201].RawCurrent/16384.0f*20.0f;//转矩电流

    Can2_M3508_MotorStatus[ID - 0x201].Power = Can2_M3508_MotorStatus[ID - 0x201].ShaftSpeed * Can2_M3508_MotorStatus[ID - 0x201].Current * 0.031413612565445f; //=Can2_M3508_MotorStatus[ID-0x201].ShaftSpeed*Can2_M3508_MotorStatus[ID-0x201].Current*M3508_TorqueConstant/9.55f;//电机功率(功率P(kW)=转轴转速v(RPM)*转矩T(N·m)/9550,转矩T=转矩电流*转矩常数)
    if (Can2_M3508_MotorStatus[ID - 0x201].Power < 0)
        Can2_M3508_MotorStatus[ID - 0x201].Power *= -1;       // 功率去负数化
    Can2_M3508_MotorStatus[ID - 0x201].Temperature = Data[6]; // 电机温度
}

void Motor_6020_Voltage1(int16_t device1, int16_t device2,
                         int16_t device3, int16_t device4,
                         CAN_HandleTypeDef *hcan) // CANID:0x1FF 电机ID:1-4 (-25000,25000)
{
    CAN_TxHeaderTypeDef Tx_Message;
    uint8_t Can_Send_Data[8];
    uint32_t send_mail_box;

    Tx_Message.StdId = 0x1FF;
    Tx_Message.IDE = CAN_ID_STD;
    Tx_Message.RTR = CAN_RTR_DATA;
    Tx_Message.DLC = 0x08;
    Can_Send_Data[0] = device1 >> 8;
    Can_Send_Data[1] = device1;
    Can_Send_Data[2] = device2 >> 8;
    Can_Send_Data[3] = device2;
    Can_Send_Data[4] = device3 >> 8;
    Can_Send_Data[5] = device3;
    Can_Send_Data[6] = device4 >> 8;
    Can_Send_Data[7] = device4;

    HAL_CAN_AddTxMessage(hcan, &Tx_Message, Can_Send_Data, &send_mail_box);
}

void Motor_6020_Voltage2(int16_t device1, int16_t device2,
                         int16_t device3, int16_t device4,
                         CAN_HandleTypeDef *hcan) // CANID:0x2FF 电机ID:5-8 (-25000,25000)
{
    CAN_TxHeaderTypeDef Tx_Message;
    uint8_t Can_Send_Data[8];
    uint32_t send_mail_box;

    Tx_Message.StdId = 0x2FF;
    Tx_Message.IDE = CAN_ID_STD;
    Tx_Message.RTR = CAN_RTR_DATA;
    Tx_Message.DLC = 0x08;
    Can_Send_Data[0] = device1 >> 8;
    Can_Send_Data[1] = device1;
    Can_Send_Data[2] = device2 >> 8;
    Can_Send_Data[3] = device2;
    Can_Send_Data[4] = device3 >> 8;
    Can_Send_Data[5] = device3;
    Can_Send_Data[6] = device4 >> 8;
    Can_Send_Data[7] = device4;

    HAL_CAN_AddTxMessage(hcan, &Tx_Message, Can_Send_Data, &send_mail_box);
}

void CAN1_M6020_DataProcess(M6020_ID ID, uint8_t *Data)
{
    uint16_t GM6020_NowAngle = (uint16_t)((((uint16_t)Data[0]) << 8) | Data[1]); // 本次机械角度原始数据

    if (GM6020_NowAngle - Can1_M6020_MotorStatus[ID - 0x205].Angle > 4000 && Can1_M6020_MotorStatus[ID - 0x205].First_Flag == 1 && Can1_M6020_MotorStatus[ID - 0x205].r > 0)
        Can1_M6020_MotorStatus[ID - 0x205].r--; // 本次机械角度原始数据和上次机械角度原始数据出现跃变
    else if (Can1_M6020_MotorStatus[ID - 0x205].Angle - GM6020_NowAngle > 4000 && Can1_M6020_MotorStatus[ID - 0x205].First_Flag == 1 && Can1_M6020_MotorStatus[ID - 0x205].r < 1)
        Can1_M6020_MotorStatus[ID - 0x205].r++;
    else if (Can1_M6020_MotorStatus[ID - 0x205].First_Flag != 1)
        Can1_M6020_MotorStatus[ID - 0x205].First_Flag++;

    Can1_M6020_MotorStatus[ID - 0x205].Angle = GM6020_NowAngle;                    // 机械角度
    Can1_M6020_MotorStatus[ID - 0x205].ANgle = GM6020_NowAngle / 8192.0f * 360.0f; // 机械角度
    if (Can1_M6020_MotorStatus[ID - 0x205].ANgle > 180)
        Can1_M6020_MotorStatus[ID - 0x205].ANgle -= 360.0f;
    else if (Can1_M6020_MotorStatus[ID - 0x205].ANgle < -180)
        Can1_M6020_MotorStatus[ID - 0x205].ANgle += 360.0f;
    Can1_M6020_MotorStatus[ID - 0x205].Position = 8192 * Can1_M6020_MotorStatus[ID - 0x205].r + GM6020_NowAngle; // 角度位置
    Can1_M6020_MotorStatus[ID - 0x205].Speed = (int16_t)((((uint16_t)Data[2]) << 8) | Data[3]);                  // 转速
    Can1_M6020_MotorStatus[ID - 0x205].Current = (int16_t)((((uint16_t)Data[4]) << 8) | Data[5]);                // 实际转矩电流
    Can1_M6020_MotorStatus[ID - 0x205].Temperature = Data[6];                                                    // 电机温度
}

void CAN2_M6020_DataProcess(M6020_ID ID, uint8_t *Data)
{
    uint16_t GM6020_NowAngle = (uint16_t)((((uint16_t)Data[0]) << 8) | Data[1]); // 本次机械角度原始数据

    if (GM6020_NowAngle - Can2_M6020_MotorStatus[ID - 0x205].Angle > 4000 && Can2_M6020_MotorStatus[ID - 0x205].First_Flag == 1 && Can2_M6020_MotorStatus[ID - 0x205].r > 0)
        Can2_M6020_MotorStatus[ID - 0x205].r--; // 本次机械角度原始数据和上次机械角度原始数据出现跃变
    else if (Can2_M6020_MotorStatus[ID - 0x205].Angle - GM6020_NowAngle > 4000 && Can2_M6020_MotorStatus[ID - 0x205].First_Flag == 1 && Can2_M6020_MotorStatus[ID - 0x205].r < 1)
        Can2_M6020_MotorStatus[ID - 0x205].r++;
    else if (Can2_M6020_MotorStatus[ID - 0x205].First_Flag != 1)
        Can2_M6020_MotorStatus[ID - 0x205].First_Flag++;

    Can2_M6020_MotorStatus[ID - 0x205].Angle = GM6020_NowAngle;                    // 机械角度
    Can2_M6020_MotorStatus[ID - 0x205].ANgle = GM6020_NowAngle / 8192.0f * 360.0f; // 机械角度
    if (Can2_M6020_MotorStatus[ID - 0x205].ANgle > 180)
        Can2_M6020_MotorStatus[ID - 0x205].ANgle -= 360.0f;
    else if (Can2_M6020_MotorStatus[ID - 0x205].ANgle < -180)
        Can2_M6020_MotorStatus[ID - 0x205].ANgle += 360.0f;
    Can2_M6020_MotorStatus[ID - 0x205].Position = 8192 * Can2_M6020_MotorStatus[ID - 0x205].r + GM6020_NowAngle; // 角度位置
    Can2_M6020_MotorStatus[ID - 0x205].Speed = (int16_t)((((uint16_t)Data[2]) << 8) | Data[3]);                  // 转速
    Can2_M6020_MotorStatus[ID - 0x205].Current = (int16_t)((((uint16_t)Data[4]) << 8) | Data[5]);                // 实际转矩电流
    Can2_M6020_MotorStatus[ID - 0x205].Temperature = Data[6];                                                    // 电机温度
}

void Motor_2006_Current1(int16_t device1, int16_t device2,
                         int16_t device3, int16_t device4,
                         CAN_HandleTypeDef *hcan) // CANID:0x200 电机ID:1-4 (-10000,10000)
{
    CAN_TxHeaderTypeDef Tx_Message;
    uint8_t Can_Send_Data[8];
    uint32_t send_mail_box;

    Tx_Message.StdId = 0x200;
    Tx_Message.IDE = CAN_ID_STD;
    Tx_Message.RTR = CAN_RTR_DATA;
    Tx_Message.DLC = 0x08;
    Can_Send_Data[0] = device1 >> 8;
    Can_Send_Data[1] = device1;
    Can_Send_Data[2] = device2 >> 8;
    Can_Send_Data[3] = device2;
    Can_Send_Data[4] = device3 >> 8;
    Can_Send_Data[5] = device3;
    Can_Send_Data[6] = device4 >> 8;
    Can_Send_Data[7] = device4;

    HAL_CAN_AddTxMessage(hcan, &Tx_Message, Can_Send_Data, &send_mail_box);
}

void Motor_2006_Current2(int16_t device1, int16_t device2,
                         int16_t device3, int16_t device4,
                         CAN_HandleTypeDef *hcan) // CANID:0x1FF 电机ID:5-8 (-10000,10000)
{
    CAN_TxHeaderTypeDef Tx_Message;
    uint8_t Can_Send_Data[8];
    uint32_t send_mail_box;

    Tx_Message.StdId = 0x1FF;
    Tx_Message.IDE = CAN_ID_STD;
    Tx_Message.RTR = CAN_RTR_DATA;
    Tx_Message.DLC = 0x08;
    Can_Send_Data[0] = device1 >> 8;
    Can_Send_Data[1] = device1;
    Can_Send_Data[2] = device2 >> 8;
    Can_Send_Data[3] = device2;
    Can_Send_Data[4] = device3 >> 8;
    Can_Send_Data[5] = device3;
    Can_Send_Data[6] = device4 >> 8;
    Can_Send_Data[7] = device4;

    HAL_CAN_AddTxMessage(hcan, &Tx_Message, Can_Send_Data, &send_mail_box);
}
void CAN1_M2006_DataProcess(M2006_ID ID, uint8_t *Data)
{
    uint16_t M2006_RotorNowAngle = (uint16_t)((((uint16_t)Data[0]) << 8) | Data[1]); // 本次转子机械角度原始数据
    if (M2006_RotorNowAngle - Can1_M2006_MotorStatus[ID - 0x201].RawRotorAngle > 4000 && Can1_M2006_MotorStatus[ID - 0x201].First_Flag == 1)
        Can1_M2006_MotorStatus[ID - 0x201].Rotor_r--; // 本次转子机械角度原始数据和上次转子机械角度原始数据出现跃变
    else if (Can1_M2006_MotorStatus[ID - 0x201].RawRotorAngle - M2006_RotorNowAngle > 4000 && Can1_M2006_MotorStatus[ID - 0x201].First_Flag == 1)
        Can1_M2006_MotorStatus[ID - 0x201].Rotor_r++;
    else if (Can1_M2006_MotorStatus[ID - 0x201].First_Flag != 1)
        Can1_M2006_MotorStatus[ID - 0x201].First_Flag = 1;

    Can1_M2006_MotorStatus[ID - 0x201].RawRotorAngle = M2006_RotorNowAngle;                                                                                 // 转子机械角度原始数据
    Can1_M2006_MotorStatus[ID - 0x201].RotorAngle = M2006_RotorNowAngle * 0.0439453125f;                                                                    //=M2006_RotorNowAngle/8192.0f*360.0f;//转子机械角度
    Can1_M2006_MotorStatus[ID - 0x201].RawRotorPosition = 8192 * Can1_M2006_MotorStatus[ID - 0x201].Rotor_r + M2006_RotorNowAngle;                          // 转子角度位置原始数据
    Can1_M2006_MotorStatus[ID - 0x201].RotorPosition = 360.0f * Can1_M2006_MotorStatus[ID - 0x201].Rotor_r + Can1_M2006_MotorStatus[ID - 0x201].RotorAngle; // 转子角度位置
    Can1_M2006_MotorStatus[ID - 0x201].RotorSpeed = (int16_t)((((uint16_t)Data[2]) << 8) | Data[3]);                                                        // 转子转速原始数据

    Can1_M2006_MotorStatus[ID - 0x201].ShaftPosition = Can1_M2006_MotorStatus[ID - 0x201].RotorPosition * 0.0277777777777778f; //=Can1_M2006_MotorStatus[ID-0x201].RotorPosition/M2006_ReductionRatio;//转轴角度位置
    Can1_M2006_MotorStatus[ID - 0x201].Shaft_r = (int64_t)(Can1_M2006_MotorStatus[ID - 0x201].ShaftPosition) / 360;
    if (Can1_M2006_MotorStatus[ID - 0x201].ShaftPosition < 0 && Can1_M2006_MotorStatus[ID - 0x201].ShaftPosition - 360.0f * Can1_M2006_MotorStatus[ID - 0x201].Shaft_r < 0)
        Can1_M2006_MotorStatus[ID - 0x201].Shaft_r--;                                                                                                       // 获取转轴圈数
    Can1_M2006_MotorStatus[ID - 0x201].ShaftAngle = Can1_M2006_MotorStatus[ID - 0x201].ShaftPosition - 360.0f * Can1_M2006_MotorStatus[ID - 0x201].Shaft_r; // 转轴机械角度
    Can1_M2006_MotorStatus[ID - 0x201].ShaftSpeed = Can1_M2006_MotorStatus[ID - 0x201].RotorSpeed * 0.0277777777777778f;                                    //=Can1_M2006_MotorStatus[ID-0x201].RotorSpeed/M2006_ReductionRatio;//转轴转速

    Can1_M2006_MotorStatus[ID - 0x201].RawCurrent = (int16_t)((((uint16_t)Data[4]) << 8) | Data[5]);     // 转矩电流原始数据
    Can1_M2006_MotorStatus[ID - 0x201].Current = Can1_M2006_MotorStatus[ID - 0x201].RawCurrent * 0.001f; //=Can1_M2006_MotorStatus[ID-0x201].RawCurrent/10000.0f*10.0f;//转矩电流

    Can1_M2006_MotorStatus[ID - 0x201].Power = Can1_M2006_MotorStatus[ID - 0x201].ShaftSpeed * Can1_M2006_MotorStatus[ID - 0x201].Current * 0.018848167539267f; //=Can1_M2006_MotorStatus[ID-0x201].ShaftSpeed*Can1_M2006_MotorStatus[ID-0x201].Current*M2006_TorqueConstant/9.55f;//电机功率(功率P(kW)=转轴转速v(RPM)*转矩T(N·m)/9550,转矩T=转矩电流*转矩常数)
    if (Can1_M2006_MotorStatus[ID - 0x201].Power < 0)
        Can1_M2006_MotorStatus[ID - 0x201].Power *= -1; // 功率去负数化
}

void CAN2_M2006_DataProcess(M2006_ID ID, uint8_t *Data)
{
    uint16_t M2006_RotorNowAngle = (uint16_t)((((uint16_t)Data[0]) << 8) | Data[1]); // 本次转子机械角度原始数据
    if (M2006_RotorNowAngle - Can2_M2006_MotorStatus[ID - 0x201].RawRotorAngle > 4000 && Can2_M2006_MotorStatus[ID - 0x201].First_Flag == 1)
        Can2_M2006_MotorStatus[ID - 0x201].Rotor_r--; // 本次转子机械角度原始数据和上次转子机械角度原始数据出现跃变
    else if (Can2_M2006_MotorStatus[ID - 0x201].RawRotorAngle - M2006_RotorNowAngle > 4000 && Can2_M2006_MotorStatus[ID - 0x201].First_Flag == 1)
        Can2_M2006_MotorStatus[ID - 0x201].Rotor_r++;
    else if (Can2_M2006_MotorStatus[ID - 0x201].First_Flag != 1)
        Can2_M2006_MotorStatus[ID - 0x201].First_Flag = 1;

    Can2_M2006_MotorStatus[ID - 0x201].RawRotorAngle = M2006_RotorNowAngle;                                                                                 // 转子机械角度原始数据
    Can2_M2006_MotorStatus[ID - 0x201].RotorAngle = M2006_RotorNowAngle * 0.0439453125f;                                                                    //=M2006_RotorNowAngle/8192.0f*360.0f;//转子机械角度
    Can2_M2006_MotorStatus[ID - 0x201].RawRotorPosition = 8192 * Can2_M2006_MotorStatus[ID - 0x201].Rotor_r + M2006_RotorNowAngle;                          // 转子角度位置原始数据
    Can2_M2006_MotorStatus[ID - 0x201].RotorPosition = 360.0f * Can2_M2006_MotorStatus[ID - 0x201].Rotor_r + Can2_M2006_MotorStatus[ID - 0x201].RotorAngle; // 转子角度位置
    Can2_M2006_MotorStatus[ID - 0x201].RotorSpeed = (int16_t)((((uint16_t)Data[2]) << 8) | Data[3]);                                                        // 转子转速原始数据

    Can2_M2006_MotorStatus[ID - 0x201].ShaftPosition = Can2_M2006_MotorStatus[ID - 0x201].RotorPosition * 0.0277777777777778f; //=Can2_M2006_MotorStatus[ID-0x201].RotorPosition/M2006_ReductionRatio;//转轴角度位置
    Can2_M2006_MotorStatus[ID - 0x201].Shaft_r = (int64_t)(Can2_M2006_MotorStatus[ID - 0x201].ShaftPosition) / 360;
    if (Can2_M2006_MotorStatus[ID - 0x201].ShaftPosition < 0 && Can2_M2006_MotorStatus[ID - 0x201].ShaftPosition - 360.0f * Can2_M2006_MotorStatus[ID - 0x201].Shaft_r < 0)
        Can2_M2006_MotorStatus[ID - 0x201].Shaft_r--;                                                                                                       // 获取转轴圈数
    Can2_M2006_MotorStatus[ID - 0x201].ShaftAngle = Can2_M2006_MotorStatus[ID - 0x201].ShaftPosition - 360.0f * Can2_M2006_MotorStatus[ID - 0x201].Shaft_r; // 转轴机械角度
    Can2_M2006_MotorStatus[ID - 0x201].ShaftSpeed = Can2_M2006_MotorStatus[ID - 0x201].RotorSpeed * 0.0277777777777778f;                                    //=Can2_M2006_MotorStatus[ID-0x201].RotorSpeed/M2006_ReductionRatio;//转轴转速

    Can2_M2006_MotorStatus[ID - 0x201].RawCurrent = (int16_t)((((uint16_t)Data[4]) << 8) | Data[5]);     // 转矩电流原始数据
    Can2_M2006_MotorStatus[ID - 0x201].Current = Can2_M2006_MotorStatus[ID - 0x201].RawCurrent * 0.001f; //=Can2_M2006_MotorStatus[ID-0x201].RawCurrent/10000.0f*10.0f;//转矩电流

    Can2_M2006_MotorStatus[ID - 0x201].Power = Can2_M2006_MotorStatus[ID - 0x201].ShaftSpeed * Can2_M2006_MotorStatus[ID - 0x201].Current * 0.018848167539267f; //=Can2_M2006_MotorStatus[ID-0x201].ShaftSpeed*Can2_M2006_MotorStatus[ID-0x201].Current*M2006_TorqueConstant/9.55f;//电机功率(功率P(kW)=转轴转速v(RPM)*转矩T(N·m)/9550,转矩T=转矩电流*转矩常数)
    if (Can2_M2006_MotorStatus[ID - 0x201].Power < 0)
        Can2_M2006_MotorStatus[ID - 0x201].Power *= -1; // 功率去负数化
}

float Motor_Encoder_Circle(float target_angle, float current_encoder_angle)
{
    float err = target_angle - current_encoder_angle;

    if (err > MOTOR_ENCODER_HALF_ECD_MAX)
    {
        // 当前值落后一圈或多圈，加一个最大值使其追上
        current_encoder_angle += MOTOR_ENCODER_ECD_MAX;
    }
    else if (err < -MOTOR_ENCODER_HALF_ECD_MAX)
    {
        // 当前值超前一圈或多圈，减一个最大值使其退回
        current_encoder_angle -= MOTOR_ENCODER_ECD_MAX;
    }
    // 如果误差在范围内，则 current_encoder_angle 不变
    return current_encoder_angle;
}
