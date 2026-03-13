#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>
#include "can.h"
typedef enum // 3508
{
    M3508_1 = 0x201, // ID1
    M3508_2 = 0x202, // ID2
    M3508_3 = 0x203, // ID3
    M3508_4 = 0x204, // ID4
    M3508_5 = 0x205, // ID5
    M3508_6 = 0x206, // ID6
    M3508_7 = 0x207, // ID7
    M3508_8 = 0x208, // ID8
} M3508_ID;          // M3508电机ID号枚举
typedef enum         // 6020
{
    GM6020_1 = 0x205, // ID1
    GM6020_2 = 0x206, // ID2
    GM6020_3 = 0x207, // ID3
    GM6020_4 = 0x208, // ID4
    GM6020_5 = 0x209, // ID5
    GM6020_6 = 0x20A, // ID6
    GM6020_7 = 0x20B, // ID7
} M6020_ID;           // GM6020电机ID号枚举
typedef enum          // 2006
{
    M2006_1 = 0x201, // ID1
    M2006_2 = 0x202, // ID2
    M2006_3 = 0x203, // ID3
    M2006_4 = 0x204, // ID4
    M2006_5 = 0x205, // ID5
    M2006_6 = 0x206, // ID6
    M2006_7 = 0x207, // ID7
    M2006_8 = 0x208, // ID8
} M2006_ID;          // M2006电机ID号枚举
typedef struct       // 3508
{
    uint8_t First_Flag;       // M3508电机首次接收标志位
    int64_t Rotor_r;          // M3508电机转子转过圈数
    uint16_t RawRotorAngle;   // 编码器原始数据(范围0~8191,对应0~360°,注意:8192对应360°)
    float RotorAngle;         // 编码器所映射的角度(0~360°)
    int64_t RawRotorPosition; // M3508电机转子角度位置原始数据
    float RotorPosition;      // M3508电机转子角度位置(单位°)
    int16_t RotorSpeed;       // M3508电机转子转速(单位RPM)

    int64_t Shaft_r;     // M3508电机转轴转过圈数
    float ShaftAngle;    // M3508电机转轴机械角度(单位°)
    float ShaftPosition; // M3508电机转轴角度位置(单位°)
    float ShaftSpeed;    // M3508电机转轴转速(单位RPM)

    int16_t RawCurrent; // M3508电机转矩电流原始数据(范围-16384~16384,对应-20A~20A,注意:16384对应20A)
    float Current;      // M3508电机转矩电流(单位A)

    float Power;         // M3508电机功率(单位W)
    uint8_t Temperature; // M3508电机电机温度(单位℃)
} M3508_Motor;           // M3508电机状态结构体(减速比3591:187(≈19:1),转矩系数0.3N·m/A)
typedef struct           // 6020
{
    int16_t Current;     // GM6020电机实际转矩电流
    uint8_t Temperature; // GM6020电机电机温度
    uint16_t Angle;      // GM6020电机机械角度
    int16_t Speed;       // GM6020电机转速

    float ANgle;        // 处理过的圆周角度
    uint8_t First_Flag; // GM6020电机首次接收标志位
    int64_t r;          // GM6020电机转过圈数(默认圈数只会出现0,1)
    int64_t Position;   // GM6020电机角度位置原始数据
} M6020_Motor;          // GM6020电机状态结构体

typedef struct // 2006
{
    uint8_t First_Flag;       // M2006电机首次接收标志位
    int64_t Rotor_r;          // M2006电机转子转过圈数
    uint16_t RawRotorAngle;   // M2006电机转子机械角度原始数据(范围0~8191,对应0~360°,注意:8192对应360°)
    float RotorAngle;         // M2006电机转子机械角度(单位°)
    int64_t RawRotorPosition; // M2006电机转子角度位置原始数据
    float RotorPosition;      // M2006电机转子角度位置(单位°)
    int16_t RotorSpeed;       // M2006电机转子转速(单位RPM)

    int64_t Shaft_r;     // M2006电机转轴转过圈数
    float ShaftAngle;    // M2006电机转轴机械角度(单位°)
    float ShaftPosition; // M2006电机转轴角度位置(单位°)
    float ShaftSpeed;    // M2006电机转轴转速(单位RPM)

    int16_t RawCurrent; // M2006电机转矩电流原始数据(范围-10000~10000,对应-10A~10A,注意:10000对应10A)
    float Current;      // M2006电机转矩电流(单位A)

    float Power; // M2006电机功率(单位W)
} M2006_Motor;   // M2006电机状态结构体(减速比36:1,转矩系数0.18N·m/A)

void Motor_3508_Current1(int16_t device1, int16_t device2,
                         int16_t device3, int16_t device4,
                         CAN_HandleTypeDef *hcan); // CANID:0x200 电机ID:1-4 (-16384,16384)
void Motor_3508_Current2(int16_t device1, int16_t device2,
                         int16_t device3, int16_t device4,
                         CAN_HandleTypeDef *hcan); // CANID:0x1FF 电机ID:5-8 (-16384,16384)
void CAN1_M3508_DataProcess(M3508_ID ID, uint8_t *Data);
void CAN2_M3508_DataProcess(M3508_ID ID, uint8_t *Data);

void Motor_6020_Voltage1(int16_t device1, int16_t device2,
                         int16_t device3, int16_t device4,
                         CAN_HandleTypeDef *hcan); // CANID:0x1FF 电机ID:1-4 (-25000,25000)
void Motor_6020_Voltage2(int16_t device1, int16_t device2,
                         int16_t device3, int16_t device4,
                         CAN_HandleTypeDef *hcan); // CANID:0x2FF 电机ID:5-8 (-25000,25000)
void CAN1_M6020_DataProcess(M6020_ID ID, uint8_t *Data);
void CAN2_M6020_DataProcess(M6020_ID ID, uint8_t *Data);

void Motor_2006_Current1(int16_t device1, int16_t device2,
                         int16_t device3, int16_t device4,
                         CAN_HandleTypeDef *hcan); // CANID:0x200 电机ID:1-4 (-10000,10000)
void Motor_2006_Current2(int16_t device1, int16_t device2,
                         int16_t device3, int16_t device4,
                         CAN_HandleTypeDef *hcan); // CANID:0x1FF 电机ID:5-8 (-10000,10000)
void CAN1_M2006_DataProcess(M2006_ID ID, uint8_t *Data);
void CAN2_M2006_DataProcess(M2006_ID ID, uint8_t *Data);

float Motor_Encoder_Circle(float target_angle, float current_encoder_angle);

#endif // MOTOR_H
