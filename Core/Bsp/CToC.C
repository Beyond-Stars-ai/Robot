#include "stm32f4xx_hal.h"
#include "string.h"

#include "CToC.h"
#include "can.h" // 提供 hcan2

/*
 *函数简介:板间通讯主机发送遥控器摇杆数据
 *参数说明:无
 *返回类型:1-发送成功,0-发送失败
 *备注:默认标准格式数据帧,8字节数据段
 *备注:使用从机ID1
 */
uint8_t CToC_MasterSendData(void)
{
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8] = {0};
    uint32_t txMailbox;

    txHeader.StdId              = CToC_SlaveID1;
    txHeader.ExtId              = 0;
    txHeader.IDE                = CAN_ID_STD;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    // txData[0] = (uint8_t)((uint16_t)Remote_RxData.Remote_R_RL >> 8);
    // txData[1] = (uint8_t)((uint16_t)Remote_RxData.Remote_R_RL & 0x00FF);
    // txData[2] = (uint8_t)((uint16_t)Remote_RxData.Remote_R_UD >> 8);
    // txData[3] = (uint8_t)((uint16_t)Remote_RxData.Remote_R_UD & 0x00FF);
    // txData[4] = (uint8_t)(Remote_RxData.Remote_L_RL >> 8);
    // txData[5] = (uint8_t)(Remote_RxData.Remote_L_RL & 0x00FF);
    // txData[6] = (uint8_t)(Remote_RxData.Remote_L_UD >> 8);
    // txData[7] = (uint8_t)(Remote_RxData.Remote_L_UD & 0x00FF);

    if (HAL_CAN_AddTxMessage(&hcan2, &txHeader, txData, &txMailbox) != HAL_OK) {
        return 0;
    }

    // 等待发送完成（可选）：简单轮询超时
    uint32_t timeout = 0;
    while (HAL_CAN_IsTxMessagePending(&hcan2, txMailbox) && timeout++ < 0xFFFF) {}
    if (timeout >= 0xFFFF) return 0;
    return 1;
}

/*
 *函数简介:板间通讯主机发送遥控器控制数据
 *参数说明:无
 *返回类型:1-发送成功,0-发送失败
 *备注:默认标准格式数据帧,8字节数据段
 *备注:使用从机ID2
 */
// uint8_t CToC_MasterSendControl(void)
// {
//     CAN_TxHeaderTypeDef txHeader;
//     uint8_t txData[8] = {0};
//     uint32_t txMailbox;

//     txHeader.StdId = CToC_SlaveID2;
//     txHeader.ExtId = 0;
//     txHeader.IDE = CAN_ID_STD;
//     txHeader.RTR = CAN_RTR_DATA;
//     txHeader.DLC = 8;
//     txHeader.TransmitGlobalTime = DISABLE;

//     // 以下对未定义符号已注释。若这些变量在项目其他处定义，可取消注释并使用。
//     /*
//     txData[0] = Remote_Status;
//     txData[1] = Remote_RxData.Remote_RS;
//     txData[2] = Remote_RxData.Remote_KeyPush_Ctrl;
//     txData[3] = Remote_RxData.Remote_KeyPush_Shift;
//     txData[4] = Remote_StartFlag;
//     txData[5] = Remote_RxData.Remote_LS;
//     txData[6] = Remote_RxData.Remote_Key_Alpha1;
//     txData[7] = Remote_RxData.Remote_Key_Alpha2;
//     */

//     if (HAL_CAN_AddTxMessage(&hcan2, &txHeader, txData, &txMailbox) != HAL_OK)
//     {
//         return 0;
//     }

//     uint32_t timeout = 0;
//     while (HAL_CAN_IsTxMessagePending(&hcan2, txMailbox) && timeout++ < 0xFFFF) { }
//     if (timeout >= 0xFFFF) return 0;
//     return 1;
// }

/*
 *函数简介:板间通讯数据处理
 *参数说明:CAN数据帧ID号,详情见CToC.h的宏定义
 *参数说明:反馈数据(8字节)
 *返回类型:无
 *接收数据:
 *	Data[0]-发射机构状态
 */
void CToC_CANDataProcess(uint32_t ID, uint8_t *Data)
{
    // 保留原逻辑，但注释掉对未定义符号的影响部分
    static uint8_t Last_ShooterStatus = 0;
    if (ID == CToC_MasterID1) {
        // 如果 RefereeSystem_ShooterStatus 在工程中存在，可恢复下面三行
        // Last_ShooterStatus = RefereeSystem_ShooterStatus;
        // RefereeSystem_ShooterStatus = Data[0];
        // if (Last_ShooterStatus == 0 && RefereeSystem_ShooterStatus == 1) RefereeSystem_ShooterOpenFlag = 1;

        // 当前仅保存/使用 Data[0] 的值到本地静态变量（示例）
        Last_ShooterStatus = Data[0];
        (void)Last_ShooterStatus; // 若需要后续使用请替换为实际逻辑
    }
}
