#ifndef __CTOC_H
#define __CTOC_H

#define CToC_MasterID1	0x019//主机ID1

#define CToC_SlaveID1	0x149//从机ID1
#define CToC_SlaveID2	0x189//从机ID2

uint8_t CToC_MasterSendData(uint16_t Remote_R_RL, uint16_t Remote_R_UD, uint16_t Remote_L_RL, uint16_t Remote_L_UD);//板间通讯主机发送遥控器摇杆数据
// uint8_t CToC_MasterSendControl(void);//板间通讯主机发送遥控器控制数据
void CToC_CANDataProcess(uint32_t ID,uint8_t *Data);//板间通讯数据处理

#endif
