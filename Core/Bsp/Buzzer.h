#ifndef LED_H
#define LED_H
#include "main.h"


typedef enum
{
	S=0,//空拍
	L1,L1_,L2,L2_,L3,L4,L4_,L5,L5_,L6,L6_,L7,//低音
	M1,M1_,M2,M2_,M3,M4,M4_,M5,M5_,M6,M6_,M7,//中音
	H1,H1_,H2,H2_,H3,H4,H4_,H5,H5_,H6,H6_,H7,//高音
}Buzzer_Tone;//蜂鸣器音调枚举

void Buzzer_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_SetFreq(uint8_t note);

#endif // LED_H

