#include "Buzzer.h"
#include "main.h"
#include "tim.h"

//定时器时钟频率为84MHz
int16_t Buzzer_ToneFreq[37]=
{
	0,//不发音
	32107,30305,28604,26999,25483,24053,22703,21429,20226,19091,18019,17008,//低音
	16053,15152,14302,13499,12742,12026,11352,10714,10113,9545 ,9010 ,8504 ,//中音
	8027 ,7576 ,7151 ,6750 ,6371 ,6013 ,5676 ,5357 ,5056 ,4773 ,4505 ,4252 ,//高音
};//蜂鸣器音调频率表

void Buzzer_Init(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

}
void Buzzer_On(void)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 5); //设置占空比，调整音量
}

void Buzzer_Off(void)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); //占空比为0，关闭蜂鸣器
}

void Buzzer_SetFreq(uint8_t note)//0-36
{
	__HAL_TIM_SET_PRESCALER(&htim4, Buzzer_ToneFreq[note]);
}



