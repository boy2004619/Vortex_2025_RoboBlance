#include "remind.h"

void Start_Buzzer_remind(void)
{
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	
	TIM12->CCR2 = 900;
	HAL_Delay(250);
	TIM12->CCR2 = 700;
	HAL_Delay(250);
	TIM12->CCR2 = 600;
	HAL_Delay(250);
	TIM12->CCR2 = 500;
	HAL_Delay(250);
	TIM12->CCR2 = 1500;

	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
	TIM12->CCR2 = 1500;
}

void Start_LED_remind(void)
{
	
}

