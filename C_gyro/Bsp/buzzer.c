#include "buzzer.h"

extern TIM_HandleTypeDef htim4;

void BuzzerInit(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

void BuzzerOn(void)
{
    static int16_t temp = 4000 ;
    if(temp < 1000)
    {    
        BuzzerOff();
        return;
    }
    __HAL_TIM_PRESCALER(&htim4,(int)(temp/1000));
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 10000);
    temp -= 5 ;
    
}

void BuzzerOff(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}


void Start_Buzzer_remind(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 900);
	HAL_Delay(250);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 700);
	HAL_Delay(250);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 600);
	HAL_Delay(250);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 500);
	HAL_Delay(250);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 1500);

	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}

void Start_Buzzer_remind2(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 500);
	HAL_Delay(100);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 300);
	HAL_Delay(100);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 200);
	HAL_Delay(100);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 100);
	HAL_Delay(100);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 1500);

	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}

