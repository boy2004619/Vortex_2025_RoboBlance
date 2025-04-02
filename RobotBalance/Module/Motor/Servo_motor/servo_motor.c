#include "servo_motor.h"

void Servo_Init(void)
{
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

//舵机设置角度
void Servo_SetAngle(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, float Angle)
{
	TIM_Set_PWM(tim_pwmHandle, Channel, Angle / 180 * 2000 + 500);	//设置占空比
}

void Open_Hatch (void)
{
    
    Servo_SetAngle(&htim1,TIM_CHANNEL_1,35);
    Servo_SetAngle(&htim1,TIM_CHANNEL_3,55);

}

void Close_Hatch (void)
{
    Servo_SetAngle(&htim1,TIM_CHANNEL_1,75);
    Servo_SetAngle(&htim1,TIM_CHANNEL_3,15);

}

