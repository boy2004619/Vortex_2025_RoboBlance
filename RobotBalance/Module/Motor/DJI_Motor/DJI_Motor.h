#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H

#include "main.h"

typedef struct
{
    uint8_t  ID;
	// uint8_t  TxData[8];	    //数据发送存储
	int16_t Motor_Out;

}DJI_Motor_Send_t;

typedef struct
{
	uint8_t   RxData[8]; //原始数据接收存储
    uint8_t   ID;        //电机ID
	int16_t angle;       //电机机械角度（0~1891）
	int16_t rpm;         //转速
	int16_t i;           //电流
	uint8_t temp;        //温度

}DJI_Motor_Receive_t;

typedef struct
{
	DJI_Motor_Send_t DJI_M2006_Send[1];
	DJI_Motor_Receive_t DJI_M2006_Receive[1];

	DJI_Motor_Send_t DJI_M3508_Send[2];
	DJI_Motor_Receive_t DJI_M3508_Receive[2];
	
    DJI_Motor_Send_t DJI_G6020_Send;
    DJI_Motor_Receive_t DJI_G6020_Receive[1];

	uint8_t  TxData[8];	    //数据发送存储
	int16_t Motor_Out[1];
}DJI_MotorInstance;

void Yaw_Motor_Control (void);
uint8_t DJI_Motor_Send(FDCAN_HandleTypeDef* hcan ,uint32_t IDEN,int16_t Motor1,int16_t Motor2,int16_t Motor3,int16_t Motor4);
void DJIMotor_MeasureData_Dispose(FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *buf);

extern DJI_MotorInstance DJI_Motor;
#endif

