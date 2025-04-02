#ifndef __LK_MOTOR_H
#define __LK_MOTOR_H

#include "main.h"

#define I_MIN -2000
#define I_MAX 2000
#define CURRENT_SMOOTH_COEF 0.9f
#define SPEED_SMOOTH_COEF 0.85f
#define REDUCTION_RATIO_DRIVEN 1
#define ECD_ANGLE_COEF_LK (360.0f / 65536.0f)
#define CURRENT_TORQUE_COEF_LK 0.003645f // 电流设定值转换成扭矩的系数,算出来的设定值除以这个系数就是扭矩值

/* 注意:协议兼容LK9025电机多电机控制模式，而非单电机控制模式*/

typedef struct //接收报文结构体
{
	//LK9025反馈报文原始数据
	uint8_t Cmd_Byte;				//命令字节，不用管
	int8_t 	Temperature;			// 温度,C°
	int16_t Real_Current;			// 实际电流
	int16_t Speed_dps;				// 电机速度,dps(每秒钟度)
	uint16_t ecd;             		// 当前编码器值
	
	//利用原始数据计算一些我们需要的数据
	uint16_t last_ecd;        // 上一次读取的编码器值
    float angle_single_round; // 单圈角度
    float speed_rads;         // speed rad/s

    float total_angle;   // 总角度
    int32_t total_round; // 总圈数

    float feed_dt;
    uint32_t feed_dwt_cnt;	
	
} LKMotor_Receive_t;

typedef struct // LK9025
{
	
	uint8_t ID;
	uint8_t LKData[8];
	
} LKMotor_Send_t;



typedef struct
{

    LKMotor_Receive_t LKMotor1_Receive; //接收报文的原始数据
	LKMotor_Receive_t LKMotor2_Receive;
	
	LKMotor_Send_t 		Send;			//发送数据结构体

	int16_t Motor_Out[2];

	Motor_Working_Type_e stop_flag; // 启停标志

} LKMotorInstance;

extern LKMotorInstance LKMotor;

uint8_t LK_Motor_Send(int16_t LKMotor1, int16_t LKMotor2);
uint8_t LK_Motor_PID_Init(void);
void LKMotorStop(LKMotorInstance motor);
void LKMotorControl(void);
void LKMotor_MeasureData_Dispose(FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *buf);

#endif
