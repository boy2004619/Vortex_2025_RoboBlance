#ifndef __DM_MOTOR_H
#define __DM_MOTOR_H

#include "main.h"
#define Motar_mode 0	//设置模式为何种模式，为0为IMT模式，为1为位置速度模式，为2为速度模式

#define P_MIN -12.5		//位置最小值
#define P_MAX 12.5		//位置最大值
#define V_MIN -45		//速度最小值
#define V_MAX 45		//速度最大值
#define KP_MIN 0.0		//Kp最小值
#define KP_MAX 500.0	//Kp最大值
#define KD_MIN 0.0		//Kd最小值
#define KD_MAX 5.0		//Kd最大值
#define T_MIN -18		//转矩最大值
#define T_MAX 18		//转矩最小值

#define DM_Motor_Disable 0x00    
#define DM_Motor_Ensable 0x01

#define DM_Motor_NUM 1 		//DM电机数目 CANx_t 结构体用
#define Pitch_DM6220 0 

typedef struct
{
	int p_int[DM_Motor_NUM],v_int[DM_Motor_NUM],t_int[DM_Motor_NUM];				//这里可根据电机数目自行修改，读取 DM_Motor_NUM 个电机的位置、速度、转矩
	float position[DM_Motor_NUM],velocity[DM_Motor_NUM],torque[DM_Motor_NUM];	// DM_Motor_NUM 个电机的位置、速度、转矩解析存储
	uint8_t  Tx_Data[8];						//数据发送存储
	uint8_t  RxData[8];							//数据接收存储
	FDCAN_RxHeaderTypeDef Rx_pHeader;
}CANx_t;

typedef struct
{
    uint8_t  ID;
	uint8_t  TxData[8];	    //数据发送存储

}DM_Motor_Send_t;

typedef struct
{
	uint8_t   RxData[8]; //原始数据接收存储
    uint8_t   ID;        //电机ID
    uint8_t   ERR_State; //电机状态及错误标志
    float  Pos;        //反馈位置
    float  Vel;        //反馈速度
    float   T;         //电机扭矩
    float   T_MOS;     //表示驱动上MOS的平均温度，单位℃
    float   T_Rotor;   //表示电机内部线圈的平均温度，单位℃
}DM_Motor_Receive_t;

typedef struct
{
    DM_Motor_Send_t DM_G6220_Send[1];
    DM_Motor_Receive_t DM_G6220_Receive[1];

    float Motor_Out[1];
}DM_MotorInstance;

extern DM_MotorInstance DM_Motor;

uint8_t MIT_CtrlMotor(FDCAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,
float _KP, float _KD, float _torq);
uint8_t PosSpeed_CtrlMotor(FDCAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel);
uint8_t Speed_CtrlMotor(FDCAN_HandleTypeDef* hcan, uint16_t ID, float _vel);
void DMMotor_MeasureData_Dispose(FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *buf);
void Motor_enable(void);
void Motor_control(void);

#endif
