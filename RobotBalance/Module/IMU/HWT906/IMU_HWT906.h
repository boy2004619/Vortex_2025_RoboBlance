#ifndef __IMU_HWT906_H
#define __IMU_HWT906_H

#include "main.h"

//维特陀螺仪原始数据包结构体
typedef __packed struct WIT_IMU_ORIData_Packet
{
	  uint8_t  header;
    uint8_t  data_id;
    int16_t  Data[4];
	  uint8_t  crcsum; 
	
		uint8_t  header2;
    uint8_t  data_id2;
    int16_t  Data2[4];
	  uint8_t  crcsum2;
	
		uint8_t  header3;
    uint8_t  data_id3;
    int16_t  Data3[4];
	  uint8_t  crcsum3;
    
    uint8_t  header4;
    uint8_t  data_id4;
    int16_t  Data4[4];
	  uint8_t  crcsum4;

}WIT_IMU_ORIData_Packet;

//陀螺仪数据结构体
typedef struct HWT906_Data_Packet_t
{

	  float  Roll_Angle;
    float  Pitch_Angle;
    float  Yaw_Angle;
    float  Roll_Speed;
    float  Pitch_Speed;
    float  Yaw_Speed;
	  float  Roll_Acc;
    float  Pitch_Acc;
    float  Yaw_Acc;
    float  Q[4];

}HWT906_Data_Packet_t;


bool IMU_HWT906_Dispose(uint8_t *pData);

extern HWT906_Data_Packet_t   HWT906_Data;

#endif
