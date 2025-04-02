#ifndef __IMU_CH040_H
#define __IMU_CH040_H

#include "main.h"

//陀螺仪原始数据包结构体 //HI91型
typedef __packed struct hi91_t
{
    uint8_t         header1;        //帧头1
    uint8_t         header2;        //帧头2
    uint16_t        data_len;       //数据长度  =76
    uint16_t        crc;      
    uint8_t         tag;            /* Data packet tag, if tag = 0x00, means that this packet is null */
    uint16_t        pps_sync_stamp; /* PPS synchronization time in milliseconds */
    int8_t          temp;           /* Temperature */
    float           air_pressure;   /* Pressure */
    uint32_t        system_time;    /* Timestamp */
    float           acc[3];         /* Accelerometer data (x, y, z) */
    float           gyr[3];         /* Gyroscope data (x, y, z) */
    float           mag[3];         /* Magnetometer data (x, y, z) */
    float           roll;           /* Roll angle */
    float           pitch;          /* Pitch angle */
    float           yaw;            /* Yaw angle */
    float           quat[4];        /* Quaternion (w, x, y, z) */
} hi91_t;


//陀螺仪数据结构体
typedef struct CH040_Data_Packet_t
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

}CH040_Data_Packet_t;


typedef __packed struct IMUC_Data_Packet_t
{
	uint8_t         header1;        //帧头1  a5
  uint8_t         header2;        //帧头2  5a
	float q[4]; // 四元数估计值

  float Gyro[3];  // 角速度
  float Accel[3]; // 加速度
  float MotionAccel_b[3]; // 机体坐标加速度
  float MotionAccel_n[3]; // 绝对系加速度

    // 位姿
  float Roll;
  float Pitch;
  float Yaw;
  float YawTotalAngle;
}IMUC_Data_Packet_t;

extern IMUC_Data_Packet_t IMUC_Data;
extern bool IMUC_ready;

bool IMU_CH040_Dispose(uint8_t *pData);
bool IMUC_Dispose(uint8_t *pData);

#endif
