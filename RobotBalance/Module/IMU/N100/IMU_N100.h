#ifndef __IMU_N100_H
#define __IMU_N100_H

#include "main.h"

extern uint8_t ttl_receive;

#define FRAME_HEADER      0X7B //Frame_header //??
#define FRAME_TAIL        0X7D //Frame_tail   //?β
#define SEND_DATA_SIZE    24
#define RECEIVE_DATA_SIZE 11

#define IMU_RS 64
#define AHRS_RS 56
#define AHRSAcc_RS 20
#define BODYVEL_RS 20
#define INSGPS_RS 80

//FDlink candata
#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GROUND 0xf0
#define TYPE_AHRSAcc 0x67
#define TYPE_BODYVEL 0x60
#define IMU_LEN  0x38   //56+8  8组数据
#define AHRS_LEN 0x30   //48+8  7组数据
#define AHRSAcc_LEN 0x0C
#define BODYVEL_LEN 0x0C
#define INSGPS_LEN 0x42    //72+8  10组数据
#define IMU_CAN 9
#define AHRS_CAN 8
#define INSGPS_CAN 11
#define F2T(X) ((unsigned int)((configTICK_RATE_HZ/(X))))
	
#define RADIAN_TO_ANGLE   57.29578f  //弧度转角度的计算系数

/*****A structure for storing triaxial data of a gyroscope accelerometer*****/
/*****用于存放陀螺仪加速度计三轴数据的结构体*********************************/
typedef struct __Mpu6050_Data_ 
{
	short X_data; //2 bytes //2个字节
	short Y_data; //2 bytes //2个字节
	short Z_data; //2 bytes //2个字节
}Mpu6050_Data;

/*******The structure of the serial port sending data************/
/*******串口发送数据的结构体*************************************/
typedef struct _SEND_DATA_  
{
	unsigned char buffer[SEND_DATA_SIZE];
	struct _Sensor_Str_
	{
		unsigned char Frame_Header; //1个字节
		short X_speed;	            //2 bytes //2个字节
		short Y_speed;              //2 bytes //2个字节
		short Z_speed;              //2 bytes //2个字节
		short Power_Voltage;        //2 bytes //2个字节
		Mpu6050_Data Accelerometer; //6 bytes //6个字节
		Mpu6050_Data Gyroscope;     //6 bytes //6个字节	
		unsigned char Frame_Tail;   //1 bytes //1个字节
	}Sensor_Str;
}SEND_DATA;

typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char Frame_Header; //1 bytes //1个字节
		float X_speed;	            //4 bytes //4个字节
		float Y_speed;              //4 bytes //4个字节
		float Z_speed;              //4 bytes //4个字节
		unsigned char Frame_Tail;   //1 bytes //1个字节
	}Control_Str;
}RECEIVE_DATA;

typedef struct IMUData_Packet_t{
		float gyroscope_x;          //unit: rad/s
		float gyroscope_y;          //unit: rad/s
		float gyroscope_z;          //unit: rad/s
		float accelerometer_x;      //m/s^2
		float accelerometer_y;      //m/s^2
		float accelerometer_z;      //m/s^2
		float magnetometer_x;       //mG
		float magnetometer_y;       //mG
		float magnetometer_z;       //mG
		float imu_temperature;      //C
		float Pressure;             //Pa
		float pressure_temperature; //C
		uint32_t Timestamp;          //us
} IMUData_Packet_t;

typedef struct AHRSData_Packet_t
{
	float RollSpeed;   //unit: rad/s
	float PitchSpeed;  //unit: rad/s
	float HeadingSpeed;//unit: rad/s
	float Roll;        //unit: rad
	float Pitch;       //unit: rad
	float Heading;     //unit: rad
	float Qw;//w          //Quaternion
	float Qx;//x
	float Qy;//y
	float Qz;//z
	uint32_t Timestamp; //unit: us
}AHRSData_Packet_t;

typedef struct  AHRSACC_Packet_t
{
	float RollAcc;   //unit: rad/s^2
	float PitchAcc;  //unit: rad/s^2
	float HeadingAcc;//unit: rad/s^2

} AHRSACC_Packet_t;


typedef struct BODYVEL_Packet_t
{
	float VEL_X;  //unit: m/s
	float VEL_Y;  //unit: m/s
	float VEL_Z;  //unit: m/s

} BODYVEL_Packet_t;

///////////////////////////////////////////////////////////

typedef struct 
{
	
	float Roll;        //unit: 角度
	float Pitch;       //unit: 角度
	float Heading;     //unit: 角度
	
}N100_IMU_Packet_t;


extern AHRSData_Packet_t CHASSIS_AHRSData_Packet;//底盘的陀螺仪结构体//弧度制
extern IMUData_Packet_t CHASSIS_IMUData_Packet;	//底盘的IMU结构体(弧度制)
extern AHRSACC_Packet_t  CHASSIS_AHRSACCData_Packet;
extern BODYVEL_Packet_t  CHASSIS_BODYVELData_Packet;
extern uint8_t AHRSAccData_buffer[AHRSAcc_RS];
extern uint8_t BODYVELData_buffer[BODYVEL_RS];
extern uint8_t AHRSData_buffer[AHRS_RS];
extern uint8_t IMUData_buffer[IMU_RS];


void IMU_N100_Dispose(uint8_t *pData);
void AHRS_TTL_Hex2Dec(AHRSData_Packet_t *Packet ,uint8_t *pData);
void IMU_TTL_Hex2Dec(IMUData_Packet_t *Packet ,uint8_t *pData);
void AHRSACC_TTL_Hex2Dec(AHRSACC_Packet_t *Packet ,uint8_t *pData);
void BODYVEL_TTL_Hex2Dec(BODYVEL_Packet_t *Packet ,uint8_t *pData);
float DATA_Trans(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4);
long long timestamp(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4);
#endif
