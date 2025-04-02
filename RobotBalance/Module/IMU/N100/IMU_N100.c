#include "IMU_N100.h"

SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
uint8_t AHRSData_buffer[AHRS_RS];
uint8_t AHRSAccData_buffer[AHRSAcc_RS];
uint8_t BODYVELData_buffer[BODYVEL_RS];
uint8_t IMUData_buffer[IMU_RS];

AHRSData_Packet_t CHASSIS_AHRSData_Packet;   //底盘的AHRS结构体(弧度制)
IMUData_Packet_t  CHASSIS_IMUData_Packet;	 //底盘的IMU结构体(弧度制)
AHRSACC_Packet_t  CHASSIS_AHRSACCData_Packet;
BODYVEL_Packet_t  CHASSIS_BODYVELData_Packet;


void IMU_N100_Dispose(uint8_t *pData)
{
	if(pData[1]==TYPE_AHRS&&pData[2]==AHRS_LEN)
	{	
	memcpy(AHRSData_buffer,pData,AHRS_RS);
	}
	
	else if(pData[1]==TYPE_IMU&&pData[2]==IMU_LEN)
	{
	memcpy(IMUData_buffer,pData,IMU_RS);
	}

    else if(pData[1]==TYPE_AHRSAcc&&pData[2]==AHRSAcc_LEN)
	{
	memcpy(AHRSAccData_buffer,pData,AHRSAcc_RS);
	}

	else if(pData[1]==TYPE_BODYVEL&&pData[2]==BODYVEL_LEN)
	{
	memcpy(BODYVELData_buffer,pData,BODYVEL_RS);
	}
}

/*******************************
数据组装
*******************************/
void AHRS_TTL_Hex2Dec(AHRSData_Packet_t *Packet ,uint8_t *pData)
{
	Packet->RollSpeed=DATA_Trans(pData[7],pData[8],pData[9],pData[10]);       //横滚角速度
	Packet->PitchSpeed=DATA_Trans(pData[11],pData[12],pData[13],pData[14]);   //俯仰角速度
	Packet->HeadingSpeed=DATA_Trans(pData[15],pData[16],pData[17],pData[18]); //偏航角速度
			
	Packet->Roll=DATA_Trans(pData[19],pData[20],pData[21],pData[22]);      //横滚角
	Packet->Pitch=DATA_Trans(pData[23],pData[24],pData[25],pData[26]);     //俯仰角
	Packet->Heading=DATA_Trans(pData[27],pData[28],pData[29],pData[30]);	 //偏航角
			
	Packet->Qw=DATA_Trans(pData[31],pData[32],pData[33],pData[34]);  //四元数
	Packet->Qx=DATA_Trans(pData[35],pData[36],pData[37],pData[38]);
	Packet->Qy=DATA_Trans(pData[39],pData[40],pData[41],pData[42]);
	Packet->Qz=DATA_Trans(pData[43],pData[44],pData[45],pData[46]);
	Packet->Timestamp=timestamp(pData[47],pData[48],pData[49],pData[50]);   //时间戳
	
}

void IMU_TTL_Hex2Dec(IMUData_Packet_t *Packet ,uint8_t *pData)
{

	Packet->gyroscope_x=DATA_Trans(pData[7],pData[8],pData[9],pData[10]);  //角速度
	Packet->gyroscope_y=DATA_Trans(pData[11],pData[12],pData[13],pData[14]);
	Packet->gyroscope_z=DATA_Trans(pData[15],pData[16],pData[17],pData[18]);
		
	Packet->accelerometer_x=DATA_Trans(pData[19],pData[20],pData[21],pData[22]);  //线加速度
	Packet->accelerometer_y=DATA_Trans(pData[23],pData[24],pData[25],pData[26]);
	Packet->accelerometer_z=DATA_Trans(pData[27],pData[28],pData[29],pData[30]);

	Packet->magnetometer_x=DATA_Trans(pData[31],pData[32],pData[33],pData[34]);  //磁力计数据
	Packet->magnetometer_y=DATA_Trans(pData[35],pData[36],pData[37],pData[38]);
	Packet->magnetometer_z=DATA_Trans(pData[39],pData[40],pData[41],pData[42]);
			
	Packet->Timestamp=timestamp(pData[55],pData[56],pData[57],pData[58]);   //时间戳
}

void AHRSACC_TTL_Hex2Dec(AHRSACC_Packet_t *Packet ,uint8_t *pData)
{

	Packet->RollAcc=DATA_Trans(pData[7],pData[8],pData[9],pData[10]);  //角加速度
	Packet->PitchAcc=DATA_Trans(pData[11],pData[12],pData[13],pData[14]);
	Packet->HeadingAcc=DATA_Trans(pData[15],pData[16],pData[17],pData[18]);

}


void BODYVEL_TTL_Hex2Dec(BODYVEL_Packet_t *Packet ,uint8_t *pData)
{

	Packet->VEL_X=DATA_Trans(pData[7],pData[8],pData[9],pData[10]);  //角加速度
	Packet->VEL_Y=DATA_Trans(pData[11],pData[12],pData[13],pData[14]);
	Packet->VEL_Z=DATA_Trans(pData[15],pData[16],pData[17],pData[18]);

}

/*************
实现16进制的can数据转换成浮点型数据
****************/
float DATA_Trans(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4)
{
    long long transition_32;
	float tmp=0;
	int sign=0;
	int exponent=0;
	float mantissa=0;
  	transition_32 = 0;
  	transition_32 |=  Data_4<<24;   
  	transition_32 |=  Data_3<<16; 
	transition_32 |=  Data_2<<8;
	transition_32 |=  Data_1;
  	sign = (transition_32 & 0x80000000) ? -1 : 1;//符号位
	//先右移操作，再按位与计算，出来结果是30到23位对应的e
	exponent = ((transition_32 >> 23) & 0xff) - 127;
	//将22~0转化为10进制，得到对应的x系数 
	mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
	tmp=sign * mantissa * pow(2, exponent);
	return tmp;
}

long long timestamp(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4)
{
  uint32_t transition_32;
  transition_32 = 0;
  transition_32 |=  Data_4<<24;   
  transition_32 |=  Data_3<<16; 
  transition_32 |=  Data_2<<8;
	transition_32 |=  Data_1;
	return transition_32;
}


/**************************************************************************
函数功能：将上位机发过来的高8位和低8位数据整合成一个short型数据后，再做单位还原换算
入口参数：高8位，低8位
返回  值：机器人X/Y/Z轴的目标速度
**************************************************************************/
float XYZ_Target_Speed_transition(uint8_t High,uint8_t Low)
{
	//Data conversion intermediate variable
	//数据转换的中间变量
	short transition; 
	
	//将高8位和低8位整合成一个16位的short型数据
	//The high 8 and low 8 bits are integrated into a 16-bit short data
	transition=((High<<8)+Low); 
	return 
		transition/1000+(transition%1000)*0.001; //Unit conversion, mm/s->m/s //单位转换, mm/s->m/s						
}

/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
函数功能：计算要发送/接收的数据校验结果
入口参数：Count_Number：校验的前几位数；Mode：0-对接收数据进行校验，1-对发送数据进行校验
返回  值：校验结果
**************************************************************************/
uint8_t Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//对要发送的数据进行校验
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Send_Data.buffer[k];
	}
	
	//Verify the data received
	//对接收到的数据进行校验
	if(Mode==0)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Receive_Data.buffer[k];
	}
	return check_sum;
}


