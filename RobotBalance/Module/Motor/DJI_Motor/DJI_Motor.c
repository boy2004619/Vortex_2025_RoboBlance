#include "DJI_Motor.h"

DJI_MotorInstance DJI_Motor;

/*
IDEN是DJI电机的报文的标识符
GM6020 : 当电机ID:1~4时IDEN = 0X1FF,ID:5~7时IDEN = 0X2FF(电压控制)
         当电机ID:1~4时IDEN = 0X1FE,ID:5~7时IDEN = 0X2FE(电流控制)
Motor1~4的范围 ：-16384~0~16384
*/
uint8_t DJI_Motor_Send(FDCAN_HandleTypeDef* hcan ,uint32_t IDEN,int16_t Motor1,int16_t Motor2,int16_t Motor3,int16_t Motor4)
{
	
  FDCAN_TxHeaderTypeDef TxHeader;
	
  TxHeader.Identifier = IDEN;													// 报文ID
  TxHeader.IdType = FDCAN_STANDARD_ID;											// 标准ID 
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;									// 数据帧 
  TxHeader.DataLength = 8 << 16;													// 发送数据固定长度为8 
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;					// 设置错误状态指示 								
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;										// 不开启可变波特率 
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;										// 普通CAN格式 
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;					// 用于发送事件FIFO控制, 不存储 
  TxHeader.MessageMarker = 0x00; 			// 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF                
   
    // 将 int16_t 变量的值按字节复制到 uint8_t 数组中
    // 由于 int16_t 是 2 字节，这里我们只复制这 2 字节
  DJI_Motor.TxData[0] = (uint8_t)(Motor1 >> 8); // 高字节
  DJI_Motor.TxData[1] = (uint8_t)(Motor1 & 0x00FF);// 低字节  
    
  DJI_Motor.TxData[2] = (uint8_t)(Motor2 >> 8); // 高字节
  DJI_Motor.TxData[3] = (uint8_t)(Motor2 & 0x00FF);// 低字节 
    
	DJI_Motor.TxData[4] = (uint8_t)(Motor3 >> 8); // 高字节;
	DJI_Motor.TxData[5] = (uint8_t)(Motor3 & 0x00FF);// 低字节;

	DJI_Motor.TxData[6] = (uint8_t)(Motor4 >> 8); // 高字节;
	DJI_Motor.TxData[7] = (uint8_t)(Motor4 & 0x00FF);// 低字节;
		
  if(HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, DJI_Motor.TxData)!=HAL_OK) 
		return 1;//发送
	return 0;	
}


void Yaw_Motor_Control (void)
{
  if(Robot_RunState.Robot_stop )
  DJI_Motor.Motor_Out[0] = 0;
  
  DJI_Motor_Send(&hfdcan1 ,0X1FF ,DJI_Motor.Motor_Out[0] ,0,0,0);

}


void DJIMotor_MeasureData_Dispose(FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *buf)
{
	switch(pRxHeader->Identifier)
	{
  case 0x204 + 1:
  // 处理 G6020 ID 为 1 的消息
  // 根据数据格式解析消息(原始数据提取)
  DJI_Motor.DJI_G6020_Receive[0].angle = (uint16_t)buf[0]<<8|buf[1];//;buf[0]*256+buf[1];
  DJI_Motor.DJI_G6020_Receive[0].rpm   = (uint16_t)buf[2]<<8|buf[3];//buf[2]*256+buf[3];
  // DJI_Motor.DJI_G6020_Receive[0].rpm   = (1-0.85)*DJI_Motor.DJI_G6020_Receive[0].rpm + 0.85*((uint16_t)buf[2]<<8|buf[3]);  //
  DJI_Motor.DJI_G6020_Receive[0].i     = buf[4]*256+buf[5];
  DJI_Motor.DJI_G6020_Receive[0].temp  = buf[6];


  break;

  case 0x200 + 1:
  // 处理 M3508 ID 为 1 的消息
  // 根据数据格式解析消息(原始数据提取)
  DJI_Motor.DJI_M3508_Receive[0].angle = buf[0]*256+buf[1];
  DJI_Motor.DJI_M3508_Receive[0].rpm   = buf[2]*256+buf[3];
  DJI_Motor.DJI_M3508_Receive[0].i     = buf[4]*256+buf[5];
  DJI_Motor.DJI_M3508_Receive[0].temp  = buf[6];
  break;

  case 0x200 + 2:
  // 处理 M3508 ID 为 2 的消息
  // 根据数据格式解析消息(原始数据提取)
  DJI_Motor.DJI_M3508_Receive[1].angle = buf[0]*256+buf[1];
  DJI_Motor.DJI_M3508_Receive[1].rpm   = buf[2]*256+buf[3];
  DJI_Motor.DJI_M3508_Receive[1].i     = buf[4]*256+buf[5];
  DJI_Motor.DJI_M3508_Receive[1].temp  = buf[6];
  break;

  case 0x200 + 3:
  // 处理 M2006 ID 为 3 的消息
  // 根据数据格式解析消息(原始数据提取)
  DJI_Motor.DJI_M2006_Receive[0].angle = buf[0]*256+buf[1];
  DJI_Motor.DJI_M2006_Receive[0].rpm   = buf[2]*256+buf[3];
  DJI_Motor.DJI_M2006_Receive[0].i     = buf[4]*256+buf[5];
  DJI_Motor.DJI_M2006_Receive[0].temp  = buf[6];
  break;
	 
  default:
  // 忽略或处理其他 ID 的消息
  break;	
	}
}



