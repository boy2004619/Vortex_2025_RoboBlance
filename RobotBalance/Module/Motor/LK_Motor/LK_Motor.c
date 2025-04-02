#include "LK_Motor.h"

LKMotorInstance LKMotor;

uint8_t LK_Motor_PID_Init(void)
{
	LKMotor1_ctrl.Kp=0.06f;
	LKMotor1_ctrl.Ki=0.4f;
	LKMotor1_ctrl.Kd=0.0f;
	
	LKMotor1_ctrl.MaxOut=2000;
	LKMotor1_ctrl.IntegralLimit=2000;
	LKMotor1_ctrl.DeadBand=0;
	LKMotor1_ctrl.Improve = (PID_Improvement_e)(PID_IMPROVE_NONE | PID_Integral_Limit);
	
	return 0;

}

uint8_t LK_Motor_Send(int16_t LKMotor1, int16_t LKMotor2)
{
	
   FDCAN_TxHeaderTypeDef TxHeader;
	
  TxHeader.Identifier = 0X280;													// 报文ID
  TxHeader.IdType = FDCAN_STANDARD_ID;											// 标准ID 
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;									// 数据帧 
  TxHeader.DataLength = 8 << 16;													// 发送数据固定长度为8 
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;							// 设置错误状态指示 								
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;										// 不开启可变波特率 
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;										// 普通CAN格式 
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;						// 用于发送事件FIFO控制, 不存储 
  TxHeader.MessageMarker = 0x00; 			// 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF                
   
	// 将 int16_t 变量的值按字节复制到 uint8_t 数组中
   // 由于 int16_t 是 2 字节，这里我们只复制这 2 字节
    LKMotor.Send.LKData[0] = (uint8_t)(LKMotor1 & 0x00FF); // 低字节
    LKMotor.Send.LKData[1] = (uint8_t)(LKMotor1 >> 8);     // 高字节
    
    LKMotor.Send.LKData[2] = (uint8_t)(LKMotor2 & 0x00FF); // 第二个变量的低字节
    LKMotor.Send.LKData[3] = (uint8_t)(LKMotor2 >> 8);     // 第二个变量的高字节
    
    // 确保只修改了前4个字节，因为 DataLength 设置为 8，意味着只有8个字节用于数据
    // 接下来的4个字节应该保持原有值或明确初始化为特定值
		LKMotor.Send.LKData[4] = 0;
		LKMotor.Send.LKData[5] = 0;
		LKMotor.Send.LKData[6] = 0;
		LKMotor.Send.LKData[7] = 0;
		
  if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, LKMotor.Send.LKData)!=HAL_OK) 
		return 1;//发送
	return 0;	
}

 //好像重复实现了没用上？ 应该是忘删了，可以删掉
// uint8_t MeasureData_Dispose(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
// {
	
// 	FDCAN_RxHeaderTypeDef pRxHeader;
// 	if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &pRxHeader, buf) == HAL_OK){
// 	switch(pRxHeader.Identifier)
// 	{
//    case 0x141:
//    // 处理 ID 为 0x141 的消息
// 	// 根据数据格式解析消息(原始数据提取)        
//    LKMotor.LKMotor1_Receive.Cmd_Byte = (int8_t)buf[0];
//    LKMotor.LKMotor1_Receive.Temperature = (int8_t)buf[1];
//    LKMotor.LKMotor1_Receive.Real_Current = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
//    LKMotor.LKMotor1_Receive.Speed_dps = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
//    LKMotor.LKMotor1_Receive.ecd = (uint16_t)(((uint16_t)buf[7] << 8) | buf[6]);
	 
// 	 //根据需求处理数据
// 	//  LKMotor.LKMotor1_Receive.speed_rads= LKMotor.LKMotor1_Receive.Speed_dps/57.3f;
//    LKMotor.LKMotor1_Receive.speed_rads= (1 - SPEED_SMOOTH_COEF) * LKMotor.LKMotor1_Receive.speed_rads + \
//                           DEGREE_2_RAD * SPEED_SMOOTH_COEF * (float)((int16_t)(buf[5] << 8 | buf[4]));
//    break;
	 
//    case 0x142:
//    // 处理 ID 为 0x142 的消息
// 	 LKMotor.LKMotor2_Receive.Cmd_Byte = (int8_t)buf[0];
// 	 LKMotor.LKMotor2_Receive.Temperature = (int8_t)buf[1];
//    LKMotor.LKMotor2_Receive.Real_Current = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
//    LKMotor.LKMotor2_Receive.Speed_dps = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
//    LKMotor.LKMotor2_Receive.ecd = (uint16_t)(((uint16_t)buf[7] << 8) | buf[6]);
	 
// 	 //根据需求处理数据
// 	//  LKMotor.LKMotor2_Receive.speed_rads= LKMotor.LKMotor2_Receive.Speed_dps/57.3f;
//   LKMotor.LKMotor2_Receive.speed_rads= (1 - SPEED_SMOOTH_COEF) * LKMotor.LKMotor2_Receive.speed_rads + \
//                           DEGREE_2_RAD * SPEED_SMOOTH_COEF * (float)((int16_t)(buf[5] << 8 | buf[4]));
//    break;
//    default:
//    // 忽略或处理其他 ID 的消息
//    break;
			
// 	}
// 	return 1;
// 	}
// 	else
// 	return 0;
// }


void LKMotor_MeasureData_Dispose(FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *buf)
{
	switch(pRxHeader->Identifier)
	{
   case 0x141:
   // 处理 ID 为 0x141 的消息
	// 根据数据格式解析消息(原始数据提取)        
   LKMotor.LKMotor1_Receive.Cmd_Byte = (int8_t)buf[0];
   LKMotor.LKMotor1_Receive.Temperature = (int8_t)buf[1];
   LKMotor.LKMotor1_Receive.Real_Current = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
   LKMotor.LKMotor1_Receive.Speed_dps = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
   LKMotor.LKMotor1_Receive.ecd = (uint16_t)(((uint16_t)buf[7] << 8) | buf[6]);
	 
	 //根据需求处理数据
	//  LKMotor.LKMotor1_Receive.speed_rads= LKMotor.LKMotor1_Receive.Speed_dps/57.3f;
   LKMotor.LKMotor1_Receive.speed_rads= (1 - SPEED_SMOOTH_COEF) * LKMotor.LKMotor1_Receive.speed_rads + \
                          DEGREE_2_RAD * SPEED_SMOOTH_COEF * (float)((int16_t)(buf[5] << 8 | buf[4]));
   break;
	 
   case 0x142:
   // 处理 ID 为 0x142 的消息
	 LKMotor.LKMotor2_Receive.Cmd_Byte = (int8_t)buf[0];
	 LKMotor.LKMotor2_Receive.Temperature = (int8_t)buf[1];
   LKMotor.LKMotor2_Receive.Real_Current = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
   LKMotor.LKMotor2_Receive.Speed_dps = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
   LKMotor.LKMotor2_Receive.ecd = (uint16_t)(((uint16_t)buf[7] << 8) | buf[6]);
	 
	 //根据需求处理数据
	//  LKMotor.LKMotor2_Receive.speed_rads= LKMotor.LKMotor2_Receive.Speed_dps/57.3f;
  LKMotor.LKMotor2_Receive.speed_rads= (1 - SPEED_SMOOTH_COEF) * LKMotor.LKMotor2_Receive.speed_rads + \
                          DEGREE_2_RAD * SPEED_SMOOTH_COEF * (float)((int16_t)(buf[5] << 8 | buf[4]));
   break;

   default:
   // 忽略或处理其他 ID 的消息
   break;	
	}
}

//重写FDCAN接收处理函数
void fdcan1_rx_callback(void)
{
  FDCAN_RxHeaderTypeDef pRxHeader;
	if(HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0, &pRxHeader, rx_data1) == HAL_OK){
   DJIMotor_MeasureData_Dispose(&pRxHeader, rx_data1);
   LKMotor_MeasureData_Dispose(&pRxHeader, rx_data1);
   }

}

void LKMotorControl(void)
{
    if (LKMotor.stop_flag == MOTOR_STOP)
    { // 若该电机处于停止状态,直接将发送置零
    LKMotor.Motor_Out[LD -1] = LKMotor.Motor_Out[RD -1] = 0;
    }
   LK_Motor_Send(LKMotor.Motor_Out[LD -1] , LKMotor.Motor_Out[RD -1]);
}

