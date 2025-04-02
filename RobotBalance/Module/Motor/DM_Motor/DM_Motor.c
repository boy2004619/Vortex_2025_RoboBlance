#include "DM_Motor.h"

DM_MotorInstance DM_Motor;
char Selection=0;
float kd = 0.1;

uint8_t Data_Enable[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};		//电机使能命令
uint8_t Data_Failure[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};		//电机失能命令
uint8_t Data_Save_zero[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};	    //电机保存零点命令


/**
 * @brief  这里对ID为0x02、0x03、0x04的3个电机进行依次使能，在对电机进行控制前进行使能
 * @param  void     	    	
 */
void Motor_enable(void)
{
	#if Motar_mode==0
	fdcanx_send_data(&hfdcan2,0x01,Data_Enable,8);	
	HAL_Delay(10);
	#elif Motar_mode==1
	fdcanx_send_data(&hfdcan2,0x101,Data_Enable,8);	
	HAL_Delay(10);
	#elif Motar_mode==2
	fdcanx_send_data(&hfdcan2,0x201,Data_Enable,8);	
	HAL_Delay(10);
	#endif
}


void Motor_disenable(void)
{
	#if Motar_mode==0
	fdcanx_send_data(&hfdcan2,0x01,Data_Failure,8);	
	HAL_Delay(10);
	#elif Motar_mode==1
	fdcanx_send_data(&hfdcan2,0x101,Data_Failure,8);	
	HAL_Delay(10);
	#elif Motar_mode==2
	fdcanx_send_data(&hfdcan2,0x201,Data_Failure,8);	
	HAL_Delay(10);
	#endif
}

/**
* @brief  这里对ID为0x02、0x03、0x04的3个电机进行依次控制，在freertos.c中CAN_Send_Task任务中1ms执行一次，
这里使用了3个电机也就是每个电机3ms发送一次,注意多个电机不能同时一起发，发送频率过快，则会使有些ID没发送到    	
 */
void Motor_control(void)
{
	#if Motar_mode==0
		switch(Selection)
		{
			case 0:
				if(Robot_RunState.Robot_stop )
				MIT_CtrlMotor(&hfdcan2,0X01, 0, 0,0, 0, 0);
				else
				MIT_CtrlMotor(&hfdcan2,0X01, 0, 0,0, 0, DM_Motor.Motor_Out[0]);	  //IMT控制模式对ID为0X02电机进行速度控制发送
				Selection=0;
				break;
		}
	#elif Motar_mode==1
		switch(Selection)
		{
			case 0:
				PosSpeed_CtrlMotor(&hfdcan2,0X101, 10, 5);	  //位置速度控制模式对ID为0X02电机进行速度控制发送
				Selection=0;
				break;
		}
	#elif Motar_mode==2
		switch(Selection)
		{
			case 0:
				Speed_CtrlMotor(&hfdcan2,0X201, 5);	  //速度控制模式对ID为0X01电机进行速度控制发送
				Selection=0;
				break;
		}
	#endif
}

/**
 * @brief  采用浮点数据等比例转换成整数
 * @param  x_int     	要转换的无符号整数
 * @param  x_min      目标浮点数的最小值
 * @param  x_max    	目标浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits){
/// converts unsigned int to float, given range and number of bits ///
 float span = x_max - x_min;
 float offset = x_min;
 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief  将浮点数转换为无符号整数
 * @param  x     			要转换的浮点数
 * @param  x_min      浮点数的最小值
 * @param  x_max    	浮点数的最大值
 * @param  bits      	无符号整数的位数
 */

int float_to_uint(float x, float x_min, float x_max, int bits){
 /// Converts a float to an unsigned int, given range and number of bits///
 float span = x_max - x_min;
 float offset = x_min;
 return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @brief  MIT模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 * @param  _KP    位置比例系数
 * @param  _KD    位置微分系数
 * @param  _torq  转矩给定值
 */
uint8_t MIT_CtrlMotor(FDCAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,
float _KP, float _KD, float _torq)
 {
    static FDCAN_TxHeaderTypeDef  Tx_Header;
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    Tx_Header.Identifier = id;													// 报文ID
    Tx_Header.IdType = FDCAN_STANDARD_ID;										// 标准ID 
    Tx_Header.TxFrameType = FDCAN_DATA_FRAME;									// 数据帧 
    Tx_Header.DataLength = 8 << 16;												// 发送数据固定长度为8 
    Tx_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;							// 设置错误状态指示 								
    Tx_Header.BitRateSwitch = FDCAN_BRS_OFF;									// 不开启可变波特率 
    Tx_Header.FDFormat = FDCAN_CLASSIC_CAN;										// 普通CAN格式 
    Tx_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;						    // 用于发送事件FIFO控制, 不存储 
    Tx_Header.MessageMarker = 0x00; 			// 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF 
	
	DM_Motor.DM_G6220_Send[0].TxData[0] = (pos_tmp >> 8);
	DM_Motor.DM_G6220_Send[0].TxData[1] = pos_tmp;
	DM_Motor.DM_G6220_Send[0].TxData[2] = (vel_tmp >> 4);
	DM_Motor.DM_G6220_Send[0].TxData[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	DM_Motor.DM_G6220_Send[0].TxData[4] = kp_tmp;
	DM_Motor.DM_G6220_Send[0].TxData[5] = (kd_tmp >> 4);
	DM_Motor.DM_G6220_Send[0].TxData[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	DM_Motor.DM_G6220_Send[0].TxData[7] = tor_tmp;
	 
    if(HAL_FDCAN_AddMessageToTxFifoQ(hcan, &Tx_Header, DM_Motor.DM_G6220_Send[0].TxData)!=HAL_OK) 
		return 1;//发送
	return 0; 
 }

/**
 * @brief  位置速度模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 */
uint8_t PosSpeed_CtrlMotor(FDCAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel)
{
    static FDCAN_TxHeaderTypeDef Tx_Header;
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&_pos;
    vbuf=(uint8_t*)&_vel;

    DM_Motor.DM_G6220_Send[0].TxData[0] = *pbuf;;
    DM_Motor.DM_G6220_Send[0].TxData[1] = *(pbuf+1);
    DM_Motor.DM_G6220_Send[0].TxData[2] = *(pbuf+2);
    DM_Motor.DM_G6220_Send[0].TxData[3] = *(pbuf+3);
    DM_Motor.DM_G6220_Send[0].TxData[4] = *vbuf;
    DM_Motor.DM_G6220_Send[0].TxData[5] = *(vbuf+1);
    DM_Motor.DM_G6220_Send[0].TxData[6] = *(vbuf+2);
    DM_Motor.DM_G6220_Send[0].TxData[7] = *(vbuf+3);

    Tx_Header.Identifier = id;													// 报文ID
    Tx_Header.IdType = FDCAN_STANDARD_ID;										// 标准ID 
    Tx_Header.TxFrameType = FDCAN_DATA_FRAME;									// 数据帧 
    Tx_Header.DataLength = 8 << 16;												// 发送数据固定长度为8 
    Tx_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;							// 设置错误状态指示 								
    Tx_Header.BitRateSwitch = FDCAN_BRS_OFF;									// 不开启可变波特率 
    Tx_Header.FDFormat = FDCAN_CLASSIC_CAN;										// 普通CAN格式 
    Tx_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;						    // 用于发送事件FIFO控制, 不存储 
    Tx_Header.MessageMarker = 0x00; 			// 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF 

    if(HAL_FDCAN_AddMessageToTxFifoQ(hcan, &Tx_Header, DM_Motor.DM_G6220_Send[0].TxData)!=HAL_OK) 
		return 1;//发送
	return 0; 
}
 
/**
 * @brief  速度模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _vel   速度给定
 */
uint8_t Speed_CtrlMotor(FDCAN_HandleTypeDef* hcan, uint16_t ID, float _vel)
{
	static FDCAN_TxHeaderTypeDef  Tx_Header;
    uint8_t *vbuf;
    vbuf=(uint8_t*)&_vel;

    DM_Motor.DM_G6220_Send[0].TxData[0] = *vbuf;
    DM_Motor.DM_G6220_Send[0].TxData[1] = *(vbuf+1);
    DM_Motor.DM_G6220_Send[0].TxData[2] = *(vbuf+2);
    DM_Motor.DM_G6220_Send[0].TxData[3] = *(vbuf+3);

    Tx_Header.Identifier = ID;													// 报文ID
    Tx_Header.IdType = FDCAN_STANDARD_ID;										// 标准ID 
    Tx_Header.TxFrameType = FDCAN_DATA_FRAME;									// 数据帧 
    Tx_Header.DataLength = 8 << 16;												// 发送数据固定长度为8 
    Tx_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;							// 设置错误状态指示 								
    Tx_Header.BitRateSwitch = FDCAN_BRS_OFF;									// 不开启可变波特率 
    Tx_Header.FDFormat = FDCAN_CLASSIC_CAN;										// 普通CAN格式 
    Tx_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;						    // 用于发送事件FIFO控制, 不存储 
    Tx_Header.MessageMarker = 0x00; 			// 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF 

    if(HAL_FDCAN_AddMessageToTxFifoQ(hcan, &Tx_Header, DM_Motor.DM_G6220_Send[0].TxData)!=HAL_OK) 
		return 1;//发送
	return 0; 
}
 
void DMMotor_MeasureData_Dispose(FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *buf)
{
    
	switch(pRxHeader->Identifier)
	{
   case 0x01:
    // 处理 ID 为 0x01 的消息
    // 根据数据格式解析消息(原始数据提取)
	DM_Motor.DM_G6220_Receive[0].ID = buf[0]&0x0F;
    DM_Motor.DM_G6220_Receive[0].ERR_State = buf[0]>>4;      
	DM_Motor.DM_G6220_Receive[0].Pos = uint_to_float((buf[1]<<8)|buf[2] , P_MIN, P_MAX, 16); // (-12.5,12.5)
	DM_Motor.DM_G6220_Receive[0].Vel = uint_to_float((buf[3]<<4)|(buf[4]>>4),V_MIN, V_MAX, 12); // (-45.0,45.0)
	DM_Motor.DM_G6220_Receive[0].T   = uint_to_float(((buf[4]&0xF)<<8)|buf[5],T_MIN, T_MAX, 12); // (-18.0,18.0)
    DM_Motor.DM_G6220_Receive[0].T_MOS = buf[6];
    DM_Motor.DM_G6220_Receive[0].T_Rotor = buf[7];
   break;
	 
   default:
   // 忽略或处理其他 ID 的消息
   break;	
	}
}


//这个可以放到 can_bsp去
void fdcan2_rx_callback(void)
{
  FDCAN_RxHeaderTypeDef pRxHeader;
	if(HAL_FDCAN_GetRxMessage(&hfdcan2,FDCAN_RX_FIFO0, &pRxHeader, rx_data2) == HAL_OK){
   DJIMotor_MeasureData_Dispose(&pRxHeader, rx_data2);
   DMMotor_MeasureData_Dispose(&pRxHeader, rx_data2);
   }

}

