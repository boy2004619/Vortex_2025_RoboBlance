#include "YS_Motor.h"

/**
 * @file YS_Motor.c (Unitree_Motor.c)  //一开始名字没起好，可以改成 Unitree_Motor
 * @author Vortex.D
 * 
 * @What? :       宇树关节电机GO-M8010-6电机通讯库。
 * @How to use? : 在串口中断回调函数中使用 YS_MOTORData_Dispose 函数解析电机数据。在控制任务中调用YS_Motor_Control完成控制
 * @todo:   暂无
 * 
 */
/********** @Include User Function **********/

      int modify_data(MOTOR_send *motor_s);      /* 发送数据处理函数 */
      int extract_data(MOTOR_recv *motor_r);     /* 接收数据处理函数 */
      HAL_StatusTypeDef SERVO_Send_recv(MOTOR_send *pData, MOTOR_recv *rData);
      bool YS_MOTORData_Dispose(uint8_t *pData , YSMotor_Control *rData); 
      void YS_Motor_Reset(MOTOR_send *crtl , MOTOR_recv *recv, uint8_t id ,int8_t Dir);
      bool joint_Motor_Reset(void);
      void YS_Motor_Control(void);

/********** @Include User Function **********/

MOTOR_send cmd;   //以全局变量声明电机控制结构体和电机数据结构体，方便在故障时通过debug查看变量值
MOTOR_recv data;
YSMotor_Control Joint_Motor; //关节电机的总结构体

// uint8_t YS_RS485_RTX_FLAG=0 ,YS_RS485_RTX_Time;
uint8_t senddata[20]={0};
uint8_t Joint_Motor_ID[4] = { LF ,LB ,RB ,RF };
int8_t Joint_Motor_Dir[4] = { LF_Dir ,LB_Dir ,RB_Dir ,RF_Dir };

float text_del_t;
uint32_t YS_dwt_cnt;


#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 } 

//按照协议处理发送的数据
int modify_data(MOTOR_send *motor_s)
{
    motor_s->hex_len = 17;
    motor_s->motor_send_data.head[0] = 0xFE;
    motor_s->motor_send_data.head[1] = 0xEE;
	
    motor_s->Pos =motor_s->Pos_output*YSMotor_RR; //输出端编码器角度

		SATURATE(motor_s->K_P,  0.0f,   25.599f);
		SATURATE(motor_s->K_W,  0.0f,   25.599f);
		SATURATE(motor_s->T,   -127.99f,  127.99f);
		SATURATE(motor_s->W,   -804.00f,  804.00f);
		SATURATE(motor_s->Pos, -411774.0f,  411774.0f);

    motor_s->motor_send_data.mode.id   = motor_s->id;
    motor_s->motor_send_data.mode.status  = motor_s->mode;
    motor_s->motor_send_data.comd.k_pos  = motor_s->K_P/25.6f*32768;
    motor_s->motor_send_data.comd.k_spd  = motor_s->K_W/25.6f*32768;
    motor_s->motor_send_data.comd.pos_des  = motor_s->Pos/6.2832f*32768;
    motor_s->motor_send_data.comd.spd_des  = motor_s->W/6.2832f*256;
    motor_s->motor_send_data.comd.tor_des  = motor_s->T*256;
		motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);
    return 0;
}

//按照协议处理接收的数据
int extract_data(MOTOR_recv *motor_r)
{
    if(motor_r->motor_recv_data.CRC16 !=
        crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, 14)){
        motor_r->correct = 0;
        return motor_r->correct;
    }
    else
		{
        motor_r->motor_id = motor_r->motor_recv_data.mode.id;
        motor_r->mode = motor_r->motor_recv_data.mode.status;
        motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
        motor_r->MError = motor_r->motor_recv_data.fbk.MError;
        motor_r->W = ((float)motor_r->motor_recv_data.fbk.speed/256)*6.2832f ;
        motor_r->T = ((float)motor_r->motor_recv_data.fbk.torque) / 256;
        motor_r->Pos = 6.2832f*((float)motor_r->motor_recv_data.fbk.pos) / 32768;
			
        motor_r->Pos_output= motor_r->Pos/YSMotor_RR;    //输出端编码器位置
        motor_r->W_output  = motor_r->W  /YSMotor_RR;    //输出端编码器速度
        motor_r->Amend_Pos = motor_r->Pos_output - motor_r->Reset_Pos;
			
				motor_r->footForce = motor_r->motor_recv_data.fbk.force;
				motor_r->correct = 1;
        return motor_r->correct;
    }
}

//发送数据包函数
HAL_StatusTypeDef SERVO_Send_recv(MOTOR_send *pData, MOTOR_recv *rData)  //实际上没有处理返回数据rData ，处理返回数据在串口中断中完成
{
  modify_data(pData);
  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&pData->motor_send_data, sizeof(pData->motor_send_data));
  DWT_Delay(0.0002f); //这个必须，不能搞太快了，因为电机是一问一答的，要留点时间让电机回传数据
  return HAL_ERROR;
}

//电机数据处理函数
bool YS_MOTORData_Dispose(uint8_t *pData , YSMotor_Control *rData)
{
  int rx_len = sizeof(MotorData_t);
  bool res = false;

  if(pData[0] == 0xFD && pData[1] == 0xEE){
  memcpy(&data, pData, rx_len);  }
  switch (data.motor_recv_data.mode.id)
  {
  case 1:
		memcpy(&rData->YSMotor_Receive[0],&data ,rx_len);
    rData->YSMotor_Receive[0].correct = 1;
    extract_data(&rData->YSMotor_Receive[0]);
    break;
	
	case 2:
		memcpy(&rData->YSMotor_Receive[1],&data ,rx_len);
    rData->YSMotor_Receive[1].correct = 1;
    extract_data(&rData->YSMotor_Receive[1]);
    break;
		
	case 3:
		memcpy(&rData->YSMotor_Receive[2],&data ,rx_len);
    rData->YSMotor_Receive[2].correct = 1;
    extract_data(&rData->YSMotor_Receive[2]);
    break;
			
	case 4:
		memcpy(&rData->YSMotor_Receive[3],&data ,rx_len);
    rData->YSMotor_Receive[3].correct = 1;
    extract_data(&rData->YSMotor_Receive[3]);
    break;
  
  default:
    break;
  }

  //text_del_t  = DWT_GetDeltaT(&YS_dwt_cnt);
  // YS_RS485_RTX_FLAG = 0;
  return res;
}


//电机复位指令。让关节电机抵住下限位，完成角度校准
void YS_Motor_Reset(MOTOR_send *crtl , MOTOR_recv *recv, uint8_t id ,int8_t Dir)
{
crtl->id = id;
crtl->mode=1;
crtl->T=Dir * YS_Reset_T;
crtl->W=0;
crtl->Pos=0;
crtl->K_P=0;
crtl->K_W=0.05;
SERVO_Send_recv(crtl, recv);
}

//完成四个关节电机的复位操作
bool joint_Motor_Reset(void)
{ 
 static uint32_t flag =0;
  //复位关节电机
  for (uint8_t i = 0; i < JOINT_CNT ; i++){
  YS_Motor_Reset(&Joint_Motor.YSMotor_Send[i], &Joint_Motor.YSMotor_Receive[i], Joint_Motor_ID[i], Joint_Motor_Dir[i]);
  }
  for (uint8_t i = 0; i < JOINT_CNT; i++) //判断是否到达
  {if(
  abs(Joint_Motor.YSMotor_Receive[i].T)  < YS_Reset_T - YS_Allow_Error_T){
  flag = 0;
  return false;}
  }
 flag++ ;
 if(flag >1000) return true;
  else return false;
}

/**
 * 关节电机主控制函数 ，在控制任务中调用对关节电机进行控制
 */
void YS_Motor_Control(void)
{
  for (uint8_t i = 0; i < JOINT_CNT; i++) //四个关节电机ID参数初始化
  Joint_Motor.YSMotor_Send[i].id = Joint_Motor_ID[i];

  if(Joint_Motor.YSMotor_Reset ==false ) //如果未进行初始复位
	{
    if(joint_Motor_Reset() == true)
    {
      for (uint8_t i = 0; i < JOINT_CNT; i++)
      {
      Joint_Motor.YSMotor_Receive[i].Reset_Pos = Joint_Motor.YSMotor_Receive[i].Pos_output;
      }
     Joint_Motor.YSMotor_Reset = true;  //初始复位完成
    }
    return; //未成功复位就不继续执行下面的的任务
  }

   if (Joint_Motor.stop_flag == MOTOR_STOP) //电机状态为停止位
   { // 若该电机处于停止状态,直接将发送置零
   for (uint8_t i = 0; i < JOINT_CNT; i++)
   {
	 Joint_Motor.YSMotor_Send[i].id = Joint_Motor_ID[i];
   Joint_Motor.YSMotor_Send[i].mode=1;
   Joint_Motor.YSMotor_Send[i].T= 0;
   Joint_Motor.YSMotor_Send[i].W=0;
   Joint_Motor.YSMotor_Send[i].Pos=0;
   Joint_Motor.YSMotor_Send[i].K_P=0;
   Joint_Motor.YSMotor_Send[i].K_W=0.0;
	 }

   }
   for (uint8_t i = 0; i < JOINT_CNT; i++)
   {
    SERVO_Send_recv(&Joint_Motor.YSMotor_Send[i], &Joint_Motor.YSMotor_Receive[i]);	//将控制指令发送给电机
   }
    
}




