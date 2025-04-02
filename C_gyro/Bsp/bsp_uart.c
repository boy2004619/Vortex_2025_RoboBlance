#include "bsp_uart.h"
#include "usart.h"
#include "ins_task.h"
#include "cmsis_os.h"
#include "buzzer.h"

typedef __packed struct IMU_Data_Packet_t
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
}IMU_Data_Packet_t;

IMU_Data_Packet_t IMU_Data;


uint8_t send_flag = 1 , data_ture;
int count;
void Send_Task(void *argument)
{
  /* USER CODE BEGIN Send_Task */
//	Start_Buzzer_remind();
  /* Infinite loop */
  for(;;)
  {
		count++;
		IMU_Data.header1 = 0XA5;
		IMU_Data.header2 = 0X5A;
		
		memcpy(&IMU_Data.q, &INS.q, sizeof(INS.q));
		memcpy(&IMU_Data.Gyro, &INS.Gyro, sizeof(INS.Gyro));
		memcpy(&IMU_Data.Accel, &INS.Gyro, sizeof(INS.Gyro));
		memcpy(&IMU_Data.MotionAccel_b, &INS.MotionAccel_b, sizeof(INS.MotionAccel_b));
		memcpy(&IMU_Data.MotionAccel_n, &INS.MotionAccel_n, sizeof(INS.MotionAccel_n));
		IMU_Data.Pitch = INS.Pitch;
		IMU_Data.Roll = INS.Roll;
		IMU_Data.Yaw = INS.Yaw;
		IMU_Data.YawTotalAngle = INS.YawTotalAngle;
		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&IMU_Data, sizeof(IMU_Data));
		if(send_flag) {
		Start_Buzzer_remind();
		send_flag  = 0;
		}
		if(count == 1)
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_RESET);
		if(count%500 == 0) HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11);
		
    osDelay(1);
  }
  /* USER CODE END Send_Task */
}



