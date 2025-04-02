#include "MotorTask.h"

float LKMotor1_Speed_Hope=360.0f;

float del_t1;
uint32_t Chassis_dwt_cnt;


void Motor_Task(void *argument) 
{
  /* USER CODE BEGIN Motor_Task */
  Motor_enable();
  Joint_Motor.YSMotor_Reset =false;
	PIDInit();

  /* Infinite loop */
  for(;;)
  {
  //  del_t1  = DWT_GetDeltaT(&Chassis_dwt_cnt);
   Motor_control();
   Shoot_Motor_Control();
   Yaw_Motor_Control();
   YS_Motor_Control();
	 LKMotorControl();

  vTaskDelay(1);
  }
  /* USER CODE END Motor_Task */
}

void Chassis_Task(void *argument)
{
  /* USER CODE BEGIN Motor_Task */
	BalanceInit();
  Gimbal_Init();
  Shoot_Init();
  RoboState_Init();
  Open_Hatch();
  /* Infinite loop */
  for(;;)
  {
  del_t1  = DWT_GetDeltaT(&Chassis_dwt_cnt);
  RoboState_Change(&RC_Rx,&PC_Rx,&Robot_RunState);
  SolveTrajectory_Parameter_Updata();
  // SuperCap_send_data();
  Shoot_Task();
  GimbalTask();
	BalanceTask();
  vTaskDelay(1);
  }
  /* USER CODE END Motor_Task */
}
