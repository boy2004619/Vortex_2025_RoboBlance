#include "Vision.h"
#include "SolveTrajectory.h"
#include "usbd_cdc_if.h"

SendPacket Vision_Send_Data;
ReceivePacket Vision_Receive_Data;
void Vision_Send_Data_Task(void);

float Vision_del_t;
uint32_t Vision_dwt_cnt;

void Vision_TASK(void *argument)
{
  /* USER CODE BEGIN Vision_TASK */
  SolveTrajectory_Init();
  /* Infinite loop */
  for(;;)
  {
    Vision_del_t = DWT_GetDeltaT(&Vision_dwt_cnt);  //测试用
    // SolveTrajectory_Parameter_Updata();
    Vision_Send_Data_Task();
    osDelay(10);
  }
  /* USER CODE END Vision_TASK */
}



bool Vision_ReceiveData_Dispose(uint8_t *pData)
{

bool res = false;
if(pData[0] == VISION_FRAME_HEADER )
    {
    // Vision_del_t = DWT_GetDeltaT(&Vision_dwt_cnt);  //测试用
    memcpy(&Vision_Receive_Data, pData, sizeof(Vision_Receive_Data));
    res = true;
    }else res = false;
return res;

}

void Vision_Send_Data_Task(void)
{
  Vision_Send_Data.header=0x5A;
	/* 写入我方 */
	Vision_Send_Data.detect_color= referee_info.GameRobotState.robot_id > 7 ? Robot_Red : Robot_Blue;  //这里是对面的颜色 ， 要反一下。
  Vision_Send_Data.reset_tracker= 0;
	
	Vision_Send_Data.yaw   = Gimbal_IMU_data.Yaw;
	Vision_Send_Data.pitch = Gimbal_IMU_data.Pitch;
	Vision_Send_Data.roll  = Gimbal_IMU_data.Roll;
	
	Vision_Send_Data.aim_x = aim_x;
	Vision_Send_Data.aim_y = aim_y;
	Vision_Send_Data.aim_z = aim_z;

    Append_CRC16_Check_Sum((uint8_t *) &Vision_Send_Data, sizeof(Vision_Send_Data));
    CDC_Transmit_HS((uint8_t *) &Vision_Send_Data, sizeof(Vision_Send_Data)); // 发送
}
