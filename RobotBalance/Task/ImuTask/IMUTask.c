#include "IMUTask.h"
#include "ins_task.h"


static uint8_t X=0,Y=1,Z=2;
uint32_t balance_IMU_cnt;
float imu_del_t;
float IMU_dataPitch;
INS_t Gimbal_IMU_data_text;

//陀螺仪姿态获取任务，在使用该函数前请确保在CubeMX中设置了IMU_Task任务
void IMU_Task(void *argument)
{
  /* USER CODE BEGIN IMU_Task */
	Chassis_IMU_data.AccelLPF = 0.0085;
	INS_Init();
  /* Infinite loop */
  for(;;)
  {
    // static float Pitch_Acc_last,Roll_Acc_last,Yaw_Acc_last;
    const float gravity[3] = {0, 0, 9.81f};
    imu_del_t = DWT_GetDeltaT(&balance_IMU_cnt);

	  Chassis_IMU_data.Pitch =IMUC_Data.Pitch *0.0174532f;  //角度  rad
    Chassis_IMU_data.Roll  =IMUC_Data.Roll *0.0174532f;
    Chassis_IMU_data.Yaw   =IMUC_Data.Yaw *0.0174532f;

    Chassis_IMU_data.Gyro[X] =IMUC_Data.Gyro[X]; //角速度   rad/s
    Chassis_IMU_data.Gyro[Y] =IMUC_Data.Gyro[Y];
    Chassis_IMU_data.Gyro[Z] =IMUC_Data.Gyro[Z];
		 
		Chassis_IMU_data.Accel[X] = IMUC_Data.Accel[X];  //加速度  m/s^2
    Chassis_IMU_data.Accel[Y] = IMUC_Data.Accel[Y];
    Chassis_IMU_data.Accel[Z] = IMUC_Data.Accel[Z];
		 
		Chassis_IMU_data.q[0] = IMUC_Data.q[0]; //四元数没必要传，后续可以连同C板发送的数据去掉四元数的发送和接收
		Chassis_IMU_data.q[1] = IMUC_Data.q[1];
    Chassis_IMU_data.q[2] = IMUC_Data.q[2];
		Chassis_IMU_data.q[3] = IMUC_Data.q[3];

    Chassis_IMU_data.MotionAccel_b[X] = IMUC_Data.MotionAccel_b[X];
    Chassis_IMU_data.MotionAccel_b[Y] = IMUC_Data.MotionAccel_b[Y];
    Chassis_IMU_data.MotionAccel_b[Z] = IMUC_Data.MotionAccel_b[Z];

    Chassis_IMU_data.MotionAccel_n[X] = IMUC_Data.MotionAccel_n[X];
    Chassis_IMU_data.MotionAccel_n[Y] = IMUC_Data.MotionAccel_n[Y];
    Chassis_IMU_data.MotionAccel_n[Z] = IMUC_Data.MotionAccel_n[Z];

/*  当底盘使用的是其他外挂陀螺仪（不是C板）时使用这个算出 MotionAccel_b ，MotionAccel_n （如果你要使用）


      //将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
      float gravity_b[3];
      EarthFrameToBodyFrame(gravity, gravity_b, Chassis_IMU_data.q);
      for (uint8_t i = 0; i < 3; ++i) // 同样过一个低通滤波
      {
        Chassis_IMU_data.MotionAccel_b[i] = (Chassis_IMU_data.Accel[i] - gravity_b[i]) * imu_del_t / (Chassis_IMU_data.AccelLPF + imu_del_t) + Chassis_IMU_data.MotionAccel_b[i] * Chassis_IMU_data.AccelLPF / (Chassis_IMU_data.AccelLPF + imu_del_t);
      }
      BodyFrameToEarthFrame(Chassis_IMU_data.MotionAccel_b, Chassis_IMU_data.MotionAccel_n, Chassis_IMU_data.q); // 转换回导航系n

 */

  //云台达妙板载陀螺仪的读取
  INS_task();
  memcpy(&Gimbal_IMU_data, &INS, sizeof(INS_t));

  vTaskDelay(1);
  }
  /* USER CODE END IMU_Task */
}

