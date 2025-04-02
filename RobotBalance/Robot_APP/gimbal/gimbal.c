#include "gimbal.h"
#include "SolveTrajectory.h"

/**
 * @file   gimbal.c
 * @author Vortex.D
 * 
 * @What? :       云台算法文件。
 * @How to use? : 整个云台控制不过就是完成Yaw和Pitch的角度跟踪，比较简单。
 *                电机接口在这里被调用。  
 * @todo: 考虑完成接口的独立：设置gimbaldata结构体设置数据用于传出电机输入值。在电机文件中设置传入电机输入值。实现数据接口函数，转存数据。
 * 
 * 
 */
/********** @Include User Function **********/

static void gimbal_PID_init(void);     /* 云台PID参数初始化 */
       void Gimbal_Init(void);         /* 云台总初始化 */
       void GimbalTask(void);          /* 云台任务函数 */

/********** @Include User Function **********/


PIDInstance gimbal_yaw_speed, gimbal_yaw_angle; // yaw轴DJI6020角度环和速度环PID结构体
PIDInstance gimbal_pitch_speed, gimbal_pitch_angle ; //pitch轴DM6220电机角度环和速度环PID结构体

uint8_t gimbal_init_flag =0;  //云台初始角度

extern INS_t Gimbal_IMU_data_text;



void Gimbal_Task(void *argument)
{
  /* USER CODE BEGIN Gimbal_Task */
    // Gimbal_Init();
  /* Infinite loop */
  for(;;)
  {
    // GimbalTask();

    // Motor_control();
    // Yaw_Motor_Control();
    osDelay(1000);
  }
  /* USER CODE END Gimbal_Task */
}

void gimbal_userPID_init(void)
{
gimbal_yaw_angle.Kp = 100 ;//100;
gimbal_yaw_angle.Ki = 0;
gimbal_yaw_angle.Kd = 10;
gimbal_yaw_angle.MaxOut = 1000;
gimbal_yaw_angle.DeadBand = 0.0f;
gimbal_yaw_angle.Improve = (PID_Improvement_e)(PID_OutputFilter);
gimbal_yaw_angle.Output_LPF_RC = 0.005;


gimbal_yaw_speed.Kp = 700 ;
gimbal_yaw_speed.Ki = 0;
gimbal_yaw_speed.Kd = 0;
gimbal_yaw_speed.MaxOut = 25000;
gimbal_yaw_speed.DeadBand = 0.0f;
gimbal_yaw_speed.IntegralLimit = 10000;
gimbal_yaw_speed.Improve = (PID_Improvement_e)(PID_Integral_Limit|PID_OutputFilter );
gimbal_yaw_speed.Output_LPF_RC = 0.05;
}
void gimbal_aimPID_init(void)
{

gimbal_yaw_angle.Kp = 100 ;//100;
gimbal_yaw_angle.Ki = 0;
gimbal_yaw_angle.Kd = 10;
gimbal_yaw_angle.MaxOut = 1000;
gimbal_yaw_angle.DeadBand = 0.0f;
gimbal_yaw_angle.Improve = (PID_Improvement_e)(PID_OutputFilter);
gimbal_yaw_angle.Output_LPF_RC = 0.005;


gimbal_yaw_speed.Kp = 2000 ;//700;
gimbal_yaw_speed.Ki = 0;
gimbal_yaw_speed.Kd = 0;
gimbal_yaw_speed.MaxOut = 25000;
gimbal_yaw_speed.DeadBand = 0.0f;
gimbal_yaw_speed.IntegralLimit = 10000;
gimbal_yaw_speed.Improve = (PID_Improvement_e)(PID_Integral_Limit|PID_OutputFilter );
gimbal_yaw_speed.Output_LPF_RC = 0.05;
}

void gimbal_PID_init(void)/* 云台PID参数初始化 */
{

gimbal_yaw_angle.Kp = 100;
gimbal_yaw_angle.Ki = 0;
gimbal_yaw_angle.Kd = 10;
gimbal_yaw_angle.MaxOut = 1000;
gimbal_yaw_angle.DeadBand = 0.0f;
gimbal_yaw_angle.Improve = (PID_Improvement_e)(PID_OutputFilter);
gimbal_yaw_angle.Output_LPF_RC = 0.005;

// gimbal_yaw_speed.Kp = 500;
// gimbal_yaw_speed.Ki = 0;
// gimbal_yaw_speed.Kd = 0;
// gimbal_yaw_speed.MaxOut = 16384;
// gimbal_yaw_speed.DeadBand = 0.0f;
// gimbal_yaw_speed.IntegralLimit = 1000;
// gimbal_yaw_speed.Improve = (PID_Improvement_e)(PID_Integral_Limit|PID_OutputFilter );
// gimbal_yaw_speed.Output_LPF_RC = 0.05;

//6020电压环模式
gimbal_yaw_speed.Kp = 700;
gimbal_yaw_speed.Ki = 0;
gimbal_yaw_speed.Kd = 0;
gimbal_yaw_speed.MaxOut = 25000;
gimbal_yaw_speed.DeadBand = 0.0f;
gimbal_yaw_speed.IntegralLimit = 10000;
gimbal_yaw_speed.Improve = (PID_Improvement_e)(PID_Integral_Limit|PID_OutputFilter );
gimbal_yaw_speed.Output_LPF_RC = 0.05;

// gimbal_pitch_angle.Kp = 500;
// gimbal_pitch_angle.Ki = 0.0;
// gimbal_pitch_angle.Kd = 15;
// gimbal_pitch_angle.MaxOut = 45;
// gimbal_pitch_angle.DeadBand = 0.0f;
// gimbal_pitch_angle.Improve = (PID_Improvement_e)(PID_OutputFilter);
// gimbal_pitch_angle.Derivative_LPF_RC = 0.005;

// gimbal_pitch_speed.Kp = 0.04;
// gimbal_pitch_speed.Ki = 0.03;
// gimbal_pitch_speed.Kd = 0;
// gimbal_pitch_speed.CoefA =0;
// gimbal_pitch_speed.CoefB =5;
// gimbal_pitch_speed.IntegralLimit=5;
// gimbal_pitch_speed.MaxOut = 18;
// gimbal_pitch_speed.DeadBand = 0.0f;
// gimbal_pitch_speed.Improve = (PID_Improvement_e)(PID_OutputFilter | PID_ChangingIntegrationRate |PID_Integral_Limit);
// gimbal_pitch_speed.Output_LPF_RC = 0.005;

gimbal_pitch_angle.Kp = 750;
gimbal_pitch_angle.Ki = 0.0;
gimbal_pitch_angle.Kd = 20;
gimbal_pitch_angle.MaxOut = 45;
gimbal_pitch_angle.DeadBand = 0.0f;
gimbal_pitch_angle.Improve = (PID_Improvement_e)(PID_OutputFilter);
gimbal_pitch_angle.Derivative_LPF_RC = 0.005;

gimbal_pitch_speed.Kp = 0.04;
gimbal_pitch_speed.Ki = 0.03;
gimbal_pitch_speed.Kd = 0;
gimbal_pitch_speed.CoefA =0;
gimbal_pitch_speed.CoefB =5;
gimbal_pitch_speed.IntegralLimit=5;
gimbal_pitch_speed.MaxOut = 18;
gimbal_pitch_speed.DeadBand = 0.0f;
gimbal_pitch_speed.Improve = (PID_Improvement_e)(PID_OutputFilter | PID_ChangingIntegrationRate |PID_Integral_Limit);
gimbal_pitch_speed.Output_LPF_RC = 0.005;

}

void Gimbal_Init(void)/* 云台总初始化 */
{
    gimbal_PID_init();

    while(INS.ins_flag==0)//等待加速度收敛
	{
	  osDelay(1);	
	}
    /* 在加速度收敛后，将期望角度设置为当前初始角度 */
    gimbal_yaw_angle.Ref = Gimbal_IMU_data.Yaw;

}

uint32_t Gimbal_dwt_cnt;
float Gimbaldel_t;
int GimbalTask_falg=0;
void GimbalTask(void)/* 云台任务函数 */
{
    // Gimbaldel_t = DWT_GetDeltaT(&Gimbal_dwt_cnt); //测试使用
    // GimbalTask_falg++;

    if( Robot_RunState.Robot_stop ) gimbal_pitch_speed.Iout = 0;
    /* 视觉模式切换 */
    if(Robot_RunState.auto_aim && Vision_Receive_Data.tracking ==1){

    gimbal_aimPID_init();  //初始化自瞄结构体
    gimbal_yaw_angle.Ref = yaw;
    gimbal_pitch_angle.Ref = pitch;
    // Vision_Receive_Data.tracking = 0;
    }
    else {

    gimbal_userPID_init();//初始化用户结构体
    gimbal_yaw_angle.Ref -= Robot_RunState.Yaw;
    gimbal_pitch_angle.Ref -= Robot_RunState.Pitch;
    }

    if(Robot_RunState.open_mag) Open_Hatch();
    else Close_Hatch();

    VAL_LIMIT( gimbal_pitch_angle.Ref,-0.7f,0.5f);
	
	if(gimbal_yaw_angle.Ref-Gimbal_IMU_data.Yaw>PI)  //限制误差在合理范围，防止疯头
	 gimbal_yaw_angle.Ref-=2*PI;
	else if(gimbal_yaw_angle.Ref-Gimbal_IMU_data.Yaw< -PI)
	 gimbal_yaw_angle.Ref+=2*PI;


  static float yaw_speed_ff, yaw_speed_k = -10;//-3000; 
  yaw_speed_ff = yaw_speed_k * steer_v_pid.Measure; // 用于小陀螺时yaw的前馈

  PIDCalculate(&gimbal_yaw_angle,Gimbal_IMU_data.Yaw, gimbal_yaw_angle.Ref);  //yaw角度环
  gimbal_yaw_speed.Ref = gimbal_yaw_angle.Output + yaw_speed_ff;

  PIDCalculate(&gimbal_yaw_speed, DJI_Motor.DJI_G6020_Receive[0].rpm,gimbal_yaw_speed.Ref); //yaw速度环
  // PIDCalculate(&gimbal_yaw_speed, DJI_Motor.DJI_G6020_Receive[0].rpm, RC_Rx.CH0 *0.3);

  PIDCalculate(&gimbal_pitch_angle, Gimbal_IMU_data.Pitch, gimbal_pitch_angle.Ref);//pitch角度环
  gimbal_pitch_speed.Ref = gimbal_pitch_angle.Output;
  PIDCalculate(&gimbal_pitch_speed, DM_Motor.DM_G6220_Receive[0].Vel, gimbal_pitch_speed.Ref);//pitch速度环


    DM_Motor.Motor_Out[0]  = gimbal_pitch_speed.Output ;
    DJI_Motor.Motor_Out[0] = gimbal_yaw_speed.Output ;//+yaw_speed_ff;

}
