#include "ins_task.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "bsp_PWM.h"
#include "mahony_filter.h"

static uint8_t X=0,Y=1,Z=2; 

INS_t INS;
INS_t Chassis_IMU_data,Gimbal_IMU_data; //

struct MAHONY_FILTER_t mahony;
Axis3f Gyro,Accel;
float gravity[3] = {0, 0, 9.81f};

uint32_t INS_DWT_Count = 0;
float ins_dt = 0.0f;
float ins_time;
int stop_time;

low_pass_filter_t Gyrox_lowpassfilter;


#define DES_TEMP    40.0f
#define KP          100.f
#define KI          50.f
#define KD          10.f
#define MAX_OUT     500

float out = 0;
float err = 0;
float err_l = 0;
float err_ll = 0;

void INS_Init(void)
{ 
   mahony_init(&mahony,1.0f,0.0f,0.001f);
   INS.AccelLPF = 0.0025f;
   Init_lowPass_alpha(&Gyrox_lowpassfilter,0.001f,1);   //初始化滤波器
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void INS_task(void)
{
	  
	ins_dt = DWT_GetDeltaT(&INS_DWT_Count);
	mahony.dt = ins_dt;

  BMI088_Read(&BMI088);
  err_ll = err_l;
  err_l = err;
err = DES_TEMP - BMI088.Temperature;
out = KP*err + KI*(err + err_l + err_ll) + KD*(err - err_l);
if (out > MAX_OUT) out = MAX_OUT;
if (out < 0) out = 0.f;
htim3.Instance->CCR4 = (uint16_t)out;

  INS.Accel[X] = BMI088.Accel[X];
  INS.Accel[Y] = BMI088.Accel[Y];
  INS.Accel[Z] = BMI088.Accel[Z];
  Accel.x=BMI088.Accel[0];
  Accel.y=BMI088.Accel[1];
  Accel.z=BMI088.Accel[2];
  INS.Gyro[X] = BMI088.Gyro[X];
  INS.Gyro[Y] = BMI088.Gyro[Y];
  INS.Gyro[Z] = BMI088.Gyro[Z];
  
//    INS.Gyro[X] = Low_pass_filter(&Gyrox_lowpassfilter,INS.Gyro[X]);
  // INS.Gyro[X] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);

  Gyro.x=BMI088.Gyro[0];
	Gyro.y=BMI088.Gyro[1];
	Gyro.z=BMI088.Gyro[2];

	mahony_input(&mahony,Gyro,Accel);
	mahony_update(&mahony);
	mahony_output(&mahony);
	RotationMatrix_update(&mahony);
				
	INS.q[0]=mahony.q0;
	INS.q[1]=mahony.q1;
	INS.q[2]=mahony.q2;
	INS.q[3]=mahony.q3;
       
	float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    for (uint8_t i = 0; i < 3; i++) 
    {
      INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * ins_dt / (INS.AccelLPF + ins_dt) 
														+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + ins_dt); 	
	}
	BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); 
		
		if(fabsf(INS.MotionAccel_n[0])<0.02f)
		{
		  INS.MotionAccel_n[0]=0.0f;
		}
		if(fabsf(INS.MotionAccel_n[1])<0.02f)
		{
		  INS.MotionAccel_n[1]=0.0f;
		}
		if(fabsf(INS.MotionAccel_n[2])<0.04f)
		{
		  INS.MotionAccel_n[2]=0.0f;
			stop_time++;
		}
		if(stop_time>10)
		{
		  stop_time=0;
		  INS.v_n=0.0f;
		}
    		
		if(ins_time>3000.0f)
		{
		  INS.v_n=INS.v_n+INS.MotionAccel_n[1]*0.001f;
		  INS.x_n=INS.x_n+INS.v_n*0.001f;
		  INS.ins_flag=1;
      INS.Pitch=mahony.roll;
		  if(INS.Pitch<0) INS.Pitch+=2*PI;
		  INS.Pitch-=PI;
		  INS.Roll=mahony.pitch;
		  INS.Yaw=mahony.yaw;
		//   INS.Yaw += 0.0001f;
		
		// INS.YawTotalAngle=INS.YawTotalAngle+INS.Gyro[2]*0.001f;
			
			if (INS.Yaw - INS.YawAngleLast > 3.1415926f)
			{
					INS.YawRoundCount--;
			}
			else if (INS.Yaw - INS.YawAngleLast < -3.1415926f)
			{
					INS.YawRoundCount++;
			}
			INS.YawTotalAngle = 6.283f* INS.YawRoundCount + INS.Yaw;
			INS.YawAngleLast = INS.Yaw;
		}
		else
		{
		 ins_time++;
		}


		
} 



/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}




