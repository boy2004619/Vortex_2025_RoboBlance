#include "shoot.h"
/**
 * @file   shoot.c
 * @author Vortex.D
 * 
 * @What? :       发射机构算法文件。
 * @How to use? : 摩擦轮和拨弹轮的控制。
 * @todo:   暂无
 * 
 */
/********** @Include User Function **********/

static void shoot_PID_init(void);      /* 发射机构PID参数初始化 */
       void Shoot_Motor_Control(void); /* 发射机构电机数据写入 */
       void Shoot_Init(void);          /* 发射机构总初始化 */
       void Shoot_Task(void);          /* 发射机构任务函数 */

/********** @Include User Function **********/

PIDInstance friction_l, friction_r; // 左右摩擦轮速度环
PIDInstance loader_speed; // 拨盘电机速度环

static void shoot_PID_init(void)
{
friction_l.Kp = 1;//20
friction_l.Ki = 0;//1
friction_l.Kd = 0;
friction_l.Derivative_LPF_RC = 0.02;
friction_l.Improve = (PID_Improvement_e)(PID_Integral_Limit| PID_Trapezoid_Intergral | PID_DerivativeFilter);
friction_l.IntegralLimit = 10000;
friction_l.MaxOut = 16384.0f;

friction_r.Kp = 1;//20
friction_r.Ki = 0;//1
friction_r.Kd = 0;
friction_r.Derivative_LPF_RC = 0.02;
friction_r.Improve = (PID_Improvement_e)(PID_Integral_Limit| PID_Trapezoid_Intergral | PID_DerivativeFilter);
friction_r.IntegralLimit = 10000;
friction_r.MaxOut = 16384.0f;

loader_speed.Kp = 0.5;//0
loader_speed.Ki = 0.1;//0
loader_speed.Kd = 0.002;
loader_speed.Improve = PID_Integral_Limit;
loader_speed.IntegralLimit = 200;
loader_speed.MaxOut = 10000;

}

void Shoot_Motor_Control (void)
{
//   if(RC_Rx.S2 == 2 ) DJI_Motor.DJI_M2006_Send[0].Motor_Out = 0;
//   DJI_Motor.DJI_M3508_Send[0].Motor_Out = DJI_Motor.DJI_M3508_Send[1].Motor_Out = 0;
  
  DJI_Motor_Send(&hfdcan2 ,0X200 ,DJI_Motor.DJI_M3508_Send[0].Motor_Out ,DJI_Motor.DJI_M3508_Send[1].Motor_Out,DJI_Motor.DJI_M2006_Send[0].Motor_Out,0);

}

void Shoot_Init(void)
{
  shoot_PID_init();
}

void Shoot_Task(void)
{

 if(Robot_RunState.Robot_friction)
  {
    friction_l.Ref =-26800*0.25;
    friction_r.Ref =26800*0.25;
  }
 else 
  {
    friction_l.Ref =0;
    friction_r.Ref =0;
  }

 if(Robot_RunState.fire)
  {
    loader_speed.Ref = 6000; // 4000;//2000;
  }
 else
  {
    loader_speed.Ref=0;
  }

  if(Robot_RunState.F_reversal){
    friction_l.Ref =26800*0.30;
    friction_r.Ref =-26800*0.30;
    loader_speed.Ref = -2000;
  }

 PIDCalculate(&friction_l, DJI_Motor.DJI_M3508_Receive[0].rpm, friction_l.Ref);
 PIDCalculate(&friction_r, DJI_Motor.DJI_M3508_Receive[1].rpm, friction_r.Ref);
 
 PIDCalculate(&loader_speed, DJI_Motor.DJI_M2006_Receive[0].rpm, loader_speed.Ref);

 DJI_Motor.DJI_M3508_Send[0].Motor_Out = friction_l.Output;
 DJI_Motor.DJI_M3508_Send[1].Motor_Out = friction_r.Output;

 DJI_Motor.DJI_M2006_Send[0].Motor_Out = loader_speed.Output;

}
