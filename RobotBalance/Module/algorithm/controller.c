/**
 * @file controller.c
 * @author wanghongxi
 * @author modified by neozng
 * @brief  PID控制器定义
 * @version beta
 * @date 2022-11-01
 *
 * @copyrightCopyright (c) 2022 HNU YueLu EC all rights reserved
 */
#include "controller.h"
#include "bsp_dwt.h"

/* ----------------------------下面是定义PID参数结构体区域---------------------------- */

PIDInstance LKMotor1_ctrl,LKMotor2_ctrl;
PIDInstance leglen_pid_l, leglen_pid_r; // 用PD模拟弹簧, 不要积分(弹簧是无积分二阶系统), 增益不可过大否则抗外界冲击响应时太"硬"
PIDInstance steer_p_pid, steer_v_pid;   // 转向PID
PIDInstance roll_compensate_pid;        // roll轴补偿,用于保持机体水平
PIDInstance anti_crash_pid;             // 抗劈叉,将输出以相反的方向叠加到左右腿的上

/* ------------------------------------------------------------------------------------- */

low_pass_filter_t YS_Motor_lowpassfilter ,chasiss_vel_lowpassfilter;


/* ----------------------------下面是pid优化环节的实现---------------------------- */

// 梯形积分
static void f_Trapezoid_Intergral(PIDInstance *pid)
{
    // 计算梯形的面积,(上底+下底)*高/2
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

// 变速积分(误差小时积分作用更强)
static void f_Changing_Integration_Rate(PIDInstance *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 积分呈累积趋势
        if (abs(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (abs(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - abs(pid->Err) + pid->CoefB) / pid->CoefA;
        else // 最大阈值,不使用积分
           {
           pid->Iout = 0;
           pid->ITerm = 0;
           }
    }
}

static void f_Integral_Limit(PIDInstance *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (abs(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0) // 积分却还在累积
        {
            pid->ITerm = 0; // 当前积分项置零
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

// 微分先行(仅使用反馈值而不计参考输入的微分)
static void f_Derivative_On_Measurement(PIDInstance *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// 微分滤波(采集微分时,滤除高频噪声)
static void f_Derivative_Filter(PIDInstance *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

// 输出滤波
static void f_Output_Filter(PIDInstance *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

// 输出限幅
static void f_Output_Limit(PIDInstance *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

// 电机堵转检测
static void f_PID_ErrorHandle(PIDInstance *pid)
{
    /*Motor Blocked Handle*/
    if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        // Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    }
}

/* ---------------------------下面是PID的外部算法接口--------------------------- */

void PIDInit(void)
{
    leglen_pid_l.Kp = 500;//400;//600;
    leglen_pid_l.Kd = 100;//200
    leglen_pid_l.Ki = 0.0;
    leglen_pid_l.MaxOut = 120;
    leglen_pid_l.DeadBand = 0.0001f;
    leglen_pid_l.Improve = (PID_Improvement_e)( PID_DerivativeFilter | PID_Derivative_On_Measurement);
    leglen_pid_l.Derivative_LPF_RC = 0.05;

    leglen_pid_r.Kp = 500;//400;//600;
    leglen_pid_r.Kd = 100;//200;
    leglen_pid_r.Ki = 0;
    leglen_pid_r.MaxOut = 120;
    leglen_pid_r.DeadBand = 0.0001f;
    leglen_pid_r.Improve = (PID_Improvement_e)(PID_DerivativeFilter | PID_Derivative_On_Measurement);
    leglen_pid_r.Derivative_LPF_RC = 0.05;

    // steer_p_pid.Kp = 5;
    // steer_p_pid.Kd = 0;
    // steer_p_pid.Ki = 0.0f;
    // steer_p_pid.MaxOut = 3;
    // steer_p_pid.DeadBand = 0.001f;
    // steer_p_pid.Improve = (PID_Improvement_e)(PID_DerivativeFilter | PID_Derivative_On_Measurement);
    // steer_p_pid.Derivative_LPF_RC = 0.05;

    // steer_v_pid.Kp = 3;
    // steer_v_pid.Kd = 0.0f;
    // steer_v_pid.Ki = 0.0f;
    // steer_v_pid.MaxOut = 20;
    // steer_v_pid.DeadBand = 0.0f;
    // steer_v_pid.Improve = (PID_Improvement_e)(PID_DerivativeFilter | PID_Derivative_On_Measurement);
    // steer_v_pid.Derivative_LPF_RC = 0.05;

    steer_p_pid.Kp = -0.005;
    steer_p_pid.Kd = 0;
    steer_p_pid.Ki = 0.0f;
    steer_p_pid.MaxOut = 3;
    steer_p_pid.DeadBand = 0.001f;
    steer_p_pid.Improve = (PID_Improvement_e)(PID_DerivativeFilter | PID_Derivative_On_Measurement);
    steer_p_pid.Derivative_LPF_RC = 0.05;

    steer_v_pid.Kp = 3;
    steer_v_pid.Kd = 0.0f;
    steer_v_pid.Ki = 0.0f;
    steer_v_pid.MaxOut = 20;
    steer_v_pid.DeadBand = 0.0f;
    steer_v_pid.Improve = (PID_Improvement_e)(PID_DerivativeFilter | PID_Derivative_On_Measurement);
    steer_v_pid.Derivative_LPF_RC = 0.05;

//Roll轴补偿
    roll_compensate_pid.Kp = 0.2f,
    roll_compensate_pid.Kd = 0.005f,
    roll_compensate_pid.Ki = 0.0f,
    roll_compensate_pid.MaxOut = 0.05,
    roll_compensate_pid.DeadBand = 0.001f,
    roll_compensate_pid.Improve = (PID_Improvement_e)(PID_DerivativeFilter | PID_Derivative_On_Measurement),
    roll_compensate_pid.Derivative_LPF_RC = 0.05,

    anti_crash_pid.Kp = 15,
    anti_crash_pid.Kd = 2,
    anti_crash_pid.Ki = 0.0,
    anti_crash_pid.MaxOut = 10,
    anti_crash_pid.DeadBand = 0.001f,
    anti_crash_pid.Improve = (PID_Improvement_e)(PID_DerivativeFilter | PID_ChangingIntegrationRate | PID_Integral_Limit),
    anti_crash_pid.Derivative_LPF_RC = 0.01,

    Init_lowPass_alpha(&chasiss_vel_lowpassfilter,0.001f,1);
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    // 堵转检测
    if (pid->Improve & PID_ErrorHandle)
        f_PID_ErrorHandle(pid);

    pid->dt = DWT_GetDeltaT(&pid->DWT_CNT); // 获取两次pid计算的时间间隔,用于积分和微分

    // 保存上次的测量值和误差,计算当前error
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    // 如果在死区外,则计算PID
    if (abs(pid->Err) > pid->DeadBand)
    {
        // 基本的pid计算,使用位置式
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

        // 梯形积分
        if (pid->Improve & PID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        // 变速积分
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // 微分先行
        if (pid->Improve & PID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        // 微分滤波器
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);
        // 积分限幅
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;                         // 累加积分
        pid->Output = pid->Pout + pid->Iout + pid->Dout; // 计算输出

        // 输出滤波
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // 输出限幅
        f_Output_Limit(pid);
    }
    else // 进入死区, 则清空积分和输出
    {
        pid->Output = 0;
        pid->ITerm = 0;
    }

    // 保存当前数据,用于下次计算
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;
		
		pid->Output_Dev+=pid->Output; //相当与增量式输出

    return pid->Output;
}


/**********************************************************************************************************
*函 数 名: Erect_pid
*功能说明: 位置式PID
*形    参: PID结构体句柄  期望值  实际值
*返 回 值: PID运算结果输出
**********************************************************************************************************/
float Erect_pid(struct PID* para,float hope, float now)
{
	(*para).err = now - hope;				//偏差
	(*para).err_last_dev = (*para).err - (*para).err_last; //偏差的微分
	(*para).out = (*para).kp*(*para).err + (*para).kd*(*para).err_last_dev + (*para).ki*(*para).err_add;
	(*para).err_last =  (*para).err;  //偏差的结果保留至下一次
	(*para).err_add+=(*para).err;			//偏差的积分
	
	if((*para).err_add > (*para).err_add_limit)   (*para).err_add =(*para).err_add_limit; 	//积分限幅
	if((*para).err_add < -(*para).err_add_limit)	(*para).err_add =-(*para).err_add_limit;
	
	if((*para).out > (*para).out_limit)   (*para).out =(*para).out_limit;		//输出限幅
	if((*para).out < -(*para).out_limit)	(*para).out =-(*para).out_limit;
	
	return (*para).out;
}

/**********************************************************************************************************
*函 数 名: Increment_pid
*功能说明: 增置式PID
*形    参: PID结构体句柄  期望值  实际值
*返 回 值: PID运算结果输出
**********************************************************************************************************/
float Increment_pid(struct PID* para,float hope,float now)
{
	(*para).err = now - hope;  																								//本次偏差
	(*para).err_last_dev	= (*para).err-(*para).err_last; 				  //获取偏差增量
	(*para).out += (*para).kp*(*para).err_last_dev + (*para).ki*(*para).err;  //对输出进行计算及累加
	(*para).err_last = (*para).err;  																					//保存至下一次运算的上次偏差
	
	if((*para).out > (*para).out_limit)   (*para).out =(*para).out_limit;     //输出限幅
	if((*para).out < -(*para).out_limit)	(*para).out =-(*para).out_limit;
	return (*para).out;  																											//输出本次运算结果
}



//源文件
/*******************************************************************************
 * @fn Init_lowPass_alpha
 * @brief 初始化低通滤波器滤波系数
 * @param filter 滤波器
 * @param ts 采用周期 单位s
 * @return fc 截至频率 单位hz
 ******************************************************************************/
void Init_lowPass_alpha(low_pass_filter_t* const filter,const float ts, const float fc)
{
  float b=2*PI*fc*ts;
  filter->ts=ts;
  filter->fc=fc;
  filter->lastYn=0;
  filter->alpha=b/(b+1);
}

/*******************************************************************************
 * @fn Low_pass_filter
 * @brief 低通滤波函数
 * @param data 采样数据
 * @return 滤波结果
 ******************************************************************************/
float Low_pass_filter(low_pass_filter_t* const filter, const float data)
{
  float tem=filter->lastYn+(filter->alpha*(data-filter->lastYn));
  filter->lastYn=tem;
  return tem;
  
}

