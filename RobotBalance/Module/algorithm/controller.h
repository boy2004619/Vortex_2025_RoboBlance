/**
 ******************************************************************************
 * @file	 controller.h
 * @author  Wang Hongxi
 * @version V1.1.3
 * @date    2021/7/3
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "main.h"

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

// PID 优化环节使能标志位,通过位与可以判断启用的优化环节;也可以改成位域的形式
typedef enum
{
    PID_IMPROVE_NONE = 0b00000000,                // 0000 0000
    PID_Integral_Limit = 0b00000001,              // 0000 0001
    PID_Derivative_On_Measurement = 0b00000010,   // 0000 0010
    PID_Trapezoid_Intergral = 0b00000100,         // 0000 0100
    PID_Proportional_On_Measurement = 0b00001000, // 0000 1000
    PID_OutputFilter = 0b00010000,                // 0001 0000
    PID_ChangingIntegrationRate = 0b00100000,     // 0010 0000
    PID_DerivativeFilter = 0b01000000,            // 0100 0000
    PID_ErrorHandle = 0b10000000,                 // 1000 0000
} PID_Improvement_e;

/* PID 报错类型枚举*/
typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    PID_MOTOR_BLOCKED_ERROR = 0x01U
} ErrorType_e;

typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

/* PID结构体 */
typedef struct {
    // -------------------------------- init config block
    // PID 控制器的配置参数

    float Kp;       // 比例增益
    float Ki;       // 积分增益
    float Kd;       // 微分增益
    float MaxOut;   // PID 输出最大值限制
    float DeadBand; // 误差的死区范围，用于消除小的误差信号

    // PID 控制器的性能改进参数

    PID_Improvement_e Improve;// 改进模式枚举，定义了改进 PID 性能的策略
    float IntegralLimit;     	// 积分项限制，防止积分饱和
    float CoefA;             	// 变速积分系数 A，用于动态调整积分项
    float CoefB;             	// 变速积分系数 B，与 CoefA 配合使用
    float Output_LPF_RC;     	// 输出低通滤波器的时间常数，用于平滑输出
    float Derivative_LPF_RC; 	// 微分低通滤波器的时间常数，用于减少噪声

    //-----------------------------------
    // 用于 PID 计算的变量
		
    float Measure;       // 当前的测量值或反馈值
    float Last_Measure;  // 上一次的测量值
    float Err;           // 当前误差，计算为 (Ref - Measure)
    float Last_Err;      // 上一次的误差
    float Last_Err_Dev;  // 上一次误差的微分，用于计算微分项

    float Pout;  // 当前比例项的输出
    float Iout;  // 当前积分项的输出
    float Dout;  // 当前微分项的输出
    float ITerm; // 当前积分项的累积值
    float Last_ITerm;  // 上一次的积分项累积值

    float Output;       // PID 计算的总输出
    float Output_Dev;   // 输出的微分，可能用于滤波或限制
    float Last_Output;  // 上一次的 PID 输出
    float Last_Dout;    // 上一次的微分项输出

    float Ref;          // PID 控制器的参考输入或期望值

    uint32_t DWT_CNT;   // 用于计时的周期计数器，基于 DWT（Data Watchpoint and Trace）单元
    float dt;           // 自上次计算以来的时间间隔

    PID_ErrorHandler_t ERRORHandler; // 错误处理结构体，包含错误检测和处理的相关信息
} PIDInstance;


struct PID
{
	float kp;
	float ki;
	float kd;
	
	float err;
	float err_last;
	float err_last_dev;
	float err_last_dev_last;
	float err_last_dev_lastdev;
	float err_add;
	float err_add_limit;
	
	float ki_p;
	
	
	float hope;
	float real;
	
	float out;
	float out_limit;
};

typedef struct
{
    float ts;       //采样周期(s)
    float fc;       //截至频率(hz)
    float lastYn;   //上一次滤波值
    float alpha;    //滤波系数
} low_pass_filter_t;
//初始化滤波系数
void Init_lowPass_alpha(low_pass_filter_t* const filter,const float ts, const float fc);
//低通滤波
float Low_pass_filter(low_pass_filter_t* const filter, const float data);

/**
 * @brief 初始化PID实例
 */
void PIDInit(void);

/**
 * @brief 计算PID输出
 *
 * @param pid     PID实例指针
 * @param measure 反馈值
 * @param ref     设定值
 * @return float  PID计算输出
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref);

float Erect_pid(struct PID* para,float hope, float now);
float Increment_pid(struct PID* para,float hope,float now);

/* ----------------------------PID结构体全局申明区域---------------------------- */

extern PIDInstance LKMotor1_ctrl,LKMotor2_ctrl;
extern PIDInstance leglen_pid_l, leglen_pid_r; // 用PD模拟弹簧, 不要积分(弹簧是无积分二阶系统), 增益不可过大否则抗外界冲击响应时太"硬"
extern PIDInstance steer_p_pid, steer_v_pid;   // 转向PID
extern PIDInstance roll_compensate_pid;        // roll轴补偿,用于保持机体水平
extern PIDInstance anti_crash_pid;             // 抗劈叉,将输出以相反的方向叠加到左右腿的上

extern low_pass_filter_t YS_Motor_lowpassfilter ,chasiss_vel_lowpassfilter;
#endif
