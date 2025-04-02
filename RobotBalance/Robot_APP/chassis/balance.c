#include "balance.h"
#include "linkNleg.h"
#include "lqr_calc.h"
#include "speed_estimation.h"
#include "fly_detection.h"
#include "controller.h"

/**
 * @file balance.c (Chassis.c)
 * @author Vortex.D
 * 
 * @What? :       平衡步兵底盘算法文件。
 * @How to use? : BalanceTask中调用的这个文件中的功能函数，需要修改底盘任务的作用时，在这个文件中修改。
 * @todo:   暂无
 * 
 */
/********** @Include User Function **********/

       void BalanceTask(void);      /* 底盘任务函数 */
       void BalanceInit(void);      /* 底盘数据初始化函数 */
static void EnableAllMotor(void);   /* 打开所有底盘电机 */
static void StopAllMotor(void);     /* 关闭所有底盘电机 */
static void WattLimitSet(void);     /* 设定运动模态的输出 */
static void WokingStateSet(void);   /* 运动状态设定 */
static void SynthesizeMotion(void); /* 腿部控制:抗劈叉; 轮子控制:转向 */
static void ParamAssemble(void);    /* imu和电机的数据组装 */
static void LegControl(void);       /* 腿长控制和Roll补偿 */
static void JumpTASK(void);         /* 跳跃 */

/********** @Include User Function **********/



float sidetarget_len_debug =0.15 ;
float side_F_debug =0 ;
float Jump_num , Jump_over =0 ;

// 计时变量
float del_t;
uint32_t balance_dwt_cnt;
uint32_t BalanceTask_cnt = 0;

//初始底盘正对位置
uint32_t chassis_init_anlge = 4088 + 2048;
 
// 两个腿的参数
LinkNPodParam l_side, r_side;
ChassisParam chassis;// 底盘的参数


void BalanceTask(void)
{
    del_t = DWT_GetDeltaT(&balance_dwt_cnt);  //测试用
    BalanceTask_cnt++ ;

    if(Robot_RunState.Robot_Reset){
    Joint_Motor.YSMotor_Reset = false;  //关节电机复位重新校准
    // 复位时清空距离和腿长积累量,保证顺利站起
    LKMotor.stop_flag = MOTOR_STOP;     //在执行复位时，关闭轮电机
    chassis.dist = chassis.target_dist = 0;
    l_side.target_len = r_side.target_len = 0.15;
    }
    
    if(Joint_Motor.YSMotor_Reset == false)  //如果关节电机没有完成校准就跳出
    return;

    // 设置目标参数和工作模式
    WokingStateSet();
    // 参数组装
    ParamAssemble();
    // 将五连杆映射成单杆
    Link2Leg(&l_side, &chassis);
    Link2Leg(&r_side, &chassis);

    // 通过卡尔曼滤波估计机体速度
    SpeedEstimation(&l_side, &r_side, &chassis, Chassis_IMU_data, del_t);

    // 根据单杆计算处的角度和杆长,计算反馈增益
    CalcLQR(&l_side, &chassis);
    CalcLQR(&r_side, &chassis);

    SynthesizeMotion();

    // if(chassis.jump_state == true)
    // JumpTASK();
    // else
    LegControl();    // 腿长控制,保持机体水平

    // VMC映射成关节输出
    VMCProject(&l_side);
    VMCProject(&r_side);

    //驱动轮支持力解算
    NormalForceSolve(&l_side, Chassis_IMU_data);
    NormalForceSolve(&r_side, Chassis_IMU_data);

    static int recover_flag = 0;
    //这里是轮电机一个数据位，随便找的，这个数据在PC模式，底盘断电的时候置0了，用于同步被裁判系统断掉底盘电在启动后轮电机和关机电机的同步启动
    if (LKMotor.LKMotor1_Receive.Cmd_Byte == 0)
    {
    Joint_Motor.YSMotor_Send[LF-1].T= 0;
    Joint_Motor.YSMotor_Send[LB-1].T= 0;
    Joint_Motor.YSMotor_Send[RF-1].T= 0;
    Joint_Motor.YSMotor_Send[RB-1].T= 0;
    chassis.dist = chassis.target_dist = 0;
    recover_flag = 1;
    Robot_RunState.L_leg_len = Robot_RunState.R_leg_len = 0.15;
    return;
    }

    // else if(recover_flag == 1) //在裁判系统切底盘电的时候，延缓一定时间在启动 //一个保险，但是实现的太shi了
    // {
    // static int recover_count = 0; 
    // recover_count ++;
    // if(recover_flag <500) return; //通电小于500ms就不执行
    // else { recover_flag = 0;
    //     recover_count = 0;
    //     }
    // }
    
    if (Joint_Motor.stop_flag == MOTOR_STOP)//关节电机关闭就退出
    return;
    if (IMUC_ready == false) //底盘陀螺仪数据没准备好就退出
    return;

   WattLimitSet();
}


void BalanceInit(void) /* 底盘数据初始化函数 */
{
    // 目标速度置0
    chassis.target_v = 0;
    // 清空腿长和距离
    l_side.target_len = r_side.target_len = 0.15;
    Robot_RunState.L_leg_len = Robot_RunState.R_leg_len = 0.15;
    l_side.leg_len_limitMax = r_side.leg_len_limitMax = 0.3;
    l_side.leg_len_limitMin = r_side.leg_len_limitMin = 0.1;
    // roll_compensate_pid.Ref = 0;
    chassis.dist = chassis.target_dist = 0;
    chassis.jump_state = false;
}


static void EnableAllMotor(void) /* 打开所有电机 */
{
    LKMotor.stop_flag = MOTOR_ENALBED; 
    Joint_Motor.stop_flag = MOTOR_ENALBED;
    if(DM_Motor.DM_G6220_Receive[Pitch_DM6220].ERR_State == DM_Motor_Disable)  //如果6220没有使能，就使能
    Motor_enable();

}

static void StopAllMotor(void) /* 关闭所有电机 */
{
    LKMotor.stop_flag = MOTOR_STOP; 
    Joint_Motor.stop_flag = MOTOR_STOP;
}

static void WattLimitSet(void) /* 设定运动模态的输出 */
{

    Joint_Motor.YSMotor_Send[RF-1].K_W =0.0f;
    Joint_Motor.YSMotor_Send[RB-1].K_W =0.0f;
    Joint_Motor.YSMotor_Send[LF-1].K_W =0.0f;
    Joint_Motor.YSMotor_Send[LB-1].K_W =0.0f;

    Joint_Motor.YSMotor_Send[LF-1].T= l_side.T_front/6.33f;
    Joint_Motor.YSMotor_Send[LB-1].T= l_side.T_back/6.33f;
    Joint_Motor.YSMotor_Send[RF-1].T= -r_side.T_front/6.33f;
    Joint_Motor.YSMotor_Send[RB-1].T= -r_side.T_back/6.33f;

    // VAL_LIMIT(Joint_Motor.YSMotor_Send[LF-1].T, -15, 15);
    // VAL_LIMIT(Joint_Motor.YSMotor_Send[LB-1].T, -15, 15);
    // VAL_LIMIT(Joint_Motor.YSMotor_Send[RF-1].T, -15, 15);
    // VAL_LIMIT(Joint_Motor.YSMotor_Send[RB-1].T, -15, 15);

    LKMotor.Motor_Out[LD -1] = 195.3125f * l_side.T_wheel;
    LKMotor.Motor_Out[RD -1] = 195.3125f * -r_side.T_wheel;
    
}


// void Info_Change(void)
// {


// }

static void WokingStateSet(void)
{
    // static float last_target_v = 0.0;
    // chassis.target_v   =  Robot_RunState.Advance;//RC_Rx.CH3*0.003;
    // if (fabs((chassis.target_v - last_target_v) / del_t) > MAX_ACC_REF)
    // chassis.target_v = last_target_v + MAX_ACC_REF * del_t * (chassis.target_v >= last_target_v ? 1 : -1);
    // last_target_v = chassis.target_v;

    //期望速度输入
    chassis.target_v   =  Robot_RunState.Advance;
    //加速度限制方案，不太好用。 @todo 可以尝试优化
    // if(Robot_RunState.Advance)
    // chassis.target_v -= sign(chassis.vel - Robot_RunState.Advance) * 4.0f * del_t;
    // else chassis.target_v   =  Robot_RunState.Advance; 

    l_side.target_len = Robot_RunState.L_leg_len;
    r_side.target_len = Robot_RunState.R_leg_len;

    if( Robot_RunState.Robot_stop )
    StopAllMotor();
    else
    EnableAllMotor(); //打开所有底盘电机

    if(Robot_RunState.Robot_Reset) LKMotor.stop_flag = MOTOR_STOP; 

    //这一句是为了在裁判系统对底盘掐电的时候对关闭关节电机
    if(Robot_RunState.joint_disenable) Joint_Motor.stop_flag = MOTOR_STOP;
    // else  Joint_Motor.stop_flag = MOTOR_ENALBED;

    // if(RC_Rx.S1 == 1 &&  chassis.jump_over == false && chassis.jump_state == false)
    // {
    // chassis.jump_state = true;
    // }
    // else if(RC_Rx.S1 != 1){
    // chassis.jump_state = false;
    // chassis.jump_over = false;}

    VAL_LIMIT(Robot_RunState.L_leg_len, l_side.leg_len_limitMin, l_side.leg_len_limitMax);
    VAL_LIMIT(Robot_RunState.R_leg_len, r_side.leg_len_limitMin, r_side.leg_len_limitMax);
    VAL_LIMIT(l_side.target_len, l_side.leg_len_limitMin, l_side.leg_len_limitMax);
    VAL_LIMIT(r_side.target_len, r_side.leg_len_limitMin, r_side.leg_len_limitMax);
}

/**
 * @brief   对YawHope值进行限制
 * @param   YAWHope   指向YawHope值的指针
 * @param   Err       指向误差值的指针
 * @param   Max       最大限制值
 * @param   Min       最小限制值
 * @retval  无
 * @attention 
 */
void YAWHope_Value_limiting(float *YAWHope, float *Err, float Max, float Min)
{
if(*Err>((abs(Max)+abs(Min))/2)) *YAWHope-= abs(Max)+abs(Min);
else if(*Err<-((abs(Max)+abs(Min))/2)) *YAWHope+= abs(Max)+abs(Min);
}

/**
 * @brief   对Hope值进行限制
 * @param   Hope      指向Hope值的指针
 * @param   Real      指向实际值的指针
 * @param   K         比较系数 ，
 * @retval  无
 * @attention 
 */
// void Hope_Value_limiting(float *Hope, float *Real, float K)
// {
// 	if(*Hope-*Real>PI)
// 	*Hope-=2*PI;
// 	else if(*Hope-*Real< -PI)
// 	*Hope+=2*PI;
// }

static void SynthesizeMotion(void) /* 腿部控制:抗劈叉; 轮子控制:转向 */
{
    //底盘陀螺仪YAW角度闭环
    // PIDCalculate(&steer_p_pid, chassis.yaw, chassis.target_yaw);
    // YAWHope_Value_limiting(&chassis.target_yaw, &steer_p_pid.Err, PI, -PI);
    // float p_ref = PIDCalculate(&steer_p_pid, chassis.yaw, chassis.target_yaw);
    // PIDCalculate(&steer_v_pid, chassis.wz, p_ref);

    PIDCalculate(&steer_p_pid, DJI_Motor.DJI_G6020_Receive[0].angle, chassis_init_anlge);
    YAWHope_Value_limiting(&chassis.target_yaw, &steer_p_pid.Err, 8191, 0);
    PIDCalculate(&steer_p_pid, DJI_Motor.DJI_G6020_Receive[0].angle, 4088 + 2048);
    static float hope_steer_v , steer_acc; 
    static uint8_t Robot_spin_flag = 0;
    if(Robot_RunState.Robot_spin){
    if(Robot_RunState.Robot_spin_high)hope_steer_v = 10.0f;
    else hope_steer_v = 8.0f;
    steer_acc = 10.0f;  //启动加速度
    steer_v_pid.Ref -= sign(chassis.wz - hope_steer_v) * steer_acc * del_t;
    Robot_spin_flag = 1;
    } 
    else {
    if(Robot_spin_flag == 1) {
    hope_steer_v = 0.0f;
    steer_v_pid.Ref -= sign(chassis.wz - hope_steer_v) * steer_acc * del_t;
    if(steer_v_pid.Ref < 1) Robot_spin_flag = 0;
    }
    if(Robot_spin_flag == 0){
    if(Robot_RunState.chassis_follow) 
    steer_v_pid.Ref =  steer_p_pid.Output; //底盘跟随
    else steer_v_pid.Ref = 0;
    }

    }
    // steer_v_pid.Ref -= sign(chassis.wz - hope_steer_v) * steer_acc * del_t; 

    PIDCalculate(&steer_v_pid, chassis.wz, steer_v_pid.Ref);
    //叠加到轮输出力矩
    l_side.T_wheel -= steer_v_pid.Output;
    r_side.T_wheel += steer_v_pid.Output;

    // 抗劈叉
    static float swerving_speed_ff, ff_coef = 1;
    swerving_speed_ff = ff_coef * steer_v_pid.Output; // 用于抗劈叉的前馈
    PIDCalculate(&anti_crash_pid, l_side.phi5 - r_side.phi5, 0);
    l_side.T_hip -= anti_crash_pid.Output - swerving_speed_ff;
    r_side.T_hip += anti_crash_pid.Output - swerving_speed_ff;

}

/**
 * @brief 将电机和imu的数据组装为LinkNPodParam结构体和chassisParam结构体
 *
 * 
 * @note GO1电机逆时针旋转为正; LK9025电机逆时针旋转为正,此处皆需要转换为模型中给定的正方向
 *
 */
static void ParamAssemble(void)
{ 
    // 机体参数,视为平面刚体
    chassis.pitch = Chassis_IMU_data.Pitch;
    chassis.pitch_w = Chassis_IMU_data.Gyro[0]; 
    chassis.yaw = Chassis_IMU_data.Yaw;
    chassis.wz = Chassis_IMU_data.Gyro[2];
    chassis.roll = Chassis_IMU_data.Roll;
    chassis.roll_w = Chassis_IMU_data.Gyro[1];

    // GO1电机的角度是逆时针为正,LK9025电机的角度是逆时针为正
    l_side.phi1 = PI + LIMIT_LINK_RAD + Joint_Motor.YSMotor_Receive[LB -1].Amend_Pos;
    l_side.phi1_w = Joint_Motor.YSMotor_Receive[LB -1].W_output;
    l_side.phi4 = Joint_Motor.YSMotor_Receive[LF -1].Amend_Pos - LIMIT_LINK_RAD;
    l_side.phi4_w = Joint_Motor.YSMotor_Receive[LF -1].W_output;
    l_side.w_ecd = LKMotor.LKMotor1_Receive.speed_rads;

    r_side.phi1 = PI + LIMIT_LINK_RAD - Joint_Motor.YSMotor_Receive[RB -1].Amend_Pos;
    r_side.phi1_w = -Joint_Motor.YSMotor_Receive[RB -1].W_output;
    r_side.phi4 = -Joint_Motor.YSMotor_Receive[RF -1].Amend_Pos - LIMIT_LINK_RAD;
    r_side.phi4_w = -Joint_Motor.YSMotor_Receive[RF -1].W_output;
    r_side.w_ecd = -LKMotor.LKMotor2_Receive.speed_rads;

}

static void LegControl(void) /* 腿长控制和Roll补偿 */
{
    
    PIDCalculate(&roll_compensate_pid, chassis.roll, Robot_RunState.roll_ref);
    l_side.target_len += roll_compensate_pid.Output;
    r_side.target_len -= roll_compensate_pid.Output;

    static float gravity_ff = 75;//60;
    static float roll_extra_comp_p = 400;
    float roll_comp = roll_extra_comp_p * (chassis.roll -Robot_RunState.roll_ref);
    
    if(BalanceTask_cnt% 10 == 0){ //限制控制频率
	l_side.F_leg = PIDCalculate(&leglen_pid_l, l_side.height, l_side.target_len) + gravity_ff - roll_comp;
	r_side.F_leg = PIDCalculate(&leglen_pid_r, r_side.height, r_side.target_len) + gravity_ff + roll_comp;
    }

}



float jump_start_time, jump_now_time;
const float k_jump_force = 220.0f;
const float k_jump_time = 0.2f;
const float k_retract_force = -120.0f;
const float k_retract_time = 0.1f;

static void JumpTASK(void) /* 跳跃 */
{
    if (l_side.height > 0.14f && r_side.height > 0.14f && chassis.last_jump_state == false) {
    l_side.target_len = 0.1f;
    r_side.target_len = 0.1f;
    LegControl();
    chassis.last_jump_state = false;
    return;
    }

    else {
    if (chassis.jump_state == true && chassis.last_jump_state == false) {
      jump_start_time = HAL_GetTick() / 1000.0f;
    }

    jump_now_time = HAL_GetTick() / 1000.0f;

    if (fabs(jump_now_time - jump_start_time) <= k_jump_time) {
      l_side.F_leg = k_jump_force;
      r_side.F_leg = k_jump_force;
    }

    jump_now_time = HAL_GetTick() / 1000.0f;

    if ((jump_now_time - jump_start_time - k_jump_time) <= k_retract_time &&
        (jump_now_time - jump_start_time) > k_jump_time) {
      l_side.F_leg = k_retract_force;
      r_side.F_leg = k_retract_force;
    }
    if ((jump_now_time - jump_start_time - k_jump_time) > k_retract_time) {
      chassis.jump_state = false;
      chassis.jump_over = true;
    }
    chassis.last_jump_state = chassis.jump_state;
  }

}
