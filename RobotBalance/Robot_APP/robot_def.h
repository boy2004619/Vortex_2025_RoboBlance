/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */

/**
 * 这个是跃鹿的文件，我们实质上是没用到的 ，后续删除，或替换成自己的实现
 */

#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

// // #include "ins_task.h"
// // #include "master_process.h"
// #include "main.h"

// /* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
// // #define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD //底盘板
// // #define GIMBAL_BOARD  //云台板

// #define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// // #define VISION_USE_UART // 使用串口发送视觉数据

// /* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// // 云台参数
// #define YAW_CHASSIS_ALIGN_ECD 2711  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
// #define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
// #define PITCH_HORIZON_ECD 3412      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
// #define PITCH_MAX_ANGLE 0           // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// #define PITCH_MIN_ANGLE 0           // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

// // 发射参数
// #define ONE_BULLET_DELTA_ANGLE 36    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
// #define REDUCTION_RATIO_LOADER 49.0f // 拨盘电机的减速比,英雄需要修改为3508的19.0f
// #define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量


// #define GYRO2GIMBAL_DIR_YAW 1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
// #define GYRO2GIMBAL_DIR_PITCH 1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
// #define GYRO2GIMBAL_DIR_ROLL 1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

// // 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
// #if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
//     (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
//     (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
// #error Conflict board definition! You can only define one board type.
// #endif

// #pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
// /* -------------------------基本控制模式和数据类型定义-------------------------*/
// /**
//  * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
//  *
//  */
// // 机器人状态
// typedef enum
// {
//     ROBOT_STOP = 0,
//     ROBOT_READY,
// } Robot_Status_e;

// // 应用状态
// typedef enum
// {
//     APP_OFFLINE = 0,
//     APP_ONLINE,
//     APP_ERROR,
// } App_Status_e;

// // UI模式设置
// typedef enum
// {
//     UI_KEEP = 0,
//     UI_REFRESH,
// } ui_mode_e;

// // 底盘模式设置
// /**
//  * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
//  *
//  */
// typedef enum
// {
//     CHASSIS_ZERO_FORCE = 0,    // 电流零输入
//     CHASSIS_ROTATE,            // 小陀螺模式
//     CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
//     CHASSIS_RESET,             // 底盘重置,双腿缩回
//     CHASSIS_FREE_DEBUG,        // 底盘单独调试模式
// } chassis_mode_e;

// // 云台模式设置
// typedef enum
// {
//     GIMBAL_ZERO_FORCE = 0, // 电流零输入
//     GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
//     GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
// } gimbal_mode_e;

// // 发射模式设置
// typedef enum
// {
//     SHOOT_OFF = 0,
//     SHOOT_ON,
// } shoot_mode_e;
// typedef enum
// {
//     FRICTION_OFF = 0, // 摩擦轮关闭
//     FRICTION_ON,      // 摩擦轮开启
// } friction_mode_e;

// typedef enum
// {
//     LID_OPEN = 0, // 弹舱盖打开
//     LID_CLOSE,    // 弹舱盖关闭
// } lid_mode_e;

// typedef enum
// {
//     LOAD_STOP = 0,  // 停止发射
//     LOAD_REVERSE,   // 反转
//     LOAD_1_BULLET,  // 单发
//     LOAD_3_BULLET,  // 三发
//     LOAD_BURSTFIRE, // 连发
// } loader_mode_e;

// // 功率限制,从裁判系统获取,是否有必要保留?
// typedef struct
// { // 功率控制
//     float chassis_power_mx;
// } Chassis_Power_Data_s;

// typedef enum
// {
//     CAHSSIS_ALIGN = 0,
//     CHASSIS_SIDLE
// } chassis_direction_e;

// /* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
// /**
//  * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
//  *
//  */
// // cmd发布的底盘控制数据,由chassis订阅
// typedef struct
// {
//     // 控制部分
//     float vx;           // 前进方向速度
//     float delta_leglen; // 腿长
//     float offset_angle; // 底盘和归中位置的夹角
//     chassis_mode_e chassis_mode;
//     chassis_direction_e direction;
//     uint8_t ui_refresh_flag;
    
// } Chassis_Ctrl_Cmd_s;



#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H
