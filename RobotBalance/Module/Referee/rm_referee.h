#ifndef RM_REFEREE_H
#define RM_REFEREE_H

#include "main.h"
#include "referee_protocol.h"


extern uint8_t UI_Seq;

#pragma pack(1)
typedef struct
{
	uint8_t  Robot_Color;		// 机器人颜色
	uint16_t Robot_ID;			// 本机器人ID
	uint16_t Cilent_ID;			// 本机器人对应的客户端ID
	uint16_t Receiver_Robot_ID; // 机器人车间通信时接收者的ID，必须和本机器人同颜色
} referee_id_t;

// 此结构体包含裁判系统接收数据以及UI绘制与机器人车间通信的相关信息
typedef struct
{
	referee_id_t referee_id;

	xFrameHeader FrameHeader; // 接收到的帧头信息
	uint16_t CmdID;
	ext_game_state_t GameState;							   // 0x0001
	ext_game_result_t GameResult;						   // 0x0002
	ext_game_robot_HP_t GameRobotHP;					   // 0x0003
	ext_event_data_t EventData;							   // 0x0101
	ext_supply_projectile_action_t SupplyProjectileAction; // 0x0102
	ext_game_robot_state_t GameRobotState;				   // 0x0201
	ext_power_heat_data_t PowerHeatData;				   // 0x0202
	ext_game_robot_pos_t GameRobotPos;					   // 0x0203
	ext_buff_musk_t BuffMusk;							   // 0x0204
	aerial_robot_energy_t AerialRobotEnergy;			   // 0x0205
	ext_robot_hurt_t RobotHurt;							   // 0x0206
	ext_shoot_data_t ShootData;							   // 0x0207

	// 自定义交互数据的接收
	Communicate_ReceiveData_t ReceiveData;

	uint8_t init_flag;

} referee_info_t;

// 模式是否切换标志位，0为未切换，1为切换，static定义默认为0
typedef struct
{
	uint32_t chassis_flag : 1;
	uint32_t gimbal_flag : 1;
	uint32_t shoot_flag : 1;
	uint32_t lid_flag : 1;
	uint32_t friction_flag : 1;
	uint32_t Power_flag : 1;
	uint32_t vision_flag : 1;
	uint32_t loader_flag : 1;
	uint32_t direction_flag : 1;
	uint32_t RobotReset_flag : 1;
	uint32_t openmag_flag : 1;
} Referee_Interactive_Flag_t;

// 此结构体包含UI绘制与机器人车间通信的需要的其他非裁判系统数据
typedef struct
{
	Referee_Interactive_Flag_t Referee_Interactive_Flag;
	// 为UI绘制以及交互数据所用
	// chassis_mode_e chassis_mode;			 // 底盘模式
	// gimbal_mode_e gimbal_mode;				 // 云台模式
	// shoot_mode_e shoot_mode;				 // 发射模式设置
	// friction_mode_e friction_mode;			 // 摩擦轮关闭
	// lid_mode_e lid_mode;					 // 弹舱盖打开
	// Chassis_Power_Data_s Chassis_Power_Data; // 功率控制
	// ui_mode_e ui_mode;                       // UI状态
	// vision_mode_e vision_mode;			 // 视觉状态
	// loader_mode_e loader_mode;				 // 拨盘模式
	// chassis_direction_e direction;			 // 对齐状态

	bool open_mag;						 //弹舱状态
	bool Robot_spin;					 //小陀螺状态
	bool aim_tracking;					 //视觉识别到目标
	bool Robot_Reset;					 //机器人复位
	float coord[6];					     // 五连杆
	float chassis_gimbal_error;			 //底盘和云台的方向差值，用于指示小陀螺动态UI和底盘方向状态

	// 上一次的模式，用于flag判断
	bool Robot_Reset_last; 
	bool open_mag_last;			

} Referee_Interactive_info_t;

#pragma pack()


extern referee_info_t referee_info;

void JudgeReadData(uint8_t *buff);
void RefereeSend(uint8_t *send, uint16_t tx_len);

#endif // !REFEREE_H
