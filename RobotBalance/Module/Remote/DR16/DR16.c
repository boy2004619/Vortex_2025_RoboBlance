#include "DR16.h"

RCData_t RC_Rx;
RCData_t PC_Rx;
//时间域(Time Domain)上的遥控数据表示
RCData_t RCData_TD[2] ={0};

Robo_runstate Robot_RunState;

void DR16_Rx(uint8_t *pData)
{
	uint16_t LvBoBuf[4];
	//初步滤波
	LvBoBuf[0] = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
	LvBoBuf[1] = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	LvBoBuf[2] = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |((int16_t)pData[4] << 10)) & 0x07FF;
	LvBoBuf[3] = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	
	//防止超出范围，超出范围的数据包丢弃
	if((LvBoBuf[0] <= RC_CH_VALUE_MAX	&& LvBoBuf[0] >= RC_CH_VALUE_MIN) &&	\
		 (LvBoBuf[1] <= RC_CH_VALUE_MAX	&& LvBoBuf[1] >= RC_CH_VALUE_MIN) &&	\
		 (LvBoBuf[2] <= RC_CH_VALUE_MAX	&& LvBoBuf[2] >= RC_CH_VALUE_MIN) &&	\
		 (LvBoBuf[3] <= RC_CH_VALUE_MAX	&& LvBoBuf[3] >= RC_CH_VALUE_MIN))
	{
		RC_Rx.CH0 = (int16_t)LvBoBuf[0];
		RC_Rx.CH1 = (int16_t)LvBoBuf[1];
		RC_Rx.CH2 = (int16_t)LvBoBuf[2];
		RC_Rx.CH3 = (int16_t)LvBoBuf[3];
		
		RC_Rx.S1 = ((pData[5] >> 4) & 0x000C) >> 2;
		RC_Rx.S2 = ((pData[5] >> 4) & 0x0003);
		RC_Rx.X  = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
		RC_Rx.Y  = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
		RC_Rx.Z  = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); 

		RC_Rx.PRESS_L = pData[12];
		RC_Rx.PRESS_R = pData[13];

		RC_Rx.kb.key_code  = pData[14] | (pData[15] << 8);
		RC_Rx.CH4 = pData[16] | (pData[17] << 8);
		
		//数据转化
		RC_Rx.CH0 -= RC_CH_VALUE_OFFSET;
		RC_Rx.CH1 -= RC_CH_VALUE_OFFSET;
		RC_Rx.CH2 -= RC_CH_VALUE_OFFSET;
		RC_Rx.CH3 -= RC_CH_VALUE_OFFSET;
		RC_Rx.CH4 -= RC_CH_VALUE_OFFSET;
		
		//防止静止数据波动
		if(RC_Rx.CH0 <= 5 && RC_Rx.CH0 >= -5) RC_Rx.CH0=0;
		if(RC_Rx.CH1 <= 5 && RC_Rx.CH1 >= -5) RC_Rx.CH1=0;
		if(RC_Rx.CH2 <= 5 && RC_Rx.CH2 >= -5) RC_Rx.CH2=0;
		if(RC_Rx.CH3 <= 5 && RC_Rx.CH3 >= -5) RC_Rx.CH3=0;
	}

}


//机器人状态位初始化
void RoboState_Init(void)
{
referee_info.GameRobotState.power_management_chassis_output = 1; //将裁判系统的底盘的初始化状态置为开
}


uint32_t Remote_cnt;
uint32_t Remote_dwt_cnt;
float Remote_del_t;
//根据遥控器或PC操控的逻辑改变机器人状态
void RoboState_Change(RCData_t *RC_Data ,RCData_t *PC_Data, Robo_runstate *Robot_RunData)
{
	Remote_del_t = DWT_GetDeltaT(&Remote_dwt_cnt);
	Robot_RunData->PC_or_RC = (remote_mode)(RC_Data->S2 ==2 && RC_Data->S1 == 1);
	Remote_cnt++;
	//***********遥控控制逻辑***********//

	if(Robot_RunData->PC_or_RC == RC)
	{
	Robot_RunData->Advance = RC_Data->CH3*0.003;
	Robot_RunData->Yaw = RC_Data->CH0*0.005* Remote_del_t;
    Robot_RunData->Pitch = RC_Data->CH1*0.001* Remote_del_t;

	Robot_RunData->L_leg_len += -RC_Data->CH2*0.0003 * Remote_del_t; //*Remote_del_t是为了保证操作手感不受该任务的执行频率的影响
	Robot_RunData->R_leg_len += -RC_Data->CH2*0.0003 * Remote_del_t;

	Robot_RunData->Robot_spin = (RC_Data->S1 == 2);
	Robot_RunData->fire =		(RC_Data->S2 == 1);
	Robot_RunData->Robot_friction = Robot_RunData->fire; //遥控器模式下摩擦轮启动键和开火键绑定
	Robot_RunData->Robot_stop = (RC_Data->S2 == 2) ;//|| (referee_info.GameRobotState.power_management_chassis_output == 0);
	Robot_RunData->Robot_Reset= (RC_Data->S2 == 2 && RC_Data->S1 == 2);
	Robot_RunData->roll_ref = 0;
	// if(referee_info.PowerHeatData.shooter_17mm_1_barrel_heat < 20 && referee_info.GameState.game_progress !=0) Robot_RunData->fire = 0; //即将超热，钳制发射
	// Robot_RunData->joint_disenable = Robot_RunData->Robot_stop;//遥控器模式下关节电机不允许单独使能

	if(referee_info.GameRobotState.power_management_chassis_output == 0)
	{
		if(Robot_RunData->Robot_spin == 1){
		Robot_RunData->Robot_spin = 0; //关闭
		RCData_TD[TEMP].key_count[KEY_PRESS][Key_E]++;
		}
		LKMotor.LKMotor1_Receive.Cmd_Byte =0;
	}
	Robot_RunData->joint_disenable = (referee_info.GameRobotState.power_management_chassis_output == 0) || (Robot_RunData->Robot_stop);

	Robot_RunData->chassis_follow = 1; //遥控器模式下默认跟随
	Robot_RunData->auto_aim = (RC_Data->S1 == 1) && (RC_Data->S2 != 2);
	}



	//***********键鼠控制逻辑***********//

	if(Robot_RunData->PC_or_RC == PC)
	{
	if(RCData_TD[TEMP].kb.bit.W || RCData_TD[TEMP].kb.bit.S){
		if(RCData_TD[TEMP].kb.bit.SHIFT)
		Robot_RunData->Advance = (RCData_TD[TEMP].kb.bit.W ? 1 : -1) * 1.7f ;//1.5;
		else
		Robot_RunData->Advance = (RCData_TD[TEMP].kb.bit.W ? 1 : -1) * 1.2f;
	}
	else {Robot_RunData->Advance = 0;}


	if(RCData_TD[TEMP].kb.bit.A || RCData_TD[TEMP].kb.bit.D){
		Robot_RunData->roll_ref = (RCData_TD[TEMP].kb.bit.A ? -1 : 1) * 0.2f;

	}
	else {Robot_RunData->roll_ref = 0;}


	//一坨shi ，不过能用
	// static uint8_t leg_flag = 0;
	// if(RCData_TD[TEMP].kb.bit.A ) 
	// {
	// Robot_RunData->L_leg_len = 0.15;
	// Robot_RunData->R_leg_len = 0.35;
	// leg_flag = 1;
	// } else if(RCData_TD[TEMP].kb.bit.D )
	// {
	// Robot_RunData->L_leg_len = 0.35;
	// Robot_RunData->R_leg_len = 0.15;
	// leg_flag = 1;
	// }else if(leg_flag == 1 && !RCData_TD[TEMP].kb.bit.A && !RCData_TD[TEMP].kb.bit.D){
	// Robot_RunData->L_leg_len = 0.15;
	// Robot_RunData->R_leg_len = 0.15;
	// leg_flag = 0;
	// }


	Robot_RunData->Yaw = PC_Data->X *0.00002;
    Robot_RunData->Pitch = -PC_Data->Y *0.00001;

	Robot_RunData->L_leg_len += -PC_Data->Z*0.00008;
	Robot_RunData->R_leg_len += -PC_Data->Z*0.00008;

	Robot_RunData->fire =		PC_Data->PRESS_L;
	Robot_RunData->auto_aim =	PC_Data->PRESS_R;

	Robot_RunData->game_mode = (referee_info.GameState.game_progress !=0 ); //获取比赛状态
	

	if(referee_info.GameRobotState.power_management_chassis_output == 0)
	{
		if(Robot_RunData->Robot_spin == 1){
		Robot_RunData->Robot_spin = 0; //关闭
		RCData_TD[TEMP].key_count[KEY_PRESS][Key_E]++;
		}
		
		LKMotor.LKMotor1_Receive.Cmd_Byte =0;
	}

	Robot_RunData->joint_disenable = (referee_info.GameRobotState.power_management_chassis_output == 0);// && (referee_info.GameState.game_progress !=0); 
	Robot_RunData->Robot_stop = 0;
	// if(referee_info.PowerHeatData.shooter_17mm_1_barrel_heat >referee_info.PowerHeatData.buffer_energy * 0.8f ) Robot_RunData->fire = 0; //即将超热，钳制发射

	
	switch (RCData_TD[TEMP].key_count[KEY_PRESS][Key_F] % 2) // 	F	摩擦轮
    {
    case 0:
	if(!Robot_RunData->fire) //开火没触发
		Robot_RunData->Robot_friction = 0;
        break;
    default:
		Robot_RunData->Robot_friction = 1;
        break;
    }
	if(Robot_RunData->fire && Robot_RunData->Robot_friction == 0) // 开火标志位启用时，必须打开摩擦轮
	{
		Robot_RunData->Robot_friction = 1; 
		RCData_TD[TEMP].key_count[KEY_PRESS][Key_F]++;
	}

	switch (RCData_TD[TEMP].key_count[KEY_PRESS][Key_E] % 2) // 	E	小陀螺
    {
    case 0:
		Robot_RunData->Robot_spin = 0;
		Robot_RunData->Robot_spin_high = 0;
        break;
    default:
		Robot_RunData->Robot_spin = 1;
		if(RCData_TD[TEMP].kb.bit.SHIFT) Robot_RunData->Robot_spin_high = 1;
		else Robot_RunData->Robot_spin_high = 0;

		if(Robot_RunData->Advance != 0 ) //有速度输入时解除小陀螺状态
		{
		Robot_RunData->Robot_spin = 0;  
		RCData_TD[TEMP].key_count[KEY_PRESS][Key_E] ++;
		}
		 
        break;
    }

	// 扣血过快主动打开小陀螺
	if(Remote_cnt % 1000 ==0)
	{
	static float last_HP ;
	if( referee_info.RobotHurt.hurt_type == 0) //在比赛状态下 扣血原因是弹丸打击造成
	{
		if((referee_info.GameRobotState.current_HP - last_HP) <= -15) //扣血太快
		if(Robot_RunData->Robot_spin == 0)  //如果这个时候没打开小陀螺
		{
			Robot_RunData->Robot_spin = 1;
			RCData_TD[TEMP].key_count[KEY_PRESS][Key_E] ++;
		}
	}
	last_HP = referee_info.GameRobotState.current_HP;
	}

	
	switch (RCData_TD[TEMP].key_count[KEY_PRESS][Key_C] % 2) // 	C	底盘跟随
    {
    case 0:
		Robot_RunData->chassis_follow = 1;  //默认跟随
        break;
    default:
		Robot_RunData->chassis_follow = 0;
		if(Robot_RunData->Advance != 0 ) //有速度输入时解除底盘脱离状态
		{
		Robot_RunData->chassis_follow = 1;  
		RCData_TD[TEMP].key_count[KEY_PRESS][Key_C] ++;
		} 
        break;
    }
	
	switch (RCData_TD[TEMP].key_count[KEY_PRESS_WITH_CTRL][Key_R] % 2) // 	CTRL+R	复位
    {
    case 0:
		Robot_RunData->Robot_Reset = 0;
        break;
    default:
		Robot_RunData->Robot_Reset = 1;
        break;
    }

	Robot_RunData->UI_update = RCData_TD[TEMP].kb.bit.B;

	switch (RCData_TD[TEMP].key_count[KEY_PRESS][Key_G] % 2) // 开弹舱
    {
    case 0:
		Robot_RunData->open_mag = 0;
        break;
    default:
		Robot_RunData->open_mag = 1;
        break;
    }

	Robot_RunData->F_reversal = RCData_TD[TEMP].kb.bit.CTRL&RCData_TD[TEMP].kb.bit.Z;   // CTRL+Z 反转摩擦轮和拨弹
	}
}

