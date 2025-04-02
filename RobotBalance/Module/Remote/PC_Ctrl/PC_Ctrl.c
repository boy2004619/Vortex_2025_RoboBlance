#include "PC_ctrl.h"

remote_control_t Imagelink_Data;

void PC_data_shift(RCData_t *RC_Data , remote_control_t *PC_Data)
{
    RC_Data->X = PC_Data->mouse_x;
    RC_Data->Y = PC_Data->mouse_y;
    RC_Data->Z = PC_Data->mouse_z;
    RC_Data->PRESS_L = PC_Data->left_button_down;
    RC_Data->PRESS_R = PC_Data->right_button_down;
    RC_Data->kb.key_code = PC_Data->keyboard_value;

	RCData_TD[TEMP].X = PC_Data->mouse_x;
    RCData_TD[TEMP].Y = PC_Data->mouse_y;
    RCData_TD[TEMP].Z = PC_Data->mouse_z;
    RCData_TD[TEMP].PRESS_L = PC_Data->left_button_down;
    RCData_TD[TEMP].PRESS_R = PC_Data->right_button_down;
    RCData_TD[TEMP].kb.key_code = PC_Data->keyboard_value;

	// memcpy(&RCData_TD[TEMP],RC_Data,sizeof(RCData_t));

	RCData_TD[TEMP].key[KEY_PRESS].key_code = RC_Data->kb.key_code;

	//  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
    if (RCData_TD[TEMP].key[KEY_PRESS].bit.CTRL) // ctrl键按下
        RCData_TD[TEMP].key[KEY_PRESS_WITH_CTRL] = RCData_TD[TEMP].key[KEY_PRESS];
    else
        memset(&RCData_TD[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (RCData_TD[TEMP].key[KEY_PRESS].bit.SHIFT) // shift键按下
        RCData_TD[TEMP].key[KEY_PRESS_WITH_SHIFT] = RCData_TD[TEMP].key[KEY_PRESS];
    else
        memset(&RCData_TD[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

	
	uint16_t key_now = RCData_TD[TEMP].key[KEY_PRESS].key_code,     // 当前按键是否按下
    key_last = RCData_TD[LAST].key[KEY_PRESS].key_code,             // 上一次按键是否按下
    key_with_ctrl = RCData_TD[TEMP].key[KEY_PRESS_WITH_CTRL].key_code,        // 当前ctrl组合键是否按下
    key_with_shift = RCData_TD[TEMP].key[KEY_PRESS_WITH_SHIFT].key_code,       //  当前shift组合键是否按下
    key_last_with_ctrl = RCData_TD[LAST].key[KEY_PRESS_WITH_CTRL].key_code,   // 上一次ctrl组合键是否按下
    key_last_with_shift = RCData_TD[LAST].key[KEY_PRESS_WITH_SHIFT].key_code;  // 上一次shift组合键是否按下

	for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        //如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            RCData_TD[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            RCData_TD[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            RCData_TD[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }
	memcpy(&RCData_TD[LAST], &RCData_TD[TEMP], sizeof(RCData_t)); // 保存上一次的数据,用于按键持续按下和切换的判断

}

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @retval 是否对正误判断做处理
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 */
void imagelinkReadData(uint8_t *buff)
{
    // uint16_t length = sizeof (remote_control_t);
	// uint16_t judge_length; // 统计一帧数据长度
	if (buff == NULL)	   // 空数据包，则不作任何处理
		return;

	// 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
	memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);
	// 判断帧头数据(0)是否为0xA5
	if (buff[SOF] == REFEREE_SOF)
	{
		// 帧头CRC8校验
		// if (Verify_CRC8_Check_Sum(buff, LEN_HEADER, 0xff) == TRUE)
		// {
		referee_info.CmdID = (buff[6] << 8 | buff[5]);
		// 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
		// 第8个字节开始才是数据 data=7
			switch (referee_info.CmdID)
			{
			case imagelink_ctrl_data:
			memcpy(&Imagelink_Data, (buff + DATA_Offset), LEN_imagelink_data);
      		PC_data_shift(&PC_Rx,&Imagelink_Data);
			break;
			}
		// }
	}
}




