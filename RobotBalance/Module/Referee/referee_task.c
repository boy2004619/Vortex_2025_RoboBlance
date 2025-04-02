/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"
#include "balance.h"

static Referee_Interactive_info_t UI_data; // UI绘制需要的机器人状态数据 ,这里的数据转移需要手动适配
// referee_info_t referee_recv_info;   // 接收到的裁判系统数据
uint8_t UI_Seq;                     // 包序号，供整个referee文件使用

/**
 * @brief  ui数据填装
 * @param  none
 * @retval none
 * @attention none
 */

static void UI_Data_paking(Referee_Interactive_info_t *UI_DATA)
{
    if (UI_DATA == NULL) {
    // 处理空指针情况，例如记录日志或返回错误码
    return;
    }
    extern LinkNPodParam l_side;
    extern PIDInstance steer_p_pid;
    memcpy(&UI_DATA->coord,&l_side.coord,sizeof(l_side.coord)); 
    UI_DATA->chassis_gimbal_error= steer_p_pid.Err /22.752f;
    UI_DATA->aim_tracking = Vision_Receive_Data.tracking;
    UI_DATA->Robot_spin = Robot_RunState.Robot_spin;

}

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  none
 * @retval none
 * @attention
 */

static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_info.referee_id.Robot_Color = referee_info.GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_info.referee_id.Robot_ID = referee_info.GameRobotState.robot_id;
    referee_info.referee_id.Cilent_ID = 0x0100 + (uint16_t)(referee_info.referee_id.Robot_ID); // 计算客户端ID
    referee_info.referee_id.Receiver_Robot_ID = 0;
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化

static Graph_Data_t UI_HP [2];         //生命值指示
static Graph_Data_t UI_energy[2];      //缓冲能量指示
static Graph_Data_t UI_shoot_dot[1];   //射击准星
static Graph_Data_t UI_shoot_line[10]; // 射击准线
static Graph_Data_t UI_spin_state[4];  // 小陀螺指示+底盘状态
static uint32_t spin_state_location[10] = {0};
// static Graph_Data_t UI_Energy[3];      // 电容能量条
// static Graph_Data_t UI_Vision[3];      // 视觉标识框
static String_Data_t UI_State_sta[6];  // 机器人状态,静态只需画一次
// static String_Data_t UI_State_dyn[6];  // 机器人状态,动态先add才能change
static uint32_t shoot_line_location[10] = {485, 960, 600, 460, 420, 350};

#define Leg_X_Offset 1650
#define Leg_Y_Offset 500
#define Leg_Gain 500
static Graph_Data_t Leg_Graph_sta[2];
static Graph_Data_t Leg_Graph_dyn[6];
static uint32_t Processed_coord[6] = {1605, 455, 1695, 410, 1785, 455}; // Y坐标取反，放大，平移后的

void MyUIInit();
bool UI_init_flag = false;

void UI_Task(void *argument)
{
  /* USER CODE BEGIN UI_Task */

  /* Infinite loop */
  for(;;)
  {
    MyUIInit();

    MyUIRefresh(&referee_info,&UI_data);
    osDelay(1);
  }
  /* USER CODE END UI_Task */
}


void MyUIInit()
{
    if(UI_init_flag == true) //不要求初始化就直接退出
    return;

    while (referee_info.GameRobotState.robot_id == 0)
    osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查

    DeterminRobotID();                                      // 确定ui要发送到的目标客户端
    UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 0); // 清空UI
    
for (int i = 0; i < 5; i++)
    {
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]);
    UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);

    UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Main, 3, 960 + 350, 540, 960 + 650, 200);
    UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Main, 3, 960 - 350, 540, 960 - 650, 200);
    UIGraphRefresh(&referee_info.referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2] ,UI_shoot_line[3]);

    UICircleDraw(&UI_shoot_dot[0],"sc0",UI_Graph_ADD,7,UI_Color_Main, 5 ,960, 485 ,15); //准星绘制
    
    UIArcDraw(&UI_HP[0],"hp0",UI_Graph_ADD,5,UI_Color_Purplish_red, 30 ,150, 20,960,540,380,380); //HP底条绘制
    UIArcDraw(&UI_energy[0],"en0",UI_Graph_ADD,5,UI_Color_Purplish_red, 210 ,330, 20,960,540,380,380);//Energy底条绘制
    UIArcDraw(&UI_HP[1],"hp1",UI_Graph_ADD,4,UI_Color_Green, 30 + 0.5 * (150-30) ,150, 20,960,540,380,380); //HP指示条绘制
    UIArcDraw(&UI_energy[1],"en1",UI_Graph_ADD,4,UI_Color_Green, 210 , 330 - 0.5 * (330-210), 20,960,540,380,380); //Energy指示条绘制
    UIGraphRefresh(&referee_info.referee_id,5,UI_HP[0],UI_energy[0],UI_HP[1],UI_energy[1],UI_shoot_dot[0]);

    UIArcDraw(&UI_spin_state[0],"sp0",UI_Graph_ADD,6,UI_Color_Purplish_red, 330, 30,  5, 960,540,340,340); 
    // UIArcDraw(&UI_spin_state[1],"sp1",UI_Graph_ADD,6,UI_Color_Green,        60,  120, 5, 960,540,340,340);
    // UIArcDraw(&UI_spin_state[2],"sp2",UI_Graph_ADD,6,UI_Color_Green,        150, 210, 5, 960,540,340,340); 
    // UIArcDraw(&UI_spin_state[3],"sp3",UI_Graph_ADD,6,UI_Color_Green,        240, 300, 5 ,960,540,340,340);
    // UIGraphRefresh(&referee_info.referee_id,5,UI_spin_state[0],UI_spin_state[1],UI_spin_state[2],UI_spin_state[3]);
    UIGraphRefresh(&referee_info.referee_id,1,UI_spin_state[0]);

    //摩擦轮实际状态指示
    UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 8, UI_Color_Purplish_red, 64, 6, 960, 200, "F"); 
    UICharRefresh(&referee_info.referee_id, UI_State_sta[0]);
    //机器人复位状态
    UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_Green, 64, 6, 1070, 200, "R");
    UICharRefresh(&referee_info.referee_id, UI_State_sta[1]);
    //弹舱状态
    UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_Green, 64, 6, 850, 200, "o");
    UICharRefresh(&referee_info.referee_id, UI_State_sta[2]);
    // UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 150, 650, "vision:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[2]);
    // UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_Cyan, 15, 2, 150, 600, "loader:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[4]);

    UILineDraw(&Leg_Graph_dyn[1], "wd1", UI_Graph_ADD, 7, UI_Color_Purplish_red, 5,
              0 + Leg_X_Offset, 0 + Leg_Y_Offset, (uint32_t)(JOINT_DISTANCE * Leg_Gain + Leg_X_Offset), 0 + Leg_Y_Offset); // 水平线
    UILineDraw(&Leg_Graph_dyn[2], "wd2", UI_Graph_ADD, 7, UI_Color_Purplish_red, 5,
              0 + Leg_X_Offset, 0 + Leg_Y_Offset, Processed_coord[0], Processed_coord[1]); // 左大腿
    UILineDraw(&Leg_Graph_dyn[3], "wd3", UI_Graph_ADD, 7, UI_Color_Purplish_red, 5,
              Processed_coord[0], Processed_coord[1], Processed_coord[2], Processed_coord[3]); // 左小腿
    UILineDraw(&Leg_Graph_dyn[4], "wd4", UI_Graph_ADD, 7, UI_Color_Purplish_red, 5,
              (uint32_t)(JOINT_DISTANCE * Leg_Gain + Leg_X_Offset), 0 + Leg_Y_Offset, Processed_coord[4], Processed_coord[5]); // 右大腿
    UILineDraw(&Leg_Graph_dyn[5], "wd5", UI_Graph_ADD, 7, UI_Color_Purplish_red, 5,
              Processed_coord[2], Processed_coord[3], Processed_coord[4], Processed_coord[5]); // 右小腿
    UIGraphRefresh(&referee_info.referee_id, 5, Leg_Graph_dyn[1], Leg_Graph_dyn[2], Leg_Graph_dyn[3], Leg_Graph_dyn[4], Leg_Graph_dyn[5]);
    }
    
    UI_init_flag = true;

}


float angle_error = 0;
float angle_init  , angle_end;
static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    if(Robot_RunState.UI_update)  UI_init_flag = false; //触发更新
    UIChangeCheck(_Interactive_data);
    UI_Data_paking(_Interactive_data);//更新数据

    if(_Interactive_data->aim_tracking) 
    UICircleDraw(&UI_shoot_dot[0],"sc0",UI_Graph_Change,7,UI_Color_Green, 5 ,960, 485 ,15); //准星绘制
    else UICircleDraw(&UI_shoot_dot[0],"sc0",UI_Graph_Change,7,UI_Color_Purplish_red, 5 ,960, 485 ,15); //准星绘制

    //动态血条绘制
    float Buf_energy_K = (float)referee_info.PowerHeatData.buffer_energy/60.0f;
    float HP_K = (float)referee_info.GameRobotState.current_HP/(float)referee_info.GameRobotState.maximum_HP;
    UIArcDraw(&UI_HP[1],"hp1",UI_Graph_Change,4,UI_Color_Green, 30 + (1-HP_K) * (150-30) ,150, 20,960,540,380,380);
    UIArcDraw(&UI_energy[1],"en1",UI_Graph_Change,4,UI_Color_Green, 210 , 330 - (1-Buf_energy_K) * (330-210), 20,960,540,380,380);

    //动态变化，底盘方向指示，小陀螺指示
    angle_error = -_Interactive_data->chassis_gimbal_error;
    angle_init = angle_error-30 , angle_end = angle_error +30;
    angle_init = (angle_init < 0) ? (angle_init + 360) : angle_init;
    angle_end = (angle_end < 0) ? (angle_end + 360) : angle_end;
    if(_Interactive_data->Robot_spin == 1){
    UIArcDraw(&UI_spin_state[0],"sp0",UI_Graph_Change,6,UI_Color_Green, angle_init, angle_end,  5, 960,540,340,340); 
    }
    else UIArcDraw(&UI_spin_state[0],"sp0",UI_Graph_Change,6,UI_Color_Purplish_red, angle_init, angle_end,  5, 960,540,340,340); 

    // UIArcDraw(&UI_spin_state[1],"sp1",UI_Graph_Change,6,UI_Color_Green,        60,  120, 5, 960,540,340,340);
    // UIArcDraw(&UI_spin_state[2],"sp2",UI_Graph_Change,6,UI_Color_Green,        150, 210, 5, 960,540,340,340); 
    // UIArcDraw(&UI_spin_state[3],"sp3",UI_Graph_Change,6,UI_Color_Green,        24`0, 300, 5 ,960,540,340,340);
    UIGraphRefresh(&referee_info.referee_id,5,UI_HP[1],UI_energy[1],UI_shoot_dot[0],UI_spin_state[0]); //推送


    //摩擦轮实际状态指示
    if(abs(DJI_Motor.DJI_M3508_Receive[0].rpm) >5000 && DJI_Motor.DJI_M3508_Receive[1].rpm>5000) //此时摩擦轮工作正常
    { UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_Change, 8, UI_Color_Green, 64, 6, 960, 200, "F"); }
    if(DJI_Motor.DJI_M3508_Receive[0].rpm <5000 && DJI_Motor.DJI_M3508_Receive[1].rpm < 5000 &&Robot_RunState.Robot_friction ==0) //摩擦轮速度低且摩擦轮位为关 //正常关闭状态
    {UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_Change, 8, UI_Color_Purplish_red, 64, 6, 960, 200, "F"); }
    if(DJI_Motor.DJI_M3508_Receive[0].rpm <5000 && DJI_Motor.DJI_M3508_Receive[1].rpm < 5000 &&Robot_RunState.Robot_friction ==1) //摩擦轮速度低且摩擦轮位为开 //堵转了
    {UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_Change, 8, UI_Color_Yellow, 64, 6, 960, 200, "F"); }
    UICharRefresh(&referee_info.referee_id, UI_State_sta[0]); //推送

    if(_Interactive_data->Referee_Interactive_Flag.RobotReset_flag == 1){
    //机器人复位状态
    if(Robot_RunState.Robot_Reset)
    {UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_Change, 8, UI_Color_Green, 64, 6, 1070, 200, "R");}
    else 
    {UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_Change, 8, UI_Color_Purplish_red, 64, 6, 1070, 200, "R");}
    UICharRefresh(&referee_info.referee_id, UI_State_sta[1]); //推送
    _Interactive_data->Referee_Interactive_Flag.RobotReset_flag = 0;

    }

    // if(_Interactive_data->Referee_Interactive_Flag.openmag_flag == 1){
    // //弹舱状态
    if(_Interactive_data->open_mag)
    {UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_Change, 8, UI_Color_Green, 64, 6, 850, 200, "o");}
    else 
    {UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_Change, 8, UI_Color_Purplish_red, 64, 6, 850, 200, "c");}
    UICharRefresh(&referee_info.referee_id, UI_State_sta[2]); //推送
    // _Interactive_data->Referee_Interactive_Flag.openmag_flag = 0;

    // }


    if (Processed_coord[0] != (uint32_t)(_Interactive_data->coord[0] * Leg_Gain + Leg_X_Offset))
    {
        Processed_coord[0] = (uint32_t)(_Interactive_data->coord[0] * Leg_Gain + Leg_X_Offset);
        Processed_coord[1] = (uint32_t)(_Interactive_data->coord[1] * -Leg_Gain + Leg_Y_Offset);
        Processed_coord[2] = (uint32_t)(_Interactive_data->coord[2] * Leg_Gain + Leg_X_Offset);
        Processed_coord[3] = (uint32_t)(_Interactive_data->coord[3] * -Leg_Gain + Leg_Y_Offset);
        Processed_coord[4] = (uint32_t)(_Interactive_data->coord[4] * Leg_Gain + Leg_X_Offset);
        Processed_coord[5] = (uint32_t)(_Interactive_data->coord[5] * -Leg_Gain + Leg_Y_Offset);

        // 腿部运动，五连杆，不需要检测变更，实时显示变化
        UILineDraw(&Leg_Graph_dyn[2], "wd2", UI_Graph_Change, 7, UI_Color_Green, 5,
                  0 + Leg_X_Offset, 0 + Leg_Y_Offset, Processed_coord[0], Processed_coord[1]); // 左大腿

        UILineDraw(&Leg_Graph_dyn[3], "wd3", UI_Graph_Change, 7, UI_Color_Green, 5,
                  Processed_coord[0], Processed_coord[1], Processed_coord[2], Processed_coord[3]); // 左小腿

        UILineDraw(&Leg_Graph_dyn[4], "wd4", UI_Graph_Change, 7, UI_Color_Purplish_red, 5,
                  (uint32_t)(JOINT_DISTANCE * Leg_Gain + Leg_X_Offset), 0 + Leg_Y_Offset, Processed_coord[4], Processed_coord[5]); // 右大腿

        UILineDraw(&Leg_Graph_dyn[5], "wd5", UI_Graph_Change, 7, UI_Color_Purplish_red, 5,
                  Processed_coord[2], Processed_coord[3], Processed_coord[4], Processed_coord[5]); // 右小腿
        UIGraphRefresh(&referee_recv_info->referee_id, 5, Leg_Graph_dyn[1], Leg_Graph_dyn[2], Leg_Graph_dyn[3], Leg_Graph_dyn[4], Leg_Graph_dyn[5]);
    }
}

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    _Interactive_data->Robot_Reset = Robot_RunState.Robot_Reset;
    _Interactive_data->open_mag    = Robot_RunState.open_mag;
    if (_Interactive_data->Robot_Reset != _Interactive_data->Robot_Reset_last)
    {
        _Interactive_data->Referee_Interactive_Flag.RobotReset_flag = 1;
        _Interactive_data->Robot_Reset_last = _Interactive_data->Robot_Reset;
    }

    if (_Interactive_data->open_mag != _Interactive_data->open_mag_last)
    {
        _Interactive_data->Referee_Interactive_Flag.openmag_flag = 1;
        _Interactive_data->open_mag_last = _Interactive_data->open_mag;
    }


}
