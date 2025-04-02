#ifndef __DR16_H
#define __DR16_H

#include "main.h"


/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<7)
#define RC_FRAME_LENGTH 18u

/* ----------------------- PC Key Definition-------------------------------- */
// 对应key[x][0~16],获取对应的键;例如通过key[KEY_PRESS][Key_W]获取W键是否按下,后续改为位域后删除
#define Key_W 0
#define Key_S 1
#define Key_D 2
#define Key_A 3
#define Key_Shift 4
#define Key_Ctrl 5
#define Key_Q 6
#define Key_E 7
#define Key_R 8
#define Key_F 9
#define Key_G 10
#define Key_Z 11
#define Key_X 12
#define Key_C 13
#define Key_V 14
#define Key_B 15


// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

#define KEY_PRESS 0
#define KEY_PRESS_WITH_CTRL 1
#define KEY_PRESS_WITH_SHIFT 2


typedef union
{
    struct // 用于访问键盘状态
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    }bit;
    uint16_t key_code; // 用于memcpy而不需要进行强制类型转换
} Key_t;

typedef struct RCDATA
{
	int16_t CH0;
	int16_t CH1;
	int16_t CH2;
	int16_t CH3;
	int8_t S1;
	int8_t S2;
	
	int16_t X;
	int16_t Y;
	int16_t Z;
	
	int8_t PRESS_L;
	int8_t PRESS_R;
	
  Key_t   kb;
  int16_t CH4;
  
  Key_t   key[3];
  int8_t  key_count[3][16];
}RCData_t;


typedef enum
{
  RC = 0,
	PC = 1	  
} remote_mode;

typedef struct Roborunstate  //由遥控器或PC提供的机器人追踪的目标值的接口结构体
{
  remote_mode PC_or_RC;

  float Yaw;
  float Pitch;

  float Advance;
  float Retreat;
  float Left;
  float Right;

  bool game_mode;   //比赛模式 ，判断是否进入了正式比赛，还是日常操控

  bool fire;        //开火
  bool auto_aim;    //自瞄
  bool Robot_stop;  //停止
  bool Robot_Reset; //复位
  bool Robot_spin;  //小陀螺
  bool Robot_friction;  //摩擦轮
  bool chassis_follow;  //底盘跟随 
  bool joint_disenable;  //关节电机失能位，用于根据裁判系统对底盘的使能状态开断关节电机
  bool F_reversal;       //摩擦轮反转
  bool UI_update;       //手动刷新ui
  bool open_mag;        //开弹仓
  bool Robot_spin_high; //较高速度的小陀螺

  float roll_ref;        //机器人roll期望角度
  float L_leg_len;
  float R_leg_len;
  bool jump;

}Robo_runstate;

extern RCData_t RC_Rx;
extern RCData_t PC_Rx;
extern RCData_t RCData_TD[2];
extern Robo_runstate Robot_RunState;
/* -----------------------PC Key-------------------------------- */
#define KEY_W               RC_Rx.kb.bit.W		
#define KEY_S               RC_Rx.kb.bit.S		
#define KEY_A               RC_Rx.kb.bit.A		
#define KEY_D               RC_Rx.kb.bit.D	
#define KEY_SHIFT           RC_Rx.kb.bit.SHIFT	
#define KEY_CTRL            RC_Rx.kb.bit.CTRL		
#define KEY_Q               RC_Rx.kb.bit.Q		
#define KEY_E               RC_Rx.kb.bit.E		
#define KEY_R               RC_Rx.kb.bit.R		
#define KEY_F               RC_Rx.kb.bit.F		
#define KEY_G               RC_Rx.kb.bit.G		
#define KEY_Z               RC_Rx.kb.bit.Z		
#define KEY_X               RC_Rx.kb.bit.X		
#define KEY_C               RC_Rx.kb.bit.C		
#define KEY_V               RC_Rx.kb.bit.V		
#define KEY_B               RC_Rx.kb.bit.B		

#define KEY_ALL_CODE        RC_Rx.kb.key_code

#define    MOUSE_X_MOVE_SPEED    (RC_Rx.X)
#define    MOUSE_Y_MOVE_SPEED    (RC_Rx.Y)
#define    MOUSE_Z_MOVE_SPEED    (RC_Rx.Z)

/* 检测鼠标按键状态 */
#define    MOUSE_LEFT    (RC_Rx.PRESS_L )
#define    MOUSE_RIGH    (RC_Rx.PRESS_R )

void DR16_Rx(uint8_t *pData);
void RoboState_Init(void);
void RoboState_Change(RCData_t *RC_Data ,RCData_t *PC_Data, Robo_runstate *Robot_RunData);

#endif
