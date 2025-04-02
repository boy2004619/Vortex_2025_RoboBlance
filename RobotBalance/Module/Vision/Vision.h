#ifndef _VISION_H
#define _VISION_H

#include "main.h"

#define  Vision_FRAME_LENGTH      sizeof(ReceivePacket)

//自瞄模式下发给MiniPC数据的长度
#define AUTO_TX_PACKET_LEN    100
#define VISION_FRAME_HEADER   0XA5


//发送数据结构体
typedef __packed struct
{
  uint8_t header;
  uint8_t detect_color : 1;  // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum;
} SendPacket;


//接收数据结构体
typedef __packed struct
{
  uint8_t header;
  bool tracking : 1;
  uint8_t id : 3;          // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal
  uint8_t reserved : 1;
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float dz;
  uint16_t checksum;
} ReceivePacket;

extern ReceivePacket Vision_Receive_Data;
bool Vision_ReceiveData_Dispose(uint8_t *pData);

#endif


