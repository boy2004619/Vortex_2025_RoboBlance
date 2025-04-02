/**
 * @file IMU_CH040.C
 * @brief 这个是文件是超核的CH040的读取实现，
 *        注意：没有验证过，但是保留下来，后续如果使用请验证无误后使用
 *
 */

#include "IMU_CH040.h"

hi91_t CH040_ORIData;
CH040_Data_Packet_t CH040_Data;

bool IMU_CH040_Dispose(uint8_t *pData)
{
    int rx_len = sizeof(hi91_t);
    bool res = false;
    if(pData[0] == 0X5A && pData[1] == 0XA5)
    {
      //这里有CRC但是没做
      memcpy(&CH040_ORIData, pData, rx_len);
      res = true;
    }else res = false;
		
		
	return res;
}

/**
 * @file  IMU_C.C
 * @brief 这个是文件是地盘C板上发的IMU数据的读取实现
 *
 */

IMUC_Data_Packet_t IMUC_Data;
bool IMUC_ready = false;

bool IMUC_Dispose(uint8_t *pData)
{
  static int IMUC_ready_num =0;
    int rx_len = sizeof(IMUC_Data_Packet_t);
    bool res = false;
    if(pData[0] ==0XA5 && pData[1] == 0X5A)
    {
      IMUC_ready_num++;
      memcpy(&IMUC_Data, pData, rx_len);

      if(IMUC_ready_num >1000)  IMUC_ready = true;
      res = true;
    }else res = false;
	return res;
}

