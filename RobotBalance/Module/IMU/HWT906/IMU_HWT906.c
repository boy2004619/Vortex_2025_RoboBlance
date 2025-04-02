#include "IMU_HWT906.h"
#include "wit_c_sdk.h"

WIT_JY901S_IMU IMU_HWT906_IMUData;
WIT_IMU_ORIData_Packet WIT_ORIData;
HWT906_Data_Packet_t   HWT906_Data;

void IMU_HWT906_Init(void)
{
 WIT_JY901S_Init(); //维特通用陀螺仪初始化
}

bool IMU_HWT906_Dispose(uint8_t *pData)
{
    int rx_len = sizeof(WIT_IMU_ORIData_Packet);
    bool res = false;
    if(pData[0] == 0X55 )
    {
      memcpy(&WIT_ORIData, pData, rx_len);  
      res = true;
    }else res = false;
		
		if(WIT_ORIData.data_id == WIT_ACC){
    HWT906_Data.Roll_Acc = WIT_ORIData.Data[0]/32768.0f*16.0f*9.8f;
    HWT906_Data.Pitch_Acc= WIT_ORIData.Data[1]/32768.0f*16.0f*9.8f;
    HWT906_Data.Yaw_Acc  = WIT_ORIData.Data[2]/32768.0f*16.0f*9.8f;
    }

    if(WIT_ORIData.data_id2 == WIT_GYRO){
    HWT906_Data.Pitch_Speed = WIT_ORIData.Data2[0]/32768.0f*2000.0f;
    HWT906_Data.Roll_Speed= WIT_ORIData.Data2[1]/32768.0f*2000.0f;
    HWT906_Data.Yaw_Speed  = WIT_ORIData.Data2[2]/32768.0f*2000.0f;
    }

    if(WIT_ORIData.data_id3 == WIT_ANGLE){
    HWT906_Data.Pitch_Angle = WIT_ORIData.Data3[0]/32768.0f*180.0f;
    HWT906_Data.Roll_Angle= WIT_ORIData.Data3[1]/32768.0f*180.0f;
    HWT906_Data.Yaw_Angle  = WIT_ORIData.Data3[2]/32768.0f*180.0f;
    }

    if(WIT_ORIData.data_id4 == WIT_QUATER){
    HWT906_Data.Q[0] = WIT_ORIData.Data4[0]/32768.0f;
    HWT906_Data.Q[1] = WIT_ORIData.Data4[1]/32768.0f;
    HWT906_Data.Q[2] = WIT_ORIData.Data4[2]/32768.0f;
    HWT906_Data.Q[3] = WIT_ORIData.Data4[3]/32768.0f;
    }
		
	return res;
}

