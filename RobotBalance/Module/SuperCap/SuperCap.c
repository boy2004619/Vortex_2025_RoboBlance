#include "SuperCap.h"

void SuperCap_send_data(void)
{
DJI_Motor_Send(&hfdcan1 ,0X210 , 55*100 ,0,0,0);
}


