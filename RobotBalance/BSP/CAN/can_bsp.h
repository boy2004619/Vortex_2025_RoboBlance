#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__

#include "main.h"


extern uint8_t rx_data1[8];
extern uint8_t rx_data2[8];

void can_bsp_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);

#endif /* __CAN_BSP_H_ */

