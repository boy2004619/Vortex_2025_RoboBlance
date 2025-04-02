/**
 ******************************************************************************
 * @file	bsp_uart.h
 * @author  Dong xicheng
 * @version V1.0.0
 * @date    2024/7/9
 * @brief
 *******************************************************************************
 */
#ifndef _BSP_UART_H
#define _BSP_UART_H

#include "main.h"

extern uint8_t uart5_buffer[20];
extern uint8_t uart7_buffer[256];
extern uint8_t uart2_buffer[256];
extern uint8_t uart10_buffer[256];
extern uint8_t uart1_buffer[256];

/**
 * @brief 串口空闲中断调用函数
 *
 * @param huart 	需要处理的串口定义
 * @return  无
 */
void Usart_Receive_Data(UART_HandleTypeDef *huart);
void UART1_TX_DMA_Send(uint8_t *buffer, uint16_t length);

void BspUart_Init(void);

#endif /* __BSP_UART_H_ */
