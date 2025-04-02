#include "bsp_uart.h"

/**
 * @file bsp_uart.c 
 * @author Vortex.D
 * 
 * @What? :       串口BSP文件，目前主要是串口空闲中断触发时的DMA中的数据处理
 * @How to use? : 
 * @todo:   后续弃用Mycallback_handler ，Usart_Receive_Data，
 *          现在是因为串口7在转移到 HAL_UARTEx_RxEventCallback 有不稳定现象。所以保留了Usart_Receive_Data 。临近比赛，暂不追究
 * 
 * @移植指导：自主实现 XXX_Dispose((uint8_t *)uartX_buffer); 函数，在指定位置调用即可。 
 *           如果使用 Mycallback_handler + Usart_Receive_Data 的方式，记得使用初始化并在it文件中调用 Usart_Receive_Data;
 */
/********** @Include User Function **********/

    void Mycallback_handler(UART_HandleTypeDef *huart);          /* 自己实现的串口数据处理 */
    void Usart_Receive_Data(UART_HandleTypeDef *huart);          /* 串口空闲中断接口函数，要使用这个函数的话在生成的 it文件中的串口中断去调用他 */
    
    void BspUart_Init(void);                                     /* 串口初始化函数 ， 这个函数在mian.c初始化区调用 */
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size);  /* 串口空闲中断回调函数 */
    void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart);      /* 串口错误中断处理 */


/********** @Include User Function **********/


//串口数据缓存数组
uint8_t uart1_buffer[256];
uint8_t uart2_buffer[256];
uint8_t uart5_buffer[20];
uint8_t uart7_buffer[256];
uint8_t uart10_buffer[256];


void Mycallback_handler(UART_HandleTypeDef *huart)
{
   	// if(huart->Instance==USART2 ){
	// HAL_UART_DMAStop(&huart2); 
      
	//  	/*写下你自己的中断处理过程*/
    // YS_MOTORData_Dispose((uint8_t *)uart2_buffer, &Joint_Motor);

    // memset(uart2_buffer,0,sizeof(uart2_buffer));
	// HAL_UART_Receive_DMA(&huart2, uart2_buffer, sizeof(uart2_buffer));             //重启开始DMA传输
	//  }

    // if(huart->Instance==UART5 ){
	// HAL_UART_DMAStop(&huart5); 
      
	// 	/*写下你自己的中断处理过程*/
    //    DR16_Rx((uint8_t *)uart5_buffer);

	// memset(uart5_buffer,0,sizeof(uart5_buffer));
	// HAL_UART_Receive_DMA(&huart5, uart5_buffer, RC_FRAME_LENGTH);             //重启开始DMA传输
	// }

    if(huart->Instance==UART7 ){
	 HAL_UART_DMAStop(&huart7); 

     /*写下你自己的中断处理过程*/
      IMUC_Dispose((uint8_t *)uart7_buffer);

	   memset(uart7_buffer,0,sizeof(uart7_buffer));
	   HAL_UART_Receive_DMA(&huart7, uart7_buffer, sizeof(uart7_buffer));             //重启开始DMA传输
	}

//    if(huart->Instance==USART10 ){
// 		HAL_UART_DMAStop(&huart10); 

//      /*写下你自己的中断处理过程*/
//     JudgeReadData((uint8_t *)uart10_buffer);

// 	   memset(uart10_buffer,0,sizeof(uart10_buffer));
// 	   HAL_UART_Receive_DMA(&huart10, uart10_buffer, sizeof(uart10_buffer));             //重启开始DMA传输
// 	}

   
   if(huart->Instance==USART1 ){
	HAL_UART_DMAStop(&huart1); 

     /*写下你自己的中断处理过程*/
	imagelinkReadData((uint8_t *)uart1_buffer);

	memset(uart1_buffer,0,sizeof(uart1_buffer));
	HAL_UART_Receive_DMA(&huart1, uart1_buffer, sizeof(uart1_buffer));             //重启开始DMA传输
	}
}


void Usart_Receive_Data(UART_HandleTypeDef *huart)
{
   if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   //判断是否是空闲中断
   {
      __HAL_UART_CLEAR_IDLEFLAG(huart);                     //清除空闲中断标志（否则会一直不断进入中断）
      Mycallback_handler(huart);                            //调用中断处理函数,这个函数自己写
   }
}

void BspUart_Init(void)
{
	// HAL_UARTEx_ReceiveToIdle_DMA(&huart10, uart10_buffer, sizeof(uart10_buffer));
	// __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
}


uint32_t uart_dwt_cnt;
float uartdel_t;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{

	if(huart->Instance == USART2)
    {
        HAL_UART_DMAStop(&huart2);
		/*写下你自己的中断处理过程*/
		YS_MOTORData_Dispose((uint8_t *)uart2_buffer, &Joint_Motor);
        memset(uart2_buffer, 0, sizeof(uart2_buffer));							   // 清除接收缓存
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_buffer, sizeof(uart2_buffer)); // 接收完毕后重启
    }
    if(huart->Instance == UART5)
    {
        HAL_UART_DMAStop(&huart5);
		/*写下你自己的中断处理过程*/
		DR16_Rx((uint8_t *)uart5_buffer);
        memset(uart5_buffer, 0, sizeof(uart5_buffer));							   // 清除接收缓存
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uart5_buffer, sizeof(uart5_buffer)); // 接收完毕后重启
        // __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
    }
	// if(huart->Instance == UART7)
    // {
	// 	// uartdel_t = DWT_GetDeltaT(&uart_dwt_cnt);
    //     HAL_UART_DMAStop(&huart7);
	// 	/*写下你自己的中断处理过程*/
	// 	IMUC_Dispose((uint8_t *)uart7_buffer);
    //     memset(uart7_buffer, 0, sizeof(uart7_buffer));							   // 清除接收缓存
    //     HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_buffer, sizeof(uart7_buffer)); // 接收完毕后重启
    //     // __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
    // }
	if(huart->Instance == USART10)
    {
        HAL_UART_DMAStop(&huart10);
		/*写下你自己的中断处理过程*/
		JudgeReadData((uint8_t *)uart10_buffer);
        memset(uart10_buffer, 0, sizeof(uart10_buffer));							   // 清除接收缓存
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, uart10_buffer, sizeof(uart10_buffer)); // 接收完毕后重启
        // __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
    }

}


void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == USART2)
    {   
        memset(uart2_buffer, 0, sizeof(uart2_buffer));
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_buffer, sizeof(uart2_buffer)); // 接收发生错误后重启
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
        
    }
    if(huart->Instance == UART5)
    {
        memset(uart5_buffer, 0, sizeof(uart5_buffer));
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uart5_buffer, sizeof(uart5_buffer)); // 接收发生错误后重启
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
        
    }
	if(huart->Instance == UART7)
    {
    memset(uart7_buffer,0,sizeof(uart7_buffer));
	HAL_UART_Receive_DMA(&huart7, uart7_buffer, sizeof(uart7_buffer));             //重启开始DMA传输
		// HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_buffer, sizeof(uart7_buffer)); // 接收发生错误后重启
		// __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
        // memset(uart7_buffer, 0, sizeof(uart7_buffer));
        
    }
	if(huart->Instance == USART10)
    {
        memset(uart10_buffer, 0, sizeof(uart10_buffer));
		HAL_UARTEx_ReceiveToIdle_DMA(&huart10, uart10_buffer, sizeof(uart10_buffer)); // 接收发生错误后重启
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
    }
}


