/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* CubeMX Create Includes */
#include "cmsis_os.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "dma.h"
#include "spi.h"


/* general Includes */
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "arm_math.h"
#include "FreeRTOS.h"	  
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "user_lib.h"


/* Define Includes */
#include "Motordef.h"

/* User_BSP Includes */
#include "can_bsp.h"
#include "bsp_dwt.h"
#include "bsp_uart.h"

/* Module Includes */
#include "remind.h"
#include "controller.h"
#include "YS_Motor.h"
#include "LK_Motor.h"
#include "DM_Motor.h"
#include "DJI_Motor.h"
#include "crc16.h"
#include "DR16.h"
#include "IMU_N100.h"
#include "ins_task.h"
#include "kalman_filter.h"
#include "robot_def.h"
#include "IMU_HWT906.h"
#include "IMU_CH040.h"
#include "SuperCap.h"
#include "rm_referee.h"
#include "servo_motor.h"
#include "CRC.h"
#include "PC_ctrl.h"
#include "Vision.h"

/* Task Includes */
#include "MotorTask.h"
#include "IMUTask.h"

/* RoboAPP Includes */
#include "balance.h"
#include "gimbal.h"
#include "shoot.h"


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint32_t g_osRuntimeCounter;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ACC_CS_Pin GPIO_PIN_0
#define ACC_CS_GPIO_Port GPIOC
#define GYRO_CS_Pin GPIO_PIN_3
#define GYRO_CS_GPIO_Port GPIOC
#define ACC_INT_Pin GPIO_PIN_10
#define ACC_INT_GPIO_Port GPIOE
#define GYRO_INT_Pin GPIO_PIN_12
#define GYRO_INT_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
