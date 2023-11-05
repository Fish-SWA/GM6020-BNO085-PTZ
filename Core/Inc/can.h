/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
#define SPEED_PID_TUNING 0        //速度环科学调参
#define ABS_ANGLE_PID_TUNING 1    //陀螺仪位置环科学调参
#define REL_ANGLE_PID_TUNING 2    //编码器位置环科学调参
#define ABS_ANGLE_PID_TEST 3      //陀螺仪位置环PID测试
#define REL_ANGLE_PID_TEST 4      //编码器位置环PID测试
#define ABS_ANGLE_STABLE_MODE 5   //陀螺仪位置模式
#define REL_ANGLE_STABLE_MODE 6   //编码器位置模式
/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

