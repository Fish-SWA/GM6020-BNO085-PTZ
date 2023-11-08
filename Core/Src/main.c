/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "pid_control_tuning.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "pid.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t ReceiveBuffer[6] = {0}; //IIC接收缓冲区

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
	MX_USB_DEVICE_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  
	HAL_CAN_Start(&hcan1);
	my_can_filter_init_recv_all(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  // enable IT
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //YAW轴PID
  pid_init(&motor_pid[0]); //速度环
  motor_pid[0].f_param_init(&motor_pid[0],PID_Speed,CURRENT_LIMIT,5000,0,0,0,8000,61.4656753,0.6697404,0);   
  pid_init(&angle_pid[0]); //位置环（相对位置）
  angle_pid[0].f_param_init(&angle_pid[0],PID_Speed,300,300,0,0,4000,0,1.3713,0.00495369,94.8474*0);
  pid_init(&abs_pid[0]); //位置环（自稳）
  abs_pid[0].f_param_init(&abs_pid[0],PID_Speed,300,300,0,0,4000,0,-0.9,-0.003,-3);

  //Pitch轴PID
  pid_init(&motor_pid[1]); //速度环
  motor_pid[1].f_param_init(&motor_pid[1],PID_Speed,CURRENT_LIMIT,5000,0,0,0,8000,4.01239768,3.0348503,0);   
  pid_init(&angle_pid[1]); //位置环（相对位置）
  angle_pid[1].f_param_init(&angle_pid[1],PID_Speed,500,500,0,0,4000,0,1.27137,0.01084,27.877753*0.4);
  pid_init(&abs_pid[1]); //位置环（自稳）
  abs_pid[1].f_param_init(&abs_pid[1],PID_Speed,300,300,0,0,4000,0,0.5,0,10);



  printf("--------------init--------------\n");
  HAL_I2C_Slave_Receive_IT(&hi2c2,ReceiveBuffer,sizeof(ReceiveBuffer));
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		char buf[256] = "hiiiiiiiiiii!\n";
		CDC_Transmit_FS(buf,sizeof(buf));
  
    switch (ABS_ANGLE_STABLE_MODE)
    {
    case SPEED_PID_TUNING: 
        speed_loop_PID_tuning(0);
      break;
    
    case ABS_ANGLE_PID_TUNING: 
        Abs_anglelop_PID_tuning(0);
      break;
    
    case REL_ANGLE_PID_TUNING:
        Rel_angleloop_PID_tuning(0);
      break;
    
    case ABS_ANGLE_PID_TEST:
        Abs_angle_PID_test_loop(0);
      break;

    case REL_ANGLE_PID_TEST:
        Rel_angle_PID_test_loop(0);
      break;

    case ABS_ANGLE_STABLE_MODE:
        Abs_angle_control_loop();
      break;
    
    case REL_ANGLE_STABLE_MODE:
        Rel_angle_control_loop();
      break;
    
    }

    /*Debug*/
    if(false)
    {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(100);
    }
    i++;
    j++;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  //printf("data1 = %d\n", ReceiveBuffer[1]);
  uint8_t IMU_data[6] = {0};
  for(int i=0; i<6; i++)
  {
    IMU_data[i] = ReceiveBuffer[i];
  }
  IMU_Angle_Raw[0] = (int16_t)(IMU_data[0]<<8 | IMU_data[1]);
  IMU_Angle_Raw[1] = (int16_t)(IMU_data[2]<<8 | IMU_data[3]);
  IMU_Angle_Raw[2] = (int16_t)(IMU_data[4]<<8 | IMU_data[5]);
  IMU_Angle[0] = (float)IMU_Angle_Raw[0]/100;
  IMU_Angle[1] = (float)IMU_Angle_Raw[1]/100;
  IMU_Angle[2] = (float)IMU_Angle_Raw[2]/10;
//  printf("%f, %d, %d, \n", 
//        (int16_t)(IMU_data[0]<<8 | IMU_data[1])/100, 
//        (int16_t)(IMU_data[2]<<8 | IMU_data[3]), 
//        (int16_t)(IMU_data[4]<<8 | IMU_data[5])*10);
  printf("%f, %f, %f\n", IMU_Angle[0], IMU_Angle[1], IMU_Angle[2]);
  HAL_I2C_Slave_Receive_IT(&hi2c2,ReceiveBuffer,sizeof(ReceiveBuffer));
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
