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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "pid.h"
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
PID_TypeDef motor_pid[4]; //速度环参数
PID_TypeDef angle_pid[2]; //位置环参数

uint8_t ReceiveBuffer[6] = {0}; //IIC接收缓冲区

PID_TypeDef abs_pid[2];           //绝对角度（自稳，陀螺仪）
float ABS_Gimbal_angle[2] = {0};  //云台绝对角度(自稳) {yaw, pitch}, 0-8192 3400-6000
int16_t ABS_IMU_angle[2] = {0};   //归一化之后的IMU角度，用于自稳 {yaw, pitch}, 0-8192
int16_t IMU_Angle_Raw[3] = {0};   //接收到的陀螺仪欧拉角
float IMU_Angle[3] = {0};         //转化为浮点的陀螺仪欧拉角

int16_t Gimbal_angle[2] = {7500, 4800};   //云台角度设定 {yaw, pitch}，编码器

int current_limit = 2000; //限流

int i = 0;
int j = 0;
int current = 0;
int speed = 0;
int time = 0;
int speed_set = 0;
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
  /* USER CODE BEGIN 2 */
  
	HAL_CAN_Start(&hcan1);
	my_can_filter_init_recv_all(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  // enable IT
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //YAW轴PID
  pid_init(&motor_pid[0]); //速度环
  motor_pid[0].f_param_init(&motor_pid[0],PID_Speed,current_limit,5000,0,0,0,8000,61.4656753,0.6697404,0);   
  pid_init(&angle_pid[0]); //位置环（相对位置）
  angle_pid[0].f_param_init(&angle_pid[0],PID_Speed,300,300,0,0,4000,0,1.3713,0.00495369,94.8474*0);
  pid_init(&abs_pid[0]); //位置环（自稳）
  abs_pid[0].f_param_init(&abs_pid[0],PID_Speed,300,300,0,0,4000,0,-0.9,-0.003,-3);

  //Pitch轴PID
  pid_init(&motor_pid[1]); //速度环
  motor_pid[1].f_param_init(&motor_pid[1],PID_Speed,current_limit,5000,0,0,0,8000,4.01239768,3.0348503,0);   
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

//陀螺仪位置控制
void Abs_angle_control_loop()
{
  /*自稳控制*/
  //IMU归一化
  ABS_IMU_angle[0] = (int)((IMU_Angle[2]*8192)/360); //yaw
  ABS_IMU_angle[1] = (int)(((IMU_Angle[0]+180.0)*8192)/360); //pitch
  //YAW计算
  int motor_ID = 0;
  abs_pid[motor_ID].target = ABS_Gimbal_angle[0];
  abs_pid[motor_ID].f_cal_pid(&abs_pid[motor_ID], ABS_IMU_angle[motor_ID], 8192);
  motor_pid[motor_ID].target = abs_pid[motor_ID].output; 																							
  motor_pid[motor_ID].f_cal_pid(&motor_pid[motor_ID],moto_chassis[motor_ID].speed_rpm, 0);

  //PITCH计算
  motor_ID = 1;
  abs_pid[motor_ID].target = ABS_Gimbal_angle[1];
  abs_pid[motor_ID].f_cal_pid(&abs_pid[motor_ID], ABS_IMU_angle[motor_ID], 8192);
  motor_pid[motor_ID].target = abs_pid[motor_ID].output; 																							
  motor_pid[motor_ID].f_cal_pid(&motor_pid[motor_ID],moto_chassis[motor_ID].speed_rpm, 0);


  set_moto_current(&hcan1, motor_pid[0].output, motor_pid[1].output, 0, 0);
  printf("%d, %d\n", moto_chassis[motor_ID].angle, ABS_IMU_angle[motor_ID]);
  HAL_Delay(1);
}

//编码器位置控制
void Rel_angle_control_loop()
{
    
    i++;
  /*自稳计算*/
  //IMU归一化
  ABS_IMU_angle[0] = (int)(IMU_Angle[2]*(8192/360)); //yaw
  ABS_IMU_angle[1] = (int)((IMU_Angle[0]+180.0)*(8192/360)); //pitch

  if(Gimbal_angle[0] > 8191) Gimbal_angle[0] = 0;
  else if(Gimbal_angle[0] < 0) Gimbal_angle[0] = 8191;
    //用PID计算电流
    //这里用for会有奇怪的问题，暂时展开
    int motor_ID = 0;
    angle_pid[motor_ID].target = Gimbal_angle[0];
    angle_pid[motor_ID].f_cal_pid(&angle_pid[motor_ID], moto_chassis[motor_ID].angle, 8192);
    motor_pid[motor_ID].target = angle_pid[motor_ID].output;
    motor_pid[motor_ID].f_cal_pid(&motor_pid[motor_ID],moto_chassis[motor_ID].speed_rpm, 0);

    motor_ID = 1;
    angle_pid[motor_ID].target = Gimbal_angle[1];
    angle_pid[motor_ID].f_cal_pid(&angle_pid[motor_ID], moto_chassis[motor_ID].angle, 8192);
    motor_pid[motor_ID].target = angle_pid[motor_ID].output; 																							
    motor_pid[motor_ID].f_cal_pid(&motor_pid[motor_ID],moto_chassis[motor_ID].speed_rpm, 0);

    set_moto_current(&hcan1, motor_pid[0].output, motor_pid[1].output, 0, 0);
    printf("%d, %d\n", moto_chassis[0].angle, moto_chassis[1].angle);
    printf("%f\n", IMU_Angle[0]);
    HAL_Delay(1); //1000hz
}

//自稳PID效果测试
void Abs_angle_PID_test_loop(int motor_ID)
{
  //自稳效果测试
  //IMU归一化
  ABS_IMU_angle[0] = (int)((IMU_Angle[2]*8192)/360); //yaw
  ABS_IMU_angle[1] = (int)(((IMU_Angle[0]+180.0)*8192)/360); //pitch
  abs_pid[motor_ID].target = 0;
  abs_pid[motor_ID].f_cal_pid(&abs_pid[motor_ID], ABS_IMU_angle[motor_ID], 8192);

  motor_pid[motor_ID].target = abs_pid[motor_ID].output; 																							
  motor_pid[motor_ID].f_cal_pid(&motor_pid[motor_ID],moto_chassis[motor_ID].speed_rpm, 0);

  if(motor_ID == 0) set_moto_current(&hcan1, motor_pid[motor_ID].output, 0, 0, 0);
  if(motor_ID == 1) set_moto_current(&hcan1, 0, motor_pid[motor_ID].output, 0, 0);
  //set_moto_current(&hcan1, 0, 0, 0, 0);
  //printf("%d\n", moto_chassis[0].speed_rpm);
  //printf("%d\n", moto_chassis[0].total_angle);
  //printf("%d, %f, %f\n", ABS_IMU_angle[motor_ID], abs_pid[motor_ID].output, motor_pid[motor_ID].output);
  printf("%d, %d\n", moto_chassis[motor_ID].angle, ABS_IMU_angle[motor_ID]);
  HAL_Delay(1);
}

//位置环PID效果测试
void Rel_angle_PID_test_loop(int motor_ID)
{
  angle_pid[motor_ID].target = 7000;
  angle_pid[motor_ID].f_cal_pid(&angle_pid[motor_ID], moto_chassis[motor_ID].angle, 8192);
  //printf("%d\n", (int)angle_pid.output);

  motor_pid[motor_ID].target = angle_pid[motor_ID].output; 																							
  motor_pid[motor_ID].f_cal_pid(&motor_pid[motor_ID],moto_chassis[motor_ID].speed_rpm, 0);

  if(motor_ID == 0) set_moto_current(&hcan1, motor_pid[motor_ID].output, 0, 0, 0);
  if(motor_ID == 1) set_moto_current(&hcan1, 0, motor_pid[motor_ID].output, 0, 0);
  //set_moto_current(&hcan1, 0, 0, 0, 0);
  //printf("%d\n", moto_chassis[0].speed_rpm);
  //printf("%d\n", moto_chassis[0].total_angle);
  printf("%f, ", (float)j);
  HAL_Delay(1);
  printf("%d, %d, %d, %d, %f, %f, %d\n", 
                    speed_set, moto_chassis[motor_ID].angle, 
                    (int)motor_pid[motor_ID].output, 
                    (int)motor_pid[motor_ID].iout, 
                    angle_pid[motor_ID].output, 
                    IMU_Angle[motor_ID]);
}

//速度环科学调参（编码器角速度）
void speed_loop_PID_tuning(int motor_ID)
{
  speed = moto_chassis[motor_ID].speed_rpm;
  time++;
  if(i <= 400) current = 0;
  else current = 1500;

  if(i >= 800) i = 0;

  printf("%f, ", (float)j);
  printf("%d, %d\n", current, speed);
  if(motor_ID == 0) set_moto_current(&hcan1, current, 0, 0, 0);
  if(motor_ID == 1) set_moto_current(&hcan1, 0, current, 0, 0);
}

//编码器位置环科学调参
void Rel_angleloop_PID_tuning(int motor_ID)
{
  if(i <= 100) speed_set = 0;
  else if(i >100 && i<=1100) speed_set = -15;
  else if(i >1100 && i<=2100) speed_set = 15;
  if(i > 2100) i = 0;
  //speed_set = 25;
  
  motor_pid[motor_ID].target = speed_set; 																							
  motor_pid[motor_ID].f_cal_pid(&motor_pid[motor_ID], moto_chassis[motor_ID].speed_rpm, 0);
  if(motor_ID == 0) set_moto_current(&hcan1, motor_pid[motor_ID].output, 0, 0, 0);
  if(motor_ID == 1) set_moto_current(&hcan1, 0, motor_pid[motor_ID].output, 0, 0);
  //set_moto_current(&hcan1, 0, 0, 0, 0);
  //printf("%d\n", moto_chassis[0].speed_rpm);
  //printf("%d\n", moto_chassis[0].total_angle);
  printf("%f, ", (float)j);
  printf("%d, %d\n", speed_set, moto_chassis[motor_ID].total_angle);
}

//陀螺仪位置环科学调参
void Abs_anglelop_PID_tuning(int motor_ID)
{
  //IMU归一化
  ABS_IMU_angle[0] = (int)((IMU_Angle[2]*8192)/360); //yaw
  ABS_IMU_angle[1] = (int)(((IMU_Angle[0]+180.0)*8192)/360); //pitch

  if(i <= 100) speed_set = 0;
  else if(i >100 && i<=1100) speed_set = -15;
  else if(i >1100 && i<=2100) speed_set = 15;
  if(i > 2100) i = 0;
  //speed_set = 25;
  
  motor_pid[motor_ID].target = speed_set; 																							
  motor_pid[motor_ID].f_cal_pid(&motor_pid[motor_ID], moto_chassis[motor_ID].speed_rpm, 0);
  if(motor_ID == 0) set_moto_current(&hcan1, motor_pid[motor_ID].output, 0, 0, 0);
  if(motor_ID == 1) set_moto_current(&hcan1, 0, motor_pid[motor_ID].output, 0, 0);
  //set_moto_current(&hcan1, 0, 0, 0, 0);
  //printf("%d\n", moto_chassis[0].speed_rpm);
  //printf("%d\n", moto_chassis[0].total_angle);
  printf("%f, ", (float)j);
  printf("%d, %d\n", speed_set, ABS_IMU_angle[motor_ID]);
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
