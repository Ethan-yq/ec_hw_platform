/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
//uint16_t ADC_Value;
//uint16_t pwmValue=0;


//uint8_t TxData[10]="hello";
//uint8_t RxData[10];

//uint16_t psc=0,pwm=0;
//#define MIN_BUZZER_PWM 10000
//#define MAX_BUZZER_PWM 20000
//#define MAX_PSC 1000
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_IWDG_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_CAN2_Init();
  MX_TIM8_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
//  HAL_TIM_Base_Start_IT(&htim1);
//  HAL_UART_Init(&huart1);
//  HAL_UART_Receive_IT(&huart1,RxData,5);

//    HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_RESET);

//    HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
//    HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);

//    HAL_TIM_Base_Start(&htim4);
//    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);

//    HAL_TIM_Base_Start(&htim1);
//    HAL_TIM_Base_Start(&htim8);
//    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
//    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
//    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
//    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
//    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//      __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,pwm);


//      pwm++;
//      psc++;
//
//      if(pwm > MAX_BUZZER_PWM)
//      {
//          pwm = MIN_BUZZER_PWM;
//      }
//      if(psc > MAX_PSC)
//      {
//          psc = 0;
//      }
//      buzzer_on(psc, pwm);
//      HAL_Delay(1);



//      HAL_UART_Transmit_IT(&huart1,TxData,5);
//      HAL_Delay(1000);


//      HAL_UART_Transmit(&huart1,TxData,5,500);
//      HAL_UART_Receive(&huart1,RxData,5,500);
//      if(RxData[0]=='1'){
//          HAL_GPIO_TogglePin(LED_B_GPIO_Port,LED_B_Pin);
//          HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
//          HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);
//          RxData[0]=0;
//      }


//      HAL_ADC_Start(&hadc3);
//      HAL_ADC_PollForConversion(&hadc3,50);
//      if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc3),HAL_ADC_STATE_REG_EOC)){
//          ADC_Value= HAL_ADC_GetValue(&hadc3);
//      }
//      HAL_Delay(500);



//      while (pwmValue<500){
//          ++pwmValue;
//          __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,pwmValue);
//          __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,pwmValue);
//          __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,pwmValue);
//          HAL_Delay(10);
//      }
//
//      while (pwmValue>0){
//          --pwmValue;
//          __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,pwmValue);
//          __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,pwmValue);
//          __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,pwmValue);
//          HAL_Delay(10);
//      }
//
//      HAL_Delay(5000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//    if(htim==&htim1){
//        HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);
//        HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
//        HAL_GPIO_TogglePin(LED_B_GPIO_Port,LED_B_Pin);
//    }
//}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//    if (htim->Instance == htim6.Instance) {
//// 因为有很多个定时器，我们�?要区别一下是哪一个定时器触发了中�?
//// 这里是tim6触发�?
//        HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
//        MainControlLoop();
//        HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
//    }
//    if(htim->Instance == htim7.Instance) {
//// 这里是tim7触发�?
//        Count();
//        Remote::remote.Handle();
//    }
//}


//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
//    switch(GPIO_PIN){
//        case KEY_Pin:
//            HAL_GPIO_TogglePin(LED_B_GPIO_Port,LED_B_Pin);
//            HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
//            HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);
//            break;
//    }
//}


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//    if(huart==&huart1){
//        HAL_UART_Receive_IT(&huart1,RxData,5);
//        if(RxData[0]=='h'){
//            HAL_GPIO_TogglePin(LED_B_GPIO_Port,LED_B_Pin);
//            HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
//            HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);
//            RxData[0]=0;
//            HAL_Delay(100);
//        }
//    }
//}


//void buzzer_on(uint16_t psc,uint16_t pwm)
//{
//    htim4.Instance->PSC=psc;
//    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,pwm);
//}




//void canFilterInit(void) {
//    CAN_FilterTypeDef filter;
//    filter.FilterActivation = ENABLE;
//    filter.FilterMode = CAN_FILTERMODE_IDMASK;
//    filter.FilterScale = CAN_FILTERSCALE_32BIT;
//    filter.FilterIdHigh = 0x0000;
//    filter.FilterIdLow = 0x0000;
//    filter.FilterMaskIdHigh = 0x0000;
//    filter.FilterMaskIdLow = 0x0000;
//    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
//    filter.SlaveStartFilterBank = 14;
//    filter.FilterBank = 0;
//    HAL_CAN_ConfigFilter(&hcan1, &filter);
//    HAL_CAN_Start(&hcan1);
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//    filter.FilterBank = 14;
//    HAL_CAN_ConfigFilter(&hcan2, &filter);
//    HAL_CAN_Start(&hcan2);
//    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
//}




//void BMI088_read(fp32 gyro[3], fp32 accel[3], fp32 *temperate)
//{
//    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
//    int16_t bmi088_raw_temp;
//    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
//    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
//    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
//    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
//    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
//    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
//    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
//    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
//    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
//    {
//        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
//        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
//        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
//        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
//        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
//        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
//    }
//
//    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
//    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
//    if (bmi088_raw_temp > 1023)
//    {
//        bmi088_raw_temp -= 2048;
//    }
//    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
//}
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
