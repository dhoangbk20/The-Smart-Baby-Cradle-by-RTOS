/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "i2c-lcd.h"
#include "DHT.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

osThreadId LCDHandle;
osThreadId DHTHandle;
osThreadId ServoMP3Handle;
osThreadId SoundHandle;
osMessageQId queue_temperatureHandle;
osMessageQId queue_humidityHandle;
osMessageQId queue_soundHandle;
osMessageQId queue_servoHandle;
/* USER CODE BEGIN PV */
DHT_DataTypedef DHT11_Data;
char cc1[] = {0x06,0x09,0x09,0x06,0x00,0x00,0x00,0x00};
uint8_t esp[6];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
void LCD_Task(void const * argument);
void DHT_Task(void const * argument);
void ServoMP3_Task(void const * argument);
void Sound_Task(void const * argument);

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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of queue_temperature */
  osMessageQDef(queue_temperature, 10, uint32_t);
  queue_temperatureHandle = osMessageCreate(osMessageQ(queue_temperature), NULL);

  /* definition and creation of queue_humidity */
  osMessageQDef(queue_humidity, 10, uint32_t);
  queue_humidityHandle = osMessageCreate(osMessageQ(queue_humidity), NULL);

  /* definition and creation of queue_sound */
  osMessageQDef(queue_sound, 1, uint32_t);
  queue_soundHandle = osMessageCreate(osMessageQ(queue_sound), NULL);

  /* definition and creation of queue_servo */
  osMessageQDef(queue_servo, 10, uint32_t);
  queue_servoHandle = osMessageCreate(osMessageQ(queue_servo), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LCD */
  osThreadDef(LCD, LCD_Task, osPriorityBelowNormal, 0, 128);
  LCDHandle = osThreadCreate(osThread(LCD), NULL);

  /* definition and creation of DHT */
  osThreadDef(DHT, DHT_Task, osPriorityNormal, 0, 128);
  DHTHandle = osThreadCreate(osThread(DHT), NULL);

  /* definition and creation of ServoMP3 */
  osThreadDef(ServoMP3, ServoMP3_Task, osPriorityBelowNormal, 0, 64);
  ServoMP3Handle = osThreadCreate(osThread(ServoMP3), NULL);

  /* definition and creation of Sound */
  osThreadDef(Sound, Sound_Task, osPriorityNormal, 0, 64);
  SoundHandle = osThreadCreate(osThread(Sound), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 959;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DHT11_Pin|MP3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DHT11_Pin MP3_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin|MP3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_LCD_Task */
/**
  * @brief  Function implementing the LCD thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LCD_Task */
void LCD_Task(void const * argument)
{
  /* USER CODE BEGIN 5 */
	char tempVal[10];
	char humiVal[10];
	char souVal[10];

	osEvent evt;

	uint32_t temperature, humidity,sound;

	lcd_init();
	lcd_send_cmd(0x40);
	for (int i=0;i<8;i++)
	{
		lcd_send_data(cc1[i]);
	}

	lcd_goto_XY(1,0);
	lcd_send_string("DA:  %RH");

	lcd_goto_XY(2,0);
	lcd_send_string("T:    C");

	lcd_goto_XY(2,5);
	lcd_send_data(0);

	lcd_goto_XY(1,9);
	lcd_send_string("AT:");

	osDelay(10);
  /* Infinite loop */
  for(;;)
  {
	  	  evt = osMessageGet(queue_temperatureHandle, 100);

		  if(evt.status == osEventMessage)
		  {
			  temperature = evt.value.v;
			  sprintf(tempVal, "%ld", temperature);
			  //HAL_UART_Transmit(&huart3, (uint8_t *)tempVal, sizeof(tempVal), 1000);
		  }
		  evt = osMessageGet(queue_humidityHandle, 100);
		  if(evt.status == osEventMessage)
		  {
			  humidity = evt.value.v;
			  sprintf(humiVal, "%ld", humidity);
			 // HAL_UART_Transmit(&huart3, (uint8_t *)humiVal, sizeof(tempVal), 1000);
		  }
		  evt = osMessageGet(queue_soundHandle, 100);
		  if(evt.status == osEventMessage)
		  {
			  sound = evt.value.v;
			  sprintf(souVal, "%ld", sound);

		  }



		  esp[0] = tempVal[0];
		  esp[1] = tempVal[1];
		  esp[2] = ',';
		  esp[3] = humiVal[0];
		  esp[4] = humiVal[1];
		  esp[5] = '\n';
		  HAL_UART_Transmit(&huart3, esp, sizeof(esp), 1000);
		  lcd_goto_XY(2,3);
		  lcd_send_string(tempVal);
		  lcd_goto_XY(1,3);
		  lcd_send_string(humiVal);
		  lcd_goto_XY(1,12);
		  lcd_send_string(souVal);



		  osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_DHT_Task */
/**
* @brief Function implementing the DHT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DHT_Task */
void DHT_Task(void const * argument)
{
  /* USER CODE BEGIN DHT_Task */
	uint32_t Temperature;
	uint32_t Humidity;
  /* Infinite loop */
  for(;;)
  {
	  DHT_GetData(&DHT11_Data);
	  Temperature = DHT11_Data.Temperature;
	  Humidity = DHT11_Data.Humidity;
	  osMessagePut(queue_temperatureHandle, Temperature, 100);
	  osMessagePut(queue_humidityHandle, Humidity, 100);




	  osDelay(1000);
  }
  /* USER CODE END DHT_Task */
}

/* USER CODE BEGIN Header_ServoMP3_Task */
/**
* @brief Function implementing the ServoMP3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ServoMP3_Task */
void ServoMP3_Task(void const * argument)
{
  /* USER CODE BEGIN ServoMP3_Task */
	uint32_t SoundMP3;
	uint8_t sendBuffer[10] = {0x7E, 0xFF, 0x06, 0x0E, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xEF};
	osEvent evt;
  /* Infinite loop */
	for(;;)
	{
		  uint16_t checksum = 0;
		  evt = osMessageGet(queue_servoHandle, osWaitForever);
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		  if(evt.status == osEventMessage)
		  {
			  SoundMP3 = evt.value.v;
			  if(SoundMP3 > 300)
			  {
				   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, RESET);
				   HAL_Delay(50);
				   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, SET);

				   for(int i = 0;i<8;i++)
				   {
					   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,120);
					   HAL_Delay(1000);
					   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,30);
					   HAL_Delay(1000);
				   }


				   for (int i = 1; i < 7; i++)
				   {
					 checksum += sendBuffer[i];
				   }
				   checksum = -checksum;

				   // Thêm checksum vào buffer
				   sendBuffer[7] = (uint8_t)(checksum >> 8);
				   sendBuffer[8] = (uint8_t)(checksum & 0xFF);
				   HAL_UART_Transmit(&huart1, sendBuffer, sizeof(sendBuffer), HAL_MAX_DELAY);
				   HAL_Delay(5000);
			  }

		  }



		osDelay(1000);
    }
  /* USER CODE END ServoMP3_Task */
}

/* USER CODE BEGIN Header_Sound_Task */
/**
* @brief Function implementing the Sound thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Sound_Task */
void Sound_Task(void const * argument)
{
  /* USER CODE BEGIN Sound_Task */
	uint32_t amthanh;

  /* Infinite loop */
  for(;;)
  {
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  amthanh = HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1);

//

	  osMessagePut(queue_servoHandle, amthanh, osWaitForever);


	  osMessagePut(queue_soundHandle, amthanh, 100);
      osDelay(1000);
  }
  /* USER CODE END Sound_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
