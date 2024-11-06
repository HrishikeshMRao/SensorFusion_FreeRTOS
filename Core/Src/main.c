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
#include <stdio.h>
#include <string.h>

#define IMU_ADDRESS 0x28 << 1 // Shifted for 7-bit address
#define MEM_ADDRESS 0x28      // Start address for IMU linear acceleration data
#define MEM_WRITE   0x3D      // Configuration register
#define WRITE_DATA  0x08      // Configuration data
#define BUFFER_SIZE      6          // Number of bytes to read for accelerometer data

// UART buffer for printing
int16_t LinACC_X, LinACC_Y, LinACC_Z;
uint8_t i2c_buffer[BUFFER_SIZE];      // Buffer for storing IMU data
char str1[256];
char str2[64];

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

osThreadId BNO055_taskHandle;
osThreadId IR_taskHandle;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
void StartBNO(void const * argument);
void StartIR(void const * argument);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  for (volatile int i = 0; i < 2000000; i++); //delay to allow the status register of imu to setup after power
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
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of BNO055_task */
  osThreadDef(BNO055_task, StartBNO, osPriorityNormal, 0, 128);
  BNO055_taskHandle = osThreadCreate(osThread(BNO055_task), NULL);

  /* definition and creation of IR_task */
  osThreadDef(IR_task, StartIR, osPriorityIdle, 0, 128);
  IR_taskHandle = osThreadCreate(osThread(IR_task), NULL);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_3;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

void i2c_write(uint8_t device_address, uint8_t memory, uint8_t data) {
    // Send a single byte (data) to a specific register (memory) on the IMU
    uint8_t payload[2] = {memory, data};
    HAL_I2C_Master_Transmit(&hi2c2, device_address, payload, 2, HAL_MAX_DELAY);
}

void i2c_read(uint8_t device_address, uint8_t memory, uint8_t* buffer, uint16_t length) {
    // Write the register address, then read the data
    HAL_I2C_Master_Transmit(&hi2c2, device_address, &memory, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c2, device_address, buffer, length, HAL_MAX_DELAY);
}

void calc(uint8_t *data, int16_t *LinACCp_X, int16_t *LinACCp_Y, int16_t *LinACCp_Z) {
    *LinACCp_X = (((int16_t)data[1]) << 8) | ((int16_t)data[0]);
    *LinACCp_X /= 100;

    *LinACCp_Y = (((int16_t)data[3]) << 8) | ((int16_t)data[2]);
    *LinACCp_Y /= 100;

    *LinACCp_Z = (((int16_t)data[5]) << 8) | ((int16_t)data[4]);
    *LinACCp_Z /= 100;
}

void print() {
	sprintf(str1, "%d", LinACC_X);
	strcat(str1, " , ");

	sprintf(str2, "%d", LinACC_Y);
	strcat(str1,str2);
	strcat(str1, " , ");

	sprintf(str2, "%d", LinACC_Z);
	strcat(str1,str2);
	strcat(str1, "\n");

	HAL_UART_Transmit(&huart1, (uint8_t*)str1, strlen(str1), HAL_MAX_DELAY);
}

/* USER CODE BEGIN Header_StartBNO */
/**
  * @brief  Function implementing the BNO055_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBNO */
void StartBNO(void const * argument)
{
    // Write configuration to the IMU (e.g., set to accelerometer mode)
    i2c_write(IMU_ADDRESS, MEM_WRITE, WRITE_DATA);

    for (;;) {
        // 1. Read 6 bytes of linear acceleration data starting from MEM_ADDRESS
        i2c_read(IMU_ADDRESS, MEM_ADDRESS, i2c_buffer, BUFFER_SIZE);

        // 2. Calculate linear acceleration from received data
        calc(i2c_buffer, &LinACC_X, &LinACC_Y, &LinACC_Z);

        // 3. Print the linear acceleration data via UART
        print();

        osDelay(100); // Adjust delay as needed
    }
}

/* USER CODE BEGIN Header_StartIR */
/**
* @brief Function implementing the IR_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIR */
void StartIR(void const * argument)
{
  /* USER CODE BEGIN StartIR */
  // Initialize necessary peripherals
  char msg[50];            // Buffer to hold UART messages
  uint16_t adcValue = 0;
  const uint16_t threshold = 300;  // Define an appropriate threshold based on sensor readings
  for(;;)
  {
	 // Start ADC conversion
	 HAL_ADC_Start(&hadc1);
	 // Wait for the conversion to finish
	 HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	 // Get the converted value
	 adcValue = HAL_ADC_GetValue(&hadc1);
	 // Check if the value exceeds the threshold
	 if (adcValue < threshold)
	 {
	   // If the value is too high, send a "Too close alert" message
	   snprintf(msg, sizeof(msg), "Distance Alert: Too Close! ADC: %u\r\n", adcValue);
	   HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	 }
	 else
	 {
	   // If the value is below the threshold, send a "Keep moving" message
	   snprintf(msg, sizeof(msg), "Distance OK: Keep Moving. ADC: %u\r\n", adcValue);
	   HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	 }
	 HAL_ADC_Stop(&hadc1); // stop adc
	 // Add a delay before the next reading
	 osDelay(100);  // Adjust delay as needed for your application
  }
  /* USER CODE END StartIR */
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
