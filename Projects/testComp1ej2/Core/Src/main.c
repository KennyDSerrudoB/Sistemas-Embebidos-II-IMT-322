/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DB_TIME 250
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
unsigned long lastState = 0;
uint8_t mainCounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
char* utoa(uint32_t value, char* buffer);
void printUtoa(uint32_t value);
void printMessages(uint8_t Index, int Fibo);
char* itoa(int value, char* buffer);
void printInt(int value);
int obtener_fibonacci(int indice);

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printMessages(mainCounter, obtener_fibonacci(mainCounter));
  if ((obtener_fibonacci(mainCounter)) == 0){
	   HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET); // Bit 0
	   HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET); // Bit 1
	   HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET); // Bit 2
  	  	  }
  while (1)
  {
	  if ((obtener_fibonacci(mainCounter)) == 0){
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
	  	  }
	  else if ((obtener_fibonacci(mainCounter) % 2)  == 0 && (obtener_fibonacci(mainCounter) < 10)){
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
	  }
	  else if ((obtener_fibonacci(mainCounter) % 2) != 0 && (obtener_fibonacci(mainCounter) < 10)){
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
	 }
	  else if ((obtener_fibonacci(mainCounter)) >= 10 ){
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN1_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_5) {
		if(HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == 0){
			if ((HAL_GetTick() - lastState) > DB_TIME){
				if (mainCounter>=0 && mainCounter <7){
					mainCounter++;
					printMessages(mainCounter, obtener_fibonacci(mainCounter));
				}
				else {
					mainCounter=0;
					printMessages(mainCounter, obtener_fibonacci(mainCounter));
				}
			}
		  	lastState=HAL_GetTick();
		}
    }
}

int obtener_fibonacci(int indice) {
	if (indice == 0) return 0;
	if (indice == 1) return 1;
	int a = 0;
	int b = 1;
	int resultado = 0;
	for (int i = 2; i <= indice; i++) {
			resultado = a + b;
			a = b;
			b = resultado;
	}
	return resultado;
}


char* utoa(uint32_t value, char* buffer) {
    char* ptr = buffer;
    char* ptr1 = buffer;
    char tmp_char;
    uint32_t tmp_value;
    do {
        tmp_value = value;
        value /= 10;
        *ptr++ = '0' + (tmp_value - (value * 10));
    } while (value != 0);

    *ptr-- = '\0';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr-- = *ptr1;
        *ptr1++ = tmp_char;
    }

    return buffer;
}
void printUtoa(uint32_t value) {
	static char buffer[12];
    utoa(value, buffer);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void printMessages(uint8_t Index, int Fibo){
	uint8_t bufferValue = 8;
	uint8_t bufferFactorial = 11;
	HAL_UART_Transmit(&huart2, (uint8_t*)"Indice: ", bufferValue, HAL_MAX_DELAY);
	printUtoa(Index);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\t", 1, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)"Fibonacci: ", bufferFactorial, HAL_MAX_DELAY);
	printInt(Fibo);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, HAL_MAX_DELAY);
}


char* itoa(int value, char* buffer) {
    char* ptr = buffer;
    char* ptr1 = buffer;
    char tmp_char;
    int tmp_value;
    bool isNegative = false;

    if (value < 0) {
        isNegative = true;
        value = -value;
    }

    // Conversión a decimal fijo (base 10)
    do {
        tmp_value = value;
        value /= 10;
        *ptr++ = '0' + (tmp_value - (value * 10));
    } while (value != 0);

    if (isNegative) {
        *ptr++ = '-';
    }

    *ptr-- = '\0';

    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr-- = *ptr1;
        *ptr1++ = tmp_char;
    }

    return buffer;
}

void printInt(int value) {

	static char buffer[22];  // Suficiente para -2147483648 + '\0'
	itoa(value, buffer);     // Convertir número a decimal con signo
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
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
