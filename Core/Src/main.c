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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ring_buffer.h"
#include "Uart.h"
#include "mk_dht11.h"
#include "bh1750.h"

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
float BH1750_lux;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
dht11_t dht;
static char message[] = "Hello World!\r\n";
uint16_t readValue;

int __io_putchar(int ch)
{
    if (ch == '\n') {
        uint8_t ch2 = '\r';
        HAL_UART_Transmit(&huart2, &ch2, 1, HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return 1;
}

bool is_button_pressed(void)
{
  if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_RESET) {
    return true;
  } else {
    return false;
  }
}

void measurement(){
	//char sendData[128];
	readDHT11(&dht);
	//uint16_t len = sprintf(sendData, "Hum: %d Temp: %d\n\r", dht.humidty, dht.temperature);
	//HAL_UART_Transmit(&huart1, (uint8_t*)sendData, len, 100);

	HAL_ADC_PollForConversion(&hadc,1000);
	readValue = HAL_ADC_GetValue(&hadc);
	//char sendData2[40];
	//uint16_t len2 = sprintf(sendData2, "Soil: %d\n\r", readValue);
	//HAL_UART_Transmit(&huart1, (uint8_t*)sendData2, len2, 100);

	//char buffer[40];
	//uint8_t size;
	if(BH1750_OK == BH1750_ReadLight(&BH1750_lux))
	{
		//size = sprintf(buffer, "BH1750 Lux: %.2f\n\r", BH1750_lux);
		//HAL_UART_Transmit(&huart1, (uint8_t*)buffer, size, 100);
	}

	char toSend[256];
	uint16_t finalSize = sprintf(toSend, "%d,%d,%d,%.2f,end", dht.humidty, dht.temperature, readValue, BH1750_lux);
	HAL_UART_Transmit(&huart1, (uint8_t*)toSend, finalSize, 100);

}

// UART transmit buffer descriptor
static RingBuffer UART_RingBuffer_Tx;
// UART transmit buffer memory pool
static char RingBufferData_Tx[1024];

// UART receive buffer descriptor
static RingBuffer UART_RingBuffer_Rx;
// UART receive buffer memory pool
static char RingBufferData_Rx[1024];

static char Response_Rx[1024];
uint8_t uart_rx_buffer;

bool UART_PutChar(char c){
	if(RingBuffer_PutChar(&UART_RingBuffer_Tx, c)){

		char temp;
		if(RingBuffer_GetChar(&UART_RingBuffer_Tx, &temp)){
			HAL_UART_Transmit(&huart1, &temp, 1, 100);
			//HAL_UART_Transmit_IT(&huart2, &temp, 1);
		}
		else return false;

		return true;
	}
	return false;
}

size_t UART_WriteData(const void *data, size_t dataSize){
	size_t i = 0;
	char const* d =(char const*)data;
	while(dataSize>0){
		i++;
		UART_PutChar(*d++);
		dataSize--;
	}
	return i;
}


size_t UART_WriteString(const char *string){
	return UART_WriteData(string, strlen(string)+2);
}

bool UART_GetChar(char *c){
	bool temp = RingBuffer_GetChar(&UART_RingBuffer_Rx, c);
	return temp;
}


size_t UART_ReadData(char *data, size_t maxSize){

	size_t i = 0;

	while(maxSize>i){

		if(!UART_GetChar(data + i)){
			return i;
		}
		i++;

	}
	return i;
}
bool pomiar = false;
void char_append(uint8_t value)
{
  if (value == '\r' || value == '\n') { //end of line

	  if(!RingBuffer_IsEmpty(&UART_RingBuffer_Rx)){
		  size_t responseSize = UART_ReadData(&Response_Rx, 10);
		  Response_Rx[responseSize] = '\0';
		  if (strcmp(Response_Rx, "on") == 0) {
			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		  }
		  else if (strcmp(Response_Rx, "off") == 0) {
			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		  }
		  else if (strcmp(Response_Rx, "aa") == 0) {
			  pomiar = true;
		  }
	  }

  }
  else {
	  RingBuffer_PutChar(&UART_RingBuffer_Rx, uart_rx_buffer);
  }

}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1) {

  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1) {

	  char_append(uart_rx_buffer);

    HAL_UART_Receive_IT(&huart1, &uart_rx_buffer, 1);
  }
}


/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == dht11.pin)
  {
    DHT_pinChangeCallBack(&dht11);
  }
}*/

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  RingBuffer_Init(&UART_RingBuffer_Tx, &RingBufferData_Tx, 1024);
  RingBuffer_Init(&UART_RingBuffer_Rx, &RingBufferData_Rx, 1024);

  init_dht11(&dht, &htim2, sensor_GPIO_Port, sensor_Pin);
  readDHT11(&dht);

  HAL_ADC_Start(&hadc);

  BH1750_Init(&hi2c1);
  BH1750_SetMode(CONTINUOUS_HIGH_RES_MODE_2);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	UART_WriteString(message);

  HAL_UART_Receive_IT(&huart1, &uart_rx_buffer, 1);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 /* if(is_button_pressed()){
		  HAL_Delay(200);
		  UART_WriteString(message);
		  HAL_Delay(200);
	  }*/
	  //HAL_Delay(5000);
	  if(pomiar){
		  measurement();
		  pomiar = false;
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
