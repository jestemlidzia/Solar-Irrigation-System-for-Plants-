/* Includes ------------------------------------------------------------------*/
#include "functions.h"

extern uint16_t soilMoisureValue;
extern uint16_t waterLvl;
extern float BH1750_lux;
extern dht11_t dht;

// UART transmit buffer descriptor
extern RingBuffer UART_RingBuffer_Tx;
// UART transmit buffer memory pool
extern char RingBufferData_Tx[1024];
// UART receive buffer descriptor
extern RingBuffer UART_RingBuffer_Rx;
// UART receive buffer memory pool
extern char RingBufferData_Rx[1024];

extern char Response_Rx[1024];
extern uint8_t uart_rx_buffer;

extern bool measurement_status;

void watering(){
	/* Soil moisture and water level measurement. */
	soilMoisureValue = GetADC_Value(9);
	waterLvl = GetADC_Value(10);

	if(3200 < soilMoisureValue){
		if(1000 < waterLvl){
			printf("Watering!\n");
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(pompka_GPIO_Port, pompka_Pin, GPIO_PIN_RESET);
			HAL_Delay(5000);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(pompka_GPIO_Port, pompka_Pin, GPIO_PIN_SET);
		}
		else{
			printf("No enought water!\n");
		}
	}
	else{
		printf("Soil is perfectly hydrated!\n");
	}
}


void measurement(){
	readDHT11(&dht);
	soilMoisureValue = GetADC_Value(9);
	waterLvl = GetADC_Value(10);
	if(BH1750_OK == BH1750_ReadLight(&BH1750_lux))
	{
		//size = sprintf(buffer, "BH1750 Lux: %.2f\n\r", BH1750_lux);
		//HAL_UART_Transmit(&huart1, (uint8_t*)buffer, size, 100);
	}
	char toSend[256];
	uint16_t finalSize = sprintf(toSend, "%d,%d,%d,%.2f,%d,end\r\n", dht.humidty, dht.temperature,
			soilMoisureValue, BH1750_lux, waterLvl);
	HAL_UART_Transmit(&huart1, (uint8_t*)toSend, finalSize, 100);

}

bool is_button_pressed(void)
{
  if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_RESET) {
    return true;
  } else {
    return false;
  }
}

uint32_t GetADC_Value(uint32_t channel)
	{
	uint32_t g_ADCValue = 0;

	ADC_ChannelConfTypeDef sConfig;
	if(channel == 9){
		sConfig.Channel = ADC_CHANNEL_9;
	}
	else if(channel == 10){
		sConfig.Channel = ADC_CHANNEL_10;
	}

	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	//start adc
	HAL_ADC_Start(&hadc);

	if (HAL_ADC_PollForConversion(&hadc,1000) == HAL_OK){
		g_ADCValue = HAL_ADC_GetValue(&hadc);
	}
	HAL_ADC_Stop(&hadc);

	//remove from channel select
	sConfig.Rank = ADC_RANK_NONE;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK){}

	return (g_ADCValue);
}

int __io_putchar(int ch)
{
    if (ch == '\n') {
        uint8_t ch2 = '\r';
        HAL_UART_Transmit(&huart2, &ch2, 1, HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return 1;
}
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
			  measurement_status = true;
		  }
	  }

  }
  else {
	  RingBuffer_PutChar(&UART_RingBuffer_Rx, uart_rx_buffer);
  }

}
