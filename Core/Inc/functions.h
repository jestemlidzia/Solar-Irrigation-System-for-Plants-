#ifndef __FUNCTIONS_H__
#define __FUNCTIONS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
#include <stdio.h>
#include "ring_buffer.h"
#include "mk_dht11.h"
#include "bh1750.h"

void watering();
void measurement();
bool is_button_pressed(void);
uint32_t GetADC_Value(uint32_t channel);

/* Auxiliary functions during UART communication.
   A circular buffer has been used. */
bool UART_PutChar(char c);
size_t UART_WriteData(const void *data, size_t dataSize);
size_t UART_WriteString(const char *string);
bool UART_GetChar(char *c);
size_t UART_ReadData(char *data, size_t maxSize);
void char_append(uint8_t value);

#endif /*__ FUNCTIONS_H__ */
