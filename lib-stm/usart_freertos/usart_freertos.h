/*
 * usart_freertos.h
 *
 *  Created on: Feb 12, 2018
 *      Author: Dr. Saldon
 */

#ifndef USART_FREERTOS_H_
#define USART_FREERTOS_H_

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
// General STM32 includes
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define _USART_PORT1					1
#define _USART_PORT2 					2

// Configuration
#define USE_USART_1 					1
#define USE_USART_2 					1

#define USE_USART_1_DMA_TX				1
#define USE_USART_2_DMA_TX				0

#define USART_1_DMA_TX_BUF_SIZE 		128
#define USART_2_DMA_TX_BUF_SIZE 		128

#define USE_AS_DEBUG_PORT 				_USART_PORT1
#define USE_FTOA						1

// Baudrates
#define BAUD_9							9600
#define BAUD_38							38400
#define BAUD_57							57600
#define BAUD_115						115200
#define BAUD_921 						921600

#define PARAM_BAUD_9					9
#define PARAM_BAUD_38					38
#define PARAM_BAUD_57					57
#define PARAM_BAUD_115					115
#define PARAM_BAUD_921 					921

#if(USE_USART_1 == 1)
#define UART1_BAUDRATE 					BAUD_115
#define PORT_UART1 						GPIOA
#define PIN_UART1_RX 					GPIO_Pin_10
#define PIN_UART1_TX 					GPIO_Pin_9

SemaphoreHandle_t 						xUsart1RxInterruptSemaphore;

#if(USE_USART_1_DMA_TX == 1)
uint8_t usart1dmaBuffer[USART_1_DMA_TX_BUF_SIZE];
uint8_t *usart1dmaBuffPtr;
SemaphoreHandle_t 						xUsart1TxDmaSemaphore;
#endif 	/* USE_USART_1_DMA_TX */
#endif 	/* USE_USART_1 */

#if(USE_USART_2 == 1)
#define UART2_BAUDRATE 					BAUD_115
#define PORT_UART2 						GPIOA
#define PIN_UART2_RX 					GPIO_Pin_3
#define PIN_UART2_TX 					GPIO_Pin_2

SemaphoreHandle_t 						xUsart2RxInterruptSemaphore;

#if(USE_USART_2_DMA_TX == 1)
uint8_t usart2dmaBuffer[USART_2_DMA_TX_BUF_SIZE];
uint8_t *usart2dmaBuffPtr;
#endif 	/* USE_USART_2_DMA_TX */
#endif 	/* USE_USART_2 */

#define MAX_PRECISION					(10)

#if(USE_AS_DEBUG_PORT == _USART_PORT1)
#define XPRINTF_USART					USART1
#if(USE_USART_1_DMA_TX == 1)
// Sending strings over USART1 (as debug port) using OS semaphore (thred-safe method)
#define DEBUG_puts(string) {	if(xSemaphoreTake(xUsart1TxDmaSemaphore, portMAX_DELAY) == pdTRUE){ \
							if(xSemaphoreTake(xUsart1RxInterruptSemaphore, portMAX_DELAY) == pdTRUE)\
							{xputs(string); \
							xSemaphoreGive(xUsart1RxInterruptSemaphore);} }}
#define DEBUG_printf(fmt, ...) {	if(xSemaphoreTake(xUsart1TxDmaSemaphore, portMAX_DELAY) == pdTRUE){	\
							if(xSemaphoreTake(xUsart1RxInterruptSemaphore, portMAX_DELAY) == pdTRUE)\
							{xprintf(fmt, __VA_ARGS__); \
							xSemaphoreGive(xUsart1RxInterruptSemaphore);} }}
#define DEBUG_put_dump(buff, addr, len, width) {	if(xSemaphoreTake(xUsart1TxDmaSemaphore, portMAX_DELAY) == pdTRUE){ \
							if(xSemaphoreTake(xUsart1RxInterruptSemaphore, portMAX_DELAY) == pdTRUE)\
							{put_dump(buff, addr, len, width); \
							xSemaphoreGive(xUsart1RxInterruptSemaphore);} }}
#else
#define DEBUG_puts(string) { if(xSemaphoreTake(xUsart1RxInterruptSemaphore, portMAX_DELAY) == pdTRUE)\
							{xputs(string); \
							xSemaphoreGive(xUsart1RxInterruptSemaphore);} }
#define DEBUG_printf(fmt, ...) { if(xSemaphoreTake(xUsart1RxInterruptSemaphore, portMAX_DELAY) == pdTRUE)\
							{xprintf(fmt, __VA_ARGS__); \
							xSemaphoreGive(xUsart1RxInterruptSemaphore);} }
#define DEBUG_put_dump(buff, addr, len, width) { if(xSemaphoreTake(xUsart1RxInterruptSemaphore, portMAX_DELAY) == pdTRUE)\
							{put_dump(buff, addr, len, width); \
							xSemaphoreGive(xUsart1RxInterruptSemaphore);} }
#endif
#elif(USE_AS_DEBUG_PORT == _USART_PORT2)
#define XPRINTF_USART					USART2
// Sending strings over USART2 (as debug port) using OS semaphore (thred-safe method)
#define DEBUG_puts(string) {	if(xSemaphoreTake(xUsart2RxInterruptSemaphore, portMAX_DELAY) == pdTRUE)\
							{xputs(string); \
							xSemaphoreGive(xUsart2RxInterruptSemaphore);} }
#define DEBUG_printf(fmt, ...) {	if(xSemaphoreTake(xUsart2RxInterruptSemaphore, portMAX_DELAY) == pdTRUE)\
							{xprintf(fmt, __VA_ARGS__); \
							xSemaphoreGive(xUsart1RxInterruptSemaphore);} }
#define DEBUG_put_dump(buff, addr, len, width) {	if(xSemaphoreTake(xUsart2RxInterruptSemaphore, portMAX_DELAY) == pdTRUE)\
							{put_dump(buff, addr, len, width); \
							xSemaphoreGive(xUsart2RxInterruptSemaphore);} }
#endif

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/

#if(USE_FTOA == 1)
static const double rounders[MAX_PRECISION + 1] =
{
	0.5,				// 0
	0.05,				// 1
	0.005,				// 2
	0.0005,				// 3
	0.00005,			// 4
	0.000005,			// 5
	0.0000005,			// 6
	0.00000005,			// 7
	0.000000005,		// 8
	0.0000000005,		// 9
	0.00000000005		// 10
};
#endif

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
#if(USE_FTOA == 1)
char * ftoa(double f, char * buf, int precision);
#endif
void uart_send_byte(USART_TypeDef* USARTx, uint8_t data);
void uart_send_str(USART_TypeDef* USARTx, char* string);
void uart_send_str_n(USART_TypeDef* USARTx, char* string, uint8_t n);
void uart_send_str_ln(USART_TypeDef* USARTx, char* string);
void uart_send_num(USART_TypeDef* USARTx, uint32_t x);
void uart_send_float_str(USART_TypeDef* USARTx, float flt, int precision);
uint32_t uart_param2baudrate(uint16_t baudrate_param);
#if(USE_USART_1 == 1)
void initUSART1(void);
#endif
#if(USE_USART_2 == 1)
void initUSART2(void);
#endif

#endif /* USART_FREERTOS_H_ */
