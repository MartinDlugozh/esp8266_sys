/*
 * usart_freertos.h
 *
 *  Created on: Feb 12, 2018
 *      Author: Dr. Saldon
 */

#ifndef USART_FREERTOS_H_
#define USART_FREERTOS_H_

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
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

/** UART1 **/
#define UART1_BAUDRATE 					BAUD_115
#define PORT_UART1 						GPIOA
#define PIN_UART1_RX 					GPIO_Pin_10
#define PIN_UART1_TX 					GPIO_Pin_9

/** UART2 **/
#define UART2_BAUDRATE 					BAUD_115
#define PORT_UART2 						GPIOA
#define PIN_UART2_RX 					GPIO_Pin_3
#define PIN_UART2_TX 					GPIO_Pin_2

#define MAX_PRECISION	(10)

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
SemaphoreHandle_t xUsart1RxInterruptSemaphore;
SemaphoreHandle_t xUsart2RxInterruptSemaphore;

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

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
char * ftoa(double f, char * buf, int precision);
void uart_send_byte(USART_TypeDef* USARTx, uint8_t data);
void uart_send_str(USART_TypeDef* USARTx, char* string);
void uart_send_str_n(USART_TypeDef* USARTx, char* string, uint8_t n);
void uart_send_str_ln(USART_TypeDef* USARTx, char* string);
void uart_send_num(USART_TypeDef* USARTx, uint32_t x);
void uart_send_float_str(USART_TypeDef* USARTx, float flt, int precision);
uint32_t uart_param2baudrate(uint16_t baudrate_param);
void initUSART1(void);
void initUSART2(void);

#endif /* USART_FREERTOS_H_ */
