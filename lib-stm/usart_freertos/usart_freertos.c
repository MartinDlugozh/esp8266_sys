/*
 * usart_freertos.c
 *
 *  Created on: Feb 12, 2018
 *      Author: Dr. Saldon
 */

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/


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

#include "usart_freertos.h"

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/
#if(USE_FTOA == 1)
char * ftoa(double f, char * buf, int precision)
{
	char * ptr = buf;
	char * p = ptr;
	char * p1;
	char c;
	long intPart;

	// check precision bounds
	if (precision > MAX_PRECISION)
		precision = MAX_PRECISION;

	// sign stuff
	if (f < 0)
	{
		f = -f;
		*ptr++ = '-';
	}

	if (precision < 0)  // negative precision == automatic precision guess
	{
		if (f < 1.0) precision = 6;
		else if (f < 10.0) precision = 5;
		else if (f < 100.0) precision = 4;
		else if (f < 1000.0) precision = 3;
		else if (f < 10000.0) precision = 2;
		else if (f < 100000.0) precision = 1;
		else precision = 0;
	}

	// round value according the precision
	if (precision)
		f += rounders[precision];

	// integer part...
	intPart = f;
	f -= intPart;

	if (!intPart)
		*ptr++ = '0';
	else
	{
		// save start pointer
		p = ptr;

		// convert (reverse order)
		while (intPart)
		{
			*p++ = '0' + intPart % 10;
			intPart /= 10;
		}

		// save end pos
		p1 = p;

		// reverse result
		while (p > ptr)
		{
			c = *--p;
			*p = *ptr;
			*ptr++ = c;
		}

		// restore end pos
		ptr = p1;
	}

	// decimal part
	if (precision)
	{
		// place decimal point
		*ptr++ = '.';

		// convert
		while (precision--)
		{
			f *= 10.0;
			c = f;
			*ptr++ = '0' + c;
			f -= c;
		}
	}

	// terminating zero
	*ptr = 0;

	return ptr;
}
#endif

void uart_send_byte(USART_TypeDef* USARTx, uint8_t data)  {
	while(!(USARTx->SR & USART_SR_TXE));
	USARTx->DR=data;
	while (!(USARTx->SR & USART_SR_TC));
}

void uart_send_str(USART_TypeDef* USARTx, char* string) {
	  while(*string != 0)
	  {
		  uart_send_byte(USARTx, *string);
		  string++;
	  }
}

void uart_send_str_n(USART_TypeDef* USARTx, char* string, uint8_t n) {
	uint8_t ptr;
	ptr = 0;

	while((n - ptr) > 0)
	  {
		  uart_send_byte(USARTx, string[ptr]);
		  ptr++;
	  }
}

void uart_send_str_ln(USART_TypeDef* USARTx, char* string) {
	uart_send_str(USARTx, string);
	uart_send_byte(USARTx, '\r');
	uart_send_byte(USARTx, '\n');
}

void uart_send_num(USART_TypeDef* USARTx, uint32_t x)
{
	  char value[10]; //a temp array to hold results of conversion
	  int i = 0; //loop index

	  do
	  {
	    value[i++] = (char)(x % 10) + '0'; //convert integer to character
	    x /= 10;
	  } while(x);

	  while(i) //send data
	  {
		  uart_send_byte(USARTx, value[--i]);
	  }
}

void uart_send_float_str(USART_TypeDef* USARTx, float flt, int precision) {
	char buf[16];
	ftoa(flt, buf, precision);
	uart_send_str(USARTx, buf);
}

uint32_t uart_param2baudrate(uint16_t baudrate_param) {
	if(baudrate_param == PARAM_BAUD_9){
		return BAUD_9;}
	if(baudrate_param == PARAM_BAUD_38){
		return BAUD_38;}
	if(baudrate_param == PARAM_BAUD_57){
		return BAUD_57;}
	if(baudrate_param == PARAM_BAUD_115){
		return BAUD_115;}
	if(baudrate_param == PARAM_BAUD_921){
		return BAUD_921;}

	return 57600;
}

#if(USE_USART_1 == 1)
void initUSART1(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, 	ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, 	ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, 	ENABLE);

#if(USE_USART_1_DMA_TX == 1)
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_InitTypeDef DMA_InitStruct;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&usart1dmaBuffer[0];
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStruct.DMA_BufferSize = sizeof(usart1dmaBuffer);
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStruct);

	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
	NVIC_InitTypeDef nvic_dma;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);		/*!< 0 bits for pre-emption priority, 4 bits for subpriority */
	nvic_dma.NVIC_IRQChannel = DMA1_Channel4_IRQn;
	nvic_dma.NVIC_IRQChannelPreemptionPriority = configLIBRARY_LOWEST_INTERRUPT_PRIORITY;
	nvic_dma.NVIC_IRQChannelSubPriority = 0;
	nvic_dma.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_dma);
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	vSemaphoreCreateBinary(xUsart1TxDmaSemaphore);
	xSemaphoreGive(xUsart1TxDmaSemaphore);
#endif 	/* USE_USART_1_DMA_TX */

	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);							/*** Configure UART Rx GPIO ***/
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = PIN_UART1_RX;
	GPIO_Init(PORT_UART1, &gpio);

	gpio.GPIO_Mode = GPIO_Mode_AF_PP;				/*** Configure UART Tx GPIO ***/
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = PIN_UART1_TX;
	GPIO_Init(PORT_UART1, &gpio);

	USART_InitTypeDef usart;
	USART_StructInit(&usart);
	usart.USART_BaudRate            = UART1_BAUDRATE;
	usart.USART_WordLength          = USART_WordLength_8b;
	usart.USART_StopBits            = USART_StopBits_1;
	usart.USART_Parity              = USART_Parity_No ;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &usart);
	USART_Cmd(USART1, ENABLE);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);    	// Enable RXNE interrupt
	NVIC_InitTypeDef nvic;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);		/*!< 0 bits for pre-emption priority, 4 bits for subpriority */
	nvic.NVIC_IRQChannel = USART1_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = configLIBRARY_LOWEST_INTERRUPT_PRIORITY-1;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	NVIC_EnableIRQ(USART1_IRQn);    					// Enable USART1 global interrupt

	vSemaphoreCreateBinary(xUsart1RxInterruptSemaphore);
}

#if(USE_USART_1_DMA_TX == 1)
void DMA1_Channel4_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

    DMA_ClearITPendingBit(DMA1_IT_TC4);
    DMA_Cmd(DMA1_Channel4, DISABLE);

    xSemaphoreGiveFromISR(xUsart1TxDmaSemaphore, &xHigherPriorityTaskWoken);
   	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
#endif
#endif

#if(USE_USART_2 == 1)
void initUSART2(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, 	ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, 	ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, 	ENABLE);

#if(USE_USART_2_DMA_TX == 1)
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_InitTypeDef DMA_InitStruct;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&usart2dmaBuffer[0];
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStruct.DMA_BufferSize = sizeof(usart2dmaBuffer);
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel7, &DMA_InitStruct);

	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);

	DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);
#endif 	/* USE_USART_1_DMA_TX */

	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);							/*** Configure UART Rx GPIO ***/
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = PIN_UART2_RX;
	GPIO_Init(PORT_UART2, &gpio);

	gpio.GPIO_Mode = GPIO_Mode_AF_PP;				/*** Configure UART Tx GPIO ***/
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = PIN_UART2_TX;
	GPIO_Init(PORT_UART2, &gpio);

	USART_InitTypeDef usart;
	USART_StructInit(&usart);
	usart.USART_BaudRate            = UART2_BAUDRATE;
	usart.USART_WordLength          = USART_WordLength_8b;
	usart.USART_StopBits            = USART_StopBits_1;
	usart.USART_Parity              = USART_Parity_No ;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &usart);
	USART_Cmd(USART2, ENABLE);

	NVIC_InitTypeDef nvic;
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);    	// Enable RXNE interrupt
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);		/*!< 0 bits for pre-emption priority, 4 bits for subpriority */
	nvic.NVIC_IRQChannel = USART2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = configLIBRARY_LOWEST_INTERRUPT_PRIORITY;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	NVIC_EnableIRQ(USART2_IRQn);    					// Enable USART1 global interrupt

	vSemaphoreCreateBinary(xUsart2RxInterruptSemaphore);
}
#endif
