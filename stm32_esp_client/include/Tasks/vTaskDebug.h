/*
 * vTaskDebug.h
 *
 *  Created on: Jan 22, 2018
 *      Author: Dr. Saldon
 */

#ifndef VTASKDEBUG_H_
#define VTASKDEBUG_H_

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define USART_DEBUG_PERIOD 			1000 		// [ms]

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "UART_freertos/UART_freertos.h"
#include "blue_pill/blue_pill.h"

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

/**
 * OS Task: USART debug task
 */
void vUSART_debug(void *pvParameters)
{
	TickType_t pxPreviousWakeTime;

	pxPreviousWakeTime = xTaskGetTickCount();
	do{
		if(xSemaphoreTake(xUsart1RxInterruptSemaphore, portMAX_DELAY) == pdTRUE){
			char buf[10];
			uart_send_str(USART1, "TIM: ");
			uart_send_num(USART1, xTaskGetTickCount());
			uart_send_str_ln(USART1, "");

			if(xSemaphoreTake(xMutex_BMP180_Data, 0) == pdTRUE){
				uart_send_str(USART1, " TMP: ");
				ftoa(BMP180_Data.Temperature, buf, 3);
				uart_send_str(USART1, buf);
				uart_send_str_ln(USART1, "");

				uart_send_str(USART1, " PRS: ");
				uart_send_num(USART1, BMP180_Data.Pressure);
				uart_send_str_ln(USART1, "");

				xSemaphoreGive(xMutex_BMP180_Data);
			}
			xSemaphoreGive(xUsart1RxInterruptSemaphore);
		}

		vTaskDelayUntil(&pxPreviousWakeTime, USART_DEBUG_PERIOD);
	}while(1);
	vTaskDelete(NULL);
}

#endif /* VTASKDEBUG_H_ */
