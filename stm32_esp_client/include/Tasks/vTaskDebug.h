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
void vUSART_debug(void *pvParameters);

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

			uart_send_str(USART1, ", L1: ");
			uart_send_num(USART1, station.MAX44009_data.lux_ambilight_1);

			uart_send_str(USART1, ", L2: ");
			uart_send_num(USART1, station.MAX44009_data.lux_ambilight_2);

			uart_send_str(USART1, ", SHT_TMP: ");
			ftoa(station.SHT11_data.temperature, buf, 3);
			uart_send_str(USART1, buf);

			uart_send_str(USART1, ", SHT_HUM: ");
			ftoa(station.SHT11_data.humidity, buf, 3);
			uart_send_str(USART1, buf);

			uart_send_str(USART1, ", BMP_TMP: ");
			ftoa(station.BMP180_data.temterature, buf, 3);
			uart_send_str(USART1, buf);

			uart_send_str(USART1, ", BMP_PRS: ");
			uart_send_num(USART1, station.BMP180_data.pressure);

			uart_send_str_ln(USART1, "");

			xSemaphoreGive(xUsart1RxInterruptSemaphore);
		}

		vTaskDelayUntil(&pxPreviousWakeTime, USART_DEBUG_PERIOD);
	}while(1);
	vTaskDelete(NULL);
}

#endif /* VTASKDEBUG_H_ */
