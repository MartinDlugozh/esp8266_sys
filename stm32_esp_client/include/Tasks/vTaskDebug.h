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

// Выводить время (раз в сек)
#define DEB_COMM	 		0
// Выводить инфо о каджом принятом сообщении (источник, ид)
#define DEB_RECEIVED_MSG 	0
// Выводить инфо о принятых данных сенсоров
#define DEB_REC_SENSOR_SIM 	0
// Выводить инфо об обмене файлами
#define DEB_FILE_OP 		1

#define DEB_HIGHER_WMARK 	0

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "usart_freertos.h"		// USART convinence functions for FreeRTOS
#include "xprintf.h"
#include "blue_pill/blue_pill.h"

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
TaskHandle_t xTaskHandleDebug;

UBaseType_t uxHighWaterMark[9];
#define HIGH_WATERMARK_BLINKER 		0
#define HIGH_WATERMARK_BMP180S 		1
#define HIGH_WATERMARK_BUTTONC 		2
#define HIGH_WATERMARK_ESP8266 		3
#define HIGH_WATERMARK_MAXAMBI 		4
#define HIGH_WATERMARK_SD_CARD 		5
#define HIGH_WATERMARK_SHT11 		6
#define HIGH_WATERMARK_DEBUGG 		7

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
//		DEBUG_printf("TSB: %lu\n", millis()); // Time Since Boot
#if(DEB_COMM == 1) 			// СОВСЕМ НЕ ОПТИМАЛЬНО!
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
			ftoa(station.BMP180_data.temperature, buf, 3);
			uart_send_str(USART1, buf);

			uart_send_str(USART1, ", BMP_PRS: ");
			uart_send_num(USART1, station.BMP180_data.pressure);

			uart_send_str_ln(USART1, "");
#endif

#if(DEB_HIGHER_WMARK == 1)
			DEBUG_printf("TSB: %lu, "
						"WM0: %d, "
						"WM1: %d, "
						"WM2: %d, "
						"WM3: %d, "
						"WM4: %d, "
						"WM5: %d, "
						"WM6: %d, "
						"WM7: %d\n",
						millis(),
						uxHighWaterMark[0],
						uxHighWaterMark[1],
						uxHighWaterMark[2],
						uxHighWaterMark[3],
						uxHighWaterMark[4],
						uxHighWaterMark[5],
						uxHighWaterMark[6],
						uxHighWaterMark[7]); // Time Since Boot
#endif

//			xSemaphoreGive(xUsart1RxInterruptSemaphore);
//		}

		vTaskDelayUntil(&pxPreviousWakeTime, USART_DEBUG_PERIOD);
		uxHighWaterMark[HIGH_WATERMARK_DEBUGG] = uxTaskGetStackHighWaterMark(NULL);
	}while(1);
	vTaskDelete(NULL);
}

#endif /* VTASKDEBUG_H_ */
