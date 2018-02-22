/*
 * vTaskESP.h
 *
 *  Created on: Jan 22, 2018
 *      Author: Dr. Saldon
 */

#ifndef VTASKESP_H_
#define VTASKESP_H_

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "common_freertos.h"					// Common convinence functions for FreeRTOS
#include "UART_freertos.h"		// USART convinence functions for FreeRTOS
#include "esp8266_simple.h"

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
int16_t temp = 0;
uint32_t press = 0;
float alt = 0;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void vESP8266Task(void *pvParameters);

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

void vESP8266Task(void *pvParameters)
{
	uint8_t mode_ap = 0;			// AP
	uint8_t server_active = 0;		// Server

//	xEspSemaphore = xSemaphoreCreateMutex();

	if(esp8266SetMode(ESP8266_MODE_AP))	{	// Configure ESP8266 as AP
		mode_ap = 1;
	}
	mavlink_enable = 1;

	do{
		if(!server_active){
			if(!mode_ap)
			{
				if(esp8266SetMode(ESP8266_MODE_AP))	{
					mode_ap = 1;
				}
			}
			if(mode_ap && !server_active){
				esp8266SetMux(1);
				if(esp8266ServerCreate(1500) != 0)	{
					server_active = 1;
					blinkParam.period = 200;
					uart_send_str_ln(USART1, "Server is ready for incoming connections");
				}
			}
		}
		vTaskDelay(1);
	}while(1);
	vTaskDelete(NULL);
}

#endif /* VTASKESP_H_ */
