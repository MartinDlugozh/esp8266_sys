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

/* Wifi network settings, replace with your settings */
#define WIFINAME            "MY_ESP8266"
#define WIFIPASS            "1234567890"
#define SERVER_ADDRESS 		"192.168.4.1"
#define SERVER_PORT			"1500"
#define SERVER_CONNECTION 	"1"

#define CONNECTION_LOSS_TIMEOUT 10000

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


/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void vESP8266Task(void *pvParameters);

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

void vESP8266Task(void *pvParameters)
{
	TickType_t pxPreviousWakeTime;
	uint16_t period = 250;

	uint8_t wifi_connection = 0;
	uint8_t server_connection = 0;
	uint8_t reconnection_counter = 0;

	pxPreviousWakeTime = xTaskGetTickCount();
	do{
//		if((millis() - station.SERVER_data.last_heartbeat_ms) >= CONNECTION_LOSS_TIMEOUT)
//		{
//			server_connection = 0;
//			wifi_connection = 0;
//		}

		if(!server_connection)
		{
			if(!wifi_connection)
			{
				if(esp8266Connect(WIFINAME, WIFIPASS) != 0)
				{
					wifi_connection = 1;
				}else{
				}
			}
			if(wifi_connection && !server_connection)
			{
				esp8266SetMux(1);
				if(esp8266TcpConnect(SERVER_ADDRESS, SERVER_PORT, SERVER_CONNECTION) != 0)
				{
					server_connection = 1;
					period = 1000;
					blinkParam.period = 500;
				}else{
					wifi_connection = 0;
				}
			}
		}

		// Send collected data to the srver
		if(server_connection == TRUE)
		{
			if(mavlink_enable == 1){
				heartbeat_send(1, &out_msg);
				systime_send(1, &out_msg, millis());

				mavlink_reset_message(&out_msg);
				mavlink_msg_eco_bmp180_pack(SYS_ID_MY, COM_ID_MY, &out_msg,
						station.BMP180_data.temterature,
						station.BMP180_data.altitude,
						station.BMP180_data.pressure);
				mavlink_send_message_tcp(1, &out_msg);

				mavlink_reset_message(&out_msg);
				mavlink_msg_eco_max44009_pack(SYS_ID_MY, COM_ID_MY, &out_msg,
						station.MAX44009_data.lux_ambilight_1,
						station.MAX44009_data.lux_ambilight_2);
				mavlink_send_message_tcp(1, &out_msg);

				mavlink_reset_message(&out_msg);
				mavlink_msg_eco_sht11_pack(SYS_ID_MY, COM_ID_MY, &out_msg,
						station.SHT11_data.temperature,
						station.SHT11_data.humidity,
						station.SHT11_data.dewpoint);
				mavlink_send_message_tcp(1, &out_msg);
			}

			// TODO: maybe, it can be better to use heartbeat for connection diagnostic
			reconnection_counter++;
			if(reconnection_counter >= 5){
				server_connection = 0;
				period = 250;
				blinkParam.period = 200;
			}
		}

		vTaskDelayUntil(&pxPreviousWakeTime, period);
	}while(1);
	vTaskDelete(NULL);
}

#endif /* VTASKESP_H_ */
