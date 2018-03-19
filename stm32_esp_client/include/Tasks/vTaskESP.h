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

#define ESP_TASK_PERIOD_SHORT 		200
#define ESP_TASK_PERIOD_LONG 		1000

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
#include "usart_freertos.h"		// USART convinence functions for FreeRTOS
#include "esp8266_simple.h"

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
TaskHandle_t xTaskHandleESP;

typedef enum{
	CONN_UNUNIT = 0,
	CONN_WIFI,
	CONN_AP_ACT,
	CONN_AP_CON,
	CONN_SERVER,
	CONN_CLIENT,
}ESP8266_ConnState_t;

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
	uint16_t period = ESP_TASK_PERIOD_SHORT;

	uint8_t esp_comnn_state = 0;
	uint8_t esp_result = 0;

	uint8_t reconnection_counter = 0;

	blinkParam.period = 200;
	mavlink_enable = 1;

	vTaskDelay(2000);

	pxPreviousWakeTime = xTaskGetTickCount();
	do{
		switch(esp_comnn_state){
		case CONN_UNUNIT:		// Wi-Fi module uninitialized; connect to AP first
			if((esp_result = esp8266Connect(WIFINAME, WIFIPASS)) == ESP_OK){
				esp_comnn_state = CONN_WIFI;
				DEBUG_printf("WIFI connected\n", NULL);
			}
			break;
		case CONN_WIFI:
			if((esp_result = esp8266SetTrans(0) == ESP_OK)){
				DEBUG_printf("Transparent mode off\n", NULL);
			}
			if((esp_result = esp8266SetMux(1)) == ESP_OK){
				DEBUG_printf("Set MUX\n", NULL);
				if((esp_result = esp8266TcpConnect(SERVER_ADDRESS, SERVER_PORT, SERVER_CONNECTION)) == ESP_OK){
					esp_comnn_state = CONN_CLIENT;
					blinkParam.period = 500;
					DEBUG_printf("Connected to TCP server\n", NULL);
				}
			}
			break;
		case CONN_CLIENT:
			if(mavlink_enable == 1){ 		// Send telemetry, if enabled
				heartbeat_send(1, &out_msg);
				systime_send(1, &out_msg, millis());

				mavlink_reset_message(&out_msg);
				mavlink_msg_eco_bmp180_pack(SYS_ID_MY, COM_ID_MY, &out_msg,
						station.BMP180_data.temperature,
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

			if(reconnection_counter >= 5){
				esp_comnn_state = CONN_UNUNIT;
				period = ESP_TASK_PERIOD_SHORT;
				blinkParam.period = 200;
			}
			break;
		default:
			break;
		}

		vTaskDelayUntil(&pxPreviousWakeTime, period);
		uxHighWaterMark[HIGH_WATERMARK_ESP8266] = uxTaskGetStackHighWaterMark(NULL);	// get free stack memory
	}while(1);
	vTaskDelete(NULL);
}

#endif /* VTASKESP_H_ */
