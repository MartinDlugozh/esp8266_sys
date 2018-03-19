/*
 * vTaskESP.h
 *
 *  Created on: Jan 22, 2018
 *      Author: Dr. Saldon
 *      Last update: Mar 05, 2018
 */

#ifndef VTASKESP_H_
#define VTASKESP_H_

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define ESP_USE_DEBUGMESSAGES 		1

#define ESP_AP_TCP_SERVER_PORT 		1500

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
#include "common_freertos.h"			// Common convinence functions for FreeRTOS
#include "usart_freertos.h"				// USART convinence functions for FreeRTOS
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
	uint32_t esp_task_period = ESP_TASK_PERIOD_SHORT;
	uint8_t esp_comnn_state = 0;
	uint8_t esp_result = 0;

	mavlink_enable = 1;

	vTaskDelay(5000);

	do{
		switch(esp_comnn_state){
		case CONN_UNUNIT:
			if((esp_result = esp8266SetMode(ESP8266_MODE_AP)) == ESP_OK){	// Configure ESP8266 as AP
				esp_comnn_state = CONN_AP_ACT;
#if(ESP_USE_DEBUGMESSAGES == 1)
				DEBUG_printf("ESP8266 Mode: AP \n", NULL);
#endif
			}
			break;
		case CONN_AP_ACT:
//			if((esp_result = esp8266ServerCreate(ESP_AP_TCP_SERVER_PORT, 0)) == ESP_OK){
//				DEBUG_printf("Server off\n", NULL);
//			}
//			if((esp_result = esp8266SetTrans(0)) == ESP_OK){
//				DEBUG_printf("Transparent mode off\n", NULL);
//			}
			if((esp_result = esp8266SetMux(1)) == ESP_OK){
				if((esp_result = esp8266ServerCreate(ESP_AP_TCP_SERVER_PORT, 1)) == ESP_OK){
					blinkParam.period = 200;
					esp_task_period = ESP_TASK_PERIOD_LONG;
					esp_comnn_state = CONN_SERVER;
#if(ESP_USE_DEBUGMESSAGES == 1)
				DEBUG_printf("ESP8266 Server created: %d port \n", ESP_AP_TCP_SERVER_PORT);
#endif
				}
			}
			break;
		case CONN_SERVER:
			for (uint8_t i = 0; i < NUM_CLIENT_STATIONS; i++){
				if(client_station[i].HEARTBEAT_data.system_id != 0){
					blinkParam.period = 500;
					esp_comnn_state = CONN_CLIENT;
				}
			}
			break;
		case CONN_CLIENT:
//			if(mavlink_enable == 1){
//				for (uint8_t i = 0; i < NUM_CLIENT_STATIONS; i++){
//					heartbeat_send(1, &out_msg);
//					systime_send(1, &out_msg, millis());
//				}
//			}
			break;
		default:
			break;
		}

#if(ESP_USE_DEBUGMESSAGES == 1)
				DEBUG_printf("Conn: %d, Len: %d, Un: %d \n", current_connection, payload_len, unprocessed_bytes);
#endif

		vTaskDelay(esp_task_period);
//		uxHighWaterMark[HIGH_WATERMARK_ESP8266] = uxTaskGetStackHighWaterMark(NULL);	// get free stack memory
	}while(1);
	vTaskDelete(NULL);
}

#endif /* VTASKESP_H_ */
