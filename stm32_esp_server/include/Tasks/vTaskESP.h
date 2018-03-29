/*
 * vTaskESP.h
 *
 *  Created on: Jan 22, 2018
 *      Author: Dr. Saldon
 * Last change: Mar 28, 2018
 *
 *	Notes:
 * 		1. При первоначальной инициализации модуля связи для установления
 * 		периода выполнения процесса испотьзуется метод vTaskDelay() (когда mavlink_enable == 0);
 * 		После инициализации управление переходит обработчику-парсеру, который выполняется только
 * 		когда принято новое сообщение. После отрабоки парсера выполняется принудительное переключение
 * 		контекста с помощью taskYELD(); таким образом процесс постоянно находится в состоянии READY и
 * 		выполняет опрос флага receive_state (дополнительно может быть установлен семафор на выполнение).
 * 		ВАЖНО: процесс vESP8266Task() всегда должен иметь приоритет 0 (tskIDLE_PRIORITY), иначе он
 * 		будет блокировать выполнение всех остальных процессов с низшим приоритетом, в том числе IDLE.
 */

#ifndef VTASKESP_H_
#define VTASKESP_H_

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define ESP_USE_DEBUGMESSAGES 				1

#define ESP_AP_TCP_SERVER_SATRT_PORT 		1500

#define ESP_TASK_PERIOD_SHORT 				1000
#define ESP_TASK_PERIOD_LONG 				3000

#define ESP_CONNECTION_LOSS_TIME 			5000
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

typedef enum _ESP8266_ConnState{
	CONN_UNUNIT = 0,
	CONN_WIFI,
	CONN_AP_ACT,
	CONN_MUX,
	CONN_AP_CON,
	CONN_SERVER,
	CONN_CLIENT,
}ESP8266_ConnState_t;

typedef enum _ESP8266_TaskMode{
	etmYeld = 0,
	etmDelay,
}ESP8266_TaskMode_t;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void vESP8266Task(void *pvParameters);
void mavlinkProcess(void);
void fileOp(void);

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

void vESP8266Task(void *pvParameters)
{
	uint32_t esp_task_period = ESP_TASK_PERIOD_LONG;
	uint8_t espTaskMode = etmDelay;
	uint8_t esp_conn_state = 0;
	uint8_t esp_result = 0;

	mavlink_enable = 0;

	vTaskDelay(2000);

	do{
		switch(esp_conn_state){
		case CONN_UNUNIT:
			espInitSataions(client_station);
			if((esp_result = esp8266SetMode(ESP8266_MODE_AP)) == ESP_OK){	// Configure ESP8266 as AP
				esp_conn_state = CONN_AP_ACT;
#if(ESP_USE_DEBUGMESSAGES == 1)
				DEBUG_printf("ESP8266 Mode: AP \n", NULL);
#endif
			}
			break;
		case CONN_AP_ACT:
			if((esp_result = esp8266ServerCreate(ESP_AP_TCP_SERVER_SATRT_PORT, 0)) == ESP_OK){
				DEBUG_printf("APSD: %d\n", ESP_AP_TCP_SERVER_SATRT_PORT); 							// Access point Server - down
			}
			vTaskDelay(200);
			if((esp_result = esp8266SetMux(1)) == ESP_OK){
				esp_conn_state = CONN_MUX;
			}
			break;
		case CONN_MUX:
			if((esp_result = esp8266ServerCreate(ESP_AP_TCP_SERVER_SATRT_PORT, 1)) == ESP_OK){
				DEBUG_printf("APSO: %d\n", ESP_AP_TCP_SERVER_SATRT_PORT); // AP Server ON: port
				blinkParam.period = 200;
				esp_task_period = ESP_TASK_PERIOD_SHORT;
				esp_conn_state = CONN_SERVER;
				mavlink_enable = 1;
				espTaskMode = etmYeld;
			}else{
				DEBUG_printf("APSE: %d\n", ESP_AP_TCP_SERVER_SATRT_PORT); // AP Server creation error: port
			}
			vTaskDelay(200);
			break;
		case CONN_SERVER:
			if(mavlink_enable == 1){
				mavlinkProcess();
				fileOp();
			}
			break;
		default:
			break;
		}

	    if(espTaskMode == etmYeld){
	    	taskYIELD();
	    }else if(espTaskMode == etmDelay){
	    	vTaskDelay(esp_task_period);
	    }

//		uxHighWaterMark[HIGH_WATERMARK_ESP8266] = uxTaskGetStackHighWaterMark(NULL);	// get free stack memory
	}while(1);
	vTaskDelete(NULL);
}

void mavlinkProcess(void){
	// Here we only parse and process MAVLink messages from the client
	if(receiveState == ESP_TCP_RECEPTION_DONE)
	{
		uint16_t i = 0;
		uint16_t bytes_left = 0;
		bytes_left = payload_len;	// received = payload_len+1; WHY?

		while((bytes_left - i) > 0)
		{
			if(mavlink_parse_char(MAVLINK_COMM_0, tcpRxBuffer[i], &in_msg, &status))
			{
				uint8_t client_id = TO_CLI_ID(in_msg.sysid);
				client_station[client_id].network.link_id = current_connection;
				switch (in_msg.msgid) {
				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					client_station[client_id].network.link_status = espLinkOk;
					client_station[client_id].network.heartbeat_timer = millis();

					DEBUG_printf("-RHB- con: %d, sid: %d\n", current_connection, in_msg.sysid);
					blinkParam.period = 500;
					break;
				}
				case MAVLINK_MSG_ID_SYSTEM_TIME:
				{
					client_station[client_id].SYSTEMTIME_data.time_boot_ms = mavlink_msg_system_time_get_time_boot_ms(&in_msg);
					break;
				}

				case MAVLINK_MSG_ID_ECO_BMP180:
				{
					client_station[client_id].BMP180_data.temperature = 	mavlink_msg_eco_bmp180_get_temperature(&in_msg);
					client_station[client_id].BMP180_data.altitude = 		mavlink_msg_eco_bmp180_get_altitude(&in_msg);
					client_station[client_id].BMP180_data.pressure = 		mavlink_msg_eco_bmp180_get_presssure(&in_msg);

					lcdParam.temp = 	client_station[client_id].BMP180_data.temperature;
					lcdParam.alt = 		client_station[client_id].BMP180_data.altitude;
					lcdParam.press = 	client_station[client_id].BMP180_data.pressure;

					break;
				}
				case MAVLINK_MSG_ID_ECO_MAX44009:
				{
					client_station[client_id].MAX44009_data.lux_ambilight_1 = mavlink_msg_eco_max44009_get_lux_ambilight_1(&in_msg);
					client_station[client_id].MAX44009_data.lux_ambilight_2 = mavlink_msg_eco_max44009_get_lux_ambilight_2(&in_msg);

					lcdParam.lux_ambilight_1 = 	client_station[client_id].MAX44009_data.lux_ambilight_1;
					lcdParam.lux_ambilight_2 = 	client_station[client_id].MAX44009_data.lux_ambilight_2;

					break;
				}
				case MAVLINK_MSG_ID_ECO_SHT11:
				{
					client_station[client_id].SHT11_data.temperature = mavlink_msg_eco_sht11_get_temperature(&in_msg);
					client_station[client_id].SHT11_data.humidity = mavlink_msg_eco_sht11_get_humidity(&in_msg);
					client_station[client_id].SHT11_data.dewpoint = mavlink_msg_eco_sht11_get_dowpoint(&in_msg);

					lcdParam.sht_temp = client_station[client_id].SHT11_data.temperature;
					lcdParam.humidity = client_station[client_id].SHT11_data.humidity;
					lcdParam.dewpoint = client_station[client_id].SHT11_data.dewpoint;

					break;
				}
				case MAVLINK_MSG_ID_ECO_FILE_REQUEST_RESPONSE:
				{		// Приняли ответ на запрос файла
					mavlink_eco_file_request_response_t response;
					mavlink_msg_eco_file_request_response_decode(&in_msg, &response);

					if(client_station[client_id].log_file_op.log_file_req_state == FILE_TRANS_STATE_REQ_LIST_RESP){
						client_station[client_id].log_file_op.log_file_cnt = response.block_cnt;
						client_station[client_id].log_file_op.log_file_seq = response.block_seq;
						// Остальные параметры (ид файла и его имя) пока не используются

						// Если приняли данные по всем файлам, можем переходить к запросу передачи самих файлов
						if(response.block_cnt == response.block_seq){
							client_station[client_id].log_file_op.log_file_seq = 0; // Обнуляем счетчик, т.к. он будет использован при приеме файлов
							client_station[client_id].log_file_op.log_file_req_state = FILE_TRANS_STATE_REQ_FILE;
						}
					}

					if(client_station[client_id].log_file_op.log_file_req_state == FILE_TRANS_STATE_REQ_FILE){
						client_station[client_id].log_file_op.log_file_req_state = FILE_TRANS_STATE_REQ_FILE_RESP;
					}

#if(DEB_FILE_OP == 1)
					DEBUG_printf("Got FILE_REQ_RESP: %d, %d of %d, %s\n", response.log_id, response.block_seq, response.block_cnt, response.log_name);
#endif
					// Принимаем имя текущего файла
					if((client_station[client_id].log_file_op.log_file_req_state == FILE_TRANS_STATE_REQ_FILE_RESP) &&
							(response.log_id == client_station[client_id].log_file_op.log_file_seq)){
						memset(client_station[client_id].log_file_op.log_file_name, 0, 128);
						memcpy(client_station[client_id].log_file_op.log_file_name, response.log_name, 128);
						client_station[client_id].log_file_op.log_file_block_cnt = response.block_cnt;
						client_station[client_id].log_file_op.log_file_block_seq = 0;

						// Create file
						sdcard.res = f_open(&sdcard.file, (const char *)client_station[client_id].log_file_op.log_file_name, FA_WRITE | FA_OPEN_ALWAYS);
					}

					// Посылаем ACK
					uint8_t ack = FILE_ACK;

					mavlink_reset_message(&out_msg);
					mavlink_msg_eco_file_ack_pack(SYS_ID_MY, COM_ID_MY, &out_msg,
							TO_SYS_ID(client_id),
							response.log_id,
							response.block_cnt,
							response.block_seq,
							ack);
					mavlink_send_message_tcp(client_station[client_id].network.link_id, &out_msg);

#if(DEB_FILE_OP == 1)
					DEBUG_printf("ACK sent\n", NULL);
#endif

					break;
				}
				case MAVLINK_MSG_ID_ECO_FILE_BLOCK:
				{
					mavlink_eco_file_block_t file_block;
					mavlink_msg_eco_file_block_decode(&in_msg, &file_block);

					UINT str_len = 0;
					sdcard.res = f_open(&sdcard.file, (const char *)client_station[client_id].log_file_op.log_file_name, FA_WRITE | FA_OPEN_ALWAYS);
					f_lseek(&sdcard.file, sdcard.offs);
					sdcard.res = f_write(&sdcard.file, file_block.data, file_block.block_len, &str_len);
					if(sdcard.res){
					}else{
						sdcard.offs += str_len;
					}

#if(DEB_FILE_OP == 1)
					DEBUG_printf("Got FILE_BLOCK: %d, %d, %d of %d\n",
							file_block.log_id,
							file_block.block_len,
							file_block.block_seq,
							file_block.block_cnt);
#endif

					// Файл полностью принят
					if(file_block.block_cnt == file_block.block_seq){
						f_sync(&sdcard.file);
						client_station[client_id].log_file_op.log_file_block_cnt = 0;
						client_station[client_id].log_file_op.log_file_block_seq = 0;
#if(DEB_FILE_OP == 1)
						DEBUG_printf("FILE_RECEIVED: %d of %d\n",
								client_station[client_id].log_file_op.log_file_seq,
								client_station[client_id].log_file_op.log_file_cnt);
#endif
						if(client_station[client_id].log_file_op.log_file_cnt == client_station[client_id].log_file_op.log_file_seq){
							// Если принят крайний лог, устанавливаем последовательнось передачи в отключенное состояние (не инициировано/завершено)
							client_station[client_id].log_file_op.log_file_req_state = FILE_TRANS_STATE_NOINIT;
							client_station[client_id].log_file_op.log_file_seq = 0;
							option[view] = 5; // LCD view -> Load log - Main
						}else{
							// Выполняем инкремент счетчика принятых файлов и разрешаем запрос следующего файла
							client_station[client_id].log_file_op.log_file_seq++;
							client_station[client_id].log_file_op.log_file_req_state = FILE_TRANS_STATE_REQ_FILE;
						}
					}

					f_close(&sdcard.file);		// Close file

					break;
				}
				default:
					break;
				}
			}
			i++;
		}
		esp8266ClearBuffer();
		esp8266ClearTcpBuffer();
		receiveState = ESP_WAIT_IPD;
	}
}

void fileOp(void){
	// Блок операций с файлами (отправка запросов)
	for (uint8_t i = 0; i < NUM_CLIENT_STATIONS; i++){
		switch(client_station[i].log_file_op.log_file_req_state){
		case FILE_TRANS_STATE_REQ_LIST:
		{
#if(DEB_FILE_OP == 1)
			DEBUG_printf("FILE_TRANS_STATE_REQ_LIST..\n", NULL);
#endif
			mavlink_reset_message(&out_msg);
			mavlink_msg_eco_file_request_pack(SYS_ID_MY, COM_ID_MY, &out_msg,
					TO_SYS_ID(client_station[i].network.cli_id), 0, LOG_TYPE_ALL, OP_TYPE_LIST);
			mavlink_send_message_tcp(client_station[i].network.link_id, &out_msg);

			client_station[i].log_file_op.log_file_req_state = FILE_TRANS_STATE_REQ_LIST_RESP;
			break;
		}
		case FILE_TRANS_STATE_REQ_FILE:
		{
#if(DEB_FILE_OP == 1)
			DEBUG_printf("FILE_TRANS_STATE_REQ_FILE: %d\n", client_station[i].log_file_op.log_file_seq);
#endif
			mavlink_reset_message(&out_msg);
			mavlink_msg_eco_file_request_pack(SYS_ID_MY, COM_ID_MY, &out_msg,
					TO_SYS_ID(client_station[i].network.cli_id),
					client_station[i].log_file_op.log_file_seq,		// log_id соответствует порядковому номеру для текущей транзакции
					LOG_TYPE_ALL,
					OP_TYPE_LOAD);
			mavlink_send_message_tcp(client_station[i].network.link_id, &out_msg);

			client_station[i].log_file_op.log_file_req_state = FILE_TRANS_STATE_REQ_LIST_RESP;
			break;
		}
		default:
			break;
		}
	}
}

#endif /* VTASKESP_H_ */
