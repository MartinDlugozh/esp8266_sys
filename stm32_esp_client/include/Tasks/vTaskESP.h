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
#define SERVER_CONNECTION 	"0"

#define ESP_TASK_PERIOD_SHORT 		200
#define ESP_TASK_PERIOD_LONG 		1000
#define ESP_TELEMETRY_PERIOD 		1000

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

typedef enum _ESP8266_ConnState{
	CONN_UNUNIT = 0,
	CONN_WIFI,
	CONN_AP_ACT,
	CONN_TRANSP,
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
//	TickType_t pxPreviousWakeTime;
	uint32_t esp_task_period = ESP_TASK_PERIOD_SHORT;
	uint8_t espTaskMode = etmDelay;

	uint8_t esp_comnn_state = 0;
	uint8_t esp_result = 0;

//	uint8_t reconnection_counter = 0;
	uint32_t telemetry_timer = 0;

	blinkParam.period = 200;
	mavlink_enable = 0;

	vTaskDelay(2000);

//	pxPreviousWakeTime = xTaskGetTickCount();
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
				esp_comnn_state = CONN_TRANSP;
			}
			break;
		case CONN_TRANSP:
			if((esp_result = esp8266SetMux(1)) == ESP_OK){
				esp_comnn_state = CONN_MUX;
				DEBUG_printf("Set MUX\n", NULL);
			}
			break;
		case CONN_MUX:
			if((esp_result = esp8266TcpConnect(SERVER_ADDRESS, SERVER_PORT, SERVER_CONNECTION)) == ESP_OK){
				esp_comnn_state = CONN_CLIENT;
				blinkParam.period = 500;
//				esp_task_period = ESP_TASK_PERIOD_LONG;
				mavlink_enable = 1;
				espTaskMode = etmYeld;
				telemetry_timer = millis();
				DEBUG_printf("Connected to TCP server\n", NULL);
			}
			break;
		case CONN_CLIENT:
			mavlink_enable = 1;
			if(mavlink_enable == 1){
				if((millis() - telemetry_timer) >= ESP_TELEMETRY_PERIOD){ 		// Send telemetry, if enabled
					heartbeat_send(0, &out_msg);
					systime_send(0, &out_msg, millis());

					mavlink_reset_message(&out_msg);
					mavlink_msg_eco_bmp180_pack(SYS_ID_MY, COM_ID_MY, &out_msg,
							station.BMP180_data.temperature,
							station.BMP180_data.altitude,
							station.BMP180_data.pressure);
					mavlink_send_message_tcp(0, &out_msg);

					mavlink_reset_message(&out_msg);
					mavlink_msg_eco_max44009_pack(SYS_ID_MY, COM_ID_MY, &out_msg,
							station.MAX44009_data.lux_ambilight_1,
							station.MAX44009_data.lux_ambilight_2);
					mavlink_send_message_tcp(0, &out_msg);

					mavlink_reset_message(&out_msg);
					mavlink_msg_eco_sht11_pack(SYS_ID_MY, COM_ID_MY, &out_msg,
							station.SHT11_data.temperature,
							station.SHT11_data.humidity,
							station.SHT11_data.dewpoint);
					mavlink_send_message_tcp(0, &out_msg);

					telemetry_timer = millis();
				}
			}

			// TODO: заменить периодическое принудительное переподключение проверкой связи
//			if(reconnection_counter >= 10){
//				esp_comnn_state = CONN_UNUNIT;
//				esp_task_period = ESP_TASK_PERIOD_SHORT;
//				blinkParam.period = 200;
//			}
			break;
		default:
			break;
		}

	    if(espTaskMode == etmYeld){
	    	taskYIELD();
	    }else if(espTaskMode == etmDelay){
	    	vTaskDelay(esp_task_period);
	    }

//		vTaskDelayUntil(&pxPreviousWakeTime, esp_task_period);
		uxHighWaterMark[HIGH_WATERMARK_ESP8266] = uxTaskGetStackHighWaterMark(NULL);	// get free stack memory
	}while(1);
	vTaskDelete(NULL);
}

void mavlinkProcess(void){
	if(receiveState == ESP_TCP_RECEPTION_DONE)
	{
		uint16_t i = 0;
		uint16_t bytes_left = 0;
		bytes_left = payload_len;	// received = payload_len+1; WHY?

		while((bytes_left - i) > 0)
		{
			if(mavlink_parse_char(MAVLINK_COMM_0, tcpRxBuffer[i], &in_msg, &status))
			{
				switch (in_msg.msgid) {
				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					station.SERVER_data.last_heartbeat_ms = millis();
					DEBUG_printf("HEARTBEAT rec: %d \n", in_msg.sysid);
					break;
				}
				case MAVLINK_MSG_ID_ECO_FILE_REQUEST:
				{
					mavlink_eco_file_request_t request;
					mavlink_msg_eco_file_request_decode(&in_msg, &request);

					if(request.target_system == SYS_ID_MY) {
						if((request.log_type == LOG_TYPE_ALL) && (request.op_type == OP_TYPE_LIST)){
							station.log_file_op.log_file_req_state = FILE_TRANS_STATE_SEN_LIST;
							station.log_file_op.log_file_cnt = 0;
							station.log_file_op.log_file_seq = 0;
							logging_enabled = 0;
						}else if((request.log_type == LOG_TYPE_ALL) && (request.op_type == OP_TYPE_LOAD)){
							station.log_file_op.log_file_req_state = FILE_TRANS_STATE_SEN_FILS;
							station.log_file_op.log_file_seq = request.log_id;
						}
						station.log_file_op.log_file_sent = 0;

#if(DEB_FILE_OP == 1)
						DEBUG_printf("Got FILE_REQUEST: %d, %d, %d \n", request.log_id, request.log_type, request.op_type);
#endif
					}
					break;
				}
				case MAVLINK_MSG_ID_ECO_FILE_ACK:
				{
					mavlink_eco_file_ack_t ack;
					mavlink_msg_eco_file_ack_decode(&in_msg, &ack);

					station.log_file_op.log_file_sent = 0;
					if(ack.ack == FILE_ACK){
						if(station.log_file_op.log_file_req_state == FILE_TRANS_STATE_SEN_LIST){
							station.log_file_op.log_file_seq++;
						}else if(station.log_file_op.log_file_block_seq != UINT16_MAX){
							station.log_file_op.log_file_block_seq++;
						}else if(station.log_file_op.log_file_block_seq == UINT16_MAX){
							station.log_file_op.log_file_block_seq = 0;
						}
					}

#if(DEB_FILE_OP == 1)
					DEBUG_printf("Got FILE_ACK: %d, %d, %d, %d \n", ack.log_id, ack.ack, ack.block_cnt, ack.block_seq);
#endif
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
	// Блок передачи файлов
	switch(station.log_file_op.log_file_req_state){
	case FILE_TRANS_STATE_SEN_LIST:
	{
#if(DEB_FILE_OP == 1)
		DEBUG_printf("FILE_TRANS_STATE_SEN_LIST:\n", NULL);
#endif
		if(station.log_file_op.log_file_cnt == 0){
#if(DEB_FILE_OP == 1)
		DEBUG_printf("Openning directory\n", NULL);
#endif
			sdcard.res = f_opendir(&sdcard.dir, "./");
			if(sdcard.res == FR_OK){
#if(DEB_FILE_OP == 1)
		DEBUG_printf("f_opendir: OK\n", NULL);
#endif
				while((sdcard.res = f_readdir(&sdcard.dir, &sdcard.fileinfo)) == 0){
					if ( !strcmp(sdcard.fileinfo.fname, ".") || !strcmp(sdcard.fileinfo.fname, "..") )
					{

					}else{
						station.log_file_op.log_file_cnt++;
					}
				}
#if(DEB_FILE_OP == 1)
		DEBUG_printf("log_file_cnt = %d \n", station.log_file_op.log_file_cnt);
#endif
//				f_closedir(&sdcard.dir);
			}else{
#if(DEB_FILE_OP == 1)
		DEBUG_printf("f_opendir: ERROR = %d\n", sdcard.res);
#endif
			}
			sdcard.res = f_opendir(&sdcard.dir, "./");
		}

		if(!station.log_file_op.log_file_sent){
			READDIR:
			if((sdcard.res = f_readdir(&sdcard.dir, &sdcard.fileinfo)) == 0){
				if ( !strcmp(sdcard.fileinfo.fname, ".") || !strcmp(sdcard.fileinfo.fname, "..") )
				{
					goto READDIR;
				}else{
					mavlink_reset_message(&out_msg);
					mavlink_msg_eco_file_request_response_pack(SYS_ID_MY, COM_ID_MY, &out_msg,
							SYS_ID_SRV,
							station.log_file_op.log_file_seq,
							station.log_file_op.log_file_cnt,
							station.log_file_op.log_file_seq,
							(const uint8_t *)sdcard.fileinfo.fname);
					mavlink_send_message_tcp(0, &out_msg);
				}
				station.log_file_op.log_file_sent = 1;
#if(DEB_FILE_OP == 1)
		DEBUG_printf("Name sent: %d, %s \n", station.log_file_op.log_file_seq, (const char *)sdcard.fileinfo.fname);
#endif
				if(station.log_file_op.log_file_seq == station.log_file_op.log_file_cnt){
//					station.log_file_op.log_file_req_state = FILE_TRANS_STATE_SEN_FILS;
#if(DEB_FILE_OP == 1)
		DEBUG_printf("FILE_TRANS_STATE_SEN_LIST: done! \n", NULL);
#endif
					station.log_file_op.log_file_seq = 0;
					sdcard.res = f_opendir(&sdcard.dir, "./");
					station.log_file_op.log_file_block_seq = UINT16_MAX;
				}
			}
		}

		break;
	}
	case FILE_TRANS_STATE_SEN_FILS:
	{
#if(DEB_FILE_OP == 1)
		DEBUG_printf("FILE_TRANS_STATE_SEN_FILS:\n", NULL);
#endif
		if(!station.log_file_op.log_file_sent){
			if(station.log_file_op.log_file_block_seq == UINT16_MAX){
				READDIR_F:
				if((sdcard.res = f_readdir(&sdcard.dir, &sdcard.fileinfo)) == 0){
					if ( !strcmp(sdcard.fileinfo.fname, ".") || !strcmp(sdcard.fileinfo.fname, "..") )
					{
						goto READDIR_F;
					}
				}
				station.log_file_op.log_file_block_cnt = (sdcard.fileinfo.fsize/128);
				strcpy(sdcard.file_name, sdcard.fileinfo.fname);

				mavlink_reset_message(&out_msg);
				mavlink_msg_eco_file_request_response_pack(SYS_ID_MY, COM_ID_MY, &out_msg,
						SYS_ID_SRV,
						station.log_file_op.log_file_seq,
						station.log_file_op.log_file_block_cnt,
						station.log_file_op.log_file_block_seq,
						(const uint8_t *)sdcard.fileinfo.fname);
				mavlink_send_message_tcp(0, &out_msg);

#if(DEB_FILE_OP == 1)
		DEBUG_printf("RESPONSE: %d, %d, %d, %s\n", station.log_file_op.log_file_seq,
				station.log_file_op.log_file_block_cnt,
				station.log_file_op.log_file_block_seq,
				(const char *)sdcard.fileinfo.fname);
#endif
				}

			sdcard.res = f_open(&sdcard.file, sdcard.file_name, FA_WRITE | FA_OPEN_ALWAYS);

			uint8_t databuffer[128];
			UINT bytes_read = 0;
			memset(databuffer, 0, 128);
			sdcard.res = f_read(&sdcard.file, databuffer, 128, &bytes_read);

			mavlink_reset_message(&out_msg);
			mavlink_msg_eco_file_block_pack(SYS_ID_MY, COM_ID_MY, &out_msg,
					SYS_ID_SRV,
					station.log_file_op.log_file_seq,
					station.log_file_op.log_file_block_cnt,
					station.log_file_op.log_file_block_seq,
					bytes_read,
					(const uint8_t *)databuffer);
			mavlink_send_message_tcp(0, &out_msg);
#if(DEB_FILE_OP == 1)
		DEBUG_printf("Block sent: %d, %d of %d\n", station.log_file_op.log_file_seq, station.log_file_op.log_file_block_seq, station.log_file_op.log_file_block_cnt);
#endif

			if(station.log_file_op.log_file_block_seq == station.log_file_op.log_file_block_cnt){
				station.log_file_op.log_file_block_seq = UINT16_MAX;
#if(DEB_FILE_OP == 1)
		DEBUG_printf("File sent: %d of %d\n", station.log_file_op.log_file_seq, station.log_file_op.log_file_cnt);
#endif
			}

			station.log_file_op.log_file_sent = 1;
			if(station.log_file_op.log_file_seq == station.log_file_op.log_file_cnt){
	//					station.log_file_op.log_file_req_state = FILE_TRANS_STATE_SEN_FILS;
				station.log_file_op.log_file_seq = 0;
				sdcard.res = f_opendir(&sdcard.dir, "./");
#if(DEB_FILE_OP == 1)
		DEBUG_printf("FILE_TRANS_STATE_SEN_FILS: done!\n", NULL);
#endif
			}
		}
		break;
	}
	default:
		break;
	}
}

#endif /* VTASKESP_H_ */
