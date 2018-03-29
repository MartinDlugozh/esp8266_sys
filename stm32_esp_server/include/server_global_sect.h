/*
 * server_global_sect.h
 *
 *  Created on: Jan 28, 2018
 *      Author: Dr. Saldon
 * Last change: Mar 28, 2018
 */

#ifndef INCLUDE_SERVER_GLOBAL_SECT_H_
#define INCLUDE_SERVER_GLOBAL_SECT_H_

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define NUM_CLIENT_STATIONS 	5
#define TO_SYS_ID(x)			(x != 0) ? (x+1) : (0)
#define TO_CLI_ID(x)	 		(x != 0) ? (x-1) : (0)

#define SYS_ID_MY 		254
#define COM_ID_MY 		0
#define SYS_ID_GCS 		255

#define CLI_TCP_PORT_TX 1500

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
uint8_t mavlink_enable = 1; 		// Use MAVLink protocol for telemetry
uint8_t led_cnt = 0;

typedef struct _SYSTEMTIME_data{
	uint32_t time_boot_ms;
} SYSTEMTIME_data_t;

typedef struct _BMP180_data{
	float temperature;
	float altitude;
	uint32_t pressure;
} BMP180_data_t;

BMP180_data_t BMP180_data;

typedef struct _MAX44009_data{
	uint32_t lux_ambilight_1;
	uint32_t lux_ambilight_2;
} MAX44009_data_t;

typedef struct _SHT11_data{
	float 		temperature;
	float 		humidity;
	float 		dewpoint;
} SHT11_data_t;

typedef struct _log_file_op{
	uint8_t 	log_file_req_state;
	uint16_t 	log_file_cnt;		// Количество лог-фалов для передачи
	uint16_t 	log_file_seq;		// Количество принятых файлов
	uint16_t 	log_file_block_cnt;		// Количество блоков в текущем файле
	uint16_t 	log_file_block_seq;		// Количество принятых блоков
	uint8_t 	log_file_name[128];
} log_file_op_t;

typedef enum _link_status{
	espLinkNo = 0,
	espLinkOk,
	espLinkLost,
	espLinkError,
} link_status_t;

typedef struct _esp_network{
	uint8_t cli_id;					// client id; == (in_msg.sys_id - 1)
	uint8_t link_id;				// link id
	uint32_t heartbeat_timer;		// last heartbeat reception time [ms]
	link_status_t link_status;
} esp_network_t;

typedef struct _client_settings{
	uint8_t logging_period;
	uint8_t enable_ihz_log;
} client_settings_t;

typedef struct _client_station{
	SYSTEMTIME_data_t 	SYSTEMTIME_data;
	MAX44009_data_t 	MAX44009_data;
	SHT11_data_t 		SHT11_data;
	BMP180_data_t 		BMP180_data;
	log_file_op_t 		log_file_op;
	esp_network_t		network;
	client_settings_t 	settings;
} client_station_t;

client_station_t client_station[NUM_CLIENT_STATIONS];
uint8_t stations_connected = 0;
SemaphoreHandle_t xStationsConnectedSemaphore;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void espInitSataions(client_station_t* client_station);

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

void espInitSataions(client_station_t* client_station){
	memset(client_station, 0, (sizeof(client_station_t) * NUM_CLIENT_STATIONS));
	for(uint8_t i = 0; i < NUM_CLIENT_STATIONS; i++){
		client_station[i].network.cli_id = i;
		client_station[i].network.link_status = espLinkNo;
	}
	vSemaphoreCreateBinary(xStationsConnectedSemaphore);
	xSemaphoreGive(xStationsConnectedSemaphore);
}

#endif /* INCLUDE_SERVER_GLOBAL_SECT_H_ */
