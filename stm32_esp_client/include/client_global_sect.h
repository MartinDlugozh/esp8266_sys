/*
 * client_global_sect.h
 *
 *  Created on: Jan 24, 2018
 *      Author: Dr. Saldon
 */

#ifndef CLIENT_GLOBAL_SECT_H_
#define CLIENT_GLOBAL_SECT_H_

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define SYS_ID_MY 		2
#define COM_ID_MY 		0
#define SYS_ID_SRV 		254
#define COM_ID_SRV		0
#define SYS_ID_GCS 		255

#define SENSOR_DEAD 	0
#define SENSOR_OK 		1
#define SENSOR_CHECK	2

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
uint8_t mavlink_enable = 1; 		// Use MAVLink protocol for telemetry
uint8_t led_cnt = 0;

typedef struct _BMP180_data{
	float 		temperature;
	float 		altitude;
	uint32_t 	pressure;
} BMP180_data_t;

typedef struct _MAX44009_data{
	uint32_t 	lux_ambilight_1;
	uint32_t 	lux_ambilight_2;
} MAX44009_data_t;

typedef struct _SHT11_data{
	float 		temperature;
	float 		humidity;
	float 		dewpoint;
} SHT11_data_t;

typedef struct _SYSTEMTIME_data{
	uint32_t 	time_boot_ms;
} SYSTEMTIME_data_t;

typedef struct _SERVER_data{
	uint32_t 	last_heartbeat_ms;
} SERVER_data_t;

typedef struct _log_file_op{
	uint8_t 	log_file_req_state;
	uint8_t  	log_file_sent;
	uint16_t 	log_file_cnt;		// Количество лог-фалов для передачи
	uint16_t 	log_file_seq;		// Количество принятых файлов
	uint16_t 	log_file_block_cnt;		// Количество блоков в текущем файле
	uint16_t 	log_file_block_seq;		// Количество принятых блоков
	uint8_t 	log_file_name[128];
} log_file_op_t;

typedef struct _station{
	SYSTEMTIME_data_t 	SYSTEMTIME_data;
	SERVER_data_t 		SERVER_data;
	MAX44009_data_t 	MAX44009_data;
	SHT11_data_t 		SHT11_data;
	BMP180_data_t 		BMP180_data;
	log_file_op_t 		log_file_op;
} station_t;

station_t station;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/


#endif /* CLIENT_GLOBAL_SECT_H_ */
