#ifndef MAVLINK_HLP_ESP_H_
#define MAVLINK_HLP_ESP_H_
#pragma once

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "common_freertos.h"	// Common convinence functions for FreeRTOS
#include "UART_freertos.h"		// USART convinence functions for FreeRTOS

#include "esp8266_simple.h"		// Simple ESP8266 driver

/*** MAVLink global varianles ***/
mavlink_message_t in_msg;
mavlink_message_t out_msg;
mavlink_status_t status;
uint8_t mavlink_parse_status = 0; // separate MAVLink message parsing flag (SET if complete message was received and waiting for handling)

void mavlink_reset_message(mavlink_message_t* msg)
{
	memset(msg, 0, sizeof(mavlink_message_t));
}

void mavlink_send_message_tcp(uint8_t tcp_link, mavlink_message_t* msg)				//функция отправки сообщения
{
	uint8_t buf[256];				//создаем буфер
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);//пишем туда наше сообщение
	esp8266TcpSend(buf, len, tcp_link);
}

void heartbeat_send(uint8_t tcp_link, mavlink_message_t* msg)
{
	mavlink_reset_message(msg);
	mavlink_msg_heartbeat_pack(SYS_ID_MY, COM_ID_MY, msg,
				MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_GENERIC,
				0, 0, 0);
	mavlink_send_message_tcp(tcp_link, msg);
}

void systime_send(uint8_t tcp_link, mavlink_message_t* msg, uint32_t millis)
{
	mavlink_reset_message(msg);
	mavlink_msg_system_time_pack(SYS_ID_MY, COM_ID_MY, msg,
			0, millis);
	mavlink_send_message_tcp(tcp_link, msg);
}

void statustext_send(uint8_t tcp_link, mavlink_message_t* msg, const char *text, uint8_t severity)
{
	mavlink_reset_message(msg);
	mavlink_msg_statustext_pack(SYS_ID_MY,
			COM_ID_MY, msg, severity, text);
	mavlink_send_message_tcp(tcp_link, msg);
}

#endif /* MAVLINK_HLP_ESP_H_ */
