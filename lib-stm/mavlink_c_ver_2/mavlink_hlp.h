#ifndef MAVLINK_HLP_H_
#define MAVLINK_HLP_H_
#pragma once

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "common_freertos.h"	// Common convinence functions for FreeRTOS
#include "UART_freertos.h"		// USART convinence functions for FreeRTOS

#define SYS_ID_MY 		2
#define COM_ID_MY 		0
#define SYS_ID_GCS 		255

/*** MAVLink global varianles ***/
mavlink_message_t in_msg;
mavlink_message_t out_msg;
mavlink_status_t status;
uint8_t mavlink_parse_status = 0; // separate MAVLink message parsing flag (SET if complete message was received and waiting for handling)

void mavlink_reset_message(mavlink_message_t* msg)
{
	memset(msg, 0, sizeof(mavlink_message_t));
}

void mavlink_send_message(USART_TypeDef* USARTx, mavlink_message_t* msg)				//функция отправки сообщения
{
	USART_ITConfig(USARTx, USART_IT_RXNE, DISABLE);
	uint8_t buf[256];				//создаем буфер
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);//пишем туда наше сообщение
	for (uint16_t i = 0; i < len; i++)					//по байту отправляем содержимое буфера
	{
		uart_send_byte(USARTx, buf[i]);
	}
	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
}

void heartbeat_send(USART_TypeDef* USARTx, mavlink_message_t* msg)
{
	mavlink_reset_message(msg);
	mavlink_msg_heartbeat_pack(SYS_ID_MY, COM_ID_MY, msg,
				MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_GENERIC,
				0, 0, 0);
	mavlink_send_message(USARTx, msg);
}

void systime_send(USART_TypeDef* USARTx, mavlink_message_t* msg, uint32_t millis)
{
	mavlink_reset_message(msg);
	mavlink_msg_system_time_pack(SYS_ID_MY, COM_ID_MY, msg,
			0, millis);
	mavlink_send_message(USARTx, msg);
}

void statustext_send(USART_TypeDef* USARTx, mavlink_message_t* msg, const char *text, uint8_t severity)
{
	mavlink_reset_message(msg);
	mavlink_msg_statustext_pack(SYS_ID_MY,
			COM_ID_MY, msg, severity, text);
	mavlink_send_message(USARTx, msg);
}

void mission_response_send(USART_TypeDef* USARTx, mavlink_message_t* msg)
{
	mavlink_reset_message(msg);
	mavlink_msg_mission_count_pack(SYS_ID_MY, COM_ID_MY, msg,
			SYS_ID_GCS, 0,
			0);
	mavlink_send_message(USARTx, msg);
}

#endif /* MAVLINK_HLP_H_ */
