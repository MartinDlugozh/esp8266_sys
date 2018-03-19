/*-----------------------------------------------------------------------------
*** ESP Communication test program - Server
***
*** Проект эколога по созданию системы сбора данных с датчиков.
*** Первый скелет для работы с отладочной платой v.0.1a.
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
// OS routine macro. Can be used for debug
#define	ERROR_ACTION()				{}
#define	ERROR_ACTION_CRITICAL()		do{ }while(1)

// Выводить время (раз в сек)
#define DEB_COMM	 		1
// Выводить инфо о каджом принятом сообщении (источник, ид)
#define DEB_RECEIVED_MSG 	0
// Выводить инфо о принятых данных сенсоров
#define DEB_REC_SENSOR_SIM 	0
// Выводить инфо об обмене файлами
#define DEB_FILE_OP 		1

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
#include "inc.h"

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/

uint16_t transactions_count = 0;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void initRCC(void);
//void initUSARTs(void);

/*-----------------------------------------------------------------------------
ENTRY POINT
-----------------------------------------------------------------------------*/
int main(void)
{
	blinkParam.period = 100;	// set initial blinking period for onboard LED (BluePill)
	lcdParam.view = 0;			// initial LCD view
	lcdParam.period = 100;		// LCD update period

	/* JTAG-DP Disabled and SW-DP Enabled */
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // needed for use PB3, PB4, PA15

	initRCC();				// system clock source initialization
	initUSART1();			// DEBUG port
	initUSART2();			// ESP8266
//	initBoardButtons();		// "keyboard" initialization
	initBluePillLed();		// BluePill onboard LED initialization
	led_indicator_init();	// motherboard led indicator initialization

	for (uint8_t i = 0; i < NUM_CLIENT_STATIONS; i++){
		client_station[i].HEARTBEAT_data.system_id = 0;
		client_station[i].HEARTBEAT_data.componnent_id = 0;
		client_station[i].SYSTEMTIME_data.time_boot_ms = 0;
		client_station[i].BMP180_data.temperature = 0;
		client_station[i].BMP180_data.altitude = 0;
		client_station[i].BMP180_data.pressure = 0;
		client_station[i].MAX44009_data.lux_ambilight_1 = 0;
		client_station[i].MAX44009_data.lux_ambilight_2 = 0;
		client_station[i].SHT11_data.dewpoint = 0;
		client_station[i].SHT11_data.humidity = 0;
		client_station[i].SHT11_data.temperature = 0;
		client_station[i].log_file_op.log_file_req_state = FILE_TRANS_STATE_NOINIT;
		client_station[i].log_file_op.log_file_block_cnt = 0;
		client_station[i].log_file_op.log_file_block_seq = 0;
		client_station[i].log_file_op.log_file_cnt = 0;
		client_station[i].log_file_op.log_file_seq = 0;
	}

	set_led_status(1);
	mavlink_enable = 1;

	if((pdTRUE != xTaskCreate(vBlinker, "Blinker", 32, &blinkParam, tskIDLE_PRIORITY + 1, NULL)))	{
		ERROR_ACTION_CRITICAL();
	}

	if((pdTRUE != xTaskCreate(vLCD_Update, "LCD", configMINIMAL_STACK_SIZE, &lcdParam, tskIDLE_PRIORITY + 1, NULL)))	{
		ERROR_ACTION_CRITICAL();
	}

	if((pdTRUE != xTaskCreate(vButtonsCheck, "Button", configMINIMAL_STACK_SIZE+128, NULL, tskIDLE_PRIORITY, NULL)))	{
		ERROR_ACTION_CRITICAL();
	}

	if((pdTRUE != xTaskCreate(vESP8266Task, "ESP", 512, NULL, tskIDLE_PRIORITY + 1, &xTaskHandleESP))) {
		ERROR_ACTION_CRITICAL();
	}

	if((pdTRUE != xTaskCreate(vTaskSDcard, "SD", 256, &sdcardParam, tskIDLE_PRIORITY + 1, NULL)))	{
		ERROR_ACTION_CRITICAL();
	}

	vTaskStartScheduler();

	do{ }while(1);
	return 0;
}

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

void vApplicationTickHook(void)
{
	disk_timerproc();			// SdFAT timer routine
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName )
{
	signed char taskName[16];
	strcpy((char*)taskName, (char*)pcTaskName);
	do{ }while(1);
}


void vApplicationIdleHook(void)
{
	// Here we only parse and process MAVLink messages from the client
	if(receiveState == ESP_TCP_RECEPTION_DONE)
	{
		if(mavlink_enable == 1)
		{
			uint16_t i = 0;
			uint16_t bytes_left = 0;
			bytes_left = payload_len;	// received = payload_len+1;

			while((bytes_left - i) > 0)
			{
				if(mavlink_parse_char(MAVLINK_COMM_0, tcpRxBuffer[i], &in_msg, &status))
				{
					uint8_t client_id = (in_msg.sysid - 1);
					switch (in_msg.msgid) {
					case MAVLINK_MSG_ID_HEARTBEAT:
					{
						client_station[client_id].HEARTBEAT_data.system_id = in_msg.sysid;
						client_station[client_id].HEARTBEAT_data.componnent_id = in_msg.compid;

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
								client_station[client_id].HEARTBEAT_data.system_id,
								response.log_id,
								response.block_cnt,
								response.block_seq,
								ack);
						mavlink_send_message_tcp(1, &out_msg);

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
		}
		esp8266ClearBuffer();
		esp8266ClearTcpBuffer();
		receiveState = ESP_WAIT_IPD;
	}

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
					client_station[i].HEARTBEAT_data.system_id, 0, LOG_TYPE_ALL, OP_TYPE_LIST);
			mavlink_send_message_tcp(1, &out_msg);

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
					client_station[i].HEARTBEAT_data.system_id,
					client_station[i].log_file_op.log_file_seq,		// log_id соответствует порядковому номеру для текущей транзакции
					LOG_TYPE_ALL,
					OP_TYPE_LOAD);
			mavlink_send_message_tcp(1, &out_msg);

			client_station[i].log_file_op.log_file_req_state = FILE_TRANS_STATE_REQ_LIST_RESP;
			break;
		}
		default:
			break;
		}
	}
}

/**
 * USART1_IRQHandler
 */
void USART1_IRQHandler(void)
{
 	portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
    	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    	char c = (char)USART_ReceiveData(USART1) & 0xff;

    	if(c == '+')
    	{
    		led_ppmm(&led_cnt, 1);
    	}
    	if(c == '-')
    	{
    		led_ppmm(&led_cnt, 0);
    	}
    }

    xSemaphoreGiveFromISR(xUsart1RxInterruptSemaphore, &xHigherPriorityTaskWoken);
   	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void initRCC(void)
{
	RCC_DeInit();												// Reset RCC
	RCC_HSEConfig(RCC_HSE_ON);									// Enable HSE

	if(RCC_WaitForHSEStartUp() == SUCCESS) 						// Wait till HSE is ready
	{
		RCC_HCLKConfig(RCC_SYSCLK_Div1);						// HCLK = SYSCLK
		RCC_PCLK2Config(RCC_HCLK_Div1);							// PCLK2 = HCLK
		RCC_PCLK1Config(RCC_HCLK_Div2);							// PCLK1 = HCLK/2
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); 	// PLLCLK = 8MHz * 9 = 72 MHz
		RCC_PLLCmd(ENABLE);										// Enable PLL
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)		// Wait till PLL is ready
		{	}
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);				// Select PLL as system clock source
		while(RCC_GetSYSCLKSource() != 0x08)					// Wait till PLL is used as system clock source
		{	}
	}
}

#pragma GCC diagnostic pop
