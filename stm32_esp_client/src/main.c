/*-----------------------------------------------------------------------------
*** ESP Communication test program - Client
***
*** v.1.6
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
#include "inc.h"

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
// OS routine macro. Can be used for debug
#define	ERROR_ACTION()				{}
#define	ERROR_ACTION_CRITICAL()		do{ }while(1)

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void initRCC(void);

/*-----------------------------------------------------------------------------
ENTRY POINT
-----------------------------------------------------------------------------*/
int main(void)
{
	blinkParam.period = 200;				// set initial blinking period for onboard LED (BluePill)
	sdcardParam.period = 1000;

	/* JTAG-DP Disabled and SW-DP Enabled */
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // needed for use PB3, PB4, PA15

	initRCC();				// system clock source initialization
	initUSART1();			// DEBUG port (uses DMA)
	initUSART2();
	initBoardButtons();		// "keyboard" initialization
	initBluePillLed();		// BluePill onboard LED initialization
	led_indicator_init();	// motherboard led indicator initialization
	I2C_Initialize(I2C1, I2C_CLOCK_STANDARD);	// init commmon I2C port (same port and speed for BMP180, MAX44009 and other digital sensors)

	/* Initialize MAX44009 ambient light sensor */
	if (I2C_IsDeviceConnected(MAX44009_I2C_PORT, MAX44009_ADDR_1)) {
		MAX44009_Init(MAX44009_ADDR_1);
		MAX44009_Data.health_1 = SENSOR_OK;
//		DEBUG_printf("1st MAX44009 configured and ready to use\n", NULL);				// Init OK
	}else{
		MAX44009_Data.health_1 = SENSOR_DEAD;
//		DEBUG_printf("1st MAX44009 error: not present\n", NULL);						// Device error
		set_led4(HIGH);
	}

	if (I2C_IsDeviceConnected(MAX44009_I2C_PORT, MAX44009_ADDR_2)) {
		MAX44009_Init(MAX44009_ADDR_2);
		MAX44009_Data.health_2 = SENSOR_OK;
//		DEBUG_printf("2nd MAX44009 configured and ready to use\n", NULL);				// Init OK
	}else{
		MAX44009_Data.health_2 = SENSOR_DEAD;
//		DEBUG_printf("2nd MAX44009 error: not present\n", NULL);						// Device error
		set_led4(HIGH);
	} /* END MAX44009 init */

	/* Initialize BMP180 pressure sensor */
	if (BMP180_Init() == BMP180_Result_Ok) {
		BMP180_GetPressureAtSeaLevel(101325, 0);
		BMP180_Data.health = SENSOR_OK;
//		DEBUG_printf("BMP180 configured and ready to use\n", NULL);					// Init OK
	} else {
		BMP180_Data.health = SENSOR_DEAD;
//		DEBUG_printf("BMP180 error\n", NULL);											// Device error
		set_led4(HIGH);
	} /* END BMP180 init */


	set_led_status(1);
	mavlink_enable = 1;

	if((pdTRUE != xTaskCreate(vBlinker,"Blinker", 32, &blinkParam, tskIDLE_PRIORITY + 2, &xTaskHandleBlinker))) {
		ERROR_ACTION_CRITICAL();
	}

//	if((pdTRUE != xTaskCreate(vButtonsCheck,"Button", 32, NULL, tskIDLE_PRIORITY + 2, &xTaskHandleBtnCheck))) {
//		ERROR_ACTION_CRITICAL();
//	}

	if((pdTRUE != xTaskCreate(vESP8266Task,"ESP_Main", 512, NULL, tskIDLE_PRIORITY + 3, &xTaskHandleESP))) {
		ERROR_ACTION_CRITICAL();
	}

	if((pdTRUE != xTaskCreate(vBMP180_sample,"BMP180", 128, NULL, tskIDLE_PRIORITY + 2, &xTaskHandleBMP180Sample))) {
		ERROR_ACTION_CRITICAL();
	}

	if((pdTRUE != xTaskCreate(vMAX44009_sample,"MAX44009", 64, NULL, tskIDLE_PRIORITY + 2, &xTaskHandleMAX44009Sample))) {
		ERROR_ACTION_CRITICAL();
	}

	if((pdTRUE != xTaskCreate(vSHT11_sample,"SHT11", 64, NULL, tskIDLE_PRIORITY + 2, &xTaskHandleSHT11Sample))) {
		ERROR_ACTION_CRITICAL();
	}

	if((pdTRUE != xTaskCreate(vUSART_debug,"debug", 64, NULL, tskIDLE_PRIORITY + 2, &xTaskHandleDebug))) {
		ERROR_ACTION_CRITICAL();
	}

	if((pdTRUE != xTaskCreate(vTaskSDcard, "SD", 256, &sdcardParam, tskIDLE_PRIORITY + 1, &xTaskHandleSDcard))) { // 128 + 1024 ?
		ERROR_ACTION_CRITICAL();
	}

	vTaskStartScheduler();

	do{ }while(1);
	return 0;
}

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

void vApplicationIdleHook(void)
{
	if(receiveState == 3)
	{
		if(mavlink_enable == 1)
		{
			uint16_t i = 0;
			uint16_t received = 0;
			received = payload_len;

			while((received - i) > 0)
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
		}
		esp8266ClearBuffer();
		esp8266ClearTcpBuffer();
		receiveState = 0;
	}

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
					mavlink_send_message_tcp(1, &out_msg);
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
				mavlink_send_message_tcp(1, &out_msg);

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
			mavlink_send_message_tcp(1, &out_msg);
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
    	/*** Some code like this =) ***/
//    	char c = (char)USART_ReceiveData(USART1) & 0xff;

//    	if(c == '+')
//    	{
//    		led_ppmm(&led_cnt, 1);
//    	}
//    	if(c == '-')
//    	{
//    		led_ppmm(&led_cnt, 0);
//    	}
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
