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
	initBluePillLed();		// BluePill onboard LED initialization
	led_indicator_init();	// motherboard led indicator initialization

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

	if((pdTRUE != xTaskCreate(vESP8266Task, "ESP", 512, NULL, tskIDLE_PRIORITY, &xTaskHandleESP))) {
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
	if((xStationsConnectedSemaphore != NULL) && (xSemaphoreTake(xStationsConnectedSemaphore, 0) == pdTRUE)){
		stations_connected = 0; // refresh connected stations number
		for(uint8_t i = 0; i < NUM_CLIENT_STATIONS; i++){ // list all stations
			if(client_station[i].network.link_status == espLinkOk){ // if station was connected (set by mavlink parser in heartbeat case)
				if((millis() - client_station[i].network.heartbeat_timer) <= ESP_CONNECTION_LOSS_TIME){ // check connection loss timeout
					stations_connected++;
				}else{
					client_station[i].network.link_status = espLinkLost;
					blinkParam.period = 200;
					DEBUG_printf("CON_LOST!\n", NULL);
				}
			}
		}
		xSemaphoreGive(xStationsConnectedSemaphore);
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
