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



// Sending strings over USART1 (debug port) using OS semaphore (thred-safe method)
#define USART1_SEND(string) {	if(xSemaphoreTake(xUsart1RxInterruptSemaphore, portMAX_DELAY) == pdTRUE)\
							{uart_send_str_ln(USART1, string); \
							xSemaphoreGive(xUsart1RxInterruptSemaphore);} }

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void initRCC(void);
void initUSARTs(void);

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
	initUSARTs();			// common USART initialization: for debugger and ESP12
	initBoardButtons();		// "keyboard" initialization
	initBluePillLed();		// BluePill onboard LED initialization
	led_indicator_init();	// motherboard led indicator initialization
	I2C_Initialize(I2C1, I2C_CLOCK_STANDARD);	// init commmon I2C port (same port and speed for BMP180, MAX44009 and other digital sensors)

	/* Initialize MAX44009 ambient light sensor */
	if (I2C_IsDeviceConnected(MAX44009_I2C_PORT, MAX44009_ADDR_1)) {
		MAX44009_Init(MAX44009_ADDR_1);
		MAX44009_Data.health_1 = SENSOR_OK;
		FU1_printf("1st MAX44009 configured and ready to use\n", NULL);				// Init OK
	}else{
		MAX44009_Data.health_1 = SENSOR_DEAD;
		FU1_printf("1st MAX44009 error: not present\n", NULL);						// Device error
		set_led4(HIGH);
	}

	if (I2C_IsDeviceConnected(MAX44009_I2C_PORT, MAX44009_ADDR_2)) {
		MAX44009_Init(MAX44009_ADDR_2);
		MAX44009_Data.health_2 = SENSOR_OK;
		FU1_printf("2nd MAX44009 configured and ready to use\n", NULL);				// Init OK
	}else{
		MAX44009_Data.health_2 = SENSOR_DEAD;
		FU1_printf("2nd MAX44009 error: not present\n", NULL);						// Device error
		set_led4(HIGH);
	} /* END MAX44009 init */

	/* Initialize BMP180 pressure sensor */
	if (BMP180_Init() == BMP180_Result_Ok) {
		BMP180_GetPressureAtSeaLevel(101325, 0);
		BMP180_Data.health = SENSOR_OK;
		FU1_printf("BMP180 configured and ready to use\n", NULL);					// Init OK
	} else {
		BMP180_Data.health = SENSOR_DEAD;
		FU1_printf("BMP180 error\n", NULL);											// Device error
		set_led4(HIGH);
	} /* END BMP180 init */


	set_led_status(1);
	mavlink_enable = 1;

	if((pdTRUE != xTaskCreate(vBlinker,"Blinker", configMINIMAL_STACK_SIZE, &blinkParam, tskIDLE_PRIORITY + 2, NULL))) {
		ERROR_ACTION_CRITICAL();
	}

	if((pdTRUE != xTaskCreate(vButtonsCheck,"Button", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL))) {
		ERROR_ACTION_CRITICAL();
	}

	if((pdTRUE != xTaskCreate(vESP8266Task,"ESP", 320, NULL, tskIDLE_PRIORITY + 2, NULL))) {
		ERROR_ACTION_CRITICAL();
	}

	if((pdTRUE != xTaskCreate(vBMP180_sample,"BMP180", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL))) {
		ERROR_ACTION_CRITICAL();
	}

	if((pdTRUE != xTaskCreate(vMAX44009_sample,"MAX44009", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL))) {
		ERROR_ACTION_CRITICAL();
	}

	if((pdTRUE != xTaskCreate(vSHT11_sample,"SHT11", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL))) {
		ERROR_ACTION_CRITICAL();
	}

//	if((pdTRUE != xTaskCreate(vUSART_debug,"debug", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL))) {
//		ERROR_ACTION_CRITICAL();
//	}

	if((pdTRUE != xTaskCreate(vTaskSDcard, "SD", configMINIMAL_STACK_SIZE+1024, &sdcardParam, tskIDLE_PRIORITY + 1, NULL)))	{
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
}

void vApplicationTickHook(void)
{
	disk_timerproc();		// SdFAT timer routine
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

void initUSARTs(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, 	ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, 	ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, 	ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, 	ENABLE);

	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);							/*** Configure UART Rx GPIO ***/
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = PIN_UART1_RX;
	GPIO_Init(PORT_UART1, &gpio);
	gpio.GPIO_Pin = PIN_UART2_RX;
	GPIO_Init(PORT_UART2, &gpio);

	gpio.GPIO_Mode = GPIO_Mode_AF_PP;				/*** Configure UART Tx GPIO ***/
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = PIN_UART1_TX;
	GPIO_Init(PORT_UART1, &gpio);
	gpio.GPIO_Pin = PIN_UART2_TX;
	GPIO_Init(PORT_UART2, &gpio);

	USART_InitTypeDef usart;
	USART_StructInit(&usart);
	usart.USART_BaudRate            = UART1_BAUDRATE;
	usart.USART_WordLength          = USART_WordLength_8b;
	usart.USART_StopBits            = USART_StopBits_1;
	usart.USART_Parity              = USART_Parity_No ;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &usart);
	USART_Cmd(USART1, ENABLE);
	usart.USART_BaudRate            = UART2_BAUDRATE;
	USART_Init(USART2, &usart);
	USART_Cmd(USART2, ENABLE);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);    	// Enable RXNE interrupt
	NVIC_InitTypeDef nvic;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);		/*!< 0 bits for pre-emption priority, 4 bits for subpriority */
	nvic.NVIC_IRQChannel = USART1_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 15;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	NVIC_EnableIRQ(USART1_IRQn);    					// Enable USART1 global interrupt

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);    	// Enable RXNE interrupt
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);		/*!< 0 bits for pre-emption priority, 4 bits for subpriority */
	nvic.NVIC_IRQChannel = USART2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 15;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	NVIC_EnableIRQ(USART2_IRQn);    					// Enable USART1 global interrupt

	vSemaphoreCreateBinary(xUsart1RxInterruptSemaphore);
	vSemaphoreCreateBinary(xUsart2RxInterruptSemaphore);
}

#pragma GCC diagnostic pop
