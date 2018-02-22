/*-----------------------------------------------------------------------------
*** ESP Communication test program - Server
***
*** Проект эколога по созданию системы сбора данных с датчиков.
*** Первый скелет для работы с отладочной платой v.0.1a.
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
#include "inc.h"

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
// OS routine macro. Can be used for debug
#define	ERROR_ACTION()		do{ }while(1)

// Sending strings over USART1 (debug port) using OS semaphore (thred-safe method)
#define USART1_SEND(string) {	if(xSemaphoreTake(xUsart1RxInterruptSemaphore, portMAX_DELAY) == pdTRUE)\
								{uart_send_str_ln(USART1, string); \
								xSemaphoreGive(xUsart1RxInterruptSemaphore);} }

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/

uint16_t transactions_count = 0;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void initRCC(void);
void initUSARTs(void);
void initLCD(void);

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
	initUSARTs();			// common USART initialization: for debugger and ESP12
	initBoardButtons();		// "keyboard" initialization
	initBluePillLed();		// BluePill onboard LED initialization
	led_indicator_init();	// motherboard led indicator initialization
	initLCD();

	client_station[0].BMP180_data.temterature = 0;
	client_station[0].BMP180_data.altitude = 0;
	client_station[0].BMP180_data.pressure = 0;
	client_station[0].MAX44009_data.lux_ambilight_1 = 0;
	client_station[0].MAX44009_data.lux_ambilight_2 = 0;

	set_led_status(1);
	mavlink_enable = 1;

	if((pdTRUE != xTaskCreate(vBlinker,"Blinker", configMINIMAL_STACK_SIZE, &blinkParam, tskIDLE_PRIORITY + 1, NULL)) ||
			(pdTRUE != xTaskCreate(vLCD_Update,"LCD", configMINIMAL_STACK_SIZE, &lcdParam, tskIDLE_PRIORITY + 1, NULL)) ||
			(pdTRUE != xTaskCreate(vButtonsCheck,"Button", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL)) ||
			(pdTRUE != xTaskCreate(vESP8266Task,"ESP", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL)))
	{
		ERROR_ACTION();
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
	// Here we only parse and process MAVLink messages from the client
	if(receiveState == 3)
	{
		if(mavlink_enable == 1)
		{
			uint16_t i = 0;
			uint16_t received = 0;
			received = payload_len;	// received = payload_len+1;

			while((received - i) > 0)
			{
				if(mavlink_parse_char(MAVLINK_COMM_0, tcpRxBuffer[i], &in_msg, &status))
				{
					switch (in_msg.msgid) {
					case MAVLINK_MSG_ID_HEARTBEAT:
					{
//						uart_send_str_ln(USART1, "HEARTBEAT RECEIVED!");
						blinkParam.period = 500;
						break;
					}
					case MAVLINK_MSG_ID_ECO_BMP180:
					{
						client_station[(in_msg.sysid - 1)].BMP180_data.temterature = 	mavlink_msg_eco_bmp180_get_temperature(&in_msg);
						client_station[(in_msg.sysid - 1)].BMP180_data.altitude = 		mavlink_msg_eco_bmp180_get_altitude(&in_msg);
						client_station[(in_msg.sysid - 1)].BMP180_data.pressure = 		mavlink_msg_eco_bmp180_get_presssure(&in_msg);

						lcdParam.temp = 	client_station[(in_msg.sysid - 1)].BMP180_data.temterature;
						lcdParam.alt = 		client_station[(in_msg.sysid - 1)].BMP180_data.altitude;
						lcdParam.press = 	client_station[(in_msg.sysid - 1)].BMP180_data.pressure;

//						uart_send_str_ln(USART1, "BMP180 data RECEIVED!");

						break;
					}
					case MAVLINK_MSG_ID_ECO_MAX44009:
					{
						client_station[(in_msg.sysid - 1)].MAX44009_data.lux_ambilight_1 = mavlink_msg_eco_max44009_get_lux_ambilight_1(&in_msg);
						client_station[(in_msg.sysid - 1)].MAX44009_data.lux_ambilight_2 = mavlink_msg_eco_max44009_get_lux_ambilight_2(&in_msg);

						lcdParam.lux_ambilight_1 = 	client_station[(in_msg.sysid - 1)].MAX44009_data.lux_ambilight_1;
						lcdParam.lux_ambilight_2 = 	client_station[(in_msg.sysid - 1)].MAX44009_data.lux_ambilight_2;

//						uart_send_str_ln(USART1, "MAX44009 data RECEIVED!");

						break;
					}
					case MAVLINK_MSG_ID_ECO_SHT11:
					{
						client_station[(in_msg.sysid - 1)].SHT11_data.temperature = mavlink_msg_eco_sht11_get_temperature(&in_msg);
						client_station[(in_msg.sysid - 1)].SHT11_data.humidity = mavlink_msg_eco_sht11_get_humidity(&in_msg);
						client_station[(in_msg.sysid - 1)].SHT11_data.dewpoint = mavlink_msg_eco_sht11_get_dowpoint(&in_msg);

						lcdParam.sht_temp = client_station[(in_msg.sysid - 1)].SHT11_data.temperature;
						lcdParam.humidity = client_station[(in_msg.sysid - 1)].SHT11_data.humidity;
						lcdParam.dewpoint = client_station[(in_msg.sysid - 1)].SHT11_data.dewpoint;

//						uart_send_str_ln(USART1, "SHT11 data RECEIVED!");

						break;
					}
					default:
						break;
					}
				}
				i++;
			}
		}
//			status.parse_state = 0;
		esp8266ClearBuffer();
		esp8266ClearTcpBuffer();
		receiveState = 0;
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

void initLCD(void)
{
	// LCD initialization: address - 0x27, rows - 2, colnums - 16

//	LCDI2C_init(0x27, 16, 2);
}

#pragma GCC diagnostic pop
