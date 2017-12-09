/*-----------------------------------------------------------------------------
*** ESP Communication test program - Server
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
typedef struct BlinkTaskParam_t {	// blink task parameters
	 uint32_t period; 				// task execution period, ticks (milliseconds)
} BlinkTaskParam;

BlinkTaskParam blinkParam;

/* Структура, содержащая передаваемую в задачу информацию */
typedef struct TaskParam_t {
	 char foo[16];
	 uint32_t period; /* период, миллисекунды*/
} TaskParam;

//TaskParam tp1;
TaskParam lcdParam;

uint8_t btn_state_curr = 0;
uint8_t btn_state_prev = 0;

uint8_t led_cnt = 0;

char lcd_buff[16];

uint16_t transctions_count = 0;

SemaphoreHandle_t xUsart1RxInterruptSemaphore;		// семафор для вызова задачи процессинга данных, принятых в прерывании USART



/**
 * Increment(decrement) onboard led number
 *
 * This is board specific method. Use with ECOLOG motherboard only.
 */
void led_ppmm(uint8_t *led_cnt , uint8_t inc)
{
	if(inc == 1)
	{
		if(*led_cnt < 5)
		{
			*led_cnt = *led_cnt + 1;
		}else{
			*led_cnt = 1;
		}
		set_led_status((*led_cnt-1));
	}else if(inc == 0){
		if(*led_cnt > 1)
		{
			*led_cnt = *led_cnt - 1;
		}else{
			*led_cnt = 5;
		}
		set_led_status((*led_cnt-1));
	}
}



/**
 * Toggle onboard led (PC13 on BluePill)
 */
void led_toggle()
{
	uint8_t led_bit;
    /* Read LED output (GPIOA PIN8) status */
    led_bit = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13);

    /* If LED output set, clear it */
    if(led_bit == (uint8_t)Bit_SET)
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    }else/* If LED output clear, set it */
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
    }
}

void buttonPressedHandle(void)
{
	// send increment over TCP
	unsigned char c[2] = {0, 0};
	c[0] = '+';
	esp8266TcpSend(c, 1, 1);
//	led_ppmm(&led_cnt, 1); // increment led counter
}

/**
 * OS Task: Blinker
 *
 * Onboard LED blinking (PC13)
 * Uses led_toggle() method
 */
void vBlinker (void *pvParameters)
{
	volatile BlinkTaskParam *pxTaskParam;
	pxTaskParam = (BlinkTaskParam *) pvParameters;

	do{
		led_toggle();
		vTaskDelay(pxTaskParam->period);
	}while(1);
	vTaskDelete(NULL);
}

void vLCD_Update(void *pvParameters)
{
	volatile TaskParam *pxTaskParam;
	TickType_t pxPreviousWakeTime;
	pxTaskParam = (TaskParam *) pvParameters;

	pxPreviousWakeTime = xTaskGetTickCount();
	do{
		// Send debug information to USART1
		uart_send_num(USART1, millis());
		uart_send_str(USART1, " ");
		uart_send_str_ln(USART1, (char*)pxTaskParam->foo);

		// Prepare LCD
		SSD1306_Fill(SSD1306_COLOR_BLACK); //

		// Print information on LCD
		itoa(millis(), lcd_buff, 10);
		SSD1306_GotoXY(0, 0);
		SSD1306_Puts("T: ", &Font_11x18, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(30, 0);
		SSD1306_Puts(lcd_buff, &Font_11x18, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(0, 19);
		SSD1306_Puts((char*)pxTaskParam->foo, &Font_11x18, SSD1306_COLOR_WHITE);
		itoa(transctions_count, lcd_buff, 10);
		SSD1306_GotoXY(0, 38);
		SSD1306_Puts(lcd_buff, &Font_11x18, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();

		vTaskDelayUntil(&pxPreviousWakeTime, pxTaskParam->period);
	}while(1);
	vTaskDelete(NULL);
}

/**
 * OS Task: Button check
 *
 * Checks state of the onboard pushbutton (PB11)
 * Uses buttonPressedHandle() method
 */
void vButtonCheck(void *pvParameters)
{
	TickType_t pxPreviousWakeTime;
	uint8_t btn_state_curr = 0;		// previous button state
	uint8_t btn_state_prev = 0;		// current button state
	uint16_t period = 20;			// checking period

	pxPreviousWakeTime = xTaskGetTickCount();
	do{
		btn_state_prev = btn_state_curr;		// shift current state to previous
		btn_state_curr = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);		// get new state
		if((btn_state_curr != 0) && (btn_state_curr != btn_state_prev))	// if state had changed to "PRESSED"
		{
			period = 400;		// change checking period (debounce)
			buttonPressedHandle();
		}else if((btn_state_curr != 0) && (btn_state_curr == btn_state_prev)){
			period = 200;		// change checking period (debounce)
			buttonPressedHandle();
		}else{
			period = 20;		// change checking period back for normal reaction
		}

		vTaskDelayUntil(&pxPreviousWakeTime, period);
	}while(1);
	vTaskDelete(NULL);
}

/**
 * OS Task: ESP8266 Task
 */
void vEspMain(void *pvParameters)
{
	TickType_t pxPreviousWakeTime;
	uint16_t period = 500;

	uint8_t wifi_connection = 0;
	uint8_t server_active = 0;

	pxPreviousWakeTime = xTaskGetTickCount();

	if(esp8266SetMode(ESP8266_MODE_AP))
	{
		wifi_connection = 1;
	}

	do{
		if(!server_active)
		{
			if(!wifi_connection)
			{
				if(esp8266SetMode(ESP8266_MODE_AP))
				{
					wifi_connection = 1;
					strcpy(lcdParam.foo, "AP");
					USART1_SEND("ESP configured as AP");
				}
			}
			if(wifi_connection && !server_active)
			{
				esp8266SetMux(1);
				if(esp8266ServerCreate(1500) != 0)
				{
					server_active = 1;
					period = 20;
					blinkParam.period = 500;
					strcpy(lcdParam.foo, "Server");
					USART1_SEND("Server is ready for incoming connections");
				}
			}
		}

		// TODO: try to move it into the IDLE task
		// TCP input parser
		if((server_active == TRUE) && (esp8266ReadTcpData() == TRUE))
		{
			transctions_count++;

			unsigned char c[3];
			tcp_getdata(c, 2);
			switch(c[0])
			{
			case '+':
			{
				led_ppmm(&led_cnt, 1);
				break;
			}
			case '-':
			{
				led_ppmm(&led_cnt, 0);
				break;
			}
			default:
				break;
			}
		}
		// END TCP input parser

		vTaskDelayUntil(&pxPreviousWakeTime, period);
	}while(1);
	vTaskDelete(NULL);
}

/*-----------------------------------------------------------------------------
ENTRY POINT
-----------------------------------------------------------------------------*/
int main(void)
{
	blinkParam.period = 150;
	strcpy(lcdParam.foo, " Wait...");
	lcdParam.period = 500;

	RCC_DeInit();							// Reset RCC
	RCC_HSEConfig(RCC_HSE_ON);				// Enable HSE

	if(RCC_WaitForHSEStartUp() == SUCCESS) 	// Wait till HSE is ready
	{
		RCC_HCLKConfig(RCC_SYSCLK_Div1);	// HCLK = SYSCLK
		RCC_PCLK2Config(RCC_HCLK_Div1);		// PCLK2 = HCLK
		RCC_PCLK1Config(RCC_HCLK_Div2);		// PCLK1 = HCLK/2
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); 	// PLLCLK = 8MHz * 9 = 72 MHz
		RCC_PLLCmd(ENABLE);					// Enable PLL
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)	// Wait till PLL is ready
		{	}
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);	// Select PLL as system clock source
		while(RCC_GetSYSCLKSource() != 0x08)		// Wait till PLL is used as system clock source
		{	}
	}
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
				RCC_APB2Periph_USART1 |
				RCC_APB2Periph_AFIO,
				ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_IPD;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	gpio.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &gpio);

	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	gpio.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &gpio);

	led_indicator_init();

	/*********************************************************************************************/
	/******************************* USART CONFIGURATION *****************************************/
	/*********************************************************************************************/
	GPIO_StructInit(&gpio);							/*** Configure UART Rx GPIO ***/
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_10;
	GPIO_Init(GPIOA, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_9;	/*** Configure UART Tx GPIO ***/
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);

	USART_InitTypeDef uart_struct;
	uart_struct.USART_BaudRate            = 115200;
	uart_struct.USART_WordLength          = USART_WordLength_8b;
	uart_struct.USART_StopBits            = USART_StopBits_1;
	uart_struct.USART_Parity              = USART_Parity_No ;
	uart_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	uart_struct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &uart_struct);
	USART_Cmd(USART1, ENABLE);
	USART_Init(USART2, &uart_struct);
	USART_Cmd(USART2, ENABLE);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);    // Enable RXNE interrupt
	NVIC_InitTypeDef nvic;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);/*!< 0 bits for pre-emption priority, 4 bits for subpriority */
	nvic.NVIC_IRQChannel = USART1_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 15;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	NVIC_EnableIRQ(USART1_IRQn);    // Enable USART1 global interrupt

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);    // Enable RXNE interrupt
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);/*!< 0 bits for pre-emption priority, 4 bits for subpriority */
	nvic.NVIC_IRQChannel = USART2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 15;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	NVIC_EnableIRQ(USART2_IRQn);    // Enable USART1 global interrupt

	vSemaphoreCreateBinary(xUsart1RxInterruptSemaphore);
	vSemaphoreCreateBinary(xUsartRxInterruptSemaphore_esp);

	SSD1306_Init();
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	SSD1306_GotoXY(30, 0);
	SSD1306_Puts("Hello", &Font_11x18, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();

	if((pdTRUE != xTaskCreate(vBlinker,"Blinker", configMINIMAL_STACK_SIZE, &blinkParam, tskIDLE_PRIORITY + 1, NULL)) ||
			(pdTRUE != xTaskCreate(vLCD_Update,"LCD", configMINIMAL_STACK_SIZE, &lcdParam, tskIDLE_PRIORITY + 1, NULL)) ||
			(pdTRUE != xTaskCreate(vButtonCheck,"Button", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL)) ||
			(pdTRUE != xTaskCreate(vEspMain,"ESP", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL)))
	{
		ERROR_ACTION();
	}

	vTaskStartScheduler();

	do{ }while(1);
	return 0;
}

void vApplicationIdleHook(void)
{
	do{
	}while(1);
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

#pragma GCC diagnostic pop
