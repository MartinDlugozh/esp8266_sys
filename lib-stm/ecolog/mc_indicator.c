/*
 * mc_indicator.c
 *
 * Created: 13.11.2017 10:00:19
 *  Author: Dr. Saldon
 *
 *
 *  LED Indicator convinence functions
 *  Used on ECOLOG motherboard v.0.1a
 */

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
#include "mc_indicator.h"

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

void led_indicator_init(void)
{
    /* JTAG-DP Disabled and SW-DP Enabled */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // needed for use PB3, PB4, PA15

	GPIO_InitTypeDef led_gpio;

	GPIO_StructInit(&led_gpio);
	led_gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	led_gpio.GPIO_Speed = GPIO_Speed_2MHz;
	led_gpio.GPIO_Pin = LED_PIN_1;
	GPIO_Init(LED_PORT_1, &led_gpio);

	led_gpio.GPIO_Pin = LED_PIN_2;
	GPIO_Init(LED_PORT_2, &led_gpio);

	led_gpio.GPIO_Pin = LED_PIN_3;
	GPIO_Init(LED_PORT_3, &led_gpio);

	led_gpio.GPIO_Pin = LED_PIN_4;
	GPIO_Init(LED_PORT_4, &led_gpio);

	GPIO_ResetBits(LED_PORT_1, LED_PIN_1);
	GPIO_ResetBits(LED_PORT_2, LED_PIN_2);
	GPIO_ResetBits(LED_PORT_3, LED_PIN_3);
	GPIO_ResetBits(LED_PORT_4, LED_PIN_4);
}

void set_led_status(uint8_t status)
{
	switch(status)
	{
	case 0:
	{
		GPIO_ResetBits(LED_PORT_1, LED_PIN_1);
		GPIO_ResetBits(LED_PORT_2, LED_PIN_2);
		GPIO_ResetBits(LED_PORT_3, LED_PIN_3);
		GPIO_ResetBits(LED_PORT_4, LED_PIN_4);
		break;
	}
	case 1:
	{
		GPIO_SetBits(LED_PORT_1, LED_PIN_1);
		GPIO_ResetBits(LED_PORT_2, LED_PIN_2);
		GPIO_ResetBits(LED_PORT_3, LED_PIN_3);
		GPIO_ResetBits(LED_PORT_4, LED_PIN_4);
		break;
	}
	case 2:
	{
		GPIO_SetBits(LED_PORT_1, LED_PIN_1);
		GPIO_SetBits(LED_PORT_2, LED_PIN_2);
		GPIO_ResetBits(LED_PORT_3, LED_PIN_3);
		GPIO_ResetBits(LED_PORT_4, LED_PIN_4);
		break;
	}
	case 3:
	{
		GPIO_SetBits(LED_PORT_1, LED_PIN_1);
		GPIO_SetBits(LED_PORT_2, LED_PIN_2);
		GPIO_SetBits(LED_PORT_3, LED_PIN_3);
		GPIO_ResetBits(LED_PORT_4, LED_PIN_4);
		break;
	}
	case 4:
	{
		GPIO_SetBits(LED_PORT_1, LED_PIN_1);
		GPIO_SetBits(LED_PORT_2, LED_PIN_2);
		GPIO_SetBits(LED_PORT_3, LED_PIN_3);
		GPIO_SetBits(LED_PORT_4, LED_PIN_4);
		break;
	}
	default:
		break;
	}
}

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

void toggle_led1(void)
{
    /* Read LED output (GPIOA PIN8) status */
    uint8_t led_bit = GPIO_ReadOutputDataBit(LED_PORT_1, LED_PIN_1);

    /* If LED output set, clear it */
    if(led_bit == (uint8_t)Bit_SET)
    {
        GPIO_ResetBits(LED_PORT_1, LED_PIN_1);
    }else/* If LED output clear, set it */
    {
        GPIO_SetBits(LED_PORT_1, LED_PIN_1);
    }
}

void toggle_led2(void)
{
    /* Read LED output (GPIOA PIN8) status */
    uint8_t led_bit = GPIO_ReadOutputDataBit(LED_PORT_2, LED_PIN_2);

    /* If LED output set, clear it */
    if(led_bit == (uint8_t)Bit_SET)
    {
        GPIO_ResetBits(LED_PORT_2, LED_PIN_2);
    }else/* If LED output clear, set it */
    {
        GPIO_SetBits(LED_PORT_2, LED_PIN_2);
    }
}
void toggle_led3(void)
{
    /* Read LED output (GPIOA PIN8) status */
    uint8_t led_bit = GPIO_ReadOutputDataBit(LED_PORT_3, LED_PIN_3);

    /* If LED output set, clear it */
    if(led_bit == (uint8_t)Bit_SET)
    {
        GPIO_ResetBits(LED_PORT_3, LED_PIN_3);
    }else/* If LED output clear, set it */
    {
        GPIO_SetBits(LED_PORT_3, LED_PIN_3);
    }
}
void toggle_led4(void)
{
    /* Read LED output (GPIOA PIN8) status */
    uint8_t led_bit = GPIO_ReadOutputDataBit(LED_PORT_4, LED_PIN_4);

    /* If LED output set, clear it */
    if(led_bit == (uint8_t)Bit_SET)
    {
        GPIO_ResetBits(LED_PORT_4, LED_PIN_4);
    }else/* If LED output clear, set it */
    {
        GPIO_SetBits(LED_PORT_4, LED_PIN_4);
    }
}
