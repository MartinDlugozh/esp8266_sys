#ifndef MC_INDICATOR_H_
#define MC_INDICATOR_H_

#define LED_PORT_RED1 		GPIOB
#define LED_PORT_RED2	 	GPIOB
#define LED_PORT_GREEN1 	GPIOB
#define LED_PORT_GREEN2 	GPIOB

#define LED_PORT_1 			LED_PORT_GREEN1
#define LED_PORT_2 			LED_PORT_GREEN2
#define LED_PORT_3 			LED_PORT_RED1
#define LED_PORT_4 			LED_PORT_RED2

#define LED_PIN_GREEN1 		GPIO_Pin_12
#define LED_PIN_GREEN2 		GPIO_Pin_13
#define LED_PIN_RED1 		GPIO_Pin_14
#define LED_PIN_RED2 		GPIO_Pin_15

#define LED_PIN_1 			LED_PIN_GREEN1
#define LED_PIN_2 			LED_PIN_GREEN2
#define LED_PIN_3 			LED_PIN_RED1
#define LED_PIN_4 			LED_PIN_RED2

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
#endif /* MC_INDICATOR_H_ */
