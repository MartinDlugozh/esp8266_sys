#include "UART_freertos.h"

char * ftoa(double f, char * buf, int precision)
{
	char * ptr = buf;
	char * p = ptr;
	char * p1;
	char c;
	long intPart;

	// check precision bounds
	if (precision > MAX_PRECISION)
		precision = MAX_PRECISION;

	// sign stuff
	if (f < 0)
	{
		f = -f;
		*ptr++ = '-';
	}

	if (precision < 0)  // negative precision == automatic precision guess
	{
		if (f < 1.0) precision = 6;
		else if (f < 10.0) precision = 5;
		else if (f < 100.0) precision = 4;
		else if (f < 1000.0) precision = 3;
		else if (f < 10000.0) precision = 2;
		else if (f < 100000.0) precision = 1;
		else precision = 0;
	}

	// round value according the precision
	if (precision)
		f += rounders[precision];

	// integer part...
	intPart = f;
	f -= intPart;

	if (!intPart)
		*ptr++ = '0';
	else
	{
		// save start pointer
		p = ptr;

		// convert (reverse order)
		while (intPart)
		{
			*p++ = '0' + intPart % 10;
			intPart /= 10;
		}

		// save end pos
		p1 = p;

		// reverse result
		while (p > ptr)
		{
			c = *--p;
			*p = *ptr;
			*ptr++ = c;
		}

		// restore end pos
		ptr = p1;
	}

	// decimal part
	if (precision)
	{
		// place decimal point
		*ptr++ = '.';

		// convert
		while (precision--)
		{
			f *= 10.0;
			c = f;
			*ptr++ = '0' + c;
			f -= c;
		}
	}

	// terminating zero
	*ptr = 0;

	return ptr;
}

//Функция отправляет байт в UART
void uart_send_byte(USART_TypeDef* USARTx, uint8_t data)  {
	while(!(USARTx->SR & USART_SR_TXE));
	USARTx->DR=data;
	while (!(USARTx->SR & USART_SR_TC));
}

void uart_send_str(USART_TypeDef* USARTx, char* string) {
	  while(*string != 0)
	  {
		  uart_send_byte(USARTx, *string);
		  string++;
	  }
}

void uart_send_str_n(USART_TypeDef* USARTx, char* string, uint8_t n) {
	uint8_t ptr;
	ptr = 0;

	while((n - ptr) > 0)
	  {
		  uart_send_byte(USARTx, string[ptr]);
		  ptr++;
	  }
}

void uart_send_str_ln(USART_TypeDef* USARTx, char* string) {
	uart_send_str(USARTx, string);
	uart_send_byte(USARTx, '\r');
	uart_send_byte(USARTx, '\n');
}

void uart_send_num(USART_TypeDef* USARTx, uint32_t x)
{
	  char value[10]; //a temp array to hold results of conversion
	  int i = 0; //loop index

	  do
	  {
	    value[i++] = (char)(x % 10) + '0'; //convert integer to character
	    x /= 10;
	  } while(x);

	  while(i) //send data
	  {
		  uart_send_byte(USARTx, value[--i]);
	  }
}

void uart_send_float_str(USART_TypeDef* USARTx, float flt, int precision) {
	char buf[16];
	ftoa(flt, buf, precision);
	uart_send_str(USARTx, buf);
}

uint32_t uart_param2baudrate(uint16_t baudrate_param)
{
	if(baudrate_param == PARAM_BAUD_9){
		return BAUD_9;}
	if(baudrate_param == PARAM_BAUD_38){
		return BAUD_38;}
	if(baudrate_param == PARAM_BAUD_57){
		return BAUD_57;}
	if(baudrate_param == PARAM_BAUD_115){
		return BAUD_115;}
	if(baudrate_param == PARAM_BAUD_921){
		return BAUD_921;}

	return 57600;
}
