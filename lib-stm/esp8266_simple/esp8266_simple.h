/*
 * esp8266_simple.h
 *
 *  Created on: Jan 26, 2018
 *      Author: Dr. Saldon
 */

#ifndef SIMPLE_ESP_H_
#define SIMPLE_ESP_H_

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "common_freertos.h"	// Common convinence functions for FreeRTOS
#include "usart_freertos.h"		// USART convinence functions for FreeRTOS

#include <ctype.h>

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
// USART used with ESP8266

#define ESP8266_USART 				USART2

// Socket diagnostic
#define SOCKET_ERROR 				1
#define INVALID_SOCKET 				1

#define ESP8266_RX_BUFFER_LEN 		266
#define TCP_RX_BUFFER_LEN 			256

#define COMMAND_RESPONSE_TIMEOUT 	500
#define COMMAND_RESPONSE_DELAY 		5
#define COMMAND_PING_TIMEOUT 		3000
#define WIFI_CONNECT_TIMEOUT 		30000
#define COMMAND_RESET_TIMEOUT 		5000
#define CLIENT_CONNECT_TIMEOUT 		5000

#define ESP8266_MAX_SOCK_NUM 		5
#define ESP8266_SOCK_NOT_AVAIL 		255

#define CHARISNUM(x)                        ((x) >= '0' && (x) <= '9')
#define CHARISHEXNUM(x)                     (((x) >= '0' && (x) <= '9') || ((x) >= 'a' && (x) <= 'f') || ((x) >= 'A' && (x) <= 'F'))
#define CHARTONUM(x)                        ((x) - '0')
#define CHARHEXTONUM(x)                     (((x) >= '0' && (x) <= '9') ? ((x) - '0') : (((x) >= 'a' && (x) <= 'f') ? ((x) - 'a' + 10) : (((x) >= 'A' && (x) <= 'F') ? ((x) - 'A' + 10) : 0)))
#define ISVALIDASCII(x)                     (((x) >= 32 && (x) <= 126) || (x) == '\r' || (x) == '\n')

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
static const char RESPONSE_PROMPT[] = 			">";
static const char RESPONSE_OK[] = 				"OK";
static const char RESPONSE_SEND_OK[] = 			"SEND OK";
static const char RESPONSE_ERROR[] = 			"ERROR";
static const char RESPONSE_FAIL[] = 			"FAIL";
static const char RESPONSE_READY[] = 			"READY!";
static const char RESPONSE_NOCHANGE[] =			"no change";
static const char RESPONSE_ALREADY[] = 			"ALREADY";
static const char RESPONSE_TCP_RECEIVED[] = 	"+IPD,";
static const char ESP8266_TEST[] = 				"AT";				// Test AT startup
static const char ESP8266_AT[] = 				"AT";
static const char ESP8266_RESET[] = 			"+RST"; 		// Restart module
static const char ESP8266_VERSION[] = 			"+GMR"; 		// View version info
static const char ESP8266_SLEEP[] = 			"+GSLP"; 		// Enter deep-sleep mode
static const char ESP8266_ECHO_ENABLE[] = 		"E1"; 			// AT commands echo
static const char ESP8266_ECHO_DISABLE[] = 		"E0"; 			// AT commands echo
static const char ESP8266_RESTORE[] = 			"+RESTORE"; 	// Factory reset
static const char ESP8266_UART[] = 				"+UART"; 		// UART configuration
static const char ESP8266_WIFI_MODE[] = 		"+CWMODE"; 		// WiFi mode (sta/AP/sta+AP)
static const char ESP8266_CONNECT_AP[] = 		"+CWJAP"; 		// Connect to AP
static const char ESP8266_LIST_AP[] = 			"+CWLAP"; 		// List available AP's
static const char ESP8266_DISCONNECT[] = 		"+CWQAP"; 		// Disconnect from AP
static const char ESP8266_AP_CONFIG[] = 		"+CWSAP"; 		// Set softAP configuration
static const char ESP8266_STATION_IP[] = 		"+CWLIF"; 		// List station IP's connected to softAP
static const char ESP8266_DHCP_EN[] = 			"+CWDHCP"; 		// Enable/disable DHCP
static const char ESP8266_AUTO_CONNECT[] = 		"+CWAUTOCONN"; 	// Connect to AP automatically
static const char ESP8266_SET_STA_MAC[] = 		"+CIPSTAMAC"; 	// Set MAC address of station
static const char ESP8266_GET_STA_MAC[] = 		"+CIPSTAMAC"; 	// Get MAC address of station
static const char ESP8266_SET_AP_MAC[] = 		"+CIPAPMAC"; 	// Set MAC address of softAP
static const char ESP8266_SET_STA_IP[] = 		"+CIPSTA"; 		// Set IP address of ESP8266 station
static const char ESP8266_SET_AP_IP[] = 		"+CIPAP"; 		// Set IP address of ESP8266 softAP
static const char ESP8266_TCP_STATUS[] = 		"+CIPSTATUS"; 	// Get connection status
static const char ESP8266_TCP_CONNECT[] = 		"+CIPSTART"; 	// Establish TCP connection or register UDP port
static const char ESP8266_TCP_SEND[] = 			"+CIPSEND"; 	// Send Data
static const char ESP8266_TCP_CLOSE[] = 		"+CIPCLOSE"; 	// Close TCP/UDP connection
static const char ESP8266_GET_LOCAL_IP[] = 		"+CIFSR"; 		// Get local IP address
static const char ESP8266_TCP_MULTIPLE[] = 		"+CIPMUX"; 		// Set multiple connections mode
static const char ESP8266_SERVER_CONFIG[] = 	"+CIPSERVER"; 	// Configure as server
static const char ESP8266_TRANSMISSION_MODE[] = "+CIPMODE"; 	// Set transmission mode
static const char ESP8266_SET_SERVER_TIMEOUT[] = "+CIPSTO"; 	// Set timeout when ESP8266 runs as TCP server
static const char ESP8266_PING[] = 				"+PING"; 		// Function PING
static const char ESP8266_PINMODE[] = 			"+PINMODE"; 	// Set GPIO mode (input/output)
static const char ESP8266_PINWRITE[] = 			"+PINWRITE"; 	// Write GPIO (high/low)
static const char ESP8266_PINREAD[] = 			"+PINREAD"; 	// Read GPIO digital value

typedef enum{
	ESP_OK = 0,
	ESP_ERROR,
	ESP_TIMEOUT,
	ESP_BUFFER_OVERFLOW,
	ESP_OTHER,
}esp8266_result;

typedef enum{
	ESP_WAIT_IPD = 0,
	ESP_WAIT_HEADER,
	ESP_RECEIVE_TCP,
	ESP_TCP_RECEPTION_DONE,
}esp8266_ipd;

typedef enum {
	ESP8266_CMD_BAD = -5,
	ESP8266_RSP_MEMORY_ERR = -4,
	ESP8266_RSP_FAIL = -3,
	ESP8266_RSP_UNKNOWN = -2,
	ESP8266_RSP_TIMEOUT = -1,
	ESP8266_RSP_SUCCESS = 0
}esp8266_cmd_rsp;

typedef enum {
	ESP8266_MODE_STA = 1,
	ESP8266_MODE_AP = 2,
	ESP8266_MODE_STAAP = 3
}esp8266_wifi_mode;

typedef enum {
	ESP8266_CMD_QUERY,
	ESP8266_CMD_SETUP,
	ESP8266_CMD_EXECUTE
}esp8266_command_type;

typedef enum  {
	ESP8266_ECN_OPEN,
	ESP8266_ECN_WPA_PSK,
	ESP8266_ECN_WPA2_PSK,
	ESP8266_ECN_WPA_WPA2_PSK
}esp8266_encryption;

typedef enum  {
	ESP8266_STATUS_GOTIP = 2,
	ESP8266_STATUS_CONNECTED = 3,
	ESP8266_STATUS_DISCONNECTED = 4,
	ESP8266_STATUS_NOWIFI = 5
}esp8266_connect_status;

typedef enum  {
	AVAILABLE = 0,
	TAKEN = 1,
}esp8266_socket_state;

typedef enum  {
	ESP8266_TCP,
	ESP8266_UDP,
	ESP8266_TYPE_UNDEFINED
}esp8266_connection_type;

typedef enum  {
	ESP8266_CLIENT,
	ESP8266_SERVER
}esp8266_tetype;

char esp8266RxBuffer[ESP8266_RX_BUFFER_LEN];
char tcpRxBuffer[TCP_RX_BUFFER_LEN];
char* bufferHead = esp8266RxBuffer; 			// Holds position of latest byte placed in buffer.
char* tcpBufferHead = tcpRxBuffer;
char* tcpDataProcPtr = esp8266RxBuffer;
uint8_t current_connection = 0;

uint8_t receiveState = 0;
uint16_t unprocessed_bytes = 0;
uint8_t payload_len = 0;
uint8_t transmission_free = 1;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void esp8266ClearBuffer(void);
void esp8266ClearTcpBuffer(void);
uint8_t esp8266RxBufferAvailable(void);
void esp8266SendCommand(const char * cmd, esp8266_command_type type, char * params);
uint8_t esp8266SearchBuffer(const char * test);
uint8_t esp8266ReadForResponses(const char *pass, const char *fail, const char *other, unsigned int timeout);
uint8_t esp8266SetMux(uint8_t mux);
uint8_t esp8266Test(void);
uint8_t esp8266SetMode(esp8266_wifi_mode mode);
uint8_t esp8266Connect(const char * ssid, const char * pwd);
uint8_t esp8266TcpConnect(const char* destination, const char* port, const char* link);
uint8_t esp8266TcpSend(uint8_t *buf, uint16_t size, uint8_t link);
uint8_t esp8266TcpClose(uint8_t link);
uint8_t esp8266ListConectedStaions(void);
uint8_t esp8266ServerCreate(uint16_t port, uint8_t enable);
uint8_t esp8266GetLinkID(void);
void esp8266RxCallBack(char c);

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

/**
 * Get decimal number from string
 */
int32_t getDecNum(char* str, char **restrict endPtr, uint8_t max_passed, esp8266_result* result){
	int32_t number = 0;
	uint8_t passed = 0;

	if(str != NULL){
		while(!isdigit(*str)){
			str++;
			passed++;
			if(passed >= max_passed){
				*result = ESP_ERROR;
				return 0;
			}
		}
		*result = ESP_OK;
		number = strtol(str, endPtr, 10);
		return number;
	}

	*result = ESP_ERROR;
	return 0;
}

/* Parses and returns number from string */
int32_t ParseNumber(const char* ptr, uint8_t* cnt) {
    uint8_t minus = 0, i = 0;
    int32_t sum = 0;

    uint8_t res = 0;
    do{
    	if(!isdigit((int)*ptr)){
    		if (*ptr == '-') {                                		/* Check for minus character */
    			minus = 1;
    			res = 1;
    		}else{
    			ptr++;
    		}
    	}else{
    		res = 1;
    	}

    }while(!res);


    while(isdigit((int)*ptr)){
    	sum = 10 * sum + CHARTONUM(*ptr);
    	ptr++;
    	i++;
    }

    if (cnt) {                                		        /* Save number of characters used for number */
        *cnt = i;
    }
    if (minus) {                                    		/* Minus detected */
        return 0 - sum;
    }
    return sum;                                       		/* Return number */
}

void esp8266ClearBuffer(void)
{
	memset(esp8266RxBuffer, 0, ESP8266_RX_BUFFER_LEN);
	bufferHead = esp8266RxBuffer;
}

void esp8266ClearTcpBuffer(void)
{
	memset(tcpRxBuffer, 0, TCP_RX_BUFFER_LEN);
	tcpBufferHead = tcpRxBuffer;
}

uint8_t esp8266RxBufferAvailable(void)
{
	return (bufferHead != esp8266RxBuffer) ? ESP_OK:ESP_ERROR;
}

void esp8266SendCommand(const char * cmd, esp8266_command_type type, char * params)
{
	esp8266ClearBuffer();	// Clear the class receive buffer (esp8266RxBuffer)
	uart_send_str(ESP8266_USART, (char *)ESP8266_AT); // "AT"
	uart_send_str(ESP8266_USART, (char *)cmd);
	if (type == ESP8266_CMD_QUERY)
		uart_send_str(ESP8266_USART, "?");
	else if (type == ESP8266_CMD_SETUP)
	{
		uart_send_str(ESP8266_USART, "=");
		uart_send_str(ESP8266_USART, (char *)params);
	}
	uart_send_str_ln(ESP8266_USART, "");
}

uint8_t esp8266SearchBuffer(const char * test)
{
	int bufferLen = strlen((const char *)esp8266RxBuffer);
	if (bufferLen < ESP8266_RX_BUFFER_LEN)	// If our buffer isn't full, just do an strstr
	{
		if(strstr((const char *)esp8266RxBuffer, test) != NULL){
			return ESP_OK;
		}else{
			return ESP_ERROR;
		}
	}else{
		return ESP_BUFFER_OVERFLOW;
	}

	return ESP_ERROR;
}

uint8_t esp8266ReadForResponse(const char * rsp, unsigned int timeout)
{
	uint32_t timeIn = millis();	// Timestamp coming into function
	while ((timeIn + timeout) > millis()) // While we haven't timed out
	{
		if (esp8266RxBufferAvailable() == ESP_OK) // If data is available on ESP8266_USART RX
		{
			if (esp8266SearchBuffer(rsp) == ESP_OK)	// Search the buffer for goodRsp
				return ESP_OK;
		}
		vTaskDelay(COMMAND_RESPONSE_DELAY);
	}
	return ESP_ERROR; // Return the timeout error code
}

uint8_t esp8266ReadForResponses(const char *pass, const char *fail, const char *other, unsigned int timeout)
{
	uint8_t result = ESP_ERROR;
	unsigned long timeIn = millis();	// Timestamp coming into function
	while ((timeIn + timeout) > millis()) // While we haven't timed out
	{
		if (esp8266RxBufferAvailable() == ESP_OK) // If data is available on UART RX
		{
			result = esp8266SearchBuffer(pass);
			if (result == ESP_OK)	// Search the buffer for goodRsp
				return ESP_OK;	// Return how number of chars read
			result = esp8266SearchBuffer(fail);
			if (result == ESP_OK)
				return ESP_ERROR;
			if(other != NULL){
				result = esp8266SearchBuffer(other);
				if (result == ESP_OK)
					return ESP_OTHER;
			}
		}
		vTaskDelay(COMMAND_RESPONSE_DELAY);
	}
	return ESP_TIMEOUT;
}

uint8_t esp8266SetTrans(uint8_t transparent)
{
	uint8_t result = ESP_ERROR;
	char params[2] = {0, 0};
	params[0] = (transparent > 0) ? '1' : '0';
	esp8266SendCommand(ESP8266_TRANSMISSION_MODE, ESP8266_CMD_SETUP, params);
	result = esp8266ReadForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
	return result;
}

uint8_t esp8266SetMux(uint8_t mux)
{
	uint8_t result = ESP_ERROR;
	char params[2] = {0, 0};
	params[0] = (mux > 0) ? '1' : '0';
	esp8266SendCommand(ESP8266_TCP_MULTIPLE, ESP8266_CMD_SETUP, params);
	result = esp8266ReadForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
	return result;
}

uint8_t esp8266Test(void)
{
	esp8266ClearBuffer();
	esp8266SendCommand(ESP8266_TEST, ESP8266_CMD_EXECUTE, 0);
	if(esp8266ReadForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT))
		return ESP_OK;
	return ESP_ERROR;
}

uint8_t esp8266SetMode(esp8266_wifi_mode mode)
{
	esp8266ClearBuffer();
	char modeChar[2] = {0, 0};
	sprintf(modeChar, "%d", mode);
	esp8266SendCommand(ESP8266_WIFI_MODE, ESP8266_CMD_SETUP, modeChar);
	return esp8266ReadForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
}

uint8_t esp8266Connect(const char * ssid, const char * pwd)
{
	// The ESP8266 can be set to one of three modes:
	//  1 - ESP8266_MODE_STA - Station only
	//  2 - ESP8266_MODE_AP - Access point only
	//  3 - ESP8266_MODE_STAAP - Station/AP combo
	uint8_t result = ESP_ERROR;
	if((result = esp8266SetMode(ESP8266_MODE_STA)) == ESP_OK)
	{
		esp8266ClearBuffer();
		uart_send_str(ESP8266_USART, (char *)ESP8266_AT);
		uart_send_str(ESP8266_USART, (char *)ESP8266_CONNECT_AP);
		uart_send_str(ESP8266_USART, "=\"");
		uart_send_str(ESP8266_USART, (char *)ssid);
		uart_send_str(ESP8266_USART, "\"");
		if (pwd != NULL)
		{
			uart_send_str(ESP8266_USART, ",");
			uart_send_str(ESP8266_USART, "\"");
			uart_send_str(ESP8266_USART, (char *)pwd);
			uart_send_str(ESP8266_USART, "\"");
		}
		uart_send_str_ln(ESP8266_USART, "");
		result = esp8266ReadForResponses(RESPONSE_OK, RESPONSE_FAIL, NULL, WIFI_CONNECT_TIMEOUT);
		return result;
	}
	return ESP_ERROR;
}

uint8_t esp8266TcpConnect(const char* destination, const char* port, const char* link)
{
	uint8_t result = ESP_ERROR;
	esp8266ClearBuffer();
	uart_send_str(ESP8266_USART, (char *)ESP8266_AT);
	uart_send_str(ESP8266_USART, (char *)ESP8266_TCP_CONNECT);
	uart_send_str(ESP8266_USART, "=");
	uart_send_str(ESP8266_USART, (char *)link);
	uart_send_str(ESP8266_USART, ",");
	uart_send_str(ESP8266_USART, "\"TCP\",");
	uart_send_str(ESP8266_USART, "\"");
	uart_send_str(ESP8266_USART, (char *)destination);
	uart_send_str(ESP8266_USART, "\",");
	uart_send_str(ESP8266_USART, (char *)port);
	uart_send_str_ln(ESP8266_USART, "");
	// Example good: CONNECT\r\n\r\nOK\r\n
	// Example bad: DNS Fail\r\n\r\nERROR\r\n
	// Example meh: ALREADY CONNECTED\r\n\r\nERROR\r\n
	if((result = esp8266ReadForResponses(RESPONSE_OK, RESPONSE_ERROR, RESPONSE_ALREADY, CLIENT_CONNECT_TIMEOUT)) == ESP_OK){
		return ESP_OK;
	}else if(result == ESP_OTHER){
		// We may see "ERROR", but be "ALREADY CONNECTED".
		// Search for "ALREADY", and return success if we see it.
//		if((result = esp8266SearchBuffer(RESPONSE_ALREADY)) == ESP_OK){
//			return ESP_OK;
//		}
		return ESP_OK;
	}
	// Return 1 on successful (new) connection
	return ESP_ERROR;
}

uint8_t esp8266TcpSend(uint8_t *buf, uint16_t size, uint8_t link)
{
	if(transmission_free == 1){
		transmission_free = 0;
		uint8_t result = ESP_ERROR;
		uint8_t *p = buf;
		char params[8];
		if (size > 2048)
			return ESP_ERROR; //ESP8266_CMD_BAD
		sprintf(params, "%d,%d", link, size); // sprintf(params, "%d,%d", link, size+1);
		esp8266SendCommand(ESP8266_TCP_SEND, ESP8266_CMD_SETUP, params); // +CIPSENDBUF

		result = esp8266ReadForResponses(RESPONSE_PROMPT, RESPONSE_ERROR, NULL, COMMAND_RESPONSE_TIMEOUT);
		if(result == ESP_OK)
		{
			esp8266ClearBuffer();
			uart_send_str_n(ESP8266_USART, (char *)p, size);
			uart_send_byte(ESP8266_USART, '\0'); // size+1
			result = esp8266ReadForResponse(RESPONSE_SEND_OK, COMMAND_RESPONSE_TIMEOUT); 	// "SEND OK"
			if (result == ESP_OK){
				transmission_free = 1;
				return ESP_OK;
			}
		}
		transmission_free = 1;
		return ESP_ERROR;
	}
}

uint8_t esp8266TcpClose(uint8_t link)
{
	uint8_t rc = ESP_ERROR;
	char params[8];
	sprintf(params, "%d", link);
	esp8266SendCommand(ESP8266_TCP_CLOSE, ESP8266_CMD_SETUP, params);
	esp8266ClearBuffer();
	rc = esp8266ReadForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
	return rc;
}

// TODO: finish implementation
uint8_t esp8266ListConectedStaions(void)
{
	esp8266SendCommand(ESP8266_STATION_IP, ESP8266_CMD_EXECUTE, 0);
	esp8266ClearBuffer();
	if(esp8266ReadForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT))
		return ESP_OK;
	return ESP_ERROR;
}

uint8_t esp8266ServerCreate(uint16_t port, uint8_t enable)
{
	char params[8];
	if(enable){
		sprintf(params, "%d,%d", 1, port);
	}else{
		sprintf(params, "%d", 0);
	}

	esp8266SendCommand(ESP8266_SERVER_CONFIG, ESP8266_CMD_SETUP, params);
	esp8266ClearBuffer();
	if(esp8266ReadForResponses(RESPONSE_OK, RESPONSE_ERROR, RESPONSE_NOCHANGE, COMMAND_RESPONSE_TIMEOUT) == ESP_OK){
		return ESP_OK;
	}
	return ESP_ERROR;
}

// Need chek!
uint8_t esp8266GetLinkID(void)
{
	uint8_t id = 0;
	char *p;
	esp8266SendCommand(ESP8266_TCP_STATUS, ESP8266_CMD_EXECUTE, 0);
	uint32_t timeIn = millis();	// Timestamp coming into function
	while (timeIn + COMMAND_RESPONSE_TIMEOUT > millis()) // While we haven't timed out
	{
		if (esp8266RxBufferAvailable() == ESP_OK)
		{
			p = strstr((const char *)esp8266RxBuffer, ESP8266_TCP_STATUS);
			p+=11;

			char cid;
			cid = *p;
			id = atoi(&cid);
		}
	}
	return id;
}

/* Check for buffer overflow */
uint8_t ESP8266_CheckOverflow(void){
	if((bufferHead - esp8266RxBuffer) >= ESP8266_RX_BUFFER_LEN)	{
		esp8266ClearBuffer();
		unprocessed_bytes = 0;
		return ESP_BUFFER_OVERFLOW;
	}
	return ESP_OK;
}

/* Check for TCP buffer overflow */
uint8_t ESP8266_CheckTCPOverflow(void){
	if((tcpBufferHead - tcpRxBuffer) >= TCP_RX_BUFFER_LEN)	{
		esp8266ClearTcpBuffer();
		esp8266ClearBuffer();
		if(receiveState != ESP_WAIT_IPD){
			receiveState = ESP_WAIT_IPD;
		}
		return ESP_BUFFER_OVERFLOW;
	}
	return ESP_OK;
}

void esp8266RxCallBack(char c)
{
	ESP8266_CheckOverflow();
	*bufferHead = c;

	PROCESS_TCP:
	switch(receiveState)
	{
	case ESP_WAIT_IPD:
	{
		if(strstr((const char *)esp8266RxBuffer, RESPONSE_TCP_RECEIVED) != NULL)	// wait for "+IPD," reception
		{
			receiveState = ESP_WAIT_HEADER;
		}
		break;
	}
	case ESP_WAIT_HEADER:
	{
		if(*bufferHead != ':'){  	// when entire header received "+IPD,<linkID>,<len>:" // *tcpDataProcPtr
			break;
		}else{
			esp8266_result result = ESP_ERROR;
			char *ptr = NULL;
			ptr = strstr((const char *)esp8266RxBuffer, RESPONSE_TCP_RECEIVED);	// get pointer to '+'
			ptr += 5;	// shift pointer to the first byte after "+IPD,"
			uint8_t connection_number = 0;
			connection_number = getDecNum(ptr, &ptr, 5, &result);
			if((result == ESP_OK) && (connection_number < 5)){
				current_connection = connection_number;
			}
			int32_t len = 0; // paload length
			len = getDecNum(ptr, &ptr, 5, &result);
			if((result == ESP_OK) && (len >= 0) && (len < TCP_RX_BUFFER_LEN)){
				payload_len = len;
				esp8266ClearTcpBuffer();
				receiveState = ESP_RECEIVE_TCP;
			}else{
				receiveState = ESP_WAIT_IPD;
				payload_len = 0;
				esp8266ClearBuffer();
			}
		}
		break;
	}
	case ESP_RECEIVE_TCP:
	{
		if(ESP8266_CheckTCPOverflow() == ESP_OK){
			*tcpBufferHead = *bufferHead;  	// *bufferHead
			tcpBufferHead++;

			if((tcpBufferHead - tcpRxBuffer) >= payload_len)
			{
				receiveState = ESP_TCP_RECEPTION_DONE;
				tcpDataProcPtr = bufferHead;
				unprocessed_bytes = 0;
				esp8266ClearBuffer();
			}
		}
		break;
	}
	case ESP_TCP_RECEPTION_DONE:
		unprocessed_bytes++;
		break;
	default:
		break;
	}

	if(receiveState != ESP_TCP_RECEPTION_DONE){
		tcpDataProcPtr = (bufferHead - unprocessed_bytes);
		if(tcpDataProcPtr != bufferHead){
			unprocessed_bytes--;
			goto PROCESS_TCP;
		}
	}

	bufferHead++;
}

/**
 * USART2_IRQHandler
 */
void USART2_IRQHandler(void)
{
 	portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
    	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    	char c = (char)USART_ReceiveData(USART2) & 0xff;
    	esp8266RxCallBack(c);
    }

    xSemaphoreGiveFromISR(xUsart2RxInterruptSemaphore, &xHigherPriorityTaskWoken);
   	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

#endif /* FUCKING_ESP_H_ */
