#ifndef FUCKING_ESP_H_
#define FUCKING_ESP_H_

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
#include "inc.h"

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define ESP8266_USART USART2

#define SOCKET_ERROR 1
#define INVALID_SOCKET 1

#define COMMAND_RESPONSE_TIMEOUT 500
#define COMMAND_RESPONSE_DELAY 50
#define COMMAND_PING_TIMEOUT 3000
#define WIFI_CONNECT_TIMEOUT 30000
#define COMMAND_RESET_TIMEOUT 5000
#define CLIENT_CONNECT_TIMEOUT 5000

#define ESP8266_MAX_SOCK_NUM 5
#define ESP8266_SOCK_NOT_AVAIL 255

#define TRUE 1
#define FALSE 0

#define ESP8266_RX_BUFFER_LEN 128	// Number of bytes in the serial receive buffer
#define TCP_RX_BUFFER_LEN 128

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
static const char RESPONSE_OK[] = "OK\r\n";
static const char RESPONSE_ERROR[] = "ERROR\r\n";
static const char RESPONSE_FAIL[] = "FAIL";
static const char RESPONSE_READY[] = "READY!";
static const char RESPONSE_RECEIVED[] = "+IPD,";
static const char ESP8266_TEST[] = "";	// Test AT startup
static const char ESP8266_RESET[] = "+RST"; // Restart module
static const char ESP8266_VERSION[] = "+GMR"; // View version info
static const char ESP8266_SLEEP[] = "+GSLP"; // Enter deep-sleep mode
static const char ESP8266_ECHO_ENABLE[] = "E1"; // AT commands echo
static const char ESP8266_ECHO_DISABLE[] = "E0"; // AT commands echo
static const char ESP8266_RESTORE[] = "+RESTORE"; // Factory reset
static const char ESP8266_UART[] = "+UART"; // UART configuration
static const char ESP8266_WIFI_MODE[] = "+CWMODE"; // WiFi mode (sta/AP/sta+AP)
static const char ESP8266_CONNECT_AP[] = "+CWJAP"; // Connect to AP
static const char ESP8266_LIST_AP[] = "+CWLAP"; // List available AP's
static const char ESP8266_DISCONNECT[] = "+CWQAP"; // Disconnect from AP
static const char ESP8266_AP_CONFIG[] = "+CWSAP"; // Set softAP configuration
static const char ESP8266_STATION_IP[] = "+CWLIF"; // List station IP's connected to softAP
static const char ESP8266_DHCP_EN[] = "+CWDHCP"; // Enable/disable DHCP
static const char ESP8266_AUTO_CONNECT[] = "+CWAUTOCONN"; // Connect to AP automatically
static const char ESP8266_SET_STA_MAC[] = "+CIPSTAMAC"; // Set MAC address of station
static const char ESP8266_GET_STA_MAC[] = "+CIPSTAMAC"; // Get MAC address of station
static const char ESP8266_SET_AP_MAC[] = "+CIPAPMAC"; // Set MAC address of softAP
static const char ESP8266_SET_STA_IP[] = "+CIPSTA"; // Set IP address of ESP8266 station
static const char ESP8266_SET_AP_IP[] = "+CIPAP"; // Set IP address of ESP8266 softAP
static const char ESP8266_TCP_STATUS[] = "+CIPSTATUS"; // Get connection status
static const char ESP8266_TCP_CONNECT[] = "+CIPSTART"; // Establish TCP connection or register UDP port
static const char ESP8266_TCP_SEND[] = "+CIPSEND"; // Send Data
static const char ESP8266_TCP_CLOSE[] = "+CIPCLOSE"; // Close TCP/UDP connection
static const char ESP8266_GET_LOCAL_IP[] = "+CIFSR"; // Get local IP address
static const char ESP8266_TCP_MULTIPLE[] = "+CIPMUX"; // Set multiple connections mode
static const char ESP8266_SERVER_CONFIG[] = "+CIPSERVER"; // Configure as server
static const char ESP8266_TRANSMISSION_MODE[] = "+CIPMODE"; // Set transmission mode
static const char ESP8266_SET_SERVER_TIMEOUT[] = "+CIPSTO"; // Set timeout when ESP8266 runs as TCP server
static const char ESP8266_PING[] = "+PING"; // Function PING
static const char ESP8266_PINMODE[] = "+PINMODE"; // Set GPIO mode (input/output)
static const char ESP8266_PINWRITE[] = "+PINWRITE"; // Write GPIO (high/low)
static const char ESP8266_PINREAD[] = "+PINREAD"; // Read GPIO digital value

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
volatile unsigned int bufferHead; // Holds position of latest byte placed in buffer.
volatile unsigned int tcpBufferHead = 0;
volatile uint8_t buffering = FALSE;

char receiveState = 0x00;
SemaphoreHandle_t xUsartRxInterruptSemaphore_esp;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void esp8266ClearBuffer(void);
uint8_t esp8266RxBufferAvailable(void);
void esp8266SendCommand(const char * cmd, esp8266_command_type type, char * params);
uint8_t esp8266SearchBuffer(const char * test);
uint8_t esp8266ReadForResponses(const char * pass, const char * fail, unsigned int timeout);
uint8_t esp8266SetMux(uint8_t mux);
uint8_t esp8266Test(void);
uint8_t esp8266Begin(void);
int tcp_getdata(unsigned char* buf, int count);
uint8_t esp8266SetMode(esp8266_wifi_mode mode);
uint8_t esp8266Connect(const char * ssid, const char * pwd);
uint8_t esp8266TcpConnect(const char* destination, const char* port, const char* link);
uint8_t esp8266TcpSend(uint8_t *buf, uint16_t size, uint8_t link);
uint8_t esp8266TcpClose(uint8_t link);
uint8_t esp8266ReadTcpData(void);
uint8_t esp8266ListConectedStaions(void);
uint8_t esp8266ServerCreate(uint16_t port);
uint8_t esp8266GetLinkID(void);

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/
void esp8266ClearBuffer(void)
{
	memset(esp8266RxBuffer, '\0', ESP8266_RX_BUFFER_LEN);
	bufferHead = 0;
}

uint8_t esp8266RxBufferAvailable(void)
{
	return (bufferHead > 0) ? TRUE:FALSE;
}

void esp8266SendCommand(const char * cmd, esp8266_command_type type, char * params)
{
	esp8266ClearBuffer();	// Clear the class receive buffer (esp8266RxBuffer)
	uart_send_str(ESP8266_USART, "AT");
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
		if(strstr((const char *)esp8266RxBuffer, test))
		{
			return TRUE;
		}
		else
		{
			return FALSE;
		}
	}

	return FALSE;
}

uint8_t esp8266ReadForResponse(const char * rsp, unsigned int timeout)
{
	uint32_t timeIn = millis();	// Timestamp coming into function
	while (timeIn + timeout > millis()) // While we haven't timed out
	{
		if (esp8266RxBufferAvailable()) // If data is available on ESP8266_USART RX
		{
			if (esp8266SearchBuffer(rsp))	// Search the buffer for goodRsp
				return TRUE;
		}
		vTaskDelay(COMMAND_RESPONSE_DELAY);
	}
	return FALSE; // Return the timeout error code
}

uint8_t esp8266ReadForResponses(const char * pass, const char * fail, unsigned int timeout)
{
	uint8_t result = FALSE;
	unsigned long timeIn = millis();	// Timestamp coming into function
	while (timeIn + timeout > millis()) // While we haven't timed out
	{
		if (esp8266RxBufferAvailable()) // If data is available on UART RX
		{
			result = esp8266SearchBuffer(pass);
			if (result)	// Search the buffer for goodRsp
				return TRUE;	// Return how number of chars read
			result = esp8266SearchBuffer(fail);
			if (result)
				return FALSE;
		}
		vTaskDelay(COMMAND_RESPONSE_DELAY);
	}
	return FALSE;
}

uint8_t esp8266SetMux(uint8_t mux)
{
	uint8_t result = FALSE;
	char params[2] = {0, 0};
	params[0] = (mux > 0) ? '1' : '0';
	esp8266SendCommand(ESP8266_TCP_MULTIPLE, ESP8266_CMD_SETUP, params);
	result = esp8266ReadForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
	return result;
//	return TRUE;
}

uint8_t esp8266Test(void)
{
	esp8266SendCommand(ESP8266_TEST, ESP8266_CMD_EXECUTE, 0);
	esp8266ClearBuffer();
	if(esp8266ReadForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT))
		return TRUE;
	return FALSE;
}

uint8_t esp8266Begin(void)
{
	uint8_t test = FALSE;
	test = esp8266Test();
	if(test)
	{
		if (esp8266SetMux(1))
			return TRUE;
		return FALSE;
	}
	return FALSE;
}

int tcp_getdata(unsigned char* buf, int count)
{
	int i;
	if(count <= TCP_RX_BUFFER_LEN)
	{
		for(i = 0; i < count; i++)
		{
			*(buf + i) = tcpRxBuffer[i];
		}
		for(i = 0; i < TCP_RX_BUFFER_LEN - count; i++)
		{
			tcpRxBuffer[i] = tcpRxBuffer[i+count];
		}
		return count;
	}
	else
	{
		return -1;
	}
}

uint8_t esp8266SetMode(esp8266_wifi_mode mode)
{
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
	uint8_t result = FALSE;
	if(esp8266SetMode(ESP8266_MODE_STA))
	{
		esp8266ClearBuffer();
		uart_send_str(ESP8266_USART, "AT");
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
		result = esp8266ReadForResponses(RESPONSE_OK, RESPONSE_FAIL, WIFI_CONNECT_TIMEOUT);
		return result;
	}
	return FALSE;
}

uint8_t esp8266TcpConnect(const char* destination, const char* port, const char* link)
{
	uint8_t rsp = FALSE;
	esp8266ClearBuffer();
	uart_send_str(ESP8266_USART, "AT");
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
	rsp = esp8266ReadForResponses(RESPONSE_OK, RESPONSE_ERROR, CLIENT_CONNECT_TIMEOUT);

	if(rsp == FALSE)
	{
		// We may see "ERROR", but be "ALREADY CONNECTED".
		// Search for "ALREADY", and return success if we see it.
		rsp = esp8266SearchBuffer("ALREADY");
		if (rsp)
			return TRUE;
		// Otherwise the connection failed. Return the error code:
		return FALSE;
	}
	// Return 1 on successful (new) connection
	return TRUE;
}

uint8_t esp8266TcpSend(uint8_t *buf, uint16_t size, uint8_t link)
{
	uint8_t rsp = FALSE;
	uint8_t *p = buf;
	char params[8];
	if (size > 2048)
		return FALSE; //ESP8266_CMD_BAD
	sprintf(params, "%d,%d", link, size);
	esp8266SendCommand(ESP8266_TCP_SEND, ESP8266_CMD_SETUP, params);

	rsp = esp8266ReadForResponses(RESPONSE_OK, RESPONSE_ERROR, COMMAND_RESPONSE_TIMEOUT);
	if(rsp)
	{
		esp8266ClearBuffer();

		uart_send_str(ESP8266_USART, (char *)p);
		rsp = esp8266ReadForResponse("SEND OK", COMMAND_RESPONSE_TIMEOUT);
		if (rsp)
			return TRUE;
	}
	return FALSE;
}

uint8_t esp8266TcpClose(uint8_t link)
{
	uint8_t rc = FALSE;
	char params[8];
	sprintf(params, "%d", link);
	esp8266SendCommand(ESP8266_SERVER_CONFIG, ESP8266_CMD_SETUP, params);
	esp8266ClearBuffer();
	rc = esp8266ReadForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT);
	return rc;
}

uint8_t esp8266ReadTcpData(void)
{
	uint8_t i = 0;
	char *p;

	if (esp8266RxBufferAvailable()) // If data is available on ESP8266_USART RX
	{
		if(esp8266SearchBuffer(RESPONSE_RECEIVED))	// Search the buffer for goodRsp
		{
			p = strstr((const char *)esp8266RxBuffer, "+IPD,");
			if(p)
			{
				do{
					p++;
				}while(*p !=':');
				p++;
				i=0;
				while(*p)
				{
					tcpRxBuffer[i] = *p;
					i++;
					p++;
				}
				esp8266ClearBuffer();
				return TRUE;
			}else{
				return FALSE;
			}
		}else{
			return FALSE;
		}
	}
	return FALSE;
}

// TODO: finish implementation
uint8_t esp8266ListConectedStaions(void)
{
	esp8266SendCommand(ESP8266_STATION_IP, ESP8266_CMD_EXECUTE, 0);
	esp8266ClearBuffer();
	if(esp8266ReadForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT))
		return TRUE;
	return FALSE;
}

uint8_t esp8266ServerCreate(uint16_t port)
{
	char params[8];
	sprintf(params, "%d,%d", 1, port);
	esp8266SendCommand(ESP8266_SERVER_CONFIG, ESP8266_CMD_SETUP, params);
	esp8266ClearBuffer();
	if(esp8266ReadForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT))
		return TRUE;
	return FALSE;
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
		if (esp8266RxBufferAvailable())
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

void USART2_IRQHandler(void)
{
 	portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	if(USART_GetITStatus(ESP8266_USART, USART_IT_RXNE) != RESET)
	{
		esp8266RxBuffer[bufferHead++] = USART_ReceiveData(ESP8266_USART);
	}
	if(bufferHead >= ESP8266_RX_BUFFER_LEN)
	{
		bufferHead = 0;
	}

	xSemaphoreGiveFromISR(xUsartRxInterruptSemaphore_esp, &xHigherPriorityTaskWoken);
   	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

#endif /* FUCKING_ESP_H_ */
