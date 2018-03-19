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
}ESP8266_Result_t;

typedef enum{
	ESP_WAIT_IPD = 0,
	ESP_WAIT_HEADER,
	ESP_RECEIVE_TCP,
	ESP_TCP_RECEPTION_DONE,
}ESP8266_IPD_t;

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
//char* tcpDataProcPtr = esp8266RxBuffer;
uint8_t current_connection = 0;

uint8_t receiveState = 0;
uint16_t unprocessed_bytes = 0;
uint8_t payload_len = 0;
uint8_t transmission_free = 1;

///**
// * \brief           Security settings for wifi network
// */
//typedef enum _ESP_Ecn_t {
//	ESP_Ecn_OPEN = 0x00,                                /*!< Wifi is open */
//	ESP_Ecn_WEP = 0x01,                                 /*!< Wired Equivalent Privacy option for wifi security. \note  This mode can't be used when setting up ESP8266 wifi */
//	ESP_Ecn_WPA_PSK = 0x02,                             /*!< Wi-Fi Protected Access */
//	ESP_Ecn_WPA2_PSK = 0x03,                            /*!< Wi-Fi Protected Access 2 */
//	ESP_Ecn_WPA_WPA2_PSK = 0x04,                        /*!< Wi-Fi Protected Access with both modes */
//} ESP_Ecn_t;
//
///**
// * \brief           AP station structure to use when searching for network
// */
//typedef struct _ESP_AP_t {
//	ESP_Ecn_t Ecn;                                      /*!< Security of Wi-Fi spot. This parameter has a value of \ref ESP_Ecn_t enumeration */
//	char SSID[20 + 1];                                  /*!< Service Set Identifier value. Wi-Fi spot name */
//	int16_t RSSI;                                       /*!< Signal strength of Wi-Fi spot */
//	uint8_t MAC[6];                                     /*!< MAC address of spot */
//	uint8_t Channel;                                    /*!< Wi-Fi channel */
//	int8_t Offset;                                      /*!< Frequency offset from base 2.4GHz in kHz */
//	uint8_t Calibration;                                /*!< Frequency offset calibration */
//} ESP_AP_t;
//
///**
// * \brief           Connected AP structure
// */
//typedef struct _ESP_ConnectedAP_t {
//	char SSID[20 + 1];                                  /*!< SSID network name */
//	uint8_t MAC[6];                                     /*!< MAC address of network */
//	uint8_t Channel;                                    /*!< Network channel */
//	int16_t RSSI;                                       /*!< Signal strength */
//} ESP_ConnectedAP_t;
//
///**
// * \brief           Structure for connected station to softAP to ESP module
// */
//typedef struct _ESP_ConnectedStation_t {
//	uint8_t IP[4];                                      /*!< IP address of connected station */
//	uint8_t MAC[6];                                     /*!< MAC address of connected station */
//} ESP_ConnectedStation_t;
//
///**
// * \brief           Connection type
// */
//typedef enum _ESP_CONN_Type_t {
//	ESP_CONN_Type_TCP = 0x00,                           /*!< Connection type is TCP */
//	ESP_CONN_Type_UDP = 0x01,                           /*!< Connection type is UDP */
//	ESP_CONN_Type_SSL = 0x02                            /*!< Connection type is SSL */
//} ESP_CONN_Type_t;
//
///**
// * \brief           Connection structure
// */
//typedef struct _ESP_CONN_t {
//	uint8_t Number;                                     /*!< Connection number */
//	uint16_t RemotePort;                                /*!< Remote PORT number */
//	uint8_t RemoteIP[4];                                /*!< IP address of device */
//    uint16_t LocalPort;                                 /*!< Local PORT number */
//	ESP_CONN_Type_t Type;                               /*!< Connection type. Parameter is valid only if connection is made as client */
//
//    uint16_t DataLength;                                /*!< Number of bytes received in connection packet */
//
//    uint32_t TotalBytesReceived;                        /*!< Number of total bytes so far received on connection */
//    uint32_t DataStartTime;                             /*!< Current time in units of milliseconds when first data packet was received on connection */
//	union {
//		struct {
//			int Active:1;                               /*!< Status if connection is active */
//			int Client:1;                               /*!< Set to 1 if connection was made as client */
//            int SSL:1;                                  /*!< Connection has been made as SSL */
//        } F;
//		uint8_t Value;                                  /*!< Value of entire union */
//	} Flags;                                            /*!< Connection flags management */
//    union {
//        struct {
//            int Connect:1;                              /*!< Connection was just connected, client or server */
//            int Closed:1;                               /*!< Connection was just disconnected, client or server */
//            int DataSent:1;                             /*!< Data were sent successfully */
//            int DataError:1;                            /*!< Error trying to send data */
//            int CallLastPartOfPacketReceived:1;         /*!< Data are processed synchronously. When there is last part of packet received and command is not idle, we must save notification for callback */
//        } F;
//        int Value;
//    } Callback;                                         /*!< Flags for callback management */
//
//    void* Arg;                                          /*!< Custom connection argument */
//
//    uint32_t PollTimeInterval;                          /*!< Interval for poll callback when connection is active but nothing happens to it */
//    uint32_t PollTime;                                  /*!< Internal next poll time */
//} ESP_CONN_t;
//
///**
// * \brief         IPD network data structure
// */
//typedef struct _ESP_IPD_t {
//	uint8_t InIPD;                                      /*!< Set to 1 when ESP is in IPD mode with data */
//	uint8_t Conn;                                 		/*!< Connection number where IPD is active */
//    uint16_t BytesRemaining;                            /*!< Remaining bytes to read from entire IPD statement */
//    uint16_t BytesRead;                                 /*!< Bytes read in current packet */
//} ESP_IPD_t;
//
//ESP_IPD_t IPD;
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
//uint8_t esp8266Begin(void);
//int tcp_getdata(unsigned char* buf, int count);
uint8_t esp8266SetMode(esp8266_wifi_mode mode);
uint8_t esp8266Connect(const char * ssid, const char * pwd);
uint8_t esp8266TcpConnect(const char* destination, const char* port, const char* link);
uint8_t esp8266TcpSend(uint8_t *buf, uint16_t size, uint8_t link);
uint8_t esp8266TcpClose(uint8_t link);
uint8_t esp8266ListConectedStaions(void);
uint8_t esp8266ServerCreate(uint16_t port, uint8_t enable);
uint8_t esp8266GetLinkID(void);
void ESP8266_RxCallBack(char c);

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

///* Returns number from hex value */
//uint8_t Hex2Num(char a) {
//    if (a >= '0' && a <= '9') {                             /* Char is num */
//        return a - '0';
//    } else if (a >= 'a' && a <= 'f') {                      /* Char is lowercase character A - Z (hex) */
//        return (a - 'a') + 10;
//    } else if (a >= 'A' && a <= 'F') {                      /* Char is uppercase character A - Z (hex) */
//        return (a - 'A') + 10;
//    }
//
//    return 0;
//}

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

///* Parses and returns HEX number from string */
//uint32_t ParseHexNumber(const char* ptr, uint8_t* cnt) {
//    uint32_t sum = 0;
//    uint8_t i = 0;
//
//    while (CHARISHEXNUM(*ptr)) {                    		/* Parse number */
//        sum <<= 4;
//        sum += Hex2Num(*ptr);
//        ptr++;
//        i++;
//    }
//
//    if (cnt) {                               		        /* Save number of characters used for number */
//        *cnt = i;
//    }
//    return sum;                                        		/* Return number */
//}
//
///* Parse MAC number in string format xx:xx:xx:xx:xx:xx */
//void ParseMAC(const char* str, uint8_t* mac, uint8_t* cnt) {
//    uint8_t i = 6;
//
//    while (i--) {
//        *mac++ = ParseHexNumber(str, NULL);
//        str += 3;
//    }
//    if (cnt) {
//        *cnt = 17;
//    }
//}
//
///* Parse IP number in string format 'xxx.xxx.xxx.xxx' */
//void ParseIP(const char* str, uint8_t* ip, uint8_t* cnt) {
//    uint8_t i = 4;
//    uint8_t c = 0;
//
//    if (cnt) {
//        *cnt = 0;
//    }
//    while (i--) {
//        *ip++ = ParseNumber(str, &c);
//        str += c + 1;
//        if (cnt) {
//            *cnt += c;
//            if (i) {
//                *cnt += 1;
//            }
//        }
//    }
//}
//
///* Parse +CWLAP statement */
//void ParseCWLAP(const char* str, ESP_AP_t* AP) {
//    uint8_t cnt;
//
//    if (*str == '(') {                                      /* Remove opening bracket */
//        str++;
//    }
//
//    memset((void *)AP, 0x00, sizeof(ESP_AP_t));             /* Reset structure first */
//
//    AP->Ecn = (ESP_Ecn_t)ParseNumber(str, &cnt);            /* Parse ECN value */
//    str += cnt + 1;
//
//    if (*str == '"') {                                      /* Remove opening " */
//        str++;
//    }
//
//    cnt = 0;                                                /* Parse SSID */
//    while (*str) {
//        if (*str == '"' && *(str + 1) == ',') {
//            break;
//        }
//        if (cnt < sizeof(AP->SSID) - 1) {
//            AP->SSID[cnt] = *str;
//        }
//
//        cnt++;
//        str++;
//    }
//
//    str += 2;                                               /* Parse RSSI */
//    AP->RSSI = ParseNumber(str, &cnt);
//
//    str += cnt + 1;
//    if (*str == '"') {
//        str++;
//    }
//    ParseMAC(str, AP->MAC, NULL);                      		/* Parse MAC */
//    str += 19;                                              /* Ignore mac, " and comma */
//    AP->Channel = ParseNumber(str, &cnt);                   /* Parse channel for wifi */
//    str += cnt + 1;
//    AP->Offset = ParseNumber(str, &cnt);                    /* Parse offset */
//    str += cnt + 1;
//    AP->Calibration = ParseNumber(str, &cnt);               /* Parse calibration number */
//    str += cnt + 1;
//}
//
///* Parse +CWJAP statement */
//void ParseCWJAP(const char* ptr, ESP_ConnectedAP_t* AP) {
//    uint8_t i, cnt;
//
//    while (*ptr && *ptr != '"') {                    		/* Find first " character */
//        ptr++;
//    }
//    if (!*ptr) {                                    		/* Check if zero detected */
//        return;
//    }
//    ptr++;                                            		/* Remove first " for SSID */
//    i = 0;                                            		/* Parse SSID part */
//    while (*ptr && (*ptr != '"' || *(ptr + 1) != ',' || *(ptr + 2) != '"')) {
//        AP->SSID[i++] = *ptr++;
//    }
//    AP->SSID[i++] = 0;
//    ptr += 3;                                        		/* Increase pointer by 3, ignore "," part */
//    ParseMAC(ptr, AP->MAC, NULL);    	               		 /* Get MAC */
//    ptr += 19;                                    		    /* Increase counter by elements in MAC address and ", part */
//    AP->Channel = ParseNumber(ptr, &cnt);	                /* Get channel */
//    ptr += cnt + 1;                                    		/* Increase position */
//    AP->RSSI = ParseNumber(ptr, &cnt);    					/* Get RSSI */
//}
//
///* Parse CWLIF statement with IP and MAC */
//void ParseCWLIF(const char* str, ESP_ConnectedStation_t* station) {
//    uint8_t cnt;
//
//    ParseIP(str, station->IP, &cnt);                   /* Parse IP address */
//    str += cnt + 1;
//    ParseMAC(str, station->MAC, &cnt);                 /* Parse MAC */
//}

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

//uint8_t esp8266Begin(void)
//{
//	uint8_t test = ESP_ERROR;
//	test = esp8266Test();
//	if(test)
//	{
//		if (esp8266SetMux(1))
//			return ESP_OK;
//		return ESP_ERROR;
//	}
//	return ESP_ERROR;
//}

//int tcp_getdata(unsigned char* buf, int count)
//{
//	int i;
//	if(count <= TCP_RX_BUFFER_LEN)
//	{
//		for(i = 0; i < count; i++)
//		{
//			*(buf + i) = tcpRxBuffer[i];
//		}
//		for(i = 0; i < TCP_RX_BUFFER_LEN - count; i++)
//		{
//			tcpRxBuffer[i] = tcpRxBuffer[i+count];
//		}
//		return count;
//	}
//	else
//	{
//		return -1;
//	}
//}

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
		sprintf(params, "%d,%d", link, size+1);
		esp8266SendCommand(ESP8266_TCP_SEND, ESP8266_CMD_SETUP, params); // +CIPSENDBUF

		result = esp8266ReadForResponses(RESPONSE_PROMPT, RESPONSE_ERROR, NULL, COMMAND_RESPONSE_TIMEOUT);
		if(result == ESP_OK)
		{
			esp8266ClearBuffer();
			uart_send_str_n(ESP8266_USART, (char *)p, size);
			uart_send_byte(ESP8266_USART, '\0'); // size+1
			result = esp8266ReadForResponse(RESPONSE_SEND_OK, COMMAND_RESPONSE_TIMEOUT); 	// "SEND OK"
//			result=1; // FIXME: затычка, оставшаяся из прошлой версии; возможно, придется вернуть
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
	esp8266SendCommand(ESP8266_SERVER_CONFIG, ESP8266_CMD_SETUP, params);
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
	if(esp8266ReadForResponse(RESPONSE_OK, COMMAND_RESPONSE_TIMEOUT)){
		return ESP_OK;}
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

//void ESP8266_RxCallBack(char c)
//{
//	if(bufferHead >= ESP8266_RX_BUFFER_LEN)
//	{
//		esp8266ClearBuffer();
//		esp8266ClearTcpBuffer();
//		receiveState = 0;
//	}
//
//	esp8266RxBuffer[bufferHead] = c;
//
//	switch(receiveState)
//	{
//	case 0:
//	{
//		if(strstr((const char *)esp8266RxBuffer, RESPONSE_TCP_RECEIVED) != NULL)	//"+IPD,"
//		{
//			receiveState = 1;
//		}
//		break;
//	}
//	case 1:
//	{
//		if(esp8266RxBuffer[bufferHead] != ':'){
//			break;
//		}else{
//			esp8266ClearTcpBuffer();
//
//			char *p;
//			p = strstr((const char *)esp8266RxBuffer, RESPONSE_TCP_RECEIVED);	//"+IPD,"
//			char strlen[3];
//
//			memcpy(strlen, p+7, (strstr((const char *)esp8266RxBuffer, ":") - (p+7)));
//
//			int16_t len;
//			len = (atoi(strlen));		// в первой версии len = (atoi(strlen) - 1);
//			if((len >= 0) && (len <= TCP_RX_BUFFER_LEN)){
//				payload_len = len;
//				receiveState = 2;
//			}else{
//				receiveState = 0;
//				payload_len = 0;
//
//				esp8266ClearBuffer();
//			}
//		}
//		break;
//	}
//	case 2:
//	{
//		tcpRxBuffer[tcpBufferHead++] = esp8266RxBuffer[bufferHead];
//
//		if(tcpBufferHead >= payload_len)
//		{
//			receiveState = 3;
//			esp8266ClearBuffer();
//			break;
//		}
//
//		if(tcpBufferHead >= TCP_RX_BUFFER_LEN){
//			esp8266ClearBuffer();
//			esp8266ClearTcpBuffer();
//			receiveState = 0;
//		}
//		break;
//	}
//	default:
//		break;
//	}
//	bufferHead++;
//}

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

//void ESP8266_RxCallBack(char c)
//{
//	taskENTER_CRITICAL_FROM_ISR();
//
//	ESP8266_CheckOverflow();
//	*bufferHead = c;
//
//	switch(receiveState)
//	{
//	case 0:
//	{
//		if(strstr((const char *)esp8266RxBuffer, RESPONSE_TCP_RECEIVED) != NULL)	//"+IPD,"
//		{
//			receiveState = 1;
//		}
//		break;
//	}
//	case 1:
//	{
//		if(*bufferHead != ':'){
//			break;
//		}else{
//			esp8266ClearTcpBuffer();
//
//			char *p;
//			p = strstr((const char *)esp8266RxBuffer, RESPONSE_TCP_RECEIVED);	//"+IPD,"
//			char strlen[3];
//
//			memcpy(strlen, p+7, (strstr((const char *)esp8266RxBuffer, ":") - (p+7)));
//
//			int16_t len;
//			len = (atoi(strlen));		// в первой версии len = (atoi(strlen) - 1);
//			if((len >= 0) && (len <= TCP_RX_BUFFER_LEN)){
//				payload_len = len;
//				receiveState = 2;
//			}else{
//				receiveState = 0;
//				payload_len = 0;
//
//				esp8266ClearBuffer();
//			}
//		}
//		break;
//	}
//	case 2:
//	{
//		*tcpBufferHead = *bufferHead;
//		tcpBufferHead++;
//
//		if((tcpBufferHead - tcpRxBuffer) >= payload_len)
//		{
//			receiveState = 3;
//			esp8266ClearBuffer();
//			break;
//		}
//
//		if((tcpBufferHead - tcpRxBuffer) >= TCP_RX_BUFFER_LEN){
//			esp8266ClearBuffer();
//			esp8266ClearTcpBuffer();
//			receiveState = 0;
//		}
//		break;
//	}
//	default:
//		break;
//	}
//	bufferHead++;
//
//	taskEXIT_CRITICAL_FROM_ISR();
//}

void ESP8266_RxCallBack(char c)
{
	ESP8266_CheckOverflow();
	*bufferHead = c;

//	PROCESS_TCP:
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
			char *ptr = NULL;
			ptr = strstr((const char *)esp8266RxBuffer, RESPONSE_TCP_RECEIVED);	// get pointer to '+'
			ptr += 5;	// shift pointer to the first byte after "+IPD,"
			uint8_t connection_number = 0;
			uint8_t cnt = 0; // bytes parsed as decimal number (in this case should be always 1)
			connection_number = ParseNumber(ptr, &cnt);
			if((connection_number < 5) && (cnt == 1)){
				ptr += cnt;
				cnt = 0;
				current_connection = connection_number;
			}
			int32_t len = 0;
			len = ParseNumber(ptr, &cnt);
			if((len >= 0) && (len < TCP_RX_BUFFER_LEN)){
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
//				tcpDataProcPtr = bufferHead;
//				unprocessed_bytes = 0;
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

//	if(receiveState != ESP_TCP_RECEPTION_DONE){
//		tcpDataProcPtr = (bufferHead - unprocessed_bytes);
//		if(tcpDataProcPtr != bufferHead){
//			unprocessed_bytes--;
//			goto PROCESS_TCP;
//		}
//	}

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
    	ESP8266_RxCallBack(c);
    }

    xSemaphoreGiveFromISR(xUsart2RxInterruptSemaphore, &xHigherPriorityTaskWoken);
   	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

#endif /* FUCKING_ESP_H_ */
