/*
 * stm32_rtc_helper.h
 *
 *  Created on: Feb 10, 2018
 *      Author: Dr. Saldon
 */

#ifndef STM32_RTC_HELPER_H_
#define STM32_RTC_HELPER_H_

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
// (UnixTime = 00:00:00 01.01.1970 = JD0 = 2440588)
#define JULIAN_DATE_BASE    2440588

// Example of __DATE__ string: "Jul 27 2012"
//                              01234567890

#define BUILD_YEAR_CH0 (__DATE__[ 7])
#define BUILD_YEAR_CH1 (__DATE__[ 8])
#define BUILD_YEAR_CH2 (__DATE__[ 9])
#define BUILD_YEAR_CH3 (__DATE__[10])

#define BUILD_MONTH_IS_JAN (__DATE__[0] == 'J' && __DATE__[1] == 'a' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_FEB (__DATE__[0] == 'F')
#define BUILD_MONTH_IS_MAR (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'r')
#define BUILD_MONTH_IS_APR (__DATE__[0] == 'A' && __DATE__[1] == 'p')
#define BUILD_MONTH_IS_MAY (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'y')
#define BUILD_MONTH_IS_JUN (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_JUL (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'l')
#define BUILD_MONTH_IS_AUG (__DATE__[0] == 'A' && __DATE__[1] == 'u')
#define BUILD_MONTH_IS_SEP (__DATE__[0] == 'S')
#define BUILD_MONTH_IS_OCT (__DATE__[0] == 'O')
#define BUILD_MONTH_IS_NOV (__DATE__[0] == 'N')
#define BUILD_MONTH_IS_DEC (__DATE__[0] == 'D')

#define BUILD_MONTH_CH0 \
    ((BUILD_MONTH_IS_OCT || BUILD_MONTH_IS_NOV || BUILD_MONTH_IS_DEC) ? '1' : '0')

#define BUILD_MONTH_CH1 \
    ( \
        (BUILD_MONTH_IS_JAN) ? '1' : \
        (BUILD_MONTH_IS_FEB) ? '2' : \
        (BUILD_MONTH_IS_MAR) ? '3' : \
        (BUILD_MONTH_IS_APR) ? '4' : \
        (BUILD_MONTH_IS_MAY) ? '5' : \
        (BUILD_MONTH_IS_JUN) ? '6' : \
        (BUILD_MONTH_IS_JUL) ? '7' : \
        (BUILD_MONTH_IS_AUG) ? '8' : \
        (BUILD_MONTH_IS_SEP) ? '9' : \
        (BUILD_MONTH_IS_OCT) ? '0' : \
        (BUILD_MONTH_IS_NOV) ? '1' : \
        (BUILD_MONTH_IS_DEC) ? '2' : \
        /* error default */    '?' \
    )

#define BUILD_DAY_CH0 ((__DATE__[4] >= '0') ? (__DATE__[4]) : '0')
#define BUILD_DAY_CH1 (__DATE__[ 5])

// Example of __TIME__ string: "21:06:19"
//                              01234567

#define BUILD_HOUR_CH0 (__TIME__[0])
#define BUILD_HOUR_CH1 (__TIME__[1])

#define BUILD_MIN_CH0 (__TIME__[3])
#define BUILD_MIN_CH1 (__TIME__[4])

#define BUILD_SEC_CH0 (__TIME__[6])
#define BUILD_SEC_CH1 (__TIME__[7])

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
#include "stm32f10x_rtc.h"

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/


typedef struct {
    uint8_t RTC_Hour;
    uint8_t RTC_Minute;
    uint8_t RTC_Second;
    uint8_t RTC_Date;
    uint8_t RTC_Wday;
    uint8_t RTC_Month;
    uint16_t RTC_Year;
} STM32_RTC_DateTimeTypeDef;

uint32_t STM32_RTC_Counter = 0;
STM32_RTC_DateTimeTypeDef STM32_RTC_DateTime;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void 		STM32_RTC_CounterToDateTime	(uint32_t STM32_RTC_Counter, STM32_RTC_DateTimeTypeDef* STM32_RTC_DateTimeStruct);
uint32_t 	STM32_RTC_DateTimeToCounter	(STM32_RTC_DateTimeTypeDef* STM32_RTC_DateTimeStruct);
uint8_t 	STM32_RTC_Init				(void);

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

// Get current date
void STM32_RTC_CounterToDateTime(uint32_t STM32_RTC_Counter, STM32_RTC_DateTimeTypeDef* STM32_RTC_DateTimeStruct)
{
	unsigned long time;
	unsigned long t1, a, b, c, d, e, m;
	int year = 0;
    int mon = 0;
    int wday = 0;
    int mday = 0;
    int hour = 0;
    int min = 0;
    int sec = 0;
    uint64_t jd = 0;;
    uint64_t jdn = 0;

    jd = ((STM32_RTC_Counter+43200)/(86400>>1)) + (2440587<<1) + 1;
    jdn = jd>>1;

    time = STM32_RTC_Counter;
    t1 = time/60;
    sec = time - t1*60;

    time = t1;
    t1 = time/60;
    min = time - t1*60;

    time = t1;
    t1 = time/24;
    hour = time - t1*24;

    wday = jdn%7;

    a = jdn + 32044;
    b = (4*a+3)/146097;
    c = a - (146097*b)/4;
    d = (4*c+3)/1461;
    e = c - (1461*d)/4;
    m = (5*e+2)/153;
    mday = e - (153*m+2)/5 + 1;
    mon = m + 3 - 12*(m/10);
    year = 100*b + d - 4800 + (m/10);

    STM32_RTC_DateTimeStruct->RTC_Year = year;
    STM32_RTC_DateTimeStruct->RTC_Month = mon;
    STM32_RTC_DateTimeStruct->RTC_Date = mday;
    STM32_RTC_DateTimeStruct->RTC_Hour = hour;
    STM32_RTC_DateTimeStruct->RTC_Minute = min;
    STM32_RTC_DateTimeStruct->RTC_Second = sec;
    STM32_RTC_DateTimeStruct->RTC_Wday = wday;
}

// Convert Date to Counter
uint32_t STM32_RTC_DateTimeToCounter(STM32_RTC_DateTimeTypeDef* STM32_RTC_DateTimeStruct)
{
    uint8_t a;
    uint16_t y;
    uint8_t m;
    uint32_t JDN;

    a=(14-STM32_RTC_DateTimeStruct->RTC_Month)/12;
    y=STM32_RTC_DateTimeStruct->RTC_Year+4800-a;
    m=STM32_RTC_DateTimeStruct->RTC_Month+(12*a)-3;

    JDN=STM32_RTC_DateTimeStruct->RTC_Date;
    JDN+=(153*m+2)/5;
    JDN+=365*y;
    JDN+=y/4;
    JDN+=-y/100;
    JDN+=y/400;
    JDN = JDN -32045;
    JDN = JDN - JULIAN_DATE_BASE;
    JDN*=86400;
    JDN+=(STM32_RTC_DateTimeStruct->RTC_Hour*3600);
    JDN+=(STM32_RTC_DateTimeStruct->RTC_Minute*60);
    JDN+=(STM32_RTC_DateTimeStruct->RTC_Second);

    return JDN;
}

uint8_t STM32_RTC_Init(void)
{
	const char str_build_hour[] = { BUILD_HOUR_CH0, BUILD_HOUR_CH1 };
	const char str_build_minute[] = { BUILD_MIN_CH0, BUILD_MIN_CH1 };
	const char str_build_second[] = { BUILD_SEC_CH0, BUILD_SEC_CH1 };
	const char str_build_day[] = { BUILD_DAY_CH0, BUILD_DAY_CH1 };
	const char str_build_month[] = { BUILD_MONTH_CH0, BUILD_MONTH_CH1 };
	const char str_build_year[] = { BUILD_YEAR_CH0, BUILD_YEAR_CH1, BUILD_YEAR_CH2, BUILD_YEAR_CH3 };

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    if ((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN)
    {
        RCC_BackupResetCmd(ENABLE);
        RCC_BackupResetCmd(DISABLE);
        RCC_LSEConfig(RCC_LSE_ON);
        while ((RCC->BDCR & RCC_BDCR_LSERDY) != RCC_BDCR_LSERDY) {}
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
        RTC_SetPrescaler(0x7FFF);
        RCC_RTCCLKCmd(ENABLE);
        RTC_WaitForSynchro();

        for(uint32_t i = (24000*500); i > 0; i--){asm("nop");}

		STM32_RTC_DateTime.RTC_Date = 	(uint8_t)atoi(str_build_day);
		STM32_RTC_DateTime.RTC_Month = 	(uint8_t)atoi(str_build_month);
		STM32_RTC_DateTime.RTC_Year = 	(uint16_t)atoi(str_build_year);

		STM32_RTC_DateTime.RTC_Hour = 	(uint8_t)atoi(str_build_hour);
		STM32_RTC_DateTime.RTC_Minute = (uint8_t)atoi(str_build_minute);
		STM32_RTC_DateTime.RTC_Second = (uint8_t)atoi(str_build_second);

		STM32_RTC_Counter = RTC_GetCounter();
		RTC_SetCounter(STM32_RTC_DateTimeToCounter(&STM32_RTC_DateTime));
        return 1;
    }
    return 0;
}

#endif /* STM32_RTC_HELPER_H_ */
