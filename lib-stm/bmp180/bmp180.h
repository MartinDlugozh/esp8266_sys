#ifndef BMP180_H
#define BMP180_H 100

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "i2c/i2c.h"
#include "math.h"

/**
 * @defgroup BMP180_Macros
 * @brief    Library defines
 * @{
 */

/* Default I2C pin */
#ifndef BMP180_I2C
#define BMP180_I2C					I2C1
#endif

/* Default I2C speed */
#ifndef BMP180_I2C_SPEED
#define BMP180_I2C_SPEED			100000
#endif

/* BMP180 I2C address */
#ifndef BMP180_I2C_ADDRESS
#define BMP180_I2C_ADDRESS			0xEE
#endif

/* Registers */
#define	BMP180_REGISTER_CONTROL 	0xF4
#define	BMP180_REGISTER_RESULT 		0xF6
#define BMP180_REGISTER_EEPROM		0xAA

/* Commands */
#define	BMP180_COMMAND_TEMPERATURE 	0x2E
#define	BMP180_COMMAND_PRESSURE_0 	0x34
#define	BMP180_COMMAND_PRESSURE_1 	0x74
#define	BMP180_COMMAND_PRESSURE_2 	0xB4
#define	BMP180_COMMAND_PRESSURE_3 	0xF4

/* Minimum waiting delay, in ms */
#define BMP180_TEMPERATURE_DELAY	8
#define BMP180_PRESSURE_0_DELAY		8
#define BMP180_PRESSURE_1_DELAY		8
#define BMP180_PRESSURE_2_DELAY		13
#define BMP180_PRESSURE_3_DELAY		25

/**
 * @}
 */
 
/**
 * @defgroup BMP180_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  BMP180 result enumerations	
 */
typedef enum {
	BMP180_Result_Ok = 0x00,            /*!< Everything OK */
	BMP180_Result_DeviceNotConnected,   /*!< Device is not connected to I2C */
	BMP180_Result_LibraryNotInitialized /*!< Library is not initialized */
} BMP180_Result_t;

/**
 * @brief  Options for oversampling settings
 * @note   This settings differs in samples for one result 
 *         and sample time for one result
 */
typedef enum {
	BMP180_Oversampling_UltraLowPower = 0x00,      /*!< 1 sample for result */
	BMP180_Oversampling_Standard = 0x01,           /*!< 2 samples for result */
	BMP180_Oversampling_HighResolution = 0x02,     /*!< 3 samples for result */
	BMP180_Oversampling_UltraHighResolution = 0x03 /*!< 4 samples for result */
} BMP180_Oversampling_t;

/**
 * @brief  BMP180 main structure		
 */
typedef struct {
	float Altitude;                        /*!< Calculated altitude at given read pressure */
	uint32_t Pressure;                     /*!< Pressure in pascals */
	float Temperature;                     /*!< Temperature in degrees */
	uint16_t Delay;                        /*!< Number of microseconds, that sensor needs to calculate data that you request to */
	BMP180_Oversampling_t Oversampling; /*!< Oversampling for pressure calculation */
} BMP180_t;

/**
 * @}
 */

/**
 * @defgroup BMP180_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes BMP180 pressure sensor
 * @param  *BMP180_Data: Pointer to @ref BMP180_t structure
 * @retval Member of @ref BMP180_Result_t
 */
BMP180_Result_t BMP180_Init();

/**
 * @brief  Starts temperature sensor on BMP180
 * @param  *BMP180_Data: Pointer to @ref BMP180_t structure
 * @retval Member of @ref BMP180_Result_t
 */
BMP180_Result_t BMP180_StartTemperature(BMP180_t* BMP180_Data);

/**
 * @brief  Reads temperature from BMP180 sensor
 * @note   Temperature has 0.1 degrees Celcius resolution
 * @param  *BMP180_Data: Pointer to @ref BMP180_t structure
 * @retval Member of @ref BMP180_Result_t
 */
BMP180_Result_t BMP180_ReadTemperature(BMP180_t* BMP180_Data);

/**
 * @brief  Starts pressure measurement on BMP180 sensor
 * @param  *BMP180_Data: Pointer to @ref BMP180_t structure
 * @param  Oversampling: Oversampling option for pressure calculation.
 *            This parameter can be a value of @ref BMP180_Oversampling_t enumeration
 * @note   Calculation time depends on selected oversampling
 * @retval Member of @ref BMP180_Result_t
 */
BMP180_Result_t BMP180_StartPressure(BMP180_t* BMP180_Data, BMP180_Oversampling_t Oversampling);

/**
 * @brief  Reads pressure from BMP180 sensor and calculate it
 * @param  *BMP180_Data: Pointer to @ref BMP180_t structure
 * @retval Member of @ref BMP180_Result_t
 */
BMP180_Result_t BMP180_ReadPressure(BMP180_t* BMP180_Data);

/**
 * @brief  Calculates pressure above sea level in pascals
 * 
 * This is good, if you read pressure from sensor at known altitude, not altitude provided from sensor.
 * Altitude from sensor is calculated in fact, that pressure above the sea is 101325 Pascals.
 * So, if you know your pressure, and you use calculated altitude, you will not get real pressure above the sea.
 * 
 * @warning You need calculated pressure from sensor, and known altitude (from other sensor or GPS data, or whatever)
 *          and then you are able to calculate pressure above the sea level.
 * @param  pressure: Pressure at known altitude in units of pascals
 * @param  altitude: Known altitude in units of meters
 * @retval Pressure above the sea in units of pascals
 */
uint32_t BMP180_GetPressureAtSeaLevel(uint32_t pressure, float altitude);

/**
 * @}
 */
 
/**
 * @}
 */
 
/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
