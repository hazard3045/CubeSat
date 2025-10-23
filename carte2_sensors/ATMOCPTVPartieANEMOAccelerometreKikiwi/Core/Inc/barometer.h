/*
 * barometer.h
 *
 *  Created on: 5 oct. 2018
 *      Author: alex
 */

#ifndef BAROMETER_H_
#define BAROMETER_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief The oversampling rate
 * @warn an higher value means a longer conversion
 */
typedef enum OSR {
    OSR_256,
    OSR_512,
    OSR_1024,
    OSR_2048,
    OSR_4096
} OSR;

/**
 * @brief Init the Barometer with default parameters for SOUTH
 */
extern void Barometer_init_S();

/**
 * @brief Init the Barometer with default parameters for EAST
 */
extern void Barometer_init_E();

/**
 * @brief Init the Barometer with default parameters for WEST
 */
extern void Barometer_init_W();

/**
 * @brief Set the OSR (Oversampling rate)
 * Setting another value from the enumeration will put the min OSR
 * @warn setting a higher value means taking more time to read the data
 * @param osr the oversampling rate (refers to OSR enumeration from barometer.h)
 */
extern void Barometer_setOSR(OSR osr);

/**
 * @brief Return the temperature for SOUTH with a 2-digit precision in Celsius
 * Example: 2000 -> 20.00°C
 *
 * @param calculate true if you want to update the value
 * @pre call Barometer_init_S
 *
 * @return the temperature
 */
extern int32_t Barometer_getTemp_S(bool calculate);

/**
 * @brief Return the pressure for SOUTH in mbar with a 2-digit precision
 * Example: 100000 -> 1000.00 mbar
 *
 * @param calculate true if you want to update the value
 * @pre call Barometer_init_S
 *
 * @return the pressure
 */
extern int32_t Barometer_getPressure_S(bool calculate);

/**
 * @brief Return the altitude for SOUTH in meters
 *
 * @param calculate true if you want to update the value
 * @pre call Barometer_init_S
 *
 * @return the altitude
 */
extern float Barometer_getAltitude_S(bool calculate);

/**
 * @brief Return the temperature for EAST with a 2-digit precision in Celsius
 */
extern int32_t Barometer_getTemp_E(bool calculate);

/**
 * @brief Return the pressure for EAST in mbar with a 2-digit precision
 */
extern int32_t Barometer_getPressure_E(bool calculate);

/**
 * @brief Return the altitude for EAST in meters
 */
extern float Barometer_getAltitude_E(bool calculate);

/**
 * @brief Return the temperature for WEST with a 2-digit precision in Celsius
 */
extern int32_t Barometer_getTemp_W(bool calculate);

/**
 * @brief Return the pressure for WEST in mbar with a 2-digit precision
 */
extern int32_t Barometer_getPressure_W(bool calculate);

/**
 * @brief Return the altitude for WEST in meters
 */
extern float Barometer_getAltitude_W(bool calculate);

/**
 * @brief Calculate/update the altitude/pressure/temperature for SOUTH
 */
extern void Barometer_calculate_S();

/**
 * @brief Calculate/update the altitude/pressure/temperature for EAST
 */
extern void Barometer_calculate_E();

/**
 * @brief Calculate/update the altitude/pressure/temperature for WEST
 */
extern void Barometer_calculate_W();

extern uint16_t prom[6];

#endif /* BAROMETER_H_ */
