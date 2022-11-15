/*!
 * \file      hw-i2c.h
 *
 * \brief     Target board I2C driver definition
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Semtech
 */

#ifndef __HW_I2C_H__
#define __HW_I2C_H__

#include <stdint.h>
#include <stdbool.h>
#include "hw-gpio.h"

/****************************************************************************\
 *  Type definition
\****************************************************************************/

/*!
 * I2C peripheral ID
 */
typedef enum
{
    I2C_1,
    I2C_2,
}I2cId_t;

/*!
 * I2C object type definition
 */
typedef struct
{
	I2C_HandleTypeDef hi2c;
    I2cId_t I2cId;
    Gpio_t Scl;
    Gpio_t Sda;
}I2c_t;

/*!
 * Operation Mode for the I2C
 */
typedef enum
{
    MODE_I2C = 0,
    MODE_SMBUS_DEVICE,
    MODE_SMBUS_HOST
}I2cMode;

/*!
 * I2C signal duty cycle
 */
typedef enum
{
    I2C_DUTY_CYCLE_2 = 0,
    I2C_DUTY_CYCLE_16_9
}I2cDutyCycle;

/*!
 * I2C select if the acknowledge in after the 7th or 10th bit
 */
typedef enum
{
    I2C_ACK_ADD_7_BIT = 0,
    I2C_ACK_ADD_10_BIT
}I2cAckAddrMode;

/*!
 * Internal device address size
 */
typedef enum
{
    I2C_ADDR_SIZE_8 = 0,
    I2C_ADDR_SIZE_16,
}I2cAddrSize;

/***************************************************************************\
 * External Functions
\***************************************************************************/

/*!
 * \brief Initializes the I2C object and MCU peripheral
 *
 * \param [IN] i2cId  I2C ID
 */
void I2cInit( I2cId_t i2cId );

/*!
 * \brief DeInitializes the I2C object and MCU peripheral
 *
 * \param [IN] obj  I2C object
 */
void I2cDeInit( I2c_t *obj );

/*!
 * \brief Reset the I2C object and MCU peripheral
 *
 * \param [IN] obj  I2C object
 */
void I2cResetBus( I2c_t *obj );

/*!
 * \brief Write data to the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] addr             data address
 * \param [IN] data             data to write
 */
uint8_t I2cWrite( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t data );

/*!
 * \brief Write data buffer to the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] addr             data address
 * \param [IN] buffer           data buffer to write
 * \param [IN] size             number of bytes to write
 */
uint8_t I2cWriteBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size );

/*!
 * \brief Read data from the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] addr             data address
 * \param [OUT] data            data to read
 */
uint8_t I2cRead( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *data );

/*!
 * \brief Read data buffer from the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] addr             data address
 * \param [OUT] buffer          data buffer to read
 * \param [IN] size             number of data bytes to read
 */
uint8_t I2cReadBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size );

/*!
 * \brief Sets the internal device address size
 *
 * \param [IN] obj              I2C object
 * \param [IN] addrSize         Internal address size
 */
void I2cSetAddrSize( I2c_t *obj, I2cAddrSize addrSize );

#endif // __HW_I2C_H__
