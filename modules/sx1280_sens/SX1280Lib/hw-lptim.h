#ifndef __HW_LPTIM_H
#define __HW_LPTIM_H
/*!
 * \file      hw-lptim.h
 *
 * \brief     Timer driver implementation
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

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include <stdint.h>

/***************************************************************************\
 * External Function Definition
\***************************************************************************/

/*!
 * \brief Creates an autoreload timer.
 *
 * \param [in] func         A pointer to a function to call when the timer
 *                          expires.
 * \param [in] param        Parameter to pass in func when the timer expires.
 * \param [in] millisec     Time to wait before the timer expiration.
 *
 */
void TimerCreateTimer(void *func, void *param, uint32_t millisec);

/*!
 * \brief Cancel the current running timer.
 *
 */
void TimerCancelTimer(void);


#endif /*__HW_LPTIM_H */

