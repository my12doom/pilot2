#ifndef __HW_USART_H
#define __HW_USART_H
/*!
 * \file      hw-usart.h
 *
 * \brief     UART driver implementation
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

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "hw.h"

/****************************************************************************\
 *  Type definition
\****************************************************************************/

// UART Events.
/**
 * Defines the event received when :
 *  - the data has been received on UART (status is SUCCESS, and param
 *      contains the data received within the UartData structure).
 *  - the receiving process has been aborted or has failed because of the
 *      buffer oversized (status is FAIL, and param is null).
 */
#define UARTEVENT_RXCOMPLETE                0
 /**
 * Defines the event received when :
 *  - the data has been transmitted on UART (status is SUCCESS, and param is
 *      null).
 *  - the transmitted process has been aborted or has failed because of the
 *      buffer oversized (status is FAIL, and param is null).
 */
#define UARTEVENT_TXCOMPLETE                1

/**
 * UartCallback
 *  This callback receives the UART events.
 *  Each events may be associated with specified status and parameters.
 */
typedef void (*UartCallback)( uint8_t uartEvent, uint8_t status, void *param );

/**
 * UartData type
 *  Define what is received by the UartCallback during the
 *  UARTEVENT_RXCOMPLETE event.
 *  It indicates:
 *  - ptrData : The data received
 *  - dataLen : The data length received
 */
typedef struct{

    uint8_t     *ptrData;

    uint16_t     dataLen;

}UartData;

/***************************************************************************\
 * External Functions
\***************************************************************************/

/*!
 * \brief Initializes the UART variables and peripheral
 *
 * \param [in] uartCallback  The UART Callback.
 *
 * \retval None
 */
void UartInit( UartCallback uartCallback );

/*!
 * \brief De-initializes the UART peripheral.
 *
 * \retval None
 */
void UartDeInit( void );

/*!
 * \brief Enable or Disable UART flow control
 *
 * \param [in] uartCallback  The UART Callback.
 * \param [in] enable  Enable or Disable Flow Control.
 *
 * \retval Status        Success(1) or Fail(0)
 */
uint8_t UartEnableFlowControl( UartCallback uartCallback, bool enable );

/*!
 * \brief Sends the amount of data given in parameter in the data buffer.
 *          Once the data transmitted the UARTEVENT_TXCOMPLETE event is sent
 *           on the callback.
 *
 * \param [in] uartTxBuffer     A pointer on the buffer to transmit.
 * \param [in] uartTxBufferLen  The length of the buffer.
 *
 * \retval Status        Success(1) or Fail(0)
 */
uint8_t UartSend( uint8_t *uartTxBuffer, uint16_t uartTxBufferLen );

/*!
 * \brief Receives data on UART and call the callback function to
 *          handle the data received along the UARTEVENT_RXCOMPLETE event.
 *
 * \retval Status        Success(1) or Fail(0)
 */
uint8_t UartReceive( void );

/*!
 * \brief Stop the sending operation. The operation is done when the
 *          UARTEVENT_TXCOMPLETE event is received on the callback.
 *
 * \retval Status        Success(1) or Fail(0)
 */
uint8_t UartStopSending( void );

/*!
 * \brief Stop the receiving operation. The operation is done when the
 *          UARTEVENT_RXCOMPLETE event is received on the callback.
 *
 * \retval Status        Success(1) or Fail(0)
 */
uint8_t UartStopReceiving( void );

#ifdef __cplusplus
}
#endif
#endif /*__HW-USART_H */
