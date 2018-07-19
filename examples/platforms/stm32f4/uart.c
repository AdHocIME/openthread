/*
 *  Copyright (c) 2016, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements the OpenThread platform abstraction for UART communication.
 *
 */

#include <openthread-core-config.h>
#include <openthread/config.h>

#include <stddef.h>
#include <stdint.h>

#include <utils/code_utils.h>
#include <openthread/types.h>
#include <openthread/platform/toolchain.h>
#include <openthread/platform/uart.h>

#include "platform.h"
#include "platform-config.h"

#include "usbd_cdc_if.h"

#if (USB_CDC_AS_SERIAL_TRANSPORT == 0)

bool sUartEnabled = false;

/**
 *  UART TX buffer variables.
 */
static const uint8_t *sTransmitBuffer = NULL;
static uint16_t       sTransmitLength = 0;
static bool           sTransmitDone   = 0;

/**
 *  UART RX ring buffer variables.
 */
static uint8_t  sReceiveBuffer[UART_RX_BUFFER_SIZE];
static uint16_t sReceiveHead = 0;
static uint16_t sReceiveTail = 0;

/**
 * Function for checking if RX buffer is full.
 *
 * @retval true  RX buffer is full.
 * @retval false RX buffer is not full.
 */
static inline bool isRxBufferFull()
{
    uint16_t next = (sReceiveHead + 1) % UART_RX_BUFFER_SIZE;
    return (next == sReceiveTail);
}

/**
 * Function for checking if RX buffer is empty.
 *
 * @retval true  RX buffer is empty.
 * @retval false RX buffer is not empty.
 */
static inline bool isRxBufferEmpty()
{
    return (sReceiveHead == sReceiveTail);
}

/**
 * Function for notifying application about new bytes received.
 */
static void processReceive(void)
{
    // Set head position to not be changed during read procedure.
    uint16_t head = sReceiveHead;

    otEXPECT(isRxBufferEmpty() == false);

    // In case head roll back to the beginning of the buffer, notify about left
    // bytes from the end of the buffer.
    if (head < sReceiveTail)
    {
        otPlatUartReceived(&sReceiveBuffer[sReceiveTail], (UART_RX_BUFFER_SIZE - sReceiveTail));
        sReceiveTail = 0;
    }

    // Notify about received bytes.
    if (head > sReceiveTail)
    {
        otPlatUartReceived(&sReceiveBuffer[sReceiveTail], (head - sReceiveTail));
        sReceiveTail = head;
    }

exit:
    return;
}

/**
 * Function for notifying application about transmission being done.
 */
static void processTransmit(void)
{
    if (sTransmitDone==false && CDC_Transmit_Done_FS()==USBD_OK)
    {
        // Clear Transmition transaction and notify application.
        sTransmitBuffer = NULL;
        sTransmitLength = 0;
        sTransmitDone   = true;
        otPlatUartSendDone();
    }

    return;
}

void stm32f4UartProcess(void)
{
    processReceive();
    processTransmit();
}

void stm32f4UartInit(void)
{
    // Intentionally empty.
}

void stm32f4UartDeinit(void)
{
    if (sUartEnabled)
    {
        otPlatUartDisable();
    }
}

otError otPlatUartEnable(void)
{
    otError error = OT_ERROR_NONE;

    otEXPECT_ACTION(sUartEnabled == false, error = OT_ERROR_ALREADY);

    sUartEnabled = true;

exit:
    return error;
}

otError otPlatUartDisable(void)
{
    otError error = OT_ERROR_NONE;

    otEXPECT_ACTION(sUartEnabled == true, error = OT_ERROR_ALREADY);

    sUartEnabled = false;

exit:
    return error;
}

otError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    otError error = OT_ERROR_NONE;

    otEXPECT_ACTION(sUartEnabled == true, error = OT_ERROR_DISABLED_FEATURE);
    otEXPECT_ACTION(CDC_Transmit_FS((uint8_t*)aBuf, aBufLength)==USBD_OK, error = OT_ERROR_BUSY);
    sTransmitDone=false;

exit:
    return error;
}

#endif // USB_CDC_AS_SERIAL_TRANSPORT == 0

/**
 * The UART driver weak functions definition.
 *
 */
OT_TOOL_WEAK void otPlatUartSendDone(void)
{
}

OT_TOOL_WEAK void otPlatUartReceived(const uint8_t *aBuf, uint16_t aBufLength)
{
    (void)aBuf;
    (void)aBufLength;
}

void otPlatUartReceived_LL(const uint8_t *aBuf, uint16_t aBufLength){
    otEXPECT(sUartEnabled == true);
    const uint8_t *end=aBuf+aBufLength;


    while(aBuf<end && !isRxBufferFull() )
    {
    	sReceiveBuffer[sReceiveHead] = *aBuf++;
    	sReceiveHead                 = (sReceiveHead + 1) % UART_RX_BUFFER_SIZE;
//    	PlatformEventSignalPending();
    }

	exit:
	    return;
}
