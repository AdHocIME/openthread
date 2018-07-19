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
 *   This file implements a random number generator.
 *
 */

#include <openthread-core-config.h>
#include <openthread/config.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <utils/code_utils.h>
#include <openthread/platform/random.h>

#include "platform-config.h"

#include <stm32f4xx.h>
#include <stm32f4xx_hal_rng.h>

static uint8_t           sBuffer[RNG_BUFFER_SIZE];
static volatile uint32_t sReadPosition;
static volatile uint32_t sWritePosition;

static inline uint32_t bufferCount(void)
{
    uint32_t writePos = sWritePosition;
    return writePos - sReadPosition;
}

static inline bool bufferIsEmpty(void)
{
    return (bufferCount() == 0);
}


static inline bool bufferIsFull(void)
{
    return (bufferCount() >= RNG_BUFFER_SIZE);
}

static inline void bufferPut(uint8_t val)
{
    if (!bufferIsFull())
    {
        sBuffer[(sWritePosition++) % RNG_BUFFER_SIZE] = val;
    }
}

static inline bool bufferIsUint32Ready(void)
{
    return (bufferCount() >= 4);
}

static inline uint8_t bufferGet()
{
    uint8_t retVal = 0;

    if (!bufferIsEmpty())
    {
        retVal = sBuffer[sReadPosition++ % RNG_BUFFER_SIZE];
    }

    return retVal;
}

static inline uint32_t bufferGetUint32()
{
    uint32_t retVal = 0;

    if (bufferIsUint32Ready())
    {
        for (uint32_t i = 0; i < 4; i++)
        {
            retVal <<= 8;
            retVal |= bufferGet();
        }
    }

    return retVal;
}

extern RNG_HandleTypeDef hrng;

static void generatorStart(void)
{
	HAL_RNG_GenerateRandomNumber_IT(&hrng);
}

static void generatorStop(void)
{
}

void HAL_RNG_ErrorCallback(RNG_HandleTypeDef *hrng){
	HAL_RNG_GenerateRandomNumber_IT(hrng);
}

void HAL_RNG_ReadyDataCallback(RNG_HandleTypeDef* hrng, uint32_t random32bit)
{
	bufferPut(random32bit);

	if (!bufferIsFull())
	{
		HAL_RNG_GenerateRandomNumber_IT(hrng);
	}
}


void stm32f4RandomInit(void)
{
    uint32_t seed = 0;

    memset(sBuffer, 0, sizeof(sBuffer));
    sReadPosition  = 0;
    sWritePosition = 0;

    generatorStart();

    // Wait for the first randomized 4 bytes, to randomize software generator seed.
    while (!bufferIsUint32Ready());

    seed = bufferGetUint32();

    srand(seed);
}

void stm32f4RandomDeinit(void)
{
    generatorStop();
}

uint32_t otPlatRandomGet(void)
{
    return (uint32_t)rand();
}

otError otPlatRandomGetTrue(uint8_t *aOutput, uint16_t aOutputLength)
{
    otError  error = OT_ERROR_NONE;
    uint8_t  copyLength;
    uint16_t index = 0;

    otEXPECT_ACTION(aOutput && aOutputLength, error = OT_ERROR_INVALID_ARGS);

    do
    {
        copyLength = (uint8_t)bufferCount();
        if (copyLength > aOutputLength - index)
        {
            copyLength = aOutputLength - index;
        }
        if (copyLength > 0)
        {

            for (uint32_t i = 0; i < copyLength; i++)
            {
                aOutput[i + index] = bufferGet();
            }

            generatorStart();

            index += copyLength;
        }
    } while (index < aOutputLength);

exit:
    return error;
}
