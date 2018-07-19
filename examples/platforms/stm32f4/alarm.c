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
 *   This file implements the OpenThread platform abstraction for the alarm.
 *
 */

#include <openthread-core-config.h>
#include <openthread/config.h>

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <openthread/platform/alarm-micro.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/diag.h>
#include <openthread/platform/time.h>

#include "platform.h"

#include "platform-config.h"

#include <openthread/config.h>
#include <openthread/types.h>

static volatile uint64_t stm32_ticks=0;

#define XTAL_ACCURACY       40

typedef struct
{
    volatile bool mFireAlarm;  ///< Information for processing function, that alarm should fire.
    uint64_t      mTargetTime; ///< Alarm fire time (in millisecond for MsTimer, in microsecond for UsTimer)
} AlarmData;
static AlarmData         sTimerData; ///< Data of the timers.

uint64_t stm32f4AlarmGetCurrentTime(void){
	return stm32_ticks;
}

void stm32f4AlarmInit(void)
{
    memset(&sTimerData, 0, sizeof(sTimerData));
}

void stm32f4AlarmDeinit(void)
{
}

void stm32f4AlarmProcess(otInstance *aInstance)
{
    if (sTimerData.mFireAlarm)
    {
        sTimerData.mFireAlarm = false;

#if OPENTHREAD_ENABLE_DIAG

        if (otPlatDiagModeGet())
        {
            otPlatDiagAlarmFired(aInstance);
        }
        else
#endif
        {
            otPlatAlarmMilliFired(aInstance);
        }
    }
}


uint32_t otPlatAlarmMilliGetNow(void)
{
	return (uint32_t)stm32_ticks;
}

void otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    (void)aInstance;

    uint64_t now;
    now = stm32f4AlarmGetCurrentTime();

    // Check if 32 LSB of `now` overflowed between getting aT0 and loading `now` value.
    if ((uint32_t)now < aT0)
    {
        now -= 0x0000000100000000;
    }

    sTimerData.mTargetTime = (now & 0xffffffff00000000) + aT0 + aDt;
    sTimerData.mFireAlarm=false;
}

void otPlatAlarmMilliStop(otInstance *aInstance)
{
    (void)aInstance;
    sTimerData.mFireAlarm = false;
}

#if OPENTHREAD_CONFIG_ENABLE_TIME_SYNC
uint64_t otPlatTimeGet(void)
{
    return stm32f4AlarmGetCurrentTime();
}

uint16_t otPlatTimeGetXtalAccuracy(void)
{
    return XTAL_ACCURACY;
}
#endif // OPENTHREAD_CONFIG_ENABLE_TIME_SYNC

void alarmTick(){
	stm32_ticks++;

	if(stm32_ticks >= sTimerData.mTargetTime){
        sTimerData.mFireAlarm = true;
	}
}
