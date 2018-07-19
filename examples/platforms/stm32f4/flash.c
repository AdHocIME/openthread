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

#include <openthread-core-config.h>
#include <openthread/config.h>

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <utils/code_utils.h>
#include <utils/flash.h>
#include <openthread/types.h>
#include <openthread/platform/alarm-milli.h>

#include <stm32f4xx.h>

#define FLASH_PAGE_ADDR_MASK 0xFFFFF000
#define FLASH_PAGE_SIZE SETTINGS_CONFIG_PAGE_SIZE

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

static uint32_t sFlashDataStart;
static uint32_t sFlashDataEnd;

static uint32_t GetSector(uint32_t Address);

static inline uint32_t mapAddress(uint32_t aAddress)
{
    return aAddress + sFlashDataStart;
}

otError utilsFlashInit(void)
{
#if defined(__CC_ARM)
    // Temporary solution for Keil compiler.
    uint32_t const bootloaderAddr = NRF_UICR->NRFFW[0];
    uint32_t const pageSize       = NRF_FICR->CODEPAGESIZE;
    uint32_t const codeSize       = NRF_FICR->CODESIZE;

    if (bootloaderAddr != 0xFFFFFFFF)
    {
        sFlashDataEnd = bootloaderAddr;
    }
    else
    {
        sFlashDataEnd = pageSize * codeSize;
    }

    sFlashDataStart = sFlashDataEnd - (pageSize * SETTINGS_CONFIG_PAGE_NUM);

#elif defined(__GNUC__) || defined(__ICCARM__)
    extern uint32_t __start_ot_flash_data;
    extern uint32_t __stop_ot_flash_data;

    sFlashDataStart = 0x10000000;
    sFlashDataEnd   = 0x10010000;

    HAL_FLASH_Unlock();

#endif

    // Just ensure that the start and end addresses are page-aligned.
    assert((sFlashDataStart % FLASH_PAGE_SIZE) == 0);
    assert((sFlashDataEnd % FLASH_PAGE_SIZE) == 0);

    return OT_ERROR_NONE;
}

uint32_t utilsFlashGetSize(void)
{
    return sFlashDataEnd - sFlashDataStart;
}

otError utilsFlashErasePage(uint32_t aAddress)
{
    otError error = OT_ERROR_NONE;
    otEXPECT_ACTION(aAddress < utilsFlashGetSize(), error = OT_ERROR_INVALID_ARGS);

    uint32_t SectorError = 0;
    FLASH_EraseInitTypeDef EraseInitStruct;

    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = GetSector(aAddress);
    EraseInitStruct.NbSectors = 1;

    memset((uint8_t *)mapAddress(aAddress), 0xFF, FLASH_PAGE_SIZE);

//    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
    {
      /*
        Error occurred while sector erase.
        User can add here some code to deal with this error.
        SectorError will contain the faulty sector and then to know the code error on this sector,
        user can call function 'HAL_FLASH_GetError()'
      */
      /*
        FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
      */
//      Error_Handler();
    }


//    error = nrf5FlashPageErase(mapAddress(aAddress & FLASH_PAGE_ADDR_MASK));

exit:
    return error;
}

otError utilsFlashStatusWait(uint32_t aTimeout)
{
    otError error = OT_ERROR_BUSY;

    if (aTimeout == 0)
    {
//        if (!nrf5FlashIsBusy())
//        {
//            error = OT_ERROR_NONE;
//        }
    }
    else
    {
        uint32_t startTime = otPlatAlarmMilliGetNow();

        do
        {
//            if (!nrf5FlashIsBusy())
//            {
                error = OT_ERROR_NONE;
                break;
//            }
        } while (otPlatAlarmMilliGetNow() - startTime < aTimeout);
    }

    return error;
}

uint32_t utilsFlashWrite(uint32_t aAddress, uint8_t *aData, uint32_t aSize)
{
    uint32_t result = 0;
    otEXPECT(aData);
    otEXPECT(aAddress < utilsFlashGetSize());
    otEXPECT(aSize);

    memcpy((uint8_t *)mapAddress(aAddress), aData, aSize);
    result = aSize;
//    result = nrf5FlashWrite(mapAddress(aAddress), aData, aSize);

exit:
    return result;
}

uint32_t utilsFlashRead(uint32_t aAddress, uint8_t *aData, uint32_t aSize)
{
    uint32_t result = 0;
    otEXPECT(aData);
    otEXPECT(aAddress < utilsFlashGetSize());

    memcpy(aData, (uint8_t *)mapAddress(aAddress), aSize);
    result = aSize;

exit:
    return result;
}



/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
  {
    sector = FLASH_SECTOR_11;
  }

  return sector;
}
