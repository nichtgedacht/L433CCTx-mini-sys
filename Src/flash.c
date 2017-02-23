#include "flash.h"

uint32_t PAGEError = 0;

static FLASH_EraseInitTypeDef EraseInitStruct;

uint64_t flash_buf[128];

uint32_t erase_flash_page(void)
{
    //Erase flash page(s)
    //Fill EraseInit structure
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page = FLASH_PAGE;
    EraseInitStruct.Banks = 0;
    EraseInitStruct.NbPages = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;

    HAL_FLASH_Unlock();

    /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
    {
        // PAGEError will contain the faulty page
        // error code from 'HAL_FLASH_GetError()'
        HAL_FLASH_Lock();
        return HAL_FLASH_GetError();
    }
    else
    {
        HAL_FLASH_Lock();
        return HAL_OK;
    }
}

uint32_t write_flash_vars(uint64_t* data, uint16_t length, uint16_t offset)
{
    uint32_t Address = FLASH_USER_START_ADDR;
    __IO uint64_t data64 = 0;

    Address += offset;

    HAL_FLASH_Unlock();

    while (length > 0)
    {
        data64 = *(__IO uint64_t *) data;

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, data64) == HAL_OK)
        {
            Address += 8;
            data++;
            length--;
        }
        else
        {
            HAL_FLASH_Lock();
            return HAL_FLASH_GetError();
        }
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

void read_flash_vars(uint32_t *data, uint16_t length, uint16_t offset)
{
    uint32_t Address = FLASH_USER_START_ADDR;
    uint16_t i;

    Address += offset;

    for (i=0; i<length; i++)
    {
            *data = *(uint32_t*) Address;
            Address += 4;
            data ++;
    }
}


