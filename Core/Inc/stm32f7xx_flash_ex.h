
#ifndef __STM32F7xx_FLASH_EX_H
#define __STM32F7xx_FLASH_EX_H

#include "stm32f7xx_hal.h"

/**
 * Define Function Prototypes
 */
void flashInit();
HAL_StatusTypeDef flashPutByte(uint32_t address, uint8_t data);
uint8_t flashGetByte(uint32_t address);
HAL_StatusTypeDef flashWriteVersion(uint8_t new_version);
void flashMemcpyToEepromWithChecksum(uint32_t destination, char *source, uint32_t size);

#endif // __STM32F7xx_FLASH_EX_H