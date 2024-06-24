
#ifndef __STM32F7XX_GRBL_H
#define __STM32F7XX_GRBL_H

/* includes */
#include <stdint.h>
#include "main.h"
#include "stm32f7xx_flash_ex.h"
#include "stm32f7xx_gpio_ex.h"
#include "system_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "utils.h"

/* exported functions */
void vLoggingPrintf(const char *pcFormatString, ...);

#endif // __STM32F7XX_GRBL_H