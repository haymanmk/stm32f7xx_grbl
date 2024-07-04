
#ifndef __UTILS_H
#define __UTILS_H

/**
 * Define Function Prototypes
 */
void utilsStartUsTimer(TIM_HandleTypeDef *htim);
void utilsDelayMs(uint16_t ms);

/**
 * TODO: Implement delay in microseconds
 */
void utilsDelayUs(uint32_t us);
void utilsUsTimerInterruptHandler();
HAL_StatusTypeDef utilsDebouncePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t ms, uint8_t state);

#endif // __UTILS_H