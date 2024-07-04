#include "stm32f7xx_grbl.h"
#include "utils.h"

extern TaskHandle_t mainGRBLTaskHandle;
TIM_HandleTypeDef* htimUs; // Timer for microsecond delay
uint32_t usDelay = 0;

void utilsStartUsTimer(TIM_HandleTypeDef *htim)
{
    htimUs = htim;
}

void utilsDelayMs(uint16_t ms)
{
    utilsDelayUs(ms * 1000);
}

/**
 * TODO: Implement delay in microseconds
 */
void utilsDelayUs(uint32_t us)
{
    // reset timer counter
    __HAL_TIM_SET_COUNTER(htimUs, 0);

    HAL_TIM_Base_Start_IT(htimUs);
    usDelay = us;

    // wait for timer to expire
    while(usDelay);
}

void utilsUsTimerInterruptHandler()
{
    if(--usDelay) return;

    // stop timer
    HAL_TIM_Base_Stop_IT(htimUs);
}

// debouncing
HAL_StatusTypeDef utilsDebouncePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t ms, uint8_t state)
{
    utilsDelayMs(ms);

    uint8_t pinState = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);

    if(pinState == state) return HAL_OK;

    return HAL_ERROR;
}