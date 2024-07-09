#include "stm32f7xx_grbl.h"
#include "utils.h"
#include "timers.h"
#include "grbl.h"

#define DEBOUNCE_TIME_MS 5

extern TaskHandle_t mainGRBLTaskHandle;
TIM_HandleTypeDef *htimUs; // Timer for microsecond delay
TimerHandle_t debouncingTimerHandle_LimitSwitch[NUM_DIMENSIONS];
uint16_t LimitSwitch_Pin[NUM_DIMENSIONS] = {X_LIMIT_Pin, Y_LIMIT_Pin, Z_LIMIT_Pin};
uint32_t usDelay = 0;

void debouncingTimerCallback_LimitSwitch(TimerHandle_t xTimer)
{
    uint32_t axis = (uint32_t)pvTimerGetTimerID(xTimer);
    uint16_t GPIO_Pin = LimitSwitch_Pin[axis];

    // read pin state
    // triggered is low and not triggered is high by default
    IO_TYPE pinState = (LIMIT_PIN & LIMIT_MASK) ^ LIMIT_MASK;

    // determine whether to trigger limit pin isr or not
    // with taking into consideration the inverted pin mask
    // if it is defined.
#ifdef INVERT_LIMIT_PIN_MASK
    pinState ^= INVERT_LIMIT_PIN_MASK;
#endif
    if (pinState & GPIO_Pin)
    {
        // stop timer
        xTimerStop(xTimer, 0);

        // trigger limit switch event
        limits_isr(GPIO_Pin);
    }
}

void utilsStartUsTimer(TIM_HandleTypeDef *htim)
{
    htimUs = htim;

    // create software timer
    for (uint32_t i = 0; i < NUM_DIMENSIONS; i++)
    {
        debouncingTimerHandle_LimitSwitch[i] = xTimerCreate("DebouncingTimer_LimitSwitch", pdMS_TO_TICKS(DEBOUNCE_TIME_MS), pdFALSE, (void *)i, debouncingTimerCallback_LimitSwitch);
    }
}

void utilsDebounceLimitSwitches(uint16_t GPIO_Pin)
{
  if (GPIO_Pin & (1 << X_LIMIT_BIT))
  {
    // disable IRQ
    HAL_NVIC_DisableIRQ(X_LIMIT_EXTI_IRQn);

    // start timer
    xTimerStartFromISR(debouncingTimerHandle_LimitSwitch[X_AXIS], 0);

    // enable IRQ
    HAL_NVIC_EnableIRQ(X_LIMIT_EXTI_IRQn);
  }
  else if (GPIO_Pin & (1 << Y_LIMIT_BIT))
  {
    // disable IRQ
    HAL_NVIC_DisableIRQ(Y_LIMIT_EXTI_IRQn);

    // start timer
    xTimerStartFromISR(debouncingTimerHandle_LimitSwitch[Y_AXIS], 0);

    // enable IRQ
    HAL_NVIC_EnableIRQ(Y_LIMIT_EXTI_IRQn);
  }
  else if (GPIO_Pin & (1 << Z_LIMIT_BIT))
  {
    // disable IRQ
    HAL_NVIC_DisableIRQ(Z_LIMIT_EXTI_IRQn);

    // start timer
    xTimerStartFromISR(debouncingTimerHandle_LimitSwitch[Z_AXIS], 0);

    // enable IRQ
    HAL_NVIC_EnableIRQ(Z_LIMIT_EXTI_IRQn);
  }
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
    while (usDelay)
        ;
}

void utilsUsTimerInterruptHandler()
{
    if (--usDelay)
        return;

    // stop timer
    HAL_TIM_Base_Stop_IT(htimUs);
}

// debouncing
HAL_StatusTypeDef utilsDebouncePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t ms, uint8_t state)
{
    utilsDelayMs(ms);

    uint8_t pinState = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);

    if (pinState == state)
        return HAL_OK;

    return HAL_ERROR;
}