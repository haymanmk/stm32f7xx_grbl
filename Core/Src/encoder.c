#include <stdlib.h>
#include "stm32f7xx_grbl.h"
#include "stm32f7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "encoder.h"

#define FULL_COUNTER 0xFFFF
#define HALF_COUNTER 0x7FFF
// resolution of degree after decimal point
#define DEGREE_RESOLUTION 100        // 2 decimal points
#define PULSE_PER_REVOLUTION 2000    // 2000 pulses per revolution
#define CALCULATE_RPM_PERIOD_MS 1000 // 1 second = 1000 ms

/* Type define */
typedef struct
{
    TIM_HandleTypeDef *htim;
    int32_t counter32Bit;
    uint16_t prevCounter;
    int16_t revolution;
} encoder_param_t;

static encoder_param_t encoder_param[NUM_DIMENSIONS] = ENCODER_PARAM_ARRAY_INIT;

volatile uint32_t previousTicks = 0;  // store previous ticks
volatile uint32_t previousDegree = 0; // store previous degree value
volatile float speedRPM = 0;          // store speed in RPM

void encoderInit()
{
    // Enable the encoder interrupt
    HAL_TIM_Encoder_Start(&X_ENCODER_TIM_HANDLE, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&Y_ENCODER_TIM_HANDLE, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&Z_ENCODER_TIM_HANDLE, TIM_CHANNEL_ALL);

    // reset encoder counter
    encoderResetCounter(X_AXIS);
    encoderResetCounter(Y_AXIS);
    encoderResetCounter(Z_AXIS);

    // start encoder read position task
    xTaskCreate(encoderReadPositionTask, "EncoderReadPositionTask", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY, NULL);
}

void encoderReadPositionTask(void *pvParameters)
{

    // Infinite loop
    for (;;)
    {
        // print encoder degree
        encoder_degree_t degree;
        encoderReadDegree(&degree);
        vLoggingPrintf("X: %.2f, Y: %.2f, Z: %.2f\n", degree[X_AXIS], degree[Y_AXIS], degree[Z_AXIS]);

        // Delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void encoderInterruptHandler()
{
    uint16_t currentCounterArr[NUM_DIMENSIONS] = {0};

    // read encoder counter value of each axis at once in order to get simultaneous value
    for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
    {
        encoder_param_t *encoder = &encoder_param[i];
        currentCounterArr[i] = __HAL_TIM_GET_COUNTER(encoder->htim);
    }

    for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
    {
        encoder_param_t *encoder = &encoder_param[i];
        // get current counter value
        uint16_t currentCounter = currentCounterArr[i];

        // Detect Overflow / Underflow event to Increase / Decrease revolution counter
        int32_t diff = currentCounter - encoder->prevCounter;

        if (diff < 0)
        {
            // overflow occurred
            if (-diff > HALF_COUNTER)
                encoder->revolution++;
        }
        else
        {
            // underflow occurred
            if (diff > HALF_COUNTER)
                encoder->revolution--;
        }

        encoder->counter32Bit = (int32_t)(encoder->revolution << 16) + currentCounter;
        encoder->prevCounter = currentCounter;
    }
}

void encoderResetCounter(axis_t axis)
{
    encoder_param[axis].counter32Bit = 0;
    encoder_param[axis].prevCounter = 0;
    encoder_param[axis].revolution = 0;

    __HAL_TIM_SET_COUNTER(encoder_param[axis].htim, 0);
}

/**
 * @brief Read the degree value from the encoder
 * @return int32_t degree value x 100
 */
void encoderReadDegree(encoder_degree_t *degree)
{
    for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
    {
        encoder_param_t *encoder = &encoder_param[i];
        *(float*)(degree)[i] = (float)(encoder->counter32Bit * (360.0 / PULSE_PER_REVOLUTION));
    }
}

/**
 * @brief Read the degree value from current timer's counter instead of reading from the recorded value
 *        which will get more accurate value.
 */
void encoderReadInstantDegree(encoder_degree_t *degree)
{
    // update counter value
    encoderInterruptHandler();

    // read degree value
    encoderReadDegree(degree);
}

/**
 * TODO: RPM calculation
 */
/*
void encoderCalculateRPM()
{
    // get current ticks in milliseconds
    uint32_t currentTicks = HAL_GetTick();
    uint32_t diffTicks = currentTicks - previousTicks;

    // if time period is less than 1 second, return
    if (diffTicks < CALCULATE_RPM_PERIOD_MS)
        return;

    // update previous ticks
    previousTicks = currentTicks;

    // get current degree
    int32_t currentDegree = encoderReadDegree();
    int32_t diffDegree = currentDegree - previousDegree;
    // update previous degree
    previousDegree = currentDegree;

    // calculate RPM
    speedRPM = ((float)diffDegree / (360.0 * (float)DEGREE_RESOLUTION)) * (60000.0 / (float)diffTicks);
}

float encoderReadRPM()
{
    return speedRPM;
}


uint32_t encoderGetCounter(axis_t axis)
{
    return encoder_param[axis].counter32Bit;
}


*/