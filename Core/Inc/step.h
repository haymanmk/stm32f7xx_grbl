
#ifndef __STEP_H
#define __STEP_H

/* defines */
#define DOUBLE_BUFFER_SIZE 64
#define RING_BUFFER_SIZE 8
#define GAP_HEAD_TAIL 3    // gap between head and tail, two buffers in used by DMA
#define MAX_TICKS 0xFFFFFFFF                                                                                // maximum value of 32-bit timer
#define PULSE_WIDTH_US 5UL                                                                                  // pulse width of HIGH state in microseconds
#define MINIMUN_LOW_PULSE_WIDTH_US 5UL                                                                      // minimum pulse width of LOW state in microseconds
#define PULSE_WIDTH_TICKS (uint32_t)((PULSE_WIDTH_US * APB1_TIMER_CLOCK) / 1000000)                         // pulse width in ticks
#define MINIMUN_LOW_PULSE_WIDTH_TICKS (uint32_t)((MINIMUN_LOW_PULSE_WIDTH_US * APB1_TIMER_CLOCK) / 1000000) // minimum pulse width in ticks
#define MINIMUM_PULSE_PERIOD_TICKS (PULSE_WIDTH_TICKS + MINIMUN_LOW_PULSE_WIDTH_TICKS)                      // minimum pulse period in ticks

/**
 * DMA 1 Stream Assignment:
 * x-axis: stream 1
 * y-axis: stream 6
 * z-axis: stream 2
 */
#define X_AXIS_TIM_BASE TIM2_BASE
#define Y_AXIS_TIM_BASE TIM2_BASE
#define Z_AXIS_TIM_BASE TIM5_BASE
#define X_AXIS_TIM ((TIM_TypeDef *)X_AXIS_TIM_BASE)
#define Y_AXIS_TIM ((TIM_TypeDef *)Y_AXIS_TIM_BASE)
#define Z_AXIS_TIM ((TIM_TypeDef *)Z_AXIS_TIM_BASE)
#define MASTER_TIM X_AXIS_TIM
#define X_AXIS_TIM_HANDLE htim2
#define Y_AXIS_TIM_HANDLE htim2
#define Z_AXIS_TIM_HANDLE htim5
#define MASTER_TIM_HANDLE X_AXIS_TIM_HANDLE
#define X_AXIS_PULSE_TIM_ACTIVE_CHANNEL HAL_TIM_ACTIVE_CHANNEL_3
#define Y_AXIS_PULSE_TIM_ACTIVE_CHANNEL HAL_TIM_ACTIVE_CHANNEL_2
#define Z_AXIS_PULSE_TIM_ACTIVE_CHANNEL HAL_TIM_ACTIVE_CHANNEL_1
#define X_AXIS_PULSE_TIM_DMA_ID TIM_DMA_ID_CC3
#define Y_AXIS_PULSE_TIM_DMA_ID TIM_DMA_ID_CC4
#define Z_AXIS_PULSE_TIM_DMA_ID TIM_DMA_ID_CC1
#define X_AXIS_PULSE_TIM_CHANNEL TIM_CHANNEL_3
#define Y_AXIS_PULSE_TIM_CHANNEL TIM_CHANNEL_4
#define Z_AXIS_PULSE_TIM_CHANNEL TIM_CHANNEL_1
#define X_AXIS_PULSE_TIM_CCRx (uint32_t)(&(X_AXIS_TIM_HANDLE.Instance->CCR3))
#define Y_AXIS_PULSE_TIM_CCRx (uint32_t)(&(Y_AXIS_TIM_HANDLE.Instance->CCR4))
#define Z_AXIS_PULSE_TIM_CCRx (uint32_t)(&(Z_AXIS_TIM_HANDLE.Instance->CCR1))
#define X_AXIS_COMPARE_EVENT_ID TIM_EVENTSOURCE_CC3
#define Y_AXIS_COMPARE_EVENT_ID TIM_EVENTSOURCE_CC4
#define Z_AXIS_COMPARE_EVENT_ID TIM_EVENTSOURCE_CC1
#define X_AXIS_STEP_BIT X_STEP_BIT
#define Y_AXIS_STEP_BIT Y_STEP_BIT
#define Z_AXIS_STEP_BIT Z_STEP_BIT
#define X_AXIS_DIRECTION_BIT X_DIRECTION_BIT
#define Y_AXIS_DIRECTION_BIT Y_DIRECTION_BIT
#define Z_AXIS_DIRECTION_BIT Z_DIRECTION_BIT
#define X_AXIS_DIRECTION_PORT X_DIRECTION_PORT
#define Y_AXIS_DIRECTION_PORT Y_DIRECTION_PORT
#define Z_AXIS_DIRECTION_PORT Z_DIRECTION_PORT
#define X_AXIS_TIM_FLAG_CCx TIM_FLAG_CC3
#define Y_AXIS_TIM_FLAG_CCx TIM_FLAG_CC4
#define Z_AXIS_TIM_FLAG_CCx TIM_FLAG_CC1

#define STEP_EVENT_COUNT 1000
#define STEP_X 600
#define STEP_Y 100
#define STEP_Z 100

#define GENERAL_NOTIFICATION_FIRST_TIME_START 0x01
#define GENERAL_NOTIFICATION_DATA_READY 0x02
#define GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_X_AXIS 0x04
#define GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_Y_AXIS 0x08
#define GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_Z_AXIS 0x10
#define GENERAL_NOTIFICATION_ALL_AXES_DMA_UPDATED 0x20
#define GENERAL_NOTIFICATION_CALCULATE_PULSE 0x40
#define GENERAL_NOTIFICATION_FORCE_STOP 0x80
#define GENERAL_NOTIFICATION_GET_NEW_BUFFER 0x100

#define GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_ALL_AXES \
    (GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_X_AXIS | GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_Y_AXIS | GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_Z_AXIS)

/* type define */

/**
 * NOTE: Here we use uint32_t instead of uint16_t as the format of the buffer
 *       in case the 32-bit timer does not support 16-bit DMA.
 *       There is an unexpected result occurred as uint16_t is used,
 *       which ends up with two replicates of 16-bit data concatenated
 *       into a single 32-bit register.
 */
typedef uint32_t doubleBufferArray_t[DOUBLE_BUFFER_SIZE];

typedef struct
{
    __IO uint32_t ISR; /*!< DMA interrupt status register */
    __IO uint32_t Reserved0;
    __IO uint32_t IFCR; /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

/* Macros */
#define CONCATENATE(A, B) A##B

#define COMPUTE_INCREMENT_FOR_PULSE(pulse_freq, clock_freq) ((uint32_t)clock_freq / (uint32_t)pulse_freq)

// get TIM capture compare register value on runtime
#define TIM_GET_COMPARE(__HANDLE__, __ACTIVE_CHANNEL__)                                                                                                                        \
    (((__ACTIVE_CHANNEL__) == HAL_TIM_ACTIVE_CHANNEL_1) ? ((__HANDLE__)->Instance->CCR1) : ((__ACTIVE_CHANNEL__) == HAL_TIM_ACTIVE_CHANNEL_2) ? ((__HANDLE__)->Instance->CCR2) \
                                                                                       : ((__ACTIVE_CHANNEL__) == HAL_TIM_ACTIVE_CHANNEL_3)   ? ((__HANDLE__)->Instance->CCR3) \
                                                                                       : ((__ACTIVE_CHANNEL__) == HAL_TIM_ACTIVE_CHANNEL_4)   ? ((__HANDLE__)->Instance->CCR4) \
                                                                                       : ((__ACTIVE_CHANNEL__) == HAL_TIM_ACTIVE_CHANNEL_5)   ? ((__HANDLE__)->Instance->CCR5) \
                                                                                                                                              : ((__HANDLE__)->Instance->CCR6))

// suspend DMA stream
#define SUSPEND_DMA_STREAM(__DMA_HANDLE__) \
    do \
    { \
        __HAL_DMA_DISABLE((__DMA_HANDLE__)); \
        while ((__DMA_HANDLE__)->Instance->CR & DMA_SxCR_EN); \
    } while (0);

// clear all DMA interrupt flags
// __DMA_HANDLE__ is the DMA_HandleTypeDef structure
#define CLEAR_DMA_IT(__DMA_HANDLE__)                                                          \
    do                                                                                        \
    {                                                                                         \
        DMA_Base_Registers *regs = (DMA_Base_Registers *)(__DMA_HANDLE__)->StreamBaseAddress; \
        regs->IFCR = 0x3FU << (__DMA_HANDLE__)->StreamIndex;                                  \
    } while (0);

// Resume DMA stream with transfer complete interrupt being set in CR
#define RESUME_DMA_STREAM_WITH_TC(__DMA_HANDLE__, __BUFFER__, __LENGTH__) \
    do                                                                    \
    {                                                                     \
        CLEAR_DMA_IT(__DMA_HANDLE__);                                     \
        (__DMA_HANDLE__)->Instance->M0AR = (uint32_t)(__BUFFER__);        \
        (__DMA_HANDLE__)->Instance->NDTR = (uint32_t)(__LENGTH__);        \
        (__DMA_HANDLE__)->Instance->CR |= DMA_IT_TC;                      \
        __HAL_DMA_ENABLE((__DMA_HANDLE__));                               \
    } while (0);

// disabling and enabling DIER register
#define OFF_AND_ON_DIER(__TIM_HANDLE__, __CCxDE__) \
    do                                             \
    {                                              \
        (__TIM_HANDLE__)->Instance->DIER &= ~(__CCxDE__); \
        (__TIM_HANDLE__)->Instance->DIER |= (__CCxDE__);  \
    } while (0);

// clear pending DMA request in timer by disabling and enabling the DMA request bit in DIER register
#define CLEAR_PENDING_DMA_REQUEST(__TIM_HANDLE__, __TIM_CHANNEL__) \
    do                                                         \
    {                                                          \
        if ((__TIM_CHANNEL__) == TIM_CHANNEL_1) {                \
            (__TIM_HANDLE__)->Instance->SR &= ~TIM_SR_CC1IF;    \
            OFF_AND_ON_DIER(__TIM_HANDLE__, TIM_DIER_CC1DE)     \
        } else if ((__TIM_CHANNEL__) == TIM_CHANNEL_2) {        \
            (__TIM_HANDLE__)->Instance->SR &= ~TIM_SR_CC2IF;    \
            OFF_AND_ON_DIER(__TIM_HANDLE__, TIM_DIER_CC2DE)     \
        } else if ((__TIM_CHANNEL__) == TIM_CHANNEL_3) {        \
            (__TIM_HANDLE__)->Instance->SR &= ~TIM_SR_CC3IF;    \
            OFF_AND_ON_DIER(__TIM_HANDLE__, TIM_DIER_CC3DE)     \
        } else if ((__TIM_CHANNEL__) == TIM_CHANNEL_4) {        \
            (__TIM_HANDLE__)->Instance->SR &= ~TIM_SR_CC4IF;    \
            OFF_AND_ON_DIER(__TIM_HANDLE__, TIM_DIER_CC4DE) }   \
    } while (0);


// Timer start counter
#define TIM_START_COUNTER(__HANDLE__) ((__HANDLE__).Instance->CR1 |= TIM_CR1_CEN)

// Timer stop counter
#define TIM_STOP_COUNTER(__HANDLE__) ((__HANDLE__).Instance->CR1 &= ~TIM_CR1_CEN)

// swap buffer
#define SWAP_BUFFER_UINT32(__BUFFER1__, __BUFFER2__) \
    do                                               \
    {                                                \
        doubleBufferArray_t *temp = __BUFFER1__;     \
        __BUFFER1__ = __BUFFER2__;                   \
        __BUFFER2__ = temp;                          \
    } while (0);

// initialize TIM_DMA_Parameters_t
#define INIT_TIM_DMA_PARAMETERS(__TARGET_ARR__, __AXIS__)                                                 \
    do                                                                                                    \
    {                                                                                                     \
        (__TARGET_ARR__)[__AXIS__].htim = &CONCATENATE(__AXIS__, _TIM_HANDLE);                            \
        (__TARGET_ARR__)[__AXIS__].TIM_DMA_ID = CONCATENATE(__AXIS__, _PULSE_TIM_DMA_ID);                 \
        (__TARGET_ARR__)[__AXIS__].TIM_DMA_ID_CC = CONCATENATE(__AXIS__, _PULSE_TIM_ACTIVE_CHANNEL);      \
        (__TARGET_ARR__)[__AXIS__].TIM_CHANNEL = CONCATENATE(__AXIS__, _PULSE_TIM_CHANNEL);               \
        (__TARGET_ARR__)[__AXIS__].TIM_ACTIVE_CHANNEL = CONCATENATE(__AXIS__, _PULSE_TIM_ACTIVE_CHANNEL); \
        (__TARGET_ARR__)[__AXIS__].CCRx_Addr = CONCATENATE(__AXIS__, _PULSE_TIM_CCRx);                    \
        (__TARGET_ARR__)[__AXIS__].CompareEventID = CONCATENATE(__AXIS__, _COMPARE_EVENT_ID);             \
        (__TARGET_ARR__)[__AXIS__].Step_Bit = CONCATENATE(__AXIS__, _STEP_BIT);                           \
        (__TARGET_ARR__)[__AXIS__].Dir_Bit = CONCATENATE(__AXIS__, _DIRECTION_BIT);                       \
        (__TARGET_ARR__)[__AXIS__].TIM_FLAG_CCx = CONCATENATE(__AXIS__, _TIM_FLAG_CCx);                   \
    } while (0);

// define ring buffer and associated variables
#define DEFINE_RING_BUFFER(__AXIS__, __BUFFER_SIZE__) \
    static pulse_t __AXIS__##PulseBuffer[__BUFFER_SIZE__];

// increment ring buffer head
#define RING_BUFFER_INCREMENT_HEAD(__AXIS__, __RING_BUFFER_SIZE__)                         \
    do                                                                                     \
    {                                                                                      \
        uint16_t nextIndex = (pulseRingBufferHead[__AXIS__] + 1) % (__RING_BUFFER_SIZE__); \
        if (nextIndex != pulseRingBufferTail[__AXIS__])                                    \
        {                                                                                  \
            pulseRingBufferHead[__AXIS__] = nextIndex;                                     \
        }                                                                                  \
    } while (0);

// increment ring buffer tail
#define RING_BUFFER_INCREMENT_TAIL(__AXIS__, __RING_BUFFER_SIZE__)                                        \
    do                                                                                                    \
    {                                                                                                     \
        if (pulseRingBufferTail[__AXIS__] != pulseRingBufferHead[__AXIS__])                               \
        {                                                                                                 \
            pulseRingBufferTail[__AXIS__] = (pulseRingBufferTail[__AXIS__] + 1) % (__RING_BUFFER_SIZE__); \
        }                                                                                                 \
    } while (0);

// get which axis the time handle belongs to
#define GET_AXIS_FROM_TIM_HANDLE(__TIM_HANDLE__)                                                                                                                                                                                           \
    (((__TIM_HANDLE__ == &X_AXIS_TIM_HANDLE) && (__TIM_HANDLE__->Channel == X_AXIS_PULSE_TIM_ACTIVE_CHANNEL)) ? X_AXIS : ((__TIM_HANDLE__ == &Y_AXIS_TIM_HANDLE) && (__TIM_HANDLE__->Channel == Y_AXIS_PULSE_TIM_ACTIVE_CHANNEL)) ? Y_AXIS \
                                                                                                                     : ((__TIM_HANDLE__ == &Z_AXIS_TIM_HANDLE) && (__TIM_HANDLE__->Channel == Z_AXIS_PULSE_TIM_ACTIVE_CHANNEL))   ? Z_AXIS \
                                                                                                                                                                                                                                  : NUM_DIMENSIONS)

// get time handle from axis
#define GET_TIM_HANDLE_FROM_AXIS(__AXIS__)                                               \
    ((__AXIS__ == X_AXIS) ? X_AXIS_TIM_HANDLE : (__AXIS__ == Y_AXIS) ? Y_AXIS_TIM_HANDLE \
                                            : (__AXIS__ == Z_AXIS)   ? Z_AXIS_TIM_HANDLE \
                                                                     : NULL)

// get the index of specified axis in DMA handlers array
#define GET_TIM_DMA_ID_FROM_AXIS(__AXIS__)                                                           \
    ((__AXIS__ == X_AXIS) ? X_AXIS_PULSE_TIM_DMA_ID : (__AXIS__ == Y_AXIS) ? Y_AXIS_PULSE_TIM_DMA_ID \
                                                  : (__AXIS__ == Z_AXIS)   ? Z_AXIS_PULSE_TIM_DMA_ID \
                                                                           : UINT8_MAX)

// check if ring buffer is exhausted
#define IS_RING_BUFFER_EXHAUSTED(__AXIS__) (pulseRingBufferHead[__AXIS__] == pulseRingBufferTail[__AXIS__])

// get direction output bit from axis
#define GET_DIRECTION_BIT_FROM_AXIS(__AXIS__)                                                  \
    ((__AXIS__ == X_AXIS) ? X_AXIS_DIRECTION_BIT : (__AXIS__ == Y_AXIS) ? Y_AXIS_DIRECTION_BIT \
                                               : (__AXIS__ == Z_AXIS)   ? Z_AXIS_DIRECTION_BIT \
                                                                        : UINT8_MAX)

// get direction output port from axis
#define GET_DIRECTION_PORT_FROM_AXIS(__AXIS__)                                                         \
    ((__AXIS__ == X_AXIS) ? &(X_AXIS_DIRECTION_PORT) : (__AXIS__ == Y_AXIS) ? &(Y_AXIS_DIRECTION_PORT) \
                                                   : (__AXIS__ == Z_AXIS)   ? &(Z_AXIS_DIRECTION_PORT) \
                                                                            : NULL)

/**
 * @param __OC_MODE__ Output compare mode should be one of the following:
 *          TIM_OCMODE_TIMING
 *          TIM_OCMODE_ACTIVE
 *          TIM_OCMODE_INACTIVE
 *          TIM_OCMODE_TOGGLE
 *          TIM_OCMODE_PWM1
 *          TIM_OCMODE_PWM2
 *          TIM_OCMODE_FORCED_ACTIVE
 *          TIM_OCMODE_FORCED_INACTIVE
 *          TIM_OCMODE_RETRIGERRABLE_OPM1
 *          TIM_OCMODE_RETRIGERRABLE_OPM2
 *          TIM_OCMODE_COMBINED_PWM1
 *          TIM_OCMODE_COMBINED_PWM2
 *          TIM_OCMODE_ASYMMETRIC_PWM1
 *          TIM_OCMODE_ASYMMETRIC_PWM2
 */
// set output compare mode for channel 1
#define SET_OC_MODE_CHANNEL_1(__TIM_HANDLE__, __OC_MODE__)      \
    stepSetTimerOC1Mode(__TIM_HANDLE__->Instance, __OC_MODE__); \

// set output compare mode for channel 2
#define SET_OC_MODE_CHANNEL_2(__TIM_HANDLE__, __OC_MODE__)      \
    stepSetTimerOC2Mode(__TIM_HANDLE__->Instance, __OC_MODE__); \

// set output compare mode for channel 3
#define SET_OC_MODE_CHANNEL_3(__TIM_HANDLE__, __OC_MODE__)      \
    stepSetTimerOC3Mode(__TIM_HANDLE__->Instance, __OC_MODE__); \

// set output compare mode for channel 4
#define SET_OC_MODE_CHANNEL_4(__TIM_HANDLE__, __OC_MODE__)      \
    stepSetTimerOC4Mode(__TIM_HANDLE__->Instance, __OC_MODE__); \

// Force output pin to LOW in output compare mode
#define FORCE_OC_OUTPUT_LOW(__TIM_HANDLE__, __TIM_CHANNEL__)           \
    do                                                                 \
    {                                                                  \
        if ((__TIM_CHANNEL__) == TIM_CHANNEL_1)                        \
            SET_OC_MODE_CHANNEL_1(__TIM_HANDLE__, TIM_OCMODE_FORCED_INACTIVE) \
        else if ((__TIM_CHANNEL__) == TIM_CHANNEL_2)                   \
            SET_OC_MODE_CHANNEL_2(__TIM_HANDLE__, TIM_OCMODE_FORCED_INACTIVE) \
        else if ((__TIM_CHANNEL__) == TIM_CHANNEL_3)                   \
            SET_OC_MODE_CHANNEL_3(__TIM_HANDLE__, TIM_OCMODE_FORCED_INACTIVE) \
        else if ((__TIM_CHANNEL__) == TIM_CHANNEL_4)                   \
            SET_OC_MODE_CHANNEL_4(__TIM_HANDLE__, TIM_OCMODE_FORCED_INACTIVE) \
    } while (0);

// Force output pin to HIGH in output compare mode
#define FORCE_OC_OUTPUT_HIGH(__TIM_HANDLE__, __TIM_CHANNEL__)        \
    do                                                               \
    {                                                                \
        if ((__TIM_CHANNEL__) == TIM_CHANNEL_1)                      \
            SET_OC_MODE_CHANNEL_1(__TIM_HANDLE__, TIM_OCMODE_FORCED_ACTIVE) \
        else if ((__TIM_CHANNEL__) == TIM_CHANNEL_2)                 \
            SET_OC_MODE_CHANNEL_2(__TIM_HANDLE__, TIM_OCMODE_FORCED_ACTIVE) \
        else if ((__TIM_CHANNEL__) == TIM_CHANNEL_3)                 \
            SET_OC_MODE_CHANNEL_3(__TIM_HANDLE__, TIM_OCMODE_FORCED_ACTIVE) \
        else if ((__TIM_CHANNEL__) == TIM_CHANNEL_4)                 \
            SET_OC_MODE_CHANNEL_4(__TIM_HANDLE__, TIM_OCMODE_FORCED_ACTIVE) \
    } while (0);

// Set output pin to LOW in output compare mode when match
#define SET_OC_OUTPUT_LOW(__TIM_HANDLE__, __TIM_CHANNEL__)           \
    do                                                                 \
    {                                                                  \
        if ((__TIM_CHANNEL__) == TIM_CHANNEL_1)                        \
            SET_OC_MODE_CHANNEL_1(__TIM_HANDLE__, TIM_OCMODE_INACTIVE) \
        else if ((__TIM_CHANNEL__) == TIM_CHANNEL_2)                   \
            SET_OC_MODE_CHANNEL_2(__TIM_HANDLE__, TIM_OCMODE_INACTIVE) \
        else if ((__TIM_CHANNEL__) == TIM_CHANNEL_3)                   \
            SET_OC_MODE_CHANNEL_3(__TIM_HANDLE__, TIM_OCMODE_INACTIVE) \
        else if ((__TIM_CHANNEL__) == TIM_CHANNEL_4)                   \
            SET_OC_MODE_CHANNEL_4(__TIM_HANDLE__, TIM_OCMODE_INACTIVE) \
    } while (0);

// Set output turned into toggle mode in output compare mode when match
#define SET_OC_OUTPUT_TOGGLE(__TIM_HANDLE__, __TIM_CHANNEL__)      \
    do                                                               \
    {                                                                \
        if ((__TIM_CHANNEL__) == TIM_CHANNEL_1)                      \
            SET_OC_MODE_CHANNEL_1(__TIM_HANDLE__, TIM_OCMODE_TOGGLE) \
        else if ((__TIM_CHANNEL__) == TIM_CHANNEL_2)                 \
            SET_OC_MODE_CHANNEL_2(__TIM_HANDLE__, TIM_OCMODE_TOGGLE) \
        else if ((__TIM_CHANNEL__) == TIM_CHANNEL_3)                 \
            SET_OC_MODE_CHANNEL_3(__TIM_HANDLE__, TIM_OCMODE_TOGGLE) \
        else if ((__TIM_CHANNEL__) == TIM_CHANNEL_4)                 \
            SET_OC_MODE_CHANNEL_4(__TIM_HANDLE__, TIM_OCMODE_TOGGLE) \
    } while (0);

// get the bit position in general notification of data not available for specified axis
#define GET_DATA_NOT_AVAILABLE_BIT(__AXIS__)                                 \
    ((__AXIS__ == X_AXIS)   ? GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_X_AXIS \
     : (__AXIS__ == Y_AXIS) ? GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_Y_AXIS \
     : (__AXIS__ == Z_AXIS) ? GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_Z_AXIS \
                            : 0)

// generate timer's event
#define GENERATE_TIM_EVENT(__TIM_HANDLE__, __EVENT__) \
    ((__TIM_HANDLE__)->Instance->EGR = (__EVENT__))

/* exported functions */
void stepInit(void);
HAL_StatusTypeDef stepCalculatePulseData(uint32_t st_addr);
void stepWakeUp();
void stepGoIdle();
void stepEnablePulseCalculate();
void stepDisablePulseCalculate();
void stepNotifyContinuePulseCalculation();
void stepBlockAxis(uint8_t axis);
uint8_t stepIsPulseDataExhausted();
void stepUpdateCounterValue();

#endif // __STEP_H