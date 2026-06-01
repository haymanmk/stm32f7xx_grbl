#include <string.h>
#include "stm32f7xx_hal.h"
#include "stm32f7xx_timer_extension.h"
#include "FreeRTOS.h"
#include "task.h"
#include "grbl.h"

#undef X_AXIS
#undef Y_AXIS
#undef Z_AXIS
#undef N_AXIS

#include "stm32f7xx_grbl.h"

#define N_AXIS NUM_DIMENSIONS
#define PULSE_FREQ 100000  // Hz
#define HALF_PERIOD 0x7FFF // 0xFFFF / 2

/* extern variables */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

/**
 * Type Defines
 */
typedef struct TIM_DMA_Parameters
{
    TIM_HandleTypeDef *htim;
    uint8_t TIM_DMA_ID;
    uint8_t TIM_DMA_ID_CC;
    uint8_t TIM_CHANNEL;
    HAL_TIM_ActiveChannel TIM_ACTIVE_CHANNEL;
    uint32_t CCRx_Addr;
    uint32_t CompareEventID;
    uint8_t Step_Bit;
    uint8_t Dir_Bit;
    uint32_t TIM_FLAG_CCx; // capture/compare interrupt flag
    volatile uint32_t *CCMRx_Addr;
    uint32_t OCxM_Mask;
    uint8_t OCxM_Shift;
    uint32_t TIM_DIER_CCxDE;
    DMA_HandleTypeDef *hdma;
    volatile uint32_t *DirOutputPort;
} TIM_DMA_Parameters_t;

static inline void stepSetOCMode(const TIM_DMA_Parameters_t *p, uint32_t mode)
{
    *p->CCMRx_Addr = (*p->CCMRx_Addr & ~p->OCxM_Mask) | (mode << p->OCxM_Shift);
}

static inline void stepClearPendingDmaRequest(const TIM_DMA_Parameters_t *p)
{
    TIM_TypeDef *t = p->htim->Instance;
    t->DIER &= ~p->TIM_DIER_CCxDE;
    t->DIER |=  p->TIM_DIER_CCxDE;
}

// tcie_bit must be either DMA_IT_TC (enable transfer-complete IRQ) or 0 (disable).
// We clear-then-OR to flip TCIE explicitly each cycle, since the dominant axis can change between slots.
static inline void stepResumeDmaStream(DMA_HandleTypeDef *hdma, uint32_t buffer_addr, uint16_t length, uint32_t tcie_bit)
{
    DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;
    regs->IFCR = 0x3FU << hdma->StreamIndex;
    hdma->Instance->M0AR = buffer_addr;
    hdma->Instance->NDTR = length;
    hdma->Instance->CR  = (hdma->Instance->CR & ~DMA_IT_TC) | tcie_bit | DMA_SxCR_EN;
}

typedef struct
{
    doubleBufferArray_t data;
    uint16_t length;
} pulse_t;

/*
 * Type defines copied from stepper.c
 */
typedef uint16_t IO_TYPE;

// Stores the planner block Bresenham algorithm execution data for the segments in the segment
// buffer. Normally, this buffer is partially in-use, but, for the worst case scenario, it will
// never exceed the number of accessible stepper buffer segments (SEGMENT_BUFFER_SIZE-1).
// NOTE: This data is copied from the prepped planner blocks so that the planner blocks may be
// discarded when entirely consumed and completed by the segment buffer. Also, AMASS alters this
// data for its own use.
typedef struct
{
    uint32_t steps[N_AXIS];
    uint32_t step_event_count;
    IO_TYPE direction_bits;
#ifdef ENABLE_DUAL_AXIS
    uint8_t direction_bits_dual;
#endif
#ifdef VARIABLE_SPINDLE
    uint8_t is_pwm_rate_adjusted; // Tracks motions that require constant laser power/rate
#endif
#ifdef STM32F7XX_ARCH
    uint8_t realtime_output_pin_status; // Tracks the realtime output pin status for the block,
                                        // control output pin and axial motion at the same time.
#endif
} st_block_t;

// Primary stepper segment ring buffer. Contains small, short line segments for the stepper
// algorithm to execute, which are "checked-out" incrementally from the first block in the
// planner buffer. Once "checked-out", the steps in the segments buffer cannot be modified by
// the planner, where the remaining planner block steps still can.
typedef struct
{
    uint16_t n_step;          // Number of step events to be executed for this segment
    uint32_t cycles_per_tick; // Step distance traveled per ISR tick, aka step rate.
    uint8_t st_block_index;   // Stepper block data index. Uses this information to execute this segment.
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint8_t amass_level; // Indicates AMASS level for the ISR to execute this segment
#else
    uint8_t prescaler; // Without AMASS, a prescaler is required to adjust for slow timing.
#endif
#ifdef VARIABLE_SPINDLE
    uint8_t spindle_pwm;
#endif
} segment_t;

// Stepper ISR data struct. Contains the running data for the main stepper ISR.
typedef struct
{
    // Used by the bresenham line algorithm
    uint32_t counter_x, // Counter variables for the bresenham line tracer
        counter_y,
        counter_z;
#ifdef STEP_PULSE_DELAY
    uint8_t step_bits; // Stores out_bits output to complete the step pulse delay
#endif

    uint8_t execute_step;     // Flags step execution for each interrupt.
    uint32_t step_pulse_time; // Step pulse reset time after step rise
    IO_TYPE step_outbits;     // The next stepping-bits to be output
    IO_TYPE dir_outbits;
#ifdef ENABLE_DUAL_AXIS
    uint8_t step_outbits_dual;
    uint8_t dir_outbits_dual;
#endif
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint32_t steps[N_AXIS];
#endif

    uint16_t step_count;      // Steps remaining in line segment motion
    uint8_t exec_block_index; // Tracks the current st_block index. Change indicates new block.
    st_block_t *exec_block;   // Pointer to the block data for the segment being executed
    segment_t *exec_segment;  // Pointer to the segment being executed
} stepper_t;

typedef struct
{
    pulse_t pulse_data[NUM_DIMENSIONS];
    uint8_t motion_control_state;
    IO_TYPE dir_outbits;
    uint8_t realtime_output_pin_status;
    uint8_t tc_axis;             // axis whose DMA TC IRQ we'll watch (latest last-counter); UINT8_MAX = none
    uint32_t tc_last_counter;    // OFF-edge counter of tc_axis's last pulse in this slot
} pulse_block_t;

/**
 * Declare Variables
 */
// Ring buffer for each axis
static pulse_block_t pulseRingBuffer[RING_BUFFER_SIZE] = {0};

// ring buffer head and tail for each axis
volatile uint16_t pulseRingBufferHead = 0;
volatile uint16_t pulseRingBufferTail = 0;

// declare an array of timer and DMA parameters for each axis
static TIM_DMA_Parameters_t axisTimerDMAParams[NUM_DIMENSIONS];

// declare an array of general notification
volatile uint32_t generalNotification = (GENERAL_NOTIFICATION_GET_NEW_BUFFER | GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_ALL_AXES | GENERAL_NOTIFICATION_FIRST_TIME_START);

// current steppers state, which can be used to determine if a stepper shall be re-enabled from idle.
volatile uint8_t currentStepperState = 0; // bit 0: x axis active, bit 1: y axis active, bit 2: z axis active

// current counter value only for calculating the pulse data
volatile uint32_t currentCounterValue = MINIMUN_LOW_PULSE_WIDTH_TICKS;

volatile uint32_t pulseBlockAddress = 0;

volatile uint8_t stepBlockedAxes = 0; // used during homing cycle

volatile uint8_t OC_DMA_Started = 0; // flag to check if OC DMA is started, bit 0: x axis, bit 1: y axis, bit 2: z axis

// handle for step task
TaskHandle_t xHandleStepTask = NULL;

/**
 * Function Prototypes
 */
void stepTask(void *pvParameters);
void stepUpdateDMABuffer(uint32_t address);
static void TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef stepTimeOCStartDMA(TIM_HandleTypeDef *htim, uint32_t Channel, const uint32_t *pData, uint16_t Length);
uint16_t stepRingBufferGetNext();
uint16_t stepRingBufferIncrementHead();
uint16_t stepRingBufferGetTail();
void stepRingBufferIncrementTail();
uint32_t stepGetFreeDataAddress();
uint32_t stepGetAvailableDataAddress();

/* ============================= */
/* === Function Declarations === */
/* ============================= */
// initialization of step function
void stepInit(void)
{
    // disable irq
    __disable_irq();

    // stop master timer
    TIM_STOP_COUNTER(MASTER_TIM_HANDLE); // timer x axis

    // initialize axis parameters (populate precomputed CCMRx pointers, masks, shifts)
    INIT_TIM_DMA_PARAMETERS(axisTimerDMAParams, X_AXIS);
    INIT_TIM_DMA_PARAMETERS(axisTimerDMAParams, Y_AXIS);
    INIT_TIM_DMA_PARAMETERS(axisTimerDMAParams, Z_AXIS);

    // force output compare mode to inactive
    stepSetOCMode(&axisTimerDMAParams[X_AXIS], TIM_OCMODE_FORCED_INACTIVE);
    stepSetOCMode(&axisTimerDMAParams[Y_AXIS], TIM_OCMODE_FORCED_INACTIVE);
    stepSetOCMode(&axisTimerDMAParams[Z_AXIS], TIM_OCMODE_FORCED_INACTIVE);

    // clear DMA interrupt flag
    CLEAR_DMA_IT(X_AXIS_TIM_HANDLE.hdma[X_AXIS_PULSE_TIM_DMA_ID]);
    CLEAR_DMA_IT(Y_AXIS_TIM_HANDLE.hdma[Y_AXIS_PULSE_TIM_DMA_ID]);
    CLEAR_DMA_IT(Z_AXIS_TIM_HANDLE.hdma[Z_AXIS_PULSE_TIM_DMA_ID]);

    // clear timer capture/compare flag
    __HAL_TIM_CLEAR_FLAG(&X_AXIS_TIM_HANDLE, X_AXIS_TIM_FLAG_CCx);
    __HAL_TIM_CLEAR_FLAG(&Y_AXIS_TIM_HANDLE, Y_AXIS_TIM_FLAG_CCx);
    __HAL_TIM_CLEAR_FLAG(&Z_AXIS_TIM_HANDLE, Z_AXIS_TIM_FLAG_CCx);

    // suspend DMA stream
    SUSPEND_DMA_STREAM(X_AXIS_TIM_HANDLE.hdma[X_AXIS_PULSE_TIM_DMA_ID]);
    SUSPEND_DMA_STREAM(Y_AXIS_TIM_HANDLE.hdma[Y_AXIS_PULSE_TIM_DMA_ID]);
    SUSPEND_DMA_STREAM(Z_AXIS_TIM_HANDLE.hdma[Z_AXIS_PULSE_TIM_DMA_ID]);

    // stop DMA OC mode
    HAL_TIM_OC_Stop_DMA(&X_AXIS_TIM_HANDLE, X_AXIS_PULSE_TIM_CHANNEL);
    HAL_TIM_OC_Stop_DMA(&Y_AXIS_TIM_HANDLE, Y_AXIS_PULSE_TIM_CHANNEL);
    HAL_TIM_OC_Stop_DMA(&Z_AXIS_TIM_HANDLE, Z_AXIS_PULSE_TIM_CHANNEL);

    // write the largest value, 0xFFFFFFFF, in capture/compare register
    // to prevent the CCxIF flag from being set when the counter is set to 0.
    __HAL_TIM_SET_COMPARE(&X_AXIS_TIM_HANDLE, X_AXIS_PULSE_TIM_CHANNEL, 0xFFFFFFFF);
    __HAL_TIM_SET_COMPARE(&Y_AXIS_TIM_HANDLE, Y_AXIS_PULSE_TIM_CHANNEL, 0xFFFFFFFF);
    __HAL_TIM_SET_COMPARE(&Z_AXIS_TIM_HANDLE, Z_AXIS_PULSE_TIM_CHANNEL, 0xFFFFFFFF);

    // initialize ring buffer head and tail
    pulseRingBufferHead = 0;
    pulseRingBufferTail = 0;

    // clear the motion control state at the head of the ring buffer
    pulseRingBuffer[pulseRingBufferHead].motion_control_state = 0;

    // initialize general notification
    generalNotification |= (GENERAL_NOTIFICATION_GET_NEW_BUFFER | GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_ALL_AXES | GENERAL_NOTIFICATION_FIRST_TIME_START);

    // initialize current stepper state
    currentStepperState = 0;

    // initialize current counter value
    currentCounterValue = MINIMUN_LOW_PULSE_WIDTH_TICKS;

    // initialize pulse block address
    pulseBlockAddress = 0;

    // reset blocked axes
    stepBlockedAxes = 0;

    // reset OC DMA started flag
    OC_DMA_Started = 0;

    // enable irq
    __enable_irq();

    // ===> debug message
    // vLoggingPrintf("Step Init\n");

    // check if task is created
    if (xHandleStepTask != NULL)
    {
        // delete the task
        vTaskDelete(xHandleStepTask);

        // set the handle to NULL
        xHandleStepTask = NULL;
    }

    // create tasks
    xTaskCreate(stepTask, "StepTask", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, &xHandleStepTask);
}

// handler for step task
void stepTask(void *pvParameters)
{
    // set PD3 to high to enable level shifter
    // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

    // set general notification to data not available to start the process
    generalNotification |= (GENERAL_NOTIFICATION_GET_NEW_BUFFER | GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_ALL_AXES | GENERAL_NOTIFICATION_FIRST_TIME_START);

    // wait notification to calculate pulse data
    vLoggingPrintf("WaitCalPul\n");
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    vLoggingPrintf("CalPul\n");

    // Infinite loop
    for (;;)
    {
        // prepare pulse data
        if ((generalNotification & GENERAL_NOTIFICATION_CALCULATE_PULSE))
        {
            // calculate pulse data
            stepper_pulse_generation_isr();
        }
        else
        {
            pulse_block_t *pulseBlock = &(pulseRingBuffer[pulseRingBufferHead]);

            // check if the pulse block is not empty
            // in case appending empty pulse block to DMA
            if (pulseBlock->motion_control_state)
            {
                // increment head
                while (stepRingBufferIncrementHead() == UINT16_MAX)
                {
                    // No more buffer available to store pulse data
                    vTaskDelay(1);
                }
                // ring buffer head has been incremented
                // clear motion control state
                pulseRingBuffer[pulseRingBufferHead].motion_control_state = 0;

                vLoggingPrintf("Get new buffer\n");
                // inform to get new free buffer
                generalNotification |= GENERAL_NOTIFICATION_GET_NEW_BUFFER;
            }
            else
            {
                vLoggingPrintf("No data available\n");
                // wait notification to calculate pulse data
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                vLoggingPrintf("leave no data\n");
            }
        }

        // check if there is a notification to resume DMA stream asking from the timer interrupt
        if (!(generalNotification & GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_ALL_AXES))
        {
            continue;
        }

        // check if there is any buffer available
        if (stepRingBufferGetTail() == UINT16_MAX)
        {
            continue;
        }

        // check if it is the "First Time" to start the process
        if (generalNotification & GENERAL_NOTIFICATION_FIRST_TIME_START)
        {
            // get available data address
            pulseBlockAddress = stepGetAvailableDataAddress();
            pulse_block_t *pulseBlock = (pulse_block_t *)pulseBlockAddress;

            // set up each axis
            for (int8_t i = NUM_DIMENSIONS - 1; i >= 0; i--)
            {
                // get pulse data
                pulse_t *pulseData = &(pulseBlock->pulse_data[i]);

                // get timer and DMA parameters of this axis
                TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[i];

                // reset timer
                __HAL_TIM_SetCounter(timDMAParamsPulse->htim, 0);

                uint8_t dirOutputBit = timDMAParamsPulse->Dir_Bit;
                volatile uint32_t *pDirOutputPort = timDMAParamsPulse->DirOutputPort;

                // set the direction output bit according to the dir_outbit
                *pDirOutputPort = (*pDirOutputPort & ~(1 << dirOutputBit)) | (pulseBlock->dir_outbits & (1 << dirOutputBit));

                // determine if this axis is active or not by checking motion_control_state coming with the pulse data
                if (pulseBlock->motion_control_state & (1 << i))
                {
                    // set output compare mode
                    stepSetOCMode(timDMAParamsPulse, TIM_OCMODE_TOGGLE);

                    // start timer output mode with DMA stream
                    HAL_StatusTypeDef ocStatus = stepTimeOCStartDMA(timDMAParamsPulse->htim, timDMAParamsPulse->TIM_CHANNEL, (uint32_t *)pulseData, pulseData->length);
                    if (ocStatus != HAL_OK)
                    {
                        vLoggingPrintf("OCStartDMA fail (first-time) axis=%d status=%d\n", i, ocStatus);
                    }

                    // HAL_DMA_Start_IT enabled TC for all streams; clear it on non-dominant axes so only tc_axis fires
                    if (i != pulseBlock->tc_axis)
                    {
                        timDMAParamsPulse->hdma->Instance->CR &= ~DMA_IT_TC;
                    }

                    // generate compare event
                    HAL_TIM_GenerateEvent(timDMAParamsPulse->htim, timDMAParamsPulse->CompareEventID);

                    // set OC DMA started flag
                    OC_DMA_Started |= (1 << i);
                }
                else // this axis is not active
                {
                    // force output pin to low in output compare mode
                    stepSetOCMode(timDMAParamsPulse, TIM_OCMODE_INACTIVE);
                }

                // check if it is the master timer
                if (timDMAParamsPulse->htim != &MASTER_TIM_HANDLE)
                {
                    // enable the timer
                    __HAL_TIM_ENABLE(timDMAParamsPulse->htim);
                }
            }

            // set realtime output pin status
            UTILS_WRITE_GPIO(REALTIME_OUTPUT_GPIO_GROUP, REALTIME_OUTPUT_PIN, (pulseBlock->realtime_output_pin_status ^ 0x01));

            // update current motion control state by the motion_control_state coming with the pulse data
            currentStepperState = pulseBlock->motion_control_state;

            // set all DMA updated flag
            generalNotification |= GENERAL_NOTIFICATION_ALL_AXES_DMA_UPDATED;

            // enable the master timer
            __HAL_TIM_ENABLE(&MASTER_TIM_HANDLE);

            // clear the flag
            generalNotification &= ~GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_ALL_AXES;
            generalNotification &= ~GENERAL_NOTIFICATION_FIRST_TIME_START;
        }
        // NOT the "First Time" to start the process, but out of data asking from the timer interrupt
        else
        {
            // get newly-available buffer
            pulseBlockAddress = stepGetAvailableDataAddress();

            if (pulseBlockAddress == 0)
            {
                continue;
            }

            stepUpdateDMABuffer(pulseBlockAddress);

            // master timer was never stopped in the new design; stepUpdateDMABuffer's
            // overshoot guard handles CNT alignment with the new buffer.
            generalNotification &= ~GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_ALL_AXES;
        }
    }
}

HAL_StatusTypeDef stepCalculatePulseData(uint32_t st_addr)
{
    UTILS_WRITE_GPIO(DEBUG_2_GPIO_Port, DEBUG_2_Pin, 1);
    static uint8_t getNewBuffer = (1 << NUM_DIMENSIONS) - 1; // bit 0: x axis, bit 1: y axis, bit 2: z axis
    static pulse_block_t *pulseBlock = {0};
    static pulse_t *pulse;
    static IO_TYPE dirOutputBits = 0;
    static uint32_t cycles_per_tick = 0;
    // static uint8_t amass_level = 0;

    stepper_t *st = (stepper_t *)st_addr;

    if (st->exec_segment)
    {
        // update variables
        cycles_per_tick = st->exec_segment->cycles_per_tick;
        // amass_level = st->exec_segment->amass_level;

        if ((st->exec_segment->cycles_per_tick == 0) || (st->exec_segment->cycles_per_tick == 0xffffffff))
        {
            // debug message
            vLoggingPrintf("errParam\n");

            return HAL_ERROR;
        }
    }

    if (!(st->step_pulse_time))
    {
        // debug message
        vLoggingPrintf("errZero\n");

        return HAL_ERROR;
    }

    // determine if a new buffer is needed
    if (getNewBuffer || (dirOutputBits != st->dir_outbits) || (generalNotification & GENERAL_NOTIFICATION_GET_NEW_BUFFER))
    {
        // check if current buffer is not empty
        pulseBlock = &(pulseRingBuffer[pulseRingBufferHead]);

        if (pulseBlock->motion_control_state)
        {
            // increment head
            if (stepRingBufferIncrementHead() == UINT16_MAX)
            {
                // No more buffer available to store pulse data
                return HAL_BUSY;
            }
            // ring buffer head has been incremented

            // get the new buffer
            pulseBlock = &(pulseRingBuffer[pulseRingBufferHead]);
        }

        // reset motion control state
        pulseBlock->motion_control_state = 0;

        // reset TC-axis selection for the new slot
        pulseBlock->tc_axis = UINT8_MAX;

        // set direction state
        pulseBlock->dir_outbits = st->dir_outbits;

        // update direction output bits
        dirOutputBits = st->dir_outbits;

        // loop through all axes
        for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
        {
            // get pulse data
            pulse = &(pulseBlock->pulse_data[i]);

            // clear pulse length
            pulse->length = 0;
        }

        // clear the flag
        generalNotification &= ~GENERAL_NOTIFICATION_GET_NEW_BUFFER;
        getNewBuffer = 0;
    }

    // loop through all axes
    for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
    {
        // get timer and DMA parameters of this axis
        TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[i];
        uint8_t step_bit = timDMAParamsPulse->Step_Bit;

        // get pulse data
        pulse = &(pulseBlock->pulse_data[i]);

        // determine if a pair of pulse data should be added in this axis or not by Bresenham line algorithm.
        // a pair of pulse data consists of a pulse data for the on state and a pulse data for the off state.
        uint16_t buf_length = pulse->length;

        if (st->step_outbits & (1 << step_bit))
        {
            uint32_t off_edge_counter = currentCounterValue + st->step_pulse_time;

            // set this axis to ACTIVE state
            pulseBlock->motion_control_state |= (1 << i);

            // add pulse data
            pulse->data[buf_length++] = currentCounterValue;   // ON edge
            pulse->data[buf_length++] = off_edge_counter;      // OFF edge

            // update the length of available data
            pulse->length = buf_length;

            // pick this axis as tc_axis if its last counter is later than the current best.
            // Signed-diff compare handles 32-bit wraparound; sentinel UINT8_MAX is "no candidate yet".
            if (pulseBlock->tc_axis == UINT8_MAX ||
                (int32_t)(off_edge_counter - pulseBlock->tc_last_counter) > 0)
            {
                pulseBlock->tc_axis = i;
                pulseBlock->tc_last_counter = off_edge_counter;
            }

            // check if the buffer is full
            if (buf_length >= DOUBLE_BUFFER_SIZE)
            {
                // set the flag to get new buffer
                getNewBuffer |= (1 << i);
            }
        }
    }

    // if there are any movements, update the motion control state reference by other associates.
    if (st->step_outbits)
    {
        // set realtime output pin status
        pulseBlock->realtime_output_pin_status = st->exec_block->realtime_output_pin_status;
    }

    // update variables
    currentCounterValue += cycles_per_tick; // * (amass_level + 1);

    UTILS_WRITE_GPIO(DEBUG_2_GPIO_Port, DEBUG_2_Pin, 0);
    return HAL_OK;
}

void stepUpdateDMABuffer(uint32_t address)
{
    pulse_block_t *pulseBlock = (pulse_block_t *)address;
    uint8_t upcomingAxesActiveState = pulseBlock->motion_control_state & (~stepBlockedAxes); // get upcoming axes active state
    // check if there is any bit that is set to 0 in currentStepperState
    // and the corresponding bit is set to 1 in upcomingAxesActiveState.
    // make these axes which shall be re-enabled from idle
    // uint8_t axisToBeReEnabled = (currentStepperState ^ upcomingAxesActiveState) & upcomingAxesActiveState;
    // uint8_t axisToContinue = currentStepperState & upcomingAxesActiveState;

    /**
     * Check if there is any axis that shall be re-enabled from idle, resume DMA stream, or turn into idle state
     */
    // manual update those axes that shall be re-enabled from idle and will stay in idle state in the upcoming cycle
    // to avoid update the same axis twice,
    // loop through all axes
    for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
    {
        // pulse data
        pulse_t *pulse = &(pulseBlock->pulse_data[i]);
        uint8_t dirOutputBit = axisTimerDMAParams[i].Dir_Bit;
        volatile uint32_t *pDirOutputPort = axisTimerDMAParams[i].DirOutputPort;

        // check if there is any axis shall be re-enabled from idle or resume DMA stream to continue on transferring data
        if (upcomingAxesActiveState & (1 << i))
        {

            // set the axis to active
            currentStepperState |= (1 << i);

            // set the direction output bit according to the dir_outbit
            *pDirOutputPort = (*pDirOutputPort & ~(1 << dirOutputBit)) | (pulseBlock->dir_outbits & (1 << dirOutputBit));

            // force output turned into toggle mode
            stepSetOCMode(&axisTimerDMAParams[i], TIM_OCMODE_TOGGLE);

            // clear the pending DMA request in timer
            // stepClearPendingDmaRequest(&axisTimerDMAParams[i]);

            // only the tc_axis gets TC IRQ enabled; the rest run silently and auto-disable on NDTR=0
            uint32_t tcie_bit = (i == pulseBlock->tc_axis) ? DMA_IT_TC : 0U;

            // resume DMA stream with updated buffer and length
            if (OC_DMA_Started & (1 << i)) // OC DMA might be already started
            {
                UTILS_WRITE_GPIO(DEBUG_4_GPIO_Port, DEBUG_4_Pin, 1);
                stepResumeDmaStream(axisTimerDMAParams[i].hdma, (uint32_t)pulse, pulse->length, tcie_bit);
                UTILS_WRITE_GPIO(DEBUG_4_GPIO_Port, DEBUG_4_Pin, 0);
            }
            else // OC DMA is not started yet
            {
                UTILS_WRITE_GPIO(DEBUG_3_GPIO_Port, DEBUG_1_Pin, 1);

                // Clear stale HAL BUSY/LOCKED state before restarting. The DMA TC IRQ is what
                // normally completes the HAL handshake: TIM_DMADelayPulseCplt resets the TIM
                // channel state, and HAL_DMA_IRQHandler resets hdma->State to READY and calls
                // __HAL_UNLOCK(hdma). But this design disables TC on the non-dominant axes, so
                // none of that runs for them — the channel state stays BUSY, hdma->State stays
                // BUSY, and (because HAL_DMA_Start_IT only unlocks on its busy/error path, never
                // on success) hdma->Lock stays HAL_LOCKED. On the next restart, HAL_DMA_Start_IT
                // hits __HAL_LOCK FIRST and returns HAL_BUSY on the stuck lock, before it ever
                // checks State — so the stream/CCxDE is never configured (NDTR stuck at 0, CCR
                // frozen). The stream is genuinely idle here (NDTR=0 => EN auto-cleared in
                // DMA_NORMAL), so forcing channel state + hdma->State READY and clearing the lock
                // is safe and lets the (re)start proceed.
                TIM_CHANNEL_STATE_SET(axisTimerDMAParams[i].htim, axisTimerDMAParams[i].TIM_CHANNEL, HAL_TIM_CHANNEL_STATE_READY);
                axisTimerDMAParams[i].hdma->State = HAL_DMA_STATE_READY;
                __HAL_UNLOCK(axisTimerDMAParams[i].hdma);

                HAL_StatusTypeDef ocStatus = stepTimeOCStartDMA(axisTimerDMAParams[i].htim, axisTimerDMAParams[i].TIM_CHANNEL, (uint32_t *)pulse, pulse->length);
                if (ocStatus != HAL_OK)
                {
                    vLoggingPrintf("OCStartDMA fail (restart) axis=%d status=%d\n", i, ocStatus);
                }

                // HAL_DMA_Start_IT enabled TC; clear it again if this isn't the tc_axis
                if (tcie_bit == 0U)
                {
                    axisTimerDMAParams[i].hdma->Instance->CR &= ~DMA_IT_TC;
                }

                // Prime the first DMA transfer. This axis's CCR still holds its idle value
                // (0xFFFFFFFF from stepInit), so no natural compare match would ever load
                // data[0] — generate one compare event to kick the first transfer.
                // NOTE: only needed here. The resume branch above gets its priming match for
                // free from the leftover CCR (old last OFF-edge), which also completes the
                // previous pulse; forcing an event there would re-introduce pulse elongation.
                GENERATE_TIM_EVENT(axisTimerDMAParams[i].htim, axisTimerDMAParams[i].CompareEventID);

                // set OC DMA started flag
                OC_DMA_Started |= (1 << i);
                UTILS_WRITE_GPIO(DEBUG_3_GPIO_Port, DEBUG_1_Pin, 0);
            }
        }
        else // idle mode(present) -> idle mode(upcoming) or active mode(present) -> idle mode(upcoming)
        {
            // Force this axis's OC output LOW. Without this, an axis left in TOGGLE mode from a
            // previous motion (with a stale CCR holding its old last value) would spuriously
            // toggle the next time CNT catches up to that old CCR.
            stepSetOCMode(&axisTimerDMAParams[i], TIM_OCMODE_FORCED_INACTIVE);

            // set the axis to idle
            currentStepperState &= ~(1 << i);
        }
    }

    // set real-time output pin status
    UTILS_WRITE_GPIO(REALTIME_OUTPUT_GPIO_GROUP, REALTIME_OUTPUT_PIN, (pulseBlock->realtime_output_pin_status ^ 0x01));

    // Overshoot guard: master timer kept running through the load, so CNT may have passed
    // some axes' first ON targets. For each active axis, signed-diff compare data[0] vs CNT;
    // if any axis's first target was passed, rewind both timers' CNT to just before the
    // earliest-passed target so the missed match fires on the next tick.
    if (upcomingAxesActiveState)
    {
        uint32_t cnt = MASTER_TIM_HANDLE.Instance->CNT;
        int32_t earliest_delta = 0;
        uint32_t earliest_target = 0;
        uint8_t any_past = 0;

        for (uint8_t i = 0; i < NUM_DIMENSIONS; i++)
        {
            if (!(upcomingAxesActiveState & (1 << i))) continue;
            uint32_t target = pulseBlock->pulse_data[i].data[0];
            int32_t delta = (int32_t)(target - cnt);
            if (delta <= 0 && (!any_past || delta < earliest_delta))
            {
                earliest_delta = delta;
                earliest_target = target;
                any_past = 1;
            }
        }

        if (any_past)
        {
            uint32_t new_cnt = earliest_target - 1U;  // next tick matches earliest_target
            MASTER_TIM_HANDLE.Instance->CNT = new_cnt;
            Z_AXIS_TIM_HANDLE.Instance->CNT = new_cnt;  // keep slave-gated TIM5 aligned
        }

        // Ensure the master timer is running. In the hot ISR path this is a no-op (CEN already 1).
        // After stepGoIdle stopped the timer for motion-end, this is what restarts it for the next motion.
        TIM_START_COUNTER(MASTER_TIM_HANDLE);
    }
}

/**
 * @brief In case currentCounterValue is far behind the current counter value of the master timer,
 *        update the currentCounterValue to a value of the current counter value of the master timer
 *        plus a minimum low pulse width. This function is called after system turns into idle from feed hold.
 *        Due to the feed hold request, the currentCounterValue might be far behind the current counter value.
 *        The step data is calculated in advance and pushed in a ring buffer
 *        before it is transferred to the DMA buffer and wait for DMA to consume it, so the currentCounterValue
 *        is far behind the current counter value of the master timer. When user requests for feed hold,
 *        currentCounterValue is still far behind the current counter value. If user did not update this value,
 *        the next step data will be calculated upon this value, and timer will take a long time to reach there.
 *        The whole system will be delayed. To avoid this, this function shall be called to update the currentCounterValue
 *        after the feed hold request is committed.
 */
void stepUpdateCounterValue()
{
    // get counter value from the master timer and update the current counter value
    currentCounterValue =
        __HAL_TIM_GET_COUNTER(&MASTER_TIM_HANDLE) + MINIMUN_LOW_PULSE_WIDTH_TICKS;
}

/* =========================================================== */
/*                   DMA Transfer Complete ISR                 */
/* =========================================================== */
/**
 * @brief  TIM DMA Pulse complete callback.
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    // set PD6 to high ===> signal the start of ISR
    UTILS_WRITE_GPIO(DEBUG_3_GPIO_Port, DEBUG_3_Pin, GPIO_PIN_SET);

    // avoid doing anything if the step agent has been reset from software.
    if (generalNotification & GENERAL_NOTIFICATION_FIRST_TIME_START)
    {
        goto exit;
    }

    // Master timer keeps running through the ISR — the last OFF edge fires naturally between TC
    // and end of this handler, so no pulse elongation. stepUpdateDMABuffer's overshoot guard
    // rewinds CNT if the new buffer's first target was passed during the load.
    pulseBlockAddress = stepGetAvailableDataAddress();

    if (pulseBlockAddress != 0)
    {
        stepUpdateDMABuffer(pulseBlockAddress);
    }
    else
    {
        // No next buffer queued. Distinguish true motion-end from a transient ring-buffer
        // underrun: at true end the calc side already ran out of segments and cleared
        // CALCULATE_PULSE (via stepDisablePulseCalculate from stepper_pulse_generation_isr);
        // during a mid-motion underrun it is still set and more data is coming, so the master
        // timer must keep running for the overshoot-guard recovery. Only stop the free-running
        // master timer at true end — otherwise it keeps counting with the OC channels left in
        // TOGGLE mode holding stale CCRs, and emits a spurious pulse when CNT eventually wraps
        // back onto an old compare value. stepGoIdle also sets FORCE_STOP so the next motion's
        // stepWakeUp realigns currentCounterValue to CNT (avoids the startup delay).
        // if (!(generalNotification & GENERAL_NOTIFICATION_CALCULATE_PULSE))
        // {
        //     stepGoIdle();
        // }
        stepGoIdle();
        // clear the pending DMA request in timer
        stepClearPendingDmaRequest(&axisTimerDMAParams[X_AXIS]);
        stepClearPendingDmaRequest(&axisTimerDMAParams[Y_AXIS]);
        stepClearPendingDmaRequest(&axisTimerDMAParams[Z_AXIS]);

        // notify main task to update pulse data
        axis_t axis = GET_AXIS_FROM_TIM_HANDLE(htim);
        generalNotification |= GET_DATA_NOT_AVAILABLE_BIT(axis);
    }

exit:
    // set PD6 to low ===> signal the end of ISR
    UTILS_WRITE_GPIO(DEBUG_3_GPIO_Port, DEBUG_3_Pin, GPIO_PIN_RESET);
}

/* =========================================================== */
/*       Assistant Functions for Ring buffer management        */
/* =========================================================== */
/**
 * Ring Buffer Manipulation Functions
 */
uint16_t stepRingBufferGetNext()
{
    /**
     * In order to avoid the data updating overwriting the data used by DMA,
     * the head and tail pointers are separated by a gap of N buffers.
     * N is defined by GAP_HEAD_TAIL. Howerver, instead of tracking the gap
     * to determine if there is a next free index, here we simply check if
     * the length of the populated data of ring buffer is less than
     * RING_BUFFER_SIZE - GAP_HEAD_TAIL.
     *
     * i.e.
     * | * | * | * | (Head)>> | 0 | 0 | >>(Tail) | * | * | * |
     * >-------- Body ------->|  GAP  |>------- Body -------->
     */
    uint16_t bodyLength = (pulseRingBufferHead - pulseRingBufferTail + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;

    if (bodyLength < RING_BUFFER_SIZE - GAP_HEAD_TAIL)
    {
        return (pulseRingBufferHead + 1) % RING_BUFFER_SIZE;
    }
    else
        return UINT16_MAX;
}

uint16_t stepRingBufferIncrementHead()
{
    uint16_t nextIndex = stepRingBufferGetNext();

    if (nextIndex != UINT16_MAX)
    {
        pulseRingBufferHead = nextIndex;
    }

    return nextIndex;
}

uint16_t stepRingBufferGetTail()
{
    if (pulseRingBufferTail != pulseRingBufferHead)
    {
        return pulseRingBufferTail;
    }
    else
        return UINT16_MAX;
}

void stepRingBufferIncrementTail()
{
    if (pulseRingBufferTail != pulseRingBufferHead)
    {
        pulseRingBufferTail = (pulseRingBufferTail + 1) % (RING_BUFFER_SIZE);
    }
}

/**
 * Get the free data address
 * @param axis Axis to get the free data address
 * @return Free data address; 0 if no data available
 */
uint32_t stepGetFreeDataAddress()
{
    if (stepRingBufferGetNext() == UINT16_MAX)
        return 0;

    return (uint32_t)(&pulseRingBuffer[pulseRingBufferHead]);
}

/**
 * Get the available data address
 * @param axis Axis to get the available data address
 * @return Available data address; 0 if no data available
 */
uint32_t stepGetAvailableDataAddress()
{
    uint16_t tail = stepRingBufferGetTail();

    if (tail == UINT16_MAX)
        return 0;

    stepRingBufferIncrementTail();

    return (uint32_t)(&pulseRingBuffer[tail]);
}

/* ===================================================================== */
/*                      Everything about Stepper                         */
/* ===================================================================== */

// Motion-end / idle-restart contract (needs hardware verification):
// grbl's stepper_pulse_generation_isr calls stepDisablePulseCalculate at end-of-motion (it runs on
// the calc side, ahead of the DMA, so it must NOT stop the timer there — the DMA is still draining).
// The actual master-timer stop happens later, on the calc-side-exhausted DMA TC in
// HAL_TIM_PWM_PulseFinishedCallback, which calls stepGoIdle once CALCULATE_PULSE is clear. That:
//   - forces OC outputs LOW + stops the master timer (no spurious wrap pulses, no startup drift),
//   - sets FORCE_STOP so stepWakeUp realigns currentCounterValue to CNT on the next motion,
//   - clears OC_DMA_Started so the next stepUpdateDMABuffer re-primes via the fresh-start path.
// Verify on hardware: (1) no spurious pulse during long idles; (2) first pulse of the next motion
// fires promptly (no realign delay), including the post-idle motion whose active axis differs.
void stepWakeUp()
{
    // check if pulse calculation is disabled
    if (generalNotification & GENERAL_NOTIFICATION_FORCE_STOP)
    {
        // Coming out of idle: hardware CNT is frozen wherever stepGoIdle stopped it, but the
        // software pulse-train cursor (currentCounterValue) kept advancing per-iteration during
        // the previous motion. Realign the cursor to CNT so the next motion's data[0] sits just
        // a few ticks ahead of CNT.
        stepUpdateCounterValue();

        // clear Force stop flag
        generalNotification &= ~GENERAL_NOTIFICATION_FORCE_STOP;
    }
}

void stepGoIdle()
{
    // Force all axes' OC outputs LOW before stopping the timer. In the new always-running design,
    // the counter could be mid-pulse (between an ON-edge match and the OFF-edge match), so stopping
    // it alone would freeze the output HIGH and invert the next motion's first toggle.
    stepSetOCMode(&axisTimerDMAParams[X_AXIS], TIM_OCMODE_FORCED_INACTIVE);
    stepSetOCMode(&axisTimerDMAParams[Y_AXIS], TIM_OCMODE_FORCED_INACTIVE);
    stepSetOCMode(&axisTimerDMAParams[Z_AXIS], TIM_OCMODE_FORCED_INACTIVE);

    // stop counter
    TIM_STOP_COUNTER(MASTER_TIM_HANDLE); // timer x axis

    // set Force stop flag
    generalNotification |= GENERAL_NOTIFICATION_FORCE_STOP;

    // Force the next motion through the fresh-start priming path. While the master timer was
    // free-running, the resume branch in stepUpdateDMABuffer could rely on a leftover-CCR compare
    // match to prime the first DMA transfer. After a full stop that match no longer occurs (CNT is
    // frozen at/past the old CCR), so clear OC_DMA_Started: the next stepUpdateDMABuffer takes the
    // "not started" branch and issues an explicit priming compare event. Safe here because the OC
    // outputs were just forced LOW, so there is no pending pulse to elongate.
    OC_DMA_Started = 0;
}

/**
 * @brief  Enable pulse calculation
 */
void stepEnablePulseCalculate()
{
    // set general notification
    generalNotification |= GENERAL_NOTIFICATION_CALCULATE_PULSE;

    stepNotifyContinuePulseCalculation();
}

/**
 * @brief  Disable pulse calculation
 */
void stepDisablePulseCalculate()
{
    // clear general notification
    generalNotification &= ~GENERAL_NOTIFICATION_CALCULATE_PULSE;
}

void stepNotifyContinuePulseCalculation()
{
    xTaskNotifyGive(xHandleStepTask);
}

void stepBlockAxis(uint8_t axis)
{
    // get timer and DMA parameters of this axis
    TIM_DMA_Parameters_t *timDMAParamsPulse = &axisTimerDMAParams[axis];

    // set the axis to idle
    stepBlockedAxes |= (1 << axis);

    // force output pin to low in output compare mode
    stepSetOCMode(timDMAParamsPulse, TIM_OCMODE_FORCED_INACTIVE);
}

uint8_t stepIsPulseDataExhausted()
{
    return ((generalNotification & GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE_ALL_AXES) > 0);
    // return (pulseRingBufferHead == pulseRingBufferTail);
}

/* ===================================================================== */
/*                      Everything about Timer                           */
/* ===================================================================== */

/**
 * @brief  Starts the TIM Output Compare signal generation in DMA mode.
 * @param  htim TIM Output Compare handle
 * @param  Channel TIM Channel to be enabled
 *          This parameter can be one of the following values:
 *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
 *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
 *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
 *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
 * @param  pData The source Buffer address.
 * @param  Length The length of data to be transferred from memory to TIM peripheral
 * @retval HAL status
 */
HAL_StatusTypeDef stepTimeOCStartDMA(TIM_HandleTypeDef *htim, uint32_t Channel, const uint32_t *pData,
                                     uint16_t Length)
{
    HAL_StatusTypeDef ret = HAL_OK;
    HAL_StatusTypeDef status = HAL_OK;
    //   uint32_t tmpsmcr;

    /* Check the parameters */
    assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));

    /* Set the TIM channel state */
    if (TIM_CHANNEL_STATE_GET(htim, Channel) == HAL_TIM_CHANNEL_STATE_BUSY)
    {
        return HAL_BUSY;
    }
    else if (TIM_CHANNEL_STATE_GET(htim, Channel) == HAL_TIM_CHANNEL_STATE_READY)
    {
        if ((pData == NULL) || (Length == 0U))
        {
            return HAL_ERROR;
        }
        else
        {
            TIM_CHANNEL_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);
        }
    }
    else
    {
        return HAL_ERROR;
    }

    switch (Channel)
    {
    case TIM_CHANNEL_1:
    {
        /* Set the DMA compare callbacks */
        htim->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = TIM_DMADelayPulseCplt;
        htim->hdma[TIM_DMA_ID_CC1]->XferHalfCpltCallback = TIM_DMADelayPulseHalfCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC1]->XferErrorCallback = TIM_DMAError;

        /* Enable the DMA stream */
        if ((ret=HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC1], (uint32_t)pData, (uint32_t)&htim->Instance->CCR1,
                             Length)) != HAL_OK)
        {
            vLoggingPrintf("HAL_DMA_Start_IT fail for CC1: status=%d\n", ret);
            /* Return error status */
            return HAL_ERROR;
        }

        /* Enable the TIM Capture/Compare 1 DMA request */
        __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC1);
        break;
    }

    case TIM_CHANNEL_2:
    {
        /* Set the DMA compare callbacks */
        htim->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = TIM_DMADelayPulseCplt;
        htim->hdma[TIM_DMA_ID_CC2]->XferHalfCpltCallback = TIM_DMADelayPulseHalfCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC2]->XferErrorCallback = TIM_DMAError;

        /* Enable the DMA stream */
        if ((ret=HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC2], (uint32_t)pData, (uint32_t)&htim->Instance->CCR2,
                             Length)) != HAL_OK)
        {
            vLoggingPrintf("HAL_DMA_Start_IT fail for CC2: status=%d\n", ret);
            /* Return error status */
            return HAL_ERROR;
        }

        /* Enable the TIM Capture/Compare 2 DMA request */
        __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC2);
        break;
    }

    case TIM_CHANNEL_3:
    {
        /* Set the DMA compare callbacks */
        htim->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = TIM_DMADelayPulseCplt;
        htim->hdma[TIM_DMA_ID_CC3]->XferHalfCpltCallback = TIM_DMADelayPulseHalfCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC3]->XferErrorCallback = TIM_DMAError;

        /* Enable the DMA stream */
        if ((ret=HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC3], (uint32_t)pData, (uint32_t)&htim->Instance->CCR3,
                             Length)) != HAL_OK)
        {
            vLoggingPrintf("HAL_DMA_Start_IT fail for CC3: status=%d\n", ret);
            /* Return error status */
            return HAL_ERROR;
        }
        /* Enable the TIM Capture/Compare 3 DMA request */
        __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC3);
        break;
    }

    case TIM_CHANNEL_4:
    {
        /* Set the DMA compare callbacks */
        htim->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = TIM_DMADelayPulseCplt;
        htim->hdma[TIM_DMA_ID_CC4]->XferHalfCpltCallback = TIM_DMADelayPulseHalfCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC4]->XferErrorCallback = TIM_DMAError;

        /* Enable the DMA stream */
        if ((ret=HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC4], (uint32_t)pData, (uint32_t)&htim->Instance->CCR4,
                             Length)) != HAL_OK)
        {
            vLoggingPrintf("HAL_DMA_Start_IT fail for CC4: status=%d\n", ret);
            /* Return error status */
            return HAL_ERROR;
        }
        /* Enable the TIM Capture/Compare 4 DMA request */
        __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC4);
        break;
    }

    default:
        status = HAL_ERROR;
        break;
    }

    if (status == HAL_OK)
    {
        /* Enable the Output compare channel */
        TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_ENABLE);

        if (IS_TIM_BREAK_INSTANCE(htim->Instance) != RESET)
        {
            /* Enable the main output */
            __HAL_TIM_MOE_ENABLE(htim);
        }
    }

    /* Return function status */
    return status;
}

/**
 * @brief  TIM DMA Delay Pulse complete callback.
 * @param  hdma pointer to DMA handle.
 * @retval None
 */
static void TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    if (hdma == htim->hdma[TIM_DMA_ID_CC1])
    {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_1;

        if (hdma->Init.Mode == DMA_NORMAL)
        {
            TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_1, HAL_TIM_CHANNEL_STATE_READY);
        }
    }
    else if (hdma == htim->hdma[TIM_DMA_ID_CC2])
    {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_2;

        if (hdma->Init.Mode == DMA_NORMAL)
        {
            TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_2, HAL_TIM_CHANNEL_STATE_READY);
        }
    }
    else if (hdma == htim->hdma[TIM_DMA_ID_CC3])
    {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_3;

        if (hdma->Init.Mode == DMA_NORMAL)
        {
            TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_3, HAL_TIM_CHANNEL_STATE_READY);
        }
    }
    else if (hdma == htim->hdma[TIM_DMA_ID_CC4])
    {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_4;

        if (hdma->Init.Mode == DMA_NORMAL)
        {
            TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_4, HAL_TIM_CHANNEL_STATE_READY);
        }
    }
    else
    {
        /* nothing to do */
    }

#if (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
    htim->PWM_PulseFinishedCallback(htim);
#else
    HAL_TIM_PWM_PulseFinishedCallback(htim);
#endif /* USE_HAL_TIM_REGISTER_CALLBACKS */

    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
}
