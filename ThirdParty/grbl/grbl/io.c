
#include "grbl.h"


/**
 * @brief Initialize the I/O pins.
 */
/*
void io_init()
{
    // reset the output pins

}
*/

/**
 * @brief Set the state of an output pin.
 */
void io_output_sync(uint8_t pin, uint8_t state)
{
    // set the state of the pin
    if (sys.state == STATE_CHECK_MODE)
    {
        return;
    }
    protocol_buffer_synchronize(); // Ensure coolant turns on when specified in program.

    switch (pin)
    {
        case PIN_STEPPERS_DISABLE:
            if (state)
            {
                STEPPERS_DISABLE_PORT |= (1 << STEPPERS_DISABLE_BIT);
            }
            else
            {
                STEPPERS_DISABLE_PORT &= ~(1 << STEPPERS_DISABLE_BIT);
            }
            break;
        case PIN_COOLANT_FLOOD:
            if (state)
            {
                COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT);
            }
            else
            {
                COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
            }
            break;
        case PIN_SPINDLE_ENABLE:
            if (state)
            {
                SPINDLE_ENABLE_PORT |= (1 << SPINDLE_ENABLE_BIT);
            }
            else
            {
                SPINDLE_ENABLE_PORT &= ~(1 << SPINDLE_ENABLE_BIT);
            }
            break;
        case PIN_SPINDLE_DIRECTION:
            if (state)
            {
                SPINDLE_DIRECTION_PORT |= (1 << SPINDLE_DIRECTION_BIT);
            }
            else
            {
                SPINDLE_DIRECTION_PORT &= ~(1 << SPINDLE_DIRECTION_BIT);
            }
            break;
        case PIN_USER_OUTPUT_0:
            if (state)
            {
                USER_OUTPUT_PORT &= ~(1 << USER_OUTPUT_0_BIT);
            }
            else
            {
                USER_OUTPUT_PORT |= (1 << USER_OUTPUT_0_BIT);
            }
            break;
        case PIN_USER_OUTPUT_1:
            if (state)
            {
                USER_OUTPUT_PORT &= ~(1 << USER_OUTPUT_1_BIT);
            }
            else
            {
                USER_OUTPUT_PORT |= (1 << USER_OUTPUT_1_BIT);
            }
            break;
        case PIN_USER_OUTPUT_2:
            if (state)
            {
                USER_OUTPUT_PORT &= ~(1 << USER_OUTPUT_2_BIT);
            }
            else
            {
                USER_OUTPUT_PORT |= (1 << USER_OUTPUT_2_BIT);
            }
            break;
        case PIN_USER_OUTPUT_3:
            if (state)
            {
                USER_OUTPUT_PORT &= ~(1 << USER_OUTPUT_3_BIT);
            }
            else
            {
                USER_OUTPUT_PORT |= (1 << USER_OUTPUT_3_BIT);
            }
            break;
    }
}
