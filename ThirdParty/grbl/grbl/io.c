
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
 * @brief Report the status of I/O pins.
 * @return uint16_t status of I/O pins; inputs: bit 0-15, outputs: bit 16-31
 */
uint32_t io_report_status()
{
    uint32_t status = 0;
    // gather the status of input pins
    status |= (LIMIT_PIN & (1 << X_LIMIT_BIT)) ? (1 << PIN_X_LIMIT) : 0;
    status |= (LIMIT_PIN & (1 << Y_LIMIT_BIT)) ? (1 << PIN_Y_LIMIT) : 0;
    status |= (LIMIT_PIN & (1 << Z_LIMIT_BIT)) ? (1 << PIN_Z_LIMIT) : 0;
    status |= (PROBE_PIN & (1 << PROBE_BIT)) ? (1 << PIN_PROBE) : 0;
    status |= (CONTROL_PIN & (1 << CONTROL_RESET_BIT)) ? (1 << PIN_RESET) : 0;
    status |= (CONTROL_PIN & (1 << CONTROL_FEED_HOLD_BIT)) ? (1 << PIN_FEED_HOLD) : 0;
    status |= (CONTROL_PIN & (1 << CONTROL_CYCLE_START_BIT)) ? (1 << PIN_CYCLE_START) : 0;
    status |= (CONTROL_PIN & (1 << CONTROL_SAFETY_DOOR_BIT)) ? (1 << PIN_SAFETY_DOOR) : 0;

    // gather the status of output pins
    status |= (X_DIRECTION_PORT & (1 << X_DIRECTION_BIT)) ? (1 << PIN_X_DIRECTION) : 0;
    status |= (Y_DIRECTION_PORT & (1 << Y_DIRECTION_BIT)) ? (1 << PIN_Y_DIRECTION) : 0;
    status |= (Z_DIRECTION_PORT & (1 << Z_DIRECTION_BIT)) ? (1 << PIN_Z_DIRECTION) : 0;
    status |= (STEPPERS_DISABLE_PORT & (1 << STEPPERS_DISABLE_BIT)) ? (1 << PIN_STEPPERS_DISABLE) : 0;
    status |= (COOLANT_FLOOD_PORT & (1 << COOLANT_FLOOD_BIT)) ? (1 << PIN_COOLANT_FLOOD) : 0;
    status |= (SPINDLE_ENABLE_PORT & (1 << SPINDLE_ENABLE_BIT)) ? (1 << PIN_SPINDLE_ENABLE) : 0;
    status |= (SPINDLE_DIRECTION_PORT & (1 << SPINDLE_DIRECTION_BIT)) ? (1 << PIN_SPINDLE_DIRECTION) : 0;

    return status;
}

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
    }
}
