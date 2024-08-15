#ifndef IO_H
#define IO_H

#include "grbl.h"

enum IO_PIN {
    // Input pins
    PIN_X_LIMIT = 0,
    PIN_Y_LIMIT,
    PIN_Z_LIMIT,
    PIN_PROBE,
    PIN_RESET,
    PIN_FEED_HOLD,
    PIN_CYCLE_START,
    PIN_SAFETY_DOOR,
    // Output pins
    PIN_X_DIRECTION = 16,
    PIN_Y_DIRECTION,
    PIN_Z_DIRECTION,
    PIN_STEPPERS_DISABLE,
    PIN_COOLANT_FLOOD,
    PIN_SPINDLE_ENABLE,
    PIN_SPINDLE_DIRECTION,
};

/* Macros */

/* Function Prototypes */
uint32_t io_report_status();
void io_output_sync(uint8_t pin, uint8_t state); // Set the state of an output pin

#endif // IO_H