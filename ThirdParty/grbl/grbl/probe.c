/*
  probe.c - code pertaining to probing methods
  Part of Grbl

  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"

// Inverts the probe pin state depending on user settings and probing cycle mode.
IO_TYPE probe_invert_mask;

#ifdef STM32F7XX_ARCH
encoder_degree_t probe_encoder_position;
#endif // STM32F7XX_ARCH

// Probe pin initialization routine.
void probe_init()
{
#if defined(AVR_ARCH)
  PROBE_DDR &= ~(PROBE_MASK); // Configure as input pins
#ifdef DISABLE_PROBE_PIN_PULL_UP
  PROBE_PORT &= ~(PROBE_MASK); // Normal low operation. Requires external pull-down.
#else
  PROBE_PORT |= PROBE_MASK; // Enable internal pull-up resistors. Normal high operation.
#endif
#elif defined(STM32F7XX_ARCH)
  // Configure the probe pin as an input with pull-up resistor.
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = PROBE_MASK;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
#ifdef DISABLE_PROBE_PIN_PULL_UP
  GPIO_InitStruct.Pull = GPIO_NOPULL;
#else
  GPIO_InitStruct.Pull = GPIO_PULLUP;
#endif
  HAL_GPIO_Init(PROBE_GPIO_GROUP, &GPIO_InitStruct);
#endif                                // AVR_ARCH
  probe_configure_invert_mask(false); // Initialize invert mask.
}

// Called by probe_init() and the mc_probe() routines. Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
void probe_configure_invert_mask(uint8_t is_probe_away)
{
  probe_invert_mask = 0; // Initialize as zero.
  if (bit_isfalse(settings.flags, BITFLAG_INVERT_PROBE_PIN))
  {
    probe_invert_mask ^= PROBE_MASK;
  }
  if (is_probe_away)
  {
    probe_invert_mask ^= PROBE_MASK;
  }
}

// Returns the probe pin state. Triggered = true. Called by gcode parser and probe state monitor.
IO_TYPE probe_get_state() { return ((PROBE_PIN & PROBE_MASK) ^ probe_invert_mask); }

// Monitors probe pin state and records the system position when detected. Called by the
// stepper ISR per ISR tick.
// NOTE: This function must be extremely efficient as to not bog down the stepper ISR.
void probe_state_monitor()
{
  if (probe_get_state())
  {
    sys_probe_state = PROBE_OFF;
    memcpy(sys_probe_position, sys_position, sizeof(sys_position));
#ifdef STM32F7XX_ARCH
    encoderReadInstantPosition(&probe_encoder_position);
#endif // STM32F7XX_ARCH
    bit_true(sys_rt_exec_state, EXEC_MOTION_CANCEL);
  }
}
