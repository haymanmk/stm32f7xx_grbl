# stm32f7xx_grbl

## Commands

- `0xA2` - Read I/O status. The format of the response from GRBL is a string with an Hexidecimal string number inside, e.g. `[IO:0x0123]`, which denotes status of the 16 inputs and 16 outputs as an unsigned 32 bits number. The definition of each bit is as shown below. _NOTE: the newline symbol, `\r`, is not necessary here._

### I/O List

- Inputs
  - `bit 0`: PIN_X_LIMIT
  - `bit 1`: PIN_Y_LIMIT
  - `bit 2`: PIN_Z_LIMIT
  - `bit 3`: PIN_PROBE
  - `bit 4`: PIN_RESET
  - `bit 5`: PIN_FEED_HOLD
  - `bit 6`: PIN_CYCLE_START
  - `bit 7`: PIN_SAFETY_DOOR
- Outputs
  - `bit 16`: PIN_X_DIRECTION
  - `bit 17`: PIN_Y_DIRECTION
  - `bit 18`: PIN_Z_DIRECTION
  - `bit 19`: PIN_STEPPERS_DISABLE
  - `bit 20`: PIN_COOLANT_FLOOD
  - `bit 21`: PIN_SPINDLE_ENABLE
  - `bit 22`: PIN_SPINDLE_DIRECTION
  - `bit 23`: PIN_USER_OUTPUT_0
  - `bit 24`: PIN_USER_OUTPUT_1
  - `bit 25`: PIN_USER_OUTPUT_2
  - `bit 26`: PIN_USER_OUTPUT_3

## Control Output Status

- `M62 P[BitID]` - Set output pin specified by `BitID` as **ACTIVE**. `BitID` is defined in the [I/O List](#io-list). e.g. `M62 P23`, whish activates output pin `PIN_USER_OUTPUT_0`.
- `M63 P[BitID]` - **DEACTIVE** output pin specified by `BitID`.

## Already Known Issues

### Output an unwanted spike during setting OC mode

When setting the output compare mode for each output pin, e.g. toggle, active, inactive, we observed unwanted spikes occasionally occurring after setting the OC mode to toggle. In this case, the open-drain is selected as the output configuration to create the sink logic required by the stepper driver. The polarity is configured as Active HIGH which means the output turns into HIGH state when the OCx is active. For the sink logic, a pull up resistor is necessary to avoid the floating state of the output. So it is obviously to know that the output will stay in HIGH when the OCx is inactive if the OC channel has been reset or restart. The OC setting process has this reset procedure in the first place, and then it is followed by setting the OC mode to the target mode and enable it again, which in turn forces the OCx to the state, LOW in our case, before the OC channel being disabled. That why the spike comes from. In order to address this issue, the **polarity** shall be set to Active LOW, by doing so, the OCx state before resetting the OC channel will be exactly the same with that after disabling the OC channel, both of them are HIGH (due to pull high) even during the transition, and then the unwanted spikes are disappeared.
