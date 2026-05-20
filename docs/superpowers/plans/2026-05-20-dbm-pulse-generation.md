# DMA Double-Buffer Mode Pulse Generation — Design Record (REJECTED)

**Status:** REJECTED — 2026-05-20
**Branch:** `feature/dbm-pulse-generation`

## Decision

After analysis, DBM is **not adopted** for this codebase's pulse generation. The current single-buffer + master-timer-pause architecture is retained. This document exists so the next person who proposes DBM doesn't re-derive the rejection.

## Original goal

Eliminate the pulse-period gap that appears at every ring-buffer slot boundary, where the master timer is paused while `HAL_TIM_PWM_PulseFinishedCallback` reconfigures the DMA source for the next slot. The gap shows up as an elongated pulse period and limits the achievable step frequency.

## Approach considered

Replace the stop-DMA → repoint → resume sequence with STM32 DMA Double-Buffer Mode (DBM):
- DMA auto-flips between `M0AR` / `M1AR` on each transfer-complete event.
- Implement `PingPongM0/M1TransferCompleteCallback` overrides to repoint the inactive `M*AR` at the next ring slot.
- Never stop the master timer, never suspend the stream.

The scaffolding (`TIM_OC_Start_DMA_Double_Buffer`, `TIM_OC_Update_DMA_Double_Buffer`, weak ping-pong callbacks) already exists in `Core/Src/stm32f7xx_timer_extension.{c,h}` from earlier exploratory work.

## Why it was rejected

The architecture is incompatible at a structural level, for two coupled reasons.

### 1. Per-axis buffers within a slot have inherently unequal lengths

`pulse_block_t` (one ring-buffer slot) holds **three** `pulse_t`, one per axis. `stepCalculatePulseData` advances the ring head when the *dominant* axis hits `DOUBLE_BUFFER_SIZE`. Non-dominant axes therefore carry a smaller `length`. End-of-motion and direction changes also force partial slots. A single slot routinely looks like:

```
slot N:  X.length = 64,  Y.length = 11,  Z.length = 0
```

STM32 DBM (RM0410 §8.3.10) requires both buffers to have the same DMA transfer size — `NDTR` is shared and reloaded across flips. So at least one axis per slot cannot ride DBM cleanly.

### 2. Partial-buffer transitions require a timer pause anyway

Sentinel-padding doesn't rescue this. In OC-DMA mode, DMA advances on `CCxDE` (compare-match), so:
- A far-future sentinel CCR (e.g., `0xFFFFFFFF`) stalls the stream waiting for a match — DMA hangs.
- A near-future sentinel produces spurious GPIO toggles — junk pulses to the motor.

No padding value is both consumed by DMA and silent on the output.

A hybrid (DBM for full-length axes, single-buffer for partial axes) was considered. It fails because every partial-axis transition still needs a manual DMA reconfigure during which the master timer is running. The counter then races the new CCR:

- **Timer left running:** counter overshoots the next scheduled CCR before DMA loads it → missed pulse / stall.
- **Timer paused:** identical to current behavior — the latency gap returns.

This was the original rationale for choosing single-buffer mode when the codebase was written.

## What was actually built before rejection

- `Core/Src/stm32f7xx_timer_extension.{c,h}` already contains DBM start/update plumbing with the `CT`-bit guard on `M*AR` updates. Left in place — harmless if unused; useful if a future redesign decouples per-axis buffers.
- This document.

No source changes were made.

## Alternative directions worth profiling first if revisiting the latency goal

In rough order of leverage:

1. **Measure the actual gap.** Toggle a GPIO at entry/exit of `HAL_TIM_PWM_PulseFinishedCallback`; capture on a logic analyzer. The number may make the next priority obvious — or reveal the gap is already small enough.
2. **Trim the ISR critical section.** `stepUpdateDMABuffer` currently iterates all axes and touches GPIOs / helper macros inside the ISR. A leaner per-axis-only critical section likely shrinks the gap meaningfully.
3. **Pre-resolve `pulseBlockAddress` in `stepTask`.** Move `stepGetAvailableDataAddress()` out of the ISR; the ISR just consumes a pre-staged pointer.
4. **Larger slot size.** Increasing `DOUBLE_BUFFER_SIZE` (e.g. 64 → 256) cuts the boundary-event rate proportionally — the gap is unchanged per occurrence but occurs 4× less often. SRAM cost only, no architectural risk.

If any of these is pursued, baseline the gap **first**, then re-measure — otherwise the improvement isn't quantifiable.

## References

- RM0410 (STM32F767 reference manual), §8.3.10 "Double-buffer mode" — same-size constraint and `NDTR` reload behavior.
- `Core/Src/step.c:455-680` — `stepCalculatePulseData` and `stepUpdateDMABuffer` showing the per-axis variable-length slot structure.
- `Core/Src/step.c:708-777` — `HAL_TIM_PWM_PulseFinishedCallback` showing the master-timer pause.
- `Core/Src/stm32f7xx_timer_extension.c` — DBM HAL extension that this analysis confirms is unused in the production path.
