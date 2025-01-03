/**
 * @brief   This file provides the extensions to complement the functions in timer HAL.
*/
#include "stm32f7xx_hal.h"

#ifndef __STM32F7xx_TIMER_EXTENSION_H
#define __STM32F7xx_TIMER_EXTENSION_H

#ifdef __cplusplus
 extern "C" {
#endif

#define SELECT_DOUBLE_BUFFER_M0 0x00U
#define SELECT_DOUBLE_BUFFER_M1 0x01U

/* Define Macro */
#define IS_LEGAL_SELECT_DOUBLE_BUFFER(SEL) (((SEL) == SELECT_DOUBLE_BUFFER_M0) || ((SEL) == SELECT_DOUBLE_BUFFER_M1))
#define IS_TARGET_DOUBLE_BUFFER_WRITEABLE(HDMA, TARGET) (HDMA->Instance->CR & DMA_SxCR_EN_Msk) && ((HDMA->Instance->CR & DMA_SxCR_CT_Msk) == ((TARGET ^ 0x01U) << DMA_SxCR_CT_Pos))

HAL_StatusTypeDef TIM_OC_Start_DMA_Double_Buffer(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint16_t DataLength);
HAL_StatusTypeDef TIM_OC_Update_DMA_Double_Buffer(DMA_HandleTypeDef *hdma, uint32_t NewAddress, uint8_t SelectBuffer);

#ifdef __cplusplus
}
#endif

#endif  /* __STM32F7xx_TIMER_EXTENSION_H */