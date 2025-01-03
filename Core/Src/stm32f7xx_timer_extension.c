/**
 * @brief   This file is the source file to extend functions in timer HAL.
 *
 */

#include "stm32f7xx_timer_extension.h"

static void TIM_DMAM0TransferCplt(DMA_HandleTypeDef *hdma);
static void TIM_DMAM1TransferCplt(DMA_HandleTypeDef *hdma);

HAL_StatusTypeDef TIM_OC_Start_DMA_Double_Buffer(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint16_t DataLength)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Check parameters */
    assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));

    /* Set TIM channel state */
    if (TIM_CHANNEL_STATE_GET(htim, Channel) == HAL_TIM_CHANNEL_STATE_BUSY)
        return HAL_BUSY;
    else if (TIM_CHANNEL_STATE_GET(htim, Channel) == HAL_TIM_CHANNEL_STATE_READY)
    {
        if ((DataLength == 0U))
            return HAL_ERROR;
        else
            TIM_CHANNEL_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);
    }
    else
        return HAL_ERROR;

    switch (Channel)
    {
    case TIM_CHANNEL_1:
    {
        /* Set the DMA compare callbacks */
        htim->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = TIM_DMAM0TransferCplt;
        htim->hdma[TIM_DMA_ID_CC1]->XferM1CpltCallback = TIM_DMAM1TransferCplt;
        // htim->hdma[TIM_DMA_ID_CC1]->XferHalfCpltCallback = TIM_DMADelayPulseHalfCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC1]->XferErrorCallback = TIM_DMAError;

        /* Enable the DMA stream */
        if (HAL_DMAEx_MultiBufferStart_IT(htim->hdma[TIM_DMA_ID_CC1], SrcAddress, DstAddress, SecondMemAddress, DataLength) != HAL_OK)
        {
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
        htim->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = TIM_DMAM0TransferCplt;
        htim->hdma[TIM_DMA_ID_CC2]->XferM1CpltCallback = TIM_DMAM1TransferCplt;
        // htim->hdma[TIM_DMA_ID_CC2]->XferHalfCpltCallback = TIM_DMADelayPulseHalfCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC2]->XferErrorCallback = TIM_DMAError;

        /* Enable the DMA stream */
        if (HAL_DMAEx_MultiBufferStart_IT(htim->hdma[TIM_DMA_ID_CC2], SrcAddress, DstAddress, SecondMemAddress, DataLength) != HAL_OK)
        {
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
        htim->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = TIM_DMAM0TransferCplt;
        htim->hdma[TIM_DMA_ID_CC3]->XferM1CpltCallback = TIM_DMAM1TransferCplt;
        // htim->hdma[TIM_DMA_ID_CC3]->XferHalfCpltCallback = TIM_DMADelayPulseHalfCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC3]->XferErrorCallback = TIM_DMAError;

        /* Enable the DMA stream */
        if (HAL_DMAEx_MultiBufferStart_IT(htim->hdma[TIM_DMA_ID_CC3], SrcAddress, DstAddress, SecondMemAddress, DataLength) != HAL_OK)
        {
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
        htim->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = TIM_DMAM0TransferCplt;
        htim->hdma[TIM_DMA_ID_CC4]->XferM1CpltCallback = TIM_DMAM1TransferCplt;
        // htim->hdma[TIM_DMA_ID_CC4]->XferHalfCpltCallback = TIM_DMADelayPulseHalfCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC4]->XferErrorCallback = TIM_DMAError;

        /* Enable the DMA stream */
        if (HAL_DMAEx_MultiBufferStart_IT(htim->hdma[TIM_DMA_ID_CC4], SrcAddress, DstAddress, SecondMemAddress, DataLength) != HAL_OK)
        {
            /* Return error status */
            return HAL_ERROR;
        }

        /* Enable the TIM Capture/Compare 4 DMA request */
        __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC4);
        break;
    }
    }

    if (status == HAL_OK)
    {
        /* Enable the Output compare channel */
        TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_ENABLE);

        // __HAL_TIM_ENABLE(htim);
    }

    return status;
}

/**
 * @brief   Update the DMA double buffer in TIM DMA double buffer mode.
 * @param   hdma pointer to DMA handle.
 * @param   NewAddress new address to update the DMA buffer.
 * @param   SelectBuffer select the M0/M1 buffer to update. Use SELECT_DOUBLE_BUFFER_M0 or SELECT_DOUBLE_BUFFER_M1.
*/
HAL_StatusTypeDef TIM_OC_Update_DMA_Double_Buffer(DMA_HandleTypeDef *hdma, uint32_t NewAddress, uint8_t SelectBuffer)
{
    if(IS_LEGAL_SELECT_DOUBLE_BUFFER(SelectBuffer) == 0)
    {
        return HAL_ERROR;
    }

    if(IS_TARGET_DOUBLE_BUFFER_WRITEABLE(hdma, SelectBuffer) == 0)
    {
        return HAL_ERROR;
    }

    if (hdma->State == HAL_DMA_STATE_READY)
    {
        /* Change DMA peripheral state */
        hdma->State = HAL_DMA_STATE_BUSY;

        /* Initialize the error code */
        hdma->ErrorCode = HAL_DMA_ERROR_NONE;

        /* Change the memory 0 address */
        if (SelectBuffer == SELECT_DOUBLE_BUFFER_M0)
        {
            hdma->Instance->M0AR = NewAddress;
        }
        /* Change the memory 1 address */
        else
        {
            hdma->Instance->M1AR = NewAddress;
        }
    }

    return HAL_OK;
}

/**
 * @brief   M0 buffer transfer complete callback in DMA Ping Pong mode.
 * @param   htim pointer to timer handle
 * @retval  None
 */
__weak void PingPongM0TransferCompleteCallback(TIM_HandleTypeDef *htim)
{
    UNUSED(htim);
}

/**
 * @brief   M1 buffer transfer complete callback in DMA Ping Pong mode.
 * @param   htim pointer to timer handle
 * @retval  None
 */
__weak void PingPongM1TransferCompleteCallback(TIM_HandleTypeDef *htim)
{
    UNUSED(htim);
}

/**
 * @brief   M0 buffer transfer complete callback in TIM DMA double buffer mode.
 * @param  hdma pointer to DMA handle.
 * @retval None
 */
static void TIM_DMAM0TransferCplt(DMA_HandleTypeDef *hdma)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    if (hdma == htim->hdma[TIM_DMA_ID_CC1])
    {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_1;
    }
    else if (hdma == htim->hdma[TIM_DMA_ID_CC2])
    {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_2;
    }
    else if (hdma == htim->hdma[TIM_DMA_ID_CC3])
    {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_3;
    }
    else if (hdma == htim->hdma[TIM_DMA_ID_CC4])
    {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_4;
    }
    else
    {
        /* nothing to do */
    }

    PingPongM0TransferCompleteCallback(htim);

    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
}

/**
 * @brief   M1 buffer transfer complete callback in TIM DMA double buffer mode.
 * @param  hdma pointer to DMA handle.
 * @retval None
 */
static void TIM_DMAM1TransferCplt(DMA_HandleTypeDef *hdma)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    if (hdma == htim->hdma[TIM_DMA_ID_CC1])
    {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_1;
    }
    else if (hdma == htim->hdma[TIM_DMA_ID_CC2])
    {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_2;
    }
    else if (hdma == htim->hdma[TIM_DMA_ID_CC3])
    {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_3;
    }
    else if (hdma == htim->hdma[TIM_DMA_ID_CC4])
    {
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_4;
    }
    else
    {
        /* nothing to do */
    }

    PingPongM1TransferCompleteCallback(htim);

    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
}