#include "bsp_usart_dma.h"

void usart_tx_dma_init(UART_HandleTypeDef* huart, DMA_HandleTypeDef *hdma_usart_tx)
{
    //enable the DMA transfer for the tramsmit request
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAT);
}

void usart_tx_dma_enable(DMA_HandleTypeDef *hdma_usart_tx, uint8_t *data, uint16_t len)
{
    //disable DMA
    __HAL_DMA_DISABLE(hdma_usart_tx);
    while(hdma_usart_tx->Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(hdma_usart_tx);
    }

    //clear flag
    __HAL_DMA_CLEAR_FLAG(hdma_usart_tx, DMA_HISR_TCIF7);
    __HAL_DMA_CLEAR_FLAG(hdma_usart_tx, DMA_HISR_HTIF7);

    //set data address
    hdma_usart_tx->Instance->M0AR = (uint32_t)(data);
    //set data length
    hdma_usart_tx->Instance->NDTR = len;
    //enable DMA
    __HAL_DMA_ENABLE(hdma_usart_tx);
}

void usart_rx_dma_init(UART_HandleTypeDef* huart, DMA_HandleTypeDef *hdma_usart_rx, uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    //enalbe idle interrupt
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
		//disable DMA
	
    __HAL_DMA_DISABLE(hdma_usart_rx);
    while(hdma_usart_rx->Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(hdma_usart_rx);
    }

    hdma_usart_rx->Instance->PAR = (uint32_t) & (huart->Instance->DR);
    //memory buffer 1
    hdma_usart_rx->Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    hdma_usart_rx->Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    hdma_usart_rx->Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    SET_BIT(hdma_usart_rx->Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    __HAL_DMA_ENABLE(hdma_usart_rx);
}

