#ifndef BSP_USART_DMA_H
#define BSP_USART_DMA_H

#include "struct_typedef.h"
#include "main.h"

extern void usart_tx_dma_init(UART_HandleTypeDef* huart, DMA_HandleTypeDef *hdma_usart_tx);
extern void usart_tx_dma_enable(DMA_HandleTypeDef *hdma_usart_tx, uint8_t *data, uint16_t len);
extern void usart_rx_dma_init(UART_HandleTypeDef* huart, DMA_HandleTypeDef *hdma_usart_rx, uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

#endif
