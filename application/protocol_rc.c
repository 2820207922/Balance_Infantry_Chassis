#include "protocol_rc.h"
#include "bsp_usart_dma.h"
#include "main.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

//remote control data 
rc_data_t rc_data;

//receive data, 18 bytes one frame, but set 36 bytes 
static uint8_t sbus_rx_buf[2][RC_RX_BUF_SIZE];

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
void rc_init(void)
{
		usart_rx_dma_init(&huart3, &hdma_usart3_rx, sbus_rx_buf[0], sbus_rx_buf[1], RC_RX_BUF_SIZE);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
const rc_data_t *get_rc_data(void)
{
    return &rc_data;
}

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf)
{
    if (sbus_buf == NULL)
    {
        return;
    }

    rc_data.rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_data.rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_data.rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                       (sbus_buf[4] << 10)) &0x07ff;
    rc_data.rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_data.rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_data.rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_data.mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_data.mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_data.mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_data.mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_data.mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_data.key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_data.rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL
					 
    rc_data.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_data.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_data.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_data.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_data.rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            this_time_rx_len = RC_RX_BUF_SIZE - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            hdma_usart3_rx.Instance->NDTR = RC_RX_BUF_SIZE;

            //set memory buffer 1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            this_time_rx_len = RC_RX_BUF_SIZE - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            hdma_usart3_rx.Instance->NDTR = RC_RX_BUF_SIZE;

            //set memory buffer 0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[1]);
            }
        }
    }
}


