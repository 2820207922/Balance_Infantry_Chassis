#include "protocol_mt.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static uint8_t can_tx_data[8];
static uint8_t can_rx_data[8];

lk_recv_data_t lk_recv_data[4];
rm_recv_data_t rm_recv_data[8];


void lk_recv_decoder(lk_recv_data_t *ptr, const uint8_t *data)
{
	ptr->flag = 1;
	ptr->temperature = (uint8_t)data[1];
	ptr->iq = (int16_t)((data[2] << 8) | data[3]);
	ptr->speed = (int16_t)((data[4] << 8) | data[5]);
	ptr->encoder = (int16_t)((data[6] << 8) | data[7]);
}

void rm_recv_decoder(rm_recv_data_t *ptr, const uint8_t *data)
{
	ptr->last_ecd = ptr->ecd;
	ptr->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
	ptr->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);
	ptr->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);
	ptr->temperate = (data)[6];
	
	// HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
}

void mt_send(mt_cmd_id_e mt_cmd_id, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	CAN_TxHeaderTypeDef tx_hander;
	tx_hander.StdId = mt_cmd_id;
	tx_hander.IDE = CAN_ID_STD;
	tx_hander.RTR = CAN_RTR_DATA;
	tx_hander.DLC = 0x08;
	
	can_tx_data[0] = (motor1 >> 8);
	can_tx_data[1] = motor1;
	can_tx_data[2] = (motor2 >> 8);
	can_tx_data[3] = motor2;
	can_tx_data[4] = (motor3 >> 8);
	can_tx_data[5] = motor3;
	can_tx_data[6] = (motor4 >> 8);
	can_tx_data[7] = motor4;
	
	uint32_t send_mail_box;
	HAL_CAN_AddTxMessage(&hcan2, &tx_hander, can_tx_data, &send_mail_box);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, can_rx_data);

		if (rx_header.StdId & LK_SINGLE_ID)
		{
			uint8_t lk_receive_id;
			lk_receive_id = rx_header.StdId - LK_SINGLE_ID;
			lk_recv_decoder(&lk_recv_data[lk_receive_id - 1], can_rx_data);
		}
		else if (rx_header.StdId & RM_C620_L_ID)
		{
			uint8_t rm_receive_id;
			rm_receive_id = rx_header.StdId - RM_C620_L_ID;
			rm_recv_decoder(&rm_recv_data[rm_receive_id - 1], can_rx_data);
		}
}

const lk_recv_data_t *get_lk_recv_iq_data(uint8_t motor_id)
{
	return &lk_recv_data[motor_id - 1];
}

const rm_recv_data_t *get_rm_recv_data(uint8_t motor_id)
{
	return &rm_recv_data[motor_id - 1];
}
