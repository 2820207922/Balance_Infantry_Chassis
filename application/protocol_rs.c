#include "protocol_rs.h"
#include "bsp_usart_dma.h"
#include "crc.h"
#include "main.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

static uint8_t rs_tx_buf[RS_TX_BUF_SIZE];
static uint8_t rs_rx_buf[2][RS_RX_BUF_SIZE];

game_status_t game_status;
game_result_t game_result;
game_robot_HP_t game_robot_HP;

event_data_t event_data;
ext_supply_projectile_action_t ext_supply_projectile_action;
referee_warning_t referee_warning;
dart_info_t dart_info;

robot_status_t robot_status;
power_heat_data_t power_heat_data;
robot_pos_t robot_pos;
buff_t buff;
air_support_data_t air_support_data;
hurt_data_t hurt_data;
shoot_data_t shoot_data;
projectile_allowance_t projectile_allowance;
rfid_status_t rfid_status;
dart_client_cmd_t dart_client_cmd;
ground_robot_position_t ground_robot_position;
radar_mark_data_t radar_mark_data;
sentry_info_t sentry_info;
radar_info_t radar_info;

robot_interaction_data_t *robot_interaction_data_send, *robot_interaction_data_recv;
interaction_layer_delete_t interaction_layer_delete;
interaction_figure_t interaction_figure;
interaction_figure_2_t interaction_figure_2;
interaction_figure_3_t interaction_figure_3;
interaction_figure_4_t interaction_figure_4;
ext_client_custom_character_t ext_client_custom_character;
sentry_cmd_t sentry_cmd;
radar_cmd_t radar_cmd;

map_command_t map_command;
map_robot_data_t map_robot_data;
map_data_t map_data;
custom_info_t custom_info;

custom_robot_data_t custom_robot_data;
remote_control_t remote_control;
custom_client_data_t custom_client_data;

msg_c2agx_t msg_c2agx;
msg_agx2c_t msg_agx2c;


void usart_printf(const char *fmt,...)
{
 static uint8_t tx_buf[256] = {0};
 static va_list ap;
 static uint16_t len;
 va_start(ap, fmt);
 //return length of string
 len = vsprintf((char *)tx_buf, fmt, ap);
 va_end(ap);
 HAL_UART_Transmit_DMA(&huart1, tx_buf, len + 9);
}

void rs_init(void)
{
	usart_tx_dma_init(&huart1, &hdma_usart1_rx);
	usart_rx_dma_init(&huart1, &hdma_usart1_rx, rs_rx_buf[0], rs_rx_buf[1], RS_RX_BUF_SIZE);
}

void rs_send(const uint16_t cmd_id, const uint8_t *data, const uint16_t len)
{
	frame_header_t frame_header;
	frame_header.sof = SOF;
	frame_header.data_length = len;
	frame_header.seq = 0;
	frame_header.crc8 = 0;
	// memset(rs_tx_buf, 0, RS_TX_BUF_SIZE);

	Append_CRC8_Check_Sum((uint8_t *)&frame_header, 5);
	memcpy(rs_tx_buf, &frame_header, 5);
	memcpy(rs_tx_buf + 5, &cmd_id, 2);
	memcpy(rs_tx_buf + 7, data, len);
	Append_CRC16_Check_Sum(rs_tx_buf, len + 9);

	HAL_UART_Transmit_DMA(&huart1, rs_tx_buf, len + 9);

	HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
}

void rs_send_interaction(uint16_t data_cmd_id, uint16_t sender_id, uint16_t receiver_id, const uint8_t *user_data, const uint16_t len)
{
	if (robot_interaction_data_send != NULL)
	{
		free(robot_interaction_data_send);
		robot_interaction_data_send = NULL;
	}
	
	robot_interaction_data_send = (robot_interaction_data_t*)malloc(len + 6);
	robot_interaction_data_send->data_cmd_id = data_cmd_id;
	robot_interaction_data_send->sender_id = sender_id;
	robot_interaction_data_send->receiver_id = receiver_id;
	memcpy(robot_interaction_data_send->user_data, user_data, len);
	
	rs_send(ROBOT_INTERACTION_DATA_ID, (uint8_t*)robot_interaction_data_send, len + 6);
}

static void rs_interation_handler(const robot_interaction_data_t *robot_interaction_data)
{
	switch (robot_interaction_data->data_cmd_id)
	{
	case INTERACTION_LAYER_DELETE_ID:
		HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
		memcpy((void *)&interaction_layer_delete, robot_interaction_data->user_data, sizeof(interaction_layer_delete_t));
		break;

	case INTERACTION_FIGURE_ID:
		memcpy((void *)&interaction_figure, robot_interaction_data->user_data, sizeof(interaction_figure_t));
		break;

	case INTERACTION_FIGURE_2_ID:
		memcpy((void *)&interaction_figure_2, robot_interaction_data->user_data, sizeof(interaction_figure_2_t));
		break;

	case INTERACTION_FIGURE_3_ID:
		memcpy((void *)&interaction_figure_3, robot_interaction_data->user_data, sizeof(interaction_figure_3_t));
		break;

	case INTERACTION_FIGURE_4_ID:
		memcpy((void *)&interaction_figure_4, robot_interaction_data->user_data, sizeof(interaction_figure_4_t));
		break;

	case EXT_CLIENT_CUSTOM_CHARACTER_ID:
		memcpy((void *)&ext_client_custom_character, robot_interaction_data->user_data, sizeof(ext_client_custom_character_t));
		break;

	case SENTRY_CMD_ID:
		memcpy((void *)&sentry_cmd, robot_interaction_data->user_data, sizeof(sentry_cmd_t));
		break;

	case RADAR_CMD_ID:
		memcpy((void *)&radar_cmd, robot_interaction_data->user_data, sizeof(radar_cmd_t));
		break;

	default:
		break;
	}
}

static void rs_recv_handler(const uint8_t data)
{
	static uint8_t recv_buf[RS_RX_BUF_SIZE];
	static uint16_t recv_index;
	static frame_header_t recv_header;

	if (recv_index == 0)
	{
		if (data == SOF)
		{
			recv_buf[recv_index++] = data;
		}
	}
	else if (recv_index == 5)
	{
		if (Verify_CRC8_Check_Sum((uint8_t *)recv_buf, 5))
		{
			memcpy(&recv_header, recv_buf, 5);
			recv_buf[recv_index++] = data;
		}
		else
		{
			recv_index = 0;
			if (data == SOF)
			{
				recv_buf[recv_index++] = data;
			}
		}
	}
	else if (recv_index == recv_header.data_length + 9)
	{
		if (Verify_CRC16_Check_Sum((uint8_t *)recv_buf, recv_header.data_length + 9))
		{
			HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);

			uint16_t cmd_id;
			memcpy(&cmd_id, recv_buf + 5, 2);
			switch (cmd_id)
			{
			case GAME_STATUS_ID:
				memcpy(&game_status, recv_buf + 7, recv_header.data_length);
				break;

			case GAME_RESULT_ID:
				memcpy(&game_result, recv_buf + 7, recv_header.data_length);
				break;

			case GAME_ROBOT_HP_ID:
				memcpy(&game_robot_HP, recv_buf + 7, recv_header.data_length);
				break;

			case EVENT_DATA_ID:
				memcpy(&event_data, recv_buf + 7, recv_header.data_length);
				break;

			case EXT_SUPPLY_PROJECTILE_ACTION_ID:
				memcpy(&ext_supply_projectile_action, recv_buf + 7, recv_header.data_length);
				break;

			case REFEREE_WARNING_ID:
				memcpy(&referee_warning, recv_buf + 7, recv_header.data_length);
				break;

			case DART_INFO_ID:
				memcpy(&dart_info, recv_buf + 7, recv_header.data_length);
				break;

			case ROBOT_STATUS_ID:
				memcpy(&robot_status, recv_buf + 7, recv_header.data_length);
				break;

			case POWER_HEAT_DATA_ID:
				memcpy(&power_heat_data, recv_buf + 7, recv_header.data_length);
				break;

			case ROBOT_POS_ID:
				memcpy(&robot_pos, recv_buf + 7, recv_header.data_length);
				break;

			case BUFF_ID:
				memcpy(&buff, recv_buf + 7, recv_header.data_length);
				break;

			case AIR_SUPPORT_DATA_ID:
				memcpy(&air_support_data, recv_buf + 7, recv_header.data_length);
				break;

			case HURT_DATA_ID:
				memcpy(&hurt_data, recv_buf + 7, recv_header.data_length);
				break;

			case SHOOT_DATA_ID:
				memcpy(&shoot_data, recv_buf + 7, recv_header.data_length);
				break;

			case PROJECTILE_ALLOWANCE_ID:
				memcpy(&projectile_allowance, recv_buf + 7, recv_header.data_length);
				break;

			case RFID_STATUS_ID:
				memcpy(&rfid_status, recv_buf + 7, recv_header.data_length);
				break;

			case DART_CLIENT_CMD_ID:
				memcpy(&dart_client_cmd, recv_buf + 7, recv_header.data_length);
				break;

			case GROUND_ROBOT_POSITION_ID:
				memcpy(&ground_robot_position, recv_buf + 7, recv_header.data_length);
				break;

			case RADAR_MARK_DATA_ID:
				memcpy(&radar_mark_data, recv_buf + 7, recv_header.data_length);
				break;

			case SENTRY_INFO_ID:
				memcpy(&sentry_info, recv_buf + 7, recv_header.data_length);
				break;

			case RADAR_INFO_ID:
				memcpy(&radar_info, recv_buf + 7, recv_header.data_length);
				break;

			case ROBOT_INTERACTION_DATA_ID:
				if (robot_interaction_data_recv != NULL)
				{
					free(robot_interaction_data_recv);
					robot_interaction_data_recv = NULL;
				}
				robot_interaction_data_recv = (robot_interaction_data_t *)malloc(recv_header.data_length);
				memcpy(robot_interaction_data_recv, recv_buf + 7, recv_header.data_length);
				rs_interation_handler(robot_interaction_data_recv);
				break;

			case MAP_COMMAND_ID:
				memcpy(&map_command, recv_buf + 7, recv_header.data_length);
				break;

			case MAP_ROBOT_DATA_ID:
				memcpy(&map_robot_data, recv_buf + 7, recv_header.data_length);
				break;

			case MAP_DATA_ID:
				memcpy(&map_data, recv_buf + 7, recv_header.data_length);
				break;

			case CUSTOM_INFO_ID:
				memcpy(&custom_info, recv_buf + 7, recv_header.data_length);
				break;

			case CUSTOM_ROBOT_DATA_ID:
				memcpy(&custom_robot_data, recv_buf + 7, recv_header.data_length);
				break;

			case REMOTE_CONTROL_ID:
				memcpy(&remote_control, recv_buf + 7, recv_header.data_length);
				break;

			case CUSTOM_CLIENT_DATA_ID:
				memcpy(&custom_client_data, recv_buf + 7, recv_header.data_length);
				break;

			case MSG_C2AGX_ID:
				memcpy(&msg_c2agx, recv_buf + 7, recv_header.data_length);
				break;

			case MSG_AGX2C_ID:
				memcpy(&msg_agx2c, recv_buf + 7, recv_header.data_length);
				break;

			default:
				break;
			}

			recv_index = 0;
			memset(recv_buf, 0, sizeof(recv_buf));
			memset(&recv_header, 0, sizeof(recv_header));

			if (data == SOF)
			{
				recv_buf[recv_index++] = data;
			}
		}
		else
		{
			recv_index = 0;
			if (data == SOF)
			{
				recv_buf[recv_index++] = data;
			}
		}
	}
	else
	{
		recv_buf[recv_index++] = data;
	}

	if (recv_index == 256)
	{
		recv_index = 0;
	}
}

void USART1_IRQHandler(void)
{
	if (huart1.Instance->SR & UART_FLAG_IDLE || huart1.Instance->SR & UART_FLAG_RXNE)
	{
		static uint16_t this_time_rx_len = 0;

		__HAL_UART_CLEAR_PEFLAG(&huart1);

		if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			/* Current memory buffer used is Memory 0 */

			// disable DMA
			__HAL_DMA_DISABLE(&hdma_usart1_rx);

			// get receive data length, length = set_data_length - remain_length
			this_time_rx_len = RS_RX_BUF_SIZE - hdma_usart1_rx.Instance->NDTR;

			// reset set_data_lenght
			hdma_usart1_rx.Instance->NDTR = RS_RX_BUF_SIZE;

			// set memory buffer 1
			hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

			// enable DMA
			__HAL_DMA_ENABLE(&hdma_usart1_rx);

			// handle raw data
			for (int i = 0; i < this_time_rx_len; ++i)
			{
				rs_recv_handler(rs_rx_buf[0][i]);
			}
		}
		else
		{
			/* Current memory buffer used is Memory 1 */
			// disable DMA
			__HAL_DMA_DISABLE(&hdma_usart1_rx);

			// get receive data length, length = set_data_length - remain_length
			this_time_rx_len = RS_RX_BUF_SIZE - hdma_usart1_rx.Instance->NDTR;

			// reset set_data_lenght
			hdma_usart1_rx.Instance->NDTR = RS_RX_BUF_SIZE;

			// set memory buffer 0
			DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

			// enable DMA
			__HAL_DMA_ENABLE(&hdma_usart1_rx);

			// handle raw data
			for (int i = 0; i < this_time_rx_len; ++i)
			{
				rs_recv_handler(rs_rx_buf[1][i]);
			}
		}
	}

	HAL_UART_IRQHandler(&huart1);
}
