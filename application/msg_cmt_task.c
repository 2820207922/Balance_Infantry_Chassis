#include "msg_cmt_task.h"
#include "protocol_rs.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "string.h"
#include "stdlib.h"

const uint8_t figure_name1[3] = "p1";

extern fp32 INS_accel[3];
extern fp32 INS_gyro[3];
extern fp32 INS_mag[3];

extern interaction_layer_delete_t interaction_layer_delete;
extern interaction_figure_t interaction_figure;
extern msg_c2agx_t msg_c2agx;

void msg_cmt_task(void const *argument)
{
	while (1)
	{
		memcpy((void *)msg_c2agx.accel, INS_accel, sizeof(INS_accel));
		memcpy((void *)msg_c2agx.gyro, INS_gyro, sizeof(INS_gyro));
		memcpy((void *)msg_c2agx.mag, INS_mag, sizeof(INS_mag));
		rs_send(MSG_C2AGX_ID, (uint8_t*)&msg_c2agx, sizeof(msg_c2agx_t));
		vTaskDelay(5);
		
//		memcpy((void *)interaction_figure.figure_name, figure_name1, sizeof(figure_name1));
//		interaction_figure.operate_tpye = 1;
//		interaction_figure.figure_tpye = 0;
//		interaction_figure.layer = 0;
//		interaction_figure.color = 0;
//		interaction_figure.width = 100;
//		interaction_figure.start_x = 360;
//		interaction_figure.start_y = 240;
//		interaction_figure.details_a = 0;
//		interaction_figure.details_b = 0;
//		interaction_figure.details_c = 0;
//		interaction_figure.details_d = 1280;
//		interaction_figure.details_e = 720;
//		
//		rs_send_interaction(INTERACTION_FIGURE_ID, 103, 0x103, (uint8_t*)&interaction_figure, sizeof(interaction_figure_t));
//		vTaskDelay(500);
//		
//		interaction_layer_delete.delete_type = 1;
//		interaction_layer_delete.layer = 0;
//		
//		rs_send_interaction(INTERACTION_LAYER_DELETE_ID, 103, 0x103, (uint8_t*)&interaction_layer_delete, sizeof(interaction_layer_delete_t));
//		vTaskDelay(500);
	}
}
