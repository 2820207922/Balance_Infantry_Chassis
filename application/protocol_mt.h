#ifndef PROTOCOL_MT_H
#define PROTOCOL_MT_H

#include "struct_typedef.h"
#include "main.h"

typedef enum
{
		MSG_GIMBAL_ID = 0x101,
		MSG_CHASSIS_ID = 0x102,
	
		LK_SINGLE_ID = 0x140,
		LK_MULTIPLE_ID = 0x280,
	
		RM_C620_L_ID = 0x200,
		RM_C620_H_ID = 0x1FF
	
} mt_cmd_id_e;

typedef struct
{
	int16_t vx;
	int16_t vy;
	int16_t vw;
	int16_t lk_speed_target;
} msg_gimbal_t;

typedef struct
{
	int16_t chassis_angle;
} msg_chassis_t;

typedef struct
{
	uint8_t flag;
	uint8_t temperature;
	int16_t iq;
	int16_t speed_dps;
	int16_t encoder;
} lk_recv_data_t;

typedef struct
{
 uint16_t ecd;
 int16_t speed_rpm;
 int16_t given_current;
 int16_t last_ecd;
 uint8_t temperate;
} rm_recv_data_t;


extern void msg_send_can1(mt_cmd_id_e mt_cmd_id, int16_t data1, int16_t data2, int16_t data3, int16_t data4);
extern void lk_send_can2(mt_cmd_id_e mt_cmd_id, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void rm_send_can2(mt_cmd_id_e mt_cmd_id, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

#endif
