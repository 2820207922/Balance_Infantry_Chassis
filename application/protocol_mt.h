#ifndef PROTOCOL_MT_H
#define PROTOCOL_MT_H

#include "struct_typedef.h"
#include "main.h"

#define RM_3508_SPEED_MAX 4000

typedef enum
{
		LK_SINGLE_ID = 0x140,
		LK_MULTIPLE_ID = 0x280,
	
		RM_C620_L_ID = 0x200,
		RM_C620_H_ID = 0x1FF
	
} mt_cmd_id_e;

typedef struct
{
	uint8_t flag;
	uint8_t temperature;
	int16_t iq;
	int16_t speed;
	int16_t encoder;
} lk_recv_data_t;

typedef struct
{
 uint16_t ecd;
 int16_t speed_rpm;
 int16_t given_current;
 uint8_t temperate;
 int16_t last_ecd;
} rm_recv_data_t;


extern void mt_send(mt_cmd_id_e mt_cmd_id, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern const lk_recv_data_t *get_lk_recv_data(uint8_t motor_id);
extern const rm_recv_data_t *get_rm_recv_data(uint8_t motor_id);

#endif
