#include "motor_ctl_task.h"
#include "bsp_can.h"
#include "protocol_mt.h"
#include "protocol_rs.h"
#include "protocol_rc.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"

// #define MECANUM_WHEEL
#define OMNI_WHEEL

#define SPEED_INPUT_MAX 3.8

//3508 motor speed PID
#define M3505_MOTOR_SPEED_PID_KP 50000.0f
#define M3505_MOTOR_SPEED_PID_KI 0.0f
#define M3505_MOTOR_SPEED_PID_KD 500.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 1200.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 120.0f


const fp32 RM_3508_PID[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};

#ifdef MECANUM_WHEEL
const uint8_t rm_3508_id_order[4] = {2, 3, 1, 0};
#endif

#ifdef OMNI_WHEEL
const uint8_t rm_3508_id_order[4] = {0, 1, 2, 3};
#endif

pid_type_def pid_3508[4];

extern rm_recv_data_t rm_recv_data[8];
extern rc_data_t rc_data;
extern msg_agx2c_t msg_agx2c;

static fp32 speed_target[4], speed_current[4];

void calc_speed_target(fp32 vx, fp32 vy, fp32 vw)
{
	#ifdef MECANUM_WHEEL
	speed_target[rm_3508_id_order[0]] = 0.3535 * ( vx + vy + vw);
	speed_target[rm_3508_id_order[1]] = 0.3535 * ( vx - vy - vw);
	speed_target[rm_3508_id_order[2]] = 0.3535 * (-vx - vy - vw);
	speed_target[rm_3508_id_order[3]] = 0.3535 * (-vx + vy + vw);
	#endif
	
	#ifdef OMNI_WHEEL
	speed_target[rm_3508_id_order[0]] = 0.3535 * (-vx - vy - vw);
	speed_target[rm_3508_id_order[1]] = 0.3535 * (-vx + vy - vw);
	speed_target[rm_3508_id_order[2]] = 0.3535 * ( vx + vy - vw);
	speed_target[rm_3508_id_order[3]] = 0.3535 * ( vx - vy - vw);
	#endif
}

void calc_speed_current()
{
	for (int i = 0; i < 4; ++i)
	{
		speed_current[rm_3508_id_order[i]] = 0.158 * rm_recv_data[rm_3508_id_order[i]].speed_rpm / (60.0 * 19);
	}
}

void motor_ctl_task(void const * argument)
{
	can_filter_init();
	for(int i = 0; i < 4; ++i)
	{
		PID_init(&pid_3508[i], PID_POSITION, RM_3508_PID, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	}
	// int k = 0;
	while(1)
	{
//		++k;
//		if (k < 2000)
//		{
//			calc_speed_target(3.8, 0, 0);
//		}
//		else if (k >= 2000 && k < 4000)
//		{
//			calc_speed_target(-3.8, 0, 0);
//		}
//		else
//		{
//			k = 0;
//		}
		
		// calc_speed_target(-rc_data.rc.ch[3] * SPEED_INPUT_MAX / 660, -rc_data.rc.ch[2] * SPEED_INPUT_MAX / 660, -rc_data.rc.ch[0] * SPEED_INPUT_MAX / 660);
		calc_speed_target(-msg_agx2c.vel[0] * SPEED_INPUT_MAX, msg_agx2c.vel[1] * SPEED_INPUT_MAX, msg_agx2c.vel[2] * SPEED_INPUT_MAX);
		
		for (int i = 0; i < 4; ++i)
		{
			PID_calc(&pid_3508[rm_3508_id_order[i]], speed_current[rm_3508_id_order[i]], speed_target[rm_3508_id_order[i]]);
		}
		
		// mt_send(RM_C620_L_ID, 0, 0, pid_3508[2].out, 0);
		mt_send(RM_C620_L_ID, pid_3508[0].out, pid_3508[1].out, pid_3508[2].out, pid_3508[3].out);
		
		calc_speed_current();
		// usart_printf("$%f %f %f %f;", speed_current[rm_3508_id_order[0]], speed_current[rm_3508_id_order[1]], speed_current[rm_3508_id_order[2]], speed_current[rm_3508_id_order[3]]);
		// usart_printf("$%f %f;", speed_target[rm_3508_id_order[0]], speed_current[rm_3508_id_order[0]]);
		vTaskDelay(1);
	}
}

