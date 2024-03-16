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

#define death_limit(a, b) a = (a > 0 && a < b) ? 0 : ((a < 0 && a > -b) ? 0 : a)

#define SPEED_INPUT_MAX 3800

//3508 motor speed PID
#define M3505_MOTOR_SPEED_PID_KP 50.0f
#define M3505_MOTOR_SPEED_PID_KI 0.0f
#define M3505_MOTOR_SPEED_PID_KD 0.5f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 1200.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 120.0f

//lk motor position PID
#define M9025_MOTOR_POSITION_PID_KP 0.1f
#define M9025_MOTOR_POSITION_PID_KI 0.0f
#define M9025_MOTOR_POSITION_PID_KD 0.0f
#define M9025_MOTOR_POSITION_PID_MAX_OUT 180.0f
#define M9025_MOTOR_POSITION_PID_MAX_IOUT 18.0f

//lk motor speed PID
#define M9025_MOTOR_SPEED_PID_KP 300.0f
#define M9025_MOTOR_SPEED_PID_KI 3.0f
#define M9025_MOTOR_SPEED_PID_KD 1.0f
#define M9025_MOTOR_SPEED_PID_MAX_OUT 1000.0f
#define M9025_MOTOR_SPEED_PID_MAX_IOUT 10.0f

#ifdef MECANUM_WHEEL
const uint8_t rm_3508_id_order[4] = {2, 3, 1, 0};
#endif

#ifdef OMNI_WHEEL
const uint8_t rm_3508_id_order[4] = {0, 1, 2, 3};
#endif

const fp32 LK_9025_POSITION_PID[3] = {M9025_MOTOR_POSITION_PID_KP, M9025_MOTOR_POSITION_PID_KI, M9025_MOTOR_POSITION_PID_KD};
const fp32 LK_9025_SPEED_PID[3] = {M9025_MOTOR_SPEED_PID_KP, M9025_MOTOR_SPEED_PID_KI, M9025_MOTOR_SPEED_PID_KD};
const fp32 RM_3508_SPEED_PID[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};

static fp32 speed_target[4], speed_current[4];
static fp32 angle_target, angle_current, vel_target, vel_current;

extern lk_recv_data_t lk_recv_data[4];
extern rm_recv_data_t rm_recv_data[8];
extern rc_data_t rc_data;
extern msg_agx2c_t msg_agx2c;

pid_type_def pid_3508_v[4], pid_9025_v[1], pid_9025_p[1];


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
		speed_current[rm_3508_id_order[i]] = 1580.0 * rm_recv_data[rm_3508_id_order[i]].speed_rpm / (60.0 * 19);
	}
}

void rm_3508_task(void)
{
	// calc_speed_target(-rc_data.rc.ch[3] * SPEED_INPUT_MAX / 660, -rc_data.rc.ch[2] * SPEED_INPUT_MAX / 660, -rc_data.rc.ch[0] * SPEED_INPUT_MAX / 660);
	calc_speed_target(-msg_agx2c.vel[0] * SPEED_INPUT_MAX, msg_agx2c.vel[1] * SPEED_INPUT_MAX, msg_agx2c.vel[2] * SPEED_INPUT_MAX);
	
	for (int i = 0; i < 4; ++i)
	{
		PID_calc(&pid_3508_v[rm_3508_id_order[i]], speed_current[rm_3508_id_order[i]], speed_target[rm_3508_id_order[i]]);
		death_limit(pid_3508_v[rm_3508_id_order[i]].out, 100);
	}
	
	// mt_send(RM_C620_L_ID, 0, 0, pid_3508[2].out, 0);
	rm_send_can2(RM_C620_L_ID, pid_3508_v[0].out, pid_3508_v[1].out, pid_3508_v[2].out, pid_3508_v[3].out);
	
	// usart_printf("$%f %f %f %f;", speed_current[rm_3508_id_order[0]], speed_current[rm_3508_id_order[1]], speed_current[rm_3508_id_order[2]], speed_current[rm_3508_id_order[3]]);
	// usart_printf("$%f %f;", speed_target[rm_3508_id_order[0]], speed_current[rm_3508_id_order[0]]);
	calc_speed_current();
}

fp32 calc_angle_target(fp32 ref, fp32 set)
{
	fp32 err = set - ref;
	while (err > 180.0)
	{
		set -= 360.0;
		err = set - ref;
	}
	while (err <= -180.0)
	{
		set += 360.0;
		err = set - ref;
	}
	
	return set;
}

void lk_9025_task(void)
{
	for (int i = 0; i < 1; ++i)
	{
		PID_calc(&pid_9025_p[i], angle_current, calc_angle_target(angle_current, angle_target));
		PID_calc(&pid_9025_v[i], vel_current, pid_9025_p[i].out);
		death_limit(pid_9025_v[i].out, 20);
	}
	
	lk_send_can2(LK_MULTIPLE_ID, pid_9025_v[0].out, 0, 0, 0);
	
	vel_current = lk_recv_data[0].speed_dps / 360.0;
	angle_current = lk_recv_data[0].encoder * 360.0 / 65536.0;
	// usart_printf("$%f %f %f;",angle_target, angle_current, pid_9025_p[0].out);
}

void lk_9025_init(void)
{
	for(int i = 0; i < 1; ++i)
	{
		PID_init(&pid_9025_p[i], PID_POSITION, LK_9025_POSITION_PID, M9025_MOTOR_POSITION_PID_MAX_OUT, M9025_MOTOR_POSITION_PID_MAX_IOUT);
		PID_init(&pid_9025_v[i], PID_POSITION, LK_9025_SPEED_PID, M9025_MOTOR_SPEED_PID_MAX_OUT, M9025_MOTOR_SPEED_PID_MAX_IOUT);
	}
}

void rm_3508_init(void)
{
	for(int i = 0; i < 4; ++i)
	{
		PID_init(&pid_3508_v[i], PID_POSITION, RM_3508_SPEED_PID, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	}
}

void motor_ctl_task(void const * argument)
{
	can_filter_init();
	
	lk_9025_init();
	rm_3508_init();
	
	int k = 0;
	while(1)
	{
		++k;
		if (k < 2500)
		{
			angle_target = -90;
		}
		else if (k >= 2500 && k < 5000)
		{
			angle_target = 180;
		}
		else if (k >= 500 && k < 7500)
		{
			angle_target = 90;
		}
		else if (k >= 7500 && k < 10000)
		{
			angle_target = 0;
		}
		else
		{
			k = 0;
		}
		
		lk_9025_task();
		// rm_3508_task();
		
		vTaskDelay(1);
	}
}

