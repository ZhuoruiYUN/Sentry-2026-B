#include "chassis_task.h"

PID_t chassis[4];
PID_t follow_pid;
chassis_motion_value_t chassis_motion;
extern VTM_Data_t key;

static uint8_t chassis_follow_enabled = 0;
static uint8_t last_key_f = 0;

static void theta_calculate(chassis_motion_value_t *chassis_motion);
static void output_map(float *value);
static void km_control(void);
static void chassis_wheel_inverse_resolution(chassis_motion_value_t *chassis_motion);
static void gimbal_coordinate_resolution(float theta, chassis_motion_value_t *chassis_motion);
static void rc_control(void);

#define CHASSIS_RC_DEADBAND 20
#define CHASSIS_ZERO_EPS_V  0.02f
#define CHASSIS_ZERO_EPS_W  0.05f

/* 底盘跟随 PID 参数（theta 单位：度，输出单位：rad/s）
 * 如果跟随方向反了，将 FOLLOW_KP 改为负值 */
#define FOLLOW_KP        0.05f
#define FOLLOW_KI        0.002f
#define FOLLOW_OUT_MAX   6.0f
#define FOLLOW_DEAD_ZONE 2.0f

static int16_t rc_delta_with_deadband(uint16_t ch)
{
	int16_t delta = (int16_t)ch - 1024;
	if ((delta > -CHASSIS_RC_DEADBAND) && (delta < CHASSIS_RC_DEADBAND))
	{
		return 0;
	}
	return delta;
}

void CHASSIS_TASK(void const * argument)
{
    uint32_t xLastWakeTime = osKernelSysTick();
    const portTickType xFrequency = 1;
    PID_Clear_Error(&chassis[0]);
    PID_Clear_Error(&chassis[1]);
    PID_Clear_Error(&chassis[2]);
    PID_Clear_Error(&chassis[3]);

    PID_Init(&chassis[0], 0.95f, 0.1f, 0.0f, 0.0f, 10.0f, 10.0f, 0.0f);
    PID_Init(&chassis[1], 0.95f, 0.1f, 0.0f, 0.0f, 10.0f, 10.0f, 0.0f);
    PID_Init(&chassis[2], 0.95f, 0.1f, 0.0f, 0.0f, 10.0f, 10.0f, 0.0f);
    PID_Init(&chassis[3], 0.95f, 0.1f, 0.0f, 0.0f, 10.0f, 10.0f, 0.0f);
    PID_Init(&follow_pid, FOLLOW_KP, FOLLOW_KI, 0.0f, 0.0f, 1.0f, FOLLOW_OUT_MAX, FOLLOW_DEAD_ZONE);

	  chassis_motion.gimbal_target_vx = 0.0f;
		chassis_motion.gimbal_target_vy = 0.0f;
		chassis_motion.chassis_target_omega = 0.0f;
    while(1)
    {
					chassis_motion.now_ecd = get_yaw_gimbal_motor_measure_point()->ecd;
					theta_calculate(&chassis_motion);
					
					for (int i = 0; i < 4; i++)
					{
							chassis_motion.wheel_now_omega[i] = get_chassis_motor_measure_point(i)->speed_rpm * 0.1047f;
					}
					if (key.sw == 1)
					{
						km_control();
					}
					else if (key.sw == 0)
					{
						rc_control();
					}

					/* 底盘跟随：当跟随使能且未处于小陀螺模式时，用 theta PID 控制底盘旋转 */
					uint8_t spinning = (key.sw == 1) ? key.key_shift : (key.pause != 0u);
					if (chassis_follow_enabled && !spinning)
					{
						float follow_omega;
						PID_Set_Values(&follow_pid, chassis_motion.theta, 0.0f);
						PID_Calculate(&follow_pid, &follow_omega);
						chassis_motion.chassis_target_omega = follow_omega;
					}

					gimbal_coordinate_resolution(chassis_motion.theta, &chassis_motion);
			if ((fabsf(chassis_motion.chassis_target_vx) < CHASSIS_ZERO_EPS_V)
			    && (fabsf(chassis_motion.chassis_target_vy) < CHASSIS_ZERO_EPS_V)
			    && (fabsf(chassis_motion.chassis_target_omega) < CHASSIS_ZERO_EPS_W))
			{
				chassis_motion.chassis_target_vx = 0.0f;
				chassis_motion.chassis_target_vy = 0.0f;
				chassis_motion.chassis_target_omega = 0.0f;
				CAN_cmd_chassis(0, 0, 0, 0);
			}

			else
			{
				PID_Set_Values(&chassis[0], chassis_motion.wheel_target_omega[0], chassis_motion.wheel_now_omega[0]);
				PID_Set_Values(&chassis[1], chassis_motion.wheel_target_omega[1], chassis_motion.wheel_now_omega[1]);
				PID_Set_Values(&chassis[2], chassis_motion.wheel_target_omega[2], chassis_motion.wheel_now_omega[2]);
				PID_Set_Values(&chassis[3], chassis_motion.wheel_target_omega[3], chassis_motion.wheel_now_omega[3]);

				PID_Calculate(&chassis[0], &chassis_motion.motor_output_value[0]);
				PID_Calculate(&chassis[1], &chassis_motion.motor_output_value[1]);
				PID_Calculate(&chassis[2], &chassis_motion.motor_output_value[2]);
				PID_Calculate(&chassis[3], &chassis_motion.motor_output_value[3]);

				output_map(&chassis_motion.motor_output_value[0]);
				output_map(&chassis_motion.motor_output_value[1]);
				output_map(&chassis_motion.motor_output_value[2]);
				output_map(&chassis_motion.motor_output_value[3]);
					CAN_cmd_chassis((int16_t)chassis_motion.motor_output_value[0], (int16_t)chassis_motion.motor_output_value[1], 
								(int16_t)chassis_motion.motor_output_value[2], (int16_t)chassis_motion.motor_output_value[3]);
			}
				osDelayUntil(&xLastWakeTime,xFrequency);
		}
			    
}

static void gimbal_coordinate_resolution(float theta, chassis_motion_value_t *chassis_motion)
{   
    float32_t sin_theta, cos_theta;
    arm_sin_cos_f32(theta, &sin_theta, &cos_theta); 
    chassis_motion->chassis_target_vx = cos_theta * chassis_motion->gimbal_target_vx - sin_theta * chassis_motion->gimbal_target_vy;
    chassis_motion->chassis_target_vy = sin_theta * chassis_motion->gimbal_target_vx + cos_theta * chassis_motion->gimbal_target_vy;
    chassis_wheel_inverse_resolution(chassis_motion);
}

static void chassis_wheel_inverse_resolution(chassis_motion_value_t *chassis_motion)
{
    chassis_motion->wheel_target_omega[0] = GEAR_RATIO * (0.707f * chassis_motion->chassis_target_vx - 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //1
    chassis_motion->wheel_target_omega[1] = GEAR_RATIO * (0.707f * chassis_motion->chassis_target_vx + 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //2
    chassis_motion->wheel_target_omega[2] = GEAR_RATIO * (-0.707f * chassis_motion->chassis_target_vx + 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //3
    chassis_motion->wheel_target_omega[3] = GEAR_RATIO * (-0.707f * chassis_motion->chassis_target_vx - 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //4
}

static void theta_calculate(chassis_motion_value_t *chassis_motion)
{
    chassis_motion->theta = (chassis_motion->now_ecd - 6128.0f) * 360.0f / 8192.0f;
}

static void output_map(float *value)
{
    *value *= 819.2f; 
}

static void km_control(void)
{
	chassis_motion.gimbal_target_vx = (key.key_w - key.key_s) * 1.5f;
	chassis_motion.gimbal_target_vy = (key.key_a - key.key_d) * 1.5f;

	/* F 键按下瞬间切换底盘跟随开关 */
	if (key.key_f && !last_key_f)
	{
		chassis_follow_enabled ^= 1u;
		PID_Clear_Error(&follow_pid);
	}
	last_key_f = key.key_f;

	if (key.key_shift)
	{
		chassis_motion.chassis_target_omega = 2 * PI; /* 小陀螺优先 */
	}
	else if (!chassis_follow_enabled)
	{
		chassis_motion.chassis_target_omega = 0.0f;
	}
	/* 跟随模式下 omega 由主循环中的 follow_pid 写入，此处不赋值 */
}

static void rc_control(void)
{
	int16_t ch0_delta = rc_delta_with_deadband(key.ch0);
	int16_t ch1_delta = rc_delta_with_deadband(key.ch1);

	chassis_motion.gimbal_target_vx = ch0_delta / 660.0f;
	chassis_motion.gimbal_target_vy = -ch1_delta / 660.0f;

	/* ch9（四拨杆）电平直读：下拨 btn_l=1 开跟随，上拨 btn_l=0 关跟随 */
	if (chassis_follow_enabled != key.btn_l)
	{
		chassis_follow_enabled = key.btn_l;
		if (!chassis_follow_enabled)
		{
			PID_Clear_Error(&follow_pid);
		}
	}

	if (key.pause != 0u)
	{
		chassis_motion.chassis_target_omega = 2.0f * PI; /* 小陀螺优先 */
	}
	else if (!chassis_follow_enabled)
	{
		chassis_motion.chassis_target_omega = 0.0f;
	}
	/* 跟随模式下 omega 由主循环中的 follow_pid 写入，此处不赋值 */
}