#include "shooter_task.h"
#include "remote_control.h"
#include <math.h>

PID_t shooter[3];
shooter_control_t shooter_control;
extern VTM_Data_t key;
extern usb_t usb;

static void km_control(VTM_Data_t *key, motion_t *motion, shooter_control_t *shooter_control);
static void rc_control(const RC_ctrl_t *rc, shooter_control_t *shooter_control);
static void output_map(float *value);

void SHOOTER_TASK(void const * argument)
{
    uint32_t xLastWakeTime = osKernelSysTick();
    const portTickType xFrequency = 1;

    PID_Init(&shooter[0], 0.15f, 0.3f, 0.002f, 0.0f, 10.0f, 10.0f, 0.0f);
    PID_Init(&shooter[1], 0.15f, 0.3f, 0.002f, 0.0f, 10.0f, 10.0f, 0.0f);
    PID_Init(&shooter[2], 0.1f, 0.02f, 0.0f, 0.0f, 60.0f, 10.0f, 0.0f);
		HeatPredict_Init(&hp, ROBOT_INFANTRY, INFANTRY_HEAT_MODE_BURST, 0.002f, 7000);
	/* Positive speed is the forward direction for the current trigger installation. */
	shooter_control.target_trigger_omega = 800.0f;
	shooter_control.desire_trigger_omega = 800.0f;
	shooter_control.max_trigger_omega = 1000.0f;

    while(1)
    {
            if (key.sw == 1u)
            {
                km_control(&key, &motion, &shooter_control);
            }
            else if (key.sw == 0u)
            {
                rc_control(get_remote_control_point(), &shooter_control);
            }
            else
            {
                shooter_control.shoot_flag = 0u;
            }
			
			shooter_control.friction_omega[0] = get_friction_motor_measure_point(0)->speed_rpm * 0.1047f;
      shooter_control.friction_omega[1] = get_friction_motor_measure_point(1)->speed_rpm * 0.1047f;
      shooter_control.trigger_omega = get_trigger_motor_measure_point()->speed_rpm / 6.0f;
			shooter_control.trigger_current = get_trigger_motor_measure_point()->given_current;
			
			HeatPredict_Update(&hp, &referee, shooter_control.trigger_current, shooter_control.trigger_omega, shooter_control.target_trigger_omega);
            // 旧实现：直接把负转速传入“发/秒”接口，函数内部会把负值钳为 0，导致无输出。
            // shooter_control.target_trigger_omega = HeatPredict_ComputeSafeFireRate(&hp, shooter_control.desire_trigger_omega, shooter_control.max_trigger_omega);

            // 新实现：转速(单位约为 rpm) -> 发速(rps) 进行热控限幅，再换回带方向的目标转速。
            {
                float desire_fire_rate = fabsf(shooter_control.desire_trigger_omega) / 60.0f;
                float max_fire_rate = fabsf(shooter_control.max_trigger_omega) / 60.0f;
                float safe_fire_rate = HeatPredict_ComputeSafeFireRate(&hp, desire_fire_rate, max_fire_rate);
                float direction = (shooter_control.desire_trigger_omega < 0.0f) ? -1.0f : 1.0f;
                shooter_control.target_trigger_omega = direction * safe_fire_rate * 60.0f;
            }
			
        if (shooter_control.shoot_flag == 1)
        {
            PID_Set_Values(&shooter[0], -600.0f, shooter_control.friction_omega[0]);
            PID_Set_Values(&shooter[1], 600.0f, shooter_control.friction_omega[1]);
            PID_Set_Values(&shooter[2], shooter_control.target_trigger_omega, shooter_control.trigger_omega);

            PID_Calculate(&shooter[0], &shooter_control.friction_output[0]);
            PID_Calculate(&shooter[1], &shooter_control.friction_output[1]);
            PID_Calculate(&shooter[2], &shooter_control.trigger_output);
			
						output_map(&shooter_control.friction_output[0]);
						output_map(&shooter_control.friction_output[1]);
						output_map(&shooter_control.trigger_output);

            CAN_cmd_shoot((int16_t)shooter_control.friction_output[0], (int16_t)shooter_control.friction_output[1], (int16_t)shooter_control.trigger_output);
        }
        else if (shooter_control.shoot_flag == 0)
        {
            CAN_cmd_shoot(0, 0, 0);
        }
				
        osDelayUntil(&xLastWakeTime,xFrequency);
    }
}

static void output_map(float *value)
{
    *value *= 819.2f; 
}

static void km_control(VTM_Data_t *key, motion_t *motion, shooter_control_t *shooter_control)
{
    shooter_control->shoot_flag = (key->mouse_btn_r == 0) ? key->mouse_btn_l : motion->auto_fire_detect;
}

#define RC_SINGLE_SHOT_HOLD_TICKS 200u

static int8_t rc_get_swc_pos(int16_t ch)
{
    if (ch > 0)
    {
        return 1;
    }
    if (ch < 0)
    {
        return -1;
    }
    return 0;
}

static void rc_control(const RC_ctrl_t *rc, shooter_control_t *shooter_control)
{
    static int8_t last_pos = 0;
    static uint16_t single_hold = 0u;
    int8_t pos = 0;

    if (rc != NULL)
    {
        pos = rc_get_swc_pos(rc->rc.ch[8]);
    }

    if (pos < 0)
    {
        single_hold = 0u;
        shooter_control->shoot_flag = 1u;
    }
    else
    {
        if ((pos > 0) && (last_pos == 0))
        {
            single_hold = RC_SINGLE_SHOT_HOLD_TICKS;
        }

        if (single_hold > 0u)
        {
            shooter_control->shoot_flag = 1u;
            single_hold--;
        }
        else
        {
            shooter_control->shoot_flag = 0u;
        }
    }

    last_pos = pos;
}

