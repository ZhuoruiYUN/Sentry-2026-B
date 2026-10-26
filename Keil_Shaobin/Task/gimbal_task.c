#include "gimbal_task.h"

Sliding yaw;
Sliding pitch;

extern usb_t usb;
extern VTM_Data_t key;
gimbal_t gimbal;

static void km_control(gimbal_t *gimbal);
static void rc_control(gimbal_t *gimbal);
static void pitch_constraint(gimbal_t *gimbal);
static void yaw_constraint(gimbal_t *gimbal);

#define RC_DEADBAND 20
#define MOUSE_DEADBAND 3
#define RC_CH_CENTER 1024.0f
#define RC_CH_RANGE 660.0f
#define RC_PITCH_ABS_MAX_DEG 32.0f

#define PITCH_RATE_LPF_ALPHA 0.10f
#define PITCH_HOLD_ERR_DEG 0.20f
#define PITCH_HOLD_RATE_DPS 2.5f
#define PITCH_CMD_LIMIT 7000.0f
#define PITCH_CMD_DEADZONE 90.0f
#define PITCH_CMD_SLEW_STEP 120.0f

static float abs_f32(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float clamp_f32(float x, float min_v, float max_v)
{
    if (x > max_v)
    {
        return max_v;
    }
    if (x < min_v)
    {
        return min_v;
    }
    return x;
}

static float slew_limit_f32(float target, float current, float step)
{
    float delta = target - current;
    if (delta > step)
    {
        delta = step;
    }
    else if (delta < -step)
    {
        delta = -step;
    }
    return current + delta;
}

static int16_t rc_delta_with_deadband(uint16_t ch)
{
    int16_t delta = (int16_t)ch - 1024;
    if ((delta > -RC_DEADBAND) && (delta < RC_DEADBAND))
    {
        return 0;
    }
    return delta;
}

static int16_t mouse_delta_with_deadband(int16_t v)
{
    if ((v > -MOUSE_DEADBAND) && (v < MOUSE_DEADBAND))
    {
        return 0;
    }
    return v;
}

void GIMBAL_TASK(void const *pvParameters)
{
    uint32_t xLastWakeTime = osKernelSysTick();
    const portTickType xFrequency = 1;

    SMC_Init(&pitch);
    SMC_Init(&yaw);

    gimbal.target_yaw = 0.0f;
    gimbal.target_pitch = 0.0f;

    SMC_SetParam_Exponent(&yaw, 0.8f, 120.0f, 15.0f, 0.8f, 2.0f, 16384.0f, 0.0f);
	SMC_SetParam_EISMC(&pitch, 1.0f, 90.0f, 10.0f, 2.5f, 12.0f, 1.0f, 12000.0f, 0.0f);

	osDelay(500);

    /* 启动时将目标角锁到当前姿态，避免追 0 引起抖动 */
    gimbal.target_yaw = mahony.YAW;
    gimbal.target_pitch = mahony.PITCH;
    SMC_IntegralClear(&yaw);
    SMC_IntegralClear(&pitch);

    float pitch_rate_dps_lpf = 0.0f;
    float yaw_rate_dps_lpf = 0.0f;
    float pitch_cmd_last = 0.0f;

    uint8_t last_sw = key.sw;
    while (1)
    {   
            if (key.sw != last_sw)
            {
                last_sw = key.sw;
                gimbal.target_yaw = mahony.YAW;
                gimbal.target_pitch = mahony.PITCH;
                SMC_IntegralClear(&yaw);
                SMC_IntegralClear(&pitch);
            }

			if (key.sw == 1)
			{
				km_control(&gimbal);
			}
			else if (key.sw == 0)
			{
				rc_control(&gimbal);
			}
				
      zerocheckangle(&gimbal.target_yaw, &mahony.YAW);
			pitch_constraint(&gimbal);
			yaw_constraint(&gimbal);

        float pitch_rate_dps = mahony.g_smc[1] * 57.2958f;
        float yaw_rate_dps = mahony.g_smc[2] * 57.2958f;
        pitch_rate_dps_lpf += PITCH_RATE_LPF_ALPHA * (pitch_rate_dps - pitch_rate_dps_lpf);
        yaw_rate_dps_lpf += PITCH_RATE_LPF_ALPHA * (yaw_rate_dps - yaw_rate_dps_lpf);

        SMC_ErrorUpdate(&yaw, gimbal.target_yaw, mahony.YAW, yaw_rate_dps_lpf);
			SMC_ErrorUpdate(&pitch, gimbal.target_pitch, mahony.PITCH, pitch_rate_dps_lpf);
        gimbal.yaw_output = SMC_Calculate(&yaw);
			gimbal.pitch_output = SMC_Calculate(&pitch);

        gimbal.pitch_output = -1.0f * gimbal.pitch_output;

        if ((abs_f32(gimbal.target_pitch - mahony.PITCH) < PITCH_HOLD_ERR_DEG)
            && (abs_f32(pitch_rate_dps_lpf) < PITCH_HOLD_RATE_DPS))
        {
            gimbal.pitch_output = 0.0f;
            SMC_IntegralClear(&pitch);
        }

        gimbal.pitch_output = clamp_f32(gimbal.pitch_output, -PITCH_CMD_LIMIT, PITCH_CMD_LIMIT);
        gimbal.pitch_output = slew_limit_f32(gimbal.pitch_output, pitch_cmd_last, PITCH_CMD_SLEW_STEP);
        pitch_cmd_last = gimbal.pitch_output;

        if (abs_f32(gimbal.pitch_output) < PITCH_CMD_DEADZONE)
        {
            gimbal.pitch_output = 0.0f;
        }
			
			if (calibrate.flag != 1)
			{
        CAN_cmd_pitch((int16_t)gimbal.pitch_output);
        CAN_cmd_yaw((int16_t)gimbal.yaw_output);
			}
			else if (calibrate.flag == 1)
			{
				CAN_cmd_pitch(0);
				CAN_cmd_shoot(0,0,0);
                gimbal.target_yaw = mahony.YAW;
                gimbal.target_pitch = mahony.PITCH;
                SMC_IntegralClear(&yaw);
                SMC_IntegralClear(&pitch);
                pitch_cmd_last = 0.0f;
			}
        osDelayUntil(&xLastWakeTime,xFrequency);
    }
}

static void km_control(gimbal_t *gimbal)
{
    if (key.mouse_btn_r == 0)
    {
        int16_t mx = mouse_delta_with_deadband(key.mouse_x);
        int16_t my = mouse_delta_with_deadband(key.mouse_y);
        gimbal->target_yaw += MOUSE_X_MAP * mx * 0.001f;
        gimbal->target_pitch += MOUSE_Y_MAP * my * 0.001f;
    }
    else if (key.mouse_btn_r == 1)
    {
        gimbal->target_yaw +=  usb.deltaYaw;
        gimbal->target_pitch +=  usb.deltaPitch;
    }
	usb.deltaPitch = 0.0f;
	usb.deltaYaw = 0.0f;
}

static void rc_control(gimbal_t *gimbal)
{
    int16_t ch3_delta = rc_delta_with_deadband(key.ch3);
    float pitch_axis = ((float)key.ch2 - RC_CH_CENTER) / RC_CH_RANGE;
    gimbal->target_yaw += MOUSE_X_MAP * ch3_delta * 0.0001f;
    gimbal->target_pitch = clamp_f32(MOUSE_Y_MAP * pitch_axis * RC_PITCH_ABS_MAX_DEG,
                                     -RC_PITCH_ABS_MAX_DEG,
                                     RC_PITCH_ABS_MAX_DEG);
}

static void pitch_constraint(gimbal_t *gimbal)
{
    gimbal->target_pitch = (gimbal->target_pitch > 32.0f) ? 32.0f : 
                           (gimbal->target_pitch < -32.0f) ? -32.0f : 
                           gimbal->target_pitch;
}

static void yaw_constraint(gimbal_t *gimbal)
{
    gimbal->target_yaw = (gimbal->target_yaw > 180.0f) ? 180.0f : 
                         (gimbal->target_yaw < -180.0f) ? -180.0f : 
                         gimbal->target_yaw;
}
