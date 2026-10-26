/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      SBUS remote control DMA + IDLE handling
  */

#include "remote_control.h"
#include "main.h"
#include "usart.h"
#include "control_task.h"

/* Debug: count RC IDLE events to help verify ISR firing */
volatile uint32_t rc_idle_count = 0;
volatile uint16_t rc_last_rx_len = 0;
volatile uint32_t rc_sbus_ok_count = 0;
volatile uint32_t rc_dbus_ok_count = 0;
volatile uint32_t rc_bad_len_count = 0;

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern VTM_Data_t key;

// remote control data
RC_ctrl_t rc_ctrl;

// receive buffers (double buffer)
uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/* Diagnostics */
volatile uint32_t rc_init_status = 0;  /* 0=not init, 1=failed DMA, 2=success */
volatile uint32_t rc_init_hal_status = 0xFFFFFFFFu;
static uint8_t rc_rx_buf_index = 0;

/* FS-i6接收机通道映射可能不同: 6/7/8/9 可按实测切换 */
#define RC_MODE_SW_CHANNEL_INDEX 6u

static uint8_t mode_sw_to_key_mode(int16_t sw_value)
{
    /* 当前接收机该通道符号与拨杆方向相反：上=1(键鼠), 下=0(遥控) */
    return (sw_value < 0) ? 1u : 0u;
}

static int16_t get_mode_sw_channel_value(const RC_ctrl_t *ctrl)
{
    if (ctrl == NULL)
    {
        return 0;
    }

    if (RC_MODE_SW_CHANNEL_INDEX < 10u)
    {
        return ctrl->rc.ch[RC_MODE_SW_CHANNEL_INDEX];
    }
    return 0;
}

static HAL_StatusTypeDef rc_start_dma_on_buffer(uint8_t buf_index)
{
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(&huart3, sbus_rx_buf[buf_index], SBUS_RX_BUF_NUM);
    rc_init_hal_status = (uint32_t)status;
    if (status == HAL_OK)
    {
        rc_init_status = 2;
    }
    return status;
}

/**
  * @brief remote control init
  */
void remote_control_init(void)
{
    (void)HAL_UART_AbortReceive(&huart3);
    (void)HAL_UART_DMAStop(&huart3);

    rc_rx_buf_index = 0;
    rc_init_status = 0;
    rc_init_hal_status = 0xFFFFFFFFu;
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    HAL_StatusTypeDef status = rc_start_dma_on_buffer(rc_rx_buf_index);
    if (status == HAL_OK)
    {
        rc_init_status = 2;  /* DMA started successfully */
    }
    else
    {
        rc_init_status = 1;  /* DMA failed */
    }
}

const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

/* Diagnostics: read USART3 status registers */
volatile uint32_t rc_usart3_sr = 0;    /* Status register */
volatile uint32_t rc_usart3_dr = 0;    /* Data register (last byte) */
volatile uint32_t rc_usart3_cr1 = 0;   /* Control register 1 */
volatile uint32_t rc_usart3_cr3 = 0;   /* Control register 3 (DMA enable) */

volatile uint8_t  rc_diag_trigger = 0; /* Set to 1 to snapshot registers */

void rc_snapshot_usart3_status(void)
{
    if (huart3.Instance != NULL)
    {
        rc_usart3_sr = huart3.Instance->SR;
        rc_usart3_dr = huart3.Instance->DR;
        rc_usart3_cr1 = huart3.Instance->CR1;
        rc_usart3_cr3 = huart3.Instance->CR3;
    }
}

// static inline int16_t rc_switch_to_channel_value(char s)
// {
//     /* DJI 三段拨杆：UP=1, MID=3, DOWN=2；映射到 11bit 通道典型值 */
//     if (s == (char)RC_SW_UP) return (int16_t)RC_CH_VALUE_MAX;
//     if (s == (char)RC_SW_DOWN) return (int16_t)RC_CH_VALUE_MIN;
//     return (int16_t)RC_CH_VALUE_OFFSET;
// }

static void dbus_to_rc(volatile const uint8_t *dbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (dbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    /* 先给所有通道一个安全默认值（防止主控用到未解出的通道时一直为 0） */
    for (uint8_t i = 0; i < 10; i++)
    {
        rc_ctrl->rc.ch[i] = (int16_t)RC_CH_VALUE_OFFSET;
    }
		/* 拨杆从左往右分别为 1 2 3 4 */
	    rc_ctrl->rc.ch[0] = (((dbus_buf[1] | dbus_buf[2] << 8) & 0x07FF) - Rocker_Mid); // 右侧摇杆左右
		rc_ctrl->rc.ch[1] = (((dbus_buf[2] >> 3 | dbus_buf[3] << 5) & 0x07FF) - Rocker_Mid); // 右侧摇杆上下
		rc_ctrl->rc.ch[2] = (((dbus_buf[3] >> 6 | dbus_buf[4] << 2 | dbus_buf[5] << 10) & 0x07FF) - Rocker_Mid); // 左侧摇杆上下
		rc_ctrl->rc.ch[3] = (((dbus_buf[5] >> 1 | dbus_buf[6] << 7) & 0x07FF) - Rocker_Mid); // 左侧摇杆左右
		rc_ctrl->rc.ch[4] = (((dbus_buf[6] >> 4 | dbus_buf[7] << 4) & 0x07FF) - Rocker_Mid); // 左侧旋钮
		rc_ctrl->rc.ch[5] = (((dbus_buf[7] >> 7 | dbus_buf[8] << 1 | dbus_buf[9] << 9) & 0x07FF) - Rocker_Mid); // 右侧旋钮
		rc_ctrl->rc.ch[6] = (((dbus_buf[9] >> 2 | dbus_buf[10] << 6) & 0x07FF) - Rocker_Mid) / Rocker_Delt_Range; // 一拨杆
		rc_ctrl->rc.ch[7] = (((dbus_buf[10] >> 5 | dbus_buf[11] << 3) & 0x07FF) - Rocker_Mid) / Rocker_Delt_Range; // 二拨杆
		rc_ctrl->rc.ch[8] = (((dbus_buf[12] | dbus_buf[13] << 8) & 0x07FF) - Rocker_Mid) / Rocker_Delt_Range;  // 三拨杆
		rc_ctrl->rc.ch[9] = (((dbus_buf[13] >> 3 | dbus_buf[14] << 5) & 0x07FF) - Rocker_Mid) / Rocker_Delt_Range; // 四拨杆
}

 static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
    /* 先给所有通道一个安全默认值（防止主控用到未解出的通道时一直为 0） */
    for (uint8_t i = 0; i < 10; i++)
    {
        rc_ctrl->rc.ch[i] = (int16_t)RC_CH_VALUE_OFFSET;
    }
		/* 拨杆从左往右分别为 1 2 3 4 */
	    rc_ctrl->rc.ch[0] = (((sbus_buf[1] | sbus_buf[2] << 8) & 0x07FF) - Rocker_Mid); /// 右侧摇杆左右
		rc_ctrl->rc.ch[1] = (((sbus_buf[2] >> 3 | sbus_buf[3] << 5) & 0x07FF) - Rocker_Mid); // 右侧摇杆上下
		rc_ctrl->rc.ch[2] = (((sbus_buf[3] >> 6 | sbus_buf[4] << 2 | sbus_buf[5] << 10) & 0x07FF) - Rocker_Mid); // 左侧摇杆上下
		rc_ctrl->rc.ch[3] = (((sbus_buf[5] >> 1 | sbus_buf[6] << 7) & 0x07FF) - Rocker_Mid); // 左侧摇杆左右
		rc_ctrl->rc.ch[4] = (((sbus_buf[6] >> 4 | sbus_buf[7] << 4) & 0x07FF) - Rocker_Mid); // 左侧旋钮
		rc_ctrl->rc.ch[5] = (((sbus_buf[7] >> 7 | sbus_buf[8] << 1 | sbus_buf[9] << 9) & 0x07FF) - Rocker_Mid); // 右侧旋钮
		rc_ctrl->rc.ch[6] = (((sbus_buf[9] >> 2 | sbus_buf[10] << 6) & 0x07FF) - Rocker_Mid) / Rocker_Delt_Range; // 一拨杆
		rc_ctrl->rc.ch[7] = (((sbus_buf[10] >> 5 | sbus_buf[11] << 3) & 0x07FF) - Rocker_Mid) / Rocker_Delt_Range; // 二拨杆
		rc_ctrl->rc.ch[8] = (((sbus_buf[12] | sbus_buf[13] << 8) & 0x07FF) - Rocker_Mid) / Rocker_Delt_Range;  // 三拨杆
		rc_ctrl->rc.ch[9] = (((sbus_buf[13] >> 3 | sbus_buf[14] << 5) & 0x07FF) - Rocker_Mid) / Rocker_Delt_Range; // 四拨杆
}

void RC_USART3_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET)
    {
        uint16_t this_time_rx_len;
        uint8_t current_buf_index = rc_rx_buf_index;

        rc_idle_count++;
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);

        this_time_rx_len = (uint16_t)(SBUS_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx));
        rc_last_rx_len = this_time_rx_len;

        (void)HAL_UART_DMAStop(&huart3);

        if((this_time_rx_len >= RC_FRAME_LENGTH_SBUS_MIN) && (this_time_rx_len <= RC_FRAME_LENGTH_SBUS))
        {
            rc_sbus_ok_count++;
            sbus_to_rc(sbus_rx_buf[current_buf_index], &rc_ctrl);
            /* 同步遥控器数据到全局key结构体 */
            key.ch0 = (uint16_t)(rc_ctrl.rc.ch[0] + 1024);
            key.ch1 = (uint16_t)(rc_ctrl.rc.ch[1] + 1024);
            key.ch2 = (uint16_t)(rc_ctrl.rc.ch[2] + 1024);
            key.ch3 = (uint16_t)(rc_ctrl.rc.ch[3] + 1024);
            /* 模式切换通道可配置，默认使用 ch6 */
            key.sw = mode_sw_to_key_mode(get_mode_sw_channel_value(&rc_ctrl));
            /* 用ch7拨杆映射到pause */
            key.pause = (rc_ctrl.rc.ch[7] > 0) ? 1u : 0u;
            /* ch9（四拨杆）：上拨<0=关，下拨>0=开，映射到btn_l供底盘跟随使用 */
            key.btn_l = (rc_ctrl.rc.ch[9] > 0) ? 1u : 0u;
        }
        else if(this_time_rx_len == RC_FRAME_LENGTH_DBUS)
        {
            rc_dbus_ok_count++;
            dbus_to_rc(sbus_rx_buf[current_buf_index], &rc_ctrl);
            /* 同步遥控器数据到全局key结构体 */
            key.ch0 = (uint16_t)(rc_ctrl.rc.ch[0] + 1024);
            key.ch1 = (uint16_t)(rc_ctrl.rc.ch[1] + 1024);
            key.ch2 = (uint16_t)(rc_ctrl.rc.ch[2] + 1024);
            key.ch3 = (uint16_t)(rc_ctrl.rc.ch[3] + 1024);
            /* 模式切换通道可配置，默认使用 ch6 */
            key.sw = mode_sw_to_key_mode(get_mode_sw_channel_value(&rc_ctrl));
            /* 用ch7拨杆映射到pause */
            key.pause = (rc_ctrl.rc.ch[7] > 0) ? 1u : 0u;
            /* ch9（四拨杆）：上拨<0=关，下拨>0=开，映射到btn_l供底盘跟随使用 */
            key.btn_l = (rc_ctrl.rc.ch[9] > 0) ? 1u : 0u;
        }
        else
        {
            rc_bad_len_count++;
        }

        rc_rx_buf_index ^= 1u;
        rc_start_dma_on_buffer(rc_rx_buf_index);
        return;
    }

    HAL_UART_IRQHandler(&huart3);
}
