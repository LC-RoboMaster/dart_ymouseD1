//
// Created by 13466 on 2026/9/25.
//
// 维特智能 JY901S 姿态模块驱动(USART1:PB6=TX / PB7=RX,模块 TXD 接 PB7)
//
// 模块主动推送数据帧,波特率 9600(与 CubeMX 中 USART1 配置一致):
//  - 帧长 11 字节:0x55 | 类型 | 8×数据 | 校验和(前 10 字节之和的低 8 位)
//  - 类型:0x51 加速度+温度,0x52 角速度+温度,0x53 角度(数据末字段为版本,忽略)
//  - 换算:加速度 = raw/32768×16g,角速度 = raw/32768×2000°/s,
//          角度 = raw/32768×180°,温度 = raw/100℃
//  - 若在上位机改过模块量程/波特率/输出内容,需同步修改下方换算系数与 CubeMX。
//
// 可靠性:
//  - 逐字节中断接收 + 状态机解析,失步自动从下一个 0x55 重新同步;
//  - 帧流超时(200ms 无有效帧)置离线;接收被错误中止时自动重新挂起。
#include "cmsis_os2.h"
#include "main.h"
#include "imu.h"
#include "usart.h"

/* 数据帧定义 */
#define JY901S_FRAME_HEAD   0x55U
#define JY901S_FRAME_LEN    11U
#define JY901S_PKT_ACC      0x51U
#define JY901S_PKT_GYRO     0x52U
#define JY901S_PKT_ANGLE    0x53U

/* 满量程换算系数(模块默认量程) */
#define JY901S_ACC_FS_G     16.0f
#define JY901S_GYRO_FS      2000.0f
#define JY901S_ANGLE_FS     180.0f
#define JY901S_TEMP_SCALE   100.0f

/* 帧流超时:超过该时长无有效帧判离线 */
#define IMU_FRAME_TIMEOUT_MS 200U
/* 在线状态检查周期 */
#define IMU_TASK_PERIOD_MS   100U

imu_data_t g_imu;

static uint8_t s_rx_byte;                       /* 单字节接收缓冲 */
static uint8_t s_frame[JY901S_FRAME_LEN];       /* 组帧缓冲 */
static uint8_t s_frame_idx;                     /* 当前组帧位置 */
static uint32_t s_last_frame_tick;              /* 最近一帧有效数据的时刻 */

static void jy901s_parse_byte(uint8_t byte);
static void jy901s_apply_frame(void);

/**
 * @brief  启动串口接收
 * @retval 0:成功 -1:失败
 */
int imu_init(void)
{
    if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U) != HAL_OK) {
        return -1;
    }
    return 0;
}

/**
 * @brief  校验通过的帧写入 g_imu(在串口中断上下文执行)
 */
static void jy901s_apply_frame(void)
{
    int16_t v[4];

    /* 每包含 3 轴数据 + 1 个附加字段(温度或版本),低字节在前 */
    v[0] = (int16_t)(s_frame[2] | (s_frame[3] << 8));
    v[1] = (int16_t)(s_frame[4] | (s_frame[5] << 8));
    v[2] = (int16_t)(s_frame[6] | (s_frame[7] << 8));
    v[3] = (int16_t)(s_frame[8] | (s_frame[9] << 8));

    switch (s_frame[1]) {
    case JY901S_PKT_ACC:
        g_imu.accel_x_raw = v[0];
        g_imu.accel_y_raw = v[1];
        g_imu.accel_z_raw = v[2];
        for (uint8_t i = 0; i < 3; i++) {
            g_imu.acc_g[i] = (float)v[i] / 32768.0f * JY901S_ACC_FS_G;
        }
        g_imu.temperature_raw = v[3];
        g_imu.temp_c = (float)v[3] / JY901S_TEMP_SCALE;
        break;
    case JY901S_PKT_GYRO:
        g_imu.gyro_x_raw = v[0];
        g_imu.gyro_y_raw = v[1];
        g_imu.gyro_z_raw = v[2];
        for (uint8_t i = 0; i < 3; i++) {
            g_imu.gyro_dps[i] = (float)v[i] / 32768.0f * JY901S_GYRO_FS;
        }
        g_imu.temperature_raw = v[3];
        g_imu.temp_c = (float)v[3] / JY901S_TEMP_SCALE;
        break;
    case JY901S_PKT_ANGLE:
        g_imu.angle_x_raw = v[0];
        g_imu.angle_y_raw = v[1];
        g_imu.angle_z_raw = v[2];
        g_imu.roll_deg = (float)v[0] / 32768.0f * JY901S_ANGLE_FS;
        g_imu.pitch_deg = (float)v[1] / 32768.0f * JY901S_ANGLE_FS;
        g_imu.yaw_deg = (float)v[2] / 32768.0f * JY901S_ANGLE_FS;
        break;
    default:
        return;
    }

    g_imu.online = 1U;
    s_last_frame_tick = HAL_GetTick();
}

/**
 * @brief  逐字节组帧,失步时从下一个 0x55 重新同步
 */
static void jy901s_parse_byte(uint8_t byte)
{
    if (s_frame_idx == 0U) {
        if (byte != JY901S_FRAME_HEAD) {
            return;
        }
        s_frame[s_frame_idx++] = byte;
    } else if (s_frame_idx == 1U) {
        /* 只解析加速度/角速度/角度包 */
        if (byte != JY901S_PKT_ACC && byte != JY901S_PKT_GYRO &&
            byte != JY901S_PKT_ANGLE) {
            s_frame_idx = 0U;
            return;
        }
        s_frame[s_frame_idx++] = byte;
    } else {
        s_frame[s_frame_idx++] = byte;
        if (s_frame_idx < JY901S_FRAME_LEN) {
            return;
        }

        /* 整帧收齐,校验:前 10 字节之和低 8 位 */
        s_frame_idx = 0U;
        uint8_t sum = 0U;
        for (uint8_t i = 0; i < JY901S_FRAME_LEN - 1U; i++) {
            sum += s_frame[i];
        }
        if (sum == s_frame[JY901S_FRAME_LEN - 1U]) {
            jy901s_apply_frame();
        } else {
            g_imu.err_cnt++;
        }
    }
}

/**
 * @brief  串口接收完成回调(中断上下文),解析并重新挂起接收
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        jy901s_parse_byte(s_rx_byte);
        HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
    }
}

/**
 * @brief  imu 任务:启动接收后周期检查在线状态
 */
void imu_task(void *argument)
{
    (void)argument;

    imu_init();

    for (;;) {
        /* 接收因溢出/帧错误被 HAL 中止时,重新挂起 */
        if (huart1.RxState == HAL_UART_STATE_READY) {
            g_imu.err_cnt++;
            imu_init();
        }

        /* 帧流超时判离线 */
        if (g_imu.online != 0U &&
            (HAL_GetTick() - s_last_frame_tick) > IMU_FRAME_TIMEOUT_MS) {
            g_imu.online = 0U;
            g_imu.err_cnt++;
        }

        osDelay(IMU_TASK_PERIOD_MS);
    }
}
