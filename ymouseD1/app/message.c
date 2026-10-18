/**
  ******************************************************************************
  * @file    message.c
  * @brief   姿态数据串口上报任务(USART2,9600 8N1,DMA 发送)
  * @note    上报功能当前是关闭的,把 MESSAGE_REPORT_ENABLE 改成 1 即可打开。
  ******************************************************************************
  */

#include "cmsis_os2.h"

#include <stdio.h>

#include "main.h"
#include "imu.h"
#include "usart.h"

/* 1 = 打开串口上报,0 = 只保持任务存在(当前) */
#define MESSAGE_REPORT_ENABLE 0

#define MESSAGE_PERIOD_MS 100U
#define MESSAGE_BUF_SIZE  100U

void message_task(void *argument)
{
  (void)argument;

#if MESSAGE_REPORT_ENABLE
  /* DMA 发送期间缓冲区必须一直有效,所以用 static 而不是局部数组 */
  static char tx_buf[MESSAGE_BUF_SIZE];

  for (;;) {
    int len;

    if (g_imu.online != 0U) {
      /* 姿态角,单位度(已保留两位小数) */
      len = snprintf(tx_buf, sizeof(tx_buf), "%.2f,%.2f,%.2f\r\n",
                     g_imu.roll_deg, g_imu.pitch_deg, g_imu.yaw_deg);
    } else {
      len = snprintf(tx_buf, sizeof(tx_buf), "IMU OFFLINE err:%lu\r\n",
                     (unsigned long)g_imu.err_cnt);
    }

    /* 上一帧 DMA 还没发完就丢弃本帧,避免改写正在发送的缓冲区 */
    if ((len > 0) && (huart2.gState == HAL_UART_STATE_READY)) {
      (void)HAL_UART_Transmit_DMA(&huart2, (uint8_t *)tx_buf, (uint16_t)len);
    }

    osDelay(MESSAGE_PERIOD_MS);
  }
#else
  /* 未启用上报:任务只保持存在,几乎不占 CPU */
  for (;;) {
    osDelay(MESSAGE_PERIOD_MS);
  }
#endif
}
