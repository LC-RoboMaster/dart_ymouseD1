//
// Created by 13466 on 2026/9/25.
//
// 姿态数据串口上报任务:读取 g_imu,经 USART2(DMA)输出
#include "cmsis_os2.h"
#include <stdio.h>
#include "main.h"
#include "imu.h"
#include "usart.h"

/* 上报周期 */
#define MESSAGE_PERIOD_MS 100U
#define MESSAGE_BUF_SIZE  100U

void message_task(void *argument) {
  (void)argument;
  /* DMA 传输进行期间缓冲区必须保持有效,故用 static */
  static char tx_buf[MESSAGE_BUF_SIZE];

  for (;;) {
    int len;
    // if (g_imu.online != 0U) {
    //   /* 温度和加速度用于自诊断:室温 T≠0、平放时 az≈1g 说明链路正常 */
    //
    //   len = snprintf(tx_buf, sizeof(tx_buf),
    //                  "%.2f ,%.2f ,%.2f \r\n ",
    //                  g_imu.roll_deg, g_imu.pitch_deg, g_imu.yaw_deg);
    //
    // } else {
    //   /* 帧流超时 200ms 判离线:查接线(模块TXD→PB7)与波特率(9600) */
    //   len = snprintf(tx_buf, sizeof(tx_buf),
    //                  "IMU OFFLINE err:%lu\r\n",
    //                  (unsigned long)g_imu.err_cnt);
    // }
    /* 上一帧 DMA 未完成时丢弃本帧,避免改写正在发送的缓冲区 */
    // if (len > 0 && huart2.gState == HAL_UART_STATE_READY) {
    //   HAL_UART_Transmit_DMA(&huart2, (uint8_t *)tx_buf, (uint16_t)len);
    // }
    osDelay(MESSAGE_PERIOD_MS);
  }
}
