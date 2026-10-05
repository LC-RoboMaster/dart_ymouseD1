/**
  ******************************************************************************
  * @file    imu_jy901s.c
  * @brief   维特智能 JY901S 九轴姿态模块后端驱动(I2C 接口)
  *          硬件:IIC1 —— PB6(SCL) / PB7(SDA),从机地址 0x50
  *          寄存器表来源:维特智能官方《WIT 标准通信协议》
  * @note    本文件只负责这颗模块本身;单位换算与任务调度在 bsp/imu.c
  ******************************************************************************
  */

#include "imu_jy901s.h"

#include <stddef.h>

#include "main.h"
#include "i2c.h"

/* 模块实际配置,上电时由 Jy901s_ReadInfo() 填充 */
jy901s_info_t g_jy901s_info;

/* ==========================================================================
 * 底层读写
 * ========================================================================== */

/**
 * @brief 连续读 n 个 16 位寄存器(小端),转成 int16
 * @retval 0 成功, -1 失败
 */
static int jy901s_read_i16(uint8_t reg, int16_t *out, uint8_t n)
{
  uint8_t buf[2U * 8U];
  uint8_t i;
  HAL_StatusTypeDef st;

  if (n == 0U || n > 8U || out == NULL) {
    return -1;
  }
  st = HAL_I2C_Mem_Read(&hi2c1, JY901S_ADDR, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT,
                        buf, (uint16_t)(2U * n), IMU_I2C_TIMEOUT_MS);
  if (st != HAL_OK) {
    g_imu_diag.last_addr = JY901S_I2C_ADDR_7BIT;
    g_imu_diag.last_status = (uint8_t)st;
    g_imu_diag.fail_cnt++;
    return -1;
  }
  for (i = 0U; i < n; i++) {
    out[i] = (int16_t)(((uint16_t)buf[2U * i + 1U] << 8) | (uint16_t)buf[2U * i]);
  }
  return 0;
}

/**
 * @brief 写模块配置寄存器(2 字节:低字节在前)
 * @retval 0 成功, -1 失败
 */
static int jy901s_write_cfg(uint8_t reg, uint8_t low, uint8_t high)
{
  uint8_t buf[2];
  HAL_StatusTypeDef st;

  buf[0] = low;
  buf[1] = high;
  st = HAL_I2C_Mem_Write(&hi2c1, JY901S_ADDR, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT,
                         buf, 2U, IMU_I2C_TIMEOUT_MS);
  if (st != HAL_OK) {
    g_imu_diag.last_addr = JY901S_I2C_ADDR_7BIT;
    g_imu_diag.last_status = (uint8_t)st;
    g_imu_diag.fail_cnt++;
    return -1;
  }
  return 0;
}

/* ==========================================================================
 * 器件专有接口
 * ========================================================================== */

int Jy901s_SaveConfig(void)
{
  if (jy901s_write_cfg(JY901S_REG_SAVE, 0x00U, 0x00U) != 0) {
    return -1;
  }
  HAL_Delay(JY901S_ZERO_DELAY_MS);
  return 0;
}

int Jy901s_ReadInfo(void)
{
  int16_t v = 0;
  uint8_t n = 0U;

  if (jy901s_read_i16(JY901S_REG_IICADDR, &v, 1U) == 0) {
    g_jy901s_info.iic_addr = (uint16_t)v;
    n++;
  }
  if (jy901s_read_i16(JY901S_REG_BANDWIDTH, &v, 1U) == 0) {
    g_jy901s_info.bandwidth = (uint16_t)v;
    n++;
  }
  if (jy901s_read_i16(JY901S_REG_GYR_RANGE, &v, 1U) == 0) {
    g_jy901s_info.gyr_range = (uint16_t)v;
    n++;
  }
  if (jy901s_read_i16(JY901S_REG_ACC_RANGE, &v, 1U) == 0) {
    g_jy901s_info.acc_range = (uint16_t)v;
    n++;
  }
  if (jy901s_read_i16(JY901S_REG_ORIENT, &v, 1U) == 0) {
    g_jy901s_info.orient = (uint16_t)v;
    n++;
  }
  if (jy901s_read_i16(JY901S_REG_ALGO, &v, 1U) == 0) {
    g_jy901s_info.algo = (uint16_t)v;
    n++;
  }
  if (jy901s_read_i16(JY901S_REG_VOLTAGE, &v, 1U) == 0) {
    g_jy901s_info.voltage_v = (float)v / 100.0f;
    n++;
  }

  g_jy901s_info.valid = (n != 0U) ? 1U : 0U;
  return (n != 0U) ? 0 : -1;
}

int Jy901s_SetAngleZero(void)
{
  /* 1. 解锁配置寄存器 */
  if (jy901s_write_cfg(JY901S_REG_KEY, JY901S_KEY_H, JY901S_KEY_L) != 0) {
    return -1;
  }
  HAL_Delay(JY901S_ZERO_DELAY_MS);

  /* 2. 角度归零(含航向角) */
  if (jy901s_write_cfg(JY901S_REG_CALSW, JY901S_CAL_ANGLE, 0x00U) != 0) {
    return -1;
  }
  HAL_Delay(JY901S_ZERO_DELAY_MS);

  /* 3. 保存 */
  return Jy901s_SaveConfig();
}

/* ==========================================================================
 * 后端接口实现
 * ========================================================================== */

/**
 * @brief 探测器件是否在总线上
 * @note  首选读版本号寄存器(0x2E) —— 比"能不能读到姿态角"更直接;
 *        万一某版固件没实现 0x2E,再退回读姿态角,保证不会比以前更差
 * @retval 0 在线, -1 不在
 */
static int jy901s_probe(void)
{
  int16_t v = 0;
  int16_t tmp[3];

  if (jy901s_read_i16(JY901S_REG_VERSION, &v, 1U) == 0) {
    g_jy901s_info.version = (uint16_t)v;
    return 0;
  }
  if (jy901s_read_i16(JY901S_REG_ROLL, tmp, 3U) == 0) {
    return 0;
  }
  return -1;
}

/**
 * @brief 后端 init:探测器件 -> 把模块配置读回来 -> 可选归零
 */
static int jy901s_init(void)
{
  g_jy901s_info.valid = 0U;

  if (jy901s_probe() != 0) {
    return -1;
  }

  /* 把模块的实际配置(地址/带宽/量程/算法/电压)读回来,方便和驱动假设对照 */
  (void)Jy901s_ReadInfo();

#if (JY901S_AUTO_ZERO_ON_INIT != 0U)
  (void)Jy901s_SetAngleZero();
#endif

  return 0;
}

/**
 * @brief 后端 read:0x34~0x40 连续读 26 字节,
 *        保证加速度/角速度/姿态角是同一时刻的快照
 */
static int jy901s_read(imu_sample_t *s)
{
  uint8_t buf[JY901S_BLOCK_BYTES];
  uint8_t i;
  HAL_StatusTypeDef st;

  if (s == NULL) {
    return -1;
  }
  st = HAL_I2C_Mem_Read(&hi2c1, JY901S_ADDR, JY901S_REG_BLOCK, I2C_MEMADD_SIZE_8BIT,
                        buf, JY901S_BLOCK_BYTES, IMU_I2C_TIMEOUT_MS);
  if (st != HAL_OK) {
    g_imu_diag.last_addr = JY901S_I2C_ADDR_7BIT;
    g_imu_diag.last_status = (uint8_t)st;
    g_imu_diag.fail_cnt++;
    return -1;
  }

  /* 0x34~0x36 加速度 */
  for (i = 0U; i < 3U; i++) {
    s->accel_raw[i] =
        (int16_t)(((uint16_t)buf[2U * i + 1U] << 8) | (uint16_t)buf[2U * i]);
  }

  /* 0x37~0x39 角速度 */
  for (i = 0U; i < 3U; i++) {
    s->gyro_raw[i] = (int16_t)(((uint16_t)buf[2U * (i + 3U) + 1U] << 8) |
                               (uint16_t)buf[2U * (i + 3U)]);
  }

  /* 0x3A~0x3C 磁场:JY901S 是九轴,这里有效;当前 imu_sample_t 未承载,跳过 */

  /* 0x3D~0x3F 姿态角(模块内部已解算) */
  for (i = 0U; i < 3U; i++) {
    s->angle_raw[i] = (int16_t)(((uint16_t)buf[2U * (i + 9U) + 1U] << 8) |
                                (uint16_t)buf[2U * (i + 9U)]);
  }

  /* 0x40 温度 */
  s->temp_raw = (int16_t)(((uint16_t)buf[25] << 8) | (uint16_t)buf[24]);

  return 0;
}

const imu_backend_t imu_backend_jy901s = {
  .name = "JY901S",
  .addr_7bit = JY901S_I2C_ADDR_7BIT,
  .has_angle = 1U,          /* 模块内部直接输出 Roll/Pitch/Yaw */
  .need_gyro_calib = 0U,    /* 出厂已标定,不需要上电做零偏标定 */
  .accel_lsb_g = (16.0f / 32768.0f),
  .gyro_lsb_dps = (2000.0f / 32768.0f),
  .angle_lsb_deg = (180.0f / 32768.0f),
  .temp_lsb_c = (1.0f / 100.0f),
  .temp_offset_c = 0.0f,
  .init = jy901s_init,
  .wait_ready = NULL,       /* 寄存器持续刷新,不需要等就绪 */
  .read = jy901s_read,
};
