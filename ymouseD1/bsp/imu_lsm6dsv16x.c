/**
  ******************************************************************************
  * @file    imu_lsm6dsv16x.c
  * @brief   LSM6DSV16XTR 六轴 IMU 后端驱动(陀螺仪为主)
  *          硬件:IIC1 —— PB6(SCL) / PB7(SDA)
  *          寄存器定义参考 ST 官方 lsm6dsv16x_reg.h / lsm6dsv16x_reg.c
  * @note    本文件只负责这颗器件本身;单位换算与任务调度在 bsp/imu.c
  ******************************************************************************
  */

#include "imu_lsm6dsv16x.h"

#include <stddef.h>

#include "main.h"
#include "i2c.h"
#include "cmsis_os2.h"

/* 实际使用的从机地址(HAL 形式:7 位地址左移 1 位),由探测确定 */
static uint16_t s_hal_addr = LSM6DSV16X_ADDR_SA0_LOW;

/* ==========================================================================
 * 底层寄存器读写
 * ========================================================================== */

static int lsm6_write_reg(uint8_t reg, uint8_t val)
{
  if (HAL_I2C_Mem_Write(&hi2c1, s_hal_addr, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT,
                        &val, 1U, IMU_I2C_TIMEOUT_MS) != HAL_OK) {
    return -1;
  }
  return 0;
}

static int lsm6_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
  if (HAL_I2C_Mem_Read(&hi2c1, s_hal_addr, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT,
                       buf, len, IMU_I2C_TIMEOUT_MS) != HAL_OK) {
    return -1;
  }
  return 0;
}

/**
 * @brief 探测 WHO_AM_I,同时确定 SA0 接法(0x6A / 0x6B)
 * @retval 0 找到, -1 没找到
 */
static int lsm6_check_whoami(void)
{
  static const uint16_t cand[2] = { LSM6DSV16X_ADDR_SA0_LOW, LSM6DSV16X_ADDR_SA0_HIGH };
  uint8_t v = 0U;
  uint8_t i;

  for (i = 0U; i < 2U; i++) {
    if (HAL_I2C_Mem_Read(&hi2c1, cand[i], LSM6DSV16X_WHO_AM_I, I2C_MEMADD_SIZE_8BIT,
                         &v, 1U, IMU_I2C_TIMEOUT_MS) == HAL_OK) {
      if (v == LSM6DSV16X_WHOAMI_ID) {
        s_hal_addr = cand[i];
        return 0;
      }
    }
  }
  return -1;
}

/**
 * @brief 软复位,并等待 sw_reset 位自清零
 * @retval 0 成功, -1 失败
 */
static int lsm6_soft_reset(void)
{
  uint8_t v;
  uint8_t i;

  if (lsm6_write_reg(LSM6DSV16X_CTRL3, LSM6DSV16X_CTRL3_SW_RESET) != 0) {
    return -1;
  }
  /* sw_reset 自清零,轮询等待(典型 < 10ms) */
  for (i = 0U; i < 25U; i++) {
    HAL_Delay(2U);
    if (lsm6_read_regs(LSM6DSV16X_CTRL3, &v, 1U) != 0) {
      return -1;
    }
    if ((v & LSM6DSV16X_CTRL3_SW_RESET) == 0U) {
      return 0;
    }
  }
  return -1;
}

/**
 * @brief 配置陀螺仪:量程、LPF1(先量程后开 ODR,避免带数据切换量程)
 * @retval 0 成功, -1 失败
 */
static int lsm6_config_gyro(void)
{
  uint8_t v;

  /* CTRL6:量程 fs_g[3:0] + LPF1 带宽 lpf1_g_bw[6:4] */
  v = (uint8_t)((IMU_GY_FS & 0x0FU) | ((IMU_GY_LPF_BW & 0x07U) << 4));
  if (lsm6_write_reg(LSM6DSV16X_CTRL6, v) != 0) {
    return -1;
  }

  /* CTRL7:LPF1 使能位 */
  if (lsm6_read_regs(LSM6DSV16X_CTRL7, &v, 1U) != 0) {
    return -1;
  }
  if (IMU_GY_LPF_ENABLE != 0U) {
    v = (uint8_t)(v | LSM6DSV16X_CTRL7_LPF1_G_EN);
  } else {
    v = (uint8_t)(v & (uint8_t)(~LSM6DSV16X_CTRL7_LPF1_G_EN));
  }
  if (lsm6_write_reg(LSM6DSV16X_CTRL7, v) != 0) {
    return -1;
  }

  /* CTRL2:输出速率 odr_g[3:0] + 工作模式 op_mode_g[6:4] */
  v = (uint8_t)((IMU_GY_ODR & 0x0FU) | ((IMU_GY_OP_MODE & 0x07U) << 4));
  if (lsm6_write_reg(LSM6DSV16X_CTRL2, v) != 0) {
    return -1;
  }

  return 0;
}

/**
 * @brief 配置加速度计:量程、ODR、模式
 * @retval 0 成功, -1 失败
 */
static int lsm6_config_accel(void)
{
  uint8_t v;

  /* CTRL8:量程 fs_xl[1:0],其余位(HP_LPF2_XL_BW 等)保持不变 */
  if (lsm6_read_regs(LSM6DSV16X_CTRL8, &v, 1U) != 0) {
    return -1;
  }
  v = (uint8_t)((v & (uint8_t)(~0x03U)) | (IMU_XL_FS & 0x03U));
  if (lsm6_write_reg(LSM6DSV16X_CTRL8, v) != 0) {
    return -1;
  }

  /* CTRL1:输出速率 odr_xl[3:0] + 工作模式 op_mode_xl[6:4] */
  v = (uint8_t)((IMU_XL_ODR & 0x0FU) | ((IMU_XL_OP_MODE & 0x07U) << 4));
  if (lsm6_write_reg(LSM6DSV16X_CTRL1, v) != 0) {
    return -1;
  }

  return 0;
}

/* ==========================================================================
 * 后端接口实现
 * ========================================================================== */

/**
 * @brief 后端 init:探测 -> 软复位 -> 配 BDU/IF_INC -> 配陀螺仪/加速度计
 */
static int lsm6_init(void)
{
  uint8_t v;

  s_hal_addr = LSM6DSV16X_ADDR_SA0_LOW;

  /* 1. 器件在不在 */
  if (lsm6_check_whoami() != 0) {
    return -1;
  }

  /* 2. 软复位,回到已知状态 */
  if (lsm6_soft_reset() != 0) {
    return -1;
  }

  /* 3. CTRL3:BDU=1(读 12 字节期间输出不被打断) + IF_INC=1(地址自增) */
  v = (uint8_t)(LSM6DSV16X_CTRL3_BDU | LSM6DSV16X_CTRL3_IF_INC);
  if (lsm6_write_reg(LSM6DSV16X_CTRL3, v) != 0) {
    return -1;
  }

  /* 复位后等器件内部启动完成再配置 */
  HAL_Delay(10U);

  /* 4. 陀螺仪配置(重点) */
  if (lsm6_config_gyro() != 0) {
    return -1;
  }

  /* 5. 加速度计配置(同一颗芯片,一并配好) */
  if (lsm6_config_accel() != 0) {
    return -1;
  }

  return 0;
}

/**
 * @brief 后端 wait_ready:等 STATUS_REG.GDA 置位
 */
static int lsm6_wait_ready(uint32_t timeout_ms)
{
  uint8_t st;
  uint32_t t0 = HAL_GetTick();

  while ((HAL_GetTick() - t0) < timeout_ms) {
    if (lsm6_read_regs(LSM6DSV16X_STATUS_REG, &st, 1U) == 0) {
      if ((st & LSM6DSV16X_STATUS_GDA) != 0U) {
        return 0;
      }
    }
    osDelay(1U);
  }
  return -1;
}

/**
 * @brief 后端 read:从 0x20 起一次连读 14 字节
 *        (温度 2 + 陀螺仪 6 + 加速度计 6),原来分成 12+2 两次读
 * @note  0x20~0x2D 是连续地址,合成一次传输可省掉一整个
 *        "START+地址+寄存器号" 的开销,约省 1/4 的 I2C 时间
 */
static int lsm6_read(imu_sample_t *s)
{
  uint8_t buf[LSM6DSV16X_OUT_BLOCK_LEN];
  uint8_t i;

  if (s == NULL) {
    return -1;
  }

  if (lsm6_read_regs(LSM6DSV16X_OUT_TEMP_L, buf, LSM6DSV16X_OUT_BLOCK_LEN) != 0) {
    return -1;
  }

  /* buf[0..1] 温度 */
  s->temp_raw = (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);

  /* buf[2..7] 陀螺仪 X/Y/Z,buf[8..13] 加速度计 X/Y/Z,都是小端 */
  for (i = 0U; i < 3U; i++) {
    s->gyro_raw[i] =
        (int16_t)(((uint16_t)buf[2U * i + 3U] << 8) | (uint16_t)buf[2U * i + 2U]);
    s->accel_raw[i] =
        (int16_t)(((uint16_t)buf[2U * i + 9U] << 8) | (uint16_t)buf[2U * i + 8U]);
    s->angle_raw[i] = 0; /* 本器件不直接输出姿态角 */
  }

  return 0;
}

const imu_backend_t imu_backend_lsm6dsv16x = {
  .name = "LSM6DSV16XTR",
  .addr_7bit = 0x6AU,
  .has_angle = 0U,          /* 只有原始角速度/加速度,姿态角需自己解算 */
  .need_gyro_calib = 1U,    /* 上电需要做零偏标定 */
  .accel_lsb_g = (IMU_XL_MG_PER_LSB / 1000.0f),
  .gyro_lsb_dps = (IMU_GY_MDPS_PER_LSB / 1000.0f),
  .angle_lsb_deg = 0.0f,
  .temp_lsb_c = (1.0f / 256.0f),
  .temp_offset_c = 25.0f,
  .init = lsm6_init,
  .wait_ready = lsm6_wait_ready,
  .read = lsm6_read,
};
