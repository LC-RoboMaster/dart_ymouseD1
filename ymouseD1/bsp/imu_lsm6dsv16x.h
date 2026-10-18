#ifndef YMOUSED1_IMU_LSM6DSV16X_H_
#define YMOUSED1_IMU_LSM6DSV16X_H_

#include <stdint.h>

#include "imu.h" /* imu_backend_t */

/* ==========================================================================
 * LSM6DSV16XTR 六轴 IMU —— 后端驱动
 * 硬件:IIC1 —— PB6(SCL) / PB7(SDA)
 * 寄存器资料:ST 官方 lsm6dsv16x_reg.h / lsm6dsv16x_reg.c (BSD-3-Clause)
 * ========================================================================== */

/* ------------------------- 寄存器地址 ------------------------- */
#define LSM6DSV16X_WHO_AM_I        0x0FU /* 器件 ID,固定 0x70 */
#define LSM6DSV16X_CTRL1           0x10U /* 加速度计 ODR + 模式 */
#define LSM6DSV16X_CTRL2           0x11U /* 陀螺仪   ODR + 模式 */
#define LSM6DSV16X_CTRL3           0x12U /* 软复位 / BDU / IF_INC / BOOT */
#define LSM6DSV16X_CTRL6           0x15U /* 陀螺仪   量程 + LPF1 带宽 */
#define LSM6DSV16X_CTRL7           0x16U /* 陀螺仪   LPF1 使能 */
#define LSM6DSV16X_CTRL8           0x17U /* 加速度计 量程 + LPF2 带宽 */
#define LSM6DSV16X_STATUS_REG      0x1EU /* 数据就绪标志 */
#define LSM6DSV16X_OUT_TEMP_L      0x20U /* 温度,2 字节小端 */
#define LSM6DSV16X_OUTX_L_G        0x22U /* 陀螺仪 X 低字节 */
/* 0x20~0x2D 是连续的:温度(2) + 陀螺仪(6) + 加速度计(6),一次读完最省总线时间 */
#define LSM6DSV16X_OUT_BLOCK_LEN   14U

#define LSM6DSV16X_WHOAMI_ID       0x70U /* LSM6DSV16X / LSM6DSV16XTR 的 ID */

/* I2C 从机地址:SA0 接地 = 0x6A,SA0 接 VDD = 0x6B(7 位)
 * 注意:HAL 的 DevAddress 参数要求"7 位地址左移 1 位",故宏里已经移好 */
#define LSM6DSV16X_ADDR_SA0_LOW    (0x6AU << 1) /* 0xD4 */
#define LSM6DSV16X_ADDR_SA0_HIGH   (0x6BU << 1) /* 0xD6 */

/* ------------------------- CTRL3 控制位 ------------------------- */
#define LSM6DSV16X_CTRL3_SW_RESET  (1U << 0) /* 软复位,自清零 */
#define LSM6DSV16X_CTRL3_IF_INC    (1U << 2) /* 寄存器地址自增(连续读必须置 1) */
#define LSM6DSV16X_CTRL3_BDU       (1U << 6) /* 输出寄存器块更新保护 */

/* ------------------------- CTRL7 控制位 ------------------------- */
#define LSM6DSV16X_CTRL7_LPF1_G_EN (1U << 0) /* 陀螺仪 LPF1 使能 */

/* ------------------------- STATUS_REG 标志位 ------------------------- */
#define LSM6DSV16X_STATUS_GDA      (1U << 1) /* 陀螺仪新数据就绪 */

/* --------------- CTRL2 odr_g[3:0] 陀螺仪输出速率 --------------- */
#define LSM6DSV16X_ODR_OFF         0x00U
#define LSM6DSV16X_ODR_1Hz875      0x01U
#define LSM6DSV16X_ODR_7Hz5        0x02U
#define LSM6DSV16X_ODR_15Hz        0x03U
#define LSM6DSV16X_ODR_30Hz        0x04U
#define LSM6DSV16X_ODR_60Hz        0x05U
#define LSM6DSV16X_ODR_120Hz       0x06U
#define LSM6DSV16X_ODR_240Hz       0x07U
#define LSM6DSV16X_ODR_480Hz       0x08U
#define LSM6DSV16X_ODR_960Hz       0x09U
#define LSM6DSV16X_ODR_1920Hz      0x0AU
#define LSM6DSV16X_ODR_3840Hz      0x0BU
#define LSM6DSV16X_ODR_7680Hz      0x0CU

/* ------------- CTRL2 op_mode_g[6:4] 陀螺仪工作模式 ------------- */
#define LSM6DSV16X_GY_HIGH_PERF    0x00U /* 高性能模式(默认,低噪声) */
#define LSM6DSV16X_GY_HIGH_ACC     0x01U /* 高精度模式 */
#define LSM6DSV16X_GY_SLEEP        0x04U /* 睡眠 */
#define LSM6DSV16X_GY_LOW_POWER    0x05U /* 低功耗 */

/* ------------------ CTRL6 fs_g[3:0] 陀螺仪量程 ------------------ */
#define LSM6DSV16X_FS_G_125DPS     0x00U /* 4.375 mdps/LSB */
#define LSM6DSV16X_FS_G_250DPS     0x01U /* 8.75  mdps/LSB */
#define LSM6DSV16X_FS_G_500DPS     0x02U /* 17.5  mdps/LSB */
#define LSM6DSV16X_FS_G_1000DPS    0x03U /* 35.0  mdps/LSB */
#define LSM6DSV16X_FS_G_2000DPS    0x04U /* 70.0  mdps/LSB */
#define LSM6DSV16X_FS_G_4000DPS    0x0CU /* 140.0 mdps/LSB */

/* -------------- CTRL6 lpf1_g_bw[6:4] 陀螺仪 LPF1 带宽 -------------- */
#define LSM6DSV16X_GY_LPF_ULTRA_LIGHT  0x00U
#define LSM6DSV16X_GY_LPF_VERY_LIGHT   0x01U
#define LSM6DSV16X_GY_LPF_LIGHT        0x02U
#define LSM6DSV16X_GY_LPF_MEDIUM       0x03U
#define LSM6DSV16X_GY_LPF_STRONG       0x04U
#define LSM6DSV16X_GY_LPF_VERY_STRONG  0x05U
#define LSM6DSV16X_GY_LPF_AGGRESSIVE   0x06U
#define LSM6DSV16X_GY_LPF_XTREME       0x07U

/* ------------- CTRL1 op_mode_xl[6:4] 加速度计工作模式 ------------- */
#define LSM6DSV16X_XL_HIGH_PERF    0x00U
#define LSM6DSV16X_XL_HIGH_ACC     0x01U
#define LSM6DSV16X_XL_LOW_POWER_2  0x04U
#define LSM6DSV16X_XL_LOW_POWER_4  0x05U
#define LSM6DSV16X_XL_LOW_POWER_8  0x06U
#define LSM6DSV16X_XL_NORMAL       0x07U

/* ------------------ CTRL8 fs_xl[1:0] 加速度计量程 ------------------ */
#define LSM6DSV16X_FS_XL_2G        0x00U /* 0.061 mg/LSB */
#define LSM6DSV16X_FS_XL_4G        0x01U /* 0.122 mg/LSB */
#define LSM6DSV16X_FS_XL_8G        0x02U /* 0.244 mg/LSB */
#define LSM6DSV16X_FS_XL_16G       0x03U /* 0.488 mg/LSB */

/* ==========================================================================
 * LSM6DSV16XTR 默认配置:改这里即可
 * ========================================================================== */

/* ---- 陀螺仪(重点) ---- */
#define IMU_GY_ODR          LSM6DSV16X_ODR_240Hz      /* 输出速率 240 Hz */
#define IMU_GY_FS           LSM6DSV16X_FS_G_2000DPS   /* 量程 ±2000 dps */
#define IMU_GY_MDPS_PER_LSB 70.0f                     /* 与上面量程对应 */
#define IMU_GY_OP_MODE      LSM6DSV16X_GY_HIGH_PERF   /* 高性能模式 */
#define IMU_GY_LPF_BW       LSM6DSV16X_GY_LPF_MEDIUM  /* LPF1 带宽档位 */
#define IMU_GY_LPF_ENABLE   1U                        /* 1:开启 LPF1 */

/* ---- 加速度计(同一颗芯片,一并初始化) ---- */
#define IMU_XL_ODR          LSM6DSV16X_ODR_240Hz
#define IMU_XL_FS           LSM6DSV16X_FS_XL_4G
#define IMU_XL_MG_PER_LSB   0.122f
#define IMU_XL_OP_MODE      LSM6DSV16X_XL_HIGH_PERF

/* ==========================================================================
 * 后端实例(供 bsp/imu.c 的适配层使用)
 * ========================================================================== */
extern const imu_backend_t imu_backend_lsm6dsv16x;

#endif /* YMOUSED1_IMU_LSM6DSV16X_H_ */
