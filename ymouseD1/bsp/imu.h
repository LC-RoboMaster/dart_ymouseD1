#ifndef YMOUSED1_IMU_H_
#define YMOUSED1_IMU_H_

#include <stdint.h>

/* ==========================================================================
 * 姿态传感器接口
 *
 * 硬件:LSM6DSV16XTR 六轴 IMU,挂在 IIC1(PB6=SCL / PB7=SDA),
 *       地址 0x6A(SA0=0) / 0x6B(SA0=1),上电自动探测。
 *
 * 应用层只包含本文件:读 g_imu,或调用 Imu_* 接口。
 * 具体器件寄存器在 imu_lsm6dsv16x.c 里,本文件不碰硬件。
 * ========================================================================== */

/* ---------------- 运行参数 ---------------- */
#define IMU_I2C_TIMEOUT_MS   50U  /* 单次 I2C 传输超时 */
#define IMU_TASK_PERIOD_MS   10U  /* 采集周期 10ms => 100Hz */
#define IMU_RETRY_DELAY_MS   500U /* 初始化失败后的重试间隔 */
#define IMU_CALIB_SAMPLES    200U /* 陀螺仪零偏标定采样点数(标定时须静止) */

/* ==========================================================================
 * 后端(具体器件驱动)描述
 * ========================================================================== */

/** @brief 一帧原始数据,由后端填充 */
typedef struct {
  int16_t accel_raw[3]; /* 加速度原始值 X/Y/Z */
  int16_t gyro_raw[3];  /* 角速度原始值 X/Y/Z */
  int16_t angle_raw[3]; /* 器件内部解算的姿态角,器件不支持时保持 0 */
  int16_t temp_raw;     /* 温度原始值 */
} imu_sample_t;

/**
 * @brief 后端描述表:一个器件一份,适配层按它做换算和调度
 * @note  lsb_* 表示"1 个原始 LSB 对应多少工程单位"
 */
typedef struct {
  const char *name;        /* 器件名 */
  uint8_t addr_7bit;       /* 从机地址(7 位) */
  uint8_t has_angle;       /* 1:器件直接输出姿态角 */
  uint8_t need_gyro_calib; /* 1:需要上电做陀螺仪零偏标定 */

  float accel_lsb_g;       /* g   / LSB */
  float gyro_lsb_dps;      /* °/s / LSB */
  float angle_lsb_deg;     /* °   / LSB */
  float temp_lsb_c;        /* ℃   / LSB */
  float temp_offset_c;     /* 温度换算的固定偏置(LSM6DSV16XTR 为 25℃) */

  /** 探测并初始化器件,0 成功;器件不在时返回 -1 */
  int (*init)(void);
  /** 等待新数据就绪,0 就绪;器件不需要判断时为 NULL */
  int (*wait_ready)(uint32_t timeout_ms);
  /** 读一帧原始数据,0 成功 */
  int (*read)(imu_sample_t *s);
} imu_backend_t;

/* 后端实例 */
extern const imu_backend_t imu_backend_lsm6dsv16x;

/* ==========================================================================
 * 统一数据与接口
 * ========================================================================== */
typedef struct {
  uint8_t online;          /* 1:在线 0:离线 */
  uint8_t has_angle;       /* 1:下面的 roll/pitch/yaw 有效 */
  const char *source;      /* 当前使用的器件名 */
  uint32_t err_cnt;        /* I2C 通信错误计数 */

  int16_t gyro_raw[3];     /* 角速度原始值 */
  float gyro_dps[3];       /* 角速度,°/s(已减零偏) */
  float gyro_bias_dps[3];  /* 上电标定得到的零偏 */

  int16_t accel_raw[3];    /* 加速度原始值 */
  float accel_g[3];        /* 加速度,g */

  /* ---- 角度,单位:度(°) ---- */
  float roll_deg;          /* 横滚角。器件自带解算时直接取器件的值;        */
  float pitch_deg;         /* 俯仰角。否则由角速度按真实时间积分得到,     */
  float yaw_deg;           /* 航向角。以调用 Imu_Init() 那一刻为零位,     */
                           /*         已折算到 (-180, 180]。              */
                           /* 注意:纯陀螺积分没有绝对参考,会缓慢漂移;     */
                           /*       需要长时间稳定的话得再用加速度计做融合。*/

  float temp_c;            /* 器件温度,℃ */
} imu_data_t;

extern imu_data_t g_imu;

/* ==========================================================================
 * 诊断信息 —— 排查"读不到数据"用
 *
 * 典型判读:
 *   g_imu_diag.count == 0            总线上一个器件都不应答
 *                                    -> 接线(SCL/SDA 是否接反、共地)/ 上拉电阻 / 模块供电
 *   g_imu_diag.count > 0 但 addr!=0x50
 *                                    -> 模块没在默认地址,可能 IICADDR(0x1A) 被改过
 *   last_status == 1 (HAL_ERROR)     从机没应答(ACK 缺失)
 *   last_status == 2 (HAL_BUSY)      外设忙,通常是上一次传输卡住了
 *   last_status == 3 (HAL_TIMEOUT)   总线被拉死(典型:没有上拉,或 SDA 被从机拉住)
 * ========================================================================== */
#define IMU_SCAN_MAX 8

typedef struct {
  uint8_t count;              /* 总线扫描时应答的器件个数 */
  uint8_t addr[IMU_SCAN_MAX]; /* 应答的 7 位地址(最多记前 8 个) */

  uint8_t last_addr;          /* 最近一次访问失败的从机地址(7 位),0 表示还没失败过 */
  uint8_t last_status;        /* 最近一次失败的 HAL 返回值:1=ERROR 2=BUSY 3=TIMEOUT */
  uint32_t fail_cnt;          /* 累计通信失败次数 */
} imu_diag_t;

extern imu_diag_t g_imu_diag;

/**
 * @brief  扫描 I2C1,列出所有会应答的 7 位地址
 * @note   结果同时写入 g_imu_diag。任意时刻都可以手动调用。
 * @retval 应答的器件个数
 */
uint8_t Imu_ScanBus(void);

/**
 * @brief  复位 I2C1 外设并重新初始化
 * @note   HAL 的 I2C 一旦因为超时或总线被拉住而卡住,之后每次传输都会直接
 *         返回 HAL_BUSY 且不会自己恢复(现象:`g_imu_diag.last_status == 2`,
 *         `fail_cnt` 一直涨)。必须 DeInit + 重新 Init 才能恢复。
 *         本函数可在调试器里手动调用,也已被 imu_task 的重试流程调用。
 */
void Imu_RecoverBus(void);

/**
 * @brief  初始化 IMU(LSM6DSV16XTR):探测 -> 软复位 -> 配陀螺仪/加速度计
 * @retval 0 成功, -1 未找到器件
 */
int Imu_Init(void);

/** @brief 读一帧并刷新 g_imu;0 成功 */
int Imu_Update(void);

/** @brief 当前是否在线 */
uint8_t Imu_IsOnline(void);

/** @brief 当前使用的器件名 */
const char *Imu_SourceName(void);

/** @brief FreeRTOS 采集任务 */
void imu_task(void *argument);

#endif /* YMOUSED1_IMU_H_ */
