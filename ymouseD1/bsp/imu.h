#ifndef YMOUSED1_IMU_H_
#define YMOUSED1_IMU_H_

#include <stdint.h>

/* ==========================================================================
 * 姿态传感器接口
 *
 * 硬件:LSM6DSV16XTR 六轴 IMU,挂在 IIC1(PB6 = SCL,PB7 = SDA),
 *       地址 0x6A(SA0 接 GND)/ 0x6B(SA0 接 VDD),初始化时自动探测。
 *
 * 应用层只包含本文件:直接读 g_imu,或调用 Imu_* 接口。
 * 具体寄存器操作在 imu_lsm6dsv16x.c 里,本文件不碰硬件细节。
 * ========================================================================== */

/* ---------------- 运行参数 ---------------- */
#define IMU_I2C_TIMEOUT_MS   50U  /* 单次 I2C 传输超时(ms) */
#define IMU_TASK_PERIOD_MS   10U  /* 采集周期 10ms -> 100Hz */
#define IMU_RETRY_DELAY_MS   500U /* 初始化失败后的重试间隔(ms) */

/* 连续通信失败多少次,才判定器件真的掉线。
 * 偶发失败(EMI、总线毛刺)只丢弃这一帧并重试,不重初始化 ——
 * 否则一次抖动就会冻结几百毫秒,且清掉滤波状态导致姿态角跳变。 */
#define IMU_ERR_TOLERANCE    5U

/* ---------------- 陀螺零偏(°/s) ---------------- */
/* 上电不做自动标定,这里直接给实测值,初始化时写进 g_imu.gyro_bias_dps[],
 * Imu_Update() 里从每一帧的角速度中扣掉。
 *
 * Z 轴(航向)的取值来自静止实测:150s 内 yaw 漂 18.3°,
 *   线性拟合 f(t) = 0.1221·t − 0.1499,R²≈1.000、RMSE=0.032
 * => 零偏 0.1221 °/s。
 *
 * 注意:陀螺零偏随温度变化,这是常温(~27℃)下的值。
 * X/Y 轴不给值(0),因为 roll/pitch 有加速度计通道长期纠正,不需要。 */
#define IMU_GYRO_BIAS_X_DPS  0.0f
#define IMU_GYRO_BIAS_Y_DPS  0.0f
#define IMU_GYRO_BIAS_Z_DPS  0.1221f

/* ---------------- 互补滤波参数 ---------------- */
/* 角度 = ALPHA * 陀螺积分 + (1 - ALPHA) * 加速度计解算
 *   ALPHA 调大 -> 更信陀螺:更平滑,但纠正漂移慢
 *   ALPHA 调小 -> 更信加速度计:纠漂快,但振动噪声会进来
 *   0.98 @100Hz 约等于 0.5s 的时间常数 */
#define IMU_COMP_ALPHA       0.98f

/* 加速度模长必须落在这个区间,才认为"此刻只有重力、没有剧烈加减速" */
#define IMU_COMP_ACC_MIN_G   0.70f
#define IMU_COMP_ACC_MAX_G   1.30f

/* ==========================================================================
 * 后端(具体器件驱动)描述
 * ========================================================================== */

/** @brief 一帧原始数据,由后端填好交回来 */
typedef struct {
  int16_t accel_raw[3]; /* 加速度原始值 X/Y/Z */
  int16_t gyro_raw[3];  /* 角速度原始值 X/Y/Z */
  int16_t angle_raw[3]; /* 器件内部解算的姿态角;器件不支持时保持 0 */
  int16_t temp_raw;     /* 温度原始值 */
} imu_sample_t;

/**
 * @brief 后端描述表:一个器件一份,适配层按它做换算和调度
 * @note  lsb_* 表示"1 个原始 LSB 对应多少工程单位"
 */
typedef struct {
  const char *name;        /* 器件名 */
  uint8_t addr_7bit;       /* 从机地址(7 位) */
  uint8_t has_angle;       /* 1:器件自己输出姿态角,不需要外部解算 */

  float accel_lsb_g;       /* g    / LSB */
  float gyro_lsb_dps;      /* °/s  / LSB */
  float angle_lsb_deg;     /* °    / LSB */
  float temp_lsb_c;        /* ℃    / LSB */
  float temp_offset_c;     /* 温度换算的固定偏置(LSM6DSV16XTR 为 25℃) */

  /** 探测并初始化器件;0 成功,器件不在时返回 -1 */
  int (*init)(void);
  /** 等待新数据就绪;0 就绪。器件不需要判断时为 NULL */
  int (*wait_ready)(uint32_t timeout_ms);
  /** 读一帧原始数据;0 成功 */
  int (*read)(imu_sample_t *s);
} imu_backend_t;

/** @brief 当前唯一的后端实例,定义在 imu_lsm6dsv16x.c */
extern const imu_backend_t imu_backend_lsm6dsv16x;

/* ==========================================================================
 * 对外数据
 * ========================================================================== */

typedef struct {
  uint8_t online;          /* 1:在线 0:离线 */
  uint32_t frames;         /* 累计成功更新帧数:看它涨不涨就知道数据流是否还活着 */
  uint32_t last_tick;      /* 最近一次成功更新的时刻(ms),可算出这份数据有多旧 */
  uint32_t err_cnt;        /* 累计 I2C 通信错误次数 */

  int16_t gyro_raw[3];     /* 角速度原始值 */
  float gyro_dps[3];       /* 角速度,°/s(已减去 gyro_bias_dps) */

  /* 零偏,°/s。上电不再自动标定,默认 0;
   * 需要补偿漂移时可直接在调试器里写这三个值 */
  float gyro_bias_dps[3];

  int16_t accel_raw[3];    /* 加速度原始值 */
  float accel_g[3];        /* 加速度,g */

  /* ---- 姿态角,单位:度(°),已折算到 (-180, 180] ---- */
  /* roll / pitch:由陀螺积分 + 加速度计做互补滤波,不会漂,
   *               读数就是相对重力方向的绝对倾角。 */
  /* yaw:没有磁力计做绝对参考,只能纯积分,长时间会漂。 */
  float roll_deg;
  float pitch_deg;
  float yaw_deg;

  float temp_c;            /* 器件温度,℃ */
} imu_data_t;

/** @brief 姿态数据,由 imu 任务周期性刷新,其它任务只读 */
extern imu_data_t g_imu;

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief  初始化 IMU:探测 -> 软复位 -> 配置陀螺仪/加速度计
 * @retval 0 成功, -1 未找到器件
 */
int Imu_Init(void);

/**
 * @brief  读一帧,换算成工程单位并刷新 g_imu
 * @retval 0 成功, -1 通信失败
 */
int Imu_Update(void);

/** @brief 是否在线(1:在线) */
uint8_t Imu_IsOnline(void);

/**
 * @brief  恢复 I2C1 总线
 * @note  从机在传输中途掉电/复位时可能一直把 SDA 拉到低电平,总线被判"忙",
 *        之后所有传输直接返回 HAL_BUSY,光 DeInit + Init 救不回来。
 *        本函数先补 9 个 SCL 时钟让从机释放总线,再重新初始化外设。
 *        imu_task 每次重试前都会调用,也可以在调试器里手动调用。
 */
void Imu_RecoverBus(void);

/** @brief FreeRTOS 采集任务 */
void imu_task(void *argument);

#endif /* YMOUSED1_IMU_H_ */
