/**
  ******************************************************************************
  * @file    imu.c
  * @brief   LSM6DSV16XTR 姿态采集:初始化、单位换算、互补滤波、总线恢复
  *
  *          数据流:
  *            imu_task ──> Imu_Update ──> 后端 read()  得到原始值
  *                                         │
  *                                         ├─ 换算成 °/s 和 g
  *                                         ├─ 互补滤波算出 roll/pitch/yaw
  *                                         └─ 写进 g_imu 供其它任务读
  ******************************************************************************
  */

#include "imu.h"

#include <math.h>
#include <stddef.h>

#include "cmsis_os2.h"
#include "i2c.h"
#include "main.h"

#include "imu_lsm6dsv16x.h"

/* ==========================================================================
 * 模块状态
 * ========================================================================== */

/* 角度与弧度互换 */
#define IMU_DEG_TO_RAD  (3.14159265f / 180.0f)
#define IMU_RAD_TO_DEG  (180.0f / 3.14159265f)

imu_data_t g_imu; /* 对外发布的姿态数据 */

static const imu_backend_t *s_active = NULL; /* 当前生效的器件驱动 */

/* 滤波状态:内部保持全精度,只有发布到 g_imu 时才取整 */
static float s_angle_deg[3];   /* roll / pitch / yaw */
static uint32_t s_last_tick;   /* 上次积分的时刻,用来算真实 dt */
static uint8_t s_angle_seeded; /* 1:已用加速度计定过初始倾角 */

/* ==========================================================================
 * 小工具
 * ========================================================================== */

/**
 * @brief 把角度折算到 (-180, 180]
 * @note  积分是连续累加的,不折算的话航向角会一直涨到几千度
 */
static float imu_wrap_deg(float deg)
{
  while (deg > 180.0f) {
    deg -= 360.0f;
  }
  while (deg <= -180.0f) {
    deg += 360.0f;
  }
  return deg;
}

/**
 * @brief 保留两位小数(0.01° 分辨率)
 * @note  用整数转换代替 roundf():M4F 上一条 vcvt 就够,
 *        而 roundf() 在 newlib-nano 里是库函数调用,每帧三次不划算
 */
static float imu_round2(float v)
{
  float bias = (v >= 0.0f) ? 0.5f : -0.5f;

  return (float)((int32_t)(v * 100.0f + bias)) * 0.01f;
}

/** @brief 几十微秒的短延时,只给手动打 I2C 时钟用 */
static void imu_short_delay(void)
{
  for (volatile uint32_t d = 0U; d < 60U; d++) {
  }
}

/** @brief 毫秒换算成 RTOS tick(不直接引 FreeRTOS 宏,保持与内核解耦) */
static uint32_t imu_ms_to_ticks(uint32_t ms)
{
  uint32_t freq = osKernelGetTickFreq();

  return (freq == 0U) ? ms : ((ms * freq) / 1000U);
}

/** @brief 陀螺零偏默认值,定义在 imu.h(来自静止实测) */
static const float k_gyro_bias_dps[3] = {
  IMU_GYRO_BIAS_X_DPS,
  IMU_GYRO_BIAS_Y_DPS,
  IMU_GYRO_BIAS_Z_DPS,
};

/** @brief 清空对外数据和滤波状态(重新初始化时调用) */
static void imu_reset_data(void)
{
  uint8_t i;

  for (i = 0U; i < 3U; i++) {
    g_imu.gyro_raw[i] = 0;
    g_imu.gyro_dps[i] = 0.0f;

    /* 零偏写入实测默认值,后面每帧从角速度里扣掉 */
    g_imu.gyro_bias_dps[i] = k_gyro_bias_dps[i];

    g_imu.accel_raw[i] = 0;
    g_imu.accel_g[i] = 0.0f;

    s_angle_deg[i] = 0.0f;
  }

  g_imu.roll_deg = 0.0f;
  g_imu.pitch_deg = 0.0f;
  g_imu.yaw_deg = 0.0f;
  g_imu.temp_c = 0.0f;

  s_last_tick = HAL_GetTick();
  s_angle_seeded = 0U; /* 下一帧重新用加速度计定初值 */
}

/* ==========================================================================
 * 角度解算
 * ========================================================================== */

/**
 * @brief 用加速度计感受到的重力方向解算绝对倾角(度)
 * @param roll_deg  输出:横滚角
 * @param pitch_deg 输出:俯仰角
 * @retval 0 成功; -1 此刻加速度不可信(模长偏离 1g 太多,说明在加减速)
 * @note  平放时 ax=0, ay=0, az=+1g:
 *          roll  = atan2(ay, az)          —— 绕 X 轴转,重力分量跑到 Y
 *          pitch = atan2(-ax, √(ay²+az²)) —— 绕 Y 轴转,重力分量跑到 X
 */
static int imu_accel_tilt(float *roll_deg, float *pitch_deg)
{
  float ax = g_imu.accel_g[0];
  float ay = g_imu.accel_g[1];
  float az = g_imu.accel_g[2];
  float norm = sqrtf((ax * ax) + (ay * ay) + (az * az));

  /* 机器人急加速/急刹车时,加速度计读到的不只是重力,这时不能拿它纠偏 */
  if ((norm < IMU_COMP_ACC_MIN_G) || (norm > IMU_COMP_ACC_MAX_G)) {
    return -1;
  }

  *roll_deg = atan2f(ay, az) * IMU_RAD_TO_DEG;
  *pitch_deg = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * IMU_RAD_TO_DEG;

  return 0;
}

/**
 * @brief 更新姿态角(单位:度)
 *
 * 器件自带解算就直接取它的;否则用互补滤波:
 *   陀螺积分  —— 短期准、响应快,但零偏会让角度缓慢漂移 -> 权重 ALPHA
 *   加速度计  —— 长期不会漂,但有振动噪声、加减速时也不准 -> 权重 1-ALPHA
 * 两者加权相加,就得到"既跟得上又不漂"的 roll/pitch。
 *
 * yaw 没有磁力计提供绝对参考,只能纯积分,原理上必然漂,这里不做纠正。
 */
static void imu_update_angles(const imu_sample_t *s)
{
  if (s_active->has_angle != 0U) {
    /* 器件自己会算,直接换算单位 */
    g_imu.roll_deg = (float)s->angle_raw[0] * s_active->angle_lsb_deg;
    g_imu.pitch_deg = (float)s->angle_raw[1] * s_active->angle_lsb_deg;
    g_imu.yaw_deg = (float)s->angle_raw[2] * s_active->angle_lsb_deg;
  } else {
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - s_last_tick) / 1000.0f;
    float roll_acc = 0.0f;
    float pitch_acc = 0.0f;
    uint8_t acc_ok;

    s_last_tick = now;

    /* 只接受合理的采样间隔:调试器停住时 dt 会很大,那一段直接丢掉,免得角度跳变 */
    if ((dt > 0.0f) && (dt < 0.5f)) {
      /* 1) 陀螺积分,负责高频
       *
       * 注意:陀螺读到的是"机体系"角速度 p/q/r,它不等于欧拉角的直接变化率。
       * 两者关系(标准运动学):
       *   φ̇ = p + q·sinφ·tanθ + r·cosφ·tanθ
       *   θ̇ = q·cosφ − r·sinφ
       *   ψ̇ = (q·sinφ + r·cosφ) / cosθ
       * 小角度平飞时二者近似相等,但固定翼压坡度巡航(φ 几十度)时就会串扰,
       * 直接当 φ̇/θ̇/ψ̇ 用会让 pitch 被 yaw 速率带偏。 */
      {
        const float p = g_imu.gyro_dps[0];
        const float q = g_imu.gyro_dps[1];
        const float r = g_imu.gyro_dps[2];

        const float phi = s_angle_deg[0] * IMU_DEG_TO_RAD;
        const float theta = s_angle_deg[1] * IMU_DEG_TO_RAD;
        const float sin_phi = sinf(phi);
        const float cos_phi = cosf(phi);
        const float cos_theta = cosf(theta);

        /* cosθ -> 0(俯仰接近 ±90°)时分母会爆,先夹一下避免除零 */
        float safe_cos = cos_theta;
        if (fabsf(safe_cos) < 0.1f) {
          safe_cos = (cos_theta >= 0.0f) ? 0.1f : -0.1f;
        }

        s_angle_deg[0] += (p + ((q * sin_phi) + (r * cos_phi)) * tanf(theta)) * dt;
        s_angle_deg[1] += ((q * cos_phi) - (r * sin_phi)) * dt;
        s_angle_deg[2] += (((q * sin_phi) + (r * cos_phi)) / safe_cos) * dt;
      }

      /* 2) 加速度计解算绝对倾角,负责低频(纠漂移) */
      acc_ok = (imu_accel_tilt(&roll_acc, &pitch_acc) == 0) ? 1U : 0U;

      if (s_angle_seeded == 0U) {
        /* 第一帧直接用加速度计定初值,省掉从 0° 慢慢爬向真实倾角的过程 */
        if (acc_ok != 0U) {
          s_angle_deg[0] = roll_acc;
          s_angle_deg[1] = pitch_acc;
        }
        s_angle_seeded = 1U;
      } else if (acc_ok != 0U) {
        /* 3) 互补滤波融合 */
        s_angle_deg[0] = (IMU_COMP_ALPHA * s_angle_deg[0]) +
                         ((1.0f - IMU_COMP_ALPHA) * roll_acc);
        s_angle_deg[1] = (IMU_COMP_ALPHA * s_angle_deg[1]) +
                         ((1.0f - IMU_COMP_ALPHA) * pitch_acc);
      }
      /* acc 不可信时这一帧只积分、不纠正:宁可短暂漂,也不要被加速度带歪 */

      g_imu.roll_deg = imu_wrap_deg(s_angle_deg[0]);
      g_imu.pitch_deg = imu_wrap_deg(s_angle_deg[1]);
      g_imu.yaw_deg = imu_wrap_deg(s_angle_deg[2]);
    }
  }

  /* 对外只保留两位小数(滤波状态仍是全精度,不吃截断误差) */
  g_imu.roll_deg = imu_round2(g_imu.roll_deg);
  g_imu.pitch_deg = imu_round2(g_imu.pitch_deg);
  g_imu.yaw_deg = imu_round2(g_imu.yaw_deg);
}

/* ==========================================================================
 * I2C 总线恢复
 * ========================================================================== */

/**
 * @brief 手动打 9 个 SCL 时钟,把被从机拉住的 SDA 释放掉
 * @note  从机在传输中途掉电/复位时可能一直把 SDA 拉在低电平,总线被判"忙",
 *        之后所有传输直接返回 HAL_BUSY。卡住的是从机不是外设,所以
 *        HAL_I2C_DeInit + Init 也救不回来,必须由主机补时钟,
 *        让从机把没发完的位吐完,最后再补一个 STOP。
 */
static void imu_i2c_bus_clear(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t i;

  /* 先把 I2C 外设和引脚放开,改成普通开漏 GPIO 手动驱动 */
  (void)HAL_I2C_DeInit(&hi2c1);

  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  /* SDA 先释放(靠上拉变高) */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

  /* 9 个时钟:足够让任何从机把当前字节吐完并释放 SDA */
  for (i = 0U; i < 9U; i++) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    imu_short_delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    imu_short_delay();
  }

  /* 补一个 STOP:SCL 为高时,SDA 由低变高 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  imu_short_delay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  imu_short_delay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  imu_short_delay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  imu_short_delay();
}

void Imu_RecoverBus(void)
{
  /* 1) 补时钟,把可能被从机拉住的 SDA 释放掉 */
  imu_i2c_bus_clear();

  /* 2) 把 I2C 外设恢复成正常模式(PB6/PB7 切回复用开漏) */
  MX_I2C1_Init();
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

int Imu_Init(void)
{
  const imu_backend_t *b = &imu_backend_lsm6dsv16x;

  s_active = NULL;
  g_imu.online = 0U;

  if (b->init() != 0) {
    return -1;
  }

  s_active = b;
  imu_reset_data();
  g_imu.err_cnt = 0U;
  g_imu.online = 1U;

  return 0;
}

int Imu_Update(void)
{
  imu_sample_t s;
  uint8_t i;

  if (s_active == NULL) {
    return -1;
  }

  /* 器件需要判断数据就绪时,先等就绪标志 */
  if (s_active->wait_ready != NULL) {
    if (s_active->wait_ready(IMU_I2C_TIMEOUT_MS) != 0) {
      g_imu.err_cnt++;
      return -1;
    }
  }

  if (s_active->read(&s) != 0) {
    g_imu.err_cnt++;
    return -1;
  }

  /* 原始值 -> 工程单位 */
  for (i = 0U; i < 3U; i++) {
    g_imu.gyro_raw[i] = s.gyro_raw[i];
    g_imu.gyro_dps[i] =
        ((float)s.gyro_raw[i] * s_active->gyro_lsb_dps) - g_imu.gyro_bias_dps[i];

    g_imu.accel_raw[i] = s.accel_raw[i];
    g_imu.accel_g[i] = (float)s.accel_raw[i] * s_active->accel_lsb_g;
  }

  imu_update_angles(&s);

  g_imu.temp_c = ((float)s.temp_raw * s_active->temp_lsb_c) + s_active->temp_offset_c;

  /* 供上层判断数据流是否还活着、这份数据有多旧 */
  g_imu.frames++;
  g_imu.last_tick = HAL_GetTick();

  return 0;
}

uint8_t Imu_IsOnline(void)
{
  return g_imu.online;
}

/* ==========================================================================
 * 采集任务
 * ========================================================================== */

void imu_task(void *argument)
{
  uint32_t next_tick = osKernelGetTickCount();
  uint8_t err_streak = 0U;

  (void)argument;

  for (;;) {
    /* ---- 还没初始化(s_active == NULL):尝试初始化,失败先恢复总线再重试 ---- */
    if (s_active == NULL) {
      if (Imu_Init() != 0) {
        g_imu.online = 0U;
        Imu_RecoverBus();
        osDelay(IMU_RETRY_DELAY_MS);
        continue;
      }
      err_streak = 0U;
      next_tick = osKernelGetTickCount();
    }

    /* ---- 固定节拍:第 N 帧永远落在 t0 + N*10ms,不受 I2C 耗时影响 ----
     * 用 osDelay 的话周期会变成 "10ms + 本帧工作耗时"(实测约 11.9ms),
     * 实际更新率只有 84Hz 左右,而且会随通信时间抖动。 */
    next_tick += imu_ms_to_ticks(IMU_TASK_PERIOD_MS);
    if ((int32_t)(osKernelGetTickCount() - next_tick) >= 0) {
      /* 已经落后一个周期以上(比如被高优先级任务占住),重新对齐,不补跑 */
      next_tick = osKernelGetTickCount();
    }
    (void)osDelayUntil(next_tick);

    /* ---- 采集一帧 ---- */
    if (Imu_Update() != 0) {
      if (++err_streak >= IMU_ERR_TOLERANCE) {
        /* 连续失败:判定真的掉线,清掉驱动状态回到外层重新初始化 */
        err_streak = 0U;
        s_active = NULL;
        g_imu.online = 0U;
      }
      /* 偶发失败:保留上一帧姿态,不重初始化,避免角度台阶和长时间冻结 */
    } else {
      err_streak = 0U;
    }
  }
}
