/**
  ******************************************************************************
  * @file    imu.c
  * @brief   姿态传感器:LSM6DSV16XTR 的采集与换算
  *
  *          硬件:IIC1 0x6A / 0x6B(SA0 决定),驱动在 bsp/imu_lsm6dsv16x.c
  *
  *          本文件负责:初始化、把原始值换算成工程单位、跑采集任务。
  *          应用层只读 g_imu。
  ******************************************************************************
  */

#include "imu.h"

#include <stddef.h>

#include "main.h"
#include "i2c.h"
#include "cmsis_os2.h"

#include "imu_lsm6dsv16x.h"

/* 统一数据,供其它任务读取 */
imu_data_t g_imu;

/* 诊断信息,供排查通信问题 */
imu_diag_t g_imu_diag;

/* 当前生效的后端 */
static const imu_backend_t *s_active = NULL;

/* 本次启动是否已做过总线扫描(避免一直失败时反复扫总线) */
static uint8_t s_scan_done = 0U;

/* 陀螺仪积分出来的角度(度),以初始化时刻为零位 */
static float s_angle_deg[3];
/* 上一次积分的时间戳(ms),用来算真实采样间隔 */
static uint32_t s_last_tick = 0U;

/* ==========================================================================
 * 内部函数
 * ========================================================================== */

/**
 * @brief 把角度折算到 (-180, 180]
 * @note  积分是连续累加的,不折算的话航向角会一直长到几千度
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
 * @note  用整数转换代替 roundf():M4F 上一次 vcvt 就够,
 *        roundf() 在 newlib-nano 里是库函数调用,每帧三次不划算
 */
static float imu_round2(float v)
{
  float bias = (v >= 0.0f) ? 0.5f : -0.5f;

  return (float)((int32_t)(v * 100.0f + bias)) * 0.01f;
}

static void imu_clear_data(void)
{
  uint8_t i;

  for (i = 0U; i < 3U; i++) {
    g_imu.gyro_raw[i] = 0;
    g_imu.gyro_dps[i] = 0.0f;
    g_imu.gyro_bias_dps[i] = 0.0f;
    g_imu.accel_raw[i] = 0;
    g_imu.accel_g[i] = 0.0f;
  }
  g_imu.roll_deg = 0.0f;
  g_imu.pitch_deg = 0.0f;
  g_imu.yaw_deg = 0.0f;
  g_imu.temp_c = 0.0f;

  /* 积分状态一起清零,初始化后从 0° 开始 */
  for (i = 0U; i < 3U; i++) {
    s_angle_deg[i] = 0.0f;
  }
  s_last_tick = HAL_GetTick();
}

/**
 * @brief 上电零偏标定:采集 samples 帧原始角速度求平均
 * @note  与具体器件无关,标定期间模块必须静止不动
 * @retval 0 成功, -1 失败
 */
static int imu_calibrate_gyro(uint16_t samples)
{
  float sum[3] = { 0.0f, 0.0f, 0.0f };
  imu_sample_t s;
  uint16_t n = 0U;
  uint16_t i;
  uint8_t k;

  if (s_active == NULL) {
    return -1;
  }
  if (samples == 0U) {
    samples = IMU_CALIB_SAMPLES;
  }

  for (i = 0U; i < samples; i++) {
    if (s_active->wait_ready != NULL) {
      if (s_active->wait_ready(IMU_I2C_TIMEOUT_MS) != 0) {
        continue;
      }
    }
    if (s_active->read(&s) != 0) {
      continue;
    }
    for (k = 0U; k < 3U; k++) {
      sum[k] += (float)s.gyro_raw[k] * s_active->gyro_lsb_dps;
    }
    n++;
  }

  if (n == 0U) {
    return -1;
  }
  for (k = 0U; k < 3U; k++) {
    g_imu.gyro_bias_dps[k] = sum[k] / (float)n;
  }
  return 0;
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief 更新姿态角(单位:度)
 *
 * 器件自带姿态解算就直接取它的;否则把角速度(°/s)按真实时间积分成角度(°)。
 * 对外发布的值统一保留两位小数,而积分状态 s_angle_deg[] 保持全精度 ——
 * 每帧都截断的话误差会越积越大。
 */
static void imu_update_angles(const imu_sample_t *s)
{
  uint8_t i;

  if (s_active->has_angle != 0U) {
    g_imu.roll_deg = (float)s->angle_raw[0] * s_active->angle_lsb_deg;
    g_imu.pitch_deg = (float)s->angle_raw[1] * s_active->angle_lsb_deg;
    g_imu.yaw_deg = (float)s->angle_raw[2] * s_active->angle_lsb_deg;
  } else {
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - s_last_tick) / 1000.0f;

    s_last_tick = now;

    /* 只接受合理的采样间隔:调试器停住时 dt 会很大,那一段直接丢掉,免得角度跳变 */
    if ((dt > 0.0f) && (dt < 0.5f)) {
      for (i = 0U; i < 3U; i++) {
        s_angle_deg[i] += g_imu.gyro_dps[i] * dt;
      }
      g_imu.roll_deg = imu_wrap_deg(s_angle_deg[0]);
      g_imu.pitch_deg = imu_wrap_deg(s_angle_deg[1]);
      g_imu.yaw_deg = imu_wrap_deg(s_angle_deg[2]);
    }
  }

  /* 对外保留两位小数(0.01° 分辨率) */
  g_imu.roll_deg = imu_round2(g_imu.roll_deg);
  g_imu.pitch_deg = imu_round2(g_imu.pitch_deg);
  g_imu.yaw_deg = imu_round2(g_imu.yaw_deg);
}

void Imu_RecoverBus(void)
{
  /* HAL 的 I2C 卡住后不会自愈,必须整个复位重来 */
  (void)HAL_I2C_DeInit(&hi2c1);
  MX_I2C1_Init();

  /* 顺手清掉"总线忙"的残留记录,方便区分恢复前后的失败 */
  g_imu_diag.last_status = 0U;
}

uint8_t Imu_ScanBus(void)
{
  uint8_t a;

  g_imu_diag.count = 0U;
  for (a = 1U; a < 128U; a++) {
    /* 只发地址等 ACK,不读写寄存器 */
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)((uint16_t)a << 1), 1U, 5U) == HAL_OK) {
      if (g_imu_diag.count < IMU_SCAN_MAX) {
        g_imu_diag.addr[g_imu_diag.count] = a;
      }
      g_imu_diag.count++;
    }
  }
  return g_imu_diag.count;
}

int Imu_Init(void)
{
  const imu_backend_t *b = &imu_backend_lsm6dsv16x;

  s_active = NULL;
  g_imu.online = 0U;
  g_imu.source = NULL;
  g_imu.has_angle = 0U;

  if (b->init() != 0) {
    return -1;
  }

  s_active = b;
  imu_clear_data();
  g_imu.err_cnt = 0U;
  g_imu.source = b->name;
  /* 角度总是有的:器件自带解算就直接用,否则由角速度积分得到 */
  g_imu.has_angle = 1U;
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

  for (i = 0U; i < 3U; i++) {
    g_imu.gyro_raw[i] = s.gyro_raw[i];
    /* 原始角速度换算成 °/s 后扣掉零偏 */
    g_imu.gyro_dps[i] =
        ((float)s.gyro_raw[i] * s_active->gyro_lsb_dps) - g_imu.gyro_bias_dps[i];

    g_imu.accel_raw[i] = s.accel_raw[i];
    g_imu.accel_g[i] = (float)s.accel_raw[i] * s_active->accel_lsb_g;
  }

  imu_update_angles(&s);

  g_imu.temp_c = ((float)s.temp_raw * s_active->temp_lsb_c) + s_active->temp_offset_c;

  return 0;
}

uint8_t Imu_IsOnline(void)
{
  return g_imu.online;
}

const char *Imu_SourceName(void)
{
  return (s_active != NULL) ? s_active->name : "none";
}

void imu_task(void *argument)
{
  (void)argument;

  for (;;) {
    /* ---------------- 选器件并初始化,失败则周期性重试 ---------------- */
    if (Imu_Init() != 0) {
      g_imu.online = 0U;
      /* I2C 一旦卡在 BUSY 就不会自愈,每次重试前先复位总线 */
      Imu_RecoverBus();
      /* 第一次失败时扫一遍总线,把结果留在 g_imu_diag 里,方便在调试器里直接看 */
      if (s_scan_done == 0U) {
        (void)Imu_ScanBus();
        s_scan_done = 1U;
      }
      osDelay(IMU_RETRY_DELAY_MS);
      continue;
    }
    s_scan_done = 0U; /* 成功了,下次再失败就重新扫 */

    /* ---------------- 需要时做上电零偏标定(此刻模块必须静止) ---------------- */
    if (s_active->need_gyro_calib != 0U) {
      (void)imu_calibrate_gyro(IMU_CALIB_SAMPLES);
    }

    /* ---------------- 周期采集 ---------------- */
    for (;;) {
      if (Imu_Update() != 0) {
        g_imu.online = 0U; /* 读不到数据,回去重新选器件/初始化 */
        break;
      }
      osDelay(IMU_TASK_PERIOD_MS);
    }

    osDelay(IMU_RETRY_DELAY_MS);
  }
}
