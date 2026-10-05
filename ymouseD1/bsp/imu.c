/**
  ******************************************************************************
  * @file    imu.c
  * @brief   姿态传感器适配层
  *
  *          硬件上两种陀螺仪二选一(不会同时使用):
  *            - LSM6DSV16XTR   IIC1 0x6A / 0x6B   (bsp/imu_lsm6dsv16x.c)
  *            - 维特智能 JY901S IIC1 0x50          (bsp/imu_jy901s.c)
  *
  *          本文件负责:选器件、把原始值换算成工程单位、跑采集任务。
  *          应用层只读 g_imu,换器件不用改应用代码。
  ******************************************************************************
  */

#include "imu.h"

#include <stddef.h>

#include "main.h"
#include "i2c.h"
#include "cmsis_os2.h"

#include "imu_lsm6dsv16x.h"
#include "imu_jy901s.h"

/* 统一数据,供其它任务读取 */
imu_data_t g_imu;

/* 诊断信息,供排查通信问题 */
imu_diag_t g_imu_diag;

/* 当前生效的后端 */
static const imu_backend_t *s_active = NULL;

/* 本次启动是否已做过总线扫描(避免一直失败时反复扫总线) */
static uint8_t s_scan_done = 0U;

/* ==========================================================================
 * 内部函数
 * ========================================================================== */

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
  const imu_backend_t *b = NULL;

  s_active = NULL;
  g_imu.online = 0U;
  g_imu.source = NULL;
  g_imu.has_angle = 0U;

#if (IMU_SENSOR_SEL == IMU_SENSOR_LSM6DSV16XTR)
  if (imu_backend_lsm6dsv16x.init() == 0) {
    b = &imu_backend_lsm6dsv16x;
  }
#elif (IMU_SENSOR_SEL == IMU_SENSOR_JY901S)
  if (imu_backend_jy901s.init() == 0) {
    b = &imu_backend_jy901s;
  }
#else /* IMU_SENSOR_AUTO:按顺序探测,谁先应答就用谁 */
  {
    static const imu_backend_t *const cand[2] = {
      &imu_backend_lsm6dsv16x,
      &imu_backend_jy901s,
    };
    uint8_t i;

    for (i = 0U; i < 2U; i++) {
      if (cand[i]->init() == 0) {
        b = cand[i];
        break;
      }
    }
  }
#endif

  if (b == NULL) {
    return -1;
  }

  s_active = b;
  imu_clear_data();
  g_imu.err_cnt = 0U;
  g_imu.source = b->name;
  g_imu.has_angle = b->has_angle;
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

  /* 只有器件自带姿态解算时才更新角度 */
  if (s_active->has_angle != 0U) {
    g_imu.roll_deg = (float)s.angle_raw[0] * s_active->angle_lsb_deg;
    g_imu.pitch_deg = (float)s.angle_raw[1] * s_active->angle_lsb_deg;
    g_imu.yaw_deg = (float)s.angle_raw[2] * s_active->angle_lsb_deg;
  }

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
