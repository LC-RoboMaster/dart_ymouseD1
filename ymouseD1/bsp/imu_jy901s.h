#ifndef YMOUSED1_IMU_JY901S_H_
#define YMOUSED1_IMU_JY901S_H_

#include <stdint.h>

#include "imu.h" /* imu_backend_t */

/* ==========================================================================
 * 维特智能 JY901S 九轴姿态模块 —— 后端驱动(I2C 接口)
 *
 * 硬件:IIC1 —— PB6(SCL) / PB7(SDA),从机地址 0x50
 *
 * 寄存器定义来源:维特智能上位机自带的 JY901 寄存器表
 *   WitMotion.exe 安装目录 \Config\ConfigurationSpace\RegEditor\JY901.xml
 *   与官方《WIT 标准通信协议》一致。
 *
 * 数据寄存器(每个 16 位、小端,地址按 1 递增):
 *   0x34~0x36  AX / AY / AZ      加速度 = raw * 16.0   / 32768  (g)
 *   0x37~0x39  GX / GY / GZ      角速度 = raw * 2000.0 / 32768  (°/s)
 *   0x3A~0x3C  HX / HY / HZ      磁场   = raw (LSB)
 *   0x3D~0x3F  角度 X / Y / Z     姿态角 = raw * 180.0  / 32768  (°)
 *   0x40       温度               = raw / 100                   (℃)
 * 故从 0x34 连读 26 字节即可一次拿到全部数据(同一时刻快照)。
 * ========================================================================== */

/* ---------------- 从机地址 ---------------- */
#define JY901S_I2C_ADDR_7BIT   0x50U /* 出厂默认,即寄存器 0x1A 的默认值 0x0050 */
#define JY901S_ADDR            (JY901S_I2C_ADDR_7BIT << 1)

/* ---------------- 数据寄存器 ---------------- */
#define JY901S_REG_AX          0x34U /* 起始:AX AY AZ       (6 字节) */
#define JY901S_REG_GX          0x37U /* 起始:GX GY GZ       (6 字节) */
#define JY901S_REG_HX          0x3AU /* 起始:HX HY HZ       (6 字节,九轴有效) */
#define JY901S_REG_ROLL        0x3DU /* 起始:角度 X/Y/Z     (6 字节) */
#define JY901S_REG_TEMP        0x40U /* 温度                (2 字节) */
/* 0x34~0x40 共 13 个寄存器 = 26 字节,一次读全 */
#define JY901S_REG_BLOCK       0x34U
#define JY901S_BLOCK_BYTES     26U

/* ---------------- 配置 / 信息寄存器 ---------------- */
#define JY901S_REG_SAVE        0x00U /* 保存配置到模块内部 Flash */
#define JY901S_REG_CALSW       0x01U /* 校准模式 */
#define JY901S_REG_OUTPUT      0x02U /* 输出内容(串口) */
#define JY901S_REG_RATE        0x03U /* 输出速率(串口) */
#define JY901S_REG_BAUD        0x04U /* 串口波特率 */
#define JY901S_REG_IICADDR     0x1AU /* 设备地址,默认 0x0050 */
#define JY901S_REG_BANDWIDTH   0x1FU /* 带宽,见 JY901S_BW_xxx */
#define JY901S_REG_GYR_RANGE   0x20U /* 陀螺仪量程 */
#define JY901S_REG_ACC_RANGE   0x21U /* 加速度量程 */
#define JY901S_REG_SLEEP       0x22U /* 休眠 */
#define JY901S_REG_ORIENT      0x23U /* 安装方向 */
#define JY901S_REG_ALGO        0x24U /* 算法 */
#define JY901S_REG_VERSION     0x2EU /* 版本号 */
#define JY901S_REG_VOLTAGE     0x5CU /* 供电电压 = raw / 100 (V) */
#define JY901S_REG_KEY         0x69U /* 解锁寄存器,写 0x88 0xB5 */

#define JY901S_KEY_H           0x88U
#define JY901S_KEY_L           0xB5U
#define JY901S_CAL_ANGLE       0x08U /* 校准模式:角度归零(含航向角) */
#define JY901S_CAL_ZAXIS       0x04U /* 校准模式:Z 轴归零 */

#define JY901S_ZERO_DELAY_MS   200U  /* 归零/保存后等模块写入内部 Flash */

/* 带宽寄存器(0x1F)取值 -> 实际带宽 */
#define JY901S_BW_256HZ        0x00U
#define JY901S_BW_188HZ        0x01U
#define JY901S_BW_98HZ         0x02U
#define JY901S_BW_42HZ         0x03U
#define JY901S_BW_20HZ         0x04U
#define JY901S_BW_10HZ         0x05U
#define JY901S_BW_5HZ          0x06U

/* 上电时是否自动把姿态角归零:1 归零, 0 不归零(默认) */
#define JY901S_AUTO_ZERO_ON_INIT  0U

/* ==========================================================================
 * 模块实际配置(上电时从模块读回,方便和驱动里的假设逐项对照)
 *
 * 典型的"对不上"就有问题:
 *   iic_addr  != 0x0050  -> 模块地址被改过
 *   gyr_range != 默认值   -> 角速度换算系数(2000)要跟着改
 *   acc_range != 默认值   -> 加速度换算系数(16)要跟着改
 *   voltage_v 明显偏低    -> 供电或接线问题
 * ========================================================================== */
typedef struct {
  uint8_t  valid;        /* 1:下面这些值已成功读回 */
  uint16_t version;      /* 0x2E 版本号 */
  uint16_t iic_addr;     /* 0x1A 设备地址,期望 0x0050 */
  uint16_t bandwidth;    /* 0x1F 带宽,见 JY901S_BW_xxx */
  uint16_t gyr_range;    /* 0x20 陀螺仪量程 */
  uint16_t acc_range;    /* 0x21 加速度量程 */
  uint16_t orient;       /* 0x23 安装方向 */
  uint16_t algo;         /* 0x24 算法 */
  float    voltage_v;    /* 0x5C 供电电压,V */
} jy901s_info_t;

extern jy901s_info_t g_jy901s_info;

/* ==========================================================================
 * 器件专有接口(通用数据请看 bsp/imu.h 的 g_imu)
 * ========================================================================== */

/** @brief 把模块配置读回到 g_jy901s_info;0 成功(至少读回一项) */
int Jy901s_ReadInfo(void);

/** @brief 姿态角归零(含航向角),并保存到模块内部 Flash;0 成功 */
int Jy901s_SetAngleZero(void);

/** @brief 把当前配置保存到模块内部 Flash;0 成功 */
int Jy901s_SaveConfig(void);

/** @brief 后端实例(供 bsp/imu.c 的适配层使用) */
extern const imu_backend_t imu_backend_jy901s;

#endif /* YMOUSED1_IMU_JY901S_H_ */
