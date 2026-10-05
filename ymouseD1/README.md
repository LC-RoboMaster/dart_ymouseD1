# ymouseD1

基于 **STM32F401CCU6**（UFQFPN48）的机器人主控固件，使用 STM32CubeMX + CMake 生成，跑 FreeRTOS（CMSIS-RTOS2 接口）。

主要外设：SPI1（与 ESP32-P4 通信）、I2C1（姿态传感器）、TIM3（4 路舵机 PWM）、USART2（串口）。

---

## 1. 目录结构

```
app/                        应用层
  robomaster.c/.h           master 任务入口
  dart_motor.c/.h           4 路舵机初始化与控制
  message.c/.h              姿态数据串口上报(当前整体注释)
  robot_data.h              共享数据(预留)
bsp/                        板级驱动
  imu.c/.h                  姿态传感器适配层(统一接口)
  imu_lsm6dsv16x.c/.h       后端①:LSM6DSV16XTR
  imu_jy901s.c/.h           后端②:维特智能 JY901S
  servo_motor.c/.h          舵机驱动
Core/                       CubeMX 生成(MSP、外设初始化、FreeRTOS)
Drivers/                    STM32F4 HAL + CMSIS
Middlewares/                FreeRTOS
cmake/, CMakeLists.txt      构建脚本
tools/                      调试配置:Ozone 工程 + OpenOCD 配置与脚本
ymouseD1.ioc                CubeMX 工程文件
```

---

## 2. 引脚配置

依据引脚配置图，并经 `ymouseD1.ioc` 与生成代码核对一致。

### STM32F401CCU6

| 引脚 | 功能 | 外设 | 对端 | main.h 宏 |
|---|---|---|---|---|
| PA5 | SPI1_SCK | SPI1 | ESP32-P4 | `PA5_SCK_Pin` / `PA5_SCK_GPIO_Port` |
| PA6 | SPI1_MISO | SPI1 | ESP32-P4 | `PA6_MISO_Pin` / `PA6_MISO_GPIO_Port` |
| PA7 | SPI1_MOSI | SPI1 | ESP32-P4 | `PA7_MOSI_Pin` / `PA7_MOSI_GPIO_Port` |
| PB6 | I2C1_SCL | IIC1 | 姿态传感器 | `PB6_SCL_Pin` / `PB6_SCL_GPIO_Port` |
| PB7 | I2C1_SDA | IIC1 | 姿态传感器 | `PB7_SDA_Pin` / `PB7_SDA_GPIO_Port` |
| PB4 | TIM3_CH1 | TIM3 | 舵机 1 | `PB4_CH1_Pin` / `PB4_CH1_GPIO_Port` |
| PB5 | TIM3_CH2 | TIM3 | 舵机 2 | `PB5_CH2_Pin` / `PB5_CH2_GPIO_Port` |
| PB0 | TIM3_CH3 | TIM3 | 舵机 3 | `PB0_CH3_Pin` / `PB0_CH3_GPIO_Port` |
| PB1 | TIM3_CH4 | TIM3 | 舵机 4 | `PB1_CH4_Pin` / `PB1_CH4_GPIO_Port` |
| PA2 | USART2_TX | USART2 | 串口(预留) | — |
| PA3 | USART2_RX | USART2 | 串口(预留) | — |
| PA13 | SWDIO | SYS | 调试 | — |
| PA14 | SWCLK | SYS | 调试 | — |
| PH0 / PH1 | OSC_IN / OSC_OUT | RCC | 外部晶振 | — |

> 图中 `PA2/PA3` 未出现（USART2 为工程内已有配置，保留未改），引脚标签按图加了 CubeMX User Label，见 `ymouseD1.ioc` 的 `*.GPIO_Label`。
>
> ⚠️ **这些标签必须在 CubeMX 的 GUI 里加**（Pinout 视图 → 右键引脚 → *Enter User Label*）。
> 直接手改 `.ioc` 里的 `GPIO_Label`，只要在 CubeMX 里点一次 Generate，就会以它内存里的
> 项目状态整体覆盖 —— `.ioc` 的标签和 `main.h` 里的宏一起消失。本工程已经被覆盖过一次，现已恢复。

### 图上其余部分（不属 STM32 引脚）

- ESP32-P4 ← SDIO(IO39~IO44) → TF 卡
  IO39=DAT0、IO40=DAT1、IO41=DAT2、IO42=CD/DAT3、IO43=CLK、IO44=CMD（ESP32-P4 侧 GPIO 编号）
- ESP32-P4 → CSI（摄像头接口）

---

## 3. 外设配置

| 外设 | 配置 | 生成文件 |
|---|---|---|
| SPI1 | 主机、全双工、8 bit、CPOL=Low/CPHA=1Edge、不分频系数 16（约 5.25 Mbit/s）、软件 NSS | `Core/Src/spi.c` |
| I2C1 | 100 kHz、7 位地址、占空比 2、时钟拉伸允许；**PB6/PB7 开漏 + 内部上拉**（`GPIO_InitStruct.Pull = GPIO_PULLUP`） | `Core/Src/i2c.c` |
| TIM3 | 4 通道 PWM，预分频 84-1（1 MHz 计数）、周期 1000-1 | `Core/Src/tim.c` |
| USART2 | 9600 8N1，TX 走 DMA1_Stream6(CH4)，使能 USART2 中断 | `Core/Src/usart.c`、`Core/Src/dma.c` |
| 系统时基 | TIM1 | `Core/Src/stm32f4xx_hal_timebase_tim.c` |

时钟：HSI 16 MHz → PLL(M=8, N=84, P=2) → **SYSCLK 84 MHz**，AHB 84 MHz，APB1 42 MHz，APB2 84 MHz。

> ⚠️ **I2C 必须有上拉**。I2C 是开漏总线，SCL/SDA 没有上拉就会悬空，I2C 外设会一直
> 认为总线忙，每次传输直接返回 `HAL_BUSY`（详见 5.8）。
> 本工程已在 CubeMX 里把 PB6/PB7 设成内部上拉（`.ioc` 里 `*.GPIO_Pu=GPIO_PULLUP`）。
> 内部上拉只有约 40 kΩ，偏弱，**强烈建议再各加一个 4.7 kΩ 到 3.3 V**。

---

## 4. FreeRTOS 任务

CMSIS-RTOS2，`configTOTAL_HEAP_SIZE = 16384`，栈溢出检查 + malloc 失败钩子已开。

| 任务 | 栈 | 优先级 | 说明 |
|---|---|---|---|
| `defaultTask` | 128×4 | Low | 喂独立看门狗 IWDG（LSI 32 kHz/64 → 500 Hz，重载 400 ≈ 0.8 s） |
| `master` | 512×4 | High | `master_task()`，当前调用 `dart_motor_task()` |
| `imu` | 512×4 | Normal | `imu_task()`，姿态传感器采集，见第 5 节 |
| `message` | 512×4 | Low | `message_task()`，串口上报（函数体当前整体注释） |

> ⚠️ **任何任务都不能长期不让出 CPU**。FreeRTOS 是抢占式的，`master` 优先级最高
> （High），一旦它调用的函数不返回（例如写了 `for (;;) {}` 空循环），`imu` /
> `message` / `defaultTask` 会被**永久饿死** —— 表现出来就是"姿态传感器读不到数据"、
> "看门狗没启动"，而调试器里看 `CPU running...` 一切正常。
>
> 本工程踩过一次：`dart_motor_task()` 里原本是
> ```c
> for (;;) { void dart_control(); }   /* 那句是"函数声明",不是调用 → 空死循环 */
> ```
> 已改成一次性执行，由 `master_task` 的 `osDelay(1)` 驱动。
>
> **排查手法**：调试器里目标变量全是 0、`fail_cnt` 等计数器也一直不动、却显示
> `CPU running`，就先怀疑这个。

---

## 5. 姿态传感器适配层

板上两种姿态传感器**二选一使用，不会同时挂载**，代码通过统一接口适配，应用层换器件不用改。

### 5.1 支持的两个后端

| | LSM6DSV16XTR | 维特智能 JY901S |
|---|---|---|
| 类型 | 六轴 IMU（原始数据） | 九轴姿态模块（内部已解算，带磁力计） |
| 总线 / 地址 | IIC1，`0x6A`(SA0=0) / `0x6B`(SA0=1) | IIC1，`0x50` |
| 器件识别 | `WHO_AM_I(0x0F) == 0x70` | 读版本号 `0x2E`（失败再退回读姿态角） |
| 姿态角输出 | 无（`roll/pitch/yaw` 保持 0） | 有，Roll/Pitch/Yaw |
| 上电零偏标定 | 需要（默认 200 点） | 不需要（出厂已标定） |
| 数据就绪 | 轮询 `STATUS_REG.GDA` | 寄存器持续刷新，无需等待 |
| 温度换算 | `raw/256 + 25 ℃` | `raw/100 ℃` |

### 5.2 代码结构

```
bsp/imu.h                统一接口与数据结构(应用层只看这里)
bsp/imu.c                适配层:选器件 + 单位换算 + 采集任务(不碰硬件)
bsp/imu_lsm6dsv16x.c/.h  后端①:LSM6DSV16XTR 寄存器驱动
bsp/imu_jy901s.c/.h       后端②:JY901S 寄存器驱动
```

适配层由一张描述表驱动两个后端（见 `bsp/imu.h` 的 `imu_backend_t`），把两端差异（地址、是否有角度、是否需标定、各量程灵敏度、温度偏置、就绪判断函数指针）全部收进表里。

### 5.3 如何选择器件

在 `bsp/imu.h` 修改 `IMU_SENSOR_SEL`：

```c
#define IMU_SENSOR_SEL  IMU_SENSOR_AUTO            /* 默认:上电自动探测,谁应答用谁 */
/* 或写死其中一个: */
/* #define IMU_SENSOR_SEL  IMU_SENSOR_LSM6DSV16XTR */
/* #define IMU_SENSOR_SEL  IMU_SENSOR_JY901S */
```

也可以用编译期宏覆盖（不改文件）：`-DIMU_SENSOR_SEL=1`（LSM6DSV16XTR）、`=2`（JY901S）、`=0`（AUTO）。

> AUTO 的探测顺序是 **LSM6DSV16XTR → JY901S**。JY901S 没有 ID 寄存器、只能靠"0x50 有应答"判断，因此若板上同时焊了两颗、或 0x50 挂了别的器件，AUTO 可能选错，这种情况请直接写死。

### 5.4 应用层怎么用

只包含 `bsp/imu.h`，读全局 `g_imu`：

```c
#include "imu.h"

if (g_imu.online) {
  /* 角速度,°/s(已减零偏) */
  float wx = g_imu.gyro_dps[0];
  /* 加速度,g */
  float ax = g_imu.accel_g[0];
  /* 姿态角,° —— 仅当 g_imu.has_angle 为 1(JY901S)时有效 */
  float yaw = g_imu.yaw_deg;
  /* 器件温度,℃;以及当前用的是哪颗 */
  float t = g_imu.temp_c;
  const char *src = g_imu.source;   /* "LSM6DSV16XTR" / "JY901S" / NULL */
}
```

`g_imu` 字段：`online`、`has_angle`、`source`、`err_cnt`、`gyro_raw[3]`、`gyro_dps[3]`、`gyro_bias_dps[3]`、`accel_raw[3]`、`accel_g[3]`、`roll_deg`、`pitch_deg`、`yaw_deg`、`temp_c`。

接口：`Imu_Init()`、`Imu_Update()`、`Imu_IsOnline()`、`Imu_SourceName()`、`imu_task()`（任务由 `freertos.c` 创建）。

### 5.5 运行流程

`imu_task()`：按 `IMU_SENSOR_SEL` 选器件并初始化 → 失败则**先 `Imu_RecoverBus()` 复位总线**
再扫一遍总线（见 5.8），然后每 500 ms 重试 → 需要时做零偏标定（此刻**模块必须静止**）
→ 以 100 Hz 采集刷新 `g_imu` → 连续读失败则判离线并重新初始化。

### 5.6 两个后端的寄存器

**LSM6DSV16XTR**（寄存器资料：ST 官方 `lsm6dsv16x_reg.h/.c`）

| 寄存器 | 内容 |
|---|---|
| `0x0F` | WHO_AM_I，固定 `0x70` |
| `0x11` CTRL2 | `odr_g[3:0]` + `op_mode_g[6:4]`，默认 240 Hz + 高性能模式 |
| `0x15` CTRL6 | `fs_g[3:0]` + `lpf1_g_bw[6:4]`，默认 ±2000 dps + MEDIUM |
| `0x16` CTRL7 | `lpf1_g_en` bit0，默认使能 |
| `0x12` CTRL3 | `sw_reset` / `if_inc` / `bdu` |
| `0x1E` | STATUS_REG，`gda` bit1 数据就绪 |
| `0x22`~`0x2D` | 陀螺仪 X/Y/Z + 加速度计 X/Y/Z（各 16 位小端，一次连读 12 字节） |
| `0x20` | 温度 |

灵敏度：2000 dps → `70.0` mdps/LSB；4 g → `0.122` mg/LSB。
配置宏都在 `bsp/imu_lsm6dsv16x.h`（`IMU_GY_ODR`、`IMU_GY_FS`、`IMU_GY_LPF_BW` 等）。

**JY901S**（寄存器资料：维特智能上位机自带的 JY901 寄存器表 —— 安装目录下 `Config\ConfigurationSpace\RegEditor\JY901.xml`，与官方《WIT 标准通信协议》一致）

数据寄存器（每个 16 位、小端，地址按 1 递增）：

| 地址 | 内容 | 换算 |
|---|---|---|
| `0x34`~`0x36` | AX / AY / AZ | `raw*16/32768` g |
| `0x37`~`0x39` | GX / GY / GZ | `raw*2000/32768` °/s |
| `0x3A`~`0x3C` | HX / HY / HZ | 磁场（九轴有效，LSB） |
| `0x3D`~`0x3F` | 角度 X / Y / Z | `raw*180/32768` ° |
| `0x40` | 温度 | `raw/100` ℃ |

故从 `0x34` 连读 26 字节即可一次拿到全部数据（同一时刻快照）。

配置 / 信息寄存器（驱动上电时会读回来，见下表）：

| 地址 | 内容 | 备注 |
|---|---|---|
| `0x00` | 保存 | 写 `0x00 0x00` 存到模块内部 Flash |
| `0x01` | 校准模式 | `0x08` = 角度归零，`0x04` = Z 轴归零 |
| `0x1A` | 设备地址 | 出厂默认 `0x0050` |
| `0x1F` | 带宽 | `0`~`6` → 256/188/98/**42**/20/10/5 Hz |
| `0x20` | **陀螺仪量程** | 改过就要改角速度换算系数 |
| `0x21` | **加速度量程** | 改过就要改加速度换算系数 |
| `0x23` / `0x24` | 安装方向 / 算法 | — |
| `0x2E` | 版本号 | 驱动用它做“器件在不在”的探测 |
| `0x5C` | 供电电压 | `raw/100` V |
| `0x69` | 解锁 | 写 `0x88 0xB5` |

> 注意：**量程寄存器是 `0x20`（陀螺仪）/ `0x21`（加速度）**，不是 `0x1D`/`0x1E` ——
> `0x1D`/`0x1E` 是磁场 Y/Z 校准范围。上面的 `16 g` / `2000 °/s` 是出厂默认量程，
> 若被改过，`imu_backend_jy901s` 里的换算系数要跟着改。

### 5.7 读回模块实际配置

上电初始化时驱动会把模块配置读进 `g_jy901s_info`（定义在 `bsp/imu_jy901s.h`），
方便和驱动里的假设逐项对照：

| 字段 | 来源寄存器 | 判读 |
|---|---|---|
| `iic_addr` | `0x1A` | 不是 `0x0050` → 模块地址被改过 |
| `bandwidth` | `0x1F` | 见上面的带宽枚举 |
| `gyr_range` / `acc_range` | `0x20` / `0x21` | 与默认不同 → 换算系数要改 |
| `orient` / `algo` | `0x23` / `0x24` | — |
| `version` | `0x2E` | 顺便做在线判断 |
| `voltage_v` | `0x5C` | 明显偏低 → 供电或接线问题 |
| `valid` | — | 1 表示上面这些值已成功读回 |

磁场（`0x3A`~`0x3C`）虽然是九轴有效数据，但当前 `imu_sample_t` 没有承载它，
驱动读这 26 字节时只是跳过。需要的话可以加进统一数据结构。

JY901S 专有接口在 `bsp/imu_jy901s.h`：

- `Jy901s_ReadInfo()` — 把模块配置读回 `g_jy901s_info`
- `Jy901s_SetAngleZero()` — 姿态角（含航向角）归零并保存到模块内部 Flash
- `Jy901s_SaveConfig()` — 保存当前配置
- `JY901S_AUTO_ZERO_ON_INIT` — 是否上电自动归零，默认 `0`（不归零）

### 5.8 读不到数据时怎么定位

适配层内置了诊断信息，**先看 `g_imu_diag`（定义在 `bsp/imu.h`），不要靠猜**：

| 字段 | 含义 |
|---|---|
| `count` | 总线扫描时应答的器件个数 |
| `addr[]` | 应答的 7 位地址 |
| `last_addr` | 最近一次失败的从机地址（7 位）。失败地址是 `80`，即 `0x50` |
| `last_status` | 最近一次失败的 HAL 返回值 |
| `fail_cnt` | 累计失败次数 |

`last_status` 最有价值，三种取值指向完全不同的原因：

| 值 | 含义 | 往哪查 |
|---|---|---|
| `1` `HAL_ERROR` | 从机没给 ACK | 地址不对 / 模块没接 / 模块没上电 |
| `2` `HAL_BUSY` | 外设认为**总线一直忙**（SCL/SDA 被拉在低电平） | **上拉电阻** / 线接反或短路 / 总线被从机拉住 |
| `3` `HAL_TIMEOUT` | 传输超时 | 总线被拉死 / 速率过高 |

配套的两个函数（也可以在调试器里随时手动调用）：

- **`Imu_ScanBus()`** — 扫 I2C1 上所有 7 位地址，结果写进 `g_imu_diag.count` / `addr[]`。
  `imu_task` 在**第一次初始化失败时会自动调一次**。
  - `count == 0` → 总线上一个器件都不应答 → 电气问题（上拉 / 接线 / 供电）
  - `count > 0` 但不含 `0x50` → 模块不在默认地址（`IICADDR 0x1A` 被改过）
- **`Imu_RecoverBus()`** — `HAL_I2C_DeInit()` + `MX_I2C1_Init()`。
  **HAL 的 I2C 卡住后不会自愈**，之后每次传输都直接返回 `HAL_BUSY`；
  `imu_task` 每次重试前都会调它。

**本项目真实踩过的两个坑**（症状都是"读不到数据"，但原因完全不同）：

| 现象 | 真正原因 |
|---|---|
| `g_imu_diag` 全 0，连 `fail_cnt` 都不涨；调试器显示 `CPU running` | **`imu_task` 一次都没跑** —— 高优先级任务不让出 CPU 把它饿死了（见第 4 节） |
| `last_addr = 80`、`last_status = 2`、`fail_cnt` 一直涨、`count = 0` | **总线悬空** —— `i2c.c` 原来是 `GPIO_NOPULL`，开漏却没上拉，I2C 外设一直判忙 |

> 内部上拉只有约 40 kΩ，偏弱。**建议在 SCL/SDA 到 3.3 V 之间各加 4.7 kΩ 外部上拉**，
> 否则线一长或速率一高就时通时不通。

还有一条：`iwdg_init()` 在 `defaultTask` 里，只有它被调度到看门狗才会启动。
如果某个高优先级任务长时间不让出 CPU，它喂不了狗，**板子会每 0.8 s 复位一次**，
诊断变量被反复清零，看起来也像"值读不到"。**看到反复复位，先查是哪个任务没让出 CPU。**

---

## 6. 编译

需要 CMake ≥ 3.22、Ninja、`arm-none-eabi-gcc`（已验证 STM32CubeCLT 1.21.0 自带的工具链）。

```bash
cmake --build --preset Debug          # 输出 build/Debug/ymouseD1.elf
```

可用 preset：`Debug`、`RelWithDebInfo`、`Release`、`MinSizeRel`（见 `CMakePresets.json`）。
linker 已加 `-u _printf_float`，newlib-nano 下 `printf` 支持 `%f`。

---

## 7. 用 SEGGER Ozone 调试（当前可用）

本机 J-Link 的 USB 接口绑定的是 SEGGER 私有驱动，而 OpenOCD 只走 libusb，
两者不兼容（详见 8.6），所以**现在用 SEGGER 自家的 Ozone 调试** —— 它直接用
SEGGER 驱动，插上就能用。OpenOCD 的配置也已经写好放在 `tools/`，等换探针或
换驱动后可直接用，见第 8 节。

### 7.1 打开工程

先编译出固件：

```powershell
cmake --build --preset Debug
```

然后打开调试工程：

```powershell
& "C:\Program Files\SEGGER\Ozone\Ozone.exe" tools\ymouseD1.jdebug
```

也可以直接双击 `tools/ymouseD1.jdebug`，或在 Ozone 里 File → Open 选它。

### 7.2 已经配好的东西

`tools/ymouseD1.jdebug` 里预置了：

| 项 | 值 |
|---|---|
| 器件 | `STM32F401CCU` |
| 接口 / 速度 | SWD / 4 MHz |
| 内核寄存器视图 | `$(InstallDir)/Config/CPU/Cortex-M4F.svd` |
| 外设寄存器视图 | `C:/ST/STM32CubeCLT_1.21.0/STMicroelectronics_CMSIS_SVD/STM32F401.svd` |
| RTOS 感知 | `FreeRTOSPlugin_CM4` |
| 程序文件 | `$(ProjectDir)/../build/Debug/ymouseD1.elf` |

两点说明：

- **器件名是 `STM32F401CCU` 而不是 `STM32F401CC`**。J-Link 器件库里按封装区分，
  `U` 后缀才是 UFQFPN48 的 STM32F401CCU6。
- 外设视图用的是 STM32CubeCLT 自带的 ST SVD（Ozone 自己只带 `STM32F103xx` 和
  `STM32F407IG`）。装了别的 CubeCLT 版本时，把 `Project.AddSvdFile(...)` 那行路径
  改成实际位置即可。接了多台 J-Link 时，取消注释 `Project.SetHostIF("USB", "...")`。

### 7.3 常用操作

- **下载并运行**：打开工程时 Ozone 会自动连上 J-Link 并按 `File.Open` 指定的 ELF
  下载；若没自动下载，用 Debug 菜单里的 *Download & Reset Program*。
- **任务视图**：挂了 FreeRTOS 插件后，Ozone 会列出各任务、状态和栈使用情况。
  本工程 4 个任务：`defaultTask` / `master` / `imu` / `message`（见第 4 节）。
- **看变量**：Watch 里加 `g_imu`、`g_imu.online`、`g_imu.source`、`g_imu.gyro_dps[0]`，
  调试 IMU 通信问题时再加 `g_imu_diag`（展开看 `count`/`last_addr`/`last_status`/`fail_cnt`）
  和 `g_jy901s_info`（展开看 `valid`/`version`/`voltage_v`）。判读见 5.7、5.8。
- **⚠️ 重新编译后要重新加载 ELF**：Ozone 是在打开工程时读 ELF 的，改完代码只
  `cmake --build` 不会自动刷新，会出现"源码是新的、符号地址是旧的"，Watch 读到的值是错的。
  改过代码后请重开工程（或 File → Open Program File 重新选 ELF），再 *Download & Reset Program*。
  核对办法：拿 `arm-none-eabi-nm build/Debug/ymouseD1.elf | findstr <符号>` 的地址
  和 Ozone Watch 里显示的 Location 对一下，不一致就是 ELF 旧了。
- **断点**：直接点行号即可。工程开的 IWDG 不会因为停断点而复位 —— 固件
  `iwdg_init()` 里置了 `DBGMCU_APB1_FZ_DBG_IWDG_STOP`，J-Link 侧 halt 时也会冻结
  看门狗，双保险。

### 7.4 连不上时先查什么

打开工程后左下角应显示 J-Link 序列号和器件名、能读出寄存器。若报
"Cannot connect to target" 之类，按顺序查：

1. 板子有没有上电；
2. SWD 三根线是否接好：`SWCLK = PA14`、`SWDIO = PA13`、以及共地；
3. 目标是不是在低功耗/复位状态 —— 可以先用 Ozone 的 *Reset* 再连；
4. 速度降到 1 MHz 试试（改 `Project.SetTIFSpeed`）。

---

## 8. 用 OpenOCD 调试

配置文件都在 `tools/`：

| 文件 | 作用 |
|---|---|
| `tools/openocd.cfg` | OpenOCD 配置：探针 + SWD + STM32F4 目标 + 速度 |
| `tools/debug.gdb` | GDB 初始化脚本：连接、复位、烧录、停在 `main` |
| `tools/openocd-debug.ps1` | 一键脚本：编译 → 起 OpenOCD → 起 GDB → 退出时收尾 |

### 8.1 一键调试

```powershell
pwsh -File tools/openocd-debug.ps1              # 编译 + 烧录 + 调试
pwsh -File tools/openocd-debug.ps1 -SkipBuild   # 不编译,用现有 elf
pwsh -File tools/openocd-debug.ps1 -SkipBuild -NoLoad   # 只 attach,不烧录(看现场)
```

### 8.2 手动两步

```powershell
# 终端 A
openocd -f tools/openocd.cfg

# 终端 B
arm-none-eabi-gdb build/Debug/ymouseD1.elf -x tools/debug.gdb
```

GDB 起来后就是常规操作：`b main`、`continue`、`next`、`p g_imu`，Ctrl+C 中断，
`monitor reset halt` 重新复位。OpenOCD 在 3333 端口提供 GDB server，
4444 是 telnet（可直接敲 OpenOCD 命令）。

### 8.3 只烧录

```powershell
openocd -f tools/openocd.cfg -c "program build/Debug/ymouseD1.elf verify reset exit"
```

### 8.4 换探针

`tools/openocd.cfg` 里只改一行即可，其余都不用动：

```tcl
source [find interface/jlink.cfg]      # 当前默认:SEGGER J-Link
# source [find interface/stlink.cfg]   # 换成 ST-Link 时对调这两行
# source [find interface/cmsis-dap.cfg]# 或 CMSIS-DAP
```

### 8.5 看门狗与断点

本工程开了独立看门狗（IWDG），但不会因为停断点而复位：

- `target/stm32f4x.cfg` 在 `examine-end` 事件里会置 `DBGMCU_APB1_FZ` 冻结 IWDG/WWDG；
- 固件里 `iwdg_init()` 也自己置了 `DBGMCU_APB1_FZ_DBG_IWDG_STOP`。

两条都有，双保险。

### 8.6 已知限制：本机 J-Link 目前连不上

实测 `openocd -f tools/openocd.cfg` 会报：

```
Warn : Failed to open device: LIBUSB_ERROR_NOT_FOUND
Error: No J-Link device found
```

原因是 **OpenOCD 用的 libusb 打不开 SEGGER 的私有 USB 驱动**：

- OpenOCD 0.12.0 的 `jlink` 驱动只走 libusb（二进制里没有任何 `JLinkARM.dll` 引用）；
- 本机 J-Link 接口（`USB\VID_1366&PID_0105&MI_02`）绑定的是 SEGGER 私有驱动
  （服务名 `jlink`，INF 版本 2.70.08.0 / 2018 年），不是 WinUSB。

三条路：

1. **换 ST-Link / CMSIS-DAP 探针** —— 免改系统，`tools/openocd.cfg` 里换一行 `source` 即可。
2. **把 J-Link 的 USB 驱动换成 WinUSB**（Zadig 之类工具）—— 换完 OpenOCD 直接可用；
   代价是换回去之前 SEGGER 自家工具（Ozone 等）也不能用了，而且需要管理员权限。
3. **不用 OpenOCD，改用 SEGGER 官方工具** —— 本机装了 Ozone，用 SEGGER 自己的驱动。

> **当前采用第 3 条**：日常调试走第 7 节的 Ozone；本节的 OpenOCD 配置保留，
> 以后换探针或换驱动后可直接使用。

---

## 9. 已知问题 / TODO

- **TIM3 周期不适合舵机**：当前预分频 84-1（1 MHz）、周期 1000-1 → 周期 1 ms（1 kHz）。舵机需要 50 Hz（20 ms），且 `dart_motor.c` 用的脉宽是 500~2500 µs，超过 1000 µs 会被计数上限截断。应把周期改为 `20000-1`。
- **`dart_init()` 的局部变量遮蔽同名全局变量**：`dart_init()` 里写的是 `servo_motor_t servo_motor1 = {...}`，初始化完就丢；而 `dart_stop()` / `servo_motor_angleset()` 用的是文件作用域那个**从未初始化**的全局（`htim == NULL`，`ServoMotor_SetPosition()` 会直接返回 -1），所以舵机现在动不了。编译器已经在警告 `unused variable 'servo_motor1..4'`。修法：把类型前缀去掉，直接给全局赋值。
- **`app/message.c` 的串口上报整体注释着**，`USART2` 目前没有实际数据流出。
- `bsp/servo_motor.c` 的 `ServoMotor_SetPosition()` 声明为 `int` 但没有 `return`。
- **CubeMX 会覆盖手改的 `.ioc`**：`GPIO_Label` 这类必须用 GUI 设置，否则 Generate 一次就丢（见第 2 节）。
- LSM6DSV16XTR 不做姿态解算，`g_imu` 的 `roll/pitch/yaw` 在该后端下恒为 0。
- JY901S 的磁场（`0x3A`~`0x3C`）没有被 `imu_sample_t` 承载，读那 26 字节时跳过了；需要的话可以加进统一数据结构。
- `g_imu` 由 `imu` 任务写、其它任务读，目前没有加锁；读取方若需要一致性快照，建议加互斥量或改为双缓冲。
- `vApplicationStackOverflowHook()` / `vApplicationMallocFailedHook()` 是空的：任务栈溢出或堆不足会**静默**发生（`configTOTAL_HEAP_SIZE = 16384`，四个任务加起来约 9 KB，目前够用）。
