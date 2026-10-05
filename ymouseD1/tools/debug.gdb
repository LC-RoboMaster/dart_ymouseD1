# ============================================================================
# GDB 初始化脚本 —— ymouseD1 / STM32F401CCU6
#
# 用法(先在另一个终端跑 openocd -f tools/openocd.cfg):
#   arm-none-eabi-gdb build/Debug/ymouseD1.elf -x tools/debug.gdb
#
# 也可以直接写进 IDE 的调试配置里(gdbinit / initCommands)。
# ============================================================================

set pagination off
set confirm off

# ARM Cortex-M 相关的美化输出
set print pretty on
set print asm-demangle on

# OpenOCD 的 GDB server 默认监听 3333
target extended-remote localhost:3333

# 复位并停在复位向量,保证不受当前运行状态影响
monitor reset halt

# 下载固件。OpenOCD 负责实际烧写并校验(等价于 flash write_image + verify_image)
load

# 停在 main,方便直接开始调试
tbreak main
continue
