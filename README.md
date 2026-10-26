# Sentry-2026-B

RoboMaster 哨兵底盘主控工程。源码直接位于仓库根目录，使用 Keil MDK 打开 [`MDK-ARM/FINAL-2026.uvprojx`](MDK-ARM/FINAL-2026.uvprojx)。

## 编译与烧录

- 需要 Keil MDK 5、ARM Compiler 5、`Keil.STM32F4xx_DFP` 2.8.0，以及 ARM CMSIS Pack。目标芯片为 STM32F407IGHx。
- 在 Keil 中选择 `FINAL-2026` 目标并编译；输出会重新生成到 `MDK-ARM/FINAL-2026/`。
- 连接调试器和目标板后，执行 **Flash → Download**。本机重新编译生成的 HEX 文件也保存在 [`firmware/FINAL-2026.hex`](firmware/FINAL-2026.hex)。
