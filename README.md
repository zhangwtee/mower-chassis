# mower-chassis

割草机下位机：STM32F407VE + FreeRTOS。

与 [mower-host](../mower-host) 配套：上位机（树莓派）负责界面、建图、导航；
下位机负责电机闭环、安全检查、刀片、传感器采集，以及和上位机之间的串口协议。

---

## 目录

```
app/        应用层：主入口、中断入口、协议、状态机、运动学、PI、安全、界面
bsp/        驱动：电机、编码器、串口、CAN、I2C、OLED、Flash、按键、蜂鸣器、LED
cfg/        两个配置文件：FreeRTOSConfig.h、stm32f4xx_conf.h
cmsis/      ARM CMSIS + ST 设备头 + 启动文件 + system_stm32f4xx
spl/        ST 标准外设库（只留了用到的外设）
rtos/       FreeRTOS 内核 + CM4F 端口 + heap_4
project/    Keil 工程 mower.uvprojx，编译产物输出到 ../OBJ/
tests/      PC 单元测试，不参与固件编译
docs/       架构、协议、规格、硬件参数、上机清单
```

## 编译

Keil MDK 5.24 + ARMCC 5.06u7，设备选 STM32F407VETx，需要 Keil.STM32F4xx_DFP 2.9.0。

IDE：打开 `project/mower.uvprojx`，Rebuild。

产物在 `OBJ/`：`mower.axf`、`mower.hex`，以及后处理生成的 `mower.bin`。

## 测试

纯逻辑模块（协议、遥测、运动学、PI、状态机、环缓、限幅、避障、驻车）不依赖任何 STM32 头，
可以直接在 PC 上编：

```
powershell -ExecutionPolicy Bypass -File tests\run_tests.ps1
```

需要 gcc（脚本会先找 PATH，再试几个常见位置）。当前 **1122 条断言**，只跑逻辑，不碰硬件。

`tests/vectors.h` 里是期望字节，由 `tests/gen_vectors.py` 按上位机的组帧逻辑生成；
改了协议就 `python tests/gen_vectors.py` 重新生成一次。

## 现状

编译通过：`Code=26840 / ZI=29460`，0 error 0 warning。

已实现：电机双闭环、六状态机与开机自检、刀片、舵机、IMU 姿态、OLED 三页诊断、
超声波避障（CAN）、参数掉电保存、急停锁存、指令丢失保护。

还没做或者关闭：

- IWDG 默认关闭（`USE_IWDG 0`）
- 蓝牙 APP 的自动回充键（收到丢掉）

首次上机按 [docs/上机清单.md](docs/上机清单.md) 逐条过；硬件参数和极性见 [docs/硬件核对.md](docs/硬件核对.md)。

## 约定

- `.c` / `.h` 用 **GBK**（Keil 默认编码下中文注释才不乱码）
- `.md` / `.gitignore` / `.ps1` 用 **UTF-8**（GitHub、PowerShell 需要）

## 许可与来源

MIT，见 [LICENSE](LICENSE)。

`cmsis/`、`spl/`、`rtos/` 三部分是第三方代码，各自保留原版权头：CMSIS/SPL 属 ST（BSD），
FreeRTOS 属 Real Time Engineers Ltd.（MIT）。这三部分**不是**本工程的版权范围，
也不应被替换成别的许可。

应用层（`app/`、`bsp/`、`cfg/`）是本工程写的部分。

安全须知见 [DISCLAIMER.md](DISCLAIMER.md)。
