# CoreXY 龙门架

一个男人做的 CAN 通讯 CoreXY 龙门架。

基于 STM32F103C8、两台 Emm_V5 闭环步进驱动器和 TJC 串口屏，实现自检测量行程、触摸定位、两点间直线运动和编码器实际位置显示。此仓库保存 2026-10-06 的工程状态。

## 当前功能

- KEY1 自检：依次向 +X、+Y、−X、−Y 探边，每次碰边后回退 5 mm，建立工作范围及左下角原点。自检时两电机转速幅值相等。
- 普通定位：点击屏幕后沿当前位置与目标的连线运动；运动中再点击，只保留最后一个待执行目标，在当前任务完成后执行。
- 到位判断：双编码器各轴误差不超过 0.5 mm，位置稳定 120 ms，再确认双轴停止应答。此门限不代表整机实测精度。
- 故障处理：电流、停转、方向、偏离路径、反馈失联、应答失败及 CAN 异常触发停止，清除待执行目标并要求重新自检。
- 屏幕红点显示点击目标，绿圈显示 STM32 回传的编码器实际位置；局部刷新脚本见 `hmi/events/`。

## 接线与参数

| 用途 | STM32 接口 | 配置 |
|---|---|---|
| Emm_V5 驱动器 | CAN RX PB8 / TX PB9，经 CAN 收发器连接 | 1 Mbps；A 地址 1，B 地址 2；广播地址 0 |
| 串口屏 | USART2：PA2 TX → 屏 RX；PA3 RX ← 屏 TX | 115200，8N1，共地 |
| 蓝牙日志/指令 | USART1：PA9 TX / PA10 RX | 115200，8N1 |
| KEY1 | PA4 | 启动自检 |

驱动器应答模式使用 `nReceive` 或 `nBoth`，以便确认目标及停止指令。实际接线及驱动器配置需与源码一致。

| 当前参数 | 数值 | 修改位置 |
|---|---|---|
| 屏幕点击定位速度 | **50 mm/s** | `hardware/hmi.h` 的 `HMI_MOVE_FEED` |
| KEY2 点动速度 / 步长 | 20 mm/s / +Y 10 mm | `hardware/app.c` |
| 自检探边 / 回退速度 | 30 / 15 mm/s | `hardware/corexy.c` |
| 碰边电流门限 | 严格大于 200 mA，连续新样本确认 | `hardware/corexy.h`、`hardware/corexy.c` |
| 屏幕触摸热区 | x=20，y=20，w=470，h=420 | `hardware/hmi.h`，屏幕 m0 同步配置 |
| 传动换算 | 40 mm/转；1280 命令脉冲/mm；65536 编码器计数/转 | `hardware/corexy.h` |

屏幕坐标以整个屏幕为基准，固件扣除热区偏移、翻转 Y，再映射到本次自检得到的软限位；工作范围不写死为历史测量值。

## 编译与使用

1. 安装 Keil MDK 和 ARM Compiler 5；本项目验证环境为 ARMCC 5.06 update 6、`Keil.STM32F1xx_DFP.2.3.0`。
2. 打开 [MDK-ARM/corexy.uvprojx](MDK-ARM/corexy.uvprojx)，选择 `corexy` 目标编译。CubeMX 配置在 [corexy.ioc](corexy.ioc)。
3. 使用 ST-Link 等工具烧录生成的 `MDK-ARM/corexy/corexy.hex`，或使用 [firmware/](firmware/) 内此次保存的固件；固件校验信息见该目录说明。
4. 使用 TJC 屏幕编辑器打开 [hmi/1.HMI](hmi/1.HMI)，核对并合并 [绿圈配置](hmi/GREEN_MARKER_SETUP.md) 与 [局部刷新说明](hmi/MARKER_FLICKER_FIX.md)，编译下载到屏幕。
5. 上电后按 KEY1 完成自检，再点击屏幕定位。普通运动中通过蓝牙发送 `S` 并换行可停止；停止/故障后重新自检。

自检仍为阻塞流程，自检期间不会实时处理普通点击或蓝牙 `S`；前后会丢弃缓存输入。用户已确认此前直线定位运行正常；局部刷新脚本仍需在屏幕编辑器编译和实机观察，不能仅凭主机测试认定显示效果已验收。

## 目录与流程

| 路径 | 内容 |
|---|---|
| `Core/` | STM32 初始化、中断和外设；main 交给 App 调度 |
| `hardware/app.*` | 按键、输入、CAN 健康与主循环调度 |
| `hardware/corexy.*` | 自检、运动状态、反馈验收及保护 |
| `hardware/motion.*`、`line_plan.*` | 任务与最后一次点击队列、共同直线路径进度 |
| `hardware/Emm_v5/` | Emm_V5 协议、CAN 分包发送及队列 |
| `hardware/hmi.*`、`tjc_usart_hmi.*` | 触摸映射、实际位置回传、串口收发 |
| `Drivers/` | STM32 HAL、CMSIS 及其许可证 |
| `hmi/` | 屏幕工程、事件代码和配置说明 |
| `tests/` | 主机模拟、协议与 CAN 队列回归测试 |
| `firmware/` | 此次可烧录固件及 SHA256 |

详细说明：[项目进展与流程](PROJECT_STATUS_AND_FLOW.md)、[定位控制流程](POSITION_CONTROL_FLOW.md)、[电机使能全过程](MOTOR_ENABLE_FLOW.md)、[屏幕配置](hmi/README.md)。[AI_HANDOFF.md](AI_HANDOFF.md) 保存历史排查记录，其中旧速度、旧固件和待验收描述对应各自历史版本，当前参数以源码及本说明为准。

## 主机测试

Windows 环境需 Python 3、MSVC 和 Windows SDK。先按本机安装位置调整三个 `tests/build_*_tests.ps1` 中的 MSVC/SDK 路径，再运行：

```powershell
powershell -ExecutionPolicy Bypass -File tests/build_boundary_tests.ps1
powershell -ExecutionPolicy Bypass -File tests/build_hmi_tests.ps1
powershell -ExecutionPolicy Bypass -File tests/build_can_tests.ps1
python tests/run_motion_tests.py
python tests/run_boundary_tests.py
python tests/run_hmi_tests.py
python tests/run_can_tests.py
```

2026-10-06 本次备份验证：Keil 编译 0 错误、0 警告；84 项运动/显示、36 项自检、50 项屏幕收发、19 项 CAN 测试，共 189 项通过。旧屏幕日志测试预期已同步为当前 50 mm/s 参数。

主机测试验证代码路径，不替代实机驱动器响应、定位精度及屏幕显示验收。构建日志和历史备份保留本地，未纳入此仓库。

## 许可证

本项目原创应用代码、原创测试及文档采用 [MIT](LICENSE)。第三方代码、CubeMX 生成文件及屏幕内第三方资源保留各自权利与条款，详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)；根目录 MIT 不覆盖这些内容。
