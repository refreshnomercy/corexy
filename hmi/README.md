# 串口屏点击实际定位

STM32接收触摸坐标，报告目标并沿两点连线实际定位：当前点击路径速度50mm/s、双电机按运动比例运行。
屏幕源文件由用户配置，本轮未修改`1.HMI`。
当前源码对应的备份固件：`../firmware/corexy_20261006_HMI50_P30_R15_CUR200.hex`。本地旧LINE20文件为此前20mm/s版本。
新增实际位置绿圈：配置步骤及可粘贴代码见[GREEN_MARKER_SETUP.md](GREEN_MARKER_SETUP.md)。
红点/绿圈明显闪烁的屏幕脚本已改为局部恢复原图、静止不重画；已有12变量配置增加7个私有变量即可，见[MARKER_FLICKER_FIX.md](MARKER_FLICKER_FIX.md)。此修改无需重烧STM32。
STM32取双编码器≤300ms的新鲜位置，每100ms响应一次屏幕取样请求；未标定/故障时隐藏。
`1.HMI`由用户在编辑器合并配置并编译下载，本次没有改写该文件。
原报告版HEX仍保留，本版点击会驱动滑块。

STM32 PA2（USART2 TX）→屏RX；屏TX→STM32 PA3（USART2 RX）；GND共地。
屏幕和STM32 USART2均使用115200、8N1；蓝牙仍为USART1 PA9/PA10，115200、8N1。

屏幕m0热区须与`hardware/hmi.h`一致：x=20，y=20，w=470，h=420。
实际有效像素为X=20..489、Y=20..439。若修改热区，同步修改这四个宏再编译。
屏幕触摸事件发送：

```text
printh 55 54
prints tch0,2
prints tch1,2
printh FF FF FF
```

`tch0/tch1`使用完整屏幕坐标。帧固定9字节：

```text
55 54 X低字节 X高字节 Y低字节 Y高字节 FF FF FF
```

例如点击(255,230)：`55 54 FF 00 E6 00 FF FF FF`。
数字用二进制小端格式发送，不能改为ASCII数字；旧OK帧`55 6F 6B FF FF FF`仍输出`ok`。

换算范围来自本次自检的`corexy_get_area()`软限位，以mm为单位：

```text
target_x = (touch_x - 20) * x_max / 469
target_y = (439 - touch_y) * y_max / 419
```

屏幕Y向下增大，滑块Y向上增大，因此Y必须翻转。四角对应关系：

| 屏幕像素 | 滑块目标mm |
|---|---|
| 左下(20,439) | (0,0) |
| 左上(20,20) | (0,y_max) |
| 右下(489,439) | (x_max,0) |
| 右上(489,20) | (x_max,y_max) |

验收步骤：

1. 烧录上述HEX，打开蓝牙串口。启动应出现`HMI POSITION: direct line, 50.00mm/s; latest pending click wins`及`HMI REGION X=20 Y=20 W=470 H=420`。
2. 未自检时点击热区，应出现原始PX/PY及`TARGET UNAVAILABLE: run KEY1 calibration`。
3. KEY1完成自检后先点击热区内靠近原点、X/Y均改变的位置，确认沿斜线移动；观察`POS START LEG=LINE`、`POS LINE`和`POS DONE`，DONE应在实际停稳后出现。
4. 再点击四角和内部点，检查上表及实际落点；越界帧应输出`REJECT: outside touch region`且不进入运动任务。
5. 运动中连续点两处，当前完整XY任务完成后只去最后一次点击处；检查`POS QUEUED LAST`。
6. 运动中经蓝牙发送`S`并换行：当前和待执行任务取消，无缓存点击续跑；重新KEY1自检后再点击。

例如本次范围为161.00×144.00mm，点击(255,230)，正常任务日志示例：

```text
HMI TOUCH PX=255 PY=230 -> TARGET X=80.67mm Y=71.83mm
POS ACCEPT ID=1 SRC=HMI X=80.67mm Y=71.83mm
POS START ID=1 LEG=LINE
POS LINE L=108.01mm T=<实际规划时长>ms MASK=3
POS DONE ID=1 ACTUAL X=80.67mm Y=71.83mm
```

实际ACTUAL取双电机编码器换算值，不一定与上例完全一致。验收为每轴目标误差≤0.5mm、
120ms累计变化≤0.05mm，再确认双轴Stop；显示两位小数不代表整机定位精度。
普通定位检测到持续>200mA、停转、位置失联、方向/同步异常、ACK失败或CAN错误时，
输出`POS FAULT BR=...`与`POS CHECK`，停止卸力、撤销标定、清空待执行目标；KEY1重新自检。
普通定位故障不会像自检碰墙那样自动回退。完整流程/BR表见`../POSITION_CONTROL_FLOW.md`。

KEY2/蓝牙普通运动已异步，主循环持续处理输入。自检仍阻塞，前后丢弃屏幕/蓝牙缓存输入，
自检期间的旧点击不会在结束后启动。蓝牙RX丢字节会取消任务，超长命令行丢弃至换行。
两台驱动器Response须为nReceive或nBoth，以便确认目标与停止接受ACK。

代码流程：USART2中断收字节→`tjc_take_event()`解析事件→`HMI_Poll()`检查标定和热区→
`HMI_MapTouch()`换算→蓝牙输出/提交`Motion_SubmitXY()`→主循环定位/到位/任务推进。
热区配置和普通定位速度在`hardware/hmi.h`；自检摸边30、回退15mm/s、电流阈值200仍为原值。
当前绿圈版Keil编译0错误0警告，84项定位/应用/直线/显示、36项自检、50项屏幕收发、19项CAN发送代码测试通过，共189项；仍需屏幕编译及上板验证。
使用共同时间曲线、沿线有限绝对位置目标和反馈修正RPM；不需要改屏幕事件帧、热区或驱动器S_Vel_IS配置。
软件路径加速度100mm/s²；路径垂距>1mm停机。驱动FD档255用于跟随更新，自检仍为ACC50且纯轴等速。
保留原先用户确认正常的AXISQ版HEX，可供回退；本版真实驱动覆盖目标的响应/路径精度需验收。
自检标记`LINE-CANQ-CUR200`；预检失败时请保留`CAL PREFLIGHT`及随后`CAL ABORT`，前者含新位置包数量、位置年龄和CAN发送错误计数。
