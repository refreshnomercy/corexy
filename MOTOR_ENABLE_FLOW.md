# 电机使能流程（2026-10-06定位版）

使能分为软件运动门禁和驱动器硬件使能。`s_enabled`决定是否允许下发运动；`s_drv_en`记录软件已下发的硬件状态。它们都不是实时驱动器使能反馈。

## 上电

调用链：`main → MX_CAN_Init → App_Init → corexy_init`。CAN初始化启动收发与中断，App初始化提示设备、串口屏和蓝牙后配置电机。

| 顺序 | 调用/动作 | 含义 |
|---|---|---|
| 1 | `s_enabled=1` | 放开软件运动门禁 |
| 2 | A：`Emm_V5_Modify_Ctrl_Mode(1,false,2)`，等20ms | 设置闭环模式，不保存参数 |
| 3 | B：同一命令，地址2，等20ms | 设置第二台闭环模式 |
| 4 | A：`Emm_V5_En_Control(1,true,false)`，等20ms | 立即使能第一台 |
| 5 | B：同一命令，地址2，等20ms | 立即使能第二台 |
| 6 | `corexy_set_zero()`，等20ms | 两驱动计数、软件模型清零；编码器基准和旧标定作废 |
| 7 | `s_drv_en=1` | 记录使能命令已发出 |

上电路径没有等待Enable ACK，也没有像故障恢复路径那样确认双轴Stop。当前设置不自动上电自检，需要KEY1触发；上电清零是把当前位置定为模型零点，不是找到了左下角。

## 一条使能命令怎样发送到电机

`Emm_V5_En_Control(addr,state,snF)`组装`[addr,F3,AB,state,snF,6B]`。`state=1`使能，`0`断使能；`snF=false`立即执行，`true`等待同步触发。本项目使能/断使能都用立即执行。

调用链为：`Emm_V5_En_Control → can_SendCmd → can_EnqueueCmd → can_SendNextCmd → HAL_CAN_AddTxMessage`。命令复制到5条深的软件队列，地址编码在扩展ID高位。地址1的使能命令：扩展ID`0x100`，数据`F3 AB 01 00 6B`；地址2对应ID`0x200`。

CAN TX中断调用三个邮箱完成回调之一，再进入`can_TxCompleteHandler()`推进分包和下一条命令。TX完成表示传输完成，不能代替驱动器接受ACK。当前接收解析不统计0xF3使能ACK。

## 碰墙后的卸力和重新使能

`bound_trigger → drive_cut_off`先清旧发送队列、停止双轴，再分别发送`En_Control(addr,false,false)`卸力，记录`s_drv_en=0`。

回退通过`corexy_move_to`走同一运动入口；因为驱动器已断使能，进入`drive_prepare_motion`：

| 顺序 | 动作 | 检查 |
|---|---|---|
| 1 | A/B清堵转，各间隔5ms | 清驱动器堵转锁存 |
| 2 | 保存A/B停止ACK计数 | 后面必须收到本次的新回复 |
| 3 | A Enable后立即A Stop，等5ms | 中间不等待Enable ACK，尽量缩短旧目标续跑窗口 |
| 4 | B Enable后立即B Stop，等5ms | 取消第二台的旧目标 |
| 5 | 等双轴各收到新`FE 02 6B` | 最多150ms；失败按BR=6停止卸力，不启动新目标 |
| 6 | `s_drv_en=1`，输出`DRIVE READY` | 停止接受确认通过，可挂新位置目标 |
| 7 | 回退重新取得各2个停止位置包 | 冻结回退起点；300ms内取不到按BR=3退出 |
| 8 | A/B下发0xFD相对位置目标，`snF=1` | 自检等待双方新的`FD 02 6B`，失败按BR=5退出 |
| 9 | 地址0广播0xFF同步启动 | 纯X/Y双轴同RPM、脉冲幅值和加速度 |
| 10 | 位置/方向/轴间差/停稳验收 | 回退合格才进入下一腿 |

使能必须先于0xFD目标，否则驱动器可能以E2拒绝；恢复使能后紧跟Stop用于撤销旧目标。普通运动在`s_drv_en=1`时跳过重新使能，但现在也强制等待所需轴FD接受ACK后才广播同步。
普通XY直线由`line_start`调用同一个`drive_prepare_motion`完成使能准备，再由`line_send`更新有限绝对目标。
每次更新运动轴的FD ACK都须通过；静止轴无需FD ACK，但位置反馈与最终Stop ACK仍须通过。
软件梯形曲线规划加速度100mm/s²、驱动FD档255快速跟随；表中ACC50等速相对目标流程用于自检/回退。

## 其他入口与状态含义

`corexy_rearm_drivers()`只由KEY1自检前的恢复检查触发，停止卸力、清堵转、重设闭环、请求位置，保留模型坐标并撤销标定。保持硬件断使能，重新自检的探测启动时再准备驱动器。普通定位命令/状态查询不自动恢复。

`corexy_set_enabled(0)`关闭软件门禁并发送Stop，保持驱动器使能与保持力矩；`corexy_set_enabled(1)`只打开软件门禁，不发送硬件Enable。`corexy_stop()`同样不卸力。

目标ACK只确认接受，模型坐标仍先更新为目标；普通运动busy现在持续到双编码器实际到位/停稳及最终Stop ACK确认。
普通定位故障调用`corexy_fault_stop`停止卸力、锁存故障并撤销标定，不自动回退；S停止活动段也作废标定，Stop ACK失败升级为BR=6。
`corexy_set_zero`先清发送队列、分别发两台清计数命令并间隔5ms；明确驱动零点，第一包位置保留真实位移，停止未确认时拒绝清零。

## 本次验证与备份

清理前源码和Keil配置保存于`backups/legacy_cleanup_20261006/source/`，SHA256清单为同目录上一级`manifest.json`。现有完整ZIP备份继续保留。

当前直线版Keil编译0错误0警告；74项定位/应用/直线、36项自检、30项屏幕、19项真实CAN发送代码测试通过，共159项。
本轮修改前关键源码/项目配置及用户确认正常的AXISQ HEX在`backups/direct_line_before_20261006_200048/`，新固件及流程见`POSITION_CONTROL_FLOW.md`。本轮尚未实机验证。
