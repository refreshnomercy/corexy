# 固件快照：2026-10-06

文件：`corexy_20261006_HMI50_P30_R15_CUR200.hex`

对应此次 GitHub 备份源码：屏幕点击 50 mm/s，自检探边 30 mm/s、回退 15 mm/s、加速度档 50、电流门限 200 mA。局部刷新脚本变化在串口屏端，仍需屏幕编辑器编译和实机验收。

```text
SHA256 6CAF70EA6CEEA9E78A2E1BB69F8E8B1BBC24C4F1EC2D390A9698E4C5C185B454
```

Keil ARMCC 5.06 update 6 构建：0 错误、0 警告。烧录到工程指定的 STM32F103C8；上电后先通过 KEY1 自检，再进行触摸定位。

此文件由 `MDK-ARM/corexy/corexy.hex` 复制保存。后续修改源码须重新编译，不应继续使用本快照作为新源码对应固件。第三方组件条款见根目录 `THIRD_PARTY_NOTICES.md`。
