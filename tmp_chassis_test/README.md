# chassis.c 的 PC 端回归测试

不需要连硬件，用 `stub/` 里的假串口和假延时把 `app/chassis.c` 跑在电脑上，
把发出去的字节录下来逐帧检查，用来在改完参数/方向表后先确认没改坏。

## 一键运行

在工程根目录执行（Windows PowerShell）：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tmp_chassis_test/run_tests.ps1
```

会编译并运行 4 种配置（8/9 字节帧 × 开/关同步），全部通过时最后输出 `ALL VARIANTS PASS`。

## 手动编译

```powershell
gcc -std=c99 -Wall -Wextra -I tmp_chassis_test/stub -I app `
    tmp_chassis_test/test_chassis.c app/chassis.c -lm -o tmp_chassis_test/test_chassis.exe
./tmp_chassis_test/test_chassis.exe
```

覆盖配置：

```
-DCHASSIS_FRAME_LAYOUT=CHASSIS_FRAME_LAYOUT_9B
-DCHASSIS_USE_SYNC=1
```

## 覆盖的检查点

- 8 / 9 字节 F6 帧的字段位置、功能码、方向、加速度、速度、同步、`0x6B` 校验；
- 示例帧 `01 F6 00 00 32 32 01 6B` 逐字节复现（8 字节格式）；
- 前进 / 后退 / 左移 / 右移 / 逆时针 / 顺时针的方向表、单轮极性、整轴取反；
- 速度覆盖（`Chassis_MoveAt`）、连续速度（`Chassis_SetWheelSpeed`）、超限限幅；
- 定时运动的阻塞时间 = 估算时间 + 帧间隔 + 停稳等待；
- 时间估算表（1 圈多少 mm / 多少度、默认 mm/s 与 °/s）。

> 说明：PC 端只能验证「发出去的帧」和「时间换算」，真实距离还要上车用卷尺量，
> 走多了调小 `CHASSIS_TIME_GAIN_PCT`，走少了调大。
