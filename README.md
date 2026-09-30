# 四麦轮小车（工程创新比赛）— STM32F103C8T6

> 本工程：基于 STM32F103C8T6 的四轮麦克纳姆轮小车，4 个「张大头」闭环步进电机驱动，
> 在 2.25 m × 2.25 m 网格场地上自动规划路径、避开障碍物，从起点走到终点。
> 支持串口遥控「记忆当前位置 / 回到记忆位置」。

---

## 1. 硬件与工具链

| 项 | 值 |
|---|---|
| MCU | STM32F103C8T6（Cortex-M3，64 KB Flash / 20 KB SRAM） |
| 主频 | 72 MHz（`system_stm32f10x.c` 里 `SYSCLK_FREQ_72MHz`） |
| 库 | 标准外设库 STDPeriph（`USE_STDPERIPH_DRIVER`） |
| 启动文件 | `startup_stm32f10x_md.s` |
| 烧录 | FlyMcu（ISP 串口烧录，`FlyMcu.exe`） |
| 开发环境 | Keil MDK（`Project.uvprojx`） |

---

## 2. 目录结构

```
f103c8t6/
├── Project.uvprojx         Keil 工程
├── Start/                  启动文件、core_cm3、system_stm32f10x
├── Library/                ST 标准外设库（全量）
├── System/Delay.c/h        延时 us/ms/s
├── Hardware/
│   ├── Serial.c/h          ★ USART1(电机) + USART2(电脑)
│   ├── OLED.c/h            0.96" OLED（软件 I2C，PA0/PA1，从机 0x78）
│   ├── Key.c/h             3 个按键（PB1 / PB11 / PB10）
│   └── LED.c/h             LED（PA1/PA2 + PC13 板载灯）
├── User/main.c             ★ 主程序
├── mingling/               旧版硬编码电机指令（已废弃，可忽略）
└── app/
    ├── motor.c/h           ★ 电机运动控制 + 位置读写
    ├── path_planner.c/h    ★ Dijkstra 路径规划
    └── path_config.h       ★ 路线参数（起终点/障碍/朝向）
```

---

## 3. 两个串口的分工（重要！）

工程里有两个串口，**用途完全不同，接错线就“没反应”**：

| 串口 | 引脚 | 接什么 | 用途 |
|---|---|---|---|
| **USART1** | PA9(TX) / PA10(RX) | 电机总线 | 发运动指令、读位置 |
| **USART2** | PA2(TX) / PA3(RX) | 电脑（USB-TTL） | 收 `k:数字` 指令、打印状态 |

- 电脑串口助手必须接 **USART2（PA2/PA3）**，波特率 **115200**。
- 电机的 4 个从机都挂在 **USART1** 一条总线上，地址 1~4。

---

## 4. 电机通信协议

### 4.1 运动帧（13 字节）

```
[地址][0xFD][方向][速度H][速度L][加速度][脉冲B3][脉冲B2][脉冲B1][脉冲B0][模式][同步][0x6B]
```

| 偏移 | 含义 | 说明 |
|---|---|---|
| [0] | 地址 | FL=0x01, FR=0x02, RL=0x03, RR=0x04 |
| [1] | 命令 | 0xFD |
| [2] | 方向 | 0x00 正转 / 0x01 反转 |
| [3..4] | 速度 | 大端 2 字节 |
| [5] | 加速度 | 1 字节 |
| [6..9] | 脉冲数 | 大端 4 字节 |
| [10] | 模式 | 0x00 相对 / 0x01 绝对位置 |
| [11] | 同步 | 0x00 立即执行 / 0x01 等同步帧 |
| [12] | 校验 | 固定 0x6B |

### 4.2 其它帧

- **同步释放帧** `00 FF 66 6B`：直行时 4 电机都设 sync=0x01，最后发这帧让它们**同时起步**。
- **使能帧** `[addr] F3 AB 01 00 6B`：上电默认失能，运动前 `motor_enable_all()` 依次使能。
- **读位置** `[addr] 36 6B` → 回 8 字节 `[addr] 36 [dir] [pos×4] 6B`。

### 4.3 麦轮方向（FR、RR 镜像安装，方向位已翻转）

| 动作 | FL(1) | FR(2) | RL(3) | RR(4) |
|---|---|---|---|---|
| 前进 `qianjin_` | 正 | 反 | 正 | 反 |
| 后退 `houtui_` | 反 | 正 | 反 | 正 |
| 旋转 `shun_yaw_`/`ni_yaw_` | 只驱动 FL+FR 两个轮子 | | | |

---

## 5. 标定参数（在 `app/motor.c` 顶部）

| 宏 | 值 | 含义 |
|---|---|---|
| `PULSE_PER_100CM` | 13008 | 平移标定（13.008 脉冲/mm） |
| `PULSE_PER_360DEG` | 17208 | 旋转标定（**TODO 待标定，占位值**） |
| `MOTOR_SPEED` | 50 | 平移速度 |
| `MOTOR_ACCEL` | 200 | 加速度 |

> 注意：运动函数参数单位是 **mm**（内部 `mm * PULSE_PER_100CM / 1000`），
> `motor.h` 注释写“cm”是笔误，以 mm 为准。

---

## 6. 路径规划

- **5×5 网格**，25 个交点，坐标单位 mm：

| | col0 | col1 | col2 | col3 | col4 |
|---|---|---|---|---|---|
| **x** | 150 | 675 | 1200 | 1725 | 2250 |
| **y** | 2250 | 1725 | 1200 | 625 | 150 |

- 障碍在 `app/path_config.h` 配置，`PATH_OBSTACLE_0..3` 默认是 4 个交叉口：
  `(675,1725)(1725,1725)(675,625)(1725,625)`。
- `path_plan(起点, 终点, ...)` 用 **Dijkstra** 算最短路，再提取「起点 + 拐弯点 + 终点」作为关键点，
  结果存 `xpath[]/ypath[]`，个数在全局 `key_point_num`。
- 附 `PATH_PLANNER_DEMO` 宏，可在 PC 上 `gcc -DPATH_PLANNER_DEMO path_planner.c` 单独验证算法。

---

## 7. 主程序流程

### 7.1 当前生效的主循环（串口遥控读写位置）

```
while(1):
  每 500ms 用 USART2 打印  now=  memo=  dir=
  读 uart_key_num：
    k:1 → motor_read_pos(); pos_memo = pos_now   # 记忆当前位置
    k:2 → motor_set_pos(pos_memo)                 # 回到记忆位置
    k:3 → pos_memo += 16                          # 微调 +
    k:4 → pos_memo -= 16                          # 微调 -
```

串口助手（USART2）发 `k:1` + 回车 即可触发。

### 7.2 主循环之后的“死代码”（完整保留，只是被 while(1) 挡住）

自动导航流程：

1. `path_planner_init()` + `path_plan(起点,终点,...)` 算路；
2. `motor_enable_all()` 使能 4 电机；
3. 依次对每个关键点调用 `translation(x0,y0,x1,y1)` 移动。

`translation()` 是按朝向状态机：先 `shun_yaw_/ni_yaw_(90°)` 转对方向（`dir` 变量记忆朝向：
1=北/2=西/3=南/4=东），再 `qianjin_/houtui_` 前进后退。

---

## 8. 已知问题 / 坑（以后回来先看这里）

1. **引脚冲突**：PA1 = OLED_SDA 和 LED1；PA2 = USART2_TX 和 LED2。
   初始化顺序上 OLED 覆盖 LED1、USART2 覆盖 LED2，所以当前固件里 **LED1/LED2 实际不可用**。
2. **`judge_trans()` 只轮询了电机 1**：`done2/3/4` 从未被置 1，`if(done1&&done2&&done3&&done4)` 永远为假，
   函数最终靠 `guard>400` 超时退出 —— 等于“固定延时”，不是“真等 4 个电机到位”。
3. **`zuo_()/you_()` 四个轮子方向位完全相同**（左平移全 0x01、右平移全 0x00），
   与 `mingling.h` 里旧定义（对角轮反向）不一致，疑似没按麦轮运动学更新；当前主流程没用到。
4. **旋转标定未完成**：`PULSE_PER_360DEG` 是占位值。
5. **网格 y 轴间距不均匀**：`2250→1725→1200` 是 Δ525，`1200→625` 是 Δ575、`625→150` 是 Δ475，
   疑似 `625` 应为 `675`。
6. `mingling/` 目录已废弃（12 字节缺 0xFD 的旧指令），被 `motor.c` 动态组帧取代。
7. `User/stm32f10x_it.c` 的中断函数都是空壳，真正的 USART1/USART2 中断在 `Serial.c` 里实现。

---

## 9. 快速上手

1. Keil 打开 `Project.uvprojx`，编译（默认已配好 72MHz + 标准库）。
2. FlyMcu 通过串口（BOOT0=1）烧录。
3. 接线：电机 → USART1(PA9/PA10)；电脑 USB-TTL → USART2(PA2/PA3)。
4. 串口助手 115200 打开 USART2，发 `k:1` 回车 = 记忆当前位置，`k:2` 回车 = 回到记忆位置，
   屏幕上每 500ms 打印 `now= memo= dir=` 状态。
