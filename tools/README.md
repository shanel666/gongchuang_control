# 上位机工具

| 文件 | 用途 |
|---|---|
| `k230_host.py` | **假主控**：电脑冒充 F103，通过 CH340 跟 K230 对话（车不在手边时用这个） |
| `chassis_host.py` | 无线串口底盘上位机（配合 `DEBUG_MODE_WIRELESS`） |

---

## 无线串口上位机（chassis_host.py）

配合固件 `main.c` 的 `DEBUG_MODE_WIRELESS` 模式使用。

## 接线

- 无线串口模块 TX → STM32 **PB11（USART3_RX）**
- 无线串口模块 RX → STM32 **PB10（USART3_TX）**
- 模块与单片机共地；波特率 **115200、8N1**

> PB10/PB11 和板载按键 2/键 3 复用，接了无线模块以后这两个按键不要再用。

## 运行

```powershell
python tools/chassis_host.py
```

或者双击 `tools/run_host.bat`。需要先装依赖：

```powershell
pip install -r tools/requirements.txt
```

没有硬件时可以自检协议函数：

```powershell
python tools/chassis_host.py --selftest
python tools/chassis_host.py --list-ports
```

## 界面

- 选择串口 → **连接**（会自动发一次 `P`，收到 `PONG` 表示链路正常）；
- 动作按钮：前进 / 后退 / 左平移 / 右平移 / 逆时针 / 顺时针，时长默认 1000 ms；
- **停止 S**、**内置测试 D**（前进 1s → 逆时针 1s → 后退 1s → 顺时针 1s）、**查角度 ?**；
- “默认速度(0.1RPM)”填 300 表示 30.0 RPM，点**应用**发 `V 300`；
- 快捷键：`W/S/A/D` 前后左右，`Q/E` 逆时针/顺时针，`空格` 停止；
- 日志里蓝色是发送（TX），绿色是接收（RX），红色是错误。

## 命令协议（固件端一行一条 ASCII）

| 发送 | 作用 |
|---|---|
| `F 1000` `B 1000` | 前进 / 后退 1000 ms |
| `L 1000` `R 1000` | 左平移 / 右平移 1000 ms |
| `Q 1000` `E 1000` | 逆时针 / 顺时针 1000 ms |
| `S` | 立即停车 |
| `D` | 内置测试：前进 1s → 逆时针 1s → 后退 1s → 顺时针 1s |
| `?` | 查询，回 `YAW <0.01°>` |
| `P` | ping，回 `PONG` |
| `V 300` | 设默认速度 0.1RPM（300 = 30.0 RPM） |
| `H` | 帮助 |

- 数字可省略（默认 1000 ms），也支持 `F1000` 写法；
- 回包：`ACK F 1000` → 动作 → `DONE F 1000`，中途按 PA8 则回 `ABORT F`；
- 时间范围 100~10000 ms，速度范围 0~30000（0~3000.0 RPM）。

---

# 假主控（k230_host.py）

车不在手边时，用电脑冒充 STM32F103 跟 K230 对话，先把视觉那一半调通。

## 接线（CH340 ↔ K230，K230 板载 GH1.25 串口2 座子）

| CH340 | K230 座子 | 说明 |
|---|---|---|
| TXD | **R**（GPIO12，UART2_RXD） | 电脑发 → K230 收 |
| RXD | **T**（GPIO11，UART2_TXD） | K230 发 → 电脑收 |
| GND | **G** | 必须共地 |
| VCC | 不接 | K230 自己供电 |

- ⚠️ CH340 必须是 **3.3V 电平**的（模块上一般有 3V3/5V 跳线，拨到 3V3）。
- ⚠️ **不要**把 K230 自己的 USB 口当成数据口：那个是 CanMV IDE 的 REPL（`OpenMV Cam USB COM Port`，VID `1209:ABD1`），往上发命令只会打进 Micropython 解释器。`k230_host.py` 会自动排除它。

## 运行

**最省事：双击 `tools\run_k230_host.bat`**（会自动挑 Python、缺 pyserial 会自动装，结束时窗口会停住方便看结果）。

或者命令行：

```powershell
python tools/k230_host.py                 # 自动找 CH340
python tools/k230_host.py --list          # 看有哪些串口、哪个能用
python tools/k230_host.py --port COM5     # 手动指定
python tools/k230_host.py --selftest      # 不用硬件，验证解析逻辑
python tools/k230_host.py --port loop:// --send circle --duration 1   # 自环冒烟测试
python tools/k230_host.py --send circle --duration 5                  # 发一次，看 5 秒
```

> ⚠️ **改这两个 .bat 必须存成 ANSI(GBK) + CRLF 换行**。存成 UTF-8 或 LF，cmd 会把中文和命令一起拆坏
> （现象：`'ho.' is not recognized as an internal or external command`）。

交互模式下按键（不用回车）：`c` = circle，`l` = linear，`s` = stop，`?` = 帮助，`q` = 退出。

## 协议

| 方向 | 内容 |
|---|---|
| 电脑 → K230 | `circle` / `linear` / `stop`（各占一行） |
| K230 → 电脑 | `C<x>,<y>,<r>`（圆心+半径）/ `L<k>,<psi>`（斜率×100 + 偏航 0.01°）/ `NG` |

解析规则和 `Hardware/Serial.c` 的 `K230_ParseLine()` 严格一致（范围校验、丢坏行都一样），
所以在电脑上调通的行为，烧进 F103 后是一致的。
