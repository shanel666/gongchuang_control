# 无线串口上位机（chassis_host.py）

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
