# vision —— 圆心跟踪（CanMV K230 / OpenMV 通用）

> **现在车上跑的是 [`圆线串口.py`](圆线串口.py)**（circle / linear 命令版 + 串口双向），
> 本文件主要讲 [`circle_track.py`](circle_track.py)（老的纯 `x,y` 格式）。
>
> 两个文件的**画面/坐标约定**（踩过坑，别再破坏）：
> 1. **采集 800×450（像素级 16:9）**，显示时合成到 800×480 画布再上屏（上下各 15 px 黑边）。
>    老配置 640×480 是 4:3，会把左右裁掉 25%（"拍不全"的根源）。
> 2. **相机帧不能直接带偏移上屏**：`show_image(sensor帧, x, y)` 会报
>    `set vo layer attr failed 1`，屏整块不刷（就是那个"黑屏"）。
>    必须先 `canvas.draw_image(frame, 0, 15)` 合成，再 `show_image(canvas)`。
>    代码里 `probe_display()` 会在启动时自动试 direct/composite/copy，打印选了哪个。
> 3. **坐标只有一套：图像像素坐标**（左上角原点）。串口发的是它，`draw_xxx` 画的是它。
>    显示层的 (x,y) 只决定"贴在屏上哪儿"，不参与 dx/dy 计算。
> 4. **一次运行只有一种 VO 输出**：`Display.VIRT` → 只有 IDE 有图、面板全黑；
>    `Display.ST7701` → 面板 + IDE 都有。"LCD 黑屏但 IDE 有画面"就是选错了这个模式。
> 5. 怀疑屏/显示有问题：单独跑 [`LCD自检.py`](LCD自检.py)（不碰相机）。
>    怀疑视野丢没丢 / 光轴偏没偏：跑 [`画面自检.py`](画面自检.py)。
> 6. **换分辨率后 F103 那边必须同步**：`VISION_CX/CY`、`VISION_TARGET_X/Y`、`VISION_MM_PER_60PX`。

给 F103 工程 `DEBUG_MODE_CIRCLE` 用的相机侧程序：**找到目标圆心，通过 UART 发 `x,y\r\n`**。

## 文件

| 文件 | 说明 |
|---|---|
| `circle_track.py` | 全部逻辑。**调参只改这个文件顶部的配置区** |
| `main.py` | 上电自动运行入口，只有 3 行，不用改 |
| `test_logic.py` | 电脑端自检：不需要相机、不需要板子 |

板子上放 `main.py` + `circle_track.py` 两个文件（同一目录 / SD 卡根目录）。

## 接线（K230 → F103）

| K230 | F103 | 说明 |
|---|---|---|
| IO11（UART2_TXD） | PA3（USART2_RX） | **唯一的信号线**，F103 只收不发 |
| GND | GND | 必须共地 |
| IO12（UART2_RXD） | 不接 | F103 的 PA2 已经让给舵机 PWM 了 |

- 波特率 **115200 8N1**，和 `main.c` 里的 `USART2_Init(115200)` 一致；
- 换引脚就改 `circle_track.py` 顶部的 `UART_TX_PIN` / `UART_RX_PIN`，FPIOA 会自动按 `UART%d_TXD` 取功能名；
- OpenMV 相机上跑的话把 `UART_ID` 改成 `3`（UART3），走 `pyb.UART` 或 `machine.UART` 自动兜底。

## 协议（和 `Hardware/Serial.c` 严格对齐）

```
"318,242\r\n"        x,y 都是图像像素坐标，左上角原点，最多 4 位，范围 0~999
```

F103 那边的判据（`K230_LineCommit`）：只接受数字 / 逗号 / 空格，格式不是 `数字,数字` 或超过 999 就整行丢弃。
所以：

- **找不到目标时一个字节都不发**。F103 是"收到新帧才走一步"，发旧坐标会让它原地反复动作；
- 坐标已经夹在 `0..639` / `0..479`，不会触发 999 上限。

## 怎么调

按这个顺序，从"能锁住"到"锁得准"：

1. **先看能不能看到圈**：`DEBUG_DRAW = True`，在 IDE 里看黄框（ROI）里有没有绿十字。
   找不到就把 `CIRCLE_THRESHOLD` 从 `2600` 往下调（2600 → 2200 → 1800），误检多就往上调。
2. **半径范围**：`CIRCLE_R_MIN/MAX` 按实际距离试，默认 12~120 像素（VGA）。
   靶心环在画面里大概 40~80 像素时比较稳。
3. **兜底策略**：`USE_BLOB_FALLBACK = True` 时，圆找不到会退回"最大蓝块的中心"
   （LAB 阈值和 `E:\main.py` 里那套一样）。台面上只有蓝框没有白环时，这条更管用。
4. **稳定判据**（防误检乱跑）：
   - `HISTORY_WINDOW = 8` / `REQUIRED_HITS = 5`：最近 8 帧里至少 5 帧找到才算；
   - `REQUIRED_CONTINUITY = 3`：相邻两次圆心的跳变必须 ≤ `MAX_CENTER_JUMP = 60` 像素；
   - `SMOOTH_SHIFT = 2`：新值占 1/4 做平滑。车抖得厉害就加大，跟得慢就减小。
5. **发送频率**：`SEND_INTERVAL_MS = 100`（10 Hz）。F103 每收一行要等四个电机回 0x03 才取下一帧，
   发更快只会把接收缓冲挤爆。**别调成 0。**

## 联调

1. F103 把 `User/main.c` 顶部改成 `DEBUG_MODE_CIRCLE`，OLED 第 3 行应显示 `X=xxx Y=xxx`；
2. 相机上电后 IDE 里应该每 0.5 s 打一行 `SEND x,y src=circle hits=5 fps=..`；
3. 只接相机不接 F103 时，先看 `[ok]` 那段自检，再拿 USB-TTL 接 IO11 用串口助手看字节：

```powershell
python test_logic.py
```

自检做的是：把 `send_xy` 真正发出去的字节，逐字节喂给**照抄 C 代码的解析器**，
证明格式不会被 F103 打回；再验证稳定判据（连续多帧才发、跳变拒绝、丢失后静默）。

## 已知限制

- `find_circles` 是 Hough，K230 上帧率大约十几到几十 FPS，OpenMV H7 上会低不少；
- 阈值（`CIRCLE_THRESHOLD`、`BLUE_THRESHOLDS`）**必须按现场灯光重调**，默认值只是起点；
- 自检只验证逻辑/协议，识别效果得上板子看画面。
