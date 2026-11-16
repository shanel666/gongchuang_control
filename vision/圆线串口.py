# -*- coding: utf-8 -*-
'''
K230 视觉 · 串口命令版（circle / linear）
平台：立创·庐山派 K230 CanMV / Lite K230D

主控发一行字符串，K230 切模式并持续回数据：

    主控 -> K230（USART2，115200 8N1）
        "circle"   开始找圆心，持续回 C 帧
        "linear"   开始找直线，持续回 L 帧
        "stop"     停止回发

    K230 -> 主控
        C<x>,<y>,<r>    圆心像素坐标 + 半径       例 C318,242,41
        L<k>,<psi>      斜率×100 + 偏航修正 0.01° 例 L4,192  /  L-12,-345
        NG              这次没找到

    k   = 斜率 ×100（整数，4 就是 0.04）
    psi = 需要的偏航修正角，单位 0.01°，
          正 = 顺时针转，负 = 逆时针转
          例 psi=192 -> +1.92°

接线（立创庐山派 GH1.25 串口2 座子）：
    K230 T(GPIO11) -> F103 PA3 (USART2_RX)
    K230 R(GPIO12) <- F103 PA2 (USART2_TX)
    GND 共地；座子的 V 是 5V，别接进 F103
=====================================================================
'''

import time
import math

try:
    import os
except Exception:
    os = None

try:
    from media.sensor import *
    from media.display import *
    from media.media import *
    HAS_MEDIA = True
except Exception:
    HAS_MEDIA = False

try:
    from machine import UART as MachineUART
    from machine import FPIOA
    HAS_MACHINE = True
except Exception:
    HAS_MACHINE = False


# ============================ 一、串口 ============================
BAUD      = 115200
UART_ID   = 2
UART_TX   = 11          # GPIO11 = UART2_TXD -> F103 PA3
UART_RX   = 12          # GPIO12 = UART2_RXD <- F103 PA2
SEND_EVERY_MS = 100     # 有数据时多久回一帧（10 Hz 够了，主控只显示）
NG_EVERY_MS   = 500     # 没找到时多久回一次 NG（别刷屏）

# ============================ 二、图像 ============================
LCD_W = 800
LCD_H = 480

# ---- 屏幕 ----
# 立创·庐山派 K230 的 3.1 寸 MIPI 屏是**扩展板**（不在主板上），HDMI 也是扩展板。
#   没接屏幕 -> Display.init 会失败或什么都不显示，这是正常的，串口照样工作。
#   想看画面有两种办法：
#     a) USE_DISPLAY = True 且接了屏：直接看屏
#     b) USE_DISPLAY = True 但不接屏：CanMV IDE 里那个帧缓冲窗口也能看（to_ide=True）
#   只想稳定跑串口、不想被显示拖累：USE_DISPLAY = False
USE_DISPLAY  = True
LCD_DRIVER   = "ST7701"    # 3.1 寸 MIPI 屏用 ST7701；HDMI 扩展板改成 "LT9611"

# 画面中心（640×480 的中心）。K230 屏上会画一个十字，方便肉眼对比偏差。
IMG_CX = 320
IMG_CY = 240
IMG_W = 640
IMG_H = 480

THRESH       = 70
THRESH_SWEEP = (0, -25, 40)   # 阈值扫描：只在上一轮不够用时才试下一个
DILATE_PX    = 1
STRIDE       = 1

# ============================ 三、圆孔判据 ============================
MIN_SIDE = 22
MIN_AREA = 180
FILL_MIN = 0.80         # 实心度：孔≈1.0；环间细缝 0.1~0.75
AR_MIN   = 0.55         # 边上会被拍成椭圆，放宽
AR_MAX   = 1.80
ENC_MIN_HITS = 4        # 8 方向里至少几个"外面是黑的"，不满足就降级成"宽松候选"
EDGE_MARGIN  = 1

# 宽松档（只用于"只看见 1 个圆时找伴圆"）
LOOSE_MIN_SIDE = 14
LOOSE_MIN_AREA = 60
LOOSE_FILL_MIN = 0.50
LOOSE_AR_MIN   = 0.45
LOOSE_AR_MAX   = 2.20

# ============================ 四、直线 ============================
EXPECT_AXIS = "H"       # 对正时圆连线在画面里的方向：H=左右（斜率小）/ V=前后
K_MAX       = 0.35      # |斜率| 上限，超了判 NG（车不可能这么歪）
TARGETS_MAX = 3
MIN_SPACING = 25.0
MIN_SPAN    = 40.0
K_SPAN_MIN  = 8.0
COMPANION_SIZE_MIN = 0.45
COMPANION_SIZE_MAX = 1.70
COMPANION_MAX_DIST = 460.0

MODE_IDLE   = "idle"
MODE_CIRCLE = "circle"
MODE_LINEAR = "linear"

VERSION = "圆线串口 v1（circle / linear 命令版）"

UART_OBJ = None        # 全局留着，退出时统一 deinit（省得下次重跑串口说"已被占用"）
DISPLAY_OK = False     # 显示初始化成功才画屏；没接屏就一路跳过，串口不受影响

_DIR8 = [(1.0, 0.0), (0.7071, 0.7071), (0.0, 1.0), (-0.7071, 0.7071),
         (-1.0, 0.0), (-0.7071, -0.7071), (0.0, -1.0), (0.7071, -0.7071)]


# ============================ 五、协议（纯逻辑，可主机测试） ============================

def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def exitpoint():
    """调一次 os.exitpoint()。

    CanMV K230 的长循环里必须每轮调，否则 IDE 的"停止"按钮和 Ctrl-C 都不生效，
    表现成"程序卡死了/停不下来"。电脑上没有这个函数，忽略即可。
    """
    try:
        if os is not None:
            os.exitpoint()
    except Exception:
        pass


def new_rx_state():
    """接收状态：buf 是半行缓冲，over 表示"这一行已经超长，等到换行才恢复"。"""
    return {"buf": [], "over": False}


def feed_lines(state, data):
    """把串口收到的字节喂进去，返回这次凑齐的所有完整行（小写、去空白）。

    state 跨帧保存半行（见 new_rx_state）。一行最多 16 字符，超长整行丢弃，
    丢到下一个换行符为止 —— 这样不会把超长行的后半截当成一条新命令。
    K230 的 uart.read() 没数据时返回 None，这里也要用。
    """
    out = []
    if not data:
        return out

    buf = state["buf"]
    for b in data:
        c = chr(b)
        if c == "\r" or c == "\n":
            if state["over"]:
                state["over"] = False          # 超长行到此结束，正常恢复
                del buf[:]
            elif buf:
                out.append("".join(buf).strip().lower())
                del buf[:]
        elif state["over"]:
            continue                           # 还在丢超长行的剩余字节
        elif len(buf) < 16:
            buf.append(c)
        else:
            del buf[:]
            state["over"] = True
    return out


def classify_command(line):
    """把一行命令变成 MODE_xxx；不认识的返回 None。"""
    s = (line or "").strip().lower()
    if s.startswith("circle"):
        return MODE_CIRCLE
    if s.startswith("linear"):
        return MODE_LINEAR
    if s.startswith("stop") or s.startswith("idle") or s.startswith("none"):
        return MODE_IDLE
    return None


def fmt_circle(x, y, r):
    """C<x>,<y>,<r>，各字段 0~999，总长 ≤ 11 字节。"""
    return "C%d,%d,%d" % (clamp(int(x), 0, 999), clamp(int(y), 0, 999),
                          clamp(int(r), 0, 999))


def fmt_line(k, psi_deg):
    """L<k*100>,<psi*100>，总长 ≤ 13 字节。"""
    k100 = clamp(int(round(k * 100.0)), -9999, 9999)
    psi100 = clamp(int(round(psi_deg * 100.0)), -18000, 18000)
    return "L%d,%d" % (k100, psi100)


def fmt_ng():
    return "NG"


# ============================ 六、找圆（纯逻辑） ============================

def blob_ar(b):
    h = b.h()
    return b.w() / float(h) if h > 0 else 0.0


def blob_fill(b):
    r = (b.w() + b.h()) * 0.25
    return b.pixels() / (math.pi * r * r) if r > 0 else 0.0


def blob_r(b):
    return (b.w() + b.h()) * 0.25


def touches_edge(b, W, H):
    m = EDGE_MARGIN
    return (b.x() <= m or b.y() <= m
            or b.x() + b.w() >= W - m or b.y() + b.h() >= H - m)


def geom_level(b, W, H):
    """2 = 严格（能当主圆）；1 = 宽松（可能被截断，只能当伴圆）；0 = 不合格。"""
    if b.w() < MIN_SIDE or b.h() < MIN_SIDE:
        return 0
    if b.pixels() < MIN_AREA:
        return 0

    ar = blob_ar(b)
    fill = blob_fill(b)

    if AR_MIN <= ar <= AR_MAX and fill >= FILL_MIN and not touches_edge(b, W, H):
        return 2

    if (LOOSE_AR_MIN <= ar <= LOOSE_AR_MAX and fill >= LOOSE_FILL_MIN
            and b.w() >= LOOSE_MIN_SIDE and b.h() >= LOOSE_MIN_SIDE
            and b.pixels() >= LOOSE_MIN_AREA):
        return 1

    return 0


def dedup_concentric(cands):
    """同心去重：圆心重合时只留小的（最内圆）。"""
    out = []
    for b in sorted(cands, key=lambda x: x.w() * x.h()):
        keep = True
        for c in out:
            lim = min(blob_r(b), blob_r(c))
            if lim < 1.0:
                continue
            if math.hypot(b.cx() - c.cx(), b.cy() - c.cy()) < lim:
                keep = False
                break
        if keep:
            out.append(b)
    return out


def pick_largest(cands):
    best = None
    for b in cands:
        if best is None or b.pixels() > best.pixels():
            best = b
    return best


def thresh_candidates():
    out = []
    for off in THRESH_SWEEP:
        th = clamp(THRESH + off, 10, 200)
        if th not in out:
            out.append(th)
    return out


# ============================ 七、拟合（纯逻辑） ============================

def fit_line(points):
    """TLS 拟合，返回 (px, py, cx, cy, resid)；点少于 2 或全重合返回 None。"""
    n = len(points)
    if n < 2:
        return None

    cx = sum(p[0] for p in points) / float(n)
    cy = sum(p[1] for p in points) / float(n)

    sxx = syy = sxy = 0.0
    for x, y in points:
        dx = x - cx
        dy = y - cy
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy

    if (sxx + syy) < 1e-6:
        return None

    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    px = math.cos(theta)
    py = math.sin(theta)

    resid = 0.0
    for x, y in points:
        d = abs(px * (y - cy) - py * (x - cx))
        if d > resid:
            resid = d
    return (px, py, cx, cy, resid)


def max_pair_dist(pts):
    d = 0.0
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            v = math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1])
            if v > d:
                d = v
    return d


def min_pair_dist(pts):
    d = None
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            v = math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1])
            if d is None or v < d:
                d = v
    return 0.0 if d is None else d


def best_triple(points):
    """多于 3 个点时挑最共线、间距够、跨度大的 3 个。"""
    if len(points) <= TARGETS_MAX:
        return list(points)
    n = len(points)
    best = None
    best_score = None
    for i in range(n):
        for j in range(i + 1, n):
            for k in range(j + 1, n):
                tri = [points[i], points[j], points[k]]
                if min_pair_dist(tri) < MIN_SPACING:
                    continue
                span = max_pair_dist(tri)
                if span < MIN_SPAN:
                    continue
                fit = fit_line(tri)
                if fit is None:
                    continue
                score = fit[4] - 0.002 * span
                if best_score is None or score < best_score:
                    best_score = score
                    best = tri
    return best if best is not None else list(points[:TARGETS_MAX])


def slope_between(ax, ay, bx, by, axis=None):
    """按期望方向算斜率。None = 无穷大（方向完全跑偏）。"""
    axis = EXPECT_AXIS if axis is None else axis
    dx = bx - ax
    dy = by - ay
    if axis == "H":
        return None if abs(dx) < 1e-6 else dy / dx
    return None if abs(dy) < 1e-6 else dx / dy


def slope_ok(k):
    return (k is not None) and (abs(k) <= K_MAX)


def ref_vector(axis=None):
    axis = EXPECT_AXIS if axis is None else axis
    return (0.0, -1.0) if axis == "V" else (1.0, 0.0)


def line_result(points, axis=None):
    """圆心点集 -> (k, psi_deg, gate_ok, resid, used)。

    k        期望方向的斜率
    psi_deg  需要的偏航修正角（度），正 = 顺时针转
    gate_ok  |k| <= K_MAX 且点够
    """
    axis = EXPECT_AXIS if axis is None else axis
    if len(points) < 2:
        return (None, None, False, None, [])

    used = best_triple(points)
    fit = fit_line(used)
    if fit is None:
        return (None, None, False, None, used)

    px, py, cx, cy, resid = fit
    rx, ry = ref_vector(axis)

    # 直线没有方向：统一到参考轴那一侧，消掉点顺序和相机装反 180° 的影响
    if rx * px + ry * py < 0.0:
        px, py = -px, -py

    psi = math.degrees(math.atan2(rx * py - ry * px, rx * px + ry * py))
    while psi > 90.0:
        psi -= 180.0
    while psi <= -90.0:
        psi += 180.0

    xs = [p[0] for p in used]
    ys = [p[1] for p in used]
    spanx = max(xs) - min(xs)
    spany = max(ys) - min(ys)

    k = None
    if axis == "H":
        if abs(px) > 1e-6 and spanx >= K_SPAN_MIN:
            k = py / px
    else:
        if abs(py) > 1e-6 and spany >= K_SPAN_MIN:
            k = px / py

    return (k, psi, slope_ok(k), resid, used)


def find_companion(anchor, pool, axis=None):
    """只看见 1 个圆时，沿小斜率带找第二个圆（允许被画面截断）。"""
    axis = EXPECT_AXIS if axis is None else axis
    ax, ay = anchor.cx(), anchor.cy()
    ra = blob_r(anchor)
    best = None

    for b in pool:
        if b is anchor:
            continue
        rb = blob_r(b)
        if rb <= 0:
            continue
        ratio = rb / (ra if ra > 0 else 1.0)
        if ratio < COMPANION_SIZE_MIN or ratio > COMPANION_SIZE_MAX:
            continue

        dist = math.hypot(b.cx() - ax, b.cy() - ay)
        if dist > COMPANION_MAX_DIST:
            continue
        if dist < max(MIN_SPACING, 0.8 * (ra + rb)):
            continue                                   # 挨太近 = 自己套着的圈

        if not slope_ok(slope_between(ax, ay, b.cx(), b.cy(), axis)):
            continue

        if best is None or b.pixels() > best.pixels():
            best = b

    return best


# ============================ 八、需要图像的部分 ============================

def enclosed_hits(img_bin, b):
    """8 个方向里有几个"外面是黑的"。"""
    cx, cy = b.cx(), b.cy()
    r0 = blob_r(b)
    W, H = img_bin.width(), img_bin.height()
    hit = 0
    for c, s in _DIR8:
        for r in (r0 + 1.0, r0 + 2.0, r0 + 3.0):
            x = int(cx + c * r)
            y = int(cy + s * r)
            if 0 <= x < W and 0 <= y < H and img_bin.get_pixel(x, y) > 128:
                hit += 1
                break
    return hit


def run_pass(gray_src, th, W, H, can_copy):
    """一轮：二值化 -> 膨胀 -> 连通域 -> 定级。返回 (blobs, cands)。"""
    g = gray_src
    if can_copy:
        try:
            g = gray_src.copy()
        except Exception:
            g = gray_src
    g.binary([(0, th)])
    if DILATE_PX > 0:
        g.dilate(DILATE_PX)

    blobs = g.find_blobs([(128, 255)],
                         x_stride=STRIDE, y_stride=1,
                         pixels_threshold=LOOSE_MIN_AREA,
                         area_threshold=LOOSE_MIN_AREA,
                         invert=True, merge=False)

    cands = []
    for b in blobs:
        lv = geom_level(b, W, H)
        if lv == 0:
            cands.append((b, 0, -1))
            continue
        hits = enclosed_hits(g, b)
        if lv == 2 and ENC_MIN_HITS > 0 and hits < ENC_MIN_HITS:
            lv = 1
        cands.append((b, lv, hits))
    return blobs, cands


def draw_label(img, x, y, text, color, size=18):
    W, H = img.width(), img.height()
    x = max(0, min(x, W - 1))
    y = max(0, min(y, H - size))
    try:
        img.draw_string_advanced(x, y, size, text, color=color)
        return
    except Exception:
        pass
    try:
        img.draw_string(x, y, text, color=color, scale=1)
    except Exception:
        pass


def open_uart():
    """打开 UART2。参数必须用固件给的常量，别自己写数字。

    ★ 踩过的坑：把 bits/parity/stop 传成 0（以为 0 是"默认值"），固件不报错，
      但帧格式变成"0 位数据位"，波特率对、数据全收不到 —— 现象和"线没接"一样。
      必须用 UART.EIGHTBITS / PARITY_NONE / STOPBITS_ONE。
    """
    if not HAS_MACHINE:
        print("没有 machine 模块，串口打不开")
        return None

    fpioa = FPIOA()
    fpioa.set_function(UART_TX, getattr(FPIOA, "UART%d_TXD" % UART_ID))
    fpioa.set_function(UART_RX, getattr(FPIOA, "UART%d_RXD" % UART_ID))

    uid = getattr(MachineUART, "UART%d" % UART_ID, UART_ID)

    try:
        u = MachineUART(uid, baudrate=BAUD,
                        bits=MachineUART.EIGHTBITS,
                        parity=MachineUART.PARITY_NONE,
                        stop=MachineUART.STOPBITS_ONE)
    except Exception as e:
        print("标准 8N1 构造失败（%s），退回最简参数" % e)
        u = MachineUART(uid, BAUD)

    print("UART%d 就绪 %d 8N1 (T=GPIO%d R=GPIO%d)"
          % (UART_ID, BAUD, UART_TX, UART_RX))
    return u


# ============================ 九、主循环 ============================

def close_uart():
    """退出时关串口。不关也能跑，但重跑脚本时更干净。"""
    global UART_OBJ
    try:
        if UART_OBJ is not None:
            UART_OBJ.deinit()
            print("串口已关闭")
    except Exception:
        pass
    UART_OBJ = None


def show(img):
    """把画面送到屏 / IDE 帧缓冲。没屏或出错就静默跳过 —— 绝不影响串口。"""
    global DISPLAY_OK
    if not DISPLAY_OK:
        return

    # 画面中心画个黄十字：肉眼就能看出圆心离中心多远
    try:
        img.draw_cross(IMG_CX, IMG_CY, color=(255, 255, 0), size=10, thickness=2)
    except Exception:
        pass

    try:
        Display.show_image(img,
                           x=round((LCD_W - img.width()) / 2),
                           y=round((LCD_H - img.height()) / 2))
    except Exception as e:
        DISPLAY_OK = False
        print("show_image 失败，后面不再画屏（串口继续）：", e)


def main():
    global UART_OBJ
    global DISPLAY_OK

    sensor = Sensor(width=1280, height=960)
    sensor.reset()
    sensor.set_framesize(width=IMG_W, height=IMG_H)
    sensor.set_pixformat(Sensor.RGB565)

    # ---- 显示：接不上屏也不能影响串口 ----
    DISPLAY_OK = False
    if USE_DISPLAY:
        try:
            drv = getattr(Display, LCD_DRIVER, Display.ST7701)
            Display.init(drv, width=LCD_W, height=LCD_H, to_ide=True)
            DISPLAY_OK = True
            print("显示已初始化：%s %dx%d（画面同时送到 IDE 帧缓冲窗口）"
                  % (LCD_DRIVER, LCD_W, LCD_H))
        except Exception as e:
            print("显示初始化失败：%s" % e)
            print("  -> 没接 3.1 寸屏 / HDMI 扩展板时这是正常的，串口继续工作。")
            print("  -> 想看画面：接屏，或用 CanMV IDE 的帧缓冲窗口。")

    MediaManager.init()
    sensor.run()
    clock = time.clock()

    u = open_uart()
    UART_OBJ = u
    print("=== %s ===" % VERSION)
    print("  等主控发 circle / linear / stop")
    print("  串口在跑的时候，IDE 里会打印 [cmd] 和每一帧回包，这就是 K230 看到的东西")

    mode = MODE_IDLE
    rx_state = new_rx_state()
    can_copy = True
    last_send = 0
    last_ng = 0
    force = False
    last_reply = "-"
    th_list = thresh_candidates()

    while True:
        exitpoint()                     # 让 IDE 的停止按钮能生效
        clock.tick()
        img = sensor.snapshot()

        # ---------- 1) 收命令 ----------
        try:
            data = u.read() if u is not None else None
        except Exception:
            data = None
        if data:
            for line in feed_lines(rx_state, data):
                cmd = classify_command(line)
                print("[cmd] %r -> %s" % (line, cmd))
                if cmd == MODE_CIRCLE or cmd == MODE_LINEAR:
                    mode = cmd
                    force = True
                    last_ng = 0
                elif cmd == MODE_IDLE:
                    mode = MODE_IDLE

        if mode == MODE_IDLE:
            draw_label(img, 0, 0, "IDLE  wait for cmd", (255, 255, 0), 22)
            draw_label(img, 0, 28, "circle / linear", (200, 200, 200), 20)
            show(img)
            continue

        # ---------- 2) 灰度源图（每轮阈值都从它复制） ----------
        try:
            gray_src = img.to_grayscale(copy=True)
        except TypeError:
            gray_src = img.to_grayscale()
            can_copy = False
        W, H = gray_src.width(), gray_src.height()

        passes = th_list if can_copy else th_list[:1]

        # ---------- 3) 按模式处理 ----------
        reply = None
        info = ""

        if mode == MODE_CIRCLE:
            # 挑"严格圆最多"的那一轮，取最大的那个
            best = None
            used_th = passes[0]
            for th in passes:
                blobs, cands = run_pass(gray_src, th, W, H, can_copy)
                strict = dedup_concentric([c[0] for c in cands if c[1] == 2])
                if strict:
                    best = pick_largest(strict)
                    used_th = th
                    break
                if not can_copy:
                    break
            if best is not None:
                reply = fmt_circle(best.cx(), best.cy(), blob_r(best))
                info = "th=%d" % used_th
                try:
                    img.draw_rectangle(best.rect(), color=(0, 255, 0))
                    img.draw_circle(best.cx(), best.cy(), int(blob_r(best)),
                                    color=(255, 0, 0), thickness=2)
                    img.draw_cross(best.cx(), best.cy(), color=(255, 0, 0),
                                   size=6, thickness=2)
                except Exception:
                    pass
            else:
                reply = fmt_ng()
                info = "no circle"

        else:  # MODE_LINEAR
            centers = []
            loose_all = []
            used_th = passes[0]
            for th in passes:
                blobs, cs = run_pass(gray_src, th, W, H, can_copy)
                strict = dedup_concentric([c[0] for c in cs if c[1] == 2])
                if (not centers) or (len(strict) > len(centers)):
                    used_th = th
                    centers = strict
                    loose_all = dedup_concentric([c[0] for c in cs if c[1] >= 1])
                if len(strict) >= 2 or not can_copy:
                    break

            pts = centers
            if len(pts) == 1 and loose_all:
                comp = find_companion(pts[0], loose_all)
                if comp is not None:
                    pts = [pts[0], comp]

            coords = [(float(b.cx()), float(b.cy())) for b in pts]
            k, psi, gate_ok, resid, used = line_result(coords)

            # 斜率超门限：丢掉最歪的点再试一次
            if (not gate_ok) and len(coords) >= 3:
                best2 = None
                best_span = -1.0
                for i in range(len(coords)):
                    for j in range(i + 1, len(coords)):
                        d = math.hypot(coords[i][0] - coords[j][0],
                                       coords[i][1] - coords[j][1])
                        if d < K_SPAN_MIN:
                            continue
                        kk = slope_between(coords[i][0], coords[i][1],
                                           coords[j][0], coords[j][1])
                        if not slope_ok(kk):
                            continue
                        if d > best_span:
                            best_span = d
                            best2 = [coords[i], coords[j]]
                if best2:
                    k, psi, gate_ok, resid, used = line_result(best2)

            if gate_ok:
                reply = fmt_line(k, psi)
                info = "k=%.3f psi=%.2f th=%d n=%d" % (k, psi, used_th, len(coords))
                for i, (ux, uy) in enumerate(used):
                    try:
                        img.draw_cross(int(ux), int(uy), color=(0, 255, 0),
                                       size=8, thickness=2)
                    except Exception:
                        pass
                    draw_label(img, int(ux) + 8, int(uy) - 22, "#%d" % i,
                               (0, 255, 0), 18)
            else:
                reply = fmt_ng()
                info = "k not ok (%s) n=%d" % (
                    "None" if k is None else "%.3f" % k, len(coords))

        # ---------- 4) 回发 ----------
        now = time.ticks_ms()
        is_ng = (reply == fmt_ng())
        gap = NG_EVERY_MS if is_ng else SEND_EVERY_MS
        if force or time.ticks_diff(now, last_send) >= gap:
            last_send = now
            force = False
            if reply is not None and u is not None:
                try:
                    u.write(reply + "\r\n")
                except Exception:
                    pass
                last_reply = reply
            print("[%s] %s  %s" % (mode, reply, info))

        # ---------- 5) 画面（屏 + IDE 帧缓冲）----------
        draw_label(img, 0, 0, "%s  %s" % (mode.upper(), last_reply),
                   (0, 255, 0), 22)
        draw_label(img, 0, 28, info, (255, 255, 255), 18)
        show(img)


if __name__ == "__main__":
    if HAS_MEDIA:
        try:
            main()
        except KeyboardInterrupt:
            print("\n已停止（Ctrl-C / IDE 停止）")
        finally:
            close_uart()
    else:
        print("没找到 media.sensor：这份脚本要在 CanMV K230 上跑。")
