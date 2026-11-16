# -*- coding: utf-8 -*-
# =====================================================================
#  圆心跟踪视觉程序 —— 在 CanMV(K230) 里用 OpenMV 那套 API 写
#
#  做什么：
#      每帧算出目标圆心在图像里的像素坐标 (x, y)，通过 UART 发给 F103：
#          "318,242\r\n"
#      对应 f103c8t6 工程 main.c 里的 DEBUG_MODE_CIRCLE：
#          USART2 / 115200 8N1 / 只收不发（F103 端 PA3）
#          x > 320 -> 后退   x < 320 -> 前进
#          y > 240 -> 右移   y < 240 -> 左移
#
#  三条硬约定（改代码时别破坏）：
#    1. 画面必须是 640x480（VGA），F103 的目标点 (320, 240) 就是按这个算的；
#    2. 没找到目标就【一个字节都不发】，绝不重复上报旧坐标
#       —— F103 那边是"收到一帧新数据才走一步"，发旧值会让它原地乱动；
#    3. 必须连续多帧一致才发，单帧误检不发，防止车被一个假圆带走。
#
#  平台：
#    - CanMV K230（CanMV 固件 / CanMV IDE）：走 machine.UART + FPIOA
#    - OpenMV 相机（OpenMV 固件 / OpenMV IDE）：走 machine.UART 或 pyb.UART
#    两边都不用改代码，靠 try/except 自动挑。找不到 csi 就用 sensor，
#    找不到 sensor 就用 csi —— 和 E:\main.py 里那套写法一致。
# =====================================================================

import time

try:
    import sensor                       # OpenMV / CanMV 老写法
except ImportError:
    sensor = None

try:
    import csi                          # 新版固件的 csi 写法
except ImportError:
    csi = None

try:
    from machine import UART as MachineUART
except ImportError:
    MachineUART = None

try:
    from machine import FPIOA           # 只有 CanMV K230 有这个（引脚复用）
except ImportError:
    FPIOA = None

try:
    import pyb                          # OpenMV 的老 API
except ImportError:
    pyb = None


# =====================================================================
#  1. 配置区：接线/调参只改这里
# =====================================================================

# ---------------- 画面 ----------------
FRAME_W = 640
FRAME_H = 480
USE_CSI_CAMERA = True           # 有 csi 模块就优先用 csi（新版固件），否则退回 sensor
LOCK_CAMERA = False             # True = 锁死增益/白平衡/曝光，灯光不变时更稳
SKIP_FRAMES_MS = 1200           # 上电等相机自动曝光稳定

# ---------------- 串口（到 F103 的 USART2） ----------------
UART_ENABLED = True
UART_ID = 2                     # CanMV K230: UART2；OpenMV 想用 UART3 就改成 3
UART_BAUDRATE = 115200          # 必须和 main.c 里 USART2_Init(115200) 一致
UART_TX_PIN = 11                # K230: IO11 = UART2_TXD  -> 接 F103 的 PA3
UART_RX_PIN = 12                # K230: IO12 = UART2_RXD  -> 可不接（F103 只收不发）
SEND_INTERVAL_MS = 100          # 最快 10 Hz；F103 每收一行走一步，发太快没意义

# ---------------- ROI：画面边缘不要 ----------------
ROI_LEFT_PERCENT = 0            # 左边裁掉百分之几
ROI_TOP_PERCENT = 0
ROI_BOTTOM_PERCENT = 0

# ---------------- 找圆（Hough，主策略） ----------------
CIRCLE_X_STRIDE = 2
CIRCLE_Y_STRIDE = 2
CIRCLE_THRESHOLD = 2600         # 越小越容易误检；识别不到就 3000 -> 2600 -> 2200 往下试
CIRCLE_R_MIN = 12
CIRCLE_R_MAX = 120
CIRCLE_R_STEP = 2
CIRCLE_R_MARGIN = 12

# ---------------- 兜底：蓝框中心（find_blobs） ----------------
# 和 E:\main.py 用的是同一组 LAB 阈值；圆环找不到时用蓝色块中心顶上。
USE_BLOB_FALLBACK = True
BLUE_THRESHOLDS = [
    (18, 92, -10, 32, -72, -10),
    (40, 100, -16, 34, -50, -6),
]
BLUE_X_STRIDE = 3
BLUE_Y_STRIDE = 3
MIN_BLUE_PIXELS = 60
MIN_BLUE_AREA = 60
MIN_BLOB_SHORT_SIDE = 10

# ---------------- 稳定判据 ----------------
HISTORY_WINDOW = 8              # 回看最近 8 帧
REQUIRED_HITS = 5               # 其中至少 5 帧找到
REQUIRED_CONTINUITY = 3         # 且相邻两次圆心跳变不超过 MAX_CENTER_JUMP
MAX_CENTER_JUMP = 60            # 像素；小于它的位移才算"同一颗圆"
SMOOTH_SHIFT = 2                # 平滑系数：新值占 1/4，旧值占 3/4

# ---------------- 调试 ----------------
DEBUG_DRAW = True               # 在画面上画框/十字（IDE 里能直接看到）
DEBUG_PRINT = True
DEBUG_PRINT_INTERVAL_MS = 500
ERROR_SLEEP_MS = 50


# =====================================================================
#  2. 基础工具
# =====================================================================

def clamp(v, low, high):
    if v < low:
        return low
    if v > high:
        return high
    return v


def roi_clip(img, x, y, w, h):
    """把 ROI 裁到画面里面，防止越界。"""
    x = int(x)
    y = int(y)
    w = int(w)
    h = int(h)
    if x < 0:
        w = w + x
        x = 0
    if y < 0:
        h = h + y
        y = 0
    if x + w > img.width():
        w = img.width() - x
    if y + h > img.height():
        h = img.height() - y
    if w < 1:
        w = 1
    if h < 1:
        h = 1
    return (x, y, w, h)


def detection_roi(img):
    x = img.width() * ROI_LEFT_PERCENT // 100
    y = img.height() * ROI_TOP_PERCENT // 100
    bottom = img.height() * ROI_BOTTOM_PERCENT // 100
    return roi_clip(img, x, y, img.width() - x, img.height() - y - bottom)


# =====================================================================
#  3. 相机 / 串口
# =====================================================================

def init_camera():
    if USE_CSI_CAMERA and csi != None:
        cam = csi.CSI()
        cam.reset()
        cam.pixformat(csi.RGB565)
        try:
            cam.framesize(csi.VGA)
        except Exception:
            cam.framesize(csi.QVGA)
        cam.snapshot(time=SKIP_FRAMES_MS)
        if LOCK_CAMERA:
            try:
                cam.auto_gain(False, gain_db=cam.gain_db())
                cam.auto_whitebal(False, rgb_gain_db=cam.rgb_gain_db())
                cam.auto_exposure(False, exposure_us=cam.exposure_us())
            except Exception:
                pass
        print("相机：csi")
        return cam

    if sensor == None:
        raise Exception("既没有 sensor 也没有 csi 模块，固件不对")

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.VGA)
    sensor.skip_frames(time=SKIP_FRAMES_MS)
    if LOCK_CAMERA:
        try:
            sensor.set_auto_gain(False)
            sensor.set_auto_whitebal(False)
            sensor.set_auto_exposure(False)
        except Exception:
            pass
    print("相机：sensor")
    return sensor


def uart_id_constant():
    """CanMV 要求传 UART.UART2 这种常量，而不是裸 2；没有常量就退回整数。"""
    name = "UART%d" % UART_ID
    try:
        return getattr(MachineUART, name)
    except (AttributeError, TypeError):
        return UART_ID


def fpioa_set(fpioa, pin, func_name):
    if pin < 0:
        return
    try:
        func = getattr(FPIOA, func_name)
    except (AttributeError, TypeError):
        return                      # 该引脚没有这个复用功能，交给下面的 init 报错
    try:
        fpioa.set_function(pin, func)
    except Exception as e:
        print("FPIOA %s 设置失败:" % func_name, e)


def setup_fpioa():
    """CanMV K230 必须先把引脚复用到 UART 功能上，否则 UART 打不开。"""
    if FPIOA == None:
        return
    try:
        fpioa = FPIOA()
    except Exception as e:
        print("FPIOA 初始化失败:", e)
        return
    fpioa_set(fpioa, UART_TX_PIN, "UART%d_TXD" % UART_ID)
    fpioa_set(fpioa, UART_RX_PIN, "UART%d_RXD" % UART_ID)


def init_uart():
    if not UART_ENABLED:
        return None

    if MachineUART != None:
        setup_fpioa()
        uid = uart_id_constant()

        # 不同固件 UART() 的参数签名不一样，从最全的往下试。
        try:
            u = MachineUART(uid, baudrate=UART_BAUDRATE,
                            timeout=20, timeout_char=20)
            print("UART OK: machine.UART(%s) %d" % (uid, UART_BAUDRATE))
            return u
        except Exception:
            pass
        try:
            u = MachineUART(uid, baudrate=UART_BAUDRATE)
            print("UART OK: machine.UART(%s) %d" % (uid, UART_BAUDRATE))
            return u
        except Exception:
            pass
        try:
            u = MachineUART(uid, UART_BAUDRATE)
            print("UART OK: machine.UART(%s) %d" % (uid, UART_BAUDRATE))
            return u
        except Exception as e:
            print("machine.UART 失败:", e)

    if pyb != None:
        try:
            u = pyb.UART(UART_ID, UART_BAUDRATE, timeout_char=20)
            print("UART OK: pyb.UART(%d) %d" % (UART_ID, UART_BAUDRATE))
            return u
        except Exception as e:
            print("pyb.UART 失败:", e)

    print("串口没起来：画面还能看，但 F103 不会收到任何数据")
    return None


def send_xy(uart, x, y):
    """发一行 "x,y\\r\\n"。格式必须严格——F103 那边数字超过 4 位或没有逗号就整行丢弃。"""
    if uart == None:
        return
    x = clamp(int(x), 0, FRAME_W - 1)
    y = clamp(int(y), 0, FRAME_H - 1)
    msg = "%d,%d\r\n" % (x, y)
    try:
        uart.write(msg)
    except Exception:
        pass


# =====================================================================
#  4. 识别：先找圆，找不到再退回蓝框中心
# =====================================================================

def find_circles_safe(img, roi):
    """find_circles 在个别固件上参数签名不同，这里降级重试，不让它把主循环打挂。"""
    try:
        return img.find_circles(roi=roi,
                               x_stride=CIRCLE_X_STRIDE,
                               y_stride=CIRCLE_Y_STRIDE,
                               threshold=CIRCLE_THRESHOLD,
                               r_min=CIRCLE_R_MIN,
                               r_max=CIRCLE_R_MAX,
                               r_step=CIRCLE_R_STEP,
                               r_margin=CIRCLE_R_MARGIN)
    except TypeError:
        pass
    except Exception:
        return []
    try:
        return img.find_circles(roi=roi,
                               threshold=CIRCLE_THRESHOLD,
                               r_min=CIRCLE_R_MIN,
                               r_max=CIRCLE_R_MAX)
    except Exception:
        return []


def circle_magnitude(c):
    try:
        return c.magnitude()
    except Exception:
        return 0


def find_blue_center(img, roi):
    """兜底：最大的蓝色块的中心。"""
    if not USE_BLOB_FALLBACK:
        return None
    try:
        blobs = img.find_blobs(BLUE_THRESHOLDS,
                              roi=roi,
                              merge=True,
                              margin=3,
                              x_stride=BLUE_X_STRIDE,
                              y_stride=BLUE_Y_STRIDE,
                              pixels_threshold=MIN_BLUE_PIXELS,
                              area_threshold=MIN_BLUE_AREA)
    except Exception:
        return None

    best = None
    best_pixels = 0
    for b in blobs:
        if b.w() < MIN_BLOB_SHORT_SIDE or b.h() < MIN_BLOB_SHORT_SIDE:
            continue
        if b.pixels() > best_pixels:
            best_pixels = b.pixels()
            best = b
    if best == None:
        return None
    return (best.cx(), best.cy())


def find_target(img, roi):
    """返回 (found, x, y, source)。source: "circle" / "blob" / "none"。"""
    circles = find_circles_safe(img, roi)

    best = None
    best_score = -1
    for c in circles:
        r = c.r()
        if r < CIRCLE_R_MIN or r > CIRCLE_R_MAX:
            continue
        # 分数同时看"投票数"和"半径"：靶心的大圆投票多，边上小噪点投票少。
        score = circle_magnitude(c) * 2 + r * 2
        if score > best_score:
            best_score = score
            best = c

    if best != None:
        return True, best.x(), best.y(), "circle"

    px = find_blue_center(img, roi)
    if px != None:
        return True, px[0], px[1], "blob"

    return False, 0, 0, "none"


# =====================================================================
#  5. 稳定器：连续多帧一致才算数，并做平滑
# =====================================================================

class CircleTracker(object):
    def __init__(self):
        self.hx = []
        self.hy = []
        self.sx = 0
        self.sy = 0
        self.valid = False
        self.hits = 0

    def reset(self):
        while len(self.hx) > 0:
            self.hx.pop(0)
        while len(self.hy) > 0:
            self.hy.pop(0)
        self.valid = False
        self.hits = 0

    def update(self, found, x, y):
        """喂一帧结果，返回 (ok, 平滑后x, 平滑后y)。ok=False 时不要发串口。"""
        if found:
            self.hx.append(x)
            self.hy.append(y)
        else:
            self.hx.append(-1)
            self.hy.append(-1)

        while len(self.hx) > HISTORY_WINDOW:
            self.hx.pop(0)
            self.hy.pop(0)

        hits = 0
        continuity = 0
        last_x = -1
        last_y = -1
        i = 0
        while i < len(self.hx):
            cx = self.hx[i]
            if cx >= 0:
                hits = hits + 1
                cy = self.hy[i]
                if last_x >= 0:
                    dx = cx - last_x
                    dy = cy - last_y
                    if dx * dx + dy * dy <= MAX_CENTER_JUMP * MAX_CENTER_JUMP:
                        continuity = continuity + 1
                last_x = cx
                last_y = cy
            i = i + 1
        self.hits = hits

        if hits < REQUIRED_HITS or continuity < REQUIRED_CONTINUITY:
            self.valid = False
            return False, 0, 0

        if self.valid:
            # 已经锁住了：慢慢跟，抗抖动
            self.sx = self.sx + ((x - self.sx) >> SMOOTH_SHIFT)
            self.sy = self.sy + ((y - self.sy) >> SMOOTH_SHIFT)
        else:
            # 刚重新捕获：直接跳到目标，别让平滑慢慢爬
            self.sx = x
            self.sy = y
            self.valid = True

        return True, self.sx, self.sy


# =====================================================================
#  6. 调试画面
# =====================================================================

def draw_debug(img, roi, x, y, source, ok, fps):
    if not DEBUG_DRAW:
        return
    try:
        img.draw_rectangle(roi, color=(255, 255, 0))
        if ok:
            img.draw_cross(int(x), int(y), color=(0, 255, 0), size=8)
            img.draw_circle(int(x), int(y), 12, color=(0, 255, 0))
        elif source != "none":
            img.draw_cross(int(x), int(y), color=(255, 0, 0), size=8)
        text = "LOST"
        if ok:
            text = "LOCK %d,%d" % (int(x), int(y))
        img.draw_string(4, 4, "%s %s FPS=%.1f" % (text, source, fps),
                        color=(255, 255, 255), scale=1)
    except Exception:
        pass


# =====================================================================
#  7. 主循环
# =====================================================================

def main():
    cam = init_camera()
    uart = init_uart()
    clock = time.clock()
    tracker = CircleTracker()
    last_send_ms = 0
    last_print_ms = 0
    print("圆心跟踪启动 %dx%d，发往 UART%d" % (FRAME_W, FRAME_H, UART_ID))

    while True:
        try:
            clock.tick()
            img = cam.snapshot()
            roi = detection_roi(img)
            found, tx, ty, source = find_target(img, roi)
            ok, ox, oy = tracker.update(found, tx, ty)
            fps = clock.fps()
            now = time.ticks_ms()

            if ok and time.ticks_diff(now, last_send_ms) >= SEND_INTERVAL_MS:
                last_send_ms = now
                send_xy(uart, ox, oy)

            draw_debug(img, roi, ox, oy, source, ok, fps)

            if DEBUG_PRINT and time.ticks_diff(now, last_print_ms) >= DEBUG_PRINT_INTERVAL_MS:
                last_print_ms = now
                if ok:
                    print("SEND %d,%d  src=%s  hits=%d  fps=%.1f" %
                          (int(ox), int(oy), source, tracker.hits, fps))
                else:
                    print("LOST  src=%s  hits=%d  fps=%.1f" %
                          (source, tracker.hits, fps))
        except Exception as e:
            print("ERR:", e)
            time.sleep_ms(ERROR_SLEEP_MS)
