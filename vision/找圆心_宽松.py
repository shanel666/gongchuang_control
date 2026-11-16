# -*- coding: utf-8 -*-
'''
单独找圆心 v4
平台：01Studio CanMV K230（CanMV 固件）

只做一件事：在画面里找"被黑环围住的那块亮孔"，画出来、打印坐标。

==================== v4 修的问题：脱开画面中心就检测不到 ====================
现象：圆明明整个在画面里，只要偏离画面中心就找不到。
原因不是判据，是**二值化阈值**：镜头有渐晕、补光也不均，画面边缘比中心暗。
固定 THRESH=70 时：
   · 边缘纸面灰度掉到 70 以下 -> 纸面也被当成"墨水" -> 亮孔被吞掉；
   · 或者边缘的墨水灰度升到 70 以上 -> 圆环断开 -> 亮孔和外面的纸面连成一片。
这两种情况都跟"圆在哪"有关，所以表现为"离中心远就丢"。

v4 的解法：**阈值扫描**（只在主阈值什么都找不到时才多跑几轮）
    第 1 轮：THRESH          （正常亮度，绝大多数情况这一轮就够）
    第 2 轮：THRESH - 25     （边缘变暗，纸面快要变墨水时用）
    第 3 轮：THRESH + 40     （边缘反光/墨水变淡，圆环断开时用）
  每轮都是"二值化 -> 找连通域 -> 过判据"，哪一轮先找到合格的圆就用哪一轮，
  所以正常情况下速度完全不变（找到就 break）。

另外把 AR 放宽到 0.55~1.80：圆在画面边上会被拍成椭圆，原限制太紧。
新增 3x3 亮度网格打印（SHOW_GRID）：直接告诉你画面边缘比中心暗多少。

===========================================================================
判据只有 5 条，都是"这是不是一个圆孔"本身：
    MIN_SIDE / MIN_AREA    太小 -> 不是目标
    FILL_MIN               实心度 = area/(pi*r^2)；孔是实心盘 ≈1.0，
                           环与环之间的缝最多到 0.75
    AR_MIN/MAX             长宽比（边上会被拍成椭圆，所以放宽）
    REJECT_TOUCH_EDGE      贴着画面边的一定是纸面/背景，不是圆

★ FILL_MIN 和 REJECT_TOUCH_EDGE 不要一起关：关掉就会出现"老老大的红圈"。

v2/v3 里加过、目前**仍然全部关闭**的：USE_ENC / USE_DEDUP / USE_SMOOTH /
USE_JUMP / USE_AUTO_THRESH / DRAW_REJECTED / PRINT_REJECTED
===========================================================================
'''

import time
import math

try:
    from media.sensor import *
    from media.display import *
    from media.media import *
    HAS_MEDIA = True
except Exception:
    HAS_MEDIA = False


# ============================ 一、图像 ============================
LCD_W = 800
LCD_H = 480
IMG_W = 640
IMG_H = 480

THRESH    = 70      # 主阈值：灰度低于它 -> 黑墨水 -> 二值图里的"白"
# 阈值扫描：相对 THRESH 的偏移。第一项必须是 0（主阈值优先）。
# 只有在上一轮一个合格圆都没找到时，才会试下一轮。
# 想要 v3 那种"只用一个阈值"的行为，改成 (0,) 就行。
THRESH_SWEEP = (0, -25, 40)

DILATE_PX = 1       # 圆环断口多就 1 -> 2
STRIDE    = 1       # find_blobs 扫描步长，1 = 最灵敏
DEBUG_BIN = False   # True = 只看二值图（调 THRESH 用，看第一轮）

# ======================== 二、什么算圆孔（5 条） ========================
MIN_SIDE = 22       # 外接矩形最小边
MIN_AREA = 180      # 最小像素数
FILL_MIN = 0.80     # 实心度下限（孔≈1.0；环间细缝 0.1~0.75）
AR_MIN   = 0.55     # 长宽比下限（画面边上会被拍扁，放宽）
AR_MAX   = 1.80     # 长宽比上限

REJECT_TOUCH_EDGE = True   # 贴画面边的一律丢（那是背景，不是圆）

# ============================ 三、选哪一个 ============================
PICK = "largest"    # "largest" 最大 / "smallest" 最小 / "center" 最靠画面中心

# ================== 四、诊断：3x3 亮度网格 ==================
# 打印画面 9 个格子的平均灰度。如果边缘那几格明显比中心小 40 以上，
# 就是"镜头渐晕/补光不均"，靠阈值扫描能救；差太多就要改灯光或加漫射板。
SHOW_GRID   = True
GRID_EVERY  = 30    # 每多少帧打一次
GRID_DRAW   = False # True = 把 9 个值也画在画面上

# ================== 五、v2/v3 的额外逻辑：仍然全关 ==================
USE_ENC          = False   # 检查"外面一圈是不是黑的"（要看背景，所以关）
ENC_MIN_HITS     = 4
USE_DEDUP        = False   # 同心去重：圆心重合时只留小的那个
USE_SMOOTH       = False   # 均值滤波
SMOOTH_N         = 3
USE_JUMP         = False   # 跳变门限
JUMP_MAX         = 70
USE_AUTO_THRESH  = False   # 自动阈值（和阈值扫描二选一，别同时开）
AUTO_K           = 0.62
AUTO_REFRESH     = 10
DRAW_REJECTED    = False   # 画没通过的候选（灰框 + 原因）
PRINT_REJECTED   = False   # 打印每个候选的实测值

EDGE_MARGIN = 1
VERSION = "单独找圆心 v4（加阈值扫描，治「离中心就丢」）"
# ====================================================================

_DIR8 = [(1.0, 0.0), (0.7071, 0.7071), (0.0, 1.0), (-0.7071, 0.7071),
         (-1.0, 0.0), (-0.7071, -0.7071), (0.0, -1.0), (0.7071, -0.7071)]


# ============================== 判据（纯逻辑） ==============================

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


def classify(b, W, H):
    """5 条判据。通过返回 (True,"OK")，否则 (False, 原因)。"""
    if b.w() < MIN_SIDE or b.h() < MIN_SIDE:
        return False, "SMALL"
    if b.pixels() < MIN_AREA:
        return False, "AREA"
    if blob_fill(b) < FILL_MIN:
        return False, "FILL"
    if blob_ar(b) < AR_MIN or blob_ar(b) > AR_MAX:
        return False, "AR"
    if REJECT_TOUCH_EDGE and touches_edge(b, W, H):
        return False, "EDGE"
    return True, "OK"


def pick_one(cands, W, H):
    if not cands:
        return None
    if PICK == "smallest":
        return min(cands, key=lambda b: b.pixels())
    if PICK == "center":
        cx, cy = W * 0.5, H * 0.5
        return min(cands, key=lambda b: (b.cx() - cx) ** 2 + (b.cy() - cy) ** 2)
    return max(cands, key=lambda b: b.pixels())


def dedup_concentric(cands):
    """同心去重（USE_DEDUP 打开时才用）。"""
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


def thresh_candidates():
    """按 THRESH_SWEEP 算出要试的阈值列表：去重、夹到 10~200，第一个是主阈值。"""
    out = []
    for off in THRESH_SWEEP:
        th = THRESH + off
        if th < 10:
            th = 10
        if th > 200:
            th = 200
        if th not in out:
            out.append(th)
    return out


def gray_mean(img):
    try:
        st = img.get_statistics()
        try:
            return float(st.l_mean())
        except Exception:
            return float(st.mean())
    except Exception:
        return -1.0


def cell_mean(img, x, y, w, h):
    """一格的平均灰度。优先用 get_statistics(roi=...)，不用分配新图像。"""
    try:
        st = img.get_statistics(roi=(x, y, w, h))
        try:
            return float(st.l_mean())
        except Exception:
            return float(st.mean())
    except Exception:
        pass
    try:
        return gray_mean(img.crop((x, y, w, h)))
    except Exception:
        return -1.0


def grid_means(img, n=3):
    """把画面切成 n*n 格，返回 [(x, y, 平均灰度), ...]。"""
    W, H = img.width(), img.height()
    out = []
    for j in range(n):
        for i in range(n):
            x = W * i // n
            y = H * j // n
            w = W // n
            h = H // n
            out.append((x, y, int(cell_mean(img, x, y, w, h))))
    return out


# ============================== 需要图像的部分 ==============================

def run_pass(gray_src, th, W, H, can_copy):
    """一轮检测：二值化 -> 膨胀 -> 找连通域 -> 过判据。返回 (blobs, passed, g)。"""
    g = gray_src
    if can_copy:
        try:
            g = gray_src.copy()      # 复制一份，别把源灰度图烧掉（后面几轮还要用）
        except Exception:
            g = gray_src
    g.binary([(0, th)])
    if DILATE_PX > 0:
        g.dilate(DILATE_PX)

    blobs = g.find_blobs([(128, 255)],
                         x_stride=STRIDE, y_stride=1,
                         pixels_threshold=MIN_AREA,
                         area_threshold=MIN_AREA,
                         invert=True, merge=False)

    passed = []
    for b in blobs:
        ok, reason = classify(b, W, H)
        if ok and USE_ENC:
            if enclosed_hits(g, b) < ENC_MIN_HITS:
                ok = False
        if ok:
            passed.append(b)
    return blobs, passed, g


def enclosed_hits(img_bin, b):
    """8 个方向里有几个"外面是黑的"（USE_ENC 打开时才用）。"""
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
        return
    except Exception:
        pass
    try:
        img.draw_string(x, y, text, color)
    except Exception:
        pass


# ============================== 主循环 ==============================

def main():
    sensor = Sensor(width=1280, height=960)
    sensor.reset()
    sensor.set_framesize(width=IMG_W, height=IMG_H)
    sensor.set_pixformat(Sensor.RGB565)

    Display.init(Display.ST7701, width=LCD_W, height=LCD_H, to_ide=True)
    MediaManager.init()
    sensor.run()

    clock = time.clock()
    frame = 0
    hist_x, hist_y, hist_r = [], [], []
    last = None
    th_list = thresh_candidates()

    print("=== %s ===" % VERSION)
    print("  5 条判据: MIN_SIDE=%d MIN_AREA=%d FILL>=%.2f AR=%.2f~%.2f 贴边丢=%s"
          % (MIN_SIDE, MIN_AREA, FILL_MIN, AR_MIN, AR_MAX, REJECT_TOUCH_EDGE))
    print("  阈值扫描 %s（只在上一轮什么都找不到时才试下一个）" % (th_list,))
    print("  额外逻辑: ENC=%s DEDUP=%s SMOOTH=%s JUMP=%s AUTO_TH=%s"
          % (USE_ENC, USE_DEDUP, USE_SMOOTH, USE_JUMP, USE_AUTO_THRESH))

    while True:
        clock.tick()
        frame += 1
        img = sensor.snapshot()

        # 灰度图单独留一份源：每一轮阈值都从它复制，保证互不影响
        try:
            gray_src = img.to_grayscale(copy=True)
        except TypeError:
            gray_src = img.to_grayscale()      # 老固件不支持 copy=，此时只能跑一轮
            can_copy = False
        else:
            can_copy = True

        W, H = gray_src.width(), gray_src.height()

        if frame == 1:
            # 老固件不支持 to_grayscale(copy=True)，那就只能跑主阈值一轮（会写在打印里）
            print("  灰度复制copy=%s（False 时阈值扫描自动退化成只跑一轮）" % can_copy)

        if frame % GRID_EVERY == 1 and SHOW_GRID:
            cells = grid_means(gray_src, 3)
            row = ""
            for j in range(3):
                row += "  "
                for i in range(3):
                    row += "%4d " % cells[j * 3 + i][2]
                if j < 2:
                    row += "/ "
            print("[grid] 3x3 平均灰度:" + row)
            if GRID_DRAW:
                for (gx, gy, gv) in cells:
                    draw_label(img, gx + 4, gy + 4, "%d" % gv, (255, 255, 0), 16)

        if DEBUG_BIN:
            g = gray_src
            if can_copy:
                try:
                    g = gray_src.copy()
                except Exception:
                    g = gray_src
            g.binary([(0, th_list[0])])
            if DILATE_PX > 0:
                g.dilate(DILATE_PX)
            Display.show_image(g,
                               x=round((LCD_W - sensor.width()) / 2),
                               y=round((LCD_H - sensor.height()) / 2))
            continue

        # ---- 阈值扫描：哪一轮先找到合格的圆就用哪一轮 ----
        passes = th_list if can_copy else th_list[:1]
        best = None
        used_th = passes[0]
        n_blobs = 0
        n_passed = 0

        for k, th in enumerate(passes):
            blobs, passed, _g = run_pass(gray_src, th, W, H, can_copy)
            if k == 0:
                n_blobs = len(blobs)          # 状态行显示主阈值那一轮的候选数
            if passed:
                if USE_DEDUP:
                    passed = dedup_concentric(passed)
                best = pick_one(passed, W, H)
                used_th = th
                n_blobs = len(blobs)
                n_passed = len(passed)
                break

        # ---- 画 ----
        if DRAW_REJECTED:
            blobs, _p, _g = run_pass(gray_src, used_th, W, H, can_copy)
            for b in blobs:
                ok, reason = classify(b, W, H)
                if best is not None and b is best:
                    continue
                try:
                    img.draw_rectangle(b.rect(), color=(120, 120, 120))
                except Exception:
                    pass
                draw_label(img, b.x(), b.y() - 18, reason, (255, 160, 0), 16)

        if best is not None:
            try:
                img.draw_rectangle(best.rect(), color=(255, 255, 0))
            except Exception:
                pass

            cx, cy, r = float(best.cx()), float(best.cy()), blob_r(best) + DILATE_PX

            if USE_SMOOTH:
                hist_x.append(cx)
                hist_y.append(cy)
                hist_r.append(r)
                while len(hist_x) > max(1, SMOOTH_N):
                    hist_x.pop(0)
                    hist_y.pop(0)
                    hist_r.pop(0)
                n = len(hist_x)
                sx, sy, sr = sum(hist_x) / n, sum(hist_y) / n, sum(hist_r) / n
            else:
                sx, sy, sr = cx, cy, r

            if USE_JUMP and last is not None:
                if math.hypot(sx - last[0], sy - last[1]) > JUMP_MAX:
                    sx, sy, sr = last
            last = (sx, sy, sr)

            xi, yi, ri = int(sx), int(sy), int(sr)
            try:
                img.draw_circle(xi, yi, ri, color=(255, 0, 0), thickness=2)
                img.draw_cross(xi, yi, color=(255, 0, 0), size=6, thickness=2)
            except Exception:
                pass
            draw_label(img, 0, 0, "c=(%d,%d) r=%d" % (xi, yi, ri), (0, 255, 0), 24)
            draw_label(img, 0, 28, "th=%d cand=%d ok=%d" % (used_th, n_blobs, n_passed),
                       (255, 255, 255), 20)
            print("[%d] FOUND c=(%d,%d) r=%d side=%dx%d area=%d fill=%.2f ar=%.2f "
                  "th=%d cand=%d fps=%.1f"
                  % (frame, best.cx(), best.cy(), int(blob_r(best)),
                     best.w(), best.h(), best.pixels(), blob_fill(best),
                     blob_ar(best), used_th, n_blobs, clock.fps()))
        else:
            last = None
            hist_x, hist_y, hist_r = [], [], []
            draw_label(img, 0, 0, "NO CIRCLE", (255, 80, 80), 24)
            draw_label(img, 0, 28, "th=%d cand=%d ok=0" % (used_th, n_blobs),
                       (255, 255, 255), 20)
            print("[%d] NONE 三个阈值 %s 都没找到（cand=%d）"
                  % (frame, passes, n_blobs))

        if PRINT_REJECTED and best is not None:
            blobs, _p, _g = run_pass(gray_src, used_th, W, H, can_copy)
            for b in blobs:
                ok, reason = classify(b, W, H)
                print("    %-5s c=(%d,%d) w=%d h=%d area=%d fill=%.2f ar=%.2f"
                      % (reason, b.cx(), b.cy(), b.w(), b.h(), b.pixels(),
                         blob_fill(b), blob_ar(b)))

        Display.show_image(img,
                           x=round((LCD_W - sensor.width()) / 2),
                           y=round((LCD_H - sensor.height()) / 2))


if __name__ == "__main__":
    if HAS_MEDIA:
        main()
    else:
        print("没找到 media.sensor：这份脚本要在 CanMV K230 上跑。")
