# -*- coding: utf-8 -*-
'''
圆心 ---> 直线 ---> 斜率 / 偏航角        (v4)
平台：01Studio CanMV K230（CanMV 固件）

================== v4：找圆部分同步 找圆心_宽松.py v4 ==================
v3 的找圆还是"一个固定阈值 + 只跑一轮"，所以圆一旦跑到画面边上就丢
（镜头渐晕，边缘比中心暗：纸面变墨水、或者圆环断开）。

v4 把单圆版调通的那三样搬过来：
  1) 阈值扫描 THRESH_SWEEP = (0,-25,40) -> 试 70 / 45 / 110
     ★ 和单圆版的区别：单圆版"找到就停"，这里要凑 2~3 个圆，
       所以按 (严格圆数, 宽松圆数) 打分选**最好的那一轮**；
       一旦严格圆够 2 个就提前停，正常情况还是第一轮就结束。
  2) AR 放宽到 0.55~1.80（圆在画面边上会被拍成椭圆）
  3) 3x3 亮度网格打印（SHOW_GRID），直接看出边缘比中心暗多少

斜率那部分（K_MAX 门限 / 补圆 / 剔歪点）与 v3 完全一致，没有改动。
======================================================================

====================== v3 相对 v2 改了什么 ======================
v2 的问题：只输出斜率，不对斜率做判断；而且只看见 1 个圆就直接放弃。
但现场有两个硬事实：
   ① 车不可能歪得很厉害  ->  斜率大于 K_MAX 的结果一定是**误检**，直接丢掉；
   ② 机械臂偏矮，看不全三个圆 ->  经常只看见 1 个（或 2 个）圆，需要"补"。

v3 把"车不会很歪"这个先验用在三个地方：
  1) 门限：拟合出的 |k| > K_MAX 就判 NG，绝不发一个错的偏航角给底盘；
  2) 补圆：只看见 1 个圆时，在它左右 ±K_MAX 的**斜率带**里找第二个圆
     （允许那个圆被画面边缘截断——这正是"看不全"的典型表现）；
  3) 剔点：三个点里有一个明显歪掉时，丢掉它，用剩下两个重新定线。

======================== 方向与符号（必须看懂）========================
EXPECT_AXIS 是一个开关，管两件事：
   "对正时，圆连线在画面里是哪个方向"  +  "斜率门限作用在哪个斜率上"
   "H" = 左右排列（斜率小）  <- 现在默认，符合"斜率应该不大"
   "V" = 前后排列（上下方向，用 dx/dy 那个斜率）
★ 如果你的三个环是前后排列（画面里竖着排），把 EXPECT_AXIS 改成 "V"，
  否则门限会用错斜率、psi 也会错 90°。

斜率有两个，门限只作用在"期望方向"那个：
   k = dy/dx    （EXPECT_AXIS="H" 时用它） 左右排列 -> dy≈0 -> k≈0
   k2 = dx/dy   （EXPECT_AXIS="V" 时用它） 上下排列 -> dx≈0 -> k2≈0

psi（偏航修正，给底盘用）：
   psi > 0 = 需要**顺时针转**（shun_yaw_ / dir=0x00）
   psi < 0 = 需要**逆时针转**（ni_yaw_   / dir=0x01）
   单位：屏幕上是度，括号里是 0.01° 整数（将来 Y<整数> 要发的值）
   推导：车顺时针转 θ -> 画面内容逆时针转 θ（图像角减小 θ）
         所以要消掉当前的图像角 α，就要顺时针转 α -> psi = α
   直线没有方向，所以 psi 折到 (-90°, +90°]，并且对"点顺序"和"相机装反 180°"都不敏感。

============================ 现场怎么确认符号 ============================
把车头往**右**推歪一点 -> psi 应变**正**。反了说明 EXPECT_AXIS 选错了（改 "V"）。
======================================================================
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


# ============================== 一、找圆参数 ==============================
LCD_W = 800
LCD_H = 480
IMG_W = 640
IMG_H = 480

THRESH       = 70
# 阈值扫描（和 找圆心_宽松.py v4 同一套）：相对 THRESH 的偏移，第一项必须是 0。
# 画面边缘比中心暗，固定阈值会让"跑到边上的圆"丢掉；多试几个阈值就能救回来。
# 只有上一轮"合格圆不够"时才会试下一轮；(0,) = 关掉扫描，退回单阈值。
THRESH_SWEEP = (0, -25, 40)
DILATE_PX    = 1        # 圆环断口多就 1 -> 2

# ---- 灵敏度（放宽，只为更容易看见）----
STRIDE    = 1
MIN_AREA  = 180
MIN_SIDE  = 22
AR_MIN    = 0.55        # 圆跑到画面边上会被拍成椭圆，原来 0.68 太紧
AR_MAX    = 1.80

# ---- 3x3 亮度网格诊断：边缘比中心暗多少，一眼看出来 ----
SHOW_GRID  = True
GRID_EVERY = 30
GRID_DRAW  = False

# ---- 结构性判据（别乱关，关了就会选中大块背景）----
FILL_MIN         = 0.72
ALLOW_TOUCH_EDGE = False
EDGE_MARGIN      = 1
ENC_MIN_HITS     = 4    # 8 方向里至少几个"外面是黑的"；不满足就**降级**为宽松候选

# ============================== 二、斜率门限 + 补圆 ==============================
# 对正时"圆连线"在画面里的方向。H = 左右排列（斜率小）；V = 前后排列。
EXPECT_AXIS = "H"

# |斜率| 上限。车不可能歪过这个角度，超了就是误检。
#   K_MAX = 0.35 约等于 19.3°；0.20 ≈ 11.3°；0.50 ≈ 26.6°。
#   这一条同时用于：丢弃歪结果 / 找第二个圆 / 剔除第三个歪点。
K_MAX = 0.35

# ---- 宽松候选池：只在"只看见 1 个圆"时用来找伴圆 ----
LOOSE_MIN_SIDE = 14      # 被画面截断的圆会更小
LOOSE_MIN_AREA = 60
LOOSE_FILL_MIN = 0.50    # 截断后实心度会掉
LOOSE_AR_MIN   = 0.45
LOOSE_AR_MAX   = 2.20

COMPANION_SIZE_MIN = 0.45   # 伴圆半径 / 主圆半径 下限（被截断会偏小）
COMPANION_SIZE_MAX = 1.70   # 上限（防止把大块背景/纸面当伴圆）
COMPANION_MAX_DIST = 460.0  # 伴圆离主圆最远多少像素
ANCHOR_MAX_SIDE    = 200    # 宽松池里挑主圆时，外接边超过它的多半是纸面/背景

# ============================== 三、拟合参数 ==============================
TARGETS_MAX   = 3
MIN_SPACING   = 25.0    # 两点至少相距多少像素
MIN_SPAN      = 40.0    # 三点里最远两点的最小距离
K_SPAN_MIN    = 8.0     # 斜率的分母跨度至少这么多像素才报（否则 INF）
FIT_WARN_PX   = 6.0     # 最大残差超过它提示"不共线"
SPAN_WEIGHT   = 0.002

# ============================== 四、显示 / 打印 / 串口 ==============================
DRAW_ALL    = True
PRINT_EVERY = 1
DEBUG_BIN   = False

# 打开后按 T1 的格式回：Y<psi*100>  或  NG
# ★ 现在 F103 的 Serial.c 只认 "数字,数字"，收到 Y 开头会整行丢掉。
UART_ENABLE   = False
UART_ID       = 2
UART_TX_PIN   = 11
UART_RX_PIN   = 12
UART_BAUDRATE = 115200
SEND_EVERY_MS = 100

VERSION = "圆心斜率 v4（找圆部分同步 v4：阈值扫描 + AR 放宽 + 亮度网格）"
# ======================================================================

_DIR8 = [(1.0, 0.0), (0.7071, 0.7071), (0.0, 1.0), (-0.7071, 0.7071),
         (-1.0, 0.0), (-0.7071, -0.7071), (0.0, -1.0), (0.7071, -0.7071)]

_REASONS = ("SMALL", "AREA", "AR", "FILL", "EDGE", "ENC", "NEST")

# 选点方式（打印出来方便判断当前走的是哪条路）
HOW_STRICT  = "strict"      # 2 个以上严格圆
HOW_PAIR    = "pair"        # 1 个严格圆 + 斜率带里找到伴圆
HOW_WEAK    = "weak-pair"   # 0 个严格圆，宽松池里凑出 2 个
HOW_SINGLE  = "single"      # 只有 1 个，定不了线
HOW_NONE    = "none"        # 什么都没找到


# ======================== 一、找圆判据（纯逻辑） ========================

def blob_ar(b):
    h = b.h()
    return b.w() / float(h) if h > 0 else 0.0


def blob_fill(b):
    r_est = (b.w() + b.h()) * 0.25
    return b.pixels() / (math.pi * r_est * r_est) if r_est > 0 else 0.0


def blob_r(b):
    return (b.w() + b.h()) * 0.25


def touches_edge(b, W, H, margin=None):
    m = EDGE_MARGIN if margin is None else margin
    return (b.x() <= m or b.y() <= m
            or b.x() + b.w() >= W - m or b.y() + b.h() >= H - m)


def geom_level(b, W, H):
    """给一个候选定级：

      2 = 严格合格（形态好、实心、不贴边）—— 可以当"主圆"
      1 = 宽松合格（可能被画面截断、实心度略低）—— 只能当"伴圆"
      0 = 不合格（太小/太细/太长）

    ENC（是否被黑环圈住）不在这里判，主循环里算完 enc 再决定要不要把 2 降成 1。
    """
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


def classify_geom(b, W, H):
    """兼容 v2 的接口：严格合格返回 (True,"OK")，否则给出原因。"""
    lv = geom_level(b, W, H)
    if lv == 2:
        return True, "OK"
    if b.w() < MIN_SIDE or b.h() < MIN_SIDE:
        return False, "SMALL"
    if b.pixels() < MIN_AREA:
        return False, "AREA"
    if blob_ar(b) < AR_MIN or blob_ar(b) > AR_MAX:
        return False, "AR"
    if blob_fill(b) < FILL_MIN:
        return False, "FILL"
    return False, "EDGE"


def classify(b, W, H, enc_hits=None):
    """严格判据 + 被环圈住。enc_hits = enclosed_hits() 的 0~8 结果。"""
    ok, reason = classify_geom(b, W, H)
    if not ok:
        return ok, reason
    if ENC_MIN_HITS > 0:
        if enc_hits is None or enc_hits < ENC_MIN_HITS:
            return False, "ENC"
    return True, "OK"


def dedup_concentric(cands):
    """同心去重：圆心重合时只留小的那个（最内圆）。"""
    out = []
    for b in sorted(cands, key=lambda x: x.w() * x.h()):
        rb = blob_r(b)
        keep = True
        for c in out:
            rc = blob_r(c)
            lim = rb if rb < rc else rc
            if lim < 1.0:
                continue
            dx = b.cx() - c.cx()
            dy = b.cy() - c.cy()
            if math.sqrt(dx * dx + dy * dy) < lim:
                keep = False
                break
        if keep:
            out.append(b)
    return out


# ============================== 二、斜率工具 ==============================

def slope_between(ax, ay, bx, by, axis=None):
    """两点连线在"期望方向"上的斜率。

    axis="H": k  = dy/dx （左右排列，k≈0 表示对正）
    axis="V": k2 = dx/dy （前后排列，k2≈0 表示对正）
    分母太小 -> 返回 None（= 斜率无穷大，方向完全跑偏）
    """
    axis = EXPECT_AXIS if axis is None else axis
    dx = bx - ax
    dy = by - ay
    if axis == "H":
        if abs(dx) < 1e-6:
            return None
        return dy / dx
    if abs(dy) < 1e-6:
        return None
    return dx / dy


def slope_ok(k):
    """斜率是否在门限内。None（= 无穷大）一律算不合格。"""
    return (k is not None) and (abs(k) <= K_MAX)


def expect_slope(m, axis=None):
    """从 line_metrics 的结果里取"期望方向那个斜率"。"""
    axis = EXPECT_AXIS if axis is None else axis
    return m["k_xy"] if axis == "H" else m["k_yx"]


# ============================== 三、选点（v3 核心） ==============================

def find_companion(anchor, pool, axis=None):
    """在"小斜率带"里给主圆找一个伴圆。

    条件（全部满足才收）：
      1) 尺寸和主圆相近（0.45~1.70 倍半径）—— 环都是差不多大的，
         太大的一定是纸面/背景，太小的多半是噪点；
      2) 离主圆足够远（不是它自己套着的那个外圈）；
      3) 到主圆的斜率 |k| <= K_MAX —— 这就是"车不会很歪"的直接用法；
      4) 在被画面边缘截断的情况下也认（这正是"看不全"时要救的对象）。

    返回候选里像素数最多的那个（最可靠），没有再返回 None。
    """
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

        dx = b.cx() - ax
        dy = b.cy() - ay
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > COMPANION_MAX_DIST:
            continue
        if dist < max(MIN_SPACING, 0.8 * (ra + rb)):
            continue                      # 挨太近 = 同一个目标的外圈

        k = slope_between(ax, ay, b.cx(), b.cy(), axis)
        if not slope_ok(k):
            continue

        if best is None or b.pixels() > best.pixels():
            best = b

    return best


def pick_weak_anchor(pool):
    """严格圆一个都没有时，从宽松池里挑一个"最像圆"的当主圆。

    额外要求：外接边不超过 ANCHOR_MAX_SIDE（否则大概率是纸面/背景）。
    """
    best = None
    for b in pool:
        if b.w() > ANCHOR_MAX_SIDE or b.h() > ANCHOR_MAX_SIDE:
            continue
        if best is None or b.pixels() > best.pixels():
            best = b
    return best


def select_line(cands):
    """决定用哪几个圆去定线。

    cands: [(blob, level, enc_hits), ...]，level 2=严格 1=宽松 0=不合格
    返回 (used_blobs, how)

    选点顺序：
      严格圆 >=2  -> 全用（最多 TARGETS_MAX 个，大的优先）        how=strict
      严格圆 ==1  -> 主圆 + 斜率带里的伴圆                        how=pair
      严格圆 ==0  -> 宽松池里挑主圆 + 伴圆                        how=weak-pair
      凑不出 2 个 -> 只有一个/没有                                how=single/none
    """
    strict = dedup_concentric([c[0] for c in cands if c[1] == 2])
    loose = dedup_concentric([c[0] for c in cands if c[1] >= 1])

    if len(strict) >= 2:
        strict.sort(key=lambda b: b.pixels(), reverse=True)
        return strict[:TARGETS_MAX], HOW_STRICT

    if len(strict) == 1:
        anchor = strict[0]
        comp = find_companion(anchor, loose)
        if comp is not None:
            return [anchor, comp], HOW_PAIR
        return [anchor], HOW_SINGLE

    anchor = pick_weak_anchor(loose)
    if anchor is not None:
        comp = find_companion(anchor, loose)
        if comp is not None:
            return [anchor, comp], HOW_WEAK
        return [anchor], HOW_SINGLE

    return [], HOW_NONE


def best_pair_within_gate(points, axis=None):
    """在点集里找一对"斜率在门限内、跨度最大"的点（用于剔掉歪掉的那个点）。

    找不到返回 None。
    """
    axis = EXPECT_AXIS if axis is None else axis
    n = len(points)
    best = None
    best_span = -1.0

    for i in range(n):
        for j in range(i + 1, n):
            ax, ay = points[i]
            bx, by = points[j]
            dx = bx - ax
            dy = by - ay
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < MIN_SPACING:
                continue
            if dist < 8.0:                     # 分母跨度太小，斜率不可信
                continue
            k = slope_between(ax, ay, bx, by, axis)
            if not slope_ok(k):
                continue
            if dist > best_span:
                best_span = dist
                best = [points[i], points[j]]

    return best


# ============================== 四、拟合（纯逻辑） ==============================

def fit_line(points):
    """总体最小二乘（TLS）拟合直线，返回 (px, py, cx, cy, resid)。"""
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


def max_pair_dist(points):
    d = 0.0
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            dx = points[i][0] - points[j][0]
            dy = points[i][1] - points[j][1]
            v = math.sqrt(dx * dx + dy * dy)
            if v > d:
                d = v
    return d


def min_pair_dist(points):
    d = None
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            dx = points[i][0] - points[j][0]
            dy = points[i][1] - points[j][1]
            v = math.sqrt(dx * dx + dy * dy)
            if d is None or v < d:
                d = v
    return 0.0 if d is None else d


def best_triple(points):
    """候选多于 3 个时，挑"最共线 + 间距够 + 跨度大"的那 3 个。"""
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
                score = fit[4] - SPAN_WEIGHT * span
                if best_score is None or score < best_score:
                    best_score = score
                    best = tri

    return best if best is not None else list(points[:TARGETS_MAX])


def ref_vector(axis):
    """参考轴单位向量（图像坐标）。"H"=(1,0) 画面右方；"V"=(0,-1) 画面上方。"""
    if axis == "V":
        return (0.0, -1.0)
    return (1.0, 0.0)


def line_metrics(points, axis=None):
    """把圆心列表算成一组可输出的数（字段含义见文件头）。"""
    axis = EXPECT_AXIS if axis is None else axis
    out = {"n": len(points), "used": [], "kind": "NONE",
           "k_xy": None, "k_yx": None, "k_expect": None, "auto": None,
           "theta": None, "yaw": None, "yaw_centi": None,
           "resid": None, "warn": False, "gate_ok": False, "axis": axis}

    if len(points) < 2:
        return out

    used = best_triple(points)
    out["used"] = used
    out["kind"] = "2P" if len(used) == 2 else "3P"

    fit = fit_line(used)
    if fit is None:
        out["kind"] = "DEG"
        return out

    px, py, cx, cy, resid = fit
    rx, ry = ref_vector(axis)

    # 统一方向：让 p 指向参考轴那一侧（消掉点顺序和相机装反 180° 的影响）
    if rx * px + ry * py < 0.0:
        px, py = -px, -py

    cross = rx * py - ry * px
    dot = rx * px + ry * py
    yaw = math.degrees(math.atan2(cross, dot))

    while yaw > 90.0:
        yaw -= 180.0
    while yaw <= -90.0:
        yaw += 180.0

    xs = [p[0] for p in used]
    ys = [p[1] for p in used]
    spanx = max(xs) - min(xs)
    spany = max(ys) - min(ys)

    if abs(px) > 1e-6 and spanx >= K_SPAN_MIN:
        out["k_xy"] = py / px
    if abs(py) > 1e-6 and spany >= K_SPAN_MIN:
        out["k_yx"] = px / py

    if out["k_xy"] is not None and out["k_yx"] is not None:
        out["auto"] = "k_xy" if spanx >= spany else "k_yx"
    elif out["k_xy"] is not None:
        out["auto"] = "k_xy"
    elif out["k_yx"] is not None:
        out["auto"] = "k_yx"

    out["theta"] = math.degrees(math.atan2(py, px))
    while out["theta"] > 90.0:
        out["theta"] -= 180.0
    while out["theta"] <= -90.0:
        out["theta"] += 180.0

    out["yaw"] = yaw
    out["yaw_centi"] = int(round(yaw * 100.0))
    out["resid"] = resid
    out["warn"] = (len(used) >= 3 and resid > FIT_WARN_PX)

    out["k_expect"] = out["k_xy"] if axis == "H" else out["k_yx"]
    out["gate_ok"] = slope_ok(out["k_expect"])
    return out


def resolve_metrics(points, axis=None):
    """line_metrics + 斜率门限失败时"丢掉最歪的点，用剩下两点重来"。

    返回 (metrics, how)：
      how = "kept"      原样可用
            "dropped"   门限不过 -> 换成了两点定线
            "rejected"  门限不过且换不出合规的两点（车真的太歪 / 误检）
    """
    axis = EXPECT_AXIS if axis is None else axis
    m = line_metrics(points, axis)
    if m["kind"] not in ("2P", "3P"):
        m["how"] = "rejected"
        return m, "rejected"

    if m["gate_ok"]:
        m["how"] = "kept"
        return m, "kept"

    pp = best_pair_within_gate(points, axis)
    if pp is not None:
        m2 = line_metrics(pp, axis)
        if m2["kind"] == "2P" and m2["gate_ok"]:
            m2["how"] = "dropped"
            return m2, "dropped"

    m["how"] = "rejected"
    return m, "rejected"


def fmt_report(m):
    """一行汇总。"""
    n = m["n"]
    if n == 0:
        return "N=0 没找到圆"
    if n == 1:
        return "N=1 只有一个圆，定不了直线"
    if m["kind"] == "NONE":
        return "N=%d 点不够" % n
    if m["kind"] == "DEG":
        return "N=%d 圆心重合，拟合退化" % n

    s = ""
    if not m["gate_ok"]:
        k = m.get("k_expect")
        s += "NG 斜率%s > %.2f " % (
            ("=INF" if k is None else "=%+.3f" % k), K_MAX)
    s += "N=%d(%s)" % (n, m["kind"])
    if m["k_xy"] is not None:
        s += " k=dy/dx%+.3f" % m["k_xy"]
    else:
        s += " k=dy/dx=INF"
    if m["k_yx"] is not None:
        s += " k=dx/dy%+.3f" % m["k_yx"]
    else:
        s += " k=dx/dy=INF"
    if m["yaw"] is not None:
        s += " psi%+.2fdeg(%d)" % (m["yaw"], m["yaw_centi"])
    if m["resid"] is not None:
        s += " fit=%.1fpx" % m["resid"]
    if m["warn"]:
        s += " [!]不共线"
    return s


def uart_frame(m):
    """要发给 F103 的 T1 回帧：合规才发 Y<0.01°>，否则 NG。"""
    if not m.get("gate_ok"):
        return "NG"
    if m["yaw_centi"] is None:
        return "NG"
    return "Y%d" % m["yaw_centi"]


def gray_mean(img_gray):
    try:
        st = img_gray.get_statistics()
        try:
            return float(st.l_mean())
        except Exception:
            return float(st.mean())
    except Exception:
        return -1.0


def thresh_candidates():
    """按 THRESH_SWEEP 算出要试的阈值：去重、夹到 10~200，第一个是主阈值。"""
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


def cell_mean(img, x, y, w, h):
    """一格的平均灰度。优先 get_statistics(roi=...)，不分配新图像。"""
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
            out.append((x, y, int(cell_mean(img, x, y, W // n, H // n))))
    return out


def level_counts(cands):
    """统计候选里严格/宽松各几个。"""
    ns = 0
    nl = 0
    for c in cands:
        if c[1] == 2:
            ns += 1
        elif c[1] == 1:
            nl += 1
    return ns, nl


def pass_score(cands):
    """一轮检测的得分：先看严格圆个数，再看宽松圆个数（越多越好）。"""
    ns, nl = level_counts(cands)
    return (ns, nl)


def clip_endpoints(cx, cy, px, py, W, H, margin=2):
    """把过质心、方向 p 的直线裁进画面，返回 (x1,y1,x2,y2)。"""
    t_hi = 1e9
    for (p0, d, hi, lo) in ((cx, px, W - 1 - margin, margin),
                            (cy, py, H - 1 - margin, margin)):
        if d > 1e-9:
            t_hi = min(t_hi, (hi - p0) / d)
        elif d < -1e-9:
            t_hi = min(t_hi, (lo - p0) / d)
    if t_hi > 1e8:
        t_hi = 0.0
    return (int(cx - px * t_hi), int(cy - py * t_hi),
            int(cx + px * t_hi), int(cy + py * t_hi))


# ============================== 五、需要图像的部分 ==============================

def enclosed_hits(img_bin, b):
    """候选外面一圈有几个方向是"白"（= 黑墨水 = 环）。返回 0~8。"""
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
    """一轮检测：二值化 -> 膨胀 -> 找连通域 -> 定级(严格/宽松)。

    返回 (blobs, cands)，cands = [(blob, level, enc_hits), ...]
      level 2 = 严格圆（可以当主圆）
            1 = 宽松圆（可能被画面截断，只能当伴圆）
            0 = 不合格
    """
    g = gray_src
    if can_copy:
        try:
            g = gray_src.copy()          # 别把源灰度图烧掉，后面几轮还要用
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
            lv = 1                       # 环有断口 -> 降级为宽松候选，不直接丢
        cands.append((b, lv, hits))
    return blobs, cands


def draw_label(img, x, y, text, color, size=18):
    W, H = img.width(), img.height()
    if x < 0:
        x = 0
    if y < 0:
        y = 0
    if y + size > H:
        y = H - size
    if x + size * len(text) > W:
        x = max(0, W - size * len(text))
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


def init_uart():
    if not UART_ENABLE:
        return None
    try:
        from machine import UART as MachineUART
        from machine import FPIOA
    except Exception as e:
        print("没有 machine 模块，串口跳过：", e)
        return None
    try:
        fpioa = FPIOA()
        fpioa.set_function(UART_TX_PIN, getattr(FPIOA, "UART%d_TXD" % UART_ID))
        fpioa.set_function(UART_RX_PIN, getattr(FPIOA, "UART%d_RXD" % UART_ID))
    except Exception as e:
        print("FPIOA 设置失败（继续试）：", e)

    uid = getattr(MachineUART, "UART%d" % UART_ID, UART_ID)
    for args in (dict(baudrate=UART_BAUDRATE, timeout=20, timeout_char=20),
                 dict(baudrate=UART_BAUDRATE),
                 None):
        try:
            u = MachineUART(uid, **args) if args else MachineUART(uid, UART_BAUDRATE)
            print("UART%d 就绪 %d" % (UART_ID, UART_BAUDRATE))
            return u
        except Exception:
            continue
    print("串口没起来")
    return None


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
    uart = init_uart()

    frame = 0
    last_send = 0
    th_list = thresh_candidates()

    print("=== %s ===" % VERSION)
    print("  EXPECT_AXIS=%s（%s）  K_MAX=%.2f（约 %.1f°）"
          % (EXPECT_AXIS,
             "左右排列，用 dy/dx" if EXPECT_AXIS != "V" else "前后排列，用 dx/dy",
             K_MAX, math.degrees(math.atan(K_MAX))))
    print("  只看见 1 个圆时：沿 ±K_MAX 斜率带找伴圆（允许被画面截断）")
    print("  阈值扫描 %s（合格圆不够时才试下一个）" % (th_list,))
    print("  符号确认：车头往右推歪 -> psi 应为正；反了就把 EXPECT_AXIS 改 V")

    while True:
        clock.tick()
        frame += 1
        img = sensor.snapshot()

        # 灰度源图单独留一份：每一轮阈值都从它复制，保证互不影响
        try:
            gray_src = img.to_grayscale(copy=True)
        except TypeError:
            gray_src = img.to_grayscale()      # 老固件不支持 copy=，只能跑一轮
            can_copy = False
        else:
            can_copy = True

        W, H = gray_src.width(), gray_src.height()

        if frame == 1:
            print("  灰度复制copy=%s（False 时阈值扫描退化为只跑主阈值）" % can_copy)

        if SHOW_GRID and frame % GRID_EVERY == 1:
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

        # ---- 1) 阈值扫描：挑"合格圆最多"的那一轮 ----
        # 和单圆版的差别：单圆版"找到就停"，这里要凑 2~3 个圆，
        # 所以按 (严格圆数, 宽松圆数) 打分取最好的那一轮；
        # 一旦"严格圆已经够 2 个"就提前停，正常情况仍然是第一轮就结束。
        passes = th_list if can_copy else th_list[:1]
        blobs = []
        cands = []
        used_th = passes[0]
        for th in passes:
            bs, cs = run_pass(gray_src, th, W, H, can_copy)
            if not cands or pass_score(cs) > pass_score(cands):
                blobs, cands, used_th = bs, cs, th
            if level_counts(cs)[0] >= 2:
                break

        # ---- 2) 选点：严格优先，不够就靠斜率带补 ----
        used, how = select_line(cands)
        centers = [(float(b.cx()), float(b.cy())) for b in used]

        # ---- 3) 定线 + 斜率门限（不过就剔歪点或 NG）----
        m, gate_how = resolve_metrics(centers)

        # ---- 4) 画 ----
        used_ids = set(id(b) for b in used)
        if DRAW_ALL:
            for (b, lv, hits) in cands:
                if id(b) in used_ids:
                    continue
                if lv == 0:
                    color = (110, 110, 110)
                else:
                    color = (200, 0, 200)     # 宽松候选但没被选上
                try:
                    img.draw_rectangle(b.rect(), color=color)
                except Exception:
                    pass
                draw_label(img, b.x(), b.y() - 18,
                           ("enc%d" % hits) if hits >= 0 else "rej",
                           (255, 160, 0), 16)

        for i, b in enumerate(used):
            is_anchor = (i == 0)
            color = (0, 255, 0) if is_anchor else (255, 160, 0)
            try:
                img.draw_rectangle(b.rect(), color=color)
                img.draw_cross(b.cx(), b.cy(), color=color, size=8, thickness=2)
            except Exception:
                pass
            draw_label(img, b.cx() + 8, b.cy() - 24,
                       "#%d%s" % (i, "" if is_anchor else "*"), color, 18)

        if len(m["used"]) >= 2:
            fit = fit_line(m["used"])
            if fit is not None:
                px, py, fcx, fcy, _ = fit
                x1, y1, x2, y2 = clip_endpoints(fcx, fcy, px, py, W, H)
                try:
                    img.draw_line(x1, y1, x2, y2,
                                  color=(0, 255, 255) if m["gate_ok"] else (255, 80, 80),
                                  thickness=2)
                except Exception:
                    pass
                rx, ry = ref_vector(EXPECT_AXIS)
                try:
                    img.draw_line(int(fcx), int(fcy),
                                  int(fcx + rx * 70), int(fcy + ry * 70),
                                  color=(255, 0, 255), thickness=2)
                except Exception:
                    pass

        # ---- 5) 屏幕文字 ----
        if m["kind"] in ("NONE", "DEG") or m["n"] < 2:
            head_color = (255, 80, 80)
        elif not m["gate_ok"]:
            head_color = (255, 80, 80)
        elif m["warn"]:
            head_color = (255, 200, 0)
        else:
            head_color = (0, 255, 0)

        draw_label(img, 0, 0, "N=%d %s" % (m["n"], how), head_color, 22)

        ke = m.get("k_expect")
        if m["kind"] in ("2P", "3P"):
            draw_label(img, 0, 26, "k=%s  (limit %.2f)" % (
                "INF" if ke is None else "%+.3f" % ke, K_MAX),
                (0, 255, 0) if m["gate_ok"] else (255, 80, 80), 20)
        else:
            draw_label(img, 0, 26, "k: 圆不够", (255, 80, 80), 20)

        if m["yaw"] is not None:
            draw_label(img, 0, 52, "psi%+.2f (%d)" % (m["yaw"], m["yaw_centi"]),
                       (0, 255, 255) if m["gate_ok"] else (255, 160, 0), 22)
        else:
            draw_label(img, 0, 52, "psi: -", (255, 80, 80), 20)

        tag = ("OK" if m["gate_ok"] else
               ("TOO TILT" if m["kind"] in ("2P", "3P") else "LOST"))
        draw_label(img, 0, 78, "gate:%s %s" % (tag, gate_how), head_color, 20)

        strict_n = len([c for c in cands if c[1] == 2])
        loose_n = len([c for c in cands if c[1] == 1])
        draw_label(img, 0, 104, "th=%d blobs=%d strict=%d loose=%d"
                   % (used_th, len(blobs), strict_n, loose_n), (180, 180, 180), 18)

        # ---- 6) IDE 打印 ----
        if PRINT_EVERY > 0 and frame % PRINT_EVERY == 0:
            print("[%d] th=%d blobs=%d strict=%d loose=%d how=%s gate=%s %s"
                  % (frame, used_th, len(blobs), strict_n, loose_n, how, gate_how,
                     fmt_report(m)))
            for i, b in enumerate(used):
                print("    used#%d c=(%d,%d) w=%d h=%d area=%d fill=%.2f ar=%.2f"
                      % (i, b.cx(), b.cy(), b.w(), b.h(), b.pixels(),
                         blob_fill(b), blob_ar(b)))

        # ---- 7) 可选：发给 F103 ----
        if uart is not None and time.ticks_diff(time.ticks_ms(), last_send) >= SEND_EVERY_MS:
            last_send = time.ticks_ms()
            try:
                uart.write(uart_frame(m) + "\r\n")
            except Exception:
                pass

        Display.show_image(img,
                           x=round((LCD_W - sensor.width()) / 2),
                           y=round((LCD_H - sensor.height()) / 2))


if __name__ == "__main__":
    if HAS_MEDIA:
        main()
    else:
        print("没找到 media.sensor：这份脚本要在 CanMV K230 上跑。")
