# -*- coding: utf-8 -*-
"""circle_track.py 的电脑端逻辑自检 —— 不需要相机、不需要板子。

它干三件事：
  1. 用假 time / 假 image 对象把 circle_track.py 跑起来；
  2. 把程序发出去的字节，逐字节喂给【照抄 Hardware/Serial.c】的解析器，
     证明格式不会被 F103 打回；
  3. 验证稳定判据（连续多帧才发、跳变拒绝、丢失后静默）。

跑法： python test_logic.py
"""

import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


# ---------------------------------------------------------------- 假 time
NOW = [0]


class FakeClock(object):
    def __init__(self):
        self.n = 0

    def tick(self):
        self.n += 1

    def fps(self):
        return 12.5


fake_time = types.ModuleType("time")
fake_time.ticks_ms = lambda: NOW[0]
fake_time.ticks_diff = lambda a, b: a - b
fake_time.ticks_us = lambda: NOW[0] * 1000
fake_time.sleep_ms = lambda ms: None
fake_time.clock = lambda: FakeClock()
sys.modules["time"] = fake_time

import circle_track as ct                                    # noqa: E402


# ------------------------------------------------- F103 侧解析器（照抄 C）
class F103Rx(object):
    """逐字节复刻 Hardware/Serial.c 的 K230_FeedByte + K230_LineCommit。"""

    def __init__(self):
        self.line = []
        self.frames = []
        self.bad_lines = 0

    def feed(self, data):
        for ch in data:
            self._byte(ch)

    def _byte(self, ch):
        if ch == "\r" or ch == "\n":
            if self.line:
                self._commit()
            self.line = []
        elif ("0" <= ch <= "9") or ch == "," or ch == " ":
            if len(self.line) < 16 - 1:
                self.line.append(ch)
            else:
                self.line = []
        else:
            self.line = []

    def _commit(self):
        s = "".join(self.line)
        i = 0
        n = len(s)

        while i < n and s[i] == " ":
            i += 1
        x = 0
        nd = 0
        while i < n and "0" <= s[i] <= "9":
            x = x * 10 + int(s[i])
            nd += 1
            if nd > 4:
                self.bad_lines += 1
                return
            i += 1
        if nd == 0:
            self.bad_lines += 1
            return

        while i < n and s[i] == " ":
            i += 1
        if i >= n or s[i] != ",":
            self.bad_lines += 1
            return
        i += 1

        while i < n and s[i] == " ":
            i += 1
        y = 0
        nd = 0
        while i < n and "0" <= s[i] <= "9":
            y = y * 10 + int(s[i])
            nd += 1
            if nd > 4:
                self.bad_lines += 1
                return
            i += 1
        if nd == 0:
            self.bad_lines += 1
            return

        while i < n and s[i] == " ":
            i += 1
        if i != n:
            self.bad_lines += 1
            return
        if x > 999 or y > 999:
            self.bad_lines += 1
            return
        self.frames.append((x, y))


# ---------------------------------------------------------------- 假串口
class FakeUart(object):
    def __init__(self):
        self.data = []

    def write(self, msg):
        self.data.append(msg)
        return len(msg)


# ------------------------------------------------------------ 假图像对象
class FakeCircle(object):
    def __init__(self, x, y, r, mag):
        self._x = x
        self._y = y
        self._r = r
        self._m = mag

    def x(self):
        return self._x

    def y(self):
        return self._y

    def r(self):
        return self._r

    def magnitude(self):
        return self._m


class FakeBlob(object):
    def __init__(self, x, y, w, h, pixels):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._p = pixels

    def cx(self):
        return self._x + self._w // 2

    def cy(self):
        return self._y + self._h // 2

    def w(self):
        return self._w

    def h(self):
        return self._h

    def pixels(self):
        return self._p


class FakeImg(object):
    def __init__(self, circles=None, blobs=None, no_circles_api=False,
                 w=640, h=480):
        self._circles = circles or []
        self._blobs = blobs or []
        self._no_api = no_circles_api
        self._w = w
        self._h = h

    def width(self):
        return self._w

    def height(self):
        return self._h

    def find_circles(self, **kw):
        if self._no_api:
            raise AttributeError("该固件没有 find_circles")
        return self._circles

    def find_blobs(self, thresholds, **kw):
        return self._blobs


# ---------------------------------------------------------------- 断言
FAILS = []


def check(name, got, want):
    if got == want:
        print("  [ok]   %-42s %r" % (name, got))
    else:
        print("  [FAIL] %-42s %r  (期望 %r)" % (name, got, want))
        FAILS.append(name)


def check_true(name, cond, detail=""):
    if cond:
        print("  [ok]   %-42s %s" % (name, detail))
    else:
        print("  [FAIL] %-42s %s" % (name, detail))
        FAILS.append(name)


# ================================================================ 用例
print("\n== 1. 串口格式（逐字节过一遍 F103 的解析器） ==")
u = FakeUart()
rx = F103Rx()
ct.send_xy(u, 318, 242)
print("     发出: %r" % u.data[0])
check("结尾是 CRLF", u.data[0][-2:], "\r\n")
rx.feed(u.data[0])
check("F103 解出坐标", rx.frames, [(318, 242)])
check("没有坏行", rx.bad_lines, 0)

u2 = FakeUart()
rx2 = F103Rx()
ct.send_xy(u2, 1000, -5)                      # 越界，应该被夹到 639 / 0
print("     越界入参 -> 发出: %r" % u2.data[0])
rx2.feed(u2.data[0])
check("越界被夹到画面内", rx2.frames, [(639, 0)])

u3 = FakeUart()
rx3 = F103Rx()
for point in [(0, 0), (320, 240), (639, 479), (999, 999)]:
    ct.send_xy(u3, point[0], point[1])
    rx3.feed(u3.data[-1])
check("边界值全部通过 F103 校验", rx3.frames,
      [(0, 0), (320, 240), (639, 479), (639, 479)])
check("边界值没有坏行", rx3.bad_lines, 0)

check("uart 为 None 时静默不炸", ct.send_xy(None, 1, 1), None)

print("\n== 2. 稳定判据 ==")
t = ct.CircleTracker()
now_ok = []
for i in range(6):
    NOW[0] = i * 50
    now_ok.append(t.update(True, 300, 220))
check("连续 6 帧后才 LOCK", [r[0] for r in now_ok],
      [False, False, False, False, True, True])
check("锁定坐标是平滑后的值", (now_ok[-1][1], now_ok[-1][2]), (300, 220))

NOW[0] = 500
ok, _, _ = t.update(False, 0, 0)              # 单帧丢失
check("丢 1 帧不退出 LOCK", ok, True)
for i in range(5):
    NOW[0] = 600 + i * 50
    ok, _, _ = t.update(False, 0, 0)
check("连续丢帧后退出 LOCK（停止发送）", ok, False)

tj = ct.CircleTracker()
for i in range(8):                            # 左右横跳的假目标
    NOW[0] = 1000 + i * 50
    x = 100 if i % 2 == 0 else 500
    ok, _, _ = tj.update(True, x, 220)
check("坐标乱跳不 LOCK", ok, False)

ts = ct.CircleTracker()
for i in range(6):
    NOW[0] = 2000 + i * 50
    ts.update(True, 300 + i, 220)
ok, sx, _ = ts.update(True, 306, 220)
check_true("锁定后平滑值跟得上慢速移动", 300 <= sx <= 306, "sx=%d" % sx)

print("\n== 3. 识别优先级：先找圆，找不到再退回蓝框 ==")
img_both = FakeImg(circles=[FakeCircle(200, 150, 40, 3000),
                            FakeCircle(500, 400, 80, 3000)],
                   blobs=[FakeBlob(10, 10, 60, 60, 900)])
found, x, y, src = ct.find_target(img_both, (0, 0, 640, 480))
check("有圆就用圆", (found, src), (True, "circle"))
check("取分数更高的那个圆（半径大）", (x, y), (500, 400))

img_r_lo = FakeImg(circles=[FakeCircle(200, 150, 3, 3000),
                            FakeCircle(500, 400, 400, 3000)])
found, x, y, src = ct.find_target(img_r_lo, (0, 0, 640, 480))
check("半径越界的圆被丢掉", (found, src), (False, "none"))

img_blob = FakeImg(circles=[], blobs=[FakeBlob(100, 100, 40, 40, 500),
                                      FakeBlob(300, 200, 50, 50, 2000)])
found, x, y, src = ct.find_target(img_blob, (0, 0, 640, 480))
check("没有圆时退回蓝框", (found, src), (True, "blob"))
check("蓝框取像素最多的", (x, y), (325, 225))

img_noapi = FakeImg(no_circles_api=True,
                    blobs=[FakeBlob(300, 200, 50, 50, 2000)])
found, x, y, src = ct.find_target(img_noapi, (0, 0, 640, 480))
check("固件没有 find_circles 也不崩", (found, src), (True, "blob"))

img_tiny = FakeImg(blobs=[FakeBlob(10, 10, 4, 4, 16)])
found, x, y, src = ct.find_target(img_tiny, (0, 0, 640, 480))
check("太小的蓝块不算目标", (found, src), (False, "none"))

print("\n== 4. ROI 裁剪 ==")
img = FakeImg()
check("默认 ROI = 全画面", ct.detection_roi(img), (0, 0, 640, 480))
ct.ROI_TOP_PERCENT = 25
check("顶部裁 25% 后高度正确", ct.detection_roi(img), (0, 120, 640, 360))
ct.ROI_TOP_PERCENT = 0
ct.ROI_LEFT_PERCENT = 50
check("左侧裁 50% 后宽度正确", ct.detection_roi(img), (320, 0, 320, 480))
ct.ROI_LEFT_PERCENT = 0
check("裁过头时 ROI 不会出现负数", ct.roi_clip(img, 600, 400, 400, 400),
      (600, 400, 40, 80))

print("\n" + "=" * 60)
if FAILS:
    print("失败 %d 项：%s" % (len(FAILS), FAILS))
    sys.exit(1)
print("全部通过")
