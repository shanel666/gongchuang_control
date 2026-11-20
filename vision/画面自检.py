# -*- coding: utf-8 -*-
# =====================================================================
#  K230「画面自检」—— 一眼看出：视野丢没丢、光轴偏没偏、显示有没有缩放
#
#  用法：在 CanMV IDE 里【单独运行本文件】（不要被 main.py 调用）。
#        它不碰串口、不碰 F103，只出画面，随便跑。
#
#  为什么它能当尺子：
#        它用的是和立创官方例程完全一样的那条通路 ——
#            Sensor(id=2) + 1920x1080 + 通道0 + Display.VIRT
#            画布尺寸 == 图像尺寸（1920x1080）
#        也就是说：显示 1:1、没有摆放偏移、没有裁剪、没有拉伸，
#        你在这张图上看到的就是镜头真实的全部视野。
#        拿它去和你工程里那个脚本比，差在哪一目了然。
#
#  画面上的东西：
#    ● 绿框        整幅图像的边界（画布=图像，四周不该有黑边）
#    ● 黄框        改成 4:3（640x480 / 1280x960）后【还能看到】的范围；
#                  黄框左右两条竖线以外 = 被裁掉的那部分视野
#    ● 红十字      图像几何中心 (960,540)；一条竖线贯穿全高
#    ● 红圆        正圆参考物（半径 200 px）：对准真实圆环，看它是圆还是扁
#    ● 灰网格      每 240 px 一格
#    ● 白字        实际输出分辨率 / 帧率（should be 1920x1080）
#
#  怎么读结果：
#    1) 黄框外还有你要看的东西
#       -> 你现在 4:3 的配置就是把它裁掉了，这就是"拍不全"（左右各丢 1/8）
#    2) 镜头正对一个左右对称的目标（桌沿正中、门框、两盏灯中间）
#       红十字没压在对称轴上
#       -> 光轴不在几何中心，串口坐标要整体加/减一个固定偏移
#          （也就是"对齐时要带 xy 偏移"的那部分，属于相机安装/裁剪问题）
#    3) 画面里的真实圆环看起来扁了/长了
#       -> 那不是裁剪，是拉伸（宽高比被硬拉），同样要改回 16:9
#
#  想看 4:3 到底丢了多少：把下面 CAP_W / CAP_H 改成 1440 / 1080 再跑一次，
#  同一位置对比两张图，框外的东西是不是"变没了"。
# =====================================================================

import time
import os

from media.sensor import *
from media.display import *
from media.media import *

# ------------------------- 可改 -------------------------
SENSOR_ID = 2              # 庐山派板载 CSI2 = id 2（默认就是 2，写出来防呆）
CAP_W = 1920               # 想对比 4:3：改成 1440
CAP_H = 1080               #              高度保持 1080
PIXFMT = Sensor.RGB565     # 想和例程完全一致可以用 Sensor.RGB888
SHOW_CROP43 = True         # 画 4:3 裁剪框（黄）
SHOW_GRID = True           # 画 240 px 网格（灰）
# --------------------------------------------------------

GREEN = (0, 255, 0)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
GRAY = (80, 80, 80)
WHITE = (255, 255, 255)


def label(img, x, y, text, color=WHITE, scale=2):
    """统一写字：优先 draw_string_advanced（K230 上更好看），退回 draw_string。"""
    if x < 0:
        x = 0
    if y < 0:
        y = 0
    if x > img.width() - 8:
        x = img.width() - 8
    if y > img.height() - 8:
        y = img.height() - 8
    try:
        img.draw_string_advanced(x, y, 16 * scale, text, color=color)
        return
    except Exception:
        pass
    try:
        img.draw_string(x, y, text, color=color, scale=scale)
    except Exception:
        pass


def safe(fn, *args, **kwargs):
    """画图接口各家固件签名略有差别，画不出来不要影响主循环。"""
    try:
        return fn(*args, **kwargs)
    except Exception:
        return None


def draw_overlay(img, fps):
    W = img.width()
    H = img.height()

    # ---- 整幅图像的四条边（1:1 时应该正好贴着画面边缘）----
    safe(img.draw_rectangle, 1, 1, W - 2, H - 2, color=GREEN, thickness=3)

    # ---- 灰网格 ----
    if SHOW_GRID:
        step = 240
        x = step
        while x < W:
            safe(img.draw_line, x, 0, x, H - 1, color=GRAY, thickness=1)
            x += step
        y = step
        while y < H:
            safe(img.draw_line, 0, y, W - 1, y, color=GRAY, thickness=1)
            y += step

    # ---- 4:3 裁剪框：这是 640x480 / 1280x960 还能看到的范围 ----
    if SHOW_CROP43:
        box_h = H
        box_w = (H * 4) // 3
        if box_w > W:
            box_w = W
        box_x = (W - box_w) // 2
        box_y = 0
        safe(img.draw_rectangle, box_x, box_y, box_w - 1, box_h - 1,
             color=YELLOW, thickness=3)
        label(img, box_x + 8, 12, "4:3 CROP = what VGA/1280x960 keeps", YELLOW, 2)
        label(img, box_x + 8, 44, "outside this box is CUT OFF", YELLOW, 2)
        label(img, 12, 12, "FULL 16:9 VIEW", GREEN, 2)

    # ---- 图像几何中心：红十字 + 贯穿竖线 ----
    cx = W // 2
    cy = H // 2
    safe(img.draw_line, cx, 0, cx, H - 1, color=RED, thickness=1)
    safe(img.draw_line, 0, cy, W - 1, cy, color=RED, thickness=1)
    safe(img.draw_cross, cx, cy, color=RED, size=60, thickness=4)
    label(img, cx + 12, cy + 12, "IMG CENTER %d,%d" % (cx, cy), RED, 2)

    # ---- 正圆参考物：真实圆环应该和它一样圆 ----
    r = H // 5
    safe(img.draw_circle, cx, cy, r, color=RED, thickness=2)
    label(img, cx - r, cy - r - 30, "circle ref r=%d" % r, RED, 2)

    # ---- 四角坐标标注，方便估位置 ----
    label(img, 12, H - 40, "0,0", GREEN, 2)
    label(img, W - 200, H - 40, "%d,%d" % (W, H), GREEN, 2)

    # ---- 状态行 ----
    label(img, 12, cy - 40, "OUT %dx%d  FPS=%.1f" % (W, H, fps), WHITE, 2)


def main():
    sensor = None
    try:
        sensor = Sensor(id=SENSOR_ID)
        sensor.reset()
        # 通道 0：整幅 16:9，不裁不拉
        sensor.set_framesize(chn=CAM_CHN_ID_0, width=CAP_W, height=CAP_H)
        sensor.set_pixformat(PIXFMT, chn=CAM_CHN_ID_0)
        print("sensor 实际输出 = %dx%d" % (sensor.width(), sensor.height()))

        # ★ 画布尺寸 == 图像尺寸：这样 show_image 不需要 x/y，
        #   显示永远 1:1，不会出现"图偏到一边"的情况。
        Display.init(Display.VIRT, width=CAP_W, height=CAP_H, to_ide=True)
        MediaManager.init()
        sensor.run()

        clock = time.clock()
        last_print = 0
        print("画面自检开始：绿框=全幅 黄框=4:3 还能看到的 红十字=几何中心")
        print("把镜头正对一个左右对称的目标，红十字应该压在对称轴上。")

        while True:
            os.exitpoint()
            clock.tick()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            fps = clock.fps()
            draw_overlay(img, fps)
            Display.show_image(img)

            now = time.ticks_ms()
            if time.ticks_diff(now, last_print) >= 1000:
                last_print = now
                print("[diag] out=%dx%d sensor=%dx%d fps=%.1f" %
                      (img.width(), img.height(),
                       sensor.width(), sensor.height(), fps))

    except KeyboardInterrupt as e:
        print("用户停止:", e)
    except BaseException as e:
        print("异常:", e)
    finally:
        try:
            if isinstance(sensor, Sensor):
                sensor.stop()
        except Exception:
            pass
        try:
            Display.deinit()
        except Exception:
            pass
        try:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        except Exception:
            pass
        time.sleep_ms(200)
        try:
            MediaManager.deinit()
        except Exception:
            pass
        print("自检结束")


main()
