# -*- coding: utf-8 -*-
# =====================================================================
#  LCD 自检 —— 只测显示，完全不碰相机
#
#  为什么要有这个脚本：
#     相机不出图时，主脚本永远走不到 show_image()，屏全黑 —— 看起来和"屏坏"
#     一模一样。这个脚本把相机整个摘掉，只喂纯色图，把"屏"和"相机"分开。
#
#  ★ 本文件同时回答一个关键问题：
#     这块固件的 VO/OSD 图层到底允许什么尺寸、什么偏移？
#     （主脚本报过 "set vo layer attr failed 1"，就是图像尺寸/偏移不被接受）
#
#  在 CanMV IDE 里单独运行。按顺序看：
#     第 1 段：红/绿/蓝/白格子（800x480 = 画布大小）—— 屏和显示通路正不正常
#     第 2 段：尺寸/偏移矩阵 —— 哪些 (w,h,x,y) 能上屏，打印 OK / FAIL
#     第 3 段：合成画布 —— 800x450 贴到 800x480 画布上再整屏送（像素级 16:9 方案）
# =====================================================================

import time
import os

from media.display import *
from media.media import *

try:
    import image
except Exception as e:
    image = None
    print("没有 image 模块：", e)

LCD_W = 800
LCD_H = 480
LCD_DRIVER = "ST7701"      # 3.1 寸屏；HDMI 扩展板改成 "LT9611"

WHITE = (255, 255, 255)


def label(img, x, y, text, color=WHITE, size=28):
    try:
        img.draw_string_advanced(x, y, size, text, color=color)
        return
    except Exception:
        pass
    try:
        img.draw_string(x, y, text, color=color, scale=2)
    except Exception:
        pass


def make(w, h, color, text=""):
    buf = image.Image(w, h, image.RGB565)
    buf.clear()
    buf.draw_rectangle(0, 0, w - 1, h - 1, color=color, thickness=6)
    buf.draw_cross(w // 2, h // 2, color=color, size=40, thickness=3)
    if text:
        label(buf, 8, 8, text, color, 24)
        label(buf, 8, h - 36, "%dx%d" % (w, h), color, 22)
    return buf


def show_solid(w, h, color, text, x=0, y=0, expect=True):
    """送一张 w x h 的图到 (x,y)，打印结果。expect 只影响打印措辞。"""
    try:
        buf = make(w, h, color, text)
    except Exception as e:
        print("   %4dx%-4d @(%d,%d)  造图失败：%s" % (w, h, x, y, e))
        return False
    try:
        if x == 0 and y == 0:
            Display.show_image(buf)
        else:
            Display.show_image(buf, x=x, y=y)
        print("   %4dx%-4d @(%d,%d)  OK" % (w, h, x, y))
        return True
    except Exception as e:
        print("   %4dx%-4d @(%d,%d)  FAIL  %s" % (w, h, x, y, e))
        return False


def phase1():
    print("[1] 纯色测试（800x480 = 画布大小，这是最稳的一条路）")
    for color, text in (((255, 0, 0), "RED"),
                        ((0, 255, 0), "GREEN"),
                        ((0, 0, 255), "BLUE")):
        print("    送 %s ..." % text)
        show_solid(LCD_W, LCD_H, color, text)
        time.sleep_ms(1200)

    print("    送白格子 ...")
    try:
        buf = image.Image(LCD_W, LCD_H, image.RGB565)
        buf.clear()
        for x in range(0, LCD_W, 100):
            buf.draw_line(x, 0, x, LCD_H - 1, color=(60, 60, 60))
        for y in range(0, LCD_H, 100):
            buf.draw_line(0, y, LCD_W - 1, y, color=(60, 60, 60))
        buf.draw_rectangle(0, 0, LCD_W - 1, LCD_H - 1, color=(255, 255, 0), thickness=3)
        buf.draw_cross(LCD_W // 2, LCD_H // 2, color=(255, 0, 0), size=60, thickness=4)
        label(buf, 20, 20, "GRID + CENTER CROSS", WHITE, 30)
        label(buf, 20, 60, "cross should be centered", WHITE, 20)
        Display.show_image(buf)
        print("    OK")
    except Exception as e:
        print("    FAIL", e)
    time.sleep_ms(1500)


def phase2():
    print("[2] 尺寸/偏移矩阵（看哪种组合被 VO 图层接受）")
    cases = [
        (LCD_W, LCD_H, 0, 0),      # 基准：图像 == 画布
        (800, 450, 0, 0),          # 纯 16:9，不偏移
        (800, 450, 0, 15),         # 纯 16:9，居中（主脚本报错的那个）
        (800, 448, 0, 16),         # 高度对齐到 16
        (640, 480, 80, 0),         # 老配置（4:3 居中）
        (640, 360, 80, 60),        # 16:9 居中
    ]
    colors = (200, 0, 200)
    for (w, h, x, y) in cases:
        show_solid(w, h, colors, "%dx%d@%d,%d" % (w, h, x, y), x, y)
        time.sleep_ms(800)


def phase3():
    print("[3] 合成画布测试（像素级 16:9：800x450 贴到 800x480 画布再整屏送）")
    try:
        canvas = image.Image(LCD_W, LCD_H, image.RGB565)
        frame = image.Image(800, 450, image.RGB565)
    except Exception as e:
        print("   FAIL 造图：%s" % e)
        return

    try:
        frame.clear()
        frame.draw_rectangle(0, 0, 799, 449, color=(0, 200, 200), thickness=6)
        frame.draw_cross(400, 225, color=(0, 200, 200), size=40, thickness=3)
        label(frame, 8, 8, "16:9 frame 800x450", (0, 200, 200), 24)

        canvas.clear()
        canvas.draw_image(frame, 0, 15)        # ← 关键：小图贴到大画布
        canvas.draw_rectangle(0, 0, LCD_W - 1, LCD_H - 1,
                              color=(255, 255, 0), thickness=2)
        label(canvas, 8, 430, "composited on 800x480 canvas", (255, 255, 0), 20)

        Display.show_image(canvas)
        print("   OK —— draw_image 可用，像素级 16:9 这条路走得通")
    except Exception as e:
        print("   FAIL  %s" % e)
        print("   -> 合成画布这条路不通，只能让 IMG_W/IMG_H == LCD_W/LCD_H")


def main():
    print("Display.init(%s, %dx%d) ..." % (LCD_DRIVER, LCD_W, LCD_H))
    try:
        Display.init(getattr(Display, LCD_DRIVER, Display.ST7701),
                     width=LCD_W, height=LCD_H, to_ide=True)
    except Exception as e:
        print("!! Display.init 失败：", e)
        print("   没接屏就改成 Display.VIRT（只送 IDE）再试")
        return
    print("   OK")

    print("MediaManager.init() ...")
    try:
        MediaManager.init()
    except Exception as e:
        print("!! 失败：", e)
        print("   -> 多半是上一次运行没 deinit。拔电重上电，再跑本脚本。")
        return
    print("   OK")

    if image is None:
        print("!! 没有 image 模块")
        return

    try:
        phase1()
        phase2()
        phase3()

        print("")
        print("[4] 总结：")
        print("    · 第 1 段看到 红/绿/蓝/白格子 -> 屏和显示通路没问题")
        print("    · 第 2 段哪些行 OK，就是这块固件允许多大的图、能不能带偏移")
        print("    · 第 3 段 OK -> 想要像素级 16:9 可以做合成画布（每帧多一次 blit）")
        print("    · 本脚本停在这里不动（按 IDE 停止即可）")

        while True:
            os.exitpoint()
            time.sleep_ms(500)

    except KeyboardInterrupt:
        print("用户停止")
    finally:
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
        print("已 deinit")


main()
