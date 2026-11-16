# -*- coding: utf-8 -*-
'''
K230 串口「只发」测试（板载 GH1.25 串口2，GPIO11=TX / GPIO12=RX）
平台：立创·庐山派 K230 CanMV / Lite K230D

只做一件事：每隔一段时间往串口发一行，不管收。用来单独确认 **发送方向** 没问题。

为什么单独做一个：之前那份 串口收发测试.py 是"发+收"混在一起，
一旦什么都收不到，你分不清是"没发出去"还是"收不到"。这份只看发。

============================= 怎么用 =============================
接法一：电脑串口助手（最快的验证）
    板子 GH1.25 串口2        USB-TTL / CH340
        T (GPIO11)   --->    RXD
        G (GND)      ---    GND
    串口助手设 115200 8N1，然后运行本脚本：
    每 500 ms 应该看到一行 "K230 #00001"、"K230 #00002"…

接法二：验证上位机 / 主控的解析
    把 MODE 改成 "frames"，发的是真正协议帧：
        C318,242,41 → C320,240,40 → L4,192 → L-12,-345 → NG → 循环
    这时跑 tools/k230_host.py 应该能把每一行都正确解析出来。

接法三：接 F103
    K230 T(GPIO11) -> F103 PA3 (USART2_RX)
    GND 共地
    MODE="frames" 时，F103 的 vision 模式按 PB1 进 circle 状态后能看到 X/Y/R 在跳
=================================================================
'''

import time

try:
    from machine import UART as MachineUART
    from machine import FPIOA
    HAS_MACHINE = True
except Exception:
    HAS_MACHINE = False

try:
    import os
except Exception:
    os = None


# ============================ 配置 ============================
# "gh2"  = 板载 GH1.25 串口2 座子     GPIO11(TX) / GPIO12(RX)   ← 默认
# "gh3"  = 板载 GH1.25 串口3 座子     K230: GPIO50/51；Lite K230D: GPIO32/33
# "hdr2" = 40Pin 排针 串口2           GPIO5(pin11 TX) / GPIO6(pin13 RX)
# "hdr1" = 40Pin 排针 串口1           GPIO3(pin8 TX)  / GPIO4(pin10 RX)
PORT = "gh2"

BAUD        = 115200
INTERVAL_MS = 500       # 多久发一行

# 发什么：
#   "text"   普通文本  K230 #00001   （给人看，不给程序解析）
#   "frames" 协议帧    C/L/NG 轮流     （给上位机/主控解析用）
MODE = "text"

PRINT_EACH = True       # 每发一行就在 IDE 里打一条
# ============================================================

# "frames" 模式轮流发的帧（都符合 Hardware/Serial.c 的解析规则）
FRAME_CYCLE = ("C318,242,41", "C320,240,40", "L4,192", "L-12,-345", "NG")

VERSION = "K230 串口只发测试 v1"

UART_OBJ = None        # 全局留着，退出时统一 deinit


def exitpoint():
    """CanMV 的长循环里必须每轮调 os.exitpoint()，否则 IDE 停止/Ctrl-C 不生效。
    电脑上没有这个函数，忽略即可。"""
    try:
        if os is not None:
            os.exitpoint()
    except Exception:
        pass


def close_uart():
    """退出时关串口，重跑脚本时更干净。"""
    global UART_OBJ
    try:
        if UART_OBJ is not None:
            UART_OBJ.deinit()
            print("串口已关闭")
    except Exception:
        pass
    UART_OBJ = None


# ============================ 纯逻辑（电脑上也能测） ============================

def board_id():
    try:
        return str(os.uname()[-1])
    except Exception:
        return "unknown"


def port_conf(port, bid=None):
    """PORT 名字 -> {id, tx, rx, name}。串口3 按板卡自动选引脚。"""
    bid = board_id() if bid is None else bid
    if port == "gh3":
        if ("lite" in bid) or ("k230d" in bid):
            return {"id": 3, "tx": 32, "rx": 33,
                    "name": "GH1.25 串口3 (Lite K230D: GPIO32/33)"}
        return {"id": 3, "tx": 50, "rx": 51,
                "name": "GH1.25 串口3 (庐山派 K230: GPIO50/51)"}
    if port == "hdr2":
        return {"id": 2, "tx": 5, "rx": 6,
                "name": "40Pin 排针 串口2 (GPIO5/GPIO6)"}
    if port == "hdr1":
        return {"id": 1, "tx": 3, "rx": 4,
                "name": "40Pin 排针 串口1 (GPIO3/GPIO4)"}
    return {"id": 2, "tx": 11, "rx": 12,
            "name": "GH1.25 串口2 (GPIO11/GPIO12)"}


def line_for(n, mode=None):
    """第 n 行（从 1 开始）要发的内容，不含 \\r\\n。"""
    mode = MODE if mode is None else mode
    if mode == "frames":
        return FRAME_CYCLE[(n - 1) % len(FRAME_CYCLE)]
    return "K230 #%05d" % n


def describe_payload(s):
    """给串口两边看得更清楚：长度 + 实际发出去的字节。"""
    return "len=%d  %r" % (len(s) + 2, s + "\\r\\n")


# ============================ 平台部分 ============================

def open_uart(conf):
    from machine import UART, FPIOA

    fpioa = FPIOA()
    fpioa.set_function(conf["tx"], getattr(FPIOA, "UART%d_TXD" % conf["id"]))
    fpioa.set_function(conf["rx"], getattr(FPIOA, "UART%d_RXD" % conf["id"]))

    uid = getattr(UART, "UART%d" % conf["id"])
    try:
        return UART(uid, baudrate=BAUD, bits=UART.EIGHTBITS,
                    parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
    except Exception:
        pass
    try:
        return UART(uid, BAUD, bits=UART.EIGHTBITS,
                    parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
    except Exception:
        pass
    return UART(uid, BAUD)


def main():
    conf = port_conf(PORT)

    print("=" * 46)
    print(VERSION)
    print("  板卡 : %s" % board_id())
    print("  接口 : %s" % conf["name"])
    print("  引脚 : TX=GPIO%d（只用这一根）  RX=GPIO%d（本脚本不读）"
          % (conf["tx"], conf["rx"]))
    print("  参数 : %d 8N1   每 %d ms 一行" % (BAUD, INTERVAL_MS))
    print("  内容 : MODE=%s" % MODE)
    print("=" * 46)

    if not HAS_MACHINE:
        print("没有 machine 模块，跑不了")
        return

    u = open_uart(conf)
    UART_OBJ = u
    print("串口已打开，开始发……（IDE 里按停止 / 重新运行本脚本 才会停）")

    n = 0
    total = 0
    t_next = time.ticks_ms()

    while True:
        exitpoint()                 # 让 IDE 的停止按钮能生效
        n += 1
        line = line_for(n, MODE)
        data = line + "\r\n"
        try:
            u.write(data)
            total += len(data)
        except Exception as e:
            print("[%d] 发送失败: %s" % (n, e))
            time.sleep_ms(INTERVAL_MS)
            continue

        if PRINT_EACH:
            print("[%4d] %s" % (n, describe_payload(line)))
        elif n % 20 == 0:
            print("[%4d] 还在发，累计 %d 字节" % (n, total))

        # 按固定周期发，不累积误差
        t_next = time.ticks_add(t_next, INTERVAL_MS)
        gap = time.ticks_diff(t_next, time.ticks_ms())
        if gap > 0:
            time.sleep_ms(gap)
        else:
            t_next = time.ticks_ms()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已停止")
    finally:
        close_uart()
