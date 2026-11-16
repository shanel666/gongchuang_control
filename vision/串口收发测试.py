# -*- coding: utf-8 -*-
'''
K230 串口收发自检（立创·庐山派 K230 CanMV / Lite K230D）
平台：CanMV K230 固件 + CanMV IDE

只干一件事：往串口发东西、把收到的东西打出来、逐帧比对。
不碰摄像头、不碰主控，纯串口。

============================= 一、怎么接 =============================
【方式 A：回环自检，只用一根杜邦线】  ← 建议第一次就做这个
    把板载 GH1.25 串口2 座子的 **T 和 R 短接**（同一根线插两个孔），V 和 G 不用接。
    ★ 回环时必须 ECHO = False，否则发出去的数据会被自己收回来再发出去，
      RX 会一帧比一帧长（8、16、24…），看着像故障其实不是。

【方式 B：接电脑串口助手】
    板子 GH1.25 串口2         USB-TTL 模块
        T (GPIO11 TX)   --->   RXD
        R (GPIO12 RX)   <---   TXD
        G (GND)         ---    GND
        V (5V)          不接
    串口助手设 115200, 8, N, 1。
    · 只想看板子发东西 -> CHECK_FRAME = False
    · 想让板子把自己收到的原样回给你 -> CHECK_FRAME = False, ECHO = True

【方式 C：以后接 F103 主控】
    K230 T(GPIO11) -> F103 PA3 (USART2_RX)
    K230 R(GPIO12) <- F103 PA2 (USART2_TX)
    GND 共地，115200 8N1
    ★ 别接 F103 的 PA9/PA10，那是步进电机总线，也是串口烧录口。

============================= 二、注意 =============================
  · 一定要共地，否则什么都收不到；
  · TX 接对方 RX，不能 TX 接 TX；
  · 电平 3.3V。USB-TTL 模块必须是 3.3V 电平的（5V 的可能烧 IO）；
  · 座子的 V 是 5V，别接进 F103；
  · 串口0（丝印 0）被 RT-Smart 系统占用，用户调不了。
=================================================================
'''

import time
import os

# ============================ 三、配置 ============================
# 用哪个口：
#   "gh2"  = 板载 GH1.25 串口2 座子     GPIO11(TX) / GPIO12(RX)   ← 默认
#   "gh3"  = 板载 GH1.25 串口3 座子     K230: GPIO50/51；Lite K230D: GPIO32/33
#   "hdr2" = 40Pin 排针 串口2           GPIO5(pin11 TX) / GPIO6(pin13 RX)
#   "hdr1" = 40Pin 排针 串口1           GPIO3(pin8 TX)  / GPIO4(pin10 RX)
PORT = "gh2"

BAUD          = 115200
SEND_EVERY_MS = 500     # 多久发一帧
READ_TIMEOUT_MS = 120   # 每帧最多等多久收回来（回环是瞬间的，不用等满 500ms）

LOOPBACK_TEST = True    # 开机先做一次长包回环自检（需要 T-R 短接）
CHECK_FRAME   = True    # 逐帧严格比对：收到的是不是就是刚发的那一帧
ECHO          = False   # 收到就原样回发。★ 回环(T-R短接)时必须 False，
                        #   否则 RX 会自我累加变长；只有接电脑调试时才打开。
PRINT_SILENT  = True    # 没收到数据时也打印（确认程序在跑）
PRINT_EVERY   = 1       # 每几帧打一行详细内容；嫌刷屏改成 10
# ==================================================================


# ============================ 纯逻辑（电脑上也能测） ============================

UART_OBJ = None        # 全局留着，退出时统一 deinit


def exitpoint():
    """CanMV 的长循环里必须每轮调 os.exitpoint()，否则 IDE 停止/Ctrl-C 不生效。"""
    try:
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


def board_id():
    """返回固件里的板卡标识，例如 "k230_canmv_lckfb"。"""
    try:
        return str(os.uname()[-1])
    except Exception:
        return "unknown"


def port_conf(port, bid=None):
    """把 PORT 名字换成 {id, tx, rx, name}。

    串口3 在两种板子上引脚不一样，这里按板卡标识自动选：
      Lite K230D -> GPIO32/33；其它（庐山派 K230）-> GPIO50/51。
    """
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


def escape_text(bs, max_txt=40):
    """把字节变成可打印的转义文本；太长就截断（避免 IDE 刷出巨型行）。"""
    txt = ""
    for c in bs:
        if 32 <= c < 127:
            txt += chr(c)
        elif c == 13:
            txt += "\\r"
        elif c == 10:
            txt += "\\n"
        elif c == 9:
            txt += "\\t"
        else:
            txt += "\\x%02X" % c
        if len(txt) >= max_txt:
            txt += "…"
            break
    return txt


def hex_text(bs, max_bytes=24):
    s = ""
    for c in bs[:max_bytes]:
        s += "%02X " % c
    if len(bs) > max_bytes:
        s += "..."
    return s.strip()


def describe(bs, max_txt=40):
    """一行摘要：长度 + 转义文本 + 十六进制。"""
    if not bs:
        return "len=0"
    return "len=%d txt=%s hex=[%s]" % (len(bs), escape_text(bs, max_txt),
                                       hex_text(bs))


def frame_bytes(n):
    """第 n 帧要发的内容：T00001\r\n 这种定长行。"""
    return b"T%05d\r\n" % n


def judge_frame(msg, got):
    """逐帧判定。返回 (ok, 说明)。

    ok=True  收到的和刚发的完全一样（干净回环）
    ok=False 说明里写清是"没收到"还是"多/少了字节"
    """
    if got == msg:
        return True, "回环 OK"
    if not got:
        return False, "一个字节都没收到"
    # 多收：ECHO 开着时，收到的会是"以前各帧 + 这一帧"，所以末尾也要看
    if len(got) > len(msg) and (got.endswith(msg) or got.startswith(msg)):
        return False, "多收了 %d 字节（ECHO 开着？上次的残留？）" % (len(got) - len(msg))
    if len(got) < len(msg) and (msg.startswith(got) or msg.endswith(got)):
        return False, "少了 %d 字节（丢字节：查波特率/线长/干扰）" % (len(msg) - len(got))
    return False, "内容和发出去的不一样"


# ============================ 平台部分 ============================

def open_uart(conf):
    """按 FPIOA 把引脚复用成 UART，再打开串口。"""
    from machine import UART, FPIOA

    fpioa = FPIOA()
    fpioa.set_function(conf["tx"], getattr(FPIOA, "UART%d_TXD" % conf["id"]))
    fpioa.set_function(conf["rx"], getattr(FPIOA, "UART%d_RXD" % conf["id"]))

    uid = getattr(UART, "UART%d" % conf["id"])

    # 不同固件的构造函数参数不完全一样，从最全的往下试
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


def read_for(u, ms):
    """在 ms 毫秒内尽量多读，返回收到的 bytes（没有就是 b""）。

    注意：K230 的 uart.read() 没数据时返回 None，不是 b""。
    """
    out = b""
    t_end = time.ticks_add(time.ticks_ms(), ms)
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        try:
            d = u.read()
        except Exception:
            d = None
        if d:
            out += d
        time.sleep_ms(5)
    return out


def drain(u):
    """把接收缓冲里的残留清掉，免得干扰判断。"""
    for _ in range(20):
        try:
            d = u.read()
        except Exception:
            d = None
        if not d:
            return
        time.sleep_ms(2)


def loopback_test(u, tag="长包回环"):
    """发一段较长的数据，看能不能一模一样收回来。"""
    pattern = b"LOOPBACK-K230-0123456789\r\n"
    drain(u)
    try:
        u.write(pattern)
    except Exception as e:
        print("%s: 发送就失败了 -> %s" % (tag, e))
        return False

    got = read_for(u, 400)
    if got == pattern:
        print("%s: PASS  %s" % (tag, describe(got)))
        return True

    print("%s: FAIL" % tag)
    print("    发出 %s" % describe(pattern))
    print("    收到 %s" % describe(got))
    if not got:
        print("    一个字节都没收到，按顺序查：")
        print("      1) T 和 R 真的短接了吗（同一根线插座子的两个孔）")
        print("      2) PORT 选对了吗（默认 gh2 = GPIO11/12）")
        print("      3) 板子上有没有别的设备占着这两个脚")
    else:
        print("    收到了但内容对不上：多半是波特率不对，或者中间有干扰/丢字节")
    return False


def main():
    global UART_OBJ
    conf = port_conf(PORT)

    print("=" * 46)
    print("K230 串口收发自检")
    print("  板卡 : %s" % board_id())
    print("  接口 : %s" % conf["name"])
    print("  引脚 : TX=GPIO%d  RX=GPIO%d" % (conf["tx"], conf["rx"]))
    print("  参数 : %d 8N1" % BAUD)
    print("  模式 : 逐帧比对=%s  原样回发=%s" % (CHECK_FRAME, ECHO))
    print("=" * 46)

    if ECHO and LOOPBACK_TEST:
        print("★ 提醒：T-R 短接时 ECHO 必须关掉，否则 RX 会一帧比一帧长。")
        print("        现在 ECHO=True，下面的逐帧比对会连续报 NG。")
        print()

    u = open_uart(conf)
    UART_OBJ = u
    print("串口已打开")

    if LOOPBACK_TEST:
        loopback_test(u)
        print("-" * 46)

    n = 0
    ok_n = 0
    ng_n = 0
    ng_shown = 0
    rx_total = 0
    tx_total = 0
    t_start = time.ticks_ms()

    while True:
        exitpoint()                 # 让 IDE 的停止按钮能生效
        n += 1
        msg = frame_bytes(n)
        t_frame = time.ticks_ms()

        try:
            u.write(msg)
            tx_total += len(msg)
        except Exception as e:
            print("[%4d] 发送失败: %s" % (n, e))
            time.sleep_ms(SEND_EVERY_MS)
            continue

        got = read_for(u, READ_TIMEOUT_MS)
        rx_total += len(got)

        if CHECK_FRAME:
            ok, why = judge_frame(msg, got)
            if ok:
                ok_n += 1
                if n <= 3 or (PRINT_EVERY > 0 and n % PRINT_EVERY == 0):
                    print("[%4d] %s  (发%d收%d)" % (n, why, len(msg), len(got)))
            else:
                ng_n += 1
                ng_shown += 1
                if ng_shown <= 5:                 # 前 5 次详细打，之后只计数
                    print("[%4d] 回环 NG: %s" % (n, why))
                    print("            发出 %s" % describe(msg))
                    print("            收到 %s" % describe(got))
        else:
            if got:
                ok_n += 1
                if n <= 3 or (PRINT_EVERY > 0 and n % PRINT_EVERY == 0):
                    print("[%4d] TX %s | RX %s" % (n, describe(msg), describe(got)))
            else:
                if PRINT_SILENT and n <= 3:
                    print("[%4d] TX %s | RX 0 字节" % (n, describe(msg)))

        if ECHO and got:
            try:
                u.write(got)
            except Exception:
                pass

        # 控制发送节奏：这一帧已经花了多久，补足到 SEND_EVERY_MS
        used = time.ticks_diff(time.ticks_ms(), t_frame)
        if used < SEND_EVERY_MS:
            time.sleep_ms(SEND_EVERY_MS - used)

        if n % 20 == 0:
            dt = time.ticks_diff(time.ticks_ms(), t_start)
            if CHECK_FRAME:
                print("    --- 统计: %d 帧  成功 %d  失败 %d  发 %d 字节 / 收 %d 字节 / %.1f 秒 ---"
                      % (n, ok_n, ng_n, tx_total, rx_total, dt / 1000.0))
            else:
                print("    --- 统计: %d 帧  有回 %d  发 %d 字节 / 收 %d 字节 / %.1f 秒 ---"
                      % (n, ok_n, tx_total, rx_total, dt / 1000.0))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已停止")
    finally:
        close_uart()
