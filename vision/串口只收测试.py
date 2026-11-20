# -*- coding: utf-8 -*-
'''
K230 串口「只收」测试（板载 GH1.25 串口2，GPIO11=TX / GPIO12=RX）
平台：立创·庐山派 K230 CanMV / Lite K230D

只做一件事：**一直读串口，把收到的字节原样打出来**，不管发。
用来单独确认 **接收方向** 没问题 —— 配合 F103 的 DEBUG_MODE_UART_TEST（模式 9）
或任意串口助手发数据，看 K230 到底能不能收到。

为什么单独做一个：串口不出东西时，你分不清是
    "对方没发" / "发到了但 K230 收不到" / "收是收到了但解析错了"。
这份只看收，前两个问题一眼就能定位。

============================= 怎么用 =============================
接法：把「对方」的 TX 接到 K230 的 R（GPIO12），GND 共地。
      K230 的 T（GPIO11）本脚本不驱动任何数据，可以不接。

场景一：接 F103（配合 main.c 的 DEBUG_MODE_UART_TEST = 9）
    F103 的 TX  ->  K230 R (GPIO12)
    GND 共地
    F103 每 500 ms 发一行 "K230TEST #00001"，本脚本应打印：
        [  1] 收到 1 行: 'K230TEST #00001'  len=15
        ★ 命中 F103 测试帧（UART_TEST）

场景二：接电脑 USB-TTL（最快验证 K230 的 RX 脚是好的）
    用串口助手发 "hello"，本脚本应立刻打印 'hello'。

场景三：T-R 短接（GPIO11 直连 GPIO12）
    再跑 串口只发测试.py 或本脚本的 LOOPBACK，确认脚本身没坏。
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

BAUD = 115200

POLL_MS      = 10       # 多久读一次（10 ms 足够，115200 下每次最多十几个字节）
HEARTBEAT_MS = 5000     # 每隔多久打一行"还活着、收到多少"
SHOW_HEX     = True     # 收到非 ASCII 字节时补一行十六进制
MAX_LINE     = 64       # 一行最多攒多少字节，超了就切断

# 只关心这个前缀的帧（F103 的 UART_TEST 发的就是它）；留空 = 不特别标记
WATCH_PREFIX = "K230TEST #"
# ============================================================

VERSION = "K230 串口只收测试 v1"

UART_OBJ = None


def exitpoint():
    """CanMV 的长循环里必须每轮调 os.exitpoint()，否则 IDE 停止/Ctrl-C 不生效。"""
    try:
        if os is not None:
            os.exitpoint()
    except Exception:
        pass


def close_uart():
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


def to_bytes(d):
    """CanMV 的 uart.read() 有时返回 bytes，有时返回 str；统一成 bytes。"""
    if d is None:
        return b""
    if isinstance(d, (bytes, bytearray)):
        return bytes(d)
    try:
        return bytes(d)
    except Exception:
        try:
            return d.encode()
        except Exception:
            return b""


def hexdump(bs):
    return " ".join("%02X" % b for b in bs)


def is_printable(bs):
    for b in bs:
        if b < 0x20 or b > 0x7E:
            if b not in (0x09, 0x0A, 0x0D):
                return False
    return True


def describe(bs):
    """人能看懂的一行：能显示就给字符串，否则给十六进制。"""
    if is_printable(bs):
        return repr(bs.decode("ascii", "replace"))
    return "HEX " + hexdump(bs)


# ============================ 平台部分 ============================

def open_uart(conf):
    from machine import UART
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

    print("=" * 52)
    print(VERSION)
    print("  板卡 : %s" % board_id())
    print("  接口 : %s" % conf["name"])
    print("  引脚 : RX=GPIO%d（对方 TX 接这里）" % conf["rx"])
    print("         TX=GPIO%d（本脚本不发数据）" % conf["tx"])
    print("  参数 : %d 8N1" % BAUD)
    print("  关注 : %s" % (("前缀 " + repr(WATCH_PREFIX)) if WATCH_PREFIX else "全部"))
    print("=" * 52)

    if not HAS_MACHINE:
        print("没有 machine 模块，跑不了")
        return

    u = open_uart(conf)
    global UART_OBJ
    UART_OBJ = u

    print("串口已打开，开始收……（IDE 里按停止 / 重新运行本脚本 才会停）")
    print("提示：先让对方发，再回头看这个窗口。一条都收不到就依次查")
    print("      1) 对方 TX 是否接到 GPIO%d（不是 GPIO%d）" % (conf["rx"], conf["tx"]))
    print("      2) GND 是否共地")
    print("      3) 波特率是否两边都是 %d" % BAUD)
    print()

    buf = b""
    n_lines = 0
    n_bytes = 0
    last_rx_ms = None
    max_gap_ms = 0
    t_beat = time.ticks_ms()

    while True:
        exitpoint()

        try:
            d = to_bytes(u.read())
        except Exception as e:
            print("[ERR] 读串口异常: %s" % e)
            time.sleep_ms(200)
            continue

        if d:
            n_bytes += len(d)
            now = time.ticks_ms()
            if last_rx_ms is not None:
                gap = time.ticks_diff(now, last_rx_ms)
                if gap > max_gap_ms:
                    max_gap_ms = gap
            last_rx_ms = now

            if SHOW_HEX and not is_printable(d):
                print("      RAW %s" % hexdump(d))

            buf += d

            # 按 \r 或 \n 切行，可能一次收到好几行
            while True:
                idx = -1
                for k, ch in enumerate(buf):
                    if ch in (0x0D, 0x0A):
                        idx = k
                        break
                if idx < 0:
                    break
                line = buf[:idx]
                buf = buf[idx + 1:]
                if not line:
                    continue
                n_lines += 1
                text = describe(line)
                hit = bool(WATCH_PREFIX) and line.startswith(WATCH_PREFIX.encode())
                print("[%4d] %s  len=%d%s"
                      % (n_lines, text, len(line), "  ★ 命中 F103 测试帧" if hit else ""))

            # 没有换行的长数据：攒太多就强行切一段出来，避免一直不显示
            if len(buf) >= MAX_LINE:
                n_lines += 1
                print("[%4d] %s  len=%d  (无换行，强制切断)"
                      % (n_lines, describe(buf), len(buf)))
                buf = b""

        # 心跳：确认脚本还在轮询（不是卡死）
        if time.ticks_diff(time.ticks_ms(), t_beat) >= HEARTBEAT_MS:
            t_beat = time.ticks_ms()
            idle = "-"
            if last_rx_ms is not None:
                idle = "%d ms 前" % time.ticks_diff(time.ticks_ms(), last_rx_ms)
            print("---- 已收 %d 字节 / %d 行   最大间隔 %d ms   最近一次 %s   半行缓存 %d 字节----"
                  % (n_bytes, n_lines, max_gap_ms, idle, len(buf)))

        time.sleep_ms(POLL_MS)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已停止")
    finally:
        close_uart()
