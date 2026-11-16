# -*- coding: utf-8 -*-
"""假主控：用电脑冒充 STM32F103，通过 CH340 跟 K230 对话。

为什么要它：车不在手边时，先用电脑把 K230 那一半调通，之后再接真主控就只剩接线问题。

接线（CH340 必须是 3.3V 电平的模块）：
    CH340 TXD  ->  K230 R (GPIO12)
    CH340 RXD  <-  K230 T (GPIO11)
    CH340 GND  --  K230 G
    CH340 VCC  不接（K230 自己供电）

用法：
    python tools/k230_host.py                 # 自动找 CH340
    python tools/k230_host.py --port COM5     # 指定串口
    python tools/k230_host.py --list          # 列出所有串口
    python tools/k230_host.py --selftest      # 不用硬件，先验证解析逻辑
    python tools/k230_host.py --send circle --duration 5    # 发一次，看 5 秒

运行时按键（不用回车）：
    c = 发 circle      l = 发 linear      s = 发 stop
    ? = 帮助           q = 退出

协议（和 Hardware/Serial.c 的 K230_ParseLine 严格一致）：
    下行  "circle\\r\\n" / "linear\\r\\n" / "stop\\r\\n"
    上行  C<x>,<y>,<r>   x,y,r 各 0~999
          L<k>,<psi>     k = 斜率×100，±9999；psi = 0.01°，±18000
          NG             没找到
          <x>,<y>        旧的纯数字格式
"""

import os
import sys
import time
import argparse

# ---------------------------------------------------------------- 常量

BAUD_DEFAULT = 115200
CH340_VIDS = (0x1A86,)          # 沁恒 CH340/CH341
# 这些是 K230 自己的 USB 口（CanMV IDE 用的 REPL 口），绝不能当数据口用：
#   往那里发 "circle" 只会打进 Micropython 的 REPL，不是我们脚本的 UART2。
NOT_DATA_HINTS = ("openmv", "canmv", "k230", "micropython")
NOT_DATA_VIDS = (0x1209,)       # pid.codes（OpenMV/CanMV 拿来当 CDC）
CMD_CIRCLE = b"circle\r\n"
CMD_LINEAR = b"linear\r\n"
CMD_STOP = b"stop\r\n"


# ---------------------------------------------------------------- 解析（照抄 C）

def _parse_int(s, i):
    """从 s[i] 读一个可带 +/- 的整数。返回 (值, 新下标) 或 (None, i)。"""
    n = len(s)
    while i < n and s[i] == " ":
        i += 1
    neg = False
    if i < n and s[i] == "-":
        neg = True
        i += 1
    elif i < n and s[i] == "+":
        i += 1
    start = i
    v = 0
    while i < n and "0" <= s[i] <= "9":
        v = v * 10 + (ord(s[i]) - 48)
        i += 1
        if i - start > 5:
            return (None, i)
    if i == start:
        return (None, i)
    return (-v if neg else v, i)


def _expect(s, i, ch):
    n = len(s)
    while i < n and s[i] == " ":
        i += 1
    if i < n and s[i] == ch:
        return (True, i + 1)
    return (False, i)


def _at_end(s, i):
    n = len(s)
    while i < n and s[i] == " ":
        i += 1
    return i >= n


def decode(line):
    """把一行文本解析成字典。和 C 的 K230_ParseLine 一一对应。

    返回 dict：
        kind    "C" / "L" / "NG" / "XY" / None
        cx,cy,r           kind=="C"
        k100,psi_centi    kind=="L"
        x,y               kind=="XY"
        raw               原始行
        why               解析失败时的原因
    """
    out = {"kind": None, "raw": line, "why": ""}

    s = (line or "").strip()
    if not s:
        out["why"] = "空行"
        return out

    c = s[0]
    if c.islower():
        c = c.upper()

    # ---- C<x>,<y>,<r> ----
    if c == "C":
        i = 1
        cx, i = _parse_int(s, i)
        if cx is None:
            out["why"] = "C 帧第一个数读不出来"
            return out
        ok, i = _expect(s, i, ",")
        if not ok:
            out["why"] = "C 帧缺逗号"
            return out
        cy, i = _parse_int(s, i)
        if cy is None:
            out["why"] = "C 帧第二个数读不出来"
            return out
        ok, i = _expect(s, i, ",")
        if not ok:
            out["why"] = "C 帧缺第二个逗号"
            return out
        r, i = _parse_int(s, i)
        if r is None:
            out["why"] = "C 帧第三个数读不出来"
            return out
        if not _at_end(s, i):
            out["why"] = "C 帧行尾有多余字符"
            return out
        for name, v in (("x", cx), ("y", cy), ("r", r)):
            if v < 0 or v > 999:
                out["why"] = "C 帧 %s 超范围 0~999" % name
                return out
        out.update({"kind": "C", "cx": cx, "cy": cy, "r": r})
        return out

    # ---- L<k>,<psi> ----
    if c == "L":
        i = 1
        k, i = _parse_int(s, i)
        if k is None:
            out["why"] = "L 帧斜率读不出来"
            return out
        ok, i = _expect(s, i, ",")
        if not ok:
            out["why"] = "L 帧缺逗号"
            return out
        psi, i = _parse_int(s, i)
        if psi is None:
            out["why"] = "L 帧偏航读不出来"
            return out
        if not _at_end(s, i):
            out["why"] = "L 帧行尾有多余字符"
            return out
        if k < -9999 or k > 9999:
            out["why"] = "L 帧斜率超范围 ±9999"
            return out
        if psi < -18000 or psi > 18000:
            out["why"] = "L 帧偏航超范围 ±18000"
            return out
        out.update({"kind": "L", "k100": k, "psi_centi": psi})
        return out

    # ---- NG ----
    if c == "N":
        rest = s[1:].strip()
        if rest.upper() == "G":
            out["kind"] = "NG"
            return out
        out["why"] = "只认 NG，认不出 %r" % s
        return out

    # ---- 旧的 x,y ----
    if "0" <= s[0] <= "9":
        x, i = _parse_int(s, 0)
        ok, i = _expect(s, i, ",")
        if x is None or not ok:
            out["why"] = "旧格式应为 x,y"
            return out
        y, i = _parse_int(s, i)
        if y is None or not _at_end(s, i):
            out["why"] = "旧格式应为 x,y"
            return out
        if x < 0 or x > 999 or y < 0 or y > 999:
            out["why"] = "旧格式数值超范围 0~999"
            return out
        out.update({"kind": "XY", "x": x, "y": y})
        return out

    out["why"] = "不认识的帧头 %r" % s[0]
    return out


def describe(d):
    """把 decode() 的结果变成一行给人看的说明。"""
    kind = d["kind"]
    if kind == "C":
        return "X=%d Y=%d R=%d" % (d["cx"], d["cy"], d["r"])
    if kind == "L":
        return "k=%+.3f  psi=%+.2f° (%d)" % (d["k100"] / 100.0,
                                             d["psi_centi"] / 100.0,
                                             d["psi_centi"])
    if kind == "NG":
        return "没找到"
    if kind == "XY":
        return "旧格式 X=%d Y=%d" % (d["x"], d["y"])
    return "!! " + d["why"]


# ---------------------------------------------------------------- 键盘

def key_pressed():
    """非阻塞读一个键；没按键返回 None。优先 msvcrt（Windows，不用回车）。"""
    try:
        import msvcrt
        if msvcrt.kbhit():
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):      # 方向键等两字节键，吞掉第二个
                msvcrt.getwch()
                return None
            return ch
        return None
    except Exception:
        pass
    try:
        import select
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if r:
            return sys.stdin.read(1)
    except Exception:
        pass
    return None


# ---------------------------------------------------------------- 串口

def is_k230_usb_port(p):
    """这个口是不是 K230 自己的 USB（REPL），不能拿来发数据。"""
    desc = (p.description or "").lower()
    for s in NOT_DATA_HINTS:
        if s in desc:
            return True
    return (p.vid in NOT_DATA_VIDS)


def list_ports():
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("没发现任何串口。CH340 插上了吗？驱动装了吗？")
        return []
    print("%-10s %-28s %-10s %s" % ("端口", "描述", "VID:PID", "判定"))
    for p in ports:
        vid = ("%04X:%04X" % (p.vid, p.pid)) if p.vid else "-"
        if p.vid in CH340_VIDS:
            mark = "CH340，可以用"
        elif is_k230_usb_port(p):
            mark = "K230 的 USB(REPL)，别当数据口"
        else:
            mark = "可以试"
        print("%-10s %-28s %-10s %s" % (p.device, (p.description or "")[:28], vid, mark))
    return ports


def auto_port():
    """优先按 VID 找 CH340；排除 K230 自己的 USB 口；只剩一个时才盲选。"""
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())

    for p in ports:
        if p.vid in CH340_VIDS:
            return p.device, "自动识别为 CH340"

    usable = [p for p in ports if not is_k230_usb_port(p)]
    if len(usable) == 1:
        return usable[0].device, "唯一的可用串口"

    if not usable and ports:
        print("只找到 K230 自己的 USB 口（那是 CanMV IDE 的 REPL），")
        print("数据口要在 GH1.25 串口2 上接 CH340。现在把 CH340 插上再试。")
        return None, None
    return None, None


def open_port(port, baud):
    import serial
    try:
        ser = serial.Serial(port, baud, timeout=0, write_timeout=1)
    except (ValueError, serial.SerialException):
        # 支持 pyserial 的 URL 形式，例如 loop://（自己发自己收，可以不带硬件先验通路）
        ser = serial.serial_for_url(port, baud, timeout=0, write_timeout=1)
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    return ser


# ---------------------------------------------------------------- 自检

def selftest():
    """不用硬件，先把解析逻辑过一遍。"""
    ok = 0
    fail = 0

    def chk(name, got, want):
        nonlocal ok, fail
        if got == want:
            ok += 1
            print("[ok] %-40s -> %r" % (name, got))
        else:
            fail += 1
            print("[NG] %-40s got=%r want=%r" % (name, got, want))

    d = decode("C318,242,41")
    chk("C 帧: 类型", d["kind"], "C")
    chk("C 帧: x", d["cx"], 318)
    chk("C 帧: y", d["cy"], 242)
    chk("C 帧: r", d["r"], 41)
    chk("C 帧: 小写", decode("c1,2,3")["kind"], "C")
    chk("C 帧: 空格", decode("C 1 , 2 , 3")["cx"], 1)
    chk("C 帧: 缺字段 -> 失败", decode("C318,242")["kind"], None)
    chk("C 帧: 超范围 -> 失败", decode("C1000,2,3")["kind"], None)

    d = decode("L4,192")
    chk("L 帧: k100", d["k100"], 4)
    chk("L 帧: psi", d["psi_centi"], 192)
    chk("L 帧: 负值", decode("L-12,-345")["psi_centi"], -345)
    chk("L 帧: 边界", decode("L0,18000")["kind"], "L")
    chk("L 帧: 超界 -> 失败", decode("L0,18001")["kind"], None)

    chk("NG", decode("NG")["kind"], "NG")
    chk("ng", decode("ng")["kind"], "NG")
    chk("N 单个字母 -> 失败", decode("N")["kind"], None)

    chk("旧格式", decode("318,242")["kind"], "XY")
    chk("旧格式: x", decode("318,242")["x"], 318)
    chk("乱码 -> 失败", decode("hello")["kind"], None)
    chk("空行 -> 失败", decode("")["kind"], None)

    chk("说明: C", describe(decode("C318,242,41")), "X=318 Y=242 R=41")
    chk("说明: L", describe(decode("L4,192")), "k=+0.040  psi=+1.92° (192)")
    print("\n%s（%d 项通过，%d 项失败）" % ("全部通过" if fail == 0 else "有失败项",
                                          ok, fail))
    return 0 if fail == 0 else 1


# ---------------------------------------------------------------- 主流程

HELP = """按键（不用回车）：
  c  发 circle  -> K230 找圆心，回 C 帧
  l  发 linear  -> K230 找直线，回 L 帧
  s  发 stop    -> 让 K230 停下来
  ?  再显示这行帮助
  q  退出
"""


def main():
    ap = argparse.ArgumentParser(description="假主控：电脑冒充 F103 跟 K230 对话")
    ap.add_argument("--port", help="串口号，例如 COM5；不填就自动找 CH340")
    ap.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    ap.add_argument("--list", action="store_true", help="列出串口后退出")
    ap.add_argument("--selftest", action="store_true", help="不用硬件，验证解析逻辑")
    ap.add_argument("--send", choices=("circle", "linear", "stop"),
                    help="只发一条命令，不进入交互模式")
    ap.add_argument("--duration", type=float, default=0,
                    help="配合 --send：跑多少秒后退出（0 = 一直跑到 Ctrl-C）")
    ap.add_argument("--log", help="把收到的原始行写到这个文件")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(selftest())

    try:
        import serial  # noqa: F401
    except ImportError:
        print("缺 pyserial。装一下：")
        print("    python -m pip install pyserial")
        sys.exit(1)

    if args.list:
        list_ports()
        sys.exit(0)

    port = args.port
    how = "指定"
    if not port:
        port, how = auto_port()
        if not port:
            print("")
            print("自动找 CH340 失败。当前所有串口如下：")
            print("")
            list_ports()
            print("")
            print("接好 CH340 再双击这个 bat；或用 --port COMx 手动指定。")
            sys.exit(1)

    print("串口 %s（%s）  %d 8N1" % (port, how, args.baud))
    try:
        ser = open_port(port, args.baud)
    except Exception as e:
        print("打不开 %s：%s" % (port, e))
        print("常见原因：别的程序（串口助手/IDE）还占着它；线没插好；驱动没装。")
        sys.exit(1)

    log = open(args.log, "w", encoding="utf-8") if args.log else None

    rx_buf = bytearray()
    n_frame = 0
    n_ng = 0
    n_bad = 0
    n_xy = 0
    last_data_t = None
    t0 = time.time()
    last_stat = t0
    last_kind = None

    def emit(s):
        print(s)
        if log:
            log.write(s + "\n")
            log.flush()

    def send(what, raw):
        ser.write(raw)
        emit("[%7.3f] -> %s" % (time.time() - t0, what))
        return time.time()

    def drain(duration):
        """收 duration 秒，边收边解析打印。返回收到的字节数。"""
        nonlocal n_frame, n_ng, n_bad, n_xy, last_data_t, last_stat, last_kind
        got = 0
        t_end = time.time() + duration if duration > 0 else None
        while True:
            try:
                chunk = ser.read(256)
            except Exception as e:
                print("读串口出错：%s" % e)
                return got
            if chunk:
                got += len(chunk)
                rx_buf.extend(chunk)
                while True:
                    idx = -1
                    for i, b in enumerate(rx_buf):
                        if b in (10, 13):
                            idx = i
                            break
                    if idx < 0:
                        if len(rx_buf) > 64:      # 没有换行还这么长，整段丢掉
                            emit("[%7.3f] !! 收到 %d 字节但没换行，丢弃"
                                 % (time.time() - t0, len(rx_buf)))
                            n_bad += 1
                            del rx_buf[:]
                        break
                    line = rx_buf[:idx].decode("ascii", "replace")
                    del rx_buf[:idx + 1]
                    line = line.strip()
                    if not line:
                        continue
                    d = decode(line)
                    if d["kind"] in ("C", "L", "NG"):
                        n_frame += 1
                        last_data_t = time.time()
                        last_kind = d["kind"]
                        if d["kind"] == "NG":
                            n_ng += 1
                        emit("[%7.3f] <- %-16s %s"
                             % (time.time() - t0, line, describe(d)))
                    elif d["kind"] == "XY":
                        n_xy += 1
                        last_data_t = time.time()
                        last_kind = "XY"
                        emit("[%7.3f] <- %-16s %s"
                             % (time.time() - t0, line, describe(d)))
                    else:
                        n_bad += 1
                        emit("[%7.3f] !! 丢弃 %-14r %s"
                             % (time.time() - t0, line, d["why"]))

                now = time.time()
                if now - last_stat >= 2.0:
                    last_stat = now
                    dt = now - t0
                    rate = n_frame / dt if dt > 0 else 0
                    emit("--- %.1fs: 收 %d 帧 (%.1f Hz)  NG %d  旧格式 %d  丢弃 %d ---"
                         % (dt, n_frame, rate, n_ng, n_xy, n_bad))

            if args.send and t_end is not None and time.time() > t_end:
                return got
            if not args.send:
                return got          # 交互模式：drain 只跑这一轮
            time.sleep(0.002)
        return got

    print("=" * 62)
    print("假主控已连上。K230 那边应该已经跑着 圆线串口.py")
    print("先在 K230 的 IDE 里看有没有 '[cmd] circle' 这类日志，能对上就通了")
    print("=" * 62)

    # ---- 一次性模式 ----
    if args.send:
        raw = {"circle": CMD_CIRCLE, "linear": CMD_LINEAR,
               "stop": CMD_STOP}[args.send]
        send(args.send, raw)
        dur = args.duration if args.duration > 0 else 3.0
        print("收 %.1f 秒……" % dur)
        drain(dur)
        print("--- 结束：收 %d 帧，NG %d，丢弃 %d ---" % (n_frame, n_ng, n_bad))
        if log:
            log.close()
        return

    # ---- 交互模式 ----
    print(HELP)
    wait_start = None
    try:
        while True:
            ch = key_pressed()
            if ch:
                ch = ch.lower()
                if ch in ("c", "l"):
                    if ch == "c":
                        send("circle", CMD_CIRCLE)
                    else:
                        send("linear", CMD_LINEAR)
                    last_data_t = None
                    wait_start = time.time()
                elif ch == "s":
                    send("stop", CMD_STOP)
                    wait_start = None
                elif ch == "?":
                    print(HELP)
                elif ch == "q":
                    break
                elif ch in ("\r", "\n"):
                    pass
                else:
                    print("（不认识的键 %r，按 ? 看帮助）" % ch)

            drain(0.05)

            if last_data_t is not None:
                wait_start = None
            elif wait_start is not None and time.time() - wait_start > 2.0:
                emit("[%7.3f] !! 命令发出 2 秒了还没收到数据，按顺序查：\n"
                     "        1) CH340 的 TXD 是不是接到了 K230 的 R(GPIO12)\n"
                     "        2) CH340 的 RXD 是不是接到了 K230 的 T(GPIO11)\n"
                     "        3) GND 有没有连（K230 的 G）\n"
                     "        4) K230 那边 圆线串口.py 跑起来了吗、IDE 里有没有 [cmd] 日志"
                     % (time.time() - t0))
                wait_start = None
    except KeyboardInterrupt:
        print("\nCtrl-C 退出")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        if log:
            log.close()
        print("--- 共收 %d 帧，NG %d，丢弃 %d ---" % (n_frame, n_ng, n_bad))


if __name__ == "__main__":
    main()
