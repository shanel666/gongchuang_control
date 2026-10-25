#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四麦轮底盘 · 无线串口上位机（Windows / Python 3.8+）

硬件链路：
    PC USB-TTL/蓝牙 → 无线串口模块 → STM32 USART3（PB10=TX / PB11=RX，115200 8N1）

固件端：main.c 的 DEBUG_MODE_WIRELESS 模式，命令是一行一条 ASCII：
    F 1000   前进 1000 ms        B 1000   后退
    L 1000   左平移              R 1000   右平移
    Q 1000   逆时针              E 1000   顺时针
    S        立即停车            D        内置测试（前进1s→逆时针1s→后退1s→顺时针1s）
    ?        查角度              P        ping
    V 300    设默认速度（0.1RPM） H        帮助

回包：READY / ACK <命令> <ms> / DONE <命令> / ABORT <命令> / YAW <0.01°> / PONG / ERR

运行：
    python tools/chassis_host.py
    python tools/chassis_host.py --selftest      # 只检查协议工具函数，不需要硬件
    python tools/chassis_host.py --list-ports    # 列出串口
"""

import argparse
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:                     # 没装 pyserial 时 GUI 里提示，--selftest 仍可用
    serial = None
    list_ports = None

BAUD = 115200
MIN_MS = 100
MAX_MS = 10000
DEFAULT_MS = 1000
MAX_SPEED_X10 = 30000

MOTION_KEYS = (
    ("F", "前进", "forward"),
    ("B", "后退", "backward"),
    ("L", "左平移", "left"),
    ("R", "右平移", "right"),
    ("Q", "逆时针", "ccw"),
    ("E", "顺时针", "cw"),
)


# ---------------------------------------------------------------------------
# 协议工具函数（不依赖串口，--selftest 直接测这些）
# ---------------------------------------------------------------------------
def build_cmd(cmd, ms=None):
    """把命令拼成固件要的一行：build_cmd("f", 1000) -> "F 1000" """
    cmd = str(cmd).strip().upper()
    if not cmd:
        raise ValueError("命令不能为空")
    if ms is None:
        return cmd
    return "{} {}".format(cmd, int(ms))


def clamp_ms(value):
    """把用户输入的毫秒数转成 int，并夹到 100~10000 ms。"""
    try:
        ms = int(float(value))
    except (TypeError, ValueError):
        raise ValueError("时间要填数字（毫秒）")
    if ms < MIN_MS:
        ms = MIN_MS
    if ms > MAX_MS:
        ms = MAX_MS
    return ms


def clamp_speed(value):
    """把速度（0.1RPM）转成 int，并夹到 0~30000。"""
    try:
        speed = int(float(value))
    except (TypeError, ValueError):
        raise ValueError("速度要填数字（0.1RPM，300 = 30.0RPM）")
    if speed < 0:
        speed = 0
    if speed > MAX_SPEED_X10:
        speed = MAX_SPEED_X10
    return speed


def parse_line(line):
    """把一行回包拆成 (kind, args)：'ACK F 1000' -> ('ACK', ['F', '1000'])"""
    parts = str(line).strip().split()
    if not parts:
        return "", []
    return parts[0].upper(), parts[1:]


def parse_yaw(line):
    """'YAW -1234' -> -1234；不是角度行或没有数据返回 None。"""
    kind, args = parse_line(line)
    if kind != "YAW" or not args:
        return None
    try:
        return int(args[0])
    except ValueError:
        return None


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------
class ChassisHost(object):
    def __init__(self, root):
        self.root = root
        self.ser = None
        self.reader = None
        self.rx_queue = queue.Queue()
        self.stop_event = threading.Event()

        self.port_var = tk.StringVar()
        self.duration_var = tk.StringVar(value=str(DEFAULT_MS))
        self.speed_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="未连接")
        self.yaw_var = tk.StringVar(value="--")

        self._build_ui()
        self._bind_keys()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(50, self._poll_queue)

    # ---------------- UI ----------------
    def _build_ui(self):
        self.root.title("四麦轮底盘 · 无线串口上位机 (115200)")
        self.root.geometry("900x680")
        self.root.minsize(760, 560)

        big = ("Microsoft YaHei UI", 10)
        title_font = ("Microsoft YaHei UI", 12, "bold")
        btn_font = ("Microsoft YaHei UI", 11, "bold")

        # 串口栏
        top = ttk.Frame(self.root, padding=(10, 8, 10, 4))
        top.pack(fill="x")
        ttk.Label(top, text="串口", font=big).pack(side="left")
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=32,
                                     state="readonly", font=big)
        self.port_box.pack(side="left", padx=(6, 6))
        ttk.Button(top, text="刷新", command=self.refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(top, text="连接", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=(8, 4))
        ttk.Label(top, text="状态:", font=big).pack(side="left", padx=(12, 2))
        ttk.Label(top, textvariable=self.status_var, font=big,
                  foreground="#1565c0").pack(side="left")

        # 运动按钮
        pad = ttk.LabelFrame(self.root, text=" 运动控制 ", padding=(10, 8))
        pad.pack(fill="x", padx=10, pady=4)

        for index, (cmd, text, _name) in enumerate(MOTION_KEYS):
            button = tk.Button(pad, text="{}  {}".format(cmd, text),
                               font=btn_font, width=13, height=2,
                               takefocus=0,
                               command=lambda c=cmd: self.send_action(c))
            button.grid(row=index // 3, column=index % 3, padx=5, pady=5, sticky="ew")
        for col in range(3):
            pad.columnconfigure(col, weight=1)

        row = ttk.Frame(pad)
        row.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(6, 0))

        ttk.Label(row, text="动作时长(ms)", font=big).pack(side="left")
        self.duration_spin = tk.Spinbox(
            row, from_=MIN_MS, to=MAX_MS, increment=100, width=8,
            textvariable=self.duration_var, font=big, justify="center")
        self.duration_spin.pack(side="left", padx=(4, 12))

        ttk.Label(row, text="默认速度(0.1RPM)", font=big).pack(side="left")
        self.speed_entry = tk.Entry(row, width=8, textvariable=self.speed_var,
                                    font=big, justify="center")
        self.speed_entry.pack(side="left", padx=(4, 4))
        tk.Button(row, text="应用", font=big, takefocus=0,
                  command=self.apply_speed).pack(side="left")

        # 特殊按钮
        row2 = ttk.Frame(pad)
        row2.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(6, 0))
        tk.Button(row2, text="停止 S", font=btn_font, takefocus=0,
                  bg="#ffcdd2", activebackground="#ef9a9a",
                  command=lambda: self.send("S")).pack(side="left", padx=4)
        tk.Button(row2, text="内置测试 D", font=btn_font, takefocus=0,
                  bg="#c8e6c9", activebackground="#a5d6a7",
                  command=lambda: self.send("D")).pack(side="left", padx=4)
        tk.Button(row2, text="查角度 ?", font=btn_font, takefocus=0,
                  command=lambda: self.send("?")).pack(side="left", padx=4)
        tk.Button(row2, text="Ping P", font=btn_font, takefocus=0,
                  command=lambda: self.send("P")).pack(side="left", padx=4)
        ttk.Label(row2, text="  当前角度(0.01°):", font=big).pack(side="left")
        ttk.Label(row2, textvariable=self.yaw_var, font=title_font,
                  foreground="#6a1b9a").pack(side="left")

        # 日志
        log_frame = ttk.LabelFrame(self.root, text=" 串口日志 ", padding=(8, 6))
        log_frame.pack(fill="both", expand=True, padx=10, pady=4)
        self.log_text = scrolledtext.ScrolledText(log_frame, height=14, wrap="none",
                                                  font=("Consolas", 10), state="disabled")
        self.log_text.pack(fill="both", expand=True)
        self.log_text.tag_config("TX", foreground="#1565c0")
        self.log_text.tag_config("RX", foreground="#2e7d32")
        self.log_text.tag_config("ERR", foreground="#c62828")

        # 手动发送
        bottom = ttk.Frame(self.root, padding=(10, 2, 10, 8))
        bottom.pack(fill="x")
        ttk.Label(bottom, text="手动命令", font=big).pack(side="left")
        self.cmd_entry = tk.Entry(bottom, font=("Consolas", 10))
        self.cmd_entry.pack(side="left", fill="x", expand=True, padx=(6, 6))
        self.cmd_entry.bind("<Return>", self._send_manual)
        ttk.Button(bottom, text="发送", command=self._send_manual).pack(side="left")

        ttk.Label(self.root,
                  text="快捷键：W 前进  S 后退  A 左移  D 右移  Q 逆时针  E 顺时针  空格 停止  "
                       "| 发光字/串口线：模块 TX→PB11，RX→PB10，共地",
                  font=("Microsoft YaHei UI", 9), foreground="#666").pack(pady=(0, 6))

    def _bind_keys(self):
        self.root.bind_all("<KeyPress>", self._on_key)

    def _on_key(self, event):
        # 输入框里打字时不抢快捷键
        if isinstance(event.widget, (tk.Entry, tk.Spinbox, ttk.Entry, ttk.Combobox)):
            return
        key = event.keysym.lower()
        mapping = {"w": "F", "s": "B", "a": "L", "d": "R",
                   "q": "Q", "e": "E", "space": "S"}
        if key == "space":
            self.send("S")
        elif key in mapping:
            self.send_action(mapping[key])

    # ---------------- 串口 ----------------
    def refresh_ports(self):
        if list_ports is None:
            self.status_var.set("未安装 pyserial")
            return
        values = []
        for port in list_ports.comports():
            values.append("{}  ({})".format(port.device, port.description))
        self.port_box["values"] = values
        if values and not self.port_var.get():
            self.port_var.set(values[0])
        elif not values:
            self.port_var.set("")

    def _port_name(self):
        text = self.port_var.get().strip()
        if not text:
            return ""
        return text.split()[0]          # 去掉后面的 "(USB-SERIAL CH340)" 描述

    def toggle_connect(self):
        if self.ser is not None:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        if serial is None:
            messagebox.showerror("缺少 pyserial", "请先安装：\npip install pyserial")
            return
        port = self._port_name()
        if not port:
            messagebox.showwarning("没有串口", "先点“刷新”，再选择一个串口")
            return
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.2, write_timeout=1.0)
        except Exception as exc:
            messagebox.showerror("打开串口失败", "{}:\n{}".format(port, exc))
            self.ser = None
            return

        self.stop_event.clear()
        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader.start()
        self.connect_btn.configure(text="断开")
        self.status_var.set("已连接 " + port)
        self._set_title_state(True)
        self.log("SYS", "已连接 {} @ {}".format(port, BAUD))
        self.send("P")                  # 先 ping 一下，确认链路

    def disconnect(self):
        self.stop_event.set()
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.reader = None
        self.connect_btn.configure(text="连接")
        self.status_var.set("未连接")
        self.yaw_var.set("--")
        self._set_title_state(False)
        self.log("SYS", "已断开")

    def _set_title_state(self, connected):
        self.port_box.configure(state="readonly" if not connected else "disabled")

    def _reader_loop(self):
        ser = self.ser
        while not self.stop_event.is_set():
            try:
                raw = ser.readline()
            except Exception:
                if not self.stop_event.is_set():
                    self.rx_queue.put("!SERIAL")
                return
            if not raw:
                continue
            text = raw.decode("ascii", "replace").strip()
            if text:
                self.rx_queue.put(text)

    # ---------------- 发送 ----------------
    def send(self, text):
        text = str(text).strip()
        if not text:
            return
        if self.ser is None:
            self.log("ERR", "串口未连接")
            self.status_var.set("串口未连接")
            return
        try:
            self.ser.write((text + "\n").encode("ascii", "replace"))
        except Exception as exc:
            self.log("ERR", "发送失败: {}".format(exc))
            return
        self.log("TX", text)
        if text[0].upper() in "FBLRQE":
            self.status_var.set("运行 " + text)

    def send_action(self, cmd):
        try:
            ms = clamp_ms(self.duration_var.get())
        except ValueError as exc:
            messagebox.showwarning("时间格式不对", str(exc))
            return
        self.duration_var.set(str(ms))
        self.send(build_cmd(cmd, ms))

    def apply_speed(self):
        text = self.speed_var.get().strip()
        if not text:
            messagebox.showwarning("速度为空", "填 0~30000，例如 300 = 30.0 RPM")
            return
        try:
            speed = clamp_speed(text)
        except ValueError as exc:
            messagebox.showwarning("速度格式不对", str(exc))
            return
        self.speed_var.set(str(speed))
        self.send(build_cmd("V", speed))

    def _send_manual(self, _event=None):
        text = self.cmd_entry.get().strip()
        if not text:
            return
        self.send(text)
        self.cmd_entry.delete(0, "end")

    # ---------------- 接收 ----------------
    def _poll_queue(self):
        try:
            while True:
                line = self.rx_queue.get_nowait()
                if line == "!SERIAL":
                    if self.ser is not None:
                        self.log("ERR", "串口读失败（线松了/模块掉电？）")
                        self.disconnect()
                    break
                self._on_line(line)
        except queue.Empty:
            pass
        self.root.after(50, self._poll_queue)

    def _on_line(self, line):
        self.log("RX", line)
        kind, args = parse_line(line)
        yaw = parse_yaw(line)
        if yaw is not None:
            self.yaw_var.set("{}  ({:.2f}°)".format(yaw, yaw / 100.0))
        elif kind == "YAW":
            self.yaw_var.set("NA")
        if kind == "ACK":
            self.status_var.set("已收到 ACK，动作中…")
        elif kind == "DONE":
            self.status_var.set("完成 " + " ".join(args))
        elif kind == "ABORT":
            self.status_var.set("已中止 " + " ".join(args))
        elif kind == "STOP":
            self.status_var.set("已停车")
        elif kind == "PONG":
            self.status_var.set("链路正常 (PONG)")
        elif kind == "SPEED":
            self.status_var.set("默认速度已设为 " + " ".join(args))
        elif kind == "ERR":
            self.status_var.set("命令错误 " + " ".join(args))
        elif kind == "READY":
            self.status_var.set("固件就绪")

    # ---------------- 其它 ----------------
    def log(self, tag, text):
        stamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", "[{}] {:<3} {}\n".format(stamp, tag, text), tag)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def on_close(self):
        self.disconnect()
        self.root.destroy()


# ---------------------------------------------------------------------------
# 自检 / 入口
# ---------------------------------------------------------------------------
def selftest():
    checks = [
        (build_cmd("f", 1000), "F 1000"),
        (build_cmd("Q", 250), "Q 250"),
        (build_cmd("s"), "S"),
        (clamp_ms("1000"), 1000),
        (clamp_ms("50"), MIN_MS),
        (clamp_ms("99999"), MAX_MS),
        (clamp_speed("300"), 300),
        (clamp_speed("-5"), 0),
        (clamp_speed("99999"), MAX_SPEED_X10),
        (parse_line("ACK F 1000"), ("ACK", ["F", "1000"])),
        (parse_yaw("YAW -1234"), -1234),
        (parse_yaw("YAW NA"), None),
        (parse_yaw("DONE F 1000"), None),
    ]
    failed = 0
    for got, want in checks:
        if got != want:
            failed += 1
            print("FAIL got={!r} want={!r}".format(got, want))
        else:
            print("ok   {!r}".format(got))
    if failed:
        print("selftest FAILED ({}/{})".format(failed, len(checks)))
        return 1
    print("selftest PASS ({})".format(len(checks)))
    return 0


def main():
    parser = argparse.ArgumentParser(description="四麦轮底盘无线串口上位机")
    parser.add_argument("--selftest", action="store_true", help="只测协议工具函数")
    parser.add_argument("--list-ports", action="store_true", help="列出可用串口")
    parser.add_argument("--port", help="直接连接这个串口（无 GUI 时不使用）")
    args = parser.parse_args()

    if args.selftest:
        return selftest()

    if args.list_ports:
        if serial is None:
            print("未安装 pyserial，请先 pip install pyserial")
            return 1
        ports = list(list_ports.comports())
        if not ports:
            print("没有找到串口")
        for port in ports:
            print("{}  {}".format(port.device, port.description))
        return 0

    if serial is None:
        print("缺少 pyserial，请先运行：pip install pyserial")
        return 1

    root = tk.Tk()
    app = ChassisHost(root)
    if args.port:
        # 允许命令行指定：python chassis_host.py --port COM5
        for value in app.port_box["values"]:
            if value.startswith(str(args.port)):
                app.port_var.set(value)
                break
        root.after(200, app.connect)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
