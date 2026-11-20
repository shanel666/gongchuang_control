# -*- coding: utf-8 -*-
'''
升降步进电机标定上位机（无线串口 + GUI 按钮）

=================================================================
用来干什么
=================================================================
通过无线串口（F103 的 USART3 = PB10/PB11，115200 8N1）遥控升降电机，
一边点按钮一边看驱动器回传的 CMD / ACT，把四个高度标出来：

    HIGH   让位高度（旋转的时候用）
    GRAB   车上的物料高度（降下去夹的高度）
    CLEAR  提料高度（夹住后抬一点）
    PLACE  放料高度

=================================================================
协议（F103 端 DEBUG_MODE = 13 实现）
=================================================================
    U<step>   上升 step 脉冲        例 U100
    D<step>   下降 step 脉冲        例 D100
    G<pos>    走到绝对位置 pos      例 G1500
    Z         回零（绝对 0）
    W         把当前位置设为驱动器零点（0x93 0x88）   ★ 对零用
    P         抓 0x36 的原始回包（判断驱动器答不答）  ★ 查通信
    E         分段下降到 0（每段 500，找堵转高度）    ★ 查堵转
    V<rpm>    运行期改速度（0 = 恢复默认 升40/降20）
    A<accel>  运行期改加速度档（0 = 恢复默认 80）
    S         查询状态 → 回 STATUS
    X         急停
    C         清故障锁
    H         帮助

=================================================================
遇到"下降总是多转、越走越低"怎么办
=================================================================
    1) 先点「原始包 P」：回包正常是 06 36 00 xx xx xx xx 6B
       全 00 / RxFlag=0  → 驱动器不答 0x36，位置完全读不到（这就是漂的根源）
    2) 对零：用 ↓ 一点点降到要当 0 的位置 → 点「设零点 W」
    3) 降速试试：速度填 10 → 点「发 V」，再走几次下降看还过不过零
    4) 想找堵转高度：点「分段下降到 0 E」，最后打印的那一段就是卡住的位置

F103 会这样回（本程序就是解析这些行）：
    [LIFT] HOST> U100
    [LIFT] UP  0 -> 100  (expect 460 ms)
    [LIFT] RAW 06 36 00 00 00 00 64 6B  -> OK act=100 cmd=100 d=0
    [LIFT] NOW cmd=100 act=100 d=0 step=100 OK

=================================================================
怎么跑
=================================================================
    python tools/lift_host.py
    或者双击 tools/run_lift_host.bat（会自动装 pyserial）

依赖：pyserial（GUI 用标准库 tkinter，不需要额外装）
    pip install pyserial

=================================================================
读数的意思（重点看这三个）
=================================================================
    CMD = 单片机"以为"的位置（我们自己维护的）
    ACT = 驱动器实际回报的位置（0x36 读回）
    Δ   = ACT − CMD
        Δ 一直是 0        → 绝对定位正常
        Δ 越跑越大        → 驱动器没在做绝对定位，位置在漂
        ACT 出现负数      → 驱动器认为自己在零点下面（你说的"降过多"）
        ACT 显示 ----     → 驱动器没回 0x36（地址 6 通信问题）
=================================================================
'''

import os
import sys
import time
import queue
import threading
import traceback

try:
    import tkinter as tk
    from tkinter import ttk, filedialog, messagebox
    HAS_TK = True
except Exception:
    HAS_TK = False

BAUD_DEFAULT = 115200

# ==================================================================
# 纯逻辑部分（不碰硬件，电脑上可以直接单测）
# ==================================================================

LOG_PREFIX = "[LIFT]"


def build_cmd(action, value=None):
    """把界面动作翻译成 F103 认识的一行命令（不含换行）。

    action: "up" / "down" / "zero" / "read" / "stop" / "help" / "go"
    value : 脉冲数（up/down/go 用）
    """
    if action in ("up", "down"):
        v = 100 if value is None else int(value)
        v = max(1, min(5000, v))
        return ("U%d" if action == "up" else "D%d") % v

    if action == "go":
        v = 0 if value is None else int(value)
        v = max(0, min(999999, v))
        return "G%d" % v

    if action == "rel":
        # 相对帧测试：value 可正可负
        v = 0 if value is None else int(value)
        v = max(-5000, min(5000, v))
        if v == 0:
            raise ValueError("相对移动不能是 0")
        return "R%d" % v

    if action == "zero":
        return "Z"
    if action == "read":
        return "S"
    if action == "stop":
        return "X"
    if action == "clear":
        return "C"
    if action == "help":
        return "H"

    # ---- 2026-11-21 新增（模式 13 的调试台）----
    if action == "setzero":
        return "W"          # 把当前位置设为驱动器零点
    if action == "raw":
        return "P"          # 抓 0x36 的原始回包
    if action == "seg":
        return "E"          # 分段下降到 0（找堵转高度）

    if action == "speed":
        v = 0 if value is None else int(value)
        v = max(0, min(60, v))
        return "V%d" % v    # 0 = 取消覆盖，恢复默认（升 40 / 降 20）

    if action == "accel":
        v = 0 if value is None else int(value)
        v = max(0, min(255, v))
        return "A%d" % v    # 0 = 恢复默认 LIFT_MOTOR_ACCEL

    raise ValueError("unknown action: %r" % (action,))


import re

RE_NOW = re.compile(r"NOW\s+cmd=(\d+)\s+act=(-?\d+)\s+d=(-?\d+)\s+step=(\d+)\s+(OK|NO-ANSWER)")
RE_STATUS = re.compile(r"STATUS\s+cmd=(\d+)\s+act=(-?\d+)\s+d=(-?\d+)\s+step=(\d+)\s+max=(\d+)\s+(OK|NO-ANSWER)")
RE_OK = re.compile(r"->\s+OK\s+act=(-?\d+)\s+cmd=(\d+)\s+d=(-?\d+)")
RE_BOOT = re.compile(r"上电实际位置\s*=\s*(-?\d+)")
RE_STEP = re.compile(r"步长\s*->\s*(\d+)")


def parse_line(text):
    """解析一行串口回传，返回 dict。认不出来返回 {}。

    返回的键（有就填）：
        kind : "now" / "status" / "ok" / "boot" / "step" / "noanswer" / "abort"
        cmd, act, d, step, maxv, ok(bool)
    """
    if not text:
        return {}

    t = text.strip()

    m = RE_NOW.search(t)
    if m:
        return {
            "kind": "now",
            "cmd": int(m.group(1)),
            "act": int(m.group(2)),
            "d": int(m.group(3)),
            "step": int(m.group(4)),
            "ok": m.group(5) == "OK",
        }

    m = RE_STATUS.search(t)
    if m:
        return {
            "kind": "status",
            "cmd": int(m.group(1)),
            "act": int(m.group(2)),
            "d": int(m.group(3)),
            "step": int(m.group(4)),
            "maxv": int(m.group(5)),
            "ok": m.group(6) == "OK",
        }

    m = RE_OK.search(t)
    if m:
        return {
            "kind": "ok",
            "act": int(m.group(1)),
            "cmd": int(m.group(2)),
            "d": int(m.group(3)),
            "ok": True,
        }

    if "-> NO ANSWER" in t:
        return {"kind": "noanswer", "ok": False}

    # 位置不符 / 故障锁定：这是"下降一路到底"的拦截提示，界面要变红
    if "位置不符" in t or "FAULT" in t:
        return {"kind": "fault", "ok": False}

    if "ABORT" in t:
        return {"kind": "abort", "ok": False}

    m = RE_BOOT.search(t)
    if m:
        return {"kind": "boot", "act": int(m.group(1)), "ok": True}

    m = RE_STEP.search(t)
    if m:
        return {"kind": "step", "step": int(m.group(1))}

    return {}


def fmt_signed(v):
    return "%+d" % (int(v),)


def describe_delta(d, ok):
    """把 Δ 翻译成一句人话，直接显示在界面上。"""
    if not ok:
        return "驱动器无应答"
    if abs(d) <= 2:
        return "跟踪正常"
    if d > 0:
        return "实际比命令高 %d（命令发小了/还在走）" % d
    return "实际比命令低 %d（下降过头/绝对定位没生效）" % (-d,)


# ==================================================================
# GUI
# ==================================================================

class LiftApp(object):
    def __init__(self, root):
        self.root = root
        self.ser = None
        self.rx_q = queue.Queue()
        self.rx_thread = None
        self.rx_stop = threading.Event()
        self.log_lines = []

        self.busy_until = 0.0

        self.cmd = 0
        self.act = 0
        self.delta = 0
        self.have_act = False

        root.title("升降电机标定上位机 —— 无线串口")
        root.geometry("760x620")
        root.minsize(720, 560)

        self._build_ui()
        self._refresh_ports()
        self.root.after(50, self._drain_queue)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------- 界面 ----------------
    def _build_ui(self):
        pad = dict(padx=6, pady=4)

        # ===== 串口行 =====
        top = ttk.LabelFrame(self.root, text="串口（无线模块接 F103 的 PB10/PB11）")
        top.pack(fill="x", **pad)

        ttk.Label(top, text="端口").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        self.cb_port = ttk.Combobox(top, width=14, state="readonly", values=[])
        self.cb_port.grid(row=0, column=1, padx=4, pady=4)

        ttk.Button(top, text="刷新", width=6, command=self._refresh_ports).grid(row=0, column=2, padx=4)
        self.btn_conn = ttk.Button(top, text="连接", width=8, command=self._toggle_conn)
        self.btn_conn.grid(row=0, column=3, padx=4)

        ttk.Label(top, text="波特率").grid(row=0, column=4, sticky="e", padx=(12, 2))
        self.cb_baud = ttk.Combobox(top, width=8, state="readonly",
                                    values=["9600", "57600", "115200", "230400", "460800"])
        self.cb_baud.set(str(BAUD_DEFAULT))
        self.cb_baud.grid(row=0, column=5, padx=4)

        self.lbl_conn = ttk.Label(top, text="未连接", foreground="#a00")
        self.lbl_conn.grid(row=0, column=6, padx=8)

        # ===== 读数大字 =====
        mid = ttk.LabelFrame(self.root, text="位置（CMD=单片机以为的 / ACT=驱动器实际回报的）")
        mid.pack(fill="x", **pad)

        f = ttk.Frame(mid)
        f.pack(fill="x", padx=6, pady=6)

        def big(parent, title, col):
            ttk.Label(parent, text=title, font=("Consolas", 10)).grid(row=0, column=col, padx=10)
            lb = ttk.Label(parent, text="--", font=("Consolas", 22, "bold"), width=8, anchor="center")
            lb.grid(row=1, column=col, padx=10)
            return lb

        self.lb_cmd = big(f, "CMD", 0)
        self.lb_act = big(f, "ACT", 1)
        self.lb_d = big(f, "Δ", 2)

        # 用经典 tk.Label 而不是 ttk.Label：故障时要能直接改背景色变红
        self.lbl_state = tk.Label(mid, text="等待连接", font=("Microsoft YaHei", 10),
                                  anchor="w", bg="#f0f0f0", fg="#333", padx=8, pady=5)
        self.lbl_state.pack(fill="x", padx=8, pady=(0, 6))

        # ===== 控制 =====
        ctl = ttk.LabelFrame(self.root, text="控制")
        ctl.pack(fill="x", **pad)

        row1 = ttk.Frame(ctl)
        row1.pack(fill="x", padx=6, pady=(6, 2))
        ttk.Label(row1, text="步长（脉冲）").pack(side="left")
        self.var_step = tk.StringVar(value="100")
        e = ttk.Entry(row1, textvariable=self.var_step, width=8)
        e.pack(side="left", padx=6)
        for v in (10, 50, 100, 500, 1000):
            ttk.Button(row1, text=str(v), width=5,
                       command=lambda x=v: self.var_step.set(str(x))).pack(side="left", padx=2)

        row2 = ttk.Frame(ctl)
        row2.pack(fill="x", padx=6, pady=4)

        # ★ 一次点击 = 一条命令，不做按住连发。
        #   按住连发在这里非常危险：手指不离键会连续发几十条，升降会一路走到底。
        self.btn_up = ttk.Button(row2, text="↑  上升",
                                 command=lambda: self._send_action("up"))
        self.btn_up.pack(side="left", padx=4)

        self.btn_down = ttk.Button(row2, text="↓  下降",
                                   command=lambda: self._send_action("down"))
        self.btn_down.pack(side="left", padx=4)

        self.btn_zero = ttk.Button(row2, text="回零 Z", width=10,
                                   command=lambda: self._send_action("zero"))
        self.btn_zero.pack(side="left", padx=4)

        self.btn_read = ttk.Button(row2, text="读位置 S", width=10,
                                   command=lambda: self._send_action("read"))
        self.btn_read.pack(side="left", padx=4)

        self.btn_stop = ttk.Button(row2, text="急停 X", width=10,
                                   command=lambda: self._send_action("stop"))
        self.btn_stop.pack(side="left", padx=4)

        self.btn_clear_fault = ttk.Button(row2, text="清故障 C", width=10,
                                          command=lambda: self._send_action("clear"))
        self.btn_clear_fault.pack(side="left", padx=4)

        # ===== 标定预设（实测出的三个关键高度） =====
        pre = ttk.LabelFrame(ctl, text="标定预设（一键走到绝对位置）")
        pre.pack(fill="x", padx=6, pady=4)

        pr1 = ttk.Frame(pre)
        pr1.pack(fill="x", padx=4, pady=(4, 2))
        for label, pos in (("0  零位", 0), ("4000 抓料", 4000), ("7600 让位", 7600)):
            ttk.Button(pr1, text=label, width=13,
                       command=lambda p=pos: self._send_line(build_cmd("go", p))
                       ).pack(side="left", padx=4)

        pr2 = ttk.Frame(pre)
        pr2.pack(fill="x", padx=4, pady=(2, 5))
        ttk.Label(pr2, text="自定义").pack(side="left")
        self.var_go = tk.StringVar(value="7600")
        ttk.Entry(pr2, textvariable=self.var_go, width=9).pack(side="left", padx=6)
        self.btn_go = ttk.Button(pr2, text="执行 G", width=10, command=self._send_go)
        self.btn_go.pack(side="left", padx=4)

        ttk.Label(pr2, text="   相对测试 R").pack(side="left")
        self.var_rel = tk.StringVar(value="1000")
        ttk.Entry(pr2, textvariable=self.var_rel, width=8).pack(side="left", padx=4)
        self.btn_rel = ttk.Button(pr2, text="R", width=4, command=self._send_rel)
        self.btn_rel.pack(side="left", padx=2)
        ttk.Label(pr2, text="（判断驱动器认不认绝对定位）",
                  foreground="#666").pack(side="left", padx=4)

        # ===== 对零 / 抓包 / 分段下降 / 速度加速度（2026-11-21 新增）=====
        pr3 = ttk.Frame(pre)
        pr3.pack(fill="x", padx=4, pady=(0, 6))

        self.btn_setzero = ttk.Button(pr3, text="设零点 W", width=10,
                                      command=lambda: self._send_action("setzero"))
        self.btn_setzero.pack(side="left", padx=4)
        ttk.Label(pr3, text="（降到要当 0 的位置再点）",
                  foreground="#666").pack(side="left", padx=2)

        self.btn_raw = ttk.Button(pr3, text="原始包 P", width=10,
                                  command=lambda: self._send_action("raw"))
        self.btn_raw.pack(side="left", padx=(10, 4))

        self.btn_seg = ttk.Button(pr3, text="分段下降到 0 E", width=14,
                                  command=lambda: self._send_action("seg"))
        self.btn_seg.pack(side="left", padx=(10, 4))

        pr4 = ttk.Frame(pre)
        pr4.pack(fill="x", padx=4, pady=(0, 6))

        ttk.Label(pr4, text="速度 RPM（空/0 = 默认升40降20）").pack(side="left")
        self.var_spd = tk.StringVar(value="")
        ttk.Entry(pr4, textvariable=self.var_spd, width=6).pack(side="left", padx=4)
        self.btn_spd = ttk.Button(pr4, text="发 V", width=6, command=self._send_speed)
        self.btn_spd.pack(side="left", padx=2)

        ttk.Label(pr4, text="    加速度档（空/0 = 默认80）").pack(side="left")
        self.var_acc = tk.StringVar(value="")
        ttk.Entry(pr4, textvariable=self.var_acc, width=6).pack(side="left", padx=4)
        self.btn_acc = ttk.Button(pr4, text="发 A", width=6, command=self._send_accel)
        self.btn_acc.pack(side="left", padx=2)

        ttk.Label(pr4, text="（改完立即对下一条移动生效，掉电恢复）",
                  foreground="#666").pack(side="left", padx=6)

        row4 = ttk.Frame(ctl)
        row4.pack(fill="x", padx=6, pady=(0, 6))
        ttk.Button(row4, text="帮助 H", width=10,
                   command=lambda: self._send_action("help")).pack(side="left", padx=4)
        self.btn_clear = ttk.Button(row4, text="清空日志", width=10, command=self._clear_log)
        self.btn_clear.pack(side="left", padx=4)

        # ===== 日志 =====
        logf = ttk.LabelFrame(self.root, text="串口回传")
        logf.pack(fill="both", expand=True, **pad)

        wrap = ttk.Frame(logf)
        wrap.pack(fill="both", expand=True, padx=6, pady=6)
        self.txt = tk.Text(wrap, height=14, wrap="none", font=("Consolas", 9), bg="#111", fg="#ddd",
                           insertbackground="#ddd")
        ys = ttk.Scrollbar(wrap, orient="vertical", command=self.txt.yview)
        xs = ttk.Scrollbar(wrap, orient="horizontal", command=self.txt.xview)
        self.txt.configure(yscrollcommand=ys.set, xscrollcommand=xs.set)
        self.txt.grid(row=0, column=0, sticky="nsew")
        ys.grid(row=0, column=1, sticky="ns")
        xs.grid(row=1, column=0, sticky="ew")
        wrap.rowconfigure(0, weight=1)
        wrap.columnconfigure(0, weight=1)

        bar = ttk.Frame(logf)
        bar.pack(fill="x", padx=6, pady=(0, 6))
        ttk.Button(bar, text="复制最近 50 行", command=self._copy_recent).pack(side="left")
        ttk.Button(bar, text="保存日志…", command=self._save_log).pack(side="left", padx=6)
        ttk.Label(bar, text="（把关键几行贴给我就能标定）", foreground="#666").pack(side="left", padx=8)

        self._set_enabled(False)

    # ---------------- 串口 ----------------
    def _refresh_ports(self):
        ports = []
        try:
            import serial.tools.list_ports as lp
            for p in lp.comports():
                ports.append(p.device)
        except Exception:
            ports = []
        self.cb_port["values"] = ports
        if ports and not self.cb_port.get():
            self.cb_port.set(ports[0])
        if not ports:
            self._append("（没找到串口。检查 USB-TTL / 无线模块是否插好，然后再点『刷新』）")

    def _toggle_conn(self):
        if self.ser is not None:
            self._disconnect()
            return
        try:
            import serial
        except Exception:
            messagebox.showerror(
                "缺少 pyserial",
                "没有安装 pyserial。\n\n在命令行执行：\n    pip install pyserial\n\n"
                "或者双击 tools/run_lift_host.bat，它会自动装。")
            return

        port = self.cb_port.get().strip()
        if not port:
            messagebox.showwarning("没选端口", "先选一个 COM 口（点『刷新』）")
            return

        try:
            baud = int(self.cb_baud.get())
        except Exception:
            baud = BAUD_DEFAULT

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
        except Exception as e:
            self.ser = None
            messagebox.showerror("打不开串口", "%s\n\n%s" % (port, e))
            return

        self.rx_stop.clear()
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

        self.lbl_conn.configure(text="已连接 %s" % port, foreground="#070")
        self.btn_conn.configure(text="断开")
        self._set_enabled(True)
        self._append("=== 已连接 %s @ %d ===" % (port, baud))
        self._send_line("H")          # 先要一次帮助，确认链路通

    def _disconnect(self):
        self.rx_stop.set()
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.lbl_conn.configure(text="未连接", foreground="#a00")
        self.btn_conn.configure(text="连接")
        self._set_enabled(False)
        self._append("=== 已断开 ===")

    def _rx_loop(self):
        while not self.rx_stop.is_set():
            try:
                raw = self.ser.readline()
            except Exception:
                if not self.rx_stop.is_set():
                    self.rx_q.put("!!! 读串口异常，已断开")
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                return
            if not raw:
                continue
            text = raw.decode("utf-8", "replace").rstrip("\r\n")
            if text:
                self.rx_q.put(text)

    # ---------------- 发送 ----------------
    def _send_line(self, line):
        if self.ser is None:
            self._append("（未连接，命令没发出去：%s）" % line)
            return False
        data = (line + "\r\n").encode("ascii", "ignore")
        try:
            self.ser.write(data)
        except Exception as e:
            self._append("!!! 发送失败：%s" % e)
            return False
        self._append(">>> %s" % line)
        self.busy_until = time.time() + 12.0
        return True

    def _send_action(self, action):
        if action in ("up", "down"):
            try:
                v = int(self.var_step.get())
            except Exception:
                messagebox.showwarning("步长不对", "步长要填整数，例如 100")
                return
            self._send_line(build_cmd(action, v))
        else:
            self._send_line(build_cmd(action))

    def _send_go(self):
        try:
            v = int(self.var_go.get())
        except Exception:
            messagebox.showwarning("目标不对", "目标位置要填整数，例如 7600")
            return
        self._send_line(build_cmd("go", v))

    def _send_rel(self):
        try:
            v = int(self.var_rel.get())
        except Exception:
            messagebox.showwarning("相对量不对", "要填整数，例如 1000 或 -1000")
            return
        try:
            self._send_line(build_cmd("rel", v))
        except ValueError:
            messagebox.showwarning("相对量不对", "不能是 0")

    def _send_speed(self):
        try:
            v = int(self.var_spd.get().strip() or "0")
        except Exception:
            messagebox.showwarning("速度不对", "速度要填整数 RPM（0 = 恢复默认）")
            return
        self._send_line(build_cmd("speed", v))

    def _send_accel(self):
        try:
            v = int(self.var_acc.get().strip() or "0")
        except Exception:
            messagebox.showwarning("加速度不对", "加速度要填整数档位（0 = 恢复默认）")
            return
        self._send_line(build_cmd("accel", v))

    # ---------------- 接收队列 ----------------
    def _drain_queue(self):
        got = 0
        while got < 200:
            try:
                line = self.rx_q.get_nowait()
            except queue.Empty:
                break
            got += 1
            self._append(line)
            self._handle_line(line)
        self.root.after(50, self._drain_queue)

    def _handle_line(self, line):
        info = parse_line(line)
        if not info:
            return

        kind = info.get("kind")

        if kind in ("now", "status", "ok", "boot"):
            if "act" in info:
                self.act = info["act"]
                self.have_act = info.get("ok", True)
            if "cmd" in info:
                self.cmd = info["cmd"]
            if "d" in info:
                self.delta = info["d"]
            else:
                self.delta = self.act - self.cmd
            self._refresh_numbers()

        if kind == "step" and "step" in info:
            self.var_step.set(str(info["step"]))

        if kind == "fault":
            self.lbl_state.configure(
                text="★ 位置不符！驱动器没停在目标上，已急停锁定 —— 点『清故障 C』或『回零 Z』",
                bg="#c00000", fg="#ffffff")
            self.busy_until = 0.0
            return

        if kind in ("now", "status", "ok", "noanswer", "abort"):
            self.busy_until = 0.0      # 这一轮动作结束了

    def _refresh_numbers(self):
        self.lb_cmd.configure(text="%d" % self.cmd)

        if self.have_act:
            self.lb_act.configure(text=fmt_signed(self.act),
                                  foreground="#070" if abs(self.delta) <= 2 else "#c60")
            self.lb_d.configure(text=fmt_signed(self.delta),
                                foreground="#070" if abs(self.delta) <= 2 else "#a00")
        else:
            self.lb_act.configure(text="----", foreground="#a00")
            self.lb_d.configure(text="--", foreground="#a00")

        hint = describe_delta(self.delta, self.have_act)
        bad = (not self.have_act) or (abs(self.delta) > 2)
        if self.have_act and self.act < 0:
            hint = "★ ACT 是负数：驱动器在零点下面（就是『降过头』）"
            bad = True

        self.lbl_state.configure(text=hint,
                                 bg="#fff3cd" if bad else "#e8f5e9",
                                 fg="#8a6d00" if bad else "#1b5e20")

    # ---------------- 日志 ----------------
    def _append(self, text):
        stamp = time.strftime("%H:%M:%S")
        line = "%s  %s" % (stamp, text)
        self.log_lines.append(line)
        self.txt.configure(state="normal")
        self.txt.insert("end", line + "\n")
        self.txt.see("end")
        self.txt.configure(state="disabled")

    def _clear_log(self):
        self.log_lines = []
        self.txt.configure(state="normal")
        self.txt.delete("1.0", "end")
        self.txt.configure(state="disabled")

    def _copy_recent(self):
        tail = self.log_lines[-50:]
        self.root.clipboard_clear()
        self.root.clipboard_append("\n".join(tail))
        self._append("（已复制最近 %d 行到剪贴板）" % len(tail))

    def _save_log(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("文本", "*.txt"), ("全部", "*.*")],
            initialfile="lift_log_%s.txt" % time.strftime("%H%M%S"))
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write("\n".join(self.log_lines))
            self._append("（日志已保存：%s）" % path)
        except Exception as e:
            messagebox.showerror("保存失败", str(e))

    # ---------------- 其它 ----------------
    def _set_enabled(self, on):
        state = "normal" if on else "disabled"
        for b in (self.btn_up, self.btn_down, self.btn_zero, self.btn_read,
                  self.btn_stop, self.btn_go, self.btn_rel, self.btn_clear_fault,
                  self.btn_setzero, self.btn_raw, self.btn_seg, self.btn_spd, self.btn_acc):
            try:
                b.configure(state=state)
            except Exception:
                pass

    def _on_close(self):
        if self.ser is not None:
            self._disconnect()
        self.root.destroy()


# ==================================================================
# 自检（不用硬件）
# ==================================================================

def selftest():
    fails = []
    total = [0]

    def check(name, got, want):
        total[0] += 1
        if got != want:
            fails.append("%s: got %r, want %r" % (name, got, want))

    check("up", build_cmd("up", 100), "U100")
    check("down", build_cmd("down", 50), "D50")
    check("up clamp lo", build_cmd("up", 0), "U1")
    check("up clamp hi", build_cmd("up", 99999), "U5000")
    check("zero", build_cmd("zero"), "Z")
    check("read", build_cmd("read"), "S")
    check("stop", build_cmd("stop"), "X")
    check("go", build_cmd("go", 1500), "G1500")
    check("go 0", build_cmd("go", 0), "G0")

    # 2026-11-21 新增的调试台命令
    check("setzero", build_cmd("setzero"), "W")
    check("raw", build_cmd("raw"), "P")
    check("seg", build_cmd("seg"), "E")
    check("speed", build_cmd("speed", 12), "V12")
    check("speed 0", build_cmd("speed", 0), "V0")
    check("speed clamp", build_cmd("speed", 999), "V60")
    check("accel", build_cmd("accel", 40), "A40")
    check("accel clamp", build_cmd("accel", 999), "A255")

    i = parse_line("[LIFT] NOW cmd=100 act=100 d=0 step=100 OK")
    check("now kind", i.get("kind"), "now")
    check("now cmd", i.get("cmd"), 100)
    check("now act", i.get("act"), 100)
    check("now d", i.get("d"), 0)
    check("now ok", i.get("ok"), True)

    i = parse_line("[LIFT] NOW cmd=200 act=-50 d=-250 step=100 NO-ANSWER")
    check("now neg act", i.get("act"), -50)
    check("now noanswer", i.get("ok"), False)

    i = parse_line("[LIFT] RAW 06 36 00 00 00 00 64 6B  -> OK act=100 cmd=100 d=0")
    check("ok kind", i.get("kind"), "ok")
    check("ok act", i.get("act"), 100)

    i = parse_line("[LIFT] RAW 06 36 00 00 00 00 64 6B  -> NO ANSWER (驱动器没回 0x36)")
    check("noanswer", i.get("kind"), "noanswer")

    i = parse_line("[LIFT] 上电实际位置 = 0 脉冲（驱动器零点参考）")
    check("boot", i.get("kind"), "boot")
    check("boot act", i.get("act"), 0)

    i = parse_line("[LIFT] 步长 -> 500 脉冲")
    check("step", i.get("step"), 500)

    i = parse_line("[LIFT] STATUS cmd=1500 act=1498 d=-2 step=100 max=12000 OK")
    check("status kind", i.get("kind"), "status")
    check("status max", i.get("maxv"), 12000)
    check("status d", i.get("d"), -2)

    check("unknown", parse_line("[LIFT] HOST> U100"), {})
    check("empty", parse_line(""), {})

    check("describe ok", "跟踪正常" in describe_delta(0, True), True)
    check("describe neg", "下降过头" in describe_delta(-30, True), True)
    check("describe no-answer", describe_delta(0, False), "驱动器无应答")

    if fails:
        print("SELFTEST FAILED:")
        for f in fails:
            print("  -", f)
        return 1

    print("SELFTEST OK（%d 项）" % total[0])
    return 0


def main():
    if len(sys.argv) > 1 and sys.argv[1] in ("--selftest", "-t"):
        return selftest()

    if not HAS_TK:
        print("没有 tkinter，跑不了 GUI。")
        print("Windows 官方 Python 自带 tkinter；如果是精简版请重装 Python 并勾选 tcl/tk。")
        return 1

    root = tk.Tk()
    try:
        root.call("tk", "scaling", 1.2)
    except Exception:
        pass
    LiftApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        traceback.print_exc()
        try:
            input("按回车退出…")
        except Exception:
            pass
        sys.exit(1)
