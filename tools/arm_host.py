# -*- coding: utf-8 -*-
'''
取放流程上位机（无线串口）

=================================================================
对应固件：User/main.c  DEBUG_MODE = 12  (DEBUG_MODE_CAR_PUT)
串口：F103 的 USART3 = PB10/PB11，115200 8N1（无线模块）
=================================================================
★ 板子上现在是「半自动」：按一次 PC3 就自动跑到视觉那步停下来等，
  再按一次 PC3 就自动夹取放料。这个上位机主要用来**单独点某一步**调试。

正向 11 步（固件 k_cp_steps[]，下标 0 = 第 1 步）：
     1 UP7600   升降步进 → 7600   抬到高位
     2 CIRCLE   K230 圆心定位      底盘自动修正，等你按 PC3
     3 ROT163   旋转舵机 → 163°   转到车上料仓
     4 LFT130   升降舵机 → 130°   抬臂姿态
     5 DN1000   升降步进 → 1000   降到车上料（带下降补偿）
     6 GRP110   夹爪 → 110°       夹住物料
     7 UP7600B  升降步进 → 7600   夹住后先抬起来
     8 ROT30    旋转舵机 → 30°    回放料侧
     9 LFT90    升降舵机 → 90°    回初始
    10 DN0      升降步进 → 0      降到地上（带下降补偿）
    11 GRP85    夹爪 → 85°        松开＝放下（不压栈）

（固件里 CP_UP_BEFORE_BACK = 0 时没有第 7 步，那时这里是 10 步、后面全部前移一位。）

「连跑 A」走完正向之后会按栈逆序恢复；板子的 PC3/PB1 半自动做完**不回退**，
停在地上那个姿态。

=================================================================
命令（F103 端实现）
=================================================================
    N   下一步
    A   连跑（正向全做完 + 自动弹栈恢复）
    B   回退一步（弹栈）
    R   复位到第 1 步（不动硬件）
    S   查询状态
    T   下降探针（往下走 200 脉冲，读回 ACT，判定下降停不停得住）
    D<n> 下降补偿（脉冲，越大停得越高；D0 关闭）
    K<n>  只执行第 n 个动作（界面上点步骤列表的一行就发这个）
    X   急停

F103 回：
    [CARPUT]   -> UP7600   MOT=7600 ROT=30 GRIP=85 LIFT=130  (fwd=1 undo=0/0)
    [CARPUT] STATUS phase=1 fwd=3/11 undo=0/0 MOT=1000 ROT=30 GRIP=110 LIFT=130
=================================================================
'''

import os
import sys
import re
import time
import queue
import threading
import traceback

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
    HAS_TK = True
except Exception:
    HAS_TK = False

BAUD_DEFAULT = 115200

# 正向 11 步（下标 0 对应第 1 步）—— 与固件 k_cp_steps[] 一一对应
STEPS = [
    ("UP7600",  "升降步进 → 7600", "先把臂抬起来"),
    ("CIRCLE",  "K230 圆心定位", "对准地面圆环★"),
    ("ROT163",  "旋转舵机 → 163°", "转到车上料仓"),
    ("LFT130",  "升降舵机 → 130°", "抬臂姿态"),
    ("DN1000",  "升降步进 → 1000", "降到车上料"),
    ("GRP110",  "夹爪 → 110°", "夹住物料"),
    ("UP7600B", "升降步进 → 7600", "夹住后抬起来"),
    ("ROT30",   "旋转舵机 → 30°", "回放料侧"),
    ("LFT90",   "升降舵机 → 90°", "回初始"),
    ("DN0",     "升降步进 → 0", "降到地上"),
    ("GRP85",   "夹爪 → 85°", "松开＝放下"),
]
STEP_TOTAL = len(STEPS)

# ==================================================================
# 纯逻辑（可单测）
# ==================================================================

def build_cmd(action, arg=None):
    table = {
        "next": "N",
        "all": "A",
        "back": "B",
        "reset": "R",
        "status": "S",
        "probe": "T",
        "vision": "V",
        "stop": "X",
        "help": "H",
    }
    if action == "do":
        n = int(arg)
        if n < 1:
            raise ValueError("do 要 >= 1")
        return "K%d" % n
    if action not in table:
        raise ValueError("unknown action: %r" % (action,))
    return table[action]


RE_STEP = re.compile(
    r"->\s+(\S+)\s+MOT=(\d+)\s+ROT=(\d+)\s+GRIP=(\d+)\s+LIFT=(\d+)\s+\(fwd=(\d+)\s+undo=(\d+)/(\d+)\)")
RE_STATUS = re.compile(
    r"STATUS\s+phase=(\d+)\s+fwd=(\d+)/(\d+)\s+undo=(\d+)/(\d+)\s+MOT=(\d+)\s+ROT=(\d+)\s+GRIP=(\d+)\s+LIFT=(\d+)")


def parse_line(text):
    """解析一行 [CARPUT] 回传，返回 dict。认不出来返回 {}。

    返回的键：
        mot       升降步进电机的绝对位置（脉冲）
        rot       旋转舵机角度
        grip      夹爪舵机角度
        lift      升降舵机角度（注意：不是步进电机！）
    """
    if not text:
        return {}
    t = text.strip()

    m = RE_STATUS.search(t)
    if m:
        return {
            "kind": "status",
            "phase": int(m.group(1)),
            "fwd": int(m.group(2)),
            "total": int(m.group(3)),
            "undo_done": int(m.group(4)),
            "undo_total": int(m.group(5)),
            "mot": int(m.group(6)),
            "rot": int(m.group(7)),
            "grip": int(m.group(8)),
            "lift": int(m.group(9)),
        }

    m = RE_STEP.search(t)
    if m:
        return {
            "kind": "step",
            "label": m.group(1),
            "mot": int(m.group(2)),
            "rot": int(m.group(3)),
            "grip": int(m.group(4)),
            "lift": int(m.group(5)),
            "fwd": int(m.group(6)),
            "undo_done": int(m.group(7)),
            "undo_total": int(m.group(8)),
        }

    if "已按栈恢复完" in t:
        return {"kind": "home"}
    if "栈是空的" in t:
        return {"kind": "stackempty"}
    if "急停" in t:
        return {"kind": "stop"}
    if "RUN 完成" in t:
        return {"kind": "run_done"}

    return {}


def phase_name(phase):
    return {0: "空闲", 1: "正向", 2: "恢复", 3: "完成"}.get(phase, "?")


def step_index_from(fwd, total=None):
    """fwd = 已经做完几步 → 当前应该做第几步（1 基）。完成时返回 total。"""
    if total is None:
        total = STEP_TOTAL
    return max(1, min(total, fwd + 1))


# ==================================================================
# GUI
# ==================================================================

class ArmApp(object):
    def __init__(self, root):
        self.root = root
        self.ser = None
        self.rx_q = queue.Queue()
        self.rx_thread = None
        self.rx_stop = threading.Event()
        self.log_lines = []
        self.busy_until = 0.0

        self.fwd = 0
        self.phase = 0
        self.mot = 0
        self.rot = 0
        self.grip = 0
        self.lift = 0          # 升降舵机（不是步进电机）

        root.title("取放流程单步调试 —— 无线串口")
        root.geometry("1000x720")
        root.minsize(960, 660)

        self._build_ui()
        self._refresh_ports()
        self.root.after(50, self._drain_queue)
        root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------- 界面 ----------------
    def _build_ui(self):
        pad = dict(padx=6, pady=4)

        top = ttk.LabelFrame(self.root, text="串口（无线模块接 F103 的 PB10/PB11）")
        top.pack(fill="x", **pad)
        ttk.Label(top, text="端口").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        self.cb_port = ttk.Combobox(top, width=14, state="readonly", values=[])
        self.cb_port.grid(row=0, column=1, padx=4)
        ttk.Button(top, text="刷新", width=6, command=self._refresh_ports).grid(row=0, column=2, padx=4)
        self.btn_conn = ttk.Button(top, text="连接", width=8, command=self._toggle_conn)
        self.btn_conn.grid(row=0, column=3, padx=4)
        ttk.Label(top, text="波特率").grid(row=0, column=4, padx=(12, 2))
        self.cb_baud = ttk.Combobox(top, width=8, state="readonly",
                                    values=["115200", "230400", "460800", "57600"])
        self.cb_baud.set(str(BAUD_DEFAULT))
        self.cb_baud.grid(row=0, column=5, padx=4)
        self.lbl_conn = ttk.Label(top, text="未连接", foreground="#a00")
        self.lbl_conn.grid(row=0, column=6, padx=8)

        # ===== 状态大字 =====
        mid = ttk.LabelFrame(self.root, text="当前状态")
        mid.pack(fill="x", **pad)

        f = ttk.Frame(mid)
        f.pack(fill="x", padx=6, pady=6)

        def big(parent, title, col):
            ttk.Label(parent, text=title, font=("Consolas", 10)).grid(row=0, column=col, padx=12)
            lb = ttk.Label(parent, text="--", font=("Consolas", 20, "bold"), width=7, anchor="center")
            lb.grid(row=1, column=col, padx=12)
            return lb

        self.lb_phase = big(f, "阶段", 0)
        self.lb_fwd = big(f, "步", 1)
        self.lb_mot = big(f, "步进 MOT", 2)
        self.lb_rot = big(f, "旋转", 3)
        self.lb_grip = big(f, "夹爪", 4)
        self.lb_lift = big(f, "升降舵", 5)

        self.lbl_msg = tk.Label(mid, text="等待连接", font=("Microsoft YaHei", 10),
                                anchor="w", bg="#f0f0f0", fg="#333", padx=8, pady=5)
        self.lbl_msg.pack(fill="x", padx=8, pady=(0, 6))

        # ===== 步骤清单（每一行都是一个按钮：点一行 = 直接做那一个动作）=====
        sl = ttk.LabelFrame(self.root, text="流程 —— 点任意一行，小车就只做那一个动作")
        sl.pack(fill="x", **pad)
        grid = ttk.Frame(sl)
        grid.pack(fill="x", padx=6, pady=6)

        self.step_labels = []
        for i, (tag, what, note) in enumerate(STEPS):
            b = tk.Button(grid,
                          text="%d  %-8s %-16s %s" % (i + 1, tag, what, note),
                          font=("Consolas", 10), anchor="w", padx=6, pady=2,
                          bg="#f0f0f0", fg="#333", relief="raised",
                          command=lambda k=i + 1: self._do_action(k))
            b.grid(row=i, column=0, sticky="ew", pady=1)
            grid.columnconfigure(0, weight=1)
            self.step_labels.append(b)

        # ===== 控制 =====
        ctl = ttk.LabelFrame(self.root, text="控制")
        ctl.pack(fill="x", **pad)
        r = ttk.Frame(ctl)
        r.pack(fill="x", padx=6, pady=6)

        self.btn_next = ttk.Button(r, text="下一步 ▶", width=14,
                                   command=lambda: self._send("next"))
        self.btn_next.pack(side="left", padx=4)
        self.btn_all = ttk.Button(r, text="连跑 ▶▶", width=12,
                                  command=lambda: self._send("all"))
        self.btn_all.pack(side="left", padx=4)
        self.btn_back = ttk.Button(r, text="◀ 回退一步", width=14,
                                   command=lambda: self._send("back"))
        self.btn_back.pack(side="left", padx=4)
        self.btn_reset = ttk.Button(r, text="复位 R", width=9,
                                    command=lambda: self._send("reset"))
        self.btn_reset.pack(side="left", padx=4)
        self.btn_status = ttk.Button(r, text="状态 S", width=9,
                                     command=lambda: self._send("status"))
        self.btn_status.pack(side="left", padx=4)
        self.btn_probe = ttk.Button(r, text="下降探针 T", width=12,
                                    command=lambda: self._send("probe"))
        self.btn_probe.pack(side="left", padx=4)
        self.btn_vision = ttk.Button(r, text="视觉对准 V", width=11,
                                     command=lambda: self._send("vision"))
        self.btn_vision.pack(side="left", padx=4)
        self.btn_stop = ttk.Button(r, text="急停 X", width=9,
                                   command=lambda: self._send("stop"))
        self.btn_stop.pack(side="left", padx=4)

        r2 = ttk.Frame(ctl)
        r2.pack(fill="x", padx=6, pady=(0, 6))
        ttk.Label(r2, text="点上面任意一行 = 只做那一个动作；"
                           "「视觉对准 V」= 单独让 K230 把车摆正（不动机械臂）",
                  foreground="#666").pack(side="left")

        # ===== 日志 =====
        logf = ttk.LabelFrame(self.root, text="串口回传")
        logf.pack(fill="both", expand=True, **pad)
        wrap = ttk.Frame(logf)
        wrap.pack(fill="both", expand=True, padx=6, pady=6)
        self.txt = tk.Text(wrap, height=12, wrap="none", font=("Consolas", 9),
                           bg="#111", fg="#ddd")
        ys = ttk.Scrollbar(wrap, orient="vertical", command=self.txt.yview)
        self.txt.configure(yscrollcommand=ys.set)
        self.txt.grid(row=0, column=0, sticky="nsew")
        ys.grid(row=0, column=1, sticky="ns")
        wrap.rowconfigure(0, weight=1)
        wrap.columnconfigure(0, weight=1)

        bar = ttk.Frame(logf)
        bar.pack(fill="x", padx=6, pady=(0, 6))
        ttk.Button(bar, text="复制最近 50 行", command=self._copy_recent).pack(side="left")
        ttk.Button(bar, text="清空", command=self._clear_log).pack(side="left", padx=6)
        ttk.Button(bar, text="帮助 H", command=lambda: self._send("help")).pack(side="left")

        self._set_enabled(False)
        self._highlight(0)

    # ---------------- 串口 ----------------
    def _refresh_ports(self):
        try:
            import serial.tools.list_ports as lp
            ports = [p.device for p in lp.comports()]
        except Exception:
            ports = []
        self.cb_port["values"] = ports
        if ports and not self.cb_port.get():
            self.cb_port.set(ports[0])
        if not ports:
            self._append("（没找到串口，检查无线模块是否插好，再点『刷新』）")

    def _toggle_conn(self):
        if self.ser is not None:
            self._disconnect()
            return
        try:
            import serial
        except Exception:
            messagebox.showerror("缺少 pyserial", "命令行执行：\n    pip install pyserial\n"
                                                  "或双击 tools/run_arm_host.bat（会自动装）")
            return
        port = self.cb_port.get().strip()
        if not port:
            messagebox.showwarning("没选端口", "先选一个 COM 口")
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
        self._send("status")

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
                return
            if not raw:
                continue
            text = raw.decode("utf-8", "replace").rstrip("\r\n")
            if text:
                self.rx_q.put(text)

    # ---------------- 发送 ----------------
    def _send(self, action):
        if self.ser is None:
            self._append("（未连接）")
            return
        line = build_cmd(action)
        try:
            self.ser.write((line + "\r\n").encode("ascii", "ignore"))
        except Exception as e:
            self._append("!!! 发送失败：%s" % e)
            return
        self._append(">>> %s" % line)
        self.busy_until = time.time() + 30.0

    def _do_action(self, n):
        """点步骤列表里的一行：只做那一个动作（固件 K<n>）。"""
        if self.ser is None:
            self._append("（未连接）")
            return
        try:
            line = build_cmd("do", n)
        except Exception as e:
            self._append("!!! %s" % e)
            return
        self._append(">>> %s   （只做第 %d 个动作：%s）"
                     % (line, n, STEPS[n - 1][1] if 1 <= n <= len(STEPS) else "?"))
        try:
            self.ser.write((line + "\r\n").encode("ascii", "ignore"))
        except Exception as e:
            self._append("!!! 发送失败：%s" % e)

    def _drain_queue(self):
        n = 0
        while n < 200:
            try:
                line = self.rx_q.get_nowait()
            except queue.Empty:
                break
            n += 1
            self._append(line)
            self._handle(line)
        self.root.after(50, self._drain_queue)

    def _handle(self, line):
        info = parse_line(line)
        if not info:
            return
        k = info["kind"]

        if k in ("step", "status"):
            self.mot = info["mot"]
            self.lift = info["lift"]
            self.rot = info["rot"]
            self.grip = info["grip"]
            self.fwd = info["fwd"]
            if k == "status":
                self.phase = info["phase"]
            elif self.phase == 0:
                self.phase = 1
            self._refresh()

        if k == "home":
            self.phase = 3
            self.fwd = STEP_TOTAL
            self._refresh()
            self.lbl_msg.configure(text="已按栈恢复完，回到初始姿态", bg="#e8f5e9", fg="#1b5e20")

        if k == "stackempty":
            self.lbl_msg.configure(text="栈是空的，没有可回退的动作", bg="#fff3cd", fg="#8a6d00")

        if k == "stop":
            self.lbl_msg.configure(text="★ 急停", bg="#c00000", fg="#fff")

        if k == "run_done":
            self.lbl_msg.configure(text="连跑完成", bg="#e8f5e9", fg="#1b5e20")

    def _refresh(self):
        self.lb_phase.configure(text=phase_name(self.phase))
        self.lb_fwd.configure(text="%d/%d" % (min(self.fwd + 1, STEP_TOTAL), STEP_TOTAL))
        self.lb_mot.configure(text="%d" % self.mot)
        self.lb_rot.configure(text="%d" % self.rot)
        self.lb_grip.configure(text="%d" % self.grip)
        self.lb_lift.configure(text="%d" % self.lift)

        idx = 0 if self.phase in (2, 3) else min(self.fwd, STEP_TOTAL - 1)
        self._highlight(idx)

        if self.phase == 2:
            self.lbl_msg.configure(text="恢复阶段：点『下一步』按栈逆序退回初始姿态",
                                   bg="#fff3cd", fg="#8a6d00")
        elif self.phase == 1:
            nxt = STEPS[idx][0]
            self.lbl_msg.configure(text="下一步应该是 %s —— 确认机械臂在位再点" % nxt,
                                   bg="#e3f2fd", fg="#0d47a1")

    def _highlight(self, idx):
        for i, lb in enumerate(self.step_labels):
            if i == idx:
                lb.configure(bg="#ffe08a", fg="#000")
            else:
                lb.configure(bg="#f0f0f0", fg="#333")

    # ---------------- 日志 ----------------
    def _append(self, text):
        line = "%s  %s" % (time.strftime("%H:%M:%S"), text)
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
        self._append("（已复制最近 %d 行）" % len(tail))

    def _set_enabled(self, on):
        st = "normal" if on else "disabled"
        for b in (self.btn_next, self.btn_all, self.btn_back, self.btn_reset,
                  self.btn_status, self.btn_probe, self.btn_vision, self.btn_stop):
            try:
                b.configure(state=st)
            except Exception:
                pass

    def _on_close(self):
        if self.ser is not None:
            self._disconnect()
        self.root.destroy()


# ==================================================================
# 自检
# ==================================================================

def selftest():
    fails = []

    def check(name, got, want):
        if got != want:
            fails.append("%s: got %r want %r" % (name, got, want))

    check("next", build_cmd("next"), "N")
    check("all", build_cmd("all"), "A")
    check("back", build_cmd("back"), "B")
    check("reset", build_cmd("reset"), "R")
    check("status", build_cmd("status"), "S")
    check("stop", build_cmd("stop"), "X")
    check("probe", build_cmd("probe"), "T")
    check("do1", build_cmd("do", 1), "K1")
    check("do9", build_cmd("do", 9), "K9")
    check("vision", build_cmd("vision"), "V")

    i = parse_line("[CARPUT]   -> UP7600   MOT=7600 ROT=30 GRIP=85 LIFT=130  (fwd=1 undo=0/0)")
    check("step kind", i.get("kind"), "step")
    check("step label", i.get("label"), "UP7600")
    check("step mot", i.get("mot"), 7600)
    check("step lift servo", i.get("lift"), 130)
    check("step fwd", i.get("fwd"), 1)

    i = parse_line("[CARPUT] STATUS phase=2 fwd=8/8 undo=3/7 MOT=1000 ROT=30 GRIP=110 LIFT=130")
    check("status kind", i.get("kind"), "status")
    check("status phase", i.get("phase"), 2)
    check("status undo total", i.get("undo_total"), 7)
    check("status mot", i.get("mot"), 1000)
    check("status grip", i.get("grip"), 110)
    check("status lift servo", i.get("lift"), 130)

    check("home", parse_line("[CARPUT] 已按栈恢复完，回到初始姿态").get("kind"), "home")
    check("stackempty", parse_line("[CARPUT] 栈是空的，没得回退").get("kind"), "stackempty")
    check("stop", parse_line("[CARPUT] PA5 -> 急停").get("kind"), "stop")
    check("unknown", parse_line("[CARPUT] HOST> N"), {})

    check("phase 1", phase_name(1), "正向")
    check("phase 2", phase_name(2), "恢复")
    check("idx from 0", step_index_from(0), 1)
    check("idx from 7", step_index_from(7), 8)
    check("idx clamp", step_index_from(99), 11)
    check("steps len", len(STEPS), 11)

    if fails:
        print("SELFTEST FAILED:")
        for f in fails:
            print("  -", f)
        return 1
    print("SELFTEST OK（%d 项）" % 23)
    return 0


def main():
    if len(sys.argv) > 1 and sys.argv[1] in ("--selftest", "-t"):
        return selftest()
    if not HAS_TK:
        print("没有 tkinter，跑不了 GUI。")
        return 1
    root = tk.Tk()
    try:
        root.call("tk", "scaling", 1.2)
    except Exception:
        pass
    ArmApp(root)
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
