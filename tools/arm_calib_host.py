# -*- coding: utf-8 -*-
'''
整臂联动标定上位机（升降步进 + 三个舵机，一个界面全调）

=================================================================
对应固件：User/main.c  DEBUG_MODE = 16  (DEBUG_MODE_ARM_ALL)
串口：F103 的 USART3 = PB10/PB11，115200 8N1（无线模块）
=================================================================
四个可调对象：
    升降步进电机   总线地址 6，脉冲 0~8000（上电 0 = 最低位）
    旋转舵机       PC9 / TIM3_CH4   上电 30°
    夹爪舵机       PC8 / TIM3_CH3   上电 85°
    升降舵机       PC7 / TIM3_CH2   上电 90°

界面分三块：
    ① 升降步进：↑ / ↓ / 回零 / 走到绝对位置，实时显示 CMD 和 ACT
    ② 舵机：三个按钮选对象，＋ / － 大按钮调角度
    ③ 姿态记录本：把当前"步进+三舵机"的完整姿态存下来，可以一键摆回去，
       也可以全部复制成文本 —— 摆好几个关键姿态后复制给我，我直接写进流程。

命令（F103 端实现）：
    U<n> / D<n>     步进上升/下降 n 脉冲
    G<n>            步进走到绝对位置 n
    L               步进回零
    S<1..3>         选舵机（1旋转 2夹爪 3升降舵机）
    +<n> / -<n>     选中舵机 角度加/减 n 度
    A<n>            选中舵机走到绝对角度 n
    R               选中舵机回上电角度
    P<s>,<r>,<g>,<l>  一次摆到完整姿态
    N<n>            设舵机步长
    Q               查询状态
    X               升降急停
=================================================================
'''

import os
import sys
import re
import json
import time
import queue
import threading
import traceback

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, simpledialog
    HAS_TK = True
except Exception:
    HAS_TK = False

BAUD_DEFAULT = 115200

SERVOS = [
    ("ROT",  "旋转", "PC9", 30),
    ("GRIP", "夹爪", "PC8", 85),
    ("LIFT", "升降", "PC7", 90),
]
SERVO_STEPS = [1, 5, 10, 45]
MOTOR_STEPS = [10, 50, 100, 500, 1000]

POSE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pose_book.json")

# ==================================================================
# 纯逻辑
# ==================================================================

def build_cmd(action, arg=None, arg2=None, arg3=None, arg4=None):
    if action == "motor_up":
        return "U%d" % int(arg)
    if action == "motor_down":
        return "D%d" % int(arg)
    if action == "motor_go":
        v = max(0, min(8000, int(arg)))
        return "G%d" % v
    if action == "motor_zero":
        return "L"
    if action == "select":
        n = int(arg)
        if n < 1 or n > 3:
            raise ValueError("select 1~3")
        return "S%d" % n
    if action in ("plus", "minus"):
        sign = "+" if action == "plus" else "-"
        return sign if arg is None else "%s%d" % (sign, int(arg))
    if action == "abs":
        return "A%d" % max(0, min(270, int(arg)))
    if action == "home":
        return "R"
    if action == "pose":
        return "P%d,%d,%d,%d" % (int(arg), int(arg2), int(arg3), int(arg4))
    if action == "step":
        return "N%d" % max(1, min(90, int(arg)))
    if action == "query":
        return "Q"
    if action == "setzero":
        return "W"
    if action == "stop":
        return "X"
    raise ValueError("unknown action %r" % (action,))


RE_STATUS = re.compile(
    r"STATUS\s+MOT=(\d+)\s+ACT=(-?\d+)\s+SEL=(\w+)\s+STEP=(\d+)\s+ROT=(\d+)\s+GRIP=(\d+)\s+LIFT=(\d+)")
RE_MOTOR = re.compile(r"MOT\s+(\d+)\s+->\s+(\d+)")
RE_ACT = re.compile(r"ACT=(-?\d+)\s+d=(-?\d+)")
RE_SERVO = re.compile(r"(ROT|GRIP|LIFT)\s+(\d+)\s+->\s+(\d+)\s+pulse=(\d+)")
RE_POSE = re.compile(r"摆姿态\s+MOT=(\d+)\s+ROT=(\d+)\s+GRIP=(\d+)\s+LIFT=(\d+)")
RE_SEL = re.compile(r"换对象\s*->\s*(\w+)")


def parse_line(text):
    """解析一行 [ARM] 回传，返回 dict。"""
    if not text:
        return {}
    t = text.strip()

    m = RE_STATUS.search(t)
    if m:
        return {
            "kind": "status",
            "mot": int(m.group(1)),
            "act": int(m.group(2)),
            "sel": m.group(3),
            "step": int(m.group(4)),
            "rot": int(m.group(5)),
            "grip": int(m.group(6)),
            "lift": int(m.group(7)),
        }

    m = RE_POSE.search(t)
    if m:
        return {
            "kind": "pose",
            "mot": int(m.group(1)),
            "rot": int(m.group(2)),
            "grip": int(m.group(3)),
            "lift": int(m.group(4)),
        }

    m = RE_MOTOR.search(t)
    if m:
        return {"kind": "motor", "before": int(m.group(1)), "after": int(m.group(2))}

    m = RE_ACT.search(t)
    if m:
        return {"kind": "act", "act": int(m.group(1)), "d": int(m.group(2))}

    m = RE_SERVO.search(t)
    if m:
        return {
            "kind": "servo",
            "id": m.group(1),
            "before": int(m.group(2)),
            "after": int(m.group(3)),
            "pulse": int(m.group(4)),
        }

    m = RE_SEL.search(t)
    if m:
        return {"kind": "obj", "name": m.group(1)}

    return {}


def servo_index(name):
    for i, s in enumerate(SERVOS):
        if s[0] == name:
            return i
    return -1


def pose_row(name, mot, rot, grip, lift):
    """一行姿态文本，方便贴给我。"""
    return "%-12s  MOT=%-5d ROT=%-3d GRIP=%-3d LIFT=%-3d" % (name, mot, rot, grip, lift)


def load_poses(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, list):
            return data
    except Exception:
        pass
    return []


def save_poses(path, poses):
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(poses, f, ensure_ascii=False, indent=2)
        return True
    except Exception:
        return False


# ==================================================================
# GUI
# ==================================================================

class ArmCalApp(object):
    def __init__(self, root):
        self.root = root
        self.ser = None
        self.rx_q = queue.Queue()
        self.rx_stop = threading.Event()
        self.log_lines = []

        self.mot = 0          # 步进 CMD
        self.act = 0          # 步进 ACT
        self.act_ok = False
        self.ang = {s[0]: s[3] for s in SERVOS}
        self.sel_servo = 0

        self.poses = load_poses(POSE_FILE)

        root.title("整臂联动标定 —— 升降步进 + 三舵机")
        root.geometry("900x760")
        root.minsize(860, 700)

        self._build_ui()
        self._refresh_ports()
        self.root.after(50, self._drain_queue)
        root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------- 界面 ----------------
    def _build_ui(self):
        pad = dict(padx=6, pady=4)

        # 串口
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

        body = ttk.Frame(self.root)
        body.pack(fill="x", **pad)

        # ===== 左：升降步进 =====
        lf = ttk.LabelFrame(body, text="① 升降步进电机（地址 6）")
        lf.pack(side="left", fill="both", expand=True, padx=(0, 4))

        num = ttk.Frame(lf)
        num.pack(fill="x", padx=6, pady=6)
        ttk.Label(num, text="CMD", font=("Consolas", 9)).grid(row=0, column=0, padx=8)
        ttk.Label(num, text="ACT", font=("Consolas", 9)).grid(row=0, column=1, padx=8)
        ttk.Label(num, text="Δ", font=("Consolas", 9)).grid(row=0, column=2, padx=8)
        self.lb_mot = ttk.Label(num, text="0", font=("Consolas", 20, "bold"), width=6, anchor="center")
        self.lb_mot.grid(row=1, column=0, padx=8)
        self.lb_act = ttk.Label(num, text="--", font=("Consolas", 20, "bold"), width=6, anchor="center")
        self.lb_act.grid(row=1, column=1, padx=8)
        self.lb_d = ttk.Label(num, text="--", font=("Consolas", 20, "bold"), width=6, anchor="center")
        self.lb_d.grid(row=1, column=2, padx=8)

        r = ttk.Frame(lf)
        r.pack(fill="x", padx=6, pady=4)
        ttk.Label(r, text="步长").pack(side="left")
        self.var_mstep = tk.StringVar(value="100")
        ttk.Entry(r, textvariable=self.var_mstep, width=6).pack(side="left", padx=4)
        for v in MOTOR_STEPS:
            ttk.Button(r, text=str(v), width=5,
                       command=lambda x=v: self.var_mstep.set(str(x))).pack(side="left", padx=2)

        r2 = ttk.Frame(lf)
        r2.pack(fill="x", padx=6, pady=6)
        self.btn_mup = tk.Button(r2, text="↑ 上升", font=("Microsoft YaHei", 13, "bold"),
                                 width=9, height=2, bg="#d9ffd9", command=self._mup)
        self.btn_mup.pack(side="left", padx=4)
        self.btn_mdown = tk.Button(r2, text="↓ 下降", font=("Microsoft YaHei", 13, "bold"),
                                   width=9, height=2, bg="#ffd9d9", command=self._mdown)
        self.btn_mdown.pack(side="left", padx=4)
        self.btn_mzero = ttk.Button(r2, text="回零 L", width=8, command=lambda: self._send("motor_zero"))
        self.btn_mzero.pack(side="left", padx=4)

        r3 = ttk.Frame(lf)
        r3.pack(fill="x", padx=6, pady=(0, 6))
        ttk.Label(r3, text="绝对位置").pack(side="left")
        self.var_go = tk.StringVar(value="7600")
        ttk.Entry(r3, textvariable=self.var_go, width=7).pack(side="left", padx=4)
        self.btn_go = ttk.Button(r3, text="走到 G", width=8, command=self._send_go)
        self.btn_go.pack(side="left", padx=4)
        self.btn_query = ttk.Button(r3, text="查询 Q", width=8, command=lambda: self._send("query"))
        self.btn_query.pack(side="left", padx=4)
        self.btn_setzero = ttk.Button(r3, text="设零点 W", width=9,
                                      command=self._confirm_setzero)
        self.btn_setzero.pack(side="left", padx=4)

        ttk.Label(lf, text="※ 机械臂先手动降到最低位，再点『设零点 W』，之后 0 就等于机械底",
                  foreground="#a05a00").pack(fill="x", padx=6, pady=(0, 6))

        # ===== 右：舵机 =====
        rf = ttk.LabelFrame(body, text="② 舵机（点按钮选对象，＋/－调角度）")
        rf.pack(side="left", fill="both", expand=True, padx=(4, 0))

        sel = ttk.Frame(rf)
        sel.pack(fill="x", padx=6, pady=6)
        self.servo_btns = []
        for i, (tag, cn, pin, init) in enumerate(SERVOS):
            b = tk.Button(sel, text="%s\n%s\n%d°" % (cn, pin, init),
                          font=("Microsoft YaHei", 9), width=8, height=3,
                          command=lambda k=i: self._select(k))
            b.pack(side="left", padx=4)
            self.servo_btns.append(b)

        big = ttk.Frame(rf)
        big.pack(fill="x", padx=6, pady=6)
        self.btn_minus = tk.Button(big, text="－", font=("Microsoft YaHei", 16, "bold"),
                                   width=5, height=2, bg="#ffd9d9",
                                   command=lambda: self._send("minus", int(self.var_sstep.get() or 5)))
        self.btn_minus.pack(side="left", padx=4)
        mid = ttk.Frame(big)
        mid.pack(side="left", expand=True, padx=4)
        self.lbl_sel = ttk.Label(mid, text="旋转", font=("Microsoft YaHei", 9))
        self.lbl_sel.pack()
        self.lbl_ang = ttk.Label(mid, text="30°", font=("Consolas", 26, "bold"))
        self.lbl_ang.pack()
        self.btn_plus = tk.Button(big, text="＋", font=("Microsoft YaHei", 16, "bold"),
                                  width=5, height=2, bg="#d9ffd9",
                                  command=lambda: self._send("plus", int(self.var_sstep.get() or 5)))
        self.btn_plus.pack(side="left", padx=4)

        r4 = ttk.Frame(rf)
        r4.pack(fill="x", padx=6, pady=4)
        ttk.Label(r4, text="步长").pack(side="left")
        self.var_sstep = tk.StringVar(value="5")
        ttk.Entry(r4, textvariable=self.var_sstep, width=5).pack(side="left", padx=3)
        for v in SERVO_STEPS:
            ttk.Button(r4, text=str(v), width=3,
                       command=lambda x=v: self.var_sstep.set(str(x))).pack(side="left", padx=2)

        r5 = ttk.Frame(rf)
        r5.pack(fill="x", padx=6, pady=(0, 6))
        ttk.Label(r5, text="绝对").pack(side="left")
        self.var_abs = tk.StringVar(value="90")
        ttk.Entry(r5, textvariable=self.var_abs, width=5).pack(side="left", padx=4)
        self.btn_abs = ttk.Button(r5, text="走到 A", width=7, command=self._send_abs)
        self.btn_abs.pack(side="left", padx=4)
        self.btn_home = ttk.Button(r5, text="回初始 R", width=8, command=lambda: self._send("home"))
        self.btn_home.pack(side="left", padx=4)

        # ===== 姿态记录本 =====
        pf = ttk.LabelFrame(self.root, text="③ 姿态记录本（摆好一个姿态就存一条，最后复制给我）")
        pf.pack(fill="x", **pad)

        pr = ttk.Frame(pf)
        pr.pack(fill="x", padx=6, pady=6)

        left = ttk.Frame(pr)
        left.pack(side="left", fill="both", expand=True)
        self.listbox = tk.Listbox(left, height=6, font=("Consolas", 9))
        self.listbox.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(left, orient="vertical", command=self.listbox.yview)
        self.listbox.configure(yscrollcommand=sb.set)
        sb.pack(side="left", fill="y")

        right = ttk.Frame(pr)
        right.pack(side="left", fill="y", padx=8)
        ttk.Button(right, text="记录当前姿态", width=14, command=self._record_pose).pack(pady=2)
        ttk.Button(right, text="应用到选中", width=14, command=self._apply_pose).pack(pady=2)
        ttk.Button(right, text="删除选中", width=14, command=self._del_pose).pack(pady=2)
        ttk.Button(right, text="复制全部（贴给我）", width=14, command=self._copy_poses).pack(pady=2)

        # ===== 日志 =====
        logf = ttk.LabelFrame(self.root, text="串口回传")
        logf.pack(fill="both", expand=True, **pad)
        wrap = ttk.Frame(logf)
        wrap.pack(fill="both", expand=True, padx=6, pady=6)
        self.txt = tk.Text(wrap, height=8, wrap="none", font=("Consolas", 9), bg="#111", fg="#ddd")
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

        self._set_enabled(False)
        self._refresh_all()
        self._refresh_pose_list()

    # ---------------- 显示 ----------------
    def _refresh_all(self):
        self.lb_mot.configure(text="%d" % self.mot)
        if self.act_ok:
            self.lb_act.configure(text="%d" % self.act)
            self.lb_d.configure(text="%+d" % (self.act - self.mot),
                                foreground="#070" if abs(self.act - self.mot) <= 20 else "#a00")
        else:
            self.lb_act.configure(text="----")
            self.lb_d.configure(text="--", foreground="#a00")

        tag, cn, pin, init = SERVOS[self.sel_servo]
        self.lbl_sel.configure(text="%s（%s）" % (cn, pin))
        self.lbl_ang.configure(text="%d°" % self.ang[tag])

        for i, b in enumerate(self.servo_btns):
            t, c, p, ini = SERVOS[i]
            if i == self.sel_servo:
                b.configure(bg="#cfe8ff", relief="sunken")
            else:
                b.configure(bg="#f0f0f0", relief="raised")
            b.configure(text="%s\n%s\n%d°" % (c, p, self.ang[t]))

    def _refresh_pose_list(self):
        self.listbox.delete(0, "end")
        for p in self.poses:
            self.listbox.insert("end", pose_row(p.get("name", "?"), p.get("mot", 0),
                                                p.get("rot", 0), p.get("grip", 0),
                                                p.get("lift", 0)))

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
            messagebox.showerror("缺少 pyserial",
                                 "命令行执行：\n    pip install pyserial\n"
                                 "或双击 tools/run_arm_calib.bat（会自动装）")
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
        threading.Thread(target=self._rx_loop, daemon=True).start()

        self.lbl_conn.configure(text="已连接 %s" % port, foreground="#070")
        self.btn_conn.configure(text="断开")
        self._set_enabled(True)
        self._append("=== 已连接 %s @ %d ===" % (port, baud))
        self._send("query")

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
                return
            if not raw:
                continue
            text = raw.decode("utf-8", "replace").rstrip("\r\n")
            if text:
                self.rx_q.put(text)

    # ---------------- 发送 ----------------
    def _write(self, line):
        if self.ser is None:
            self._append("（未连接）")
            return
        try:
            self.ser.write((line + "\r\n").encode("ascii", "ignore"))
        except Exception as e:
            self._append("!!! 发送失败：%s" % e)
            return
        self._append(">>> %s" % line)

    def _send(self, action, arg=None):
        try:
            self._write(build_cmd(action, arg))
        except Exception as e:
            self._append("!!! %s" % e)

    def _mup(self):
        try:
            self._write(build_cmd("motor_up", int(self.var_mstep.get())))
        except Exception:
            messagebox.showwarning("步长不对", "步长要填整数")

    def _mdown(self):
        try:
            self._write(build_cmd("motor_down", int(self.var_mstep.get())))
        except Exception:
            messagebox.showwarning("步长不对", "步长要填整数")

    def _send_go(self):
        try:
            self._write(build_cmd("motor_go", int(self.var_go.get())))
        except Exception:
            messagebox.showwarning("位置不对", "绝对位置要填整数")

    def _send_abs(self):
        try:
            self._write(build_cmd("abs", int(self.var_abs.get())))
        except Exception:
            messagebox.showwarning("角度不对", "绝对角度要填整数")

    def _confirm_setzero(self):
        if not messagebox.askyesno(
                "设零点确认",
                "把驱动器**当前所在位置**设为 0 点？\n\n"
                "请先确认机械臂已经停在最低位（或限位上）。\n"
                "设完之后所有绝对位置都从这一处重新算，之前的\ndeg/脉冲偏移会全部失效。"):
            return
        self._write(build_cmd("setzero"))

    def _select(self, idx):
        self.sel_servo = idx
        self._refresh_all()
        self._write(build_cmd("select", idx + 1))

    # ---------------- 接收 ----------------
    def _drain_queue(self):
        n = 0
        while n < 300:
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

        if k == "status":
            self.mot = info["mot"]
            self.act = info["act"]
            self.act_ok = info["act"] >= 0
            self.ang["ROT"] = info["rot"]
            self.ang["GRIP"] = info["grip"]
            self.ang["LIFT"] = info["lift"]
            self._refresh_all()

        elif k == "pose":
            self.mot = info["mot"]
            self.ang["ROT"] = info["rot"]
            self.ang["GRIP"] = info["grip"]
            self.ang["LIFT"] = info["lift"]
            self._refresh_all()

        elif k == "motor":
            self.mot = info["after"]
            self._refresh_all()

        elif k == "act":
            self.act = info["act"]
            self.act_ok = True
            self._refresh_all()

        elif k == "servo":
            self.ang[info["id"]] = info["after"]
            self._refresh_all()

        elif k == "obj":
            idx = servo_index(info["name"])
            if idx < 0 and info["name"] == "MOTOR":
                idx = self.sel_servo
            if idx >= 0:
                self.sel_servo = idx
                self._refresh_all()

    # ---------------- 姿态记录本 ----------------
    def _record_pose(self):
        name = simpledialog.askstring("记录姿态", "给这个姿态起个名字（例如 抓料位）：",
                                      parent=self.root)
        if not name:
            return
        self.poses.append({
            "name": name.strip(),
            "mot": self.mot,
            "rot": self.ang["ROT"],
            "grip": self.ang["GRIP"],
            "lift": self.ang["LIFT"],
            "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
        })
        save_poses(POSE_FILE, self.poses)
        self._refresh_pose_list()
        self._append("（已记录姿态 %s：MOT=%d ROT=%d GRIP=%d LIFT=%d）"
                     % (name, self.mot, self.ang["ROT"], self.ang["GRIP"], self.ang["LIFT"]))

    def _apply_pose(self):
        sel = self.listbox.curselection()
        if not sel:
            messagebox.showinfo("没选中", "先在列表里选一条姿态")
            return
        p = self.poses[sel[0]]
        try:
            line = build_cmd("pose", p.get("mot", 0), p.get("rot", 30),
                             p.get("grip", 85), p.get("lift", 90))
        except Exception as e:
            messagebox.showerror("姿态数据坏了", str(e))
            return
        self._write(line)

    def _del_pose(self):
        sel = self.listbox.curselection()
        if not sel:
            return
        if not messagebox.askyesno("删除", "删除选中的这条姿态？"):
            return
        del self.poses[sel[0]]
        save_poses(POSE_FILE, self.poses)
        self._refresh_pose_list()

    def _copy_poses(self):
        if not self.poses:
            messagebox.showinfo("空的", "还没有记录任何姿态")
            return
        lines = ["%-12s  %5s  %4s  %5s  %5s" % ("名称", "步进", "旋转", "夹爪", "升降舵")]
        for p in self.poses:
            lines.append(pose_row(p.get("name", "?"), p.get("mot", 0), p.get("rot", 0),
                                  p.get("grip", 0), p.get("lift", 0)))
        text = "\n".join(lines)
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        self._append("（已复制 %d 条姿态到剪贴板，直接贴给对面就行）" % len(self.poses))

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
        for b in (self.btn_mup, self.btn_mdown, self.btn_mzero, self.btn_go,
                  self.btn_query, self.btn_setzero, self.btn_plus, self.btn_minus,
                  self.btn_abs, self.btn_home):
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

    check("mup", build_cmd("motor_up", 100), "U100")
    check("mdown", build_cmd("motor_down", 50), "D50")
    check("mgo", build_cmd("motor_go", 7600), "G7600")
    check("mgo clamp", build_cmd("motor_go", 99999), "G8000")
    check("zero", build_cmd("motor_zero"), "L")
    check("sel1", build_cmd("select", 1), "S1")
    check("plus", build_cmd("plus", 5), "+5")
    check("minus", build_cmd("minus", 5), "-5")
    check("abs", build_cmd("abs", 135), "A135")
    check("home", build_cmd("home"), "R")
    check("pose", build_cmd("pose", 4000, 163, 115, 90), "P4000,163,115,90")
    check("step", build_cmd("step", 10), "N10")
    check("query", build_cmd("query"), "Q")

    i = parse_line("[ARM] STATUS MOT=200 ACT=200 SEL=LIFT STEP=10 ROT=30 GRIP=85 LIFT=90")
    check("status kind", i.get("kind"), "status")
    check("status mot", i.get("mot"), 200)
    check("status lift servo", i.get("lift"), 90)
    check("status step", i.get("step"), 10)
    check("status act no-read", parse_line("[ARM] STATUS MOT=200 ACT=-1 SEL=MOTOR STEP=100 ROT=30 GRIP=85 LIFT=90").get("act"), -1)

    i = parse_line("[ARM] UP   MOT 0 -> 100  (expect 460 ms)")
    check("motor kind", i.get("kind"), "motor")
    check("motor after", i.get("after"), 100)

    i = parse_line("[ARM]   ACT=100  d=0")
    check("act kind", i.get("kind"), "act")
    check("act d", i.get("d"), 0)

    i = parse_line("[ARM] GRIP 85 -> 90   pulse=1167us")
    check("servo kind", i.get("kind"), "servo")
    check("servo id", i.get("id"), "GRIP")
    check("servo pulse", i.get("pulse"), 1167)

    i = parse_line("[ARM] 摆姿态 MOT=4000 ROT=163 GRIP=115 LIFT=90")
    check("pose kind", i.get("kind"), "pose")
    check("pose rot", i.get("rot"), 163)

    check("obj line", parse_line("[ARM] 换对象 -> LIFT").get("name"), "LIFT")
    check("unknown", parse_line("[ARM] HOST> U100"), {})

    check("servo idx", servo_index("LIFT"), 2)
    row = pose_row("抓料位", 4000, 163, 115, 90)
    check("pose row has mot", "4000" in row, True)
    check("pose row has name", "抓料位" in row, True)

    if fails:
        print("SELFTEST FAILED:")
        for f in fails:
            print("  -", f)
        return 1
    print("SELFTEST OK（%d 项）" % 29)
    return 0


def main():
    if len(sys.argv) > 1 and sys.argv[1] in ("--selftest", "-t"):
        return selftest()
    if not HAS_TK:
        print("没有 tkinter，跑不了 GUI。")
        return 1
    root = tk.Tk()
    try:
        root.call("tk", "scaling", 1.15)
    except Exception:
        pass
    ArmCalApp(root)
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
