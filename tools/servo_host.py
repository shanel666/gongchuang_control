# -*- coding: utf-8 -*-
'''
三舵机标定上位机（无线串口）

=================================================================
对应固件：User/main.c  DEBUG_MODE = 15  (DEBUG_MODE_SERVO3)
串口：F103 的 USART3 = PB10/PB11，115200 8N1（无线模块）
=================================================================
三路舵机（都是 TIM3 完全重映射到 PC 口）：
    1  旋转  SERVO_ROT   PC9 / TIM3_CH4   上电 30°
    2  夹爪  SERVO_GRIP  PC8 / TIM3_CH3   上电 85°
    3  升降  SERVO_LIFT  PC7 / TIM3_CH2   上电 90°

界面：
    顶部三个按钮选"要调哪一路"（当前选中的会高亮）
    两个大按钮调"正方向 / 反方向"（每点一次按步长走一步）
    下面有步长、绝对角度、回初始、查询

命令（F103 端实现）：
    S1 / S2 / S3   选舵机
    +N / -N        当前舵机 加/减 N 度（不给 N 用当前步长）
    A<N>           当前舵机 走到绝对角度 N
    N<N>           设步长（1~90）
    Z              当前舵机 回初始角度
    Q              查询三路角度

F103 回：
    [SERVO3] LIFT 90 -> 95   pulse=1204us
    [SERVO3] STATUS sel=LIFT step=5  ROT=30 GRIP=85 LIFT=95
=================================================================
'''

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

# (id, 短名, 长名, 引脚, 上电角度)
SERVOS = [
    ("ROT",  "旋转", "PC9 / TIM3_CH4", 30),
    ("GRIP", "夹爪", "PC8 / TIM3_CH3", 85),
    ("LIFT", "升降", "PC7 / TIM3_CH2", 90),
]

STEPS = [1, 5, 10, 45]

# ==================================================================
# 纯逻辑
# ==================================================================

def build_cmd(action, arg=None):
    if action == "select":
        n = int(arg)
        if n < 1 or n > 3:
            raise ValueError("select 1~3")
        return "S%d" % n
    if action in ("plus", "minus"):
        sign = "+" if action == "plus" else "-"
        if arg is None:
            return sign
        v = int(arg)
        if v <= 0 or v > 270:
            raise ValueError("步长 1~270")
        return "%s%d" % (sign, v)
    if action == "abs":
        v = int(arg)
        v = max(0, min(270, v))
        return "A%d" % v
    if action == "step":
        v = int(arg)
        v = max(1, min(90, v))
        return "N%d" % v
    if action == "home":
        return "Z"
    if action == "query":
        return "Q"
    if action == "help":
        return "H"
    raise ValueError("unknown action %r" % (action,))


RE_STATUS = re.compile(r"STATUS\s+sel=(\w+)\s+step=(\d+)\s+ROT=(\d+)\s+GRIP=(\d+)\s+LIFT=(\d+)")
RE_MOVE = re.compile(r"(ROT|GRIP|LIFT)\s+(\d+)\s+->\s+(\d+)\s+pulse=(\d+)")
RE_SEL = re.compile(r"选中\s+(ROT|GRIP|LIFT)")


def parse_line(text):
    """解析一行 [SERVO3] 回传。"""
    if not text:
        return {}
    t = text.strip()

    m = RE_STATUS.search(t)
    if m:
        return {
            "kind": "status",
            "sel": m.group(1),
            "step": int(m.group(2)),
            "ang": {
                "ROT": int(m.group(3)),
                "GRIP": int(m.group(4)),
                "LIFT": int(m.group(5)),
            },
        }

    m = RE_MOVE.search(t)
    if m:
        return {
            "kind": "move",
            "id": m.group(1),
            "before": int(m.group(2)),
            "after": int(m.group(3)),
            "pulse": int(m.group(4)),
        }

    m = RE_SEL.search(t)
    if m:
        return {"kind": "select", "sel": m.group(1)}

    m = re.search(r"步长\s*->\s*(\d+)", t)
    if m:
        return {"kind": "step", "step": int(m.group(1))}

    return {}


def name_to_index(n):
    for i, s in enumerate(SERVOS):
        if s[0] == n:
            return i
    return -1


# ==================================================================
# GUI
# ==================================================================

class ServoApp(object):
    def __init__(self, root):
        self.root = root
        self.ser = None
        self.rx_q = queue.Queue()
        self.rx_stop = threading.Event()
        self.log_lines = []
        self.busy_until = 0.0

        self.sel = 0                       # 当前选中的舵机下标
        self.ang = {s[0]: s[3] for s in SERVOS}

        root.title("三舵机标定 —— 无线串口")
        root.geometry("760x660")
        root.minsize(720, 600)

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

        # 选舵机
        sel = ttk.LabelFrame(self.root, text="① 点这里选要调哪一路（当前选中的会变大、变蓝）")
        sel.pack(fill="x", **pad)
        self.servo_btns = []
        for i, (tag, cn, pin, init) in enumerate(SERVOS):
            b = tk.Button(sel, text="%d\n%s\n%s" % (i + 1, cn, pin),
                          font=("Microsoft YaHei", 10), width=14, height=3,
                          relief="raised", bd=2,
                          command=lambda k=i: self._select(k))
            b.pack(side="left", padx=8, pady=8)
            self.servo_btns.append(b)

        # 正反方向
        ctl = ttk.LabelFrame(self.root, text="② 点这里调角度（每点一次走一个步长）")
        ctl.pack(fill="x", **pad)

        big = ttk.Frame(ctl)
        big.pack(fill="x", padx=8, pady=8)

        self.btn_minus = tk.Button(big, text="－\n反方向", font=("Microsoft YaHei", 18, "bold"),
                                   width=10, height=2, bg="#ffd9d9",
                                   command=lambda: self._send("minus"))
        self.btn_minus.pack(side="left", padx=10)

        mid = ttk.Frame(big)
        mid.pack(side="left", expand=True, fill="both", padx=10)
        ttk.Label(mid, text="当前舵机", font=("Microsoft YaHei", 9)).pack()
        self.lbl_sel = ttk.Label(mid, text="旋转", font=("Microsoft YaHei", 18, "bold"))
        self.lbl_sel.pack()
        self.lbl_ang = ttk.Label(mid, text="30°", font=("Consolas", 34, "bold"))
        self.lbl_ang.pack()
        self.lbl_pulse = ttk.Label(mid, text="脉宽 -- us", font=("Consolas", 9), foreground="#666")
        self.lbl_pulse.pack()

        self.btn_plus = tk.Button(big, text="＋\n正方向", font=("Microsoft YaHei", 18, "bold"),
                                  width=10, height=2, bg="#d9ffd9",
                                  command=lambda: self._send("plus"))
        self.btn_plus.pack(side="left", padx=10)

        # 参数
        prm = ttk.LabelFrame(self.root, text="③ 步长 / 绝对角度 / 其它")
        prm.pack(fill="x", **pad)

        r1 = ttk.Frame(prm)
        r1.pack(fill="x", padx=8, pady=5)
        ttk.Label(r1, text="步长").pack(side="left")
        self.var_step = tk.StringVar(value="5")
        ttk.Entry(r1, textvariable=self.var_step, width=5).pack(side="left", padx=4)
        ttk.Label(r1, text="度").pack(side="left")
        for v in STEPS:
            ttk.Button(r1, text=str(v), width=4,
                       command=lambda x=v: self._set_step(x)).pack(side="left", padx=2)

        ttk.Label(r1, text="   绝对角度").pack(side="left")
        self.var_abs = tk.StringVar(value="90")
        ttk.Entry(r1, textvariable=self.var_abs, width=5).pack(side="left", padx=4)
        self.btn_abs = ttk.Button(r1, text="走到", width=6, command=self._send_abs)
        self.btn_abs.pack(side="left", padx=4)

        r2 = ttk.Frame(prm)
        r2.pack(fill="x", padx=8, pady=(0, 6))
        self.btn_home = ttk.Button(r2, text="回初始 Z", width=10,
                                   command=lambda: self._send("home"))
        self.btn_home.pack(side="left", padx=4)
        self.btn_query = ttk.Button(r2, text="查询 Q", width=9,
                                    command=lambda: self._send("query"))
        self.btn_query.pack(side="left", padx=4)
        ttk.Label(r2, text="（回初始 = 旋转 30° / 夹爪 85° / 升降 90°）",
                  foreground="#666").pack(side="left", padx=8)

        # 三路角度一览
        allf = ttk.LabelFrame(self.root, text="三路当前角度")
        allf.pack(fill="x", **pad)
        af = ttk.Frame(allf)
        af.pack(fill="x", padx=8, pady=6)
        self.lbl_all = {}
        for i, (tag, cn, pin, init) in enumerate(SERVOS):
            ttk.Label(af, text="%s" % cn, font=("Microsoft YaHei", 9)).grid(row=0, column=i * 2, padx=(10, 2))
            lb = ttk.Label(af, text="%d°" % init, font=("Consolas", 16, "bold"), width=5)
            lb.grid(row=0, column=i * 2 + 1, padx=(0, 10))
            self.lbl_all[tag] = lb

        # 日志
        logf = ttk.LabelFrame(self.root, text="串口回传")
        logf.pack(fill="both", expand=True, **pad)
        wrap = ttk.Frame(logf)
        wrap.pack(fill="both", expand=True, padx=6, pady=6)
        self.txt = tk.Text(wrap, height=10, wrap="none", font=("Consolas", 9), bg="#111", fg="#ddd")
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
        self._refresh_sel()

    def _refresh_sel(self):
        tag, cn, pin, init = SERVOS[self.sel]
        self.lbl_sel.configure(text=cn)
        self.lbl_ang.configure(text="%d°" % self.ang[tag])
        self.lbl_pulse.configure(text="脉宽 -- us")
        for i, b in enumerate(self.servo_btns):
            if i == self.sel:
                b.configure(bg="#cfe8ff", relief="sunken", font=("Microsoft YaHei", 12, "bold"))
            else:
                b.configure(bg="#f0f0f0", relief="raised", font=("Microsoft YaHei", 10))
        for t, lb in self.lbl_all.items():
            lb.configure(text="%d°" % self.ang[t])

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
                                 "或双击 tools/run_servo_host.bat（会自动装）")
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
    def _send(self, action):
        if self.ser is None:
            self._append("（未连接）")
            return
        try:
            line = build_cmd(action)
        except Exception as e:
            self._append("!!! %s" % e)
            return
        self._write(line)

    def _write(self, line):
        try:
            self.ser.write((line + "\r\n").encode("ascii", "ignore"))
        except Exception as e:
            self._append("!!! 发送失败：%s" % e)
            return
        self._append(">>> %s" % line)

    def _select(self, idx):
        self.sel = idx
        self._refresh_sel()
        if self.ser is not None:
            self._write(build_cmd("select", idx + 1))
        else:
            self._append("（未连接，先选中本地状态）")

    def _set_step(self, v):
        self.var_step.set(str(v))
        if self.ser is not None:
            self._write(build_cmd("step", v))

    def _send_abs(self):
        try:
            v = int(self.var_abs.get())
        except Exception:
            messagebox.showwarning("绝对角度不对", "要填整数，例如 90")
            return
        if self.ser is None:
            self._append("（未连接）")
            return
        self._write(build_cmd("abs", v))

    # ---------------- 接收 ----------------
    def _drain_queue(self):
        n = 0
        while n < 200:
            try:
                line = self.rx_q.get_nowait()
            except queue.Empty:
                break
            n += 1
            self._append(line)
            info = parse_line(line)
            if not info:
                continue
            k = info["kind"]

            if k == "status":
                for tag, v in info["ang"].items():
                    self.ang[tag] = v
                idx = name_to_index(info["sel"])
                if idx >= 0:
                    self.sel = idx
                self.var_step.set(str(info["step"]))
                self._refresh_sel()

            elif k == "move":
                self.ang[info["id"]] = info["after"]
                idx = name_to_index(info["id"])
                if idx == self.sel:
                    self.lbl_ang.configure(text="%d°" % info["after"])
                    self.lbl_pulse.configure(text="脉宽 %d us" % info["pulse"])
                self._refresh_sel()

            elif k == "select":
                idx = name_to_index(info["sel"])
                if idx >= 0:
                    self.sel = idx
                    self._refresh_sel()

            elif k == "step":
                self.var_step.set(str(info["step"]))

        self.root.after(50, self._drain_queue)

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
        for b in (self.btn_plus, self.btn_minus, self.btn_home,
                  self.btn_query, self.btn_abs):
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

    check("sel1", build_cmd("select", 1), "S1")
    check("sel3", build_cmd("select", 3), "S3")
    check("plus", build_cmd("plus"), "+")
    check("plus5", build_cmd("plus", 5), "+5")
    check("minus", build_cmd("minus"), "-")
    check("minus10", build_cmd("minus", 10), "-10")
    check("abs", build_cmd("abs", 135), "A135")
    check("abs clamp", build_cmd("abs", 999), "A270")
    check("step", build_cmd("step", 5), "N5")
    check("step clamp", build_cmd("step", 999), "N90")
    check("home", build_cmd("home"), "Z")
    check("query", build_cmd("query"), "Q")

    i = parse_line("[SERVO3] STATUS sel=LIFT step=5  ROT=30 GRIP=85 LIFT=95")
    check("status kind", i.get("kind"), "status")
    check("status sel", i.get("sel"), "LIFT")
    check("status lift", i["ang"]["LIFT"], 95)
    check("status step", i.get("step"), 5)

    i = parse_line("[SERVO3] UP   LIFT 90 -> 95   pulse=1204us")
    check("move kind", i.get("kind"), "move")
    check("move after", i.get("after"), 95)
    check("move pulse", i.get("pulse"), 1204)
    check("move id", i.get("id"), "LIFT")

    check("sel line", parse_line("[SERVO3] 选中 GRIP（PC8），当前 85°").get("sel"), "GRIP")
    check("step line", parse_line("[SERVO3] 步长 -> 10°").get("step"), 10)
    check("unknown", parse_line("[SERVO3] HOST> S1"), {})

    check("name idx", name_to_index("GRIP"), 1)
    check("name bad", name_to_index("XXX"), -1)

    if fails:
        print("SELFTEST FAILED:")
        for f in fails:
            print("  -", f)
        return 1
    print("SELFTEST OK（%d 项）" % 26)
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
    ServoApp(root)
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
