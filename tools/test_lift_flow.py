# -*- coding: utf-8 -*-
'''
升降步进流程「帧级演算」—— 在电脑上把固件会发出的每一帧还原出来，不用硬件

=================================================================
为什么要这个
=================================================================
"下降总是超过零点"到底是代码问题还是机械/驱动器问题，靠现场反复试很难分。
这个脚本把 User/main.c 的 9 步流程 + app/lift_motor.c 的组帧逻辑 1:1 复制到电脑上，
把每一步真正会发出去的 13 字节帧打出来，一眼就能看出：

    - 脉冲字段填的是不是"目标绝对位置"
    - 方向位对不对
    - byte[10] 是不是 0x01（绝对）
    - 有没有哪一步算出离谱的值

如果这里算出来完全正确，那问题就在驱动器或机械，不在代码。

=================================================================
用法
=================================================================
    python tools/test_lift_flow.py

输出示例：
    [步骤 1] UP7600   目标=7600  dir=UP   帧: 06 FD 01 00 28 50 00 00 1D B0 01 00 6B
                                驱动器(绝对) -> 7600
=================================================================
'''

import sys

# ---- 与固件一致的常量（改一处这里也要改）----
LIFT_ADDR = 0x06
LIFT_SPEED_UP = 40
LIFT_SPEED_DOWN = 20
LIFT_ACCEL = 80
LIFT_MAX = 8000
LIFT_MIN = 0

MS_PER_1000P_UP = 600
MS_PER_1000P_DOWN = 1100
EXTRA_MS = 400
WAIT_CAP_MS = 9000

# 软着陆：下降最后 SOFT_ZONE 脉冲用爬行速度
SOFT_ZONE = 300
SPEED_CREEP = 8
PULSES_PER_REV = 3200

# 与 main.c 的 k_cp_steps[] 一致：(kind, 目标, 标签)
#   kind: "MOT" = 升降步进, "ROT"/"GRIP"/"LIFT" = 三路舵机
FLOW = [
    ("MOT",  7600, "UP7600"),   # 步进抬到高位
    ("ROT",   163, "ROT163"),   # 旋转舵机转到车上料仓
    ("LIFT",  130, "LFT130"),   # 升降舵机抬臂
    ("MOT",  1000, "DN1000"),   # 步进降到车上料
    ("GRIP",  110, "GRP110"),   # 夹爪夹住
    ("ROT",    30, "ROT30"),    # 旋转回放料侧
    ("LIFT",   90, "LFT90"),    # 升降舵机回初始
    ("MOT",     0, "DN0"),      # 步进回零
    ("GRIP",   85, "GRP85"),    # 夹爪松开
]


def u32_be(v):
    v &= 0xFFFFFFFF
    return [(v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF]


def build_frame(target, cur_track, byte10=0x01):
    """复刻 lift_motor_move()：方向按目标 vs 跟踪位置，脉冲字段填绝对目标，
    速度按方向取（上升 40 RPM / 下降 20 RPM）。"""
    t = max(LIFT_MIN, min(LIFT_MAX, target))
    direction = 0x01 if t >= cur_track else 0x00
    speed = LIFT_SPEED_UP if direction == 0x01 else LIFT_SPEED_DOWN
    frame = [LIFT_ADDR, 0xFD, direction,
             (speed >> 8) & 0xFF, speed & 0xFF,
             LIFT_ACCEL]
    frame += u32_be(t)
    frame += [byte10, 0x00, 0x6B]
    return frame, t, direction


def move_ms(frm, to, rpm=None):
    """复刻 lift_motor_move_ms / lift_motor_ms_for：理论时间 × 1.25 + 400ms。"""
    d = abs(frm - to)
    if rpm is None:
        rpm = LIFT_SPEED_DOWN if to < frm else LIFT_SPEED_UP
    per = max(1, 18750 // rpm)          # 每 1000 脉冲的耗时（理论值）
    return min(WAIT_CAP_MS, d * per * 5 // 4000 + EXTRA_MS)


def soft_split(frm, to):
    """复刻 lift_motor_soft_split：长距离下降拆两段，返回 mid 或 None。"""
    if to >= frm:
        return None
    if (frm - to) <= SOFT_ZONE:
        return None
    return to + SOFT_ZONE


def hexs(frame):
    return " ".join("%02X" % b for b in frame)


class Driver(object):
    """模拟绝对位置驱动器：记住自己的计数器，收到帧就去那个坐标。"""

    def __init__(self, start=0):
        self.pos = start
        self.log = []

    def move(self, frame):
        if frame[1] != 0xFD:
            return
        target = (frame[6] << 24) | (frame[7] << 16) | (frame[8] << 8) | frame[9]
        absolute = (frame[10] == 0x01)
        if absolute:
            self.pos = target
        else:
            self.pos += target if frame[2] == 0x01 else -target
        self.log.append((frame, self.pos))


def run_flow(verbose=True, stall_at=None, start_pos=0):
    """跑一遍 9 步流程，返回 (driver, per_step)。
    per_step = [(label, 目标, [这一步步进发出的帧...]), ...]，只含 MOT 步骤。"""
    drv = Driver(start_pos)
    track = start_pos            # g_lift_target
    per_step = []

    if verbose:
        print("=" * 78)
        print("升降步进电机：绝对位置模式帧演算")
        print("  地址=%d  速度: 上升 %d / 下降 %d RPM  加速度=%d  软限位=[%d, %d]"
              % (LIFT_ADDR, LIFT_SPEED_UP, LIFT_SPEED_DOWN, LIFT_ACCEL, LIFT_MIN, LIFT_MAX))
        print("=" * 78)

    for i, (kind, target, label) in enumerate(FLOW, 1):
        if kind != "MOT":
            if verbose:
                print("[步骤 %d] %-7s 舵机 -> %d°" % (i, label, target))
            continue

        # ---- lift_motor_sync() ----
        act = drv.pos
        act_c = max(LIFT_MIN, min(LIFT_MAX, act))
        if abs(act_c - track) > 500:
            if verbose:
                print("       [sync] 跟踪 %d -> 驱动器 %d" % (track, act))
            track = act_c

        if target == track:
            if verbose:
                print("[步骤 %d] %-7s 已经在 %d，不发帧" % (i, label, target))
            continue

        mid = soft_split(track, target)

        if mid is not None:
            # ---- 第 1 段：下降速度到 目标+SOFT_ZONE ----
            f1, t1, d1 = build_frame(mid, track, 0x01)
            ms1 = move_ms(track, mid, LIFT_SPEED_DOWN)
            # ---- 第 2 段：爬行速度收最后 SOFT_ZONE ----
            f2, t2, d2 = build_frame(target, mid, 0x01)
            ms2 = move_ms(mid, target, SPEED_CREEP)

            if verbose:
                print("[步骤 %d] %-7s 目标=%-5d 跟踪=%-5d  DOWN 两段软着陆"
                      % (i, label, target, track))
                print("            段1 %4d->%4d  等%4dms  帧: %s" % (track, mid, ms1, hexs(f1)))
                print("            段2 %4d->%4d  等%4dms  帧: %s   (%d RPM 爬行)"
                      % (mid, target, ms2, hexs(f2), SPEED_CREEP))

            drv.move(f1)
            drv.move(f2)
            track = t2
            last_frames = [f1, f2]
        else:
            ms = move_ms(track, target)
            frame, t_used, direction = build_frame(target, track, 0x01)

            if verbose:
                print("[步骤 %d] %-7s 目标=%-5d 跟踪=%-5d %s  等待=%4dms"
                      % (i, label, target, track, "UP  " if direction else "DOWN", ms))
                print("            帧: %s" % hexs(frame))

            drv.move(frame)
            track = t_used
            last_frames = [frame]

        if stall_at is not None and drv.pos < stall_at:
            if verbose:
                print("            ** 机械限位堵转：驱动器计数已是 %d，实际停在 %d **"
                      % (drv.pos, stall_at))
                drv.pos = stall_at

        if verbose:
            print("            驱动器(绝对) -> %d" % drv.pos)

        per_step.append((label, target, last_frames))

    return drv, per_step


def main():
    drv, per_step = run_flow()

    print("=" * 78)
    print("结果检查")
    print("=" * 78)

    fails = []

    # 1) 每一步：所有帧都必须是 byte10=0x01；**最后一帧**的脉冲 == 该步目标
    for label, want, frames in per_step:
        vals = [(f[6] << 24) | (f[7] << 16) | (f[8] << 8) | f[9] for f in frames]
        all_abs = all(f[10] == 0x01 for f in frames)
        ok = all_abs and (vals[-1] == want)
        tag = "OK"
        if not all_abs:
            tag = "!! byte10 不是 0x01"
        elif vals[-1] != want:
            tag = "!! 末帧 %d != 目标 %d" % (vals[-1], want)
        print("  %-7s 目标 %-5d 帧序列 %-16s byte10=0x%02X  %s"
              % (label, want, ",".join(str(v) for v in vals),
                 frames[-1][10], tag))
        if not ok:
            fails.append(label)

    # 2) 每一帧的方向位必须和它的位置变化方向一致
    for label, want, frames in per_step:
        prev = None
        for f in frames:
            val = (f[6] << 24) | (f[7] << 16) | (f[8] << 8) | f[9]
            if prev is not None:
                want_dir = 0x01 if val > prev else 0x00
                if f[2] != want_dir:
                    print("  %-7s 方向位错：%d->%d 发了 0x%02X 应为 0x%02X"
                          % (label, prev, val, f[2], want_dir))
                    fails.append(label + "-dir")
            prev = val

    # 3) 最终必须回到 0
    print()
    if drv.pos == 0:
        print("  ✅ 终态驱动器位置 = 0（正确回零）")
    else:
        print("  ❌ 终态驱动器位置 = %d（没有回零！）" % drv.pos)
        fails.append("final")

    # 4) 任何帧都不能超过软限位
    for label, want, frames in per_step:
        for f in frames:
            val = (f[6] << 24) | (f[7] << 16) | (f[8] << 8) | f[9]
            if val > LIFT_MAX:
                print("  ❌ %s 出现超过软限位的目标 %d" % (label, val))
                fails.append("over")

    print()
    if fails:
        print("结论：代码有问题 —— 上面这些步骤的帧不对：%s" % ", ".join(fails))
        return 1

    print("结论：**代码侧完全正确**。9 步流程发出的每一帧都是"
          "『绝对位置 + byte10=0x01 + 正确方向』，最终回到 0。")
    print()
    print("所以现场『下降超过零点』不是这套组帧逻辑的问题，往这两个方向查：")
    print("  A. 驱动器那边的零点和我们以为的不一样（做过「设零点」或断电后计数变化）")
    print("  B. 机械上：上电位置不是最低点，或者 0 点下方还有行程")
    print()
    print("最快的判别办法：跑 tools/lift_host.py，")
    print("  1) 点「回零 Z」，看机械臂停在哪、屏上 ACT 是几")
    print("  2) 如果 ACT 显示 0 但机械臂还在往下走 → 驱动器/机械问题，代码无关")
    print("  3) 如果 ACT 显示负数 → 驱动器自己知道过头了，说明它的零点定义变了")
    return 0


if __name__ == "__main__":
    sys.exit(main())
