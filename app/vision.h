#ifndef __VISION_H
#define __VISION_H

#include <stdint.h>

/* ============================================================================
 * 视觉闭环（主控 <-> K230）—— v2：会真的动电机，注意安全
 * ----------------------------------------------------------------------------
 * PB1：切换模式（IDLE → CIRCLE → LINEAR → CIRCLE …）
 * PA7：点动自检（前进→后退→左移→右移→顺时针→逆时针 轮流各一次）
 * PA5：停止（含电机急停）
 *
 * 【两种模式都先举高】
 *   进 CIRCLE 或 LINEAR 之前，都把升降电机（地址 6）抬到 VISION_LIFT_UP = 9000 脉冲，
 *   方向 0x01（CCW = 升）——机械臂低了摄像头看不见圆/环。
 *   上电时升降在最低位限位，所以 9000 = 从最低点抬起来。
 *   按 60 RPM 约 2.8 s。已经在目标高度就跳过，不再等一遍。
 *
 * 【CIRCLE 模式】把圆心拉到"夹爪位置"
 *   K230 回 C<x>,<y>,<r>，主控算 dx = x - VISION_TARGET_X、dy = y - VISION_TARGET_Y，
 *   然后平移补偿（2026-11-17 现场实测的方向关系）：
 *       小车前进 -> 圆心 x 变大   =>  dx>0 走**后退**，dx<0 走前进
 *       小车右移 -> 圆心 y 变大   =>  dy>0 走**左移**，dy<0 走右移
 *   |dx|、|dy| 都 ≤ VISION_DEADBAND_PX 就算对准，不动。
 *   像素→毫米沿用老标定：60 px = 35 mm，再打 VISION_STEP_GAIN_PCT 折扣，
 *   单步绝不超过 VISION_MAX_STEP_MM = 10 mm。
 *
 * 【为什么要慢 + 打折】位置模式是"发脉冲 → 硬起硬停"，每步一次冲击，会"抽一下"、
 *   把 K230 晃掉线、电源抖。所以平移/旋转都走速度模式（F6）：
 *   VISION_SPEED_X10 = 16 (1.6 RPM ≈ 6.6 mm/s，是原来 8 RPM 的 1/5)、
 *   VISION_ACCEL_RPM_S = 120，软起软停；步长再打 VISION_STEP_GAIN_PCT = 50% 折扣。
 *   识别不准时"补一半"比"补满"稳得多：补满必然冲过头，来回摆。
 *   想退回位置模式把 VISION_USE_SPEED_MODE 改 0。
 *
 * 【LINEAR 模式】看三环 → 修车身偏航
 *   发 "linear"，K230 回 L<k>,<psi>；psi 单位 0.01°：
 *   3) psi 单位 0.01°：正 = 顺时针（motor_rotate_centideg dir=0x00），
 *      负 = 逆时针（dir=0x01）。|psi| ≤ VISION_PSI_DEADBAND 就算对准。
 *      单次最多转 VISION_PSI_MAX_STEP，防止某一帧误检把车甩飞。
 *   4) 连着两次"越修越大"就判定符号反了 / 目标丢了：停下来报警，不再动。
 *
 * 【画面中心 320,240 与"夹爪偏置"】
 *   K230 采集是 640×480，图像中心恒为 (320,240)（不是 0,0）。
 *   但摄像头不在夹爪正上方，所以程序不把圆往 320,240 拉，而是往
 *   **VISION_TARGET_X / VISION_TARGET_Y**（夹爪投影到画面的像素位置）拉。
 *   这两个宏就在下面，标定方法写在它们前面的注释里 —— 要调偏置只改这两个数。
 *   ★ 若把 K230 分辨率改成 QVGA(320×240)，中心变 (160,120)，目标点也要跟着改。
 *
 * OLED：
 *   1 行  模式 + 状态（OK / WAIT / TMO）
 *   2 行  CIRCLE: X:318 Y:242      LINEAR: K+0.04 P+1.92
 *   3 行  N:<序号> NG:<次数>
 *   4 行  最后一次动作，例如 [BACK 12mm] / [CW 3.50deg] / [LIFT 10000]
 *
 * 安全：所有电机动作都带超时（judge_wait 有轮数上限），PA5 随时急停；
 *       举高过程（0→10000 约 7 s）内也每 10 ms 检查一次 PA5。
 * ========================================================================== */
#define VISION_IDLE        0u
#define VISION_CIRCLE      1u
#define VISION_LINEAR      2u
#define VISION_QUIET_MAX   250u     /* 每个循环约 4 ms，250 × 4 ms ≈ 1 s */

/* ============================================================================
 * ★★★ 摄像头 / 夹爪偏置：你要改的就是下面这两个数 ★★★
 * ----------------------------------------------------------------------------
 * 摄像头不在夹爪正上方，所以"圆心在画面正中"并不等于"夹爪在圆心正上方"。
 * 做法很简单：把目标点从画面中心挪到"夹爪在画面里的位置"。
 *
 *   VISION_TARGET_X / VISION_TARGET_Y = 夹爪投影到画面上的像素坐标
 *
 * 【怎么标定（最省事的办法）】
 *   1) 手推小车，让**夹爪正好对准圆心**（人眼对）；
 *   2) 这时看 K230 那边报的圆心坐标（IDE 打印或 OLED 第 2 行的 X、Y）；
 *   3) 把这两个数直接抄进 VISION_TARGET_X / VISION_TARGET_Y，重新烧录。
 *   之后程序会把圆心往这个点上拉，而不是往画面正中拉。
 *
 * 【一定要用毫米算的话】
 *   像素差 = 毫米 * 60 / 35（沿用"60 px = 35 mm"的老标定，但这是在某个相机
 *   高度下标出来的，高度一变比例就变，所以还是上面那个"抄坐标"的办法最准）。
 *   例：夹爪在摄像头右边 20 mm，标定出来约为 34 px，就把 X 改成 320 + 34。
 *
 * ★ 符号自己不用推：以 K230 报的坐标为准，正负它自然就对了。
 * ========================================================================== */
#define VISION_TARGET_X    470      /* ← 夹爪在画面里的 X（现场实测：470） */
#define VISION_TARGET_Y    170      /* ← 夹爪在画面里的 Y（现场实测：170） */

/* 画面中心（只用来显示"偏离多少"做参考，以及算默认目标点） */
#define VISION_CX          320
#define VISION_CY          240

/* ---- CIRCLE：把圆心拉到目标点 ---- */
/* 死区放宽到 15 px：识别本身有抖动，死区太小会一直来回修（看起来像走过头）。 */
#define VISION_DEADBAND_PX   15     /* ±15 px 以内不动（≈9 mm） */
#define VISION_MM_PER_60PX   35u    /* 老标定：60 px = 35 mm */
#define VISION_MAX_STEP_MM   6u     /* ★ 单步最多 6 mm：一点点挪，但别太碎 */
/* ★ 步长增益：只补"理论距离"的这个百分比。
 *   识别不准 + 相机慢（3 Hz）时，一次补满必然冲过头，所以打个折：
 *   这次补一半，剩下的下次再补 —— 收敛慢一点，但不会冲过去来回摆。
 *   走得过冲就调小（40）；半天不到位就调大（70）。 */
#define VISION_STEP_GAIN_PCT 50u
/* ★ 每走一步之后停多久（ms）：
 *   1) 等车体停稳、等 K230 拍下一帧清晰的；
 *   2) 让电源电压恢复 —— 连续动作的电流冲击叠起来最容易把 K230 拉到重启。
 *   这个停顿**不阻塞主循环**：照样收帧、照样刷屏，只是暂时不发新命令。 */
#define VISION_STEP_GAP_MS   150u
/* 梯形加减速本身要花的时间余量（估"这一步还要多久"时加上）。
 * ★ 必须和 VISION_POS_ACCEL 一起改：加速度越小，斜坡越长。
 *   accel=20 时 120 ms 够用；accel=8 要 400 ms 左右，否则会"上一步还在爬坡就发下一条"。 */
#define VISION_RAMP_MS       400u
/* 主循环一拍的周期（ms）：Delay_ms(4) + 按键扫描 + 偶尔刷屏。
 * 用来把"这一步要多少 ms"换算成"要等多少拍"。 */
#define VISION_LOOP_MS       4u

/* 方向映射（现场实测，翻了两次才对，记在这里别再改回去）：
 *   dx>0  ->  **前进**   （第一次按"前进让 x 变大"推成后退，实测反了；
 *                          2026-11-17 二次实测：dx>0 要走前进）
 *   dy>0  ->  **右移**   （同理，实测右移才对）
 * 注意这两个宏和 motor.c 里"前进/后退"的字面含义无关：
 *   如果哪天发现"日志说 BACK、车却往前走"，那说明 motor.c 的方向表反了
 *   （改 MOTOR_FLIP_FWD），不是这两个宏的事。
 *   **判断方法**：看串口那一行 `-> BACK 3mm`，对照车实际往哪走。 */
#define VISION_DX_PLUS_DO_BACK   0  /* 0: dx>0 前进 / dx<0 后退（实测值） */
#define VISION_DY_PLUS_DO_RIGHT  1  /* 1: dy>0 右移 / dy<0 左移（实测值） */
/* 旋转方向：psi>0 我们约定是"顺时针"，如果实测越修越大（发散），把它改 1 翻过来。
 * 另外还有一道保险：连续两轮 |psi| 变大就 DIVERGE STOP 停车，不会一直转。 */
#define VISION_YAW_INVERT        0  /* 0 = psi>0 顺时针；1 = psi>0 逆时针 */

/* ============================================================================
 * 运动方式：位置模式（FD）+ 梯形加减速  ← 当前方案
 * ----------------------------------------------------------------------------
 * 速度模式（F6）在这台车上晃得厉害，而且它的加减速由驱动器内部管，
 * 不好精确控制斜坡；位置模式是"发固定脉冲 + 指定速度和加速度档"，
 * 加减速由驱动器按梯形曲线走，速度/加速度都能压到很低，动作最干净。
 *
 * 三个量一起压低才有效（缺一个都会抽）：
 *   VISION_POS_SPEED_RPM  速度 8 RPM（默认 70）
 *   VISION_POS_ACCEL      加速度档 20（默认 200，越小斜坡越缓 = 梯形越平）
 *   VISION_MAX_STEP_MM    每次最多走 4 mm
 * 想再慢：speed 降到 5、accel 降到 10；但太小可能带不动（听到声音不走）。
 * ========================================================================== */
#define VISION_USE_SPEED_MODE   0     /* 0 = 位置模式（当前）；1 = F6 速度模式 */
#define VISION_POS_SPEED_RPM    8u    /* 位置模式速度，RPM */
#define VISION_POS_ACCEL        8u    /* 位置模式加速度档 0~255。20 → 8：
                                       * 斜坡更缓、起步电流更小，代价是"到位"更慢 */

/* 下面三个只在 VISION_USE_SPEED_MODE = 1 时才用（F6 速度模式那套参数） */
#define VISION_SPEED_X10        16u   /* 平移速度，0.1RPM：16 = 1.6 RPM */
#define VISION_ROT_SPEED_X10    20u   /* 旋转速度，0.1RPM：20 = 2.0 RPM */
#define VISION_ACCEL_RPM_S      120u  /* 加速度 RPM/s */

/* ---- 举高（两个模式都用）---- */
/* 升降电机的目标绝对位置（脉冲）。上电时升降在最低位限位，所以这就是"抬起来多少"。
 * 9000 脉冲按 60 RPM 约 2.8 s 走完。 */
#define VISION_LIFT_UP            9000u
/* 每 1000 脉冲大约要等多久：60 RPM ≈ 0.32 s，取 0.35 留余量。
 * ★ 这个值和 LIFT_MOTOR_SPEED 是绑在一起的，改那个就得改这个，否则等待和实际对不上。
 * 等待时间按"当前位置→目标位置"的差值算，已经在高度上就不等。 */
#define VISION_LIFT_MS_PER_1000P  550u
#define VISION_LIFT_WAIT_MAX      6000u  /* 单次最多等 6 s */
#define VISION_PSI_DEADBAND  60     /* 0.01°：|psi| ≤ 0.6° 算对准 */
#define VISION_PSI_MAX_STEP  2000   /* 单次最多 20° */
#define VISION_ALIGN_ROUNDS  8u     /* 最多调几轮 */
#define VISION_JUDGE_ROUNDS  60u    /* judge_wait 轮数上限（一轮约 220 ms） */

/* ---- K230 掉线自恢复 ----
 * K230 被电源拉重启后，会忘掉当前模式、停止回数据，主控这边只看到 TMO。
 * 静默超过 1 s 就自动重发一次模式命令，让它自己接上（不用人按按键）。
 * 每 VISION_RESEND_GAP 拍重发一次，最多 VISION_RESEND_MAX 次。 */
#define VISION_RESEND_MAX    5u
#define VISION_RESEND_GAP    500u   /* 拍数，约 2 s */

/* 下面这些是 vision.c 内部的静态量和内部函数，不对外暴露：
 *   vision_last[5][17] / vision_last_valid   OLED 行缓存
 *   v_dbx / v_dby / v_dx_ok / v_calib_ticks  点动标定的结果
 *   vision_row() / vision_row_invalidate()   OLED 整行写 + 缓存
 *   vision_fmt_2dp() / vision_fmt_signed()   数字格式化
 *   vision_px_to_step()                      像素 → 步长（增益 + 上限）
 *   vision_move_fwd/lat/rotate()             三种基本动作
 *   vision_step_ms() / vision_rot_ms()       动作耗时估算
 */

/* ---------------- 动作层：这里才开始真的动电机 ---------------- */

/* ---------------- 三个"动一下"的封装：速度模式 / 位置模式在这里切换 ---------------- */


/* PA7 点动自检：每按一次做一种动作，轮流覆盖 前进/后退/左移/右移/顺时针/逆时针。
 * 用途：接视觉之前先确认"车到底动不动、每个方向对不对"。
 * 幅度都很小（50 mm / 15°），但轮子悬空时看不出方向，所以最好架空或垫起来试。
 * 返回 1 正常 / 0 电机没回报到位。 */
#define VISION_JOG_MM    50u
#define VISION_JOG_DEG   15u
#define VISION_JOG_COUNT 6u

/* ---- 对外接口 ---- */

/* 视觉模式主循环（DEBUG_MODE_VISION 时由 main() 调用，不返回） */
void vision_link_test(void);

/* 把圆心拉到"夹爪位置"（CIRCLE 模式的一步），非阻塞。
 * 返回这一步大概要多少 ms（0 = 已在死区）。 */
uint16_t vision_circle_issue(int32_t cx, int32_t cy, char *act);

/* 按 psi 修车身偏航（LINEAR 模式的一步），非阻塞。返回估算耗时 ms。 */
uint16_t vision_yaw_issue(int16_t psi_centi, char *act);

/* 六方向点动自检（PA7）。返回 1 = 电机有应答，0 = 没回 0x3A */
uint8_t vision_jog(uint8_t idx, char *act);

/* 举高机械臂到 target 脉冲（绝对位置）。返回 1 = 到位，0 = 被 PA5 中止 */
uint8_t vision_lift_to(uint32_t target, char *act);

/* 四轮 + 升降全急停 */
void vision_stop_all(void);

#endif /* __VISION_H */