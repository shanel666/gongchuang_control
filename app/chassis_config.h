#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* ============================================================================
 * chassis_config.h —— 速度模式底盘（F6）的全部可调参数
 * ----------------------------------------------------------------------------
 * 这个文件是「改参数的地方」，chassis.c 里不再写死任何速度/加速度/标定值。
 *
 * 文件分工：
 *   chassis_config.h   参数（本文件）          ← 平时只改这里
 *   chassis.h          对外接口
 *   chassis.c          帧拼装 + 方向表 + 定时运动
 *
 * 和旧版 motor.c 的关系：
 *   motor.c   走 FD 位置模式（发脉冲，再用 0x3A 查到位），适合走精确距离；
 *   本模块    走 F6 速度模式（发速度，按时间停车），适合连续跑 / 需要加减速的场合。
 *   两套互不影响，可以同时编进工程。地址、单轮极性、整轴取反直接复用
 *   motor.h 里的 MOTOR_* 宏，不在本文件重复定义。
 * ========================================================================== */

#include "motor.h"

/* ---------------------------------------------------------------------------
 * 1. 帧格式（按驱动器手册选一种，只改一行）
 * ---------------------------------------------------------------------------
 * 9 字节（按字段说明：加速度 2 字节 + 速度 2 字节）：
 *     [地址][F6][方向][加速度H][加速度L][速度H][速度L][同步][6B]
 *
 * 8 字节（示例帧 01 F6 00 00 32 32 01 6B 正好能逐字节对上）：
 *     [地址][F6][方向][速度H][速度L][加速度][同步][6B]
 *
 * 说明：你给的字段表写「加速度 0000-FFFF、速度 0000-7E30」（2+2 字节，9 字节帧），
 *       但示例帧只有 8 字节，两者对不上。两种格式本模块都实现了，默认用 8 字节，
 *       因为示例帧按 8 字节解析正好是：速度 0x0032 = 5.0 RPM、加速度 0x32 = 50、
 *       同步 01。如果驱动器确实是 2 字节加速度的版本，把下面改成 9B 即可。
 * ------------------------------------------------------------------------- */
#define CHASSIS_FRAME_LAYOUT_8B   8
#define CHASSIS_FRAME_LAYOUT_9B   9

#ifndef CHASSIS_FRAME_LAYOUT
#define CHASSIS_FRAME_LAYOUT      CHASSIS_FRAME_LAYOUT_8B
#endif

#if (CHASSIS_FRAME_LAYOUT != CHASSIS_FRAME_LAYOUT_8B) && \
    (CHASSIS_FRAME_LAYOUT != CHASSIS_FRAME_LAYOUT_9B)
#error "CHASSIS_FRAME_LAYOUT must be CHASSIS_FRAME_LAYOUT_8B or CHASSIS_FRAME_LAYOUT_9B"
#endif

#define CHASSIS_FUNC_SPEED        0xF6u   /* 速度模式功能码 */
#define CHASSIS_FRAME_TAIL        0x6Bu   /* 固定校验码 */
#define CHASSIS_FUNC_ENABLE       0xF3u   /* 使能功能码 */

/* ---------------------------------------------------------------------------
 * 2. 默认速度 / 加速度
 *    速度单位 = 0.1 RPM（协议单位），上限 30000（3000.0 RPM）
 *    加速度单位 = RPM/s；8 字节帧里加速度只有 1 字节，超过 255 会被限幅
 *    换算参考：30.0 RPM ≈ 123 mm/s；40.0 RPM ≈ 45 °/s（按下面第 3 节的标定）
 * ------------------------------------------------------------------------- */
#define CHASSIS_SPEED_TRANSLATE    300u    /* 平移默认 30.0 RPM */
#define CHASSIS_SPEED_ROTATE       400u    /* 旋转默认 40.0 RPM */
#define CHASSIS_ACCEL_DEFAULT      200u    /* 默认 200 RPM/s */
#define CHASSIS_SPEED_MAX          30000u  /* 协议上限 0x7E30 */

/* ---------------------------------------------------------------------------
 * 3. 标定（改完这里，chassis 和 motor 两套逻辑一起生效）
 * ---------------------------------------------------------------------------
 *   CHASSIS_PULSES_PER_REV   驱动器细分：电机转一圈的脉冲数（张大头默认 3200）
 *   MOTOR_PULSE_PER_100CM    平移标定：车走 1000 mm 需要多少脉冲（motor.h）
 *   MOTOR_PULSE_PER_360DEG   旋转标定：车原地转 360° 需要多少脉冲（motor.h）
 *
 * 只有做「速度模式定时运动」时才需要 PULSES_PER_REV：
 *   电机转一圈车走多远  = PULSES_PER_REV * 1000 / PULSE_PER_100CM
 *   电机转一圈车转多少度 = PULSES_PER_REV * 360  / PULSE_PER_360DEG
 * 如果你的驱动器细分不是 3200，改这里；改完先用自检模式的第 4 页跑 1000 mm
 * 实测一下，差多少就调下面的 CHASSIS_TIME_GAIN_PCT。
 * ------------------------------------------------------------------------- */
#ifndef CHASSIS_PULSES_PER_REV
#define CHASSIS_PULSES_PER_REV     3200u
#endif

#define CHASSIS_PULSE_PER_1000MM   MOTOR_PULSE_PER_100CM
#define CHASSIS_PULSE_PER_360DEG   MOTOR_PULSE_PER_360DEG

/* 由上面三个数推出来的物理量，不要手改 */
#define CHASSIS_MM_PER_REV   ((float)CHASSIS_PULSES_PER_REV * 1000.0f / (float)CHASSIS_PULSE_PER_1000MM)
#define CHASSIS_DEG_PER_REV  ((float)CHASSIS_PULSES_PER_REV * 360.0f  / (float)CHASSIS_PULSE_PER_360DEG)

/* 单次动作的安全上限，防止误传大数把 Delay 拉爆 */
#define CHASSIS_MAX_MM             100000u
#define CHASSIS_MAX_DEG            36000u
#define CHASSIS_MAX_RUN_MS         60000u   /* Chassis_RunMs() 单次最长 60 s */

/* ---------------------------------------------------------------------------
 * 4. 动作方向表（顺序固定 FL, FR, RL, RR；0x00=正转/CW，0x01=反转/CCW）
 * ---------------------------------------------------------------------------
 * 数值和 motor.c 里现场实测的方向表一致（滚子是"同手"的，别照教科书改）。
 * 某条轴整体反了不要动这里，改 motor.h 的：
 *     MOTOR_FLIP_FWD（前后轴）  MOTOR_FLIP_LAT（左右轴）  MOTOR_YAW_SWAP（旋转）
 * 单个轮子反了，改 motor.h 的 MOTOR_DIR_INV_1..4（按地址号生效）。
 * ------------------------------------------------------------------------- */
#define CHASSIS_DIR_FORWARD   { 1u, 0u, 0u, 1u }
#define CHASSIS_DIR_BACKWARD  { 0u, 1u, 1u, 0u }
#define CHASSIS_DIR_LEFT      { 1u, 1u, 0u, 0u }
#define CHASSIS_DIR_RIGHT     { 0u, 0u, 1u, 1u }
#define CHASSIS_DIR_CCW       { 1u, 1u, 1u, 1u }
#define CHASSIS_DIR_CW        { 0u, 0u, 0u, 0u }

/* ---------------------------------------------------------------------------
 * 5. 时序与时间修正
 * ------------------------------------------------------------------------- */
#define CHASSIS_FRAME_GAP_MS       2u     /* 每帧之间留的间隔 */
#define CHASSIS_SYNC_GAP_MS        2u     /* 同步释放帧之后留的间隔 */
#define CHASSIS_STOP_SETTLE_MS     300u   /* 停车后再等车体停稳才返回 */
#define CHASSIS_WAIT_SLICE_MS      10u    /* 等待切片，用来插中止检查 */
#ifndef CHASSIS_USE_SYNC
#define CHASSIS_USE_SYNC           0u     /* 1=四轮等 00 FF 66 6B 统一起步 */
#endif

/* 定时运动的时间修正：
 *   实际走多了 → 把 GAIN 调小；走少了 → 调大。
 *   例：命令 1000 mm 实际走了 950 mm，就把 GAIN 改成 105（约 100*1000/950）。 */
#define CHASSIS_TIME_GAIN_PCT      100
#define CHASSIS_TIME_OFFSET_MS     0

#endif /* CHASSIS_CONFIG_H */
