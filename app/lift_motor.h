#ifndef __LIFT_MOTOR_H
#define __LIFT_MOTOR_H

#include <stdint.h>

/* ============================================================================
 * 升降步进电机（张大头 Emm42，总线地址 6）
 * ----------------------------------------------------------------------------
 * 位置模式（FD），**绝对位置**：目标 = 从上电位置算起的脉冲数。
 * 上电时机械臂在最低位限位，所以"抬起来 9000 脉冲"就是直接发 9000。
 *
 * 用法：
 *     motor_enable_addr(LIFT_MOTOR_ADDR);      // 地址 6 上电默认失能，先使能
 *     lift_motor_move(9000u, 0x01u);           // 0x01 = CCW = 升
 *     lift_motor_stop();                       // 急停（FE 98 帧）
 *
 * 原来这两个函数在 main.c 里，vision 和夹取流程都要用，所以单独成文件。
 * ========================================================================== */

#define LIFT_MOTOR_ADDR       0x06u   /* 升降步进电机地址 */

/* ★ 上升 / 下降用**不同速度**。
 *   现场现象："下降总是超过零点，上升没事" —— 这是垂直轴受重力的典型特征：
 *   往下走时负载被重力加速，减速段需要更大的制动转矩，速度越快越容易冲过头。
 *   所以下降速度单独降一档。要调就改这两个数（越慢越不容易冲，但越费时间）。 */
#define LIFT_MOTOR_SPEED      40u     /* 上升速度 RPM */
#define LIFT_MOTOR_SPEED_DOWN 20u     /* 下降速度 RPM（比上升慢一半） */

/* ★ 软着陆：下降最后一段用爬行速度走，减小到底部的冲击/过冲。
 *   SOFT_ZONE = 从目标往上的多少脉冲开始切成爬行速度。 */
#define LIFT_MOTOR_SPEED_CREEP 8u     /* 爬行速度 RPM */
#define LIFT_SOFT_ZONE         300u   /* 最后 300 脉冲用爬行速度 */

/* 步进电机一圈的脉冲数（用来把 RPM 换算成"每 1000 脉冲要多久"）。
 * 张大头 Emm42：200 步/圈 × 16 细分 = 3200。 */
#define LIFT_PULSES_PER_REV    3200u

#define LIFT_MOTOR_ACCEL      80u     /* 加速度档（原来 150；越小斜坡越缓，起步电流越小） */

/* ★★★ 2026-11-21 现场定案：所有移动都发**相对帧**（脉冲字段 = 要走多少）★★★
 *   现象（用户实测）：上位机一直点"↑"，电机是一点点升的；一点"↓"就降一大段、
 *   直接堵转。把两次观察放在一起只有一个解释：
 *       这个驱动器把脉冲字段当成 **"要走多少"（相对量）**，
 *       而我以前发的是 **"走到哪个坐标"（绝对量）**。
 *     · 上升：第 1 次发 100 → 走 100；第 2 次发 200 → 又走 200；第 3 次走 300……
 *             看起来"一点点升"，其实每次都在变大（往上不容易察觉）。
 *     · 下降：发的是"目标坐标"（比如 400），被当成"向下走 400" → 一大段 → 撞底。
 *
 *   所以现在 lift_motor_move() 里统一换算成**差值**再发，方向位按大小关系填：
 *       cmd[6..9] = |目标 - 当前|      cmd[10] = 0x00（相对）
 *       cmd[2]    = 目标>当前 ? 0x01 : 0x00
 *   这样无论驱动器是"认相对帧"还是"其实在按绝对位置跑"，落点都是目标位置。
 *
 *     1 = 上下都用相对帧（★当前，修的就是这个问题）
 *     0 = 老行为：上升绝对帧、下降也绝对帧（只用来对比） */
#define LIFT_USE_REL_FRAME    1u
#define LIFT_MOTOR_HOME       0u      /* 上电位置 = 绝对 0 */
#define LIFT_MOTOR_UP         2000u   /* 夹取流程里的"抬升"量（vision 用 9000，见 VISION_LIFT_UP） */

/* ★ 软上下限（命令侧永远不会超出）。2026-11 实测：让位 ≈7600、抓料 ≈4000。 */
#define LIFT_MOTOR_MAX        8000u
#define LIFT_MOTOR_MIN        0u

/* 行程时间估算（调用方用它决定"等多久才发下一条"）。
 * 上升 600 ms/1000P（≈40 RPM），下降 1100 ms/1000P（≈20 RPM），各留 400 ms 余量。 */
#define LIFT_MS_PER_1000P_UP    600u
#define LIFT_MS_PER_1000P_DOWN 1100u
#define LIFT_MOVE_EXTRA_MS      400u
#define LIFT_MOVE_MS_CAP       9000u

/* 估算 from → to 这一段要走多久（ms），按方向自动选上升/下降速度。 */
uint16_t lift_motor_move_ms(uint32_t from, uint32_t to);

/* 同上，但**指定速度**（爬行段用）。rpm 传 0 表示用默认上升速度。 */
uint16_t lift_motor_ms_for(uint32_t from, uint32_t to, uint8_t rpm);

/* ★ 柔和下降的分段规划。
 *   长距离下降拆成两段：先以 SPEED_DOWN 走到 target+SOFT_ZONE，
 *   再以 SPEED_CREEP 走完最后 SOFT_ZONE 脉冲 —— 底部减速余量足，不容易硬顶。
 *   返回 1 = 要分两段（*mid 填中继点）；返回 0 = 一段走完就行。
 *   前提：target < from（下降）且行程 > SOFT_ZONE。 */
uint8_t lift_motor_soft_split(uint32_t from, uint32_t to, uint32_t *mid);

/* 用指定速度走到绝对位置。lift_motor_move() 内部就是调它。 */
void lift_motor_move_at(uint32_t abs_pulses, uint8_t speed_rpm);

/* ============================================================================
 * ★ 运行期覆盖「速度 / 加速度」—— 现场排查下降堵转专用
 * ----------------------------------------------------------------------------
 * 头文件里的 LIFT_MOTOR_SPEED / SPEED_DOWN / ACCEL 是编译期常量，改一次要重烧。
 * 排查堵转时需要在同一套机械上连续试 20 / 15 / 12 / 8 / 5 RPM 和不同加速度档，
 * 所以这里给一组运行期开关（只改 RAM，掉电恢复默认）：
 *
 *     lift_motor_set_speed_ovr(12);   // 之后所有位置移动都用 12 RPM
 *     lift_motor_set_speed_ovr(0);    // 0 = 取消覆盖，回到"上升40/下降20"
 *     lift_motor_set_accel(40);       // 加速度档，0 = 恢复默认 LIFT_MOTOR_ACCEL
 *
 * ★ 覆盖值同时作用于**时间估算**（lift_motor_ms_for），否则等待时间会和实际
 *   动作对不上，位置校验会误判成"没走到"。
 * ========================================================================== */
void    lift_motor_set_speed_ovr(uint8_t rpm);
uint8_t lift_motor_get_speed_ovr(void);
void    lift_motor_set_accel(uint8_t accel);
uint8_t lift_motor_get_accel(void);

/* ★ 把**当前位置设为驱动器零点**（Emm42 的 0x93 0x88 指令）。
 *   用途：机械臂手动降到最低位后调一次，之后"0"就等于机械底，
 *         绝对位置不会再整体偏移。调用后 g_lift_target 归 0。 */
void lift_motor_set_zero(void);

/* ★ 驱动方式：**绝对位置**（2026-11 现场确认）。
 *   现象判据：同一目标值连发多次只有第一次会动，改了数值才会再动 → 绝对定位。
 *   所以帧里直接填目标绝对位置（byte[10] = 0x01）。
 *   外部调用照旧写绝对值（7600 / 1000 / 0 这些标定值直接用）。 */
extern uint32_t g_lift_target;

/* 走到绝对位置 abs_pulses。非阻塞；会被夹到 [LIFT_MOTOR_MIN, LIFT_MOTOR_MAX]。
 * dir 参数已废弃（方向由目标与当前位置的关系自动填），保留只是为了不改老调用点。 */
void lift_motor_move(uint32_t abs_pulses, uint8_t dir);

/* 发一条**相对**位置命令（帧里 byte[10] = 0x00）。
 * 用途：万一驱动器其实没在做绝对定位，用这个对比一次就能确定。
 * pulses 会被夹到 LIFT_MOTOR_MAX。 */
void lift_motor_move_rel(uint32_t pulses, uint8_t dir);

/* 立即停止（0xFE 0x98 帧），中止时用，别让它继续往目标位置走 */
void lift_motor_stop(void);

/* 读驱动器**实际位置**（0x36 指令，8 字节回包）。
 *   返回 1 = 读到了，*out 是当前位置（可能为负，说明驱动器认为自己在零点下面）；
 *   返回 0 = 没回包 / 帧格式不对（这时 *out 不动）。 */
uint8_t lift_motor_read_pos(int32_t *out);

/* ★ 用驱动器实际位置校准我们跟踪的 g_lift_target。
 *   在每次"走到绝对位置"之前调一次，可以自动纠正：
 *     - 上一次移动因为等待时间不够被下一条命令打断
 *     - 驱动器丢步
 *     - 上电时机械臂不在最低位
 *   只有当差值超过 500 脉冲才真正改（避免噪声把跟踪值抖来抖去）。 */
void lift_motor_sync(void);

/* 读一次实际位置并打一行无线串口（含原始 8 字节，方便看帧对不对）。
 * 返回 1 = 读到了。 */
uint8_t lift_motor_report(void);

#endif /* __LIFT_MOTOR_H */
