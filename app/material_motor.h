#ifndef __MATERIAL_MOTOR_H
#define __MATERIAL_MOTOR_H

#include <stdint.h>

/* ============================================================================
 * 物料电机（张大头 Emm42，总线地址 5）—— 分度盘 / 转盘
 * ----------------------------------------------------------------------------
 * 位置模式（FD），相对位置：每次发"转多少脉冲"，驱动器自己累加。
 * 按键：PA12 = 转一个分度（MATERIAL_DEG_PER_STEP），PA11 = 中途叫停。
 *
 * 这个模块是从 main.c 里整块搬出来的（逻辑未改），包括宏、静态量和 7 个函数。
 * ========================================================================== */

#define MATERIAL_MOTOR_ADDR      0x05u   /* 物料电机驱动器地址 */
#define MATERIAL_MOTOR_DIR       0x00u   /* 0x00 正转 / 0x01 反转（转反了就改这里） */
#define MATERIAL_MOTOR_SPEED     100u    /* 速度 RPM：0~3000，越大越快 */
#define MATERIAL_MOTOR_ACCEL     200u    /* 加速度档 0~255（0 = 不用曲线加减速） */
#define MATERIAL_PULSES_PER_REV  3200u   /* 实测值：物料轴转一整圈的脉冲数 */
#define MATERIAL_DEG_PER_STEP    60u     /* ★ 每次按 PA12 转多少度 */
#define MATERIAL_DEG_FULL        360u    /* 一整圈的角度，只用来做换算 */

/* 标定界面状态（第 4 行） */
#define MAT_CAL_IDLE       0u   /* 等按键 */
#define MAT_CAL_MOVING     1u   /* 正在转 */
#define MAT_CAL_DONE       2u   /* 驱动器回到位、已停 */
#define MAT_CAL_NO_ANSWER  3u   /* 0x3A 没回包：驱动器不在线或关了应答 */
#define MAT_CAL_ABORT      4u   /* PA11 中途叫停 */

#define MAT_CAL_ROUND_MS   20u  /* 一次"查询 + 等待"大约 20 ms，用来算超时轮数 */

/* 角度 → 脉冲数（按 MATERIAL_PULSES_PER_REV / MATERIAL_DEG_FULL 换算） */
uint32_t material_deg_to_pulses(uint32_t deg);

/* 发一条相对位置命令（转 pulses 个脉冲），非阻塞 */
void material_motor_move(uint32_t pulses, uint8_t dir);

/* 立即停止（0xFE 0x98 帧） */
void material_motor_stop(void);

/* 按脉冲数估算"要走几轮 0x3A 查询" */
uint16_t material_calib_rounds(uint32_t pulses);

/* 等物料电机到位：1 = 到位，MAT_CAL_* 之一表示别的结果 */
uint8_t material_motor_wait_arrive(uint16_t max_rounds);

/* 画标定界面（第 1 行角度、第 2 行本次脉冲、第 3 行累计、第 4 行状态） */
void material_calib_draw(uint8_t state);

/* 物料电机分度标定主循环（PA12 转一步 / PA11 中止） */
void material_calib_test(void);

#endif /* __MATERIAL_MOTOR_H */
