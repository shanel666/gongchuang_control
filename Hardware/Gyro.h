#ifndef __GYRO_H
#define __GYRO_H

#include "stm32f10x.h"

/* JY901/WitMotion 兼容角度帧：55 53 ...，总长 11 字节。 */
#define GYRO_FRAME_LEN       11u
#define GYRO_FRAME_HEADER    0x55u
#define GYRO_TYPE_ANGLE      0x53u
#define GYRO_ANGLE_SCALE_NUM 18000L
#define GYRO_ANGLE_SCALE_DEN 32768L

/* 在 USART2 接收中断中调用一次，传入刚收到的一个字节。 */
void Gyro_ProcessByte(uint8_t byte);

/* 清空解析状态和上一次数据。 */
void Gyro_Init(void);

/* 返回最近一次校验通过的 Z 轴角度，单位为 0.01 度。 */
uint8_t Gyro_GetYawCentiDegrees(int32_t *centi_degrees);

/* 返回最近一次校验通过的 Z 轴原始有符号值。 */
uint8_t Gyro_GetYawRaw(int16_t *raw);

/* 最近一次有效帧的计数，可用于判断传感器是否正在输出。 */
extern volatile uint32_t Gyro_FrameCount;
extern volatile int32_t yaw_centi_degrees;

/* ============================================================================
 * 陀螺仪闭环转向（绝对角度；全图路线和"闭环旋转测试"共用同一份实现）
 * ----------------------------------------------------------------------------
 * 角度一律用"绝对角度"：开机时记一次零点 yaw_zero，之后 北=0°、西=+90°、
 * 南=180°、东=-90°（逆时针为正，和陀螺仪一致）。每次转向都是"转到某个绝对角度"，
 * 所以多段路线之间不会累积偏差。
 *
 * 修正方式：按当前误差直接算脉冲，一步到位，不再按比例一点点逼近：
 *     脉冲 = |误差| / 36000 * PULSE_PER_360DEG
 *   例：误差 10.00° → 17208/360×10 = 478 脉冲；误差 36.00° → 1721 脉冲。
 *   每次发完都等电机到位、等车体停稳，再读一次全局角度重算误差，
 *   最多补 TURN_MAX_CORR 次（误差按 1-标定误差 的比例衰减，标定差一半也能收敛）。
 *
 * 已经踩过的坑：
 *   - 补差必须换算成"度/脉冲"再发；把 0.01° 的数当度发，会每次被限幅成 30°（超调）。
 *   - 电机刚停时车体还在晃，必须等角度稳定再测，否则会反复过冲。
 *
 * 这几个函数原来在 main.c 里，因为"路线模式"和"旋转测试模式"都要用，
 * 搬到这里让陀螺仪相关的东西集中在一个文件。
 * ========================================================================== */

#define TURN_TOL_CENTI       10L    /* 到位容差 0.1°（单位 0.01°） */
#define TURN_MAX_CORR        15u    /* 补差次数上限（误差按 1-k 衰减，多给几次能兜住标定差） */
#define TURN_SETTLE_MS      100u    /* 等停稳的采样间隔 */
#define TURN_SETTLE_TOL      50L    /* 相邻两次采样差 < 0.5° 认为停稳 */
#define TURN_SETTLE_TRIES     5u    /* 最多等 5 轮（0.5 s） */

/* 收到过有效角度帧（1 = 陀螺仪在线） */
extern uint8_t gyro_ok;

/* 开机零点：所有绝对角度都相对它 */
extern int32_t yaw_zero;
extern uint8_t yaw_zero_ok;

/* 最近一次闭环转向的过程量（测试模式直接显示，路线模式用来复盘） */
extern volatile int32_t turn_dbg_target;   /* 目标绝对角度，0.01° */
extern volatile int32_t turn_dbg_meas;     /* 当前绝对角度，0.01° */
extern volatile int32_t turn_dbg_err;      /* 最后误差，0.01° */
extern volatile uint8_t turn_dbg_corr;     /* 补差次数 */
extern volatile uint8_t turn_dbg_result;   /* 1 到位 0 失败 */

/* 把角度差限制到 [-180, 180) 度（0.01° 单位） */
int32_t gyro_wrap_180(int32_t angle);

/* 等一帧新的有效角度，避免用旧数据重复计算 PD。返回 1 = 拿到新数据 */
uint8_t gyro_wait_new_sample(uint32_t *last_frame_count, int32_t *yaw);

/* 等车体停稳：间隔 100 ms 的两次角度差 < 0.5° 就算停稳，最多等 0.5 s。 */
uint8_t gyro_turn_settle(int32_t *yaw);

/* 抓一次开机零点航向角（yaw_zero / yaw_zero_ok） */
uint8_t yaw_zero_capture(void);

/* 闭环转到绝对角度（0.01°）。返回 1 = 到位 */
uint8_t gyro_turn_to_abs(int32_t target_abs);

#endif /* __GYRO_H */
