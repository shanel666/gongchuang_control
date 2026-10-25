#ifndef CHASSIS_H
#define CHASSIS_H

/* ============================================================================
 * chassis.h —— 速度模式底盘（F6）对外接口
 * ----------------------------------------------------------------------------
 * 单位约定（和协议一致，改的时候不容易搞混）：
 *   速度     0.1 RPM（例如 300 = 30.0 RPM）
 *   加速度   RPM/s
 *   平移量   mm（>0 前进 / 左移，<0 后退 / 右移）
 *   旋转量   度（>0 逆时针 CCW，<0 顺时针 CW）
 *
 * 用法（阻塞式，和旧的 qianjin_/houtou_ 一样，发完命令等动作做完才返回）：
 *     Chassis_Init();
 *     Chassis_Move(1000);        // 前进 1000 mm
 *     Chassis_Strafe(-250);      // 右平移 250 mm
 *     Chassis_Rotate(90);        // 逆时针 90°
 *     Chassis_Stop();
 *
 * 想连续控制（遥控 / 上层自己闭环）就用 Chassis_SetWheelSpeed()，
 * 它不做运动学解算，四个轮子的 0.1 RPM 有符号速度直接下发。
 * ========================================================================== */

#include <stdint.h>

/* 六个基本动作。方向表在 chassis_config.h 里，改成别的动作组合也只需要加一行。 */
typedef enum {
    CHASSIS_ACTION_FORWARD = 0,   /* 前进：参数 mm */
    CHASSIS_ACTION_BACKWARD,      /* 后退：参数 mm */
    CHASSIS_ACTION_LEFT,          /* 左平移：参数 mm */
    CHASSIS_ACTION_RIGHT,         /* 右平移：参数 mm */
    CHASSIS_ACTION_CCW,           /* 逆时针：参数 度 */
    CHASSIS_ACTION_CW,            /* 顺时针：参数 度 */
    CHASSIS_ACTION_COUNT
} ChassisAction;

/* 中止检查回调：返回非 0 表示立即停车。定时运动每 CHASSIS_WAIT_SLICE_MS
 * 调一次，用来接 PA8 之类的按键。传 0 取消检查。 */
typedef uint8_t (*ChassisAbortFn)(void);

/* 初始化：设默认加速度 + 使能 4 个电机（上电默认失能，不动先查这里） */
void Chassis_Init(void);

/* 使能 / 失能 4 个电机（F3 帧），enable=1 使能 */
void Chassis_Enable(uint8_t enable);

/* 四轮速度清零。速度模式下这是"按当前加速度减速停车"，不是硬刹车。 */
void Chassis_Stop(void);

/* 改默认加速度（RPM/s）；传 0 恢复 CHASSIS_ACCEL_DEFAULT */
void Chassis_SetAccel(uint16_t accel_rpm_s);

/* 装中止检查回调（见 ChassisAbortFn） */
void Chassis_SetAbortCheck(ChassisAbortFn fn);

/* 连续速度：speed_x10[FL, FR, RL, RR]，单位 0.1 RPM，带符号（负 = 该轮反向）。
 * 立即执行、不等待、不做运动学解算。返回 1 表示已下发。 */
uint8_t Chassis_SetWheelSpeed(const int32_t speed_x10[4]);

/* 执行一个动作并阻塞到预计结束（含停车和停稳等待）。
 *   action        CHASSIS_ACTION_*
 *   amount        平移量 mm 或旋转量 度（由 action 决定量纲）
 *   speed_x10     0 = 用该动作的默认速度
 *   accel_rpm_s   0 = 用默认加速度
 * 返回 1 完成 / 0 参数非法或被中止。 */
uint8_t Chassis_Run(ChassisAction action, uint32_t amount,
                    uint16_t speed_x10, uint16_t accel_rpm_s);

/* 按时间跑一个动作：发速度 → 跑 ms 毫秒 → 停车 → 等停稳。
 *   speed_x10 = 0 用默认速度，单次最长 CHASSIS_MAX_RUN_MS。
 * 无线/上位机手动测试用这个（例如"前进 1 s"）：Chassis_RunMs(FORWARD, 1000, 0)。
 * 返回 1 完成 / 0 参数非法或被中止。 */
uint8_t Chassis_RunMs(ChassisAction action, uint32_t ms, uint16_t speed_x10);

/* 只算不发：估算一次动作的时间（ms），用于调试和标定 */
uint32_t Chassis_EstimateMs(ChassisAction action, uint32_t amount, uint16_t speed_x10);

/* 常用封装：默认速度 */
uint8_t Chassis_Move(int32_t mm);            /* >0 前进，<0 后退 */
uint8_t Chassis_Strafe(int32_t mm);          /* >0 左移，<0 右移 */
uint8_t Chassis_Rotate(int32_t deg);         /* >0 逆时针，<0 顺时针 */

/* 常用封装：指定速度（0.1 RPM） */
uint8_t Chassis_MoveAt(int32_t mm, uint16_t speed_x10);
uint8_t Chassis_StrafeAt(int32_t mm, uint16_t speed_x10);
uint8_t Chassis_RotateAt(int32_t deg, uint16_t speed_x10);

/* 底层逃生口：按当前帧格式原样发一帧 F6（不做极性/方向表处理），
 * 以及发同步释放帧 00 FF 66 6B。以后加别的功能码可以照着改。 */
void Chassis_SendSpeed(uint8_t addr, uint8_t dir, uint16_t accel_rpm_s,
                       uint16_t speed_x10, uint8_t sync);
void Chassis_SendSync(void);

#endif /* CHASSIS_H */
