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
#define LIFT_MOTOR_SPEED      40u     /* 速度 RPM。60 太快，电流冲击大（会把 K230 拉重启），
                                       * 降到 40；9000 脉冲约 4.2 s 走完 */
#define LIFT_MOTOR_ACCEL      80u     /* 加速度档（原来 150；越小斜坡越缓，起步电流越小） */
#define LIFT_MOTOR_HOME       0u      /* 上电位置 = 绝对 0 */
#define LIFT_MOTOR_UP         2000u   /* 夹取流程里的"抬升"量（vision 用 9000，见 VISION_LIFT_UP） */

/* 升降电机当前被命令到的绝对位置（脉冲）。
 * 只用于显示和"已经在目标高度就跳过"的判断，不读驱动器实际位置。 */
extern uint32_t g_lift_target;

/* 发一条绝对位置命令。dir: 0x01 = CCW = 升，0x00 = CW = 降。
 * 非阻塞：发完就返回（5 ms 帧间隔），到位要靠调用方等待。 */
void lift_motor_move(uint32_t abs_pulses, uint8_t dir);

/* 立即停止（0xFE 0x98 帧），中止时用，别让它继续往目标位置走 */
void lift_motor_stop(void);

#endif /* __LIFT_MOTOR_H */
