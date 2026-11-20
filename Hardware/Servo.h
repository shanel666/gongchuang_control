#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

/*
 * 三路 270° 舵机 PWM（TIM3 完全重映射，20 ms 周期 / 50 Hz，500~2500 us = 0~270°）
 *
 *   PC7 = TIM3_CH2 = 升降舵机   SERVO_LIFT
 *   PC8 = TIM3_CH3 = 夹爪舵机   SERVO_GRIP
 *   PC9 = TIM3_CH4 = 旋转舵机   SERVO_ROT
 *
 * ★ 前提：PC7/PC8/PC9 只有 64 脚及以上封装才有。本板是 STM32F103**RCT6**（LQFP64），
 *   所以可用。若换回 48 脚的 C8T6（只有 PC13/14/15），这三路就都没了。
 *
 * ★ 注意：这三根脚对应 TIM3 的**完全重映射**（GPIO_FullRemap_TIM3），
 *   不是部分重映射。完全重映射：CH1=PC6 CH2=PC7 CH3=PC8 CH4=PC9。
 *
 * 历史：换 C8T6 期间曾在 PA0(TIM2_CH1)=夹爪 / PA1(TIM2_CH2)=旋转 上跑；
 *       换回 RCT6 后恢复 PC7/PC8/PC9 三路。
 *
 * 三路共用一个时基，三个 CCR 各自独立，改一路不影响另两路。
 * 脉宽映射：0° → 500 us，270° → 2500 us，中间线性；
 * 超出 0~270° 自动限幅，避免舵机撞机械限位。
 */

typedef enum {
    SERVO_ROT = 0,      /* PC9 / TIM3_CH4 旋转舵机 */
    SERVO_GRIP,         /* PC8 / TIM3_CH3 夹爪舵机 */
    SERVO_LIFT,         /* PC7 / TIM3_CH2 升降舵机 */
    SERVO_COUNT
} ServoId;

/* 上电安全角度（现场实测标定的位置）：
 *   旋转 30°  = 夹物料位置（初始就停在这里）
 *   夹爪 85°  = 松开物料
 *   升降 90°  = 中间位（不要一上电就顶到任一端）
 * Servo_Init() 一上电就直接输出这三个角度，不会先甩到 0°。 */
#define SERVO_ROT_INIT_DEG      30u
#define SERVO_GRIP_INIT_DEG     85u
#define SERVO_LIFT_INIT_DEG     90u

/* 现场角度微调（单位 度，可正可负）：某个舵机装偏了只改这里，
 * 流程里写的目标角度不用动。实际输出角度 = 目标角度 + 微调。 */
#define SERVO_TRIM_ROT      0
#define SERVO_TRIM_GRIP     0
#define SERVO_TRIM_LIFT     0

/* 方向取反（0 = 正常，1 = 反过来）：
 * 舵机齿轮箱/摇臂装反时，写 240° 会跑到另一头。这里取反后，
 * "逻辑角度"仍然是 240°（流程、OLED 显示都不变），只是实际脉宽按 270-角度 算。
 *
 * 现场实测：旋转舵机上电输出 240°（2278 us）却跑到"放物料"那一端，
 * 说明它的方向和约定相反，所以 ROT 取反。
 * 夹爪/升降如果也是反的，把对应宏改成 1。 */
#define SERVO_INVERT_ROT    1
#define SERVO_INVERT_GRIP   0
#define SERVO_INVERT_LIFT   0

#define SERVO_MIN_PULSE_US  500u    /* 0° 对应脉宽 */
#define SERVO_MAX_PULSE_US  2500u   /* 270° 对应脉宽 */
#define SERVO_ANGLE_MIN     0u
#define SERVO_ANGLE_MAX     270u

/* 初始化三路 PWM，并直接输出上面的安全角度 */
void Servo_Init(void);

/* 设置某一路角度（自动加微调、限幅到 0~270°） */
void Servo_SetAngle(ServoId id, uint16_t deg);

/* 当前实际角度（含微调）/ 当前实际脉宽（写进 CCR 的值，单位 us） */
uint16_t Servo_GetAngle(ServoId id);
uint16_t Servo_GetPulse(ServoId id);

#endif /* __SERVO_H */
