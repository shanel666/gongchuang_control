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
 * 三路共用一个时基，三个 CCR 各自独立，改一路不影响另外两路。
 * 脉宽映射：0° → 500 us，270° → 2500 us，中间线性；
 * 超出 0~270° 自动限幅，避免舵机撞机械限位。
 */

typedef enum {
    SERVO_ROT = 0,      /* PC9 / CH4 旋转舵机 */
    SERVO_LIFT,         /* PC7 / CH2 升降舵机 */
    SERVO_GRIP,         /* PC8 / CH3 夹爪舵机 */
    SERVO_COUNT
} ServoId;

/* 上电安全角度（机构定义的安全姿态）：旋转 240°、升降 90°、夹爪 65°。
 * Servo_Init() 一上电就直接输出这三个角度，不会先甩到 0°。 */
#define SERVO_ROT_INIT_DEG     240u
#define SERVO_LIFT_INIT_DEG     90u
#define SERVO_GRIP_INIT_DEG     65u

/* 现场角度微调（单位 度，可正可负）：某个舵机装偏了只改这里，
 * 流程里写的目标角度不用动。实际输出角度 = 目标角度 + 微调。 */
#define SERVO_TRIM_ROT      0
#define SERVO_TRIM_LIFT     0
#define SERVO_TRIM_GRIP     0

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
