#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

/*
 * 舵机 PWM 驱动
 *
 * 引脚：PC8 = TIM3_CH3（TIM3 完全重映射）
 * 信号：20 ms 周期，高电平 500~2500 us
 *
 * 当前舵机实际可行角度为 0~100°。
 * 夹爪状态：38° = 张开，44° = 加紧；上电默认张开。
 */

#define SERVO_MIN_PULSE_US  500u    /* 最小脉宽 */
#define SERVO_MAX_PULSE_US  2500u   /* 最大脉宽 */
#define SERVO_ANGLE_MAX     100u    /* 舵机可行角度上限，脉宽 500~2500 us 对应 0~100° */

#define SERVO_ANGLE_OPEN    38u     /* 夹爪张开角度 */
#define SERVO_ANGLE_CLAMP   44u     /* 夹爪加紧角度 */

/* 角度 -> 脉宽（编译期常量，四舍五入） */
#define SERVO_ANGLE_TO_PULSE(angle) \
	(SERVO_MIN_PULSE_US + ((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * (angle) \
	 + SERVO_ANGLE_MAX / 2u) / SERVO_ANGLE_MAX)

/* 初始化 PC8 的 PWM，上电默认输出张开角度（38°，1260us） */
void Servo_Init(void);

/* 夹爪张开（38°）/ 加紧（44°） */
void Servo_Open(void);
void Servo_Clamp(void);

/* 直接设置脉宽，单位 us；超出 500~2500 会自动限幅 */
void Servo_SetPulse(uint16_t pulse_us);

/* 设置角度，范围 0~SERVO_ANGLE_MAX；超出会自动限幅 */
void Servo_SetAngle(uint16_t angle_deg);

/* 读取当前脉宽，单位 us */
uint16_t Servo_GetPulse(void);

#endif /* __SERVO_H */
