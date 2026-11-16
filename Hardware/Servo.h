#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

/*
 * 两路 270° 舵机 PWM（TIM2 默认通道，20 ms 周期 / 50 Hz，500~2500 us = 0~270°）
 *
 *   PA1 = TIM2_CH2 = 旋转舵机   SERVO_ROT
 *   PA0 = TIM2_CH1 = 夹爪舵机   SERVO_GRIP
 *
 * ★ PA2 已弃用：实测 PA2 输出异常（舵机信号线挪到 PA1 才动），疑似该脚损坏，
 *   所以旋转舵机从 PA2 改到 PA1，夹爪从 PA1 改到 PA0。升降舵机不使用。
 *
 * 注意 PA0/PA1 原本是 LED/步进使能相关脚，已全部让给舵机：
 *   LED1/LED2 函数已删除；USART2 改成只收（RX 仍在 PA3）。
 *
 * 两路共用一个时基，两个 CCR 各自独立，改一路不影响另一路。
 * 脉宽映射：0° → 500 us，270° → 2500 us，中间线性；
 * 超出 0~270° 自动限幅，避免舵机撞机械限位。
 */

typedef enum {
    SERVO_ROT = 0,      /* PA1 / TIM2_CH2 旋转舵机 */
    SERVO_GRIP,         /* PA0 / TIM2_CH1 夹爪舵机 */
    SERVO_COUNT
} ServoId;

/* 上电安全角度（现场实测标定的位置）：
 *   旋转 30°  = 夹物料位置（初始就停在这里）
 *   夹爪 85°  = 松开物料
 * Servo_Init() 一上电就直接输出这两个角度，不会先甩到 0°。
 *
 * 另外两个标定位置（写在动作表里）：
 *   旋转 200° = 放物料位置（满行程；实际只用一半 → 115°，见 main.c 的 SERVO_ROT_PLACE_DEG）
 *   夹爪 115° = 夹紧物料 */
#define SERVO_ROT_INIT_DEG      30u
#define SERVO_GRIP_INIT_DEG     85u

/* 现场角度微调（单位 度，可正可负）：某个舵机装偏了只改这里，
 * 流程里写的目标角度不用动。实际输出角度 = 目标角度 + 微调。 */
#define SERVO_TRIM_ROT      0
#define SERVO_TRIM_GRIP     0

/* 方向取反（0 = 正常，1 = 反过来）：
 * 舵机齿轮箱/摇臂装反时，写 240° 会跑到另一头。这里取反后，
 * "逻辑角度"仍然是 240°（流程、OLED 显示都不变），只是实际脉宽按 270-角度 算。
 *
 * 现场实测：旋转舵机上电输出 240°（2278 us）却跑到"放物料"那一端，
 * 说明它的方向和约定相反，所以 ROT 取反。
 * 如果夹爪也是反的（该夹紧时反而张开），把 SERVO_INVERT_GRIP 改成 1。 */
#define SERVO_INVERT_ROT    1
#define SERVO_INVERT_GRIP   0

#define SERVO_MIN_PULSE_US  500u    /* 0° 对应脉宽 */
#define SERVO_MAX_PULSE_US  2500u   /* 270° 对应脉宽 */
#define SERVO_ANGLE_MIN     0u
#define SERVO_ANGLE_MAX     270u

/* 初始化两路 PWM，并直接输出上面的安全角度 */
void Servo_Init(void);

/* 设置某一路角度（自动加微调、限幅到 0~270°） */
void Servo_SetAngle(ServoId id, uint16_t deg);

/* 当前实际角度（含微调）/ 当前实际脉宽（写进 CCR 的值，单位 us） */
uint16_t Servo_GetAngle(ServoId id);
uint16_t Servo_GetPulse(ServoId id);

#endif /* __SERVO_H */
