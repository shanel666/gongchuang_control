#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

/*
 * 张大头闭环步进电机 · 四麦轮小车 运动控制
 *
 * 硬件地址：FL=1  FR=2  RL=3  RR=4，其中 FR、RR 镜像安装（方向已在本模块处理）。
 * 移动命令（13 字节）：[地址][FD][方向][速度2B][加速度][脉冲4B][相对][同步][校验]
 * 平移标定：12658 脉冲 = 100 cm
 */

/* 前进 / 后退：参数 = 距离 mm */
void qianjin_(uint32_t mm);
void houtui_(uint32_t mm);

/* 左右平移：参数 = 距离 mm */
void zuo_(uint32_t mm);
void you_(uint32_t mm);

/* 左转（逆时针）/ 右转（顺时针）：参数 = 角度 度；每度脉冲见 motor.c 的 PULSE_PER_DEG */
void shun_yaw_(uint32_t deg);
void ni_yaw_(uint32_t deg);

/* F6 速度模式：方向 0=CW，1=CCW；速度单位 RPM；当前用于陀螺仪转向测试。 */
void motor_rotate_speed(uint8_t direction, uint16_t rpm);
void motor_rotate_speed_stop(void);

/* 使能 4 个电机：上电默认失能，移动前先调一次 */
void motor_enable_all(void);
extern volatile uint8_t uart_key_num;
extern int32_t  pos_now;    // 电机当前位置（有符号）
extern int32_t  pos_memo;    // 记忆位置（有符号）
extern uint8_t  pos_dir;    // 方向位暂存
void motor_read_pos(void);

extern uint8_t cmd[13];
void motor_set_pos(int32_t pos);
#endif /* __MOTOR_H */
