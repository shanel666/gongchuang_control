#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

/*
 * 张大头闭环步进电机 · 四麦轮小车 运动控制
 *
 * 移动命令（13 字节）：[地址][FD][方向][速度2B][加速度][脉冲4B][相对][同步][校验]
 * 平移标定：13008 脉冲 = 100 cm
 *
 * ★ 换过电机之后，驱动器地址和轮子的对应关系可能和下面这份默认表不一致。
 *   先用手动模式里的「单电机点动」确认"哪个地址是哪个轮子"，再改这 4 个宏：
 *     MOTOR_ADDR_FL  左前轮对应的地址
 *     MOTOR_ADDR_FR  右前轮
 *     MOTOR_ADDR_RL  左后轮
 *     MOTOR_ADDR_RR  右后轮
 *   运动函数体里不再写死 1/2/3/4，改这一处整车方向就都跟着变。
 */

#define MOTOR_ADDR_FL   0x01u   /* 左前 FL */
#define MOTOR_ADDR_FR   0x02u   /* 右前 FR（代码里已按镜像反装处理方向） */
#define MOTOR_ADDR_RL   0x03u   /* 左后 RL */
#define MOTOR_ADDR_RR   0x04u   /* 右后 RR（镜像反装） */

/* ★ 单轮方向极性：某个地址的轮子转向和整车逻辑相反时，把对应宏改成 1u。
 *   按“地址号”生效，和这个地址对应哪只轮子无关。
 *   2026-10-23 换电机后已经把实测结果直接写进 motor.c 的方向表，
 *   所以这里默认全 0；以后若单独换某只电机，用单电机点动对比后再把对应位置 1。 */
#define MOTOR_DIR_INV_1   0u
#define MOTOR_DIR_INV_2   0u
#define MOTOR_DIR_INV_3   0u
#define MOTOR_DIR_INV_4   0u

/* ★ 旋转方向：当前映射下"四个轮子同向"才是旋转。
 *   shun_yaw_ 先发全 0x01、ni_yaw_ 先发全 0x00；若 P3/P4 的顺时针/逆时针仍然相反，
 *   把这个改成 1u 即可整体对调。 */
#define MOTOR_YAW_SWAP    0u

/* ★ 整轴极性（现场快速修，不用改 motor.c 的方向表）：
 *   某条轴整个反了，就把对应宏置 1u —— 该轴四个轮子的方向位会全部取反。
 *     前后轴：P1 按「前进」车却往后退        → MOTOR_FLIP_FWD = 1u
 *     左右轴：P2 按「左平移」车却往右走      → MOTOR_FLIP_LAT = 1u
 *   旋转轴用上面的 MOTOR_YAW_SWAP。 */
#define MOTOR_FLIP_FWD    0u
#define MOTOR_FLIP_LAT    0u

/* 前进 / 后退：参数 = 距离 mm（头文件旧注释写 cm，以 motor.c 实现为准） */
void qianjin_(uint32_t mm);
void houtui_(uint32_t mm);

/* 左右平移：参数 = 距离 mm */
void zuo_(uint32_t mm);
void you_(uint32_t mm);

/* 左转（逆时针）/ 右转（顺时针）：参数 = 角度 度；每度脉冲见 motor.c 的 PULSE_PER_360DEG */
void shun_yaw_(uint32_t deg);
void ni_yaw_(uint32_t deg);

/* 单电机点动：只给 addr 这一个地址发位置命令（立即执行，不等同步）。
 * reverse=1 反转。用于换电机后核对"地址 ↔ 轮子"和转向极性。 */
void motor_jog_addr(uint8_t addr, uint8_t reverse, uint32_t pulses);

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
