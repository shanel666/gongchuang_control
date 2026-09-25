#include "motor.h"
#include "Serial.h"
#include "Delay.h"

/*
 * 依赖你工程里的串口/延时函数（请确保对应头文件已包含）：
 *   Serial_SendArray(buf, len)   Serial_SendByte(byte)   Delay_ms(ms)
 */

/* —— 标定 —— */
#define PULSE_PER_100CM  12658u   /* 平移标定：12658 脉冲 = 100 cm（已标定） */
#define PULSE_PER_360DEG    16028u     /* 旋转标定：每度脉冲数 —— TODO 待标定！当前为占位值 360度 15800*/

/* 速度 / 加速度（可调） */
#define MOTOR_SPEED      150    /* 速度 0x07D0；上位机默认 200 已较快 */
#define MOTOR_ACCEL      200     /* 加速度 0x64 */
#define MOTOR_SPEED_H    ((uint8_t)(MOTOR_SPEED >> 8))   /* 速度高字节 */
#define MOTOR_SPEED_L    ((uint8_t)(MOTOR_SPEED & 0xFF)) /* 速度低字节 */

/*
 * 移动命令（13 字节）：
 *   [地址][FD][方向][速度H][速度L][加速度][脉冲B3][脉冲B2][脉冲B1][脉冲B0][相对][同步][校验]
 *   方向 0x00=正转/CW，0x01=反转/CCW。
 *   FR(地址2)、RR(地址4) 镜像安装，前进时发反转。
 *   同步 0x01=等待同步信号（直行用，末尾触发帧统一起步），0x00=立即执行（旋转用）。
 *   校验固定 0x6B。
 */

/* 前进：mm 毫米 */
void qianjin_(uint32_t mm)
{
    uint32_t p = mm * PULSE_PER_100CM / 1000u;   /* 脉冲数 */
    uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

    cmd[6] = (uint8_t)(p >> 24);
    cmd[7] = (uint8_t)(p >> 16);
    cmd[8] = (uint8_t)(p >> 8);
    cmd[9] = (uint8_t)p;

    cmd[0] = 0x01; cmd[2] = 0x00;   /* FL 正转 */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x02; cmd[2] = 0x01;   /* FR 反转（镜像） */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x03; cmd[2] = 0x00;   /* RL 正转 */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x04; cmd[2] = 0x01;   /* RR 反转（镜像） */
    Serial_SendArray(cmd, 13); Delay_ms(5);

    /* 同步释放：直行 sync=0x01，此帧让 4 个电机同时起步 */
    Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
	Delay_ms(5);
}

/* 后退：mm 毫米 */
void houtui_(uint32_t mm)
{
    uint32_t p = mm * PULSE_PER_100CM / 1000u;
    uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

    cmd[6] = (uint8_t)(p >> 24);
    cmd[7] = (uint8_t)(p >> 16);
    cmd[8] = (uint8_t)(p >> 8);
    cmd[9] = (uint8_t)p;

    cmd[0] = 0x01; cmd[2] = 0x01;   /* FL 反转 */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x02; cmd[2] = 0x00;   /* FR 正转（镜像） */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x03; cmd[2] = 0x01;   /* RL 反转 */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x04; cmd[2] = 0x00;   /* RR 正转（镜像） */
    Serial_SendArray(cmd, 13); Delay_ms(5);

    Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
	Delay_ms(5);
}

/* 左平移 */
void zuo_(uint32_t mm)
{
    uint32_t p = mm * PULSE_PER_100CM / 1000u;
    uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

    cmd[6] = (uint8_t)(p >> 24);
    cmd[7] = (uint8_t)(p >> 16);
    cmd[8] = (uint8_t)(p >> 8);
    cmd[9] = (uint8_t)p;

    cmd[0] = 0x01; cmd[2] = 0x01;   /* FL */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x02; cmd[2] = 0x01;   /* FR */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x03; cmd[2] = 0x01;   /* RL */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x04; cmd[2] = 0x01;   /* RR */
    Serial_SendArray(cmd, 13); Delay_ms(5);

    Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
	Delay_ms(5);
}

/* 右平移*/
void you_(uint32_t mm)
{
    uint32_t p = mm * PULSE_PER_100CM / 1000u;
    uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

    cmd[6] = (uint8_t)(p >> 24);
    cmd[7] = (uint8_t)(p >> 16);
    cmd[8] = (uint8_t)(p >> 8);
    cmd[9] = (uint8_t)p;

    cmd[0] = 0x01; cmd[2] = 0x00;   /* FL */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x02; cmd[2] = 0x00;   /* FR */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x03; cmd[2] = 0x00;   /* RL */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x04; cmd[2] = 0x00;   /* RR */
    Serial_SendArray(cmd, 13); Delay_ms(5);

    Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
	Delay_ms(5);
}

/* 右旋转*/
void shun_yaw_(uint32_t deg)
{
    uint32_t p = deg *PULSE_PER_360DEG/360u;
    uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

    cmd[6] = (uint8_t)(p >> 24);
    cmd[7] = (uint8_t)(p >> 16);
    cmd[8] = (uint8_t)(p >> 8);
    cmd[9] = (uint8_t)p;

//    cmd[0] = 0x01; cmd[2] = 0x00;   /* FL */
//    Serial_SendArray(cmd, 13); Delay_ms(50);
//    cmd[0] = 0x02; cmd[2] = 0x00;   /* FR */
//    Serial_SendArray(cmd, 13); Delay_ms(50);
    cmd[0] = 0x03; cmd[2] = 0x00;   /* RL */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x04; cmd[2] = 0x00;   /* RR */
    Serial_SendArray(cmd, 13); Delay_ms(5);

    Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
	Delay_ms(5);
}

/* 左旋转*/
void ni_yaw_(uint32_t deg)
{
    uint32_t p = deg*PULSE_PER_360DEG/360u ;
    uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

    cmd[6] = (uint8_t)(p >> 24);
    cmd[7] = (uint8_t)(p >> 16);
    cmd[8] = (uint8_t)(p >> 8);
    cmd[9] = (uint8_t)p;

//    cmd[0] = 0x01; cmd[2] = 0x01;   /* FL */
//    Serial_SendArray(cmd, 13); Delay_ms(50);
//    cmd[0] = 0x02; cmd[2] = 0x01;   /* FR */
//    Serial_SendArray(cmd, 13); Delay_ms(50);
    cmd[0] = 0x03; cmd[2] = 0x01;   /* RL */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x04; cmd[2] = 0x01;   /* RR */
    Serial_SendArray(cmd, 13); Delay_ms(5);

    Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
	Delay_ms(5);
}

/* 使能 4 个电机：[地址][F3][AB][01][同步][校验]，上电默认失能，移动前先调一次 */
void motor_enable_all(void)
{
    uint8_t i;
    for (i = 1; i <= 4; i++) {
        Serial_SendByte(i);
        Serial_SendByte(0xF3);
        Serial_SendByte(0xAB);
        Serial_SendByte(0x01);
        Serial_SendByte(0x00);
        Serial_SendByte(0x6B);
        Delay_ms(5);
    }
}
