#include "motor.h"
#include "Serial.h"
#include "Delay.h"

/*
 * 依赖你工程里的串口/延时函数（请确保对应头文件已包含）：
 *   Serial_SendArray(buf, len)   Serial_SendByte(byte)   Delay_ms(ms)
 */

/* —— 标定 —— */
#define PULSE_PER_100CM  13008u   /* 平移标定：12658 脉冲 = 100 cm（已标定） */
#define PULSE_PER_360DEG    17208u     /* 旋转标定：每度脉冲数 —— TODO 待标定！当前为占位值 360度 15800*/

/* 速度 / 加速度（可调） */
#define MOTOR_SPEED      50    /* 速度 0x07D0；上位机默认 200 已较快 */
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

    cmd[0] = 0x01; cmd[2] = 0x01;   /* FL 正转 */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x02; cmd[2] = 0x00;   /* FR 反转（镜像） */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x03; cmd[2] = 0x01;   /* RL 正转 */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x04; cmd[2] = 0x00;   /* RR 反转（镜像） */
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

    cmd[0] = 0x01; cmd[2] = 0x00;   /* FL 反转 */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x02; cmd[2] = 0x01;   /* FR 正转（镜像） */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x03; cmd[2] = 0x00;   /* RL 反转 */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x04; cmd[2] = 0x01;   /* RR 正转（镜像） */
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

    cmd[0] = 0x01; cmd[2] = 0x00;   /* FL */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x02; cmd[2] = 0x00;   /* FR */
    Serial_SendArray(cmd, 13); Delay_ms(5);
//    cmd[0] = 0x03; cmd[2] = 0x00;   /* RL */
//    Serial_SendArray(cmd, 13); Delay_ms(5);
//    cmd[0] = 0x04; cmd[2] = 0x00;   /* RR */
//    Serial_SendArray(cmd, 13); Delay_ms(5);

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

    cmd[0] = 0x01; cmd[2] = 0x01;   /* FL */
    Serial_SendArray(cmd, 13); Delay_ms(5);
    cmd[0] = 0x02; cmd[2] = 0x01;   /* FR */
    Serial_SendArray(cmd, 13); Delay_ms(5);
//    cmd[0] = 0x03; cmd[2] = 0x01;   /* RL */
//    Serial_SendArray(cmd, 13); Delay_ms(5);
//    cmd[0] = 0x04; cmd[2] = 0x01;   /* RR */
//    Serial_SendArray(cmd, 13); Delay_ms(5);

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
volatile uint8_t uart_key_num = 0;
int32_t  pos_now  = 0;    // 电机当前位置（有符号）
int32_t  pos_memo = 0;    // 记忆位置（有符号）
uint8_t  pos_dir  = 0;    // 方向位暂存
void motor_read_pos(void)
{
    Serial_SendByte(0x01);
    Serial_SendByte(0x36);
    Serial_SendByte(0x6B);

    Serial_RxFlag = 0;
    uint32_t timeout = 200000;
    while (Serial_RxFlag == 0 && timeout > 0) timeout--;

    if (Serial_RxFlag &&
        Serial_RxPacket[0] == 0x01 &&
        Serial_RxPacket[1] == 0x36 &&
        Serial_RxPacket[7] == 0x6B)
    {
        pos_dir = Serial_RxPacket[2];

        uint32_t p = ((uint32_t)Serial_RxPacket[3] << 24) |
                     ((uint32_t)Serial_RxPacket[4] << 16) |
                     ((uint32_t)Serial_RxPacket[5] << 8)  |
                     ((uint32_t)Serial_RxPacket[6]);

        /* 方向位 0x01 代表负，就转成负数 */
        if (pos_dir == 0x01)
            pos_now = -(int32_t)p;
        else
            pos_now =  (int32_t)p;
    }
    Serial_RxFlag = 0;
}

uint8_t cmd[13] = {
    0x01,                       // [0] 地址（原来是 0x00，改成 0x01）
    0xFD,                       // [1] 命令
    0x00,                       // [2] 方向，运行时覆盖
    0x00, 0x64,                 // [3][4] 速度 = 0x1194 = 4500
    0xC8,                       // [5] 加速度 = 200
    0x00, 0x00, 0x00, 0x00,     // [6..9] 位置，运行时覆盖
    0x01,                       // [10] 绝对位置模式
    0x00,                       // [11] 保留
    0x6B                        // [12] 尾部
};
void motor_set_pos(int32_t pos)
{
    uint32_t absp;
    uint8_t  dir;

    if (pos < 0)
    {
        dir  = 0x01;              // 负数方向
        absp = (uint32_t)(-pos);  // 绝对值
    }
    else
    {
        dir  = 0x00;              // 正数方向
        absp = (uint32_t)pos;
    }

    cmd[2] = dir;
    cmd[6] = (uint8_t)(absp >> 24);
    cmd[7] = (uint8_t)(absp >> 16);
    cmd[8] = (uint8_t)(absp >> 8);
    cmd[9] = (uint8_t)(absp);

    Serial_SendArray(cmd, 13);
}
