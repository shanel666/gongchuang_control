#include "stm32f10x.h"
#include "lift_motor.h"
#include "Serial.h"
#include "Delay.h"

/* ============================================================================
 * 升降步进电机（地址 6）—— 从 main.c 原样搬过来，逻辑未改
 * ========================================================================== */

/* 撤销栈 + 升降电机当前被命令到的绝对位置（只用于显示和"已在高度就跳过"） */
uint32_t g_lift_target = LIFT_MOTOR_HOME;

/* 升降步进电机（地址 6）：绝对位置模式、立即执行 */
void lift_motor_move(uint32_t abs_pulses, uint8_t dir)
{
    uint8_t cmd[13] = {
        LIFT_MOTOR_ADDR,
        0xFD,                                            /* 位置模式 */
        dir,                                             /* 0x01 = CCW 升 */
        (uint8_t)(LIFT_MOTOR_SPEED >> 8),
        (uint8_t)(LIFT_MOTOR_SPEED & 0xFFu),
        (uint8_t)LIFT_MOTOR_ACCEL,
        0x00, 0x00, 0x00, 0x00,                          /* 脉冲数，下面填 */
        0x01,                                            /* 绝对位置：目标 = 上电位置起的脉冲数 */
        0x00,                                            /* 立即执行 */
        0x6B
    };

    cmd[6] = (uint8_t)(abs_pulses >> 24);
    cmd[7] = (uint8_t)(abs_pulses >> 16);
    cmd[8] = (uint8_t)(abs_pulses >> 8);
    cmd[9] = (uint8_t)abs_pulses;

    Serial_SendArray(cmd, 13);
    Delay_ms(5);

    g_lift_target = abs_pulses;
}

/* 立即停止（0xFE 0x98）：中止时用，别让它继续往目标位置走 */
void lift_motor_stop(void)
{
    Serial_SendByte(LIFT_MOTOR_ADDR);
    Serial_SendByte(0xFE);
    Serial_SendByte(0x98);
    Serial_SendByte(0x00);
    Serial_SendByte(0x6B);
    Delay_ms(5);
}
