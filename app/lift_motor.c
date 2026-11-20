#include "stm32f10x.h"
#include "lift_motor.h"
#include "Serial.h"
#include "Delay.h"

/* ============================================================================
 * 升降步进电机（地址 6）—— 从 main.c 原样搬过来，逻辑未改
 * 后来加了：软上限、读驱动器实际位置（0x36）、带原始帧的报告
 * ========================================================================== */

/* 升降电机"跟踪位置"（我们自己维护的，用来显示和"已经在位就跳过"）。
 * 上电 = 0 = 机械最低位。外部调用都按绝对位置写，帧里也直接填绝对位置。 */
uint32_t g_lift_target = LIFT_MOTOR_HOME;

/* ★ 运行期覆盖（排查下降堵转用；0 = 不覆盖，按编译期常量走） */
static uint8_t g_lift_speed_ovr = 0u;
static uint8_t g_lift_accel_now = (uint8_t)LIFT_MOTOR_ACCEL;

void lift_motor_set_speed_ovr(uint8_t rpm)  { g_lift_speed_ovr = rpm; }
uint8_t lift_motor_get_speed_ovr(void)      { return g_lift_speed_ovr; }
void lift_motor_set_accel(uint8_t accel)    { g_lift_accel_now = (accel == 0u) ? (uint8_t)LIFT_MOTOR_ACCEL : accel; }
uint8_t lift_motor_get_accel(void)          { return g_lift_accel_now; }

/* 按指定速度估算 from → to 要走多久（ms）。
 * 理论时间 = 脉冲数 ÷ (rpm × 3200 / 60)；这里再乘 1.25 留余量，加固定开销。 */
uint16_t lift_motor_ms_for(uint32_t from, uint32_t to, uint8_t rpm)
{
    uint32_t d   = (from > to) ? (from - to) : (to - from);
    uint32_t per;                 /* 每 1000 脉冲的耗时（ms，理论值） */
    uint32_t ms;

    if (rpm == 0u) rpm = (uint8_t)LIFT_MOTOR_SPEED;
    if (g_lift_speed_ovr != 0u) rpm = g_lift_speed_ovr;   /* 覆盖值同时影响时间估算 */

    /* 1000 脉冲 ÷ (rpm*3200/60) 秒 = 60000*1000/(rpm*3200) ms = 18750/rpm */
    per = 18750u / (uint32_t)rpm;
    if (per == 0u) per = 1u;

    ms = (d * per * 5u) / 4000u + LIFT_MOVE_EXTRA_MS;   /* ×1.25 余量 */

    if (ms > LIFT_MOVE_MS_CAP) ms = LIFT_MOVE_MS_CAP;
    return (uint16_t)ms;
}

/* 估算 from → to：按方向自动选上升/下降速度 */
uint16_t lift_motor_move_ms(uint32_t from, uint32_t to)
{
    uint8_t rpm = (to < from) ? (uint8_t)LIFT_MOTOR_SPEED_DOWN
                              : (uint8_t)LIFT_MOTOR_SPEED;
    return lift_motor_ms_for(from, to, rpm);
}

/* 柔和下降分段：返回 1 = 要两段 */
uint8_t lift_motor_soft_split(uint32_t from, uint32_t to, uint32_t *mid)
{
    if (mid == 0) return 0u;
    if (to >= from) return 0u;                       /* 不是下降 */
    if ((from - to) <= (uint32_t)LIFT_SOFT_ZONE) return 0u;   /* 行程太短，不用分 */

    *mid = to + (uint32_t)LIFT_SOFT_ZONE;
    return 1u;
}

/* ============================================================================
 * 升降步进电机（地址 6）—— ★ 绝对位置模式（2026-11 现场确认）★
 * ----------------------------------------------------------------------------
 * 【怎么确认的】
 *   现场现象：同一目标值连发多次，只有第一次会动，之后不动；把脉冲值改成
 *   另一个数才会再动。这正是"绝对定位"的表现（驱动器去那个坐标，已经在就停）。
 *   之前我一度按"相对"改了发差值，结果反而更糟 —— 已经改回绝对。
 *
 * 【所以帧里直接填目标绝对位置】cmd[6..9] = abs_pulses，byte[10] = 0x01。
 * 【方向位】仍然按"目标 vs 当前位置"算，绝对模式下多数固件会忽略它，
 *   但填对了不会有副作用，错填则可能被当反向。
 * 【速度】上升 40 RPM / 下降 20 RPM —— 下降受重力，慢一档才不容易冲过头。
 * ========================================================================== */
void lift_motor_move(uint32_t abs_pulses, uint8_t dir)
{
    uint8_t speed;

    (void)dir;

    /* 方向按目标与当前位置的关系定（绝对模式下一般被忽略，填对无害） */
    speed = (abs_pulses >= g_lift_target) ? (uint8_t)LIFT_MOTOR_SPEED
                                          : (uint8_t)LIFT_MOTOR_SPEED_DOWN;
    lift_motor_move_at(abs_pulses, speed);
}

/* 用指定速度走到绝对位置（绝对位置帧 + byte[10]=0x01） */
void lift_motor_move_at(uint32_t abs_pulses, uint8_t speed_rpm)
{
    uint8_t cmd[13] = {
        LIFT_MOTOR_ADDR,
        0xFD,                                            /* 位置模式 */
        0x00,                                            /* 方向，下面填 */
        0x00, 0x00,                                      /* 速度，下面填 */
        (uint8_t)LIFT_MOTOR_ACCEL,
        0x00, 0x00, 0x00, 0x00,                          /* 绝对位置，下面填 */
        0x01,                                            /* 绝对位置模式 */
        0x00,                                            /* 立即执行 */
        0x6B
    };

    /* 软限位 */
    if (abs_pulses > LIFT_MOTOR_MAX) abs_pulses = LIFT_MOTOR_MAX;
    if (speed_rpm == 0u) speed_rpm = (uint8_t)LIFT_MOTOR_SPEED;
    if (g_lift_speed_ovr != 0u) speed_rpm = g_lift_speed_ovr;   /* ★ 排查堵转用的运行期覆盖 */

    /* 方向位按目标与当前位置的关系填 */
    cmd[2] = (abs_pulses >= g_lift_target) ? 0x01u : 0x00u;

    cmd[5] = g_lift_accel_now;                                  /* ★ 加速度档也可运行期改 */

    cmd[3] = (uint8_t)(speed_rpm >> 8);
    cmd[4] = (uint8_t)(speed_rpm & 0xFFu);

    /* ★ 关键：填**绝对位置**，不是差值 */
    cmd[6] = (uint8_t)(abs_pulses >> 24);
    cmd[7] = (uint8_t)(abs_pulses >> 16);
    cmd[8] = (uint8_t)(abs_pulses >> 8);
    cmd[9] = (uint8_t)abs_pulses;

    Serial_SendArray(cmd, 13);
    Delay_ms(5);

    g_lift_target = abs_pulses;
}

/* 把当前位置设为驱动器零点：Emm42 指令 [addr][0x93][0x88][0x6B] */
void lift_motor_set_zero(void)
{
    Serial_SendByte(LIFT_MOTOR_ADDR);
    Serial_SendByte(0x93);
    Serial_SendByte(0x88);
    Serial_SendByte(0x6B);
    Delay_ms(30);

    g_lift_target = LIFT_MOTOR_HOME;
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

/* 相对位置帧（byte[10] = 0x00）。
 * 只用来做一次对比实验：如果相对帧的行程和绝对帧不一样，
 * 说明驱动器根本没在做绝对定位 —— 那"下降到 4000 却一路到底"就能解释了。
 * 注意：这里不更新 g_lift_target（相对移动之后我们不知道自己在哪了）。 */
void lift_motor_move_rel(uint32_t pulses, uint8_t dir)
{
    uint8_t cmd[13] = {
        LIFT_MOTOR_ADDR,
        0xFD,                                            /* 位置模式 */
        dir,
        (uint8_t)(LIFT_MOTOR_SPEED >> 8),
        (uint8_t)(LIFT_MOTOR_SPEED & 0xFFu),
        (uint8_t)LIFT_MOTOR_ACCEL,
        0x00, 0x00, 0x00, 0x00,
        0x00,                                            /* ★ 相对位置 */
        0x00,                                            /* 立即执行 */
        0x6B
    };

    if(pulses > LIFT_MOTOR_MAX) pulses = LIFT_MOTOR_MAX;

    cmd[6] = (uint8_t)(pulses >> 24);
    cmd[7] = (uint8_t)(pulses >> 16);
    cmd[8] = (uint8_t)(pulses >> 8);
    cmd[9] = (uint8_t)pulses;

    Serial_SendArray(cmd, 13);
    Delay_ms(5);
}

/* 用驱动器实际位置校准跟踪值。
 * 只在差得比较多（>500 脉冲）时才改，避免读数噪声来回抖。 */
void lift_motor_sync(void)
{
    int32_t act = 0;

    if(!lift_motor_read_pos(&act)) return;

    if(act < (int32_t)LIFT_MOTOR_MIN) act = (int32_t)LIFT_MOTOR_MIN;
    if(act > (int32_t)LIFT_MOTOR_MAX) act = (int32_t)LIFT_MOTOR_MAX;

    if((act > (int32_t)g_lift_target + 500) || (act < (int32_t)g_lift_target - 500))
    {
        USART3_Printf("[LIFT] 位置自校准：跟踪 %u -> 驱动器 %d\r\n",
                      (unsigned)g_lift_target, (int)act);
        g_lift_target = (uint32_t)act;
    }
}

/* 读驱动器实际位置（0x36 指令）
 * ----------------------------------------------------------------------------
 * 发送：[6][0x36][0x6B]                    3 字节
 * 回包：[6][0x36][符号][位置 B3..B0][0x6B] 8 字节
 *   符号 0x00 = 正，0x01 = 负（说明驱动器认为自己在零点下面 —— 正是要抓的现象）
 * ========================================================================== */
uint8_t lift_motor_read_pos(int32_t *out)
{
    uint32_t timeout;
    uint32_t p;
    uint8_t  sign;

    Serial_SendByte(LIFT_MOTOR_ADDR);
    Serial_SendByte(0x36);
    Serial_SendByte(0x6B);

    Serial_RxFlag = 0;
    timeout = 200000u;
    while ((Serial_RxFlag == 0) && (timeout > 0u)) timeout--;

    if (Serial_RxFlag == 0) return 0u;
    if (Serial_RxPacket[0] != LIFT_MOTOR_ADDR) return 0u;
    if (Serial_RxPacket[1] != 0x36)            return 0u;
    if (Serial_RxPacket[7] != 0x6B)            return 0u;

    sign = Serial_RxPacket[2];
    p = ((uint32_t)Serial_RxPacket[3] << 24) |
        ((uint32_t)Serial_RxPacket[4] << 16) |
        ((uint32_t)Serial_RxPacket[5] << 8)  |
        ((uint32_t)Serial_RxPacket[6]);

    if (out != 0) *out = (sign == 0x01u) ? -(int32_t)p : (int32_t)p;

    Serial_RxFlag = 0;
    return 1u;
}

/* 十六进制打印一个字节（USART3_Printf 不带 %X，自己拼） */
static void lift_put_hex8(uint8_t v)
{
    static const char hx[] = "0123456789ABCDEF";

    USART3_SendByte((uint8_t)hx[(v >> 4) & 0x0Fu]);
    USART3_SendByte((uint8_t)hx[v & 0x0Fu]);
}

/* 读一次实际位置 + 打一行串口（带原始 8 字节）。
 * 调试升降高度时用它，能一眼看出：
 *   - 驱动器有没有应答
 *   - 回包格式对不对（8 字节、头 06 36、尾 6B）
 *   - 实际位置和 g_lift_target 差多少 */
uint8_t lift_motor_report(void)
{
    int32_t  act = 0;
    int32_t  d;
    uint8_t  ok;

    ok = lift_motor_read_pos(&act);

    USART3_SendString("[LIFT] RAW ");
    lift_put_hex8(Serial_RxPacket[0]);
    USART3_SendByte(' ');
    lift_put_hex8(Serial_RxPacket[1]);
    USART3_SendByte(' ');
    lift_put_hex8(Serial_RxPacket[2]);
    USART3_SendByte(' ');
    lift_put_hex8(Serial_RxPacket[3]);
    USART3_SendByte(' ');
    lift_put_hex8(Serial_RxPacket[4]);
    USART3_SendByte(' ');
    lift_put_hex8(Serial_RxPacket[5]);
    USART3_SendByte(' ');
    lift_put_hex8(Serial_RxPacket[6]);
    USART3_SendByte(' ');
    lift_put_hex8(Serial_RxPacket[7]);

    if (ok)
    {
        d = act - (int32_t)g_lift_target;
        USART3_Printf("  -> OK act=%d cmd=%u d=%d\r\n",
                      (int)act, (unsigned)g_lift_target, (int)d);
        return 1u;
    }

    USART3_SendString("  -> NO ANSWER (驱动器没回 0x36)\r\n");
    return 0u;
}
