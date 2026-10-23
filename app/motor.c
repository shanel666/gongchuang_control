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
#define MOTOR_SPEED      70    /* 速度 0x07D0；上位机默认 200 已较快 */
#define MOTOR_ACCEL      200     /* 加速度 0x64 */
#define MOTOR_SPEED_H    ((uint8_t)(MOTOR_SPEED >> 8))   /* 速度高字节 */
#define MOTOR_SPEED_L    ((uint8_t)(MOTOR_SPEED & 0xFF)) /* 速度低字节 */

/* F6 速度模式参数：速度 0~3000 RPM，加速度 0~255 档。 */
#define SPEED_MODE_FUNCTION 0xF6u
#define TURN_SPEED_ACCEL    150u

/*
 * 移动命令（13 字节）：
 *   [地址][FD][方向][速度H][速度L][加速度][脉冲B3][脉冲B2][脉冲B1][脉冲B0][相对][同步][校验]
 *   方向 0x00=正转/CW，0x01=反转/CCW。
 *   FR(地址2)、RR(地址4) 镜像安装，前进时发反转。
 *   同步 0x01=等待同步信号（直行用，末尾触发帧统一起步），0x00=立即执行（旋转用）。
 *   校验固定 0x6B。
 */

/* 方向位按地址取反：换电机/拆装轮子后某只轮子转向相反时，靠 motor.h 里的
 * MOTOR_DIR_INV_x 翻回来，不用去改每个动作里那张方向表。 */
static uint8_t motor_dir_bit(uint8_t addr, uint8_t dir)
{
    uint8_t inv = 0u;

    if      (addr == 0x01u) inv = MOTOR_DIR_INV_1;
    else if (addr == 0x02u) inv = MOTOR_DIR_INV_2;
    else if (addr == 0x03u) inv = MOTOR_DIR_INV_3;
    else if (addr == 0x04u) inv = MOTOR_DIR_INV_4;

    return (uint8_t)(dir ^ (inv ? 0x01u : 0x00u));
}

/* 给一个地址发一帧：地址 + 逻辑方向 → 实际方向位 → 发送。
 * 把"地址"和"方向"写在一起，改 MOTOR_ADDR_* 时不会漏改另一处。 */
#define SEND_WHEEL(a, d)                                    \
    do {                                                    \
        cmd[0] = (uint8_t)(a);                              \
        cmd[2] = motor_dir_bit((uint8_t)(a), (uint8_t)(d)); \
        Serial_SendArray(cmd, 13);                          \
        Delay_ms(5);                                        \
    } while(0)

/* ============================================================================
 * 方向表（2026-10-23 换电机后实测标定）
 * ----------------------------------------------------------------------------
 * 这套轮子的滚子朝向是"同手"的（不是常规左右镜像），所以运动学映射和教科书
 * 不一样，实测结果是：
 *
 *   前进  FL 1  FR 0  RL 0  RR 1     ← 对角轮一组（FL/RR vs FR/RL）
 *   后退  全部取反
 *   右平移 FL 0  FR 0  RL 1  RR 1    ← 前后一组
 *   左平移 全部取反
 *   旋转  四个轮子同向（全 0 或全 1）
 *   注意：  FL 1 FR 0 RL 1 RR 0 这种"左右一组"会让四个轮子互相抵消，车不动。
 *
 * 对应关系是现场按 P1/P2/P3 逐项试出来的，别再照教科书改回去。
 * 2026-10-23 二次修正：P1 前进/后退实测相反，两张表已对调（即前进改成原来的后退表）。
 * 若以后换电机后某个轮子个别反了，用单电机点动对比，再把 motor.h 里的
 * MOTOR_DIR_INV_x 置 1，不用改下面这几张表。
 * ========================================================================== */

/* 前进：mm 毫米 */
void qianjin_(uint32_t mm)
{
    uint32_t p = mm * PULSE_PER_100CM / 1000u;   /* 脉冲数 */
    uint8_t  f = (uint8_t)(MOTOR_FLIP_FWD ? 0x01u : 0x00u);   /* 整轴取反开关 */
    uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

    cmd[6] = (uint8_t)(p >> 24);
    cmd[7] = (uint8_t)(p >> 16);
    cmd[8] = (uint8_t)(p >> 8);
    cmd[9] = (uint8_t)p;

    SEND_WHEEL(MOTOR_ADDR_FL, 0x01 ^ f);   /* 实测前进表：FL 1 FR 0 RL 0 RR 1 */
    SEND_WHEEL(MOTOR_ADDR_FR, 0x00 ^ f);
    SEND_WHEEL(MOTOR_ADDR_RL, 0x00 ^ f);
    SEND_WHEEL(MOTOR_ADDR_RR, 0x01 ^ f);

    /* 同步释放：直行 sync=0x01，此帧让 4 个电机同时起步 */
    Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
	Delay_ms(5);
}

/* 后退：mm 毫米 */
void houtui_(uint32_t mm)
{
    uint32_t p = mm * PULSE_PER_100CM / 1000u;
    uint8_t  f = (uint8_t)(MOTOR_FLIP_FWD ? 0x01u : 0x00u);   /* 与前进共用同一个整轴开关 */
    uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

    cmd[6] = (uint8_t)(p >> 24);
    cmd[7] = (uint8_t)(p >> 16);
    cmd[8] = (uint8_t)(p >> 8);
    cmd[9] = (uint8_t)p;

    SEND_WHEEL(MOTOR_ADDR_FL, 0x00 ^ f);   /* 后退 = 前进表取反 */
    SEND_WHEEL(MOTOR_ADDR_FR, 0x01 ^ f);
    SEND_WHEEL(MOTOR_ADDR_RL, 0x01 ^ f);
    SEND_WHEEL(MOTOR_ADDR_RR, 0x00 ^ f);

    Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
	Delay_ms(5);
}

/* 左平移：实测是"前后一组"（FL/FR 一组、RL/RR 另一组反着转）。
 * 注意旧的"四个轮子同向"其实是旋转，已按实测改掉。 */
void zuo_(uint32_t mm)
{
    uint32_t p = mm * PULSE_PER_100CM / 1000u;
    uint8_t  f = (uint8_t)(MOTOR_FLIP_LAT ? 0x01u : 0x00u);   /* 左右轴整轴取反开关 */
    uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

    cmd[6] = (uint8_t)(p >> 24);
    cmd[7] = (uint8_t)(p >> 16);
    cmd[8] = (uint8_t)(p >> 8);
    cmd[9] = (uint8_t)p;

    SEND_WHEEL(MOTOR_ADDR_FL, 0x01 ^ f);   /* 实测左平移表：FL 1 FR 1 RL 0 RR 0 */
    SEND_WHEEL(MOTOR_ADDR_FR, 0x01 ^ f);
    SEND_WHEEL(MOTOR_ADDR_RL, 0x00 ^ f);
    SEND_WHEEL(MOTOR_ADDR_RR, 0x00 ^ f);

    Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
	Delay_ms(5);
}

/* 右平移*/
void you_(uint32_t mm)
{
    uint32_t p = mm * PULSE_PER_100CM / 1000u;
    uint8_t  f = (uint8_t)(MOTOR_FLIP_LAT ? 0x01u : 0x00u);   /* 与左平移共用同一个整轴开关 */
    uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

    cmd[6] = (uint8_t)(p >> 24);
    cmd[7] = (uint8_t)(p >> 16);
    cmd[8] = (uint8_t)(p >> 8);
    cmd[9] = (uint8_t)p;

    SEND_WHEEL(MOTOR_ADDR_FL, 0x00 ^ f);   /* 实测右平移表：FL 0 FR 0 RL 1 RR 1 */
    SEND_WHEEL(MOTOR_ADDR_FR, 0x00 ^ f);
    SEND_WHEEL(MOTOR_ADDR_RL, 0x01 ^ f);
    SEND_WHEEL(MOTOR_ADDR_RR, 0x01 ^ f);

    Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
	Delay_ms(5);
}

/* 右旋转（同步触发）*/
void shun_yaw_(uint32_t deg)
{
    uint32_t p = deg * PULSE_PER_360DEG / 360u;
    uint8_t  d = (uint8_t)(0x00u ^ MOTOR_YAW_SWAP);   /* 四个轮子同向：0 与 1 是相反的两个转向 */
    uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

    cmd[6] = (uint8_t)(p >> 24);
    cmd[7] = (uint8_t)(p >> 16);
    cmd[8] = (uint8_t)(p >> 8);
    cmd[9] = (uint8_t)p;

    SEND_WHEEL(MOTOR_ADDR_FL, d);
    SEND_WHEEL(MOTOR_ADDR_FR, d);
    SEND_WHEEL(MOTOR_ADDR_RL, d);
    SEND_WHEEL(MOTOR_ADDR_RR, d);
	
    Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
	Delay_ms(5);
}

/* 左旋转（同步触发）*/
void ni_yaw_(uint32_t deg)
{
    uint32_t p = deg * PULSE_PER_360DEG / 360u;
    uint8_t  d = (uint8_t)(0x01u ^ MOTOR_YAW_SWAP);   /* 四个轮子同向：0 与 1 是相反的两个转向 */
    uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

    cmd[6] = (uint8_t)(p >> 24);
    cmd[7] = (uint8_t)(p >> 16);
    cmd[8] = (uint8_t)(p >> 8);
    cmd[9] = (uint8_t)p;

    SEND_WHEEL(MOTOR_ADDR_FL, d);
    SEND_WHEEL(MOTOR_ADDR_FR, d);
    SEND_WHEEL(MOTOR_ADDR_RL, d);
    SEND_WHEEL(MOTOR_ADDR_RR, d);
	
    Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
	Delay_ms(5);
	
}

/* 发送 F6 速度模式命令：[地址][F6][方向][速度H][速度L][加速度][同步][6B]。
 * 旋转仍使用 FL(1) 和 FR(2)，两个电机方向按现有旋转逻辑保持一致。
 * 速度模式是持续运行命令，rpm=0 用于停止。 */
static void motor_send_speed_command(uint8_t address, uint8_t direction, uint16_t rpm)
{
    uint8_t cmd[8];

    cmd[0] = address;
    cmd[1] = SPEED_MODE_FUNCTION;
    cmd[2] = direction;
    cmd[3] = (uint8_t)(rpm >> 8);
    cmd[4] = (uint8_t)(rpm & 0xFFu);
    cmd[5] = TURN_SPEED_ACCEL;
    cmd[6] = 0x00;                 /* 立即执行，不等待同步释放 */
    cmd[7] = 0x6B;

    Serial_SendArray(cmd, 8);
    Delay_ms(2);
}

void motor_rotate_speed(uint8_t direction, uint16_t rpm)
{
    motor_send_speed_command(MOTOR_ADDR_FL, direction, rpm);
    motor_send_speed_command(MOTOR_ADDR_FR, direction, rpm);
}

void motor_rotate_speed_stop(void)
{
    motor_rotate_speed(0x00, 0x0000);
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
/* 单电机点动：只驱动一个地址，立即执行（同步位 = 0，不需要同步释放帧）。
 * 转速和加速度取得比整车低，方便肉眼确认"这个地址是哪个轮子、往哪转"。
 * 换电机/改地址后先用它把 4 个轮子对上号，再改 motor.h 里的 MOTOR_ADDR_*。 */
#define JOG_SPEED   30u     /* 点动速度，比 MOTOR_SPEED 慢 */
#define JOG_ACCEL   100u    /* 点动加速度 */

void motor_jog_addr(uint8_t addr, uint8_t reverse, uint32_t pulses)
{
    uint8_t cmd[13] = {
        addr, 0xFD, 0x00,
        (uint8_t)(JOG_SPEED >> 8), (uint8_t)(JOG_SPEED & 0xFFu), (uint8_t)JOG_ACCEL,
        0x00, 0x00, 0x00, 0x00,
        0x00,   /* 相对位置模式 */
        0x00,   /* 立即执行，不等同步帧 */
        0x6B
    };

    /* 点动也走同一套极性修正；注意当前映射下"前进"是 FL0 FR1 RL1 RR0 的混合表，
     * 所以 JOG+ 只代表该地址的一个固定方向，用来核对"地址 ↔ 轮子"就够了。 */
    cmd[2] = motor_dir_bit(addr, reverse ? 0x01u : 0x00u);
    cmd[6] = (uint8_t)(pulses >> 24);
    cmd[7] = (uint8_t)(pulses >> 16);
    cmd[8] = (uint8_t)(pulses >> 8);
    cmd[9] = (uint8_t)pulses;

    Serial_SendArray(cmd, 13);
    Delay_ms(5);
}

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

    cmd[2] = motor_dir_bit(cmd[0], dir);   /* 同样套用单轮极性修正 */
    cmd[6] = (uint8_t)(absp >> 24);
    cmd[7] = (uint8_t)(absp >> 16);
    cmd[8] = (uint8_t)(absp >> 8);
    cmd[9] = (uint8_t)(absp);

    Serial_SendArray(cmd, 13);
}
