#include "stm32f10x.h"
#include "material_motor.h"
#include "Serial.h"
#include "Delay.h"
#include "OLED.h"
#include "motor.h"      /* judge_wait */
#include "Key.h"

/* ============================================================================
 * 物料电机（地址 5）—— 从 main.c 整块搬出来，逻辑未改
 * ========================================================================== */

/* 上电以来累计发出的脉冲数（原来是 main.c 的文件级 static，跟着搬过来） */
static uint32_t material_total_pulses = 0;

/* ===== moved from main.c (logic unchanged): material_deg_to_pulses ===== */
/* 角度 → 脉冲：3200 脉冲 = 360°，四舍五入（60° → 533，360° → 3200）。 */
uint32_t material_deg_to_pulses(uint32_t deg)
{
    return (MATERIAL_PULSES_PER_REV * deg + MATERIAL_DEG_FULL / 2u) / MATERIAL_DEG_FULL;
}

/* ===== moved from main.c (logic unchanged): material_motor_move ===== */
/* 给物料电机发一帧位置命令：相对位置、立即执行（单电机不需要同步释放帧）。 */
void material_motor_move(uint32_t pulses, uint8_t dir)
{
    uint8_t cmd[13] = {
        MATERIAL_MOTOR_ADDR,
        0xFD,                                            /* 位置模式 */
        dir,
        (uint8_t)(MATERIAL_MOTOR_SPEED >> 8),
        (uint8_t)(MATERIAL_MOTOR_SPEED & 0xFFu),
        (uint8_t)MATERIAL_MOTOR_ACCEL,
        0x00, 0x00, 0x00, 0x00,                        /* 脉冲数，下面填 */
        0x00,                                            /* 相对位置：从当前位置再走 pulses */
        0x00,                                            /* 立即执行 */
        0x6B
    };

    cmd[6] = (uint8_t)(pulses >> 24);
    cmd[7] = (uint8_t)(pulses >> 16);
    cmd[8] = (uint8_t)(pulses >> 8);
    cmd[9] = (uint8_t)pulses;

    Serial_SendArray(cmd, 13);
    Delay_ms(5);

    material_total_pulses += pulses;
}

/* ===== moved from main.c (logic unchanged): material_motor_stop ===== */
/* 立即停止：[地址][0xFE][0x98][同步][0x6B]（Emm42 的"立即停止"命令）。 */
void material_motor_stop(void)
{
    Serial_SendByte(MATERIAL_MOTOR_ADDR);
    Serial_SendByte(0xFE);
    Serial_SendByte(0x98);
    Serial_SendByte(0x00);
    Serial_SendByte(0x6B);
    Delay_ms(5);
}

/* ===== moved from main.c (logic unchanged): material_calib_rounds ===== */
/* 这一下要走多久 → 超时轮数：脉冲 ÷ (3200 脉冲/圈 × 速度RPM) 换算成毫秒，
 * 再加 2 s 余量。3200 是驱动器细分决定的"电机一圈"，和上面按角度换算的脉冲无关，
 * 所以角度/脉冲填大了，这里的超时也会跟着变长。 */
uint16_t material_calib_rounds(uint32_t pulses)
{
    uint32_t ms, rounds;

    ms = (pulses / 4u) * 75u / MATERIAL_MOTOR_SPEED + 2000u;   /* 60000/3200 = 18.75 = 75/4 */
    rounds = ms / MAT_CAL_ROUND_MS;

    if(rounds < 100u)  rounds = 100u;    /* 至少 2 s，别刚发完就判超时 */
    if(rounds > 3000u) rounds = 3000u;   /* 最多 60 s */

    return (uint16_t)rounds;
}

/* ===== moved from main.c (logic unchanged): material_motor_wait_arrive ===== */
/* 轮询 0x3A 到位状态：回包 [地址][0x3A][状态][0x6B]，状态 bit1 = 已到位。
 * 返回 MAT_CAL_DONE / MAT_CAL_NO_ANSWER / MAT_CAL_ABORT。 */
uint8_t material_motor_wait_arrive(uint16_t max_rounds)
{
    uint16_t round;
    uint8_t  i;

    /* 刚发完命令时驱动器可能还留着上一轮的"到位"标志，先等它真正起步 */
    Delay_ms(50);

    for(round = 0; round < max_rounds; round++){
        /* PA11 随时中止，避免参数填太大时一直转 */
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == Bit_RESET){
            material_motor_stop();
            while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == Bit_RESET) Delay_ms(10);  /* 等松手 */
            return MAT_CAL_ABORT;
        }

        Serial_RxFlag = 0;
        Serial_RxPacket[0] = Serial_RxPacket[1] = Serial_RxPacket[2] = Serial_RxPacket[3] = 0;
        Serial_SendByte(MATERIAL_MOTOR_ADDR);
        Serial_SendByte(0x3A);
        Serial_SendByte(0x6B);
        for(i = 0; i < 10u; i++){
            if(Serial_GetRxFlag()) break;
            Delay_ms(1);
        }

        if(Serial_RxPacket[0] == MATERIAL_MOTOR_ADDR &&
           Serial_RxPacket[1] == 0x3A &&
           (Serial_RxPacket[2] & 0x02u) != 0u){
            return MAT_CAL_DONE;
        }

        Delay_ms(10);
    }

    return MAT_CAL_NO_ANSWER;
}

/* ===== moved from main.c (logic unchanged): material_calib_draw ===== */
/* 画标定界面：第 1 行角度，第 2 行本次脉冲，第 3 行累计脉冲，第 4 行状态。
 * 角度最多 3 位（≤999°）才能和 "DEG ADDR5" 拼在 16 列内。 */
void material_calib_draw(uint8_t state)
{
    uint8_t  n;
    uint32_t step = material_deg_to_pulses(MATERIAL_DEG_PER_STEP);

    n = OLED_Digits(MATERIAL_DEG_PER_STEP);
    OLED_ShowString(1, 1, "MAT ");
    OLED_ShowNum(1, 5, MATERIAL_DEG_PER_STEP, n);
    OLED_ShowString(1, (uint8_t)(5u + n), "DEG ADDR5");

    /* 标签 5 列 + 数字最多 10 列 + "P" = 16 列，正好不出屏 */
    OLED_ClearLine(2);
    OLED_ShowString(2, 1, "STEP:");
    OLED_ShowNum(2, 6, step, OLED_Digits(step));
    OLED_ShowString(2, (uint8_t)(6u + OLED_Digits(step)), "P");

    OLED_ClearLine(3);
    OLED_ShowString(3, 1, "TOTAL:");
    OLED_ShowNum(3, 7, material_total_pulses, OLED_Digits(material_total_pulses));

    switch(state){
        case MAT_CAL_MOVING:    OLED_ShowString(4, 1, "MOVING...       "); break;
        case MAT_CAL_DONE:      OLED_ShowString(4, 1, "OK STOPPED      "); break;
        case MAT_CAL_NO_ANSWER: OLED_ShowString(4, 1, "NO 3A ANSWER    "); break;
        case MAT_CAL_ABORT:     OLED_ShowString(4, 1, "STOP BY PA11    "); break;
        default:                OLED_ShowString(4, 1, "PA12: TURN STEP "); break;
    }
}

/* ===== moved from main.c (logic unchanged): material_calib_test ===== */
/* 分度转动主循环：不返回。 */
void material_calib_test(void)
{
    uint8_t  key, state;
    uint32_t step;

    /* 地址 5 不在 motor_enable_all() 的 1~4 里，失能状态下运动命令会被忽略 */
    motor_enable_addr(MATERIAL_MOTOR_ADDR);

    step = material_deg_to_pulses(MATERIAL_DEG_PER_STEP);

    OLED_Clear();
    material_calib_draw(MAT_CAL_IDLE);

    while(1){
        key = Key_GetNum();

        /* 只认 PA12（键码 4）；PA11 是运动中的停止键，空闲时按下忽略 */
        if(key != 4u || step == 0u){
            Delay_ms(10);
            continue;
        }

        material_motor_move(step, MATERIAL_MOTOR_DIR);
        material_calib_draw(MAT_CAL_MOVING);

        state = material_motor_wait_arrive(material_calib_rounds(step));
        if(state == MAT_CAL_DONE) material_motor_stop();   /* 到位后再补一发立即停止 */
        material_calib_draw(state);
    }
}
