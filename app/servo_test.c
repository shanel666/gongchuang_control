#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "servo_test.h"
#include "Servo.h"
#include "OLED.h"
#include "Delay.h"
#include "Serial.h"
#include "Key.h"
#include "LED.h"
#include "motion.h"

/* ============================================================================
 * 舵机测试 —— 从 main.c 整块搬过来，逻辑未改
 * ========================================================================== */

/* ===== moved from main.c (logic unchanged): servo270_draw ===== */
/* 画界面：angle 是当前目标角度，脉宽直接取 Servo_GetPulse(SERVO_ROT)，避免两处各算一遍。 */
void servo270_draw(uint16_t angle)
{
    uint16_t pulse = Servo_GetPulse(SERVO_ROT);
    uint8_t  n;

    OLED_ShowString(1, 1, "SERVO PC9  270D");

    /* 第 2 行角度：先擦整行再画，数字位数变化不会留尾巴 */
    OLED_ShowString(2, 1, "                ");
    OLED_ShowString(2, 1, "ANG:");
    n = OLED_Digits(angle);
    OLED_ShowNum(2, 5, angle, n);
    OLED_ShowString(2, (uint8_t)(5u + n), "D");

    /* 第 3 行脉宽 */
    OLED_ShowString(3, 1, "                ");
    OLED_ShowString(3, 1, "PULSE:");
    n = OLED_Digits(pulse);
    OLED_ShowNum(3, 7, pulse, n);
    OLED_ShowString(3, (uint8_t)(7u + n), "us");

    OLED_ShowString(4, 1, "A12:+10 A11:-10 ");
}

/* ===== moved from main.c (logic unchanged): servo270_test ===== */
/* 270° 舵机按键标定主循环：不返回。 */
void servo270_test(void)
{
    uint8_t  key;
    uint16_t angle = SERVO_ANGLE_MIN;   /* 从 0° 开始，方便对照实物 */

    Servo_SetAngle(SERVO_ROT, angle);
    OLED_Clear();
    servo270_draw(angle);

    while(1){
        key = Key_GetNum();

        if(key == 4u){                                  /* PA12：+10° */
            if((uint32_t)angle + SERVO_STEP_DEG > SERVO_ANGLE_MAX){
                Delay_ms(10);
                continue;                               /* 到顶：PWM、OLED 都不动 */
            }
            angle = (uint16_t)(angle + SERVO_STEP_DEG);
        }else if(key == 5u){                            /* PA11：-10° */
            if(angle < SERVO_STEP_DEG){
                Delay_ms(10);
                continue;                               /* 到底：PWM、OLED 都不动 */
            }
            angle = (uint16_t)(angle - SERVO_STEP_DEG);
        }else{
            Delay_ms(10);
            continue;
        }

        Servo_SetAngle(SERVO_ROT, angle);
        servo270_draw(angle);
    }
}

/* ===== moved from main.c (logic unchanged): servo_tune_test ===== */
/* ============================================================================
 * 舵机手动调节测试（当前使用的测试）
 * ----------------------------------------------------------------------------
 * 三个按键（都是"按下为高"，已配下拉输入）：
 *   PA5 = 切换当前调节的舵机：旋转(PA1) ⇄ 夹爪(PA0)
 *   PB1 = 当前舵机角度 +10°
 *   PA7 = 当前舵机角度 -10°
 *
 * 角度范围 0~270°；到边界后再按不会变化（不会顶到机械限位死顶）。
 * 每次按键 PC13 闪一下，确认按键被识别。
 *
 * OLED：
 *   第 1 行  SERVO TUNE
 *   第 2 行  SEL:ROT 240      选中的舵机 + 逻辑角度
 *   第 3 行  PUL:722 us       实际输出脉宽（旋转已取反，所以 240° = 722 us）
 *   第 4 行  PB1+ PA7- PA5:SW 按键提示
 * ========================================================================== */
void servo_tune_test(void)
{
    uint8_t  sel = (uint8_t)SERVO_ROT;      /* 当前调节哪一路 */
    uint16_t ang[SERVO_COUNT];              /* 两路的逻辑角度 */
    uint16_t pulse;
    uint8_t  dirty = 1u;

    /* 上电先到安全角度 */
    ang[SERVO_ROT]  = SERVO_ROT_INIT_DEG;
    ang[SERVO_GRIP] = SERVO_GRIP_INIT_DEG;
    Servo_SetAngle(SERVO_ROT,  ang[SERVO_ROT]);
    Servo_SetAngle(SERVO_GRIP, ang[SERVO_GRIP]);

    grab_key_init();                        /* PB1 / PA7 / PA5 下拉输入 */

    OLED_Clear();
    OLED_ShowString(1, 1, "SERVO TUNE      ");
    OLED_ShowString(4, 1, "PB1+ PA7- PA5:SW");

    while(1){
        /* ---------- PA5：切换舵机 ---------- */
        if(grab_key_down(GPIOA, GPIO_Pin_5)){
            Delay_ms(20);
            if(grab_key_down(GPIOA, GPIO_Pin_5)){
                grab_wait_release(GPIOA, GPIO_Pin_5);
                Delay_ms(20);
                sel = (sel == (uint8_t)SERVO_ROT) ? (uint8_t)SERVO_GRIP : (uint8_t)SERVO_ROT;
                dirty = 1u;
                pc13_step_blink();
                continue;
            }
        }

        /* ---------- PB1：+10° ---------- */
        if(grab_key_down(GPIOB, GPIO_Pin_1)){
            Delay_ms(20);
            if(grab_key_down(GPIOB, GPIO_Pin_1)){
                grab_wait_release(GPIOB, GPIO_Pin_1);
                Delay_ms(20);
                if((uint16_t)(ang[sel] + 10u) <= SERVO_ANGLE_MAX){
                    ang[sel] = (uint16_t)(ang[sel] + 10u);
                    Servo_SetAngle((ServoId)sel, ang[sel]);
                    dirty = 1u;
                }
                pc13_step_blink();
                continue;
            }
        }

        /* ---------- PA7：-10° ---------- */
        if(grab_key_down(GPIOA, GPIO_Pin_7)){
            Delay_ms(20);
            if(grab_key_down(GPIOA, GPIO_Pin_7)){
                grab_wait_release(GPIOA, GPIO_Pin_7);
                Delay_ms(20);
                if(ang[sel] >= 10u){
                    ang[sel] = (uint16_t)(ang[sel] - 10u);
                    Servo_SetAngle((ServoId)sel, ang[sel]);
                    dirty = 1u;
                }
                pc13_step_blink();
                continue;
            }
        }

        /* ---------- 显示刷新（只在角度/选择变化时刷） ---------- */
        if(dirty){
            dirty = 0u;
            pulse = Servo_GetPulse((ServoId)sel);

            OLED_ShowString(2, 1, "SEL:            ");        /* 先清整行，避免旧数字残留 */
            OLED_ShowString(2, 1, "SEL:");
            OLED_ShowString(2, 5, (sel == (uint8_t)SERVO_ROT) ? "ROT " : "GRP ");
            OLED_ShowNum(2, 9, ang[sel], OLED_Digits(ang[sel]));

            OLED_ShowString(3, 1, "PUL:            ");
            OLED_ShowString(3, 1, "PUL:");
            OLED_ShowNum(3, 5, pulse, OLED_Digits(pulse));
            OLED_ShowString(3, 9, "us");

            /* 到边界时给个提示，方便知道"再按也不会动" */
            if(ang[sel] >= SERVO_ANGLE_MAX)      OLED_ShowString(3, 12, "MAX");
            else if(ang[sel] == 0u)              OLED_ShowString(3, 12, "MIN");
            else                                 OLED_ShowString(3, 12, "   ");
        }

        Delay_ms(5);
    }
}
