#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "vision.h"
#include "Serial.h"
#include "OLED.h"
#include "Delay.h"
#include "motor.h"
#include "lift_motor.h"
#include "chassis.h"
#include "chassis_config.h"
#include "Key.h"

/* ============================================================================
 * 视觉闭环 —— 从 main.c 整块搬过来，逻辑未改
 * ========================================================================== */

/* OLED 行缓存：内容和上次一样就不重写（省 I2C 时间，见 vision_row 的注释） */
static char    vision_last[5][17];
static uint8_t vision_last_valid = 0u;

/* 点动标定结果：没有串口时就靠这个在屏上看"坐标变了多少" */
static int16_t  v_dbx = 0;              /* 上一次点动后 X 的变化（像素） */
static int16_t  v_dby = 0;              /* 上一次点动后 Y 的变化 */
static uint8_t  v_dx_ok = 0u;           /* 1 = 这两个数是真测到的（视觉有数据） */
static uint16_t v_calib_ticks = 0u;     /* 标定画面还要显示多少拍（0 = 不显示） */

/* ===== moved from main.c (logic unchanged): vision_row_invalidate ===== */
/* 清屏 / 强制整屏重画之前调用：让缓存失效，否则会以为"已经画过了"而不写 */
void vision_row_invalidate(void)
{
    vision_last_valid = 0u;
}

/* ===== moved from main.c (logic unchanged): vision_row ===== */
void vision_row(uint8_t line, const char *text)
{
    char    buf[17];
    uint8_t i;

    if((line < 1u) || (line > 4u)) return;

    for(i = 0u; i < 16u; i++) buf[i] = ' ';
    buf[16] = '\0';
    for(i = 0u; (i < 16u) && (text[i] != '\0'); i++) buf[i] = text[i];

    if(vision_last_valid)
    {
        for(i = 0u; i <= 16u; i++)
        {
            if(vision_last[line][i] != buf[i]) break;
        }
        if(i > 16u) return;                    /* 和屏上一模一样，不用写 */
    }

    OLED_ShowString(line, 1, buf);

    for(i = 0u; i <= 16u; i++) vision_last[line][i] = buf[i];
    vision_last_valid = 1u;
}

/* ===== moved from main.c (logic unchanged): vision_fmt_2dp ===== */
/* 把 0.01 单位的整数变成 "+1.92" / "-0.04" */
void vision_fmt_2dp(char *out, int32_t v)
{
    int32_t a = (v < 0L) ? -v : v;

    sprintf(out, "%c%d.%02d", (v < 0L) ? '-' : '+',
            (int)(a / 100L), (int)(a % 100L));
}

/* ===== moved from main.c (logic unchanged): vision_fmt_signed ===== */
/* 把整数变成 "+96" / "-12"（Keil 的 sprintf 对 %+d 支持不稳，自己拼） */
void vision_fmt_signed(char *out, int32_t v)
{
    int32_t a = (v < 0L) ? -v : v;

    sprintf(out, "%c%d", (v < 0L) ? '-' : '+', (int)a);
}

/* ===== moved from main.c (logic unchanged): vision_px_to_step ===== */
/* 像素偏差 → 这一步实际走多少毫米。
 *   1) 按 60px = 35mm 算理论距离
 *   2) 乘 VISION_STEP_GAIN_PCT 打折（识别不准 + 相机 3 Hz，一次补满必然冲过头）
 *   3) 夹到 1 ~ VISION_MAX_STEP_MM（单步绝对不会超过这个数）
 * px=0 返回 0（不动）。
 *
 * 注意顺序：先打折、再夹上限。反过来的话"上限 10mm × 50%"就只剩 5mm 了，
 * VISION_MAX_STEP_MM 这个数就失去意义。 */
uint32_t vision_px_to_step(int32_t px)
{
    uint32_t p, mm;

    if(px < 0L) p = (uint32_t)(-(px + 1L)) + 1u;   /* 两补码取绝对值，INT32_MIN 也安全 */
    else        p = (uint32_t)px;
    if(p == 0u) return 0u;

    mm = (p * VISION_MM_PER_60PX + 30u) / 60u;              /* 理论距离 */
    mm = (mm * VISION_STEP_GAIN_PCT) / 100u;                /* 打折 */
    if(mm < 1u) mm = 1u;
    if(mm > VISION_MAX_STEP_MM) mm = VISION_MAX_STEP_MM;    /* 单步上限 */
    return mm;
}

/* ===== moved from main.c (logic unchanged): vision_stop_all ===== */
/* 急停：4 个底盘电机 + 升降电机，全发 [addr] FE 98 00 6B。
 * 速度模式下再补一个 Chassis_Stop()（按加速度斜坡停车），双保险。 */
void vision_stop_all(void)
{
    uint8_t a;

    for(a = 1u; a <= 4u; a++)
    {
        Serial_SendByte(a);
        Serial_SendByte(0xFE);
        Serial_SendByte(0x98);
        Serial_SendByte(0x00);
        Serial_SendByte(0x6B);
        Delay_ms(2);
    }
    lift_motor_stop();
#if VISION_USE_SPEED_MODE
    Chassis_Stop();
#endif
}

/* ===== moved from main.c (logic unchanged): vision_move_fwd ===== */
/* 前后平移：mm > 0 前进，mm < 0 后退 */
void vision_move_fwd(int32_t mm)
{
    if(mm == 0L) return;
#if VISION_USE_SPEED_MODE
    (void)Chassis_MoveAt(mm, (uint16_t)VISION_SPEED_X10);
#else
    if(mm > 0L) qianjin_((uint32_t)mm);
    else        houtui_((uint32_t)(-mm));
#endif
}

/* ===== moved from main.c (logic unchanged): vision_move_lat ===== */
/* 左右平移：mm > 0 左移，mm < 0 右移（chassis.h 的约定） */
void vision_move_lat(int32_t mm)
{
    if(mm == 0L) return;
#if VISION_USE_SPEED_MODE
    (void)Chassis_StrafeAt(mm, (uint16_t)VISION_SPEED_X10);
#else
    if(mm > 0L) zuo_((uint32_t)mm);
    else        you_((uint32_t)(-mm));
#endif
}

/* ===== moved from main.c (logic unchanged): vision_rotate ===== */
/* 旋转：centi > 0 顺时针，centi < 0 逆时针（单位 0.01°）。
 * 注意 chassis.h 的约定是"度，>0 逆时针"，和这里正好相反，所以取负。
 * VISION_YAW_INVERT=1 时整体翻转（实测方向反了就用它，不用改别的地方）。 */
void vision_rotate(int32_t centi)
{
    if(centi == 0L) return;

#if VISION_YAW_INVERT
    centi = -centi;
#endif

#if VISION_USE_SPEED_MODE
    {
        int32_t deg = (centi + ((centi > 0L) ? 50L : -50L)) / 100L;   /* 四舍五入到整度 */
        if(deg != 0L) (void)Chassis_RotateAt(-deg, (uint16_t)VISION_ROT_SPEED_X10);
    }
#else
    {
        uint8_t  dir = (centi > 0L) ? 0x00u : 0x01u;
        uint32_t p   = (uint32_t)((centi > 0L) ? centi : -centi);
        (void)motor_rotate_centideg(dir, (int32_t)p);
    }
#endif
}

/* ===== moved from main.c (logic unchanged): vision_step_ms ===== */
/* 一步大概要多久（ms）：脉冲数 ÷ 每秒脉冲 + 梯形斜坡余量 + 停顿。
 * 用来把"这一步要多少 ms"换算成主循环要等多少拍（见 VISION_LOOP_MS）。
 * 只是个估算，但足够避免"前一步还没走完就叠下一条命令"。 */
uint16_t vision_step_ms(int32_t mm)
{
    uint32_t p = (uint32_t)((mm < 0L) ? -mm : mm);
    uint32_t pulses, pps, ms;

    if(p == 0u) return 0u;

    pulses = p * MOTOR_PULSE_PER_100CM / 1000u;                    /* 约 13 脉冲/mm */
    pps    = (uint32_t)VISION_POS_SPEED_RPM
             * (uint32_t)CHASSIS_PULSES_PER_REV / 60u;             /* 8 RPM -> 426 脉冲/s */
    if(pps == 0u) pps = 1u;

    ms  = pulses * 1000u / pps;                                    /* 匀速段 */
    ms += VISION_RAMP_MS;                                          /* 梯形加减速 */
    ms += VISION_STEP_GAP_MS;                                      /* 停顿 */
    if(ms < 120u) ms = 120u;
    return (uint16_t)ms;
}

/* ===== moved from main.c (logic unchanged): vision_circle_issue ===== */
/* CIRCLE 一步：**不阻塞**。发完命令立刻返回，主循环接着收帧/刷屏。
 *
 * 返回 = 这一步大概需要多少 ms（0 = 已在死区，没动）。
 * 调用方用它决定"多少拍之内不发新命令"，而不是傻等。
 *
 * 一次只动一个轴（电流最小、最不抽），优先修偏差大的那个。
 * 这样两个轴会交替收敛，比"永远先修 x 再修 y"快。 */
uint16_t vision_circle_issue(int32_t cx, int32_t cy, char *act)
{
    int32_t  dx = cx - (int32_t)VISION_TARGET_X;
    int32_t  dy = cy - (int32_t)VISION_TARGET_Y;
    int32_t  adx = (dx < 0L) ? -dx : dx;
    int32_t  ady = (dy < 0L) ? -dy : dy;
    uint8_t  dx_bad = ((dx > VISION_DEADBAND_PX) || (dx < -VISION_DEADBAND_PX)) ? 1u : 0u;
    uint8_t  dy_bad = ((dy > VISION_DEADBAND_PX) || (dy < -VISION_DEADBAND_PX)) ? 1u : 0u;
    uint32_t mm;

    if(!dx_bad && !dy_bad)
    {
        sprintf(act, "ALIGNED");
        return 0u;
    }

    /* 两个轴都超死区时，先修大的那个 */
    if(dx_bad && (!dy_bad || (adx >= ady)))
    {
        mm = vision_px_to_step(dx);
        if(dx > 0L)
        {
#if VISION_DX_PLUS_DO_BACK
            vision_move_fwd(-(int32_t)mm);
            sprintf(act, "BACK %umm", (unsigned)mm);
#else
            vision_move_fwd((int32_t)mm);
            sprintf(act, "AHEAD %umm", (unsigned)mm);
#endif
        }
        else
        {
#if VISION_DX_PLUS_DO_BACK
            vision_move_fwd((int32_t)mm);
            sprintf(act, "AHEAD %umm", (unsigned)mm);
#else
            vision_move_fwd(-(int32_t)mm);
            sprintf(act, "BACK %umm", (unsigned)mm);
#endif
        }
        return vision_step_ms((int32_t)mm);
    }

    mm = vision_px_to_step(dy);
    if(dy > 0L)
    {
#if VISION_DY_PLUS_DO_RIGHT
        vision_move_lat(-(int32_t)mm);                /* dy>0 -> 右移 */
        sprintf(act, "RIGHT %umm", (unsigned)mm);
#else
        vision_move_lat((int32_t)mm);                 /* dy>0 -> 左移 */
        sprintf(act, "LEFT %umm", (unsigned)mm);
#endif
    }
    else
    {
#if VISION_DY_PLUS_DO_RIGHT
        vision_move_lat((int32_t)mm);                 /* dy<0 -> 左移 */
        sprintf(act, "LEFT %umm", (unsigned)mm);
#else
        vision_move_lat(-(int32_t)mm);                /* dy<0 -> 右移 */
        sprintf(act, "RIGHT %umm", (unsigned)mm);
#endif
    }
    return vision_step_ms((int32_t)mm);
}

/* ===== moved from main.c (logic unchanged): vision_rot_ms ===== */
/* 旋转一步大概要多久（ms）：脉冲数 ÷ 每秒脉冲 + 梯形斜坡 + 停顿。
 * 脉冲/度 = MOTOR_PULSE_PER_360DEG / 360 = 17208/360 ≈ 47.8。
 * 注意 p_centi 是 0.01°，所以除 36000。 */
uint16_t vision_rot_ms(uint32_t p_centi)
{
    uint32_t pulses, pps, ms;

    if(p_centi == 0u) return 0u;

    pulses = p_centi * (uint32_t)MOTOR_PULSE_PER_360DEG / 36000u;
    pps    = (uint32_t)VISION_POS_SPEED_RPM
             * (uint32_t)CHASSIS_PULSES_PER_REV / 60u;
    if(pps == 0u) pps = 1u;

    ms  = pulses * 1000u / pps;
    ms += VISION_RAMP_MS;
    ms += VISION_STEP_GAP_MS;
    if(ms < 120u) ms = 120u;
    return (uint16_t)ms;
}

/* ===== moved from main.c (logic unchanged): vision_yaw_issue ===== */
/* LINEAR 一步：**不阻塞**，返回这一步大概要多久（0 = 已在死区）。
 *
 * ★ 这里不再等 0x3A 回包。原因：0x3A 只说明"驱动器回没回应"，和
 *   "命令有没有发出去"是两回事 —— 用它当失败条件就会出现
 *   "电机明明在动，OLED 却一直显示 MOTOR TIMEOUT"。
 *   想确认驱动器在不在，用 PA7 点动（那条专门查 0x3A）。 */
uint16_t vision_yaw_issue(int16_t psi_centi, char *act)
{
    int32_t  v = (int32_t)psi_centi;
    uint8_t  dir;
    uint32_t p;

    if((v <= (int32_t)VISION_PSI_DEADBAND) &&
       (v >= -(int32_t)VISION_PSI_DEADBAND))
    {
        sprintf(act, "YAW OK");
        return 0u;
    }

    if(v >  (int32_t)VISION_PSI_MAX_STEP) v =  (int32_t)VISION_PSI_MAX_STEP;
    if(v < -(int32_t)VISION_PSI_MAX_STEP) v = -(int32_t)VISION_PSI_MAX_STEP;

    /* 旋转也一样打折：别一次转到满，慢慢靠过去，不然画面晃得识别更不准 */
    v = (v * (int32_t)VISION_STEP_GAIN_PCT) / 100L;
    if(v == 0L){ sprintf(act, "YAW OK"); return 0u; }

    if(v > 0L){ dir = 0x00u; p = (uint32_t)v; }        /* 正 = 顺时针 */
    else      { dir = 0x01u; p = (uint32_t)(-v); }     /* 负 = 逆时针 */

#if VISION_YAW_INVERT
    dir ^= 0x01u;                                      /* 实测方向反了就翻过来 */
#endif

    sprintf(act, "%s %u.%02udeg", (dir == 0x00u) ? "CW" : "CCW",
            (unsigned)(p / 100u), (unsigned)(p % 100u));

    vision_rotate((v > 0L) ? (int32_t)p : -(int32_t)p);   /* 正数=CW */

    return vision_rot_ms(p);
}

/* ===== moved from main.c (logic unchanged): vision_jog ===== */
uint8_t vision_jog(uint8_t idx, char *act)
{
    switch(idx % VISION_JOG_COUNT)
    {
        case 0u: qianjin_(VISION_JOG_MM);  sprintf(act, "JOG AHEAD");  break;
        case 1u: houtui_(VISION_JOG_MM);   sprintf(act, "JOG BACK");   break;
        case 2u: zuo_(VISION_JOG_MM);      sprintf(act, "JOG LEFT");   break;
        case 3u: you_(VISION_JOG_MM);      sprintf(act, "JOG RIGHT");  break;
        case 4u: shun_yaw_(VISION_JOG_DEG);sprintf(act, "JOG CW");     break;
        default: ni_yaw_(VISION_JOG_DEG);  sprintf(act, "JOG CCW");    break;
    }

    USART3_Printf("[VISION] %s（%s %u）\r\n", act,
                  ((idx % VISION_JOG_COUNT) < 4u) ? "mm" : "deg",
                  (unsigned)(((idx % VISION_JOG_COUNT) < 4u) ? VISION_JOG_MM
                                                             : VISION_JOG_DEG));

    return judge_wait(VISION_JUDGE_ROUNDS) ? 1u : 0u;
}

/* ===== moved from main.c (logic unchanged): vision_lift_to ===== */
/* 举高机械臂到 target 脉冲（绝对位置），方向 0x01 = CCW = 升。
 * ★ 已经在目标高度就**直接返回**，不重发也不白等 —— CIRCLE ⇄ LINEAR 来回切
 *   的时候很常见（两个模式都要举到同一个高度）。
 * 等待时间按"当前位置 → 目标位置"的差值估算，每 10 ms 查一次 PA5，随时能中止。
 * 返回 1 = 到位（或本来就在），0 = 被 PA5 中止。 */
uint8_t vision_lift_to(uint32_t target, char *act)
{
    uint16_t t, total;
    uint32_t from, diff, ms;

    sprintf(act, "LIFT %u", (unsigned)target);

    if(g_lift_target == target)
    {
        USART3_Printf("[VISION] lift already at %u，跳过\r\n", (unsigned)target);
        return 1u;
    }

    from = g_lift_target;                    /* 发命令前先记下旧值 */

    motor_enable_addr(LIFT_MOTOR_ADDR);      /* 地址 6 上电默认失能 */
    Delay_ms(20);

    lift_motor_move(target, 0x01u);          /* 绝对位置 + CCW 升 */

    diff = (target > from) ? (target - from) : (from - target);
    ms   = (diff * VISION_LIFT_MS_PER_1000P) / 1000u;
    if(ms < 500u) ms = 500u;
    if(ms > VISION_LIFT_WAIT_MAX) ms = VISION_LIFT_WAIT_MAX;
    total = (uint16_t)(ms / 10u);

    USART3_Printf("[VISION] lift %u -> %u（%u 脉冲，约 %u ms）\r\n",
                  (unsigned)from, (unsigned)target, (unsigned)diff, (unsigned)ms);

    for(t = 0u; t < total; t++)
    {
        if(grab_abort_hit())
        {
            lift_motor_stop();
            return 0u;
        }

        /* 超过 1 秒才报进度，别让人以为卡住了 */
        if((total > 100u) && ((t % 100u) == 0u))
        {
            USART3_Printf("[VISION] lifting %u%%\r\n",
                          (unsigned)(((uint32_t)t * 100u) / (uint32_t)total));
        }

        Delay_ms(10);
    }
    USART3_Printf("[VISION] lift done\r\n");
    return 1u;
}

/* ===== moved from main.c (logic unchanged): vision_link_test ===== */
void vision_link_test(void)
{
    uint8_t  mode    = VISION_IDLE;
    uint8_t  got_any = 0u;
    uint8_t  last_ng = 0u;
    uint32_t cseq = 0u, lseq = 0u, ngseq = 0u;
    uint16_t quiet = 0u;
    uint16_t tick  = 0u;
    uint8_t  rounds = 0u;               /* LINEAR 已经调了几轮 */
    uint8_t  jog_idx = 0u;              /* PA7 点动自检轮到哪个动作 */
    uint16_t v_wait = 0u;               /* CIRCLE：还要等多少拍才能发下一步（非阻塞） */
    uint16_t v_n = 0u;                  /* CIRCLE：已经走了多少步（统计频率用） */
    uint16_t v_tick = 0u;               /* CIRCLE：上一步是在哪个 tick 发的 */
    uint32_t v_interval_sum = 0u;       /* CIRCLE：累计间隔（tick 数），算平均用 */
    uint8_t  v_retry = 0u;              /* K230 静默后重发模式命令的次数 */
    uint16_t v_resend_wait = 0u;        /* 距离下次重发还有多少拍 */
    int32_t  last_abs_psi = -1L;        /* 上一轮的 |psi|，用来判"越修越大" */
    char     row[24];
    char     kbuf[12], pbuf[12];
    char     sbx[8], sby[8];
    char     dbx[8], dby[8];
    char     act[17];                   /* 第 4 行显示的动作 */

    grab_key_init();                     /* PB1 / PA7 / PA5：下拉输入，按下为高 */

#if VISION_USE_SPEED_MODE
    /* 速度模式底盘：使能四轮 + 设一个柔和的加速度 */
    Chassis_Init();
    Chassis_SetAccel((uint16_t)VISION_ACCEL_RPM_S);
#else
    /* 位置模式：把速度和加速度档都压到很低 —— 驱动器按梯形曲线软起软停。
     * 这一步很关键：光降速度不降加速度，起步/停车那一下仍然是冲击。 */
    motor_set_profile((uint16_t)VISION_POS_SPEED_RPM, (uint16_t)VISION_POS_ACCEL);
#endif

    /* ---------- 开机先画一屏静态内容，专门用来验屏 ----------
     * 如果这一屏也"只有上半截"，那就是驱动/接线的事，跟视觉数据无关；
     * 这一屏正常、进了循环才坏，那就是刷新太频繁 / I2C 事务丢失。 */
    OLED_Clear();
    vision_row_invalidate();
    vision_row(1u, "OLED SELF TEST");
    vision_row(2u, "ABCDEFGHIJKLMNOP");
    vision_row(3u, "1234567890+-,.:;");
    vision_row(4u, "VISION v2");
    Delay_ms(1200);

    sprintf(act, "READY");

    OLED_Clear();
    vision_row_invalidate();
    vision_row(1u, "VISION IDLE");
    vision_row(2u, "PB1 MODE PA7 JOG");
    vision_row(3u, "PA5 STOP");
    vision_row(4u, "[READY]");

    USART3_SendString("\r\n[VISION v3] PB1=mode  PA7=jog  PA5=stop\r\n");
    USART3_Printf("[VISION] 目标点(夹爪) = (%d,%d)，画面中心 = (%d,%d)\r\n",
                  VISION_TARGET_X, VISION_TARGET_Y, VISION_CX, VISION_CY);
    USART3_Printf("[VISION] 两种模式都会先举高到 %u 脉冲（约 %u ms）\r\n",
                  (unsigned)VISION_LIFT_UP,
                  (unsigned)((VISION_LIFT_UP * VISION_LIFT_MS_PER_1000P) / 1000u));
#if VISION_USE_SPEED_MODE
    USART3_Printf("[VISION] 速度模式：平移 %u.%uRPM 旋转 %u.%uRPM 加速度 %u RPM/s\r\n",
                  (unsigned)(VISION_SPEED_X10 / 10u), (unsigned)(VISION_SPEED_X10 % 10u),
                  (unsigned)(VISION_ROT_SPEED_X10 / 10u), (unsigned)(VISION_ROT_SPEED_X10 % 10u),
                  (unsigned)VISION_ACCEL_RPM_S);
#else
    USART3_Printf("[VISION] 位置模式：速度 %u RPM，加速度档 %u（梯形软起软停）\r\n",
                  (unsigned)motor_get_speed(), (unsigned)motor_get_accel());
#endif
    USART3_Printf("[VISION] 步长：增益 %u%%，单步上限 %u mm，步间停 %u ms，死区 ±%d px\r\n",
                  (unsigned)VISION_STEP_GAIN_PCT, (unsigned)VISION_MAX_STEP_MM,
                  (unsigned)VISION_STEP_GAP_MS, VISION_DEADBAND_PX);
    USART3_Printf("[VISION] K230 静默会每 %u ms 自动重发模式命令，最多 %u 次\r\n",
                  (unsigned)(VISION_RESEND_GAP * VISION_LOOP_MS),
                  (unsigned)VISION_RESEND_MAX);
    USART3_Printf("[VISION] dx>0 -> %s，dy>0 -> %s\r\n",
                  VISION_DX_PLUS_DO_BACK ? "后退" : "前进",
                  VISION_DY_PLUS_DO_RIGHT ? "右移" : "左移");
#if VISION_YAW_INVERT
    USART3_SendString("[VISION] 旋转方向已翻转（VISION_YAW_INVERT=1）\r\n");
#endif

    while(1)
    {
        uint8_t changed = 0u;

        /* ---------- PB1：切换模式（IDLE → CIRCLE → LINEAR → CIRCLE …） ----------
         * 两个模式进来都先把机械臂举到 VISION_LIFT_UP：机械臂低了摄像头看不见圆。 */
        if(grab_step_hit())
        {
            uint8_t next = (mode == VISION_CIRCLE) ? VISION_LINEAR : VISION_CIRCLE;

            mode = next;
            got_any = 0u; last_ng = 0u; quiet = 0u;
            rounds = 0u; last_abs_psi = -1L;
            v_wait = 0u; v_n = 0u; v_interval_sum = 0u;      /* 循环统计重新开始 */
            changed = 1u;

            vision_row(1u, (next == VISION_CIRCLE) ? "CIRCLE LIFTING" : "LINEAR LIFTING");
            vision_row(4u, "[LIFTING]");

            if(!vision_lift_to(VISION_LIFT_UP, act))
            {
                mode = VISION_IDLE;
                sprintf(act, "LIFT ABORT");
                USART3_SendString("[VISION] lift aborted by PA5\r\n");
            }
            else if(next == VISION_CIRCLE)
            {
                cseq = K230_CircleSeq;
                K230_SendLine(K230_CMD_CIRCLE);
                sprintf(act, "CIRCLE");
                USART3_SendString("[VISION] TX circle\r\n");
            }
            else
            {
                lseq = K230_LineSeq;
                K230_SendLine(K230_CMD_LINEAR);
                USART3_SendString("[VISION] TX linear\r\n");
            }
            changed = 1u;
        }
        else if(grab_abort_hit())                            /* PA5：全部停下 */
        {
            vision_stop_all();
            K230_SendLine(K230_CMD_STOP);
            mode = VISION_IDLE;
            got_any = 0u; last_ng = 0u; quiet = 0u; rounds = 0u;
            v_wait = 0u; v_n = 0u; v_interval_sum = 0u;
            v_calib_ticks = 0u;                  /* PA5 也用来清掉标定画面 */
            sprintf(act, "STOPPED");
            changed = 1u;
            USART3_SendString("[VISION] STOP (motors stopped)\r\n");
        }
        else if(grab_start_hit())                            /* PA7：点动自检 */
        {
            /* 前后左右 + 两个旋转轮流来一遍，用来确认"车到底动不动、方向对不对"。
             * 和视觉无关，所以任何时候按都有效。
             * 如果视觉正在出数据（CIRCLE 模式），顺手量一下"这个动作让圆心坐标变了多少"
             * —— 这就是标定方向宏的第一手数据，不用再猜。 */
            int32_t  x0 = (int32_t)K230_Cx;
            int32_t  y0 = (int32_t)K230_Cy;
            uint32_t s0 = K230_CircleSeq;
            uint16_t w;

            v_dx_ok = 0u;                       /* 先假设测不到，下面真测到再置 1 */

            if(!vision_jog(jog_idx, act))
            {
                sprintf(act, "JOG NO REPLY");
                USART3_Printf("[VISION] PA7 点动：命令发出去了但没收到 0x3A 回包。\r\n");
                USART3_Printf("         这不等于电机没动 —— 先看轮子转了没有：\r\n");
                USART3_Printf("           转了   -> 只是 PA10(RX) 没接/接错，回包收不到，控制照常能用\r\n");
                USART3_Printf("           没转   -> 查 24V、PA9(TX) 到驱动器 RX、共地、驱动器地址、F3 使能\r\n");
                vision_stop_all();
            }
            jog_idx++;
            changed = 1u;

            if((mode == VISION_CIRCLE) && (s0 != 0u))
            {
                for(w = 0u; w < 200u; w++)          /* 最多等 1 s 等新一帧 */
                {
                    if(K230_CircleSeq != s0) break;
                    if(grab_abort_hit()) break;
                    Delay_ms(5);
                }
                if(K230_CircleSeq != s0)
                {
                    v_dbx = (int16_t)((int32_t)K230_Cx - x0);
                    v_dby = (int16_t)((int32_t)K230_Cy - y0);
                    v_dx_ok = 1u;
                    vision_fmt_signed(dbx, (int32_t)v_dbx);
                    vision_fmt_signed(dby, (int32_t)v_dby);
                    USART3_Printf("[VISION] 方向标定：%s 之后 dx%s dy%s\r\n",
                                  act, dbx, dby);
                }
            }

            /* 没有串口也能看到：把标定画面顶到屏上 8 秒左右（1000 拍 × 4ms ≈ 4 s） */
            v_calib_ticks = 1000u;
            changed = 1u;
        }

        /* ---------- CIRCLE：非阻塞的循环控制 ----------
         * 每来一帧新坐标就想动一步；"上一步还在走"时先等 —— 用计数等，
         * 不用 Delay，所以等的时候照常收帧、照常刷屏、照常响应按键。 */
        if(mode == VISION_CIRCLE)
        {
            if(v_wait > 0u) v_wait--;

            if((K230_CircleSeq != cseq) && (v_wait == 0u))
            {
                uint16_t need;

                cseq = K230_CircleSeq;
                got_any = 1u; last_ng = 0u; quiet = 0u;

                need = vision_circle_issue((int32_t)K230_Cx, (int32_t)K230_Cy, act);
                v_wait = (uint16_t)((need + VISION_LOOP_MS - 1u) / VISION_LOOP_MS);

                if(need != 0u)
                {
                    /* 统计闭环频率：最近 20 步的平均间隔（tick 是 uint16_t，要按模算差值） */
                    if(v_n != 0u)
                    {
                        v_interval_sum += (uint32_t)((uint16_t)(tick - v_tick));
                    }
                    v_tick = tick;
                    v_n++;

                    if((v_n % 20u) == 0u)
                    {
                        uint32_t gap_ms = v_interval_sum * VISION_LOOP_MS / (uint32_t)(v_n - 1u);
                        if(gap_ms == 0u) gap_ms = 1u;
                        USART3_Printf("[VISION] 已走 %u 步，平均 %u ms/步（约 %u 步/秒）\r\n",
                                      (unsigned)v_n, (unsigned)gap_ms,
                                      (unsigned)(1000u / gap_ms));
                    }
                }

                USART3_Printf("[VISION] circle (%d,%d) -> %s (%u ms)\r\n",
                              (int)K230_Cx, (int)K230_Cy, act, (unsigned)need);
            }
        }

        /* ---------- LINEAR：非阻塞，来了新偏航就修一步 ---------- */
        if(mode == VISION_LINEAR)
        {
            if(v_wait > 0u) v_wait--;

            if((K230_LineSeq != lseq) && (v_wait == 0u))
            {
                int16_t  psi = K230_PsiCenti;
                int32_t  ap  = (psi < 0) ? (int32_t)(-psi) : (int32_t)psi;
                uint16_t need;

                lseq = K230_LineSeq;
                got_any = 1u; last_ng = 0u; quiet = 0u; changed = 1u;

                if((rounds > 0u) &&
                   (last_abs_psi >= (int32_t)VISION_PSI_DEADBAND) &&
                   (ap > last_abs_psi))
                {
                    /* 越修越大：方向反了，或者目标换了 -> 立刻停，别再动 */
                    sprintf(act, "DIVERGE STOP");
                    USART3_Printf("[VISION] psi %d -> %d 变大了，判定方向反了，停止\r\n",
                                  (int)last_abs_psi, (int)ap);
                    vision_stop_all();
                    mode = VISION_IDLE;
                }
                else if(rounds >= VISION_ALIGN_ROUNDS)
                {
                    sprintf(act, "ROUNDS OUT");
                    USART3_Printf("[VISION] 调了 %u 轮还没到 0.6°，停下\r\n",
                                  (unsigned)rounds);
                    mode = VISION_IDLE;
                }
                else
                {
                    need = vision_yaw_issue(psi, act);          /* 不等回包，只看时间 */
                    v_wait = (uint16_t)((need + VISION_LOOP_MS - 1u) / VISION_LOOP_MS);
                    rounds++;
                    last_abs_psi = ap;
                }

                USART3_Printf("[VISION] psi=%d -> %s (%u ms, round %u)\r\n",
                              (int)psi, act, (unsigned)need, (unsigned)rounds);
            }
        }

        /* NG：只记数，绝不动作 */
        if(K230_NgSeq != ngseq)
        {
            ngseq = K230_NgSeq; got_any = 1u; last_ng = 1u; quiet = 0u; changed = 1u;
        }

        if(quiet < VISION_QUIET_MAX) quiet++;
        if(quiet <= 1u) v_retry = 0u;        /* 刚收到新帧：重试计数清零 */

        /* ---------- K230 掉线自己接回来 ----------
         * K230 被电源拉重启后会忘掉当前模式、不再回数据，主控只看到 TMO。
         * 这里静默超过 1 s 就重发一次模式命令；最多连发 VISION_RESEND_MAX 次。 */
        if((mode != VISION_IDLE) && (quiet >= VISION_QUIET_MAX))
        {
            if(v_resend_wait > 0u)
            {
                v_resend_wait--;
            }
            else
            {
                if(v_retry < VISION_RESEND_MAX)
                {
                    K230_SendLine((mode == VISION_CIRCLE) ? K230_CMD_CIRCLE
                                                          : K230_CMD_LINEAR);
                    v_retry++;
                    changed = 1u;
                    USART3_Printf("[VISION] 静默 %ums，重发模式命令（第 %u 次）\r\n",
                                  (unsigned)quiet * VISION_LOOP_MS,
                                  (unsigned)v_retry);
                }
                v_resend_wait = VISION_RESEND_GAP;
            }
        }
        else
        {
            v_resend_wait = 0u;
        }

        /* 每约 2 秒：强制整屏重刷一次（防止偶发丢事务留下半个字），
         * 同时把 OLED 的 I2C 失败次数打到调试串口 —— 这个数是排查的关键：
         *   一直在涨  -> I2C 时序/接线问题（100 kHz 还涨就得查线和上拉）
         *   一直是 0  -> 不是总线问题，看内容/坐标 */
        if((tick % 500u) == 0u)
        {
            vision_row_invalidate();
            USART3_Printf("[oled] i2c_err=%u\r\n", (unsigned)OLED_GetI2cErrors());
        }

        /* ---------- 刷屏：约每 300 ms 一次，按键/切模式立刻刷 ----------
         * ★ 不能每来一帧就刷：OLED 是 100 kHz 的 I2C，刷 3 行大约要 70 ms，
         *   比一步动作还长。以前每次收到坐标就整屏重画，把闭环频率直接砍半。 */
        if(changed || ((tick % 75u) == 0u))
        {
            const char *label;
            const char *state;

            /* ---------- 优先显示"点动标定"画面 ----------
             * 没有串口的时候，这就是唯一能看到"发了什么命令 + 坐标变了多少"的地方。
             * 你只需要看两件事：上面写的命令是什么、车实际往哪走了。 */
            if(v_calib_ticks > 0u)
            {
                v_calib_ticks--;

                vision_row(1u, act);                       /* 例如 JOG AHEAD */

                if(v_dx_ok)
                {
                    vision_fmt_signed(sbx, v_dbx);
                    vision_fmt_signed(sby, v_dby);
                    sprintf(row, "dx%s dy%s", sbx, sby);
                    vision_row(2u, row);
                }
                else
                {
                    vision_row(2u, "dx? dy? no vis");
                }

                vision_row(3u, "car went where?");
                vision_row(4u, "PA5 to clear");
                goto draw_done;
            }

            if(mode == VISION_CIRCLE)      label = "CIRCLE";
            else if(mode == VISION_LINEAR) label = "LINEAR";
            else                           label = "VISION";   /* -> "VISION IDLE" */

            if(mode == VISION_IDLE)              state = "IDLE";
            else if(!got_any)                    state = "WAIT";
            else if(quiet >= VISION_QUIET_MAX)   state = "TMO";
            else                                 state = "RUN";

            sprintf(row, "%s %s", label, state);
            vision_row(1u, row);

            if(mode == VISION_CIRCLE)
            {
                if(last_ng) vision_row(2u, "NG: no circle");
                else
                {
                    sprintf(row, "X%d Y%d R%d", (int)K230_Cx, (int)K230_Cy,
                            (int)K230_R);
                    vision_row(2u, row);
                }

                /* 第 3 行：相对**目标点**（夹爪位置）的偏差，这才是程序真正用来动的量 */
                vision_fmt_signed(sbx, (int32_t)K230_Cx - (int32_t)VISION_TARGET_X);
                vision_fmt_signed(sby, (int32_t)K230_Cy - (int32_t)VISION_TARGET_Y);
                sprintf(row, "dX%s dY%s", sbx, sby);
                vision_row(3u, row);
            }
            else if(mode == VISION_LINEAR)
            {
                if(last_ng) vision_row(2u, "NG: no line");
                else
                {
                    vision_fmt_2dp(pbuf, (int32_t)K230_PsiCenti);
                    sprintf(row, "psi%sdeg", pbuf);
                    vision_row(2u, row);
                }

                vision_fmt_2dp(kbuf, (int32_t)K230_K100);
                sprintf(row, "k%s NG%u", kbuf, (unsigned)K230_NgSeq);
                vision_row(3u, row);
            }
            else
            {
                vision_row(2u, "PB1 SW MODE");
                vision_row(3u, "PA7 JOG PA5 STP");
            }

            /* 第 4 行：待机时显示当前方向映射（没串口也能确认），其余显示最后的动作 */
            if(mode == VISION_IDLE)
            {
                sprintf(row, "%s %s",
                        VISION_DX_PLUS_DO_BACK ? "DX+BACK" : "DX+FWD",
                        VISION_DY_PLUS_DO_RIGHT ? "DY+RGT" : "DY+LFT");
                vision_row(4u, row);
            }
            else
            {
                sprintf(row, "[%s]", act);
                vision_row(4u, row);
            }

draw_done:
            ;
        }

        tick++;
        Delay_ms(4);
    }
}
