#include "Gyro.h"
#include "Delay.h"
#include "OLED.h"
#include "motor.h"
#include "motion.h"

static uint8_t frame[GYRO_FRAME_LEN];
static uint8_t frame_pos;
static volatile int16_t yaw_raw;
/* ★ 原来这里是 static，但 Gyro.h 里声明的是 extern —— 同一个名字两种链接性，
 *   单独编译会报 "static declaration follows non-static declaration"。
 *   按头文件的意图去掉 static（只改链接性，读写逻辑一点没动）。 */
volatile int32_t yaw_centi_degrees;
static volatile uint8_t yaw_valid;
volatile uint32_t Gyro_FrameCount;

/* ---- 闭环转向用的状态（原来是 main.c 的文件级 static，跟着函数搬过来）----
 * 改成全局是因为 main.c 里的路线逻辑和 OLED 显示还要读它们
 * （只改链接性，读写逻辑一点没动）。 */
uint8_t  gyro_ok;                  /* 收到过有效角度帧 */
int32_t  yaw_zero;                 /* 开机零点：所有绝对角度都相对它 */
uint8_t  yaw_zero_ok;
volatile int32_t turn_dbg_target;  /* 目标绝对角度，0.01° */
volatile int32_t turn_dbg_meas;    /* 当前绝对角度，0.01° */
volatile int32_t turn_dbg_err;     /* 最后误差，0.01° */
volatile uint8_t turn_dbg_corr;    /* 补差次数 */
volatile uint8_t turn_dbg_result;  /* 1 到位 0 失败 */

static void Gyro_ResetParser(void)
{
    frame_pos = 0;
}

static void Gyro_CommitAngle(void)
{
    uint16_t raw_u16;
    int16_t raw;

    raw_u16 = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);
    if (raw_u16 & 0x8000u)
        raw = (int16_t)(-((int32_t)(0x10000u - raw_u16)));
    else
        raw = (int16_t)raw_u16;

    yaw_raw = raw;
    yaw_centi_degrees = ((int32_t)raw * GYRO_ANGLE_SCALE_NUM) /
                        GYRO_ANGLE_SCALE_DEN;
    yaw_valid = 1;
    Gyro_FrameCount++;

}

void Gyro_Init(void)
{
    Gyro_ResetParser();
    yaw_raw = 0;
    yaw_centi_degrees = 0;
    yaw_valid = 0;
    Gyro_FrameCount = 0;
}

void Gyro_ProcessByte(uint8_t byte)
{
    uint8_t i;
    uint8_t checksum;

    /* 等待帧头。 */
    if (frame_pos == 0u)
    {
        if (byte == GYRO_FRAME_HEADER)
        {
            frame[0] = byte;
            frame_pos = 1u;
        }
        return;
    }

    /* 第二字节必须是角度帧类型；55 52 等其他 JY901 帧直接丢弃。 */
    if (frame_pos == 1u)
    {
        if (byte == GYRO_TYPE_ANGLE)
        {
            frame[1] = byte;
            frame_pos = 2u;
        }
        else if (byte == GYRO_FRAME_HEADER)
        {
            /* 当前字节可能已经是下一帧的帧头。 */
            frame[0] = byte;
            frame_pos = 1u;
        }
        else
        {
            Gyro_ResetParser();
        }
        return;
    }

    frame[frame_pos++] = byte;
    if (frame_pos < GYRO_FRAME_LEN)
        return;

    checksum = 0u;
    for (i = 0u; i < GYRO_FRAME_LEN - 1u; i++)
        checksum = (uint8_t)(checksum + frame[i]);

    if (checksum == frame[GYRO_FRAME_LEN - 1u])
    {
        Gyro_CommitAngle();
    }

    Gyro_ResetParser();
}

uint8_t Gyro_GetYawCentiDegrees(int32_t *centi_degrees)
{
    if (centi_degrees == 0 || !yaw_valid)
        return 0u;

    *centi_degrees = yaw_centi_degrees;
    return 1u;
}

uint8_t Gyro_GetYawRaw(int16_t *raw)
{
    if (raw == 0 || !yaw_valid)
        return 0u;

    *raw = yaw_raw;
    return 1u;
}

/* ===== moved from main.c (logic unchanged): gyro_wrap_180 ===== */
/* 把角度差限制到 [-180, 180) 度，输入/输出单位均为 0.01 度。 */
int32_t gyro_wrap_180(int32_t angle)
{
    while(angle >= 18000L) angle -= 36000L;
    while(angle < -18000L) angle += 36000L;
    return angle;
}

/* ===== moved from main.c (logic unchanged): gyro_wait_new_sample ===== */
/* 等待一帧新的有效角度，避免用旧数据重复计算 PD。 */
u8 gyro_wait_new_sample(uint32_t *last_frame_count, int32_t *yaw)
{
    uint16_t timeout;

    for(timeout = 0; timeout < 200u; timeout++){
        if(Gyro_FrameCount != *last_frame_count){
            if(Gyro_GetYawCentiDegrees(yaw)){
                *last_frame_count = Gyro_FrameCount;
                return 1;
            }
        }
        Delay_ms(10);
    }
    return 0;
}

/* ===== moved from main.c (logic unchanged): gyro_turn_settle ===== */
/* 等车体停稳：间隔 100 ms 的两次角度差 < 0.5° 就算停稳，最多等 0.5 s。
 * 电机的 0x3A"到位"只代表脉冲发完，车体还会继续晃，不等稳就测会反复过冲。 */
u8 gyro_turn_settle(int32_t *yaw)
{
    int32_t  y0, y1, d;
    uint32_t frame;
    u8       i;

    if(!Gyro_GetYawCentiDegrees(&y0)) return 0u;

    for(i = 0u; i < TURN_SETTLE_TRIES; i++){
        Delay_ms(TURN_SETTLE_MS);
        frame = Gyro_FrameCount;
        if(!gyro_wait_new_sample(&frame, &y1)) return 0u;

        d = (y1 > y0) ? (y1 - y0) : (y0 - y1);
        if(d <= TURN_SETTLE_TOL){ *yaw = y1; return 1u; }
        y0 = y1;
    }

    *yaw = y1;                       /* 没完全停也返回，交给后面的补差处理 */
    return 1u;
}

/* ===== moved from main.c (logic unchanged): yaw_zero_capture ===== */
/* 记开机零点：之后所有朝向都用相对它的绝对角度。最多等 1 s 等陀螺仪出数。 */
u8 yaw_zero_capture(void)
{
    int32_t z;
    u16     i;

    for(i = 0u; i < 100u; i++){
        if(Gyro_GetYawCentiDegrees(&z)){
            yaw_zero    = z;
            yaw_zero_ok = 1u;
            gyro_ok     = 1u;
            return 1u;
        }
        Delay_ms(10);
    }
    yaw_zero    = 0L;
    yaw_zero_ok = 0u;
    return 0u;
}

/* ===== moved from main.c (logic unchanged): gyro_turn_to_abs ===== */
/* 转到绝对角度 target_abs（0.01°，相对开机零点），返回 1 到位 / 0 失败。
 * 每次补差都按"当前误差 → 脉冲"一次到位，过程量写进 turn_dbg_*。 */
u8 gyro_turn_to_abs(int32_t target_abs)
{
    int32_t yaw = 0L, err;
    u8      corr = 0u;

    turn_dbg_target = target_abs;
    turn_dbg_meas   = 0L;
    turn_dbg_err    = 0L;
    turn_dbg_corr   = 0u;
    turn_dbg_result = 0u;

    if(!Gyro_GetYawCentiDegrees(&yaw)) return 0u;   /* 没有陀螺仪没法闭环 */

    while(corr < TURN_MAX_CORR){
        turn_dbg_meas = gyro_wrap_180(yaw - yaw_zero);   /* 当前绝对角 */
        err           = gyro_wrap_180(target_abs - yaw); /* 还差多少 */
        turn_dbg_err  = err;

        /* 第 2 行右侧显示当前绝对角（judge 只用 1~5 列） */
        OLED_ShowString(2, 7, "Y");
        OLED_ShowSignedNum(2, 8, turn_dbg_meas / 100L, 3);

        if(err <= TURN_TOL_CENTI && err >= -TURN_TOL_CENTI){
            turn_dbg_result = 1u;
            return 1u;
        }

        /* ★ 按当前误差直接换脉冲：误差多少度就转多少度，不再一点点逼近。
         *   err > 0 需要逆时针(0x01)，否则顺时针(0x00)。 */
        (void)motor_rotate_centideg((err > 0L) ? 0x01u : 0x00u, err);

        if(!wait_motors()) return 0u;
        if(!gyro_turn_settle(&yaw)) return 0u;           /* 等停稳再测下一轮 */

        corr++;
        turn_dbg_corr = corr;
    }

    /* 次数用尽：残留 ≤1° 也算到位，免得为一两度打断整条路线 */
    turn_dbg_meas   = gyro_wrap_180(yaw - yaw_zero);
    turn_dbg_err    = gyro_wrap_180(target_abs - yaw);
    turn_dbg_result = (turn_dbg_err <= 100L && turn_dbg_err >= -100L) ? 1u : 0u;
    return turn_dbg_result;
}
