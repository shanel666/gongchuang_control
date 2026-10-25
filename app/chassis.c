#include "chassis.h"
#include "chassis_config.h"
#include "motor.h"
#include "Serial.h"
#include "Delay.h"

/* ============================================================================
 * chassis.c —— 速度模式底盘（F6）实现
 * ----------------------------------------------------------------------------
 * 这一层的职责只有三件事：
 *   1. 组帧：按 chassis_config.h 选的帧格式，把「方向 + 加速度 + 速度 + 同步」
 *      拼成 8 或 9 字节的 F6 速度帧，套用 motor.h 的单轮极性；
 *   2. 动作表：前进/后退/左移/右移/逆时针/顺时针，四个轮子的方向位各一行，
 *      整轴取反开关也在这里套；
 *   3. 定时运动：速度模式没有位置回包，按「梯形速度曲线」算出该跑多久，
 *      到点发 0 速度停车，再等车体停稳才返回。
 *
 * 速度模式 vs 位置模式：
 *   F6 只告诉驱动器"转多快"，不告诉"转多少"。要走固定距离，本模块用
 *   时间 = 距离 / 速度 来近似，加减速段按 加速度 RPM/s 补偿。实际距离会受
 *   驱动器内部加减速、通信延迟影响，用 CHASSIS_TIME_GAIN_PCT 现场校正。
 *
 * 为什么不用位置模式（FD）：
 *   FD 精度更高，但命令是"发脉冲 + 轮询 0x3A 到位"，运动是分段、顿挫的；
 *   F6 可以带加速度斜坡连续跑，适合需要平顺起步/停车的场合。两套都保留，
 *   走精确路线用 motor.c，走平顺/连续运动用本模块。
 * ========================================================================== */

#define CHASSIS_WHEEL_COUNT   4u
#define CHASSIS_FRAME_MAX_LEN 9u

/* ---------------------------------------------------------------------------
 * 帧格式表
 * ---------------------------------------------------------------------------
 * 字段位置集中在这一个结构里，以后驱动器换协议（多一字节、换个校验）只改
 * 下面两行中的一行，组帧/发送代码完全不用动。
 *
 * 9 字节：地址 F6 方向 加速度H 加速度L 速度H 速度L 同步 6B
 *         偏移    0  1   2     3      4      5      6     7   8
 * 8 字节：地址 F6 方向 速度H 速度L 加速度 同步 6B
 *         偏移    0  1   2     3      4     5    6   7
 * ------------------------------------------------------------------------- */
typedef struct {
    uint8_t len;          /* 整帧长度 */
    uint8_t ofs_addr;     /* 地址 */
    uint8_t ofs_func;     /* 功能码 0xF6 */
    uint8_t ofs_dir;      /* 方向 0x00/0x01 */
    uint8_t ofs_accel;    /* 加速度字段偏移 */
    uint8_t accel_bytes;  /* 加速度字段字节数 1/2 */
    uint8_t ofs_speed;    /* 速度字段偏移 */
    uint8_t speed_bytes;  /* 速度字段字节数 1/2 */
    uint8_t ofs_sync;     /* 同步标志 */
    uint8_t ofs_tail;     /* 校验 0x6B */
} chassis_frame_def_t;

#if CHASSIS_FRAME_LAYOUT == CHASSIS_FRAME_LAYOUT_9B
static const chassis_frame_def_t k_frame = {
    9u, 0u, 1u, 2u, 3u, 2u, 5u, 2u, 7u, 8u
};
#else
static const chassis_frame_def_t k_frame = {
    8u, 0u, 1u, 2u, 5u, 1u, 3u, 2u, 6u, 7u
};
#endif

/* ---------------------------------------------------------------------------
 * 动作表
 * ---------------------------------------------------------------------------
 * dir[] 顺序固定 FL, FR, RL, RR，数值来自 chassis_config.h 的 CHASSIS_DIR_*；
 * flip 是整轴取反（motor.h 的 MOTOR_FLIP_FWD / MOTOR_FLIP_LAT / MOTOR_YAW_SWAP）；
 * is_rotate=0 表示参数是 mm，1 表示参数是度。
 * ------------------------------------------------------------------------- */
typedef struct {
    uint8_t dir[CHASSIS_WHEEL_COUNT];
    uint8_t flip;
    uint8_t is_rotate;
} chassis_action_t;

static const chassis_action_t k_actions[CHASSIS_ACTION_COUNT] = {
    { CHASSIS_DIR_FORWARD,  MOTOR_FLIP_FWD, 0u },   /* CHASSIS_ACTION_FORWARD */
    { CHASSIS_DIR_BACKWARD, MOTOR_FLIP_FWD, 0u },   /* CHASSIS_ACTION_BACKWARD */
    { CHASSIS_DIR_LEFT,     MOTOR_FLIP_LAT, 0u },   /* CHASSIS_ACTION_LEFT */
    { CHASSIS_DIR_RIGHT,    MOTOR_FLIP_LAT, 0u },   /* CHASSIS_ACTION_RIGHT */
    { CHASSIS_DIR_CCW,      MOTOR_YAW_SWAP, 1u },   /* CHASSIS_ACTION_CCW */
    { CHASSIS_DIR_CW,       MOTOR_YAW_SWAP, 1u }    /* CHASSIS_ACTION_CW */
};

/* 四个轮子对应的驱动器地址：顺序和方向表一致（FL, FR, RL, RR） */
static const uint8_t k_wheel_addr[CHASSIS_WHEEL_COUNT] = {
    MOTOR_ADDR_FL, MOTOR_ADDR_FR, MOTOR_ADDR_RL, MOTOR_ADDR_RR
};

static uint16_t   g_accel_rpm_s = CHASSIS_ACCEL_DEFAULT;
static ChassisAbortFn g_abort_fn = 0;

/* ===========================================================================
 * 一、组帧 / 发送
 * ======================================================================== */

/* 按字节数写大端整数（1 或 2 字节） */
static void chassis_put_be(uint8_t *p, uint8_t bytes, uint16_t value)
{
    if (bytes >= 2u) {
        p[0] = (uint8_t)(value >> 8);
        p[1] = (uint8_t)(value & 0xFFu);
    } else {
        p[0] = (uint8_t)(value & 0xFFu);
    }
}

/* 当前帧格式的加速度上限：8 字节帧只有 1 字节，最大 255 */
static uint16_t chassis_limit_accel(uint16_t accel_rpm_s)
{
    if (k_frame.accel_bytes == 1u && accel_rpm_s > 0xFFu) return 0xFFu;
    return accel_rpm_s;
}

/* 组一帧 F6 速度帧（不发送） */
static void chassis_frame_build(uint8_t addr, uint8_t dir, uint16_t accel_rpm_s,
                                uint16_t speed_x10, uint8_t sync, uint8_t *buf)
{
    uint8_t  i;
    uint16_t accel = chassis_limit_accel(accel_rpm_s);
    uint16_t speed = (speed_x10 > CHASSIS_SPEED_MAX) ? CHASSIS_SPEED_MAX : speed_x10;

    for (i = 0u; i < k_frame.len; i++) buf[i] = 0x00u;

    buf[k_frame.ofs_addr] = addr;
    buf[k_frame.ofs_func] = CHASSIS_FUNC_SPEED;
    buf[k_frame.ofs_dir]  = (uint8_t)(dir ? 0x01u : 0x00u);
    chassis_put_be(&buf[k_frame.ofs_accel], k_frame.accel_bytes, accel);
    chassis_put_be(&buf[k_frame.ofs_speed], k_frame.speed_bytes, speed);
    buf[k_frame.ofs_sync] = (uint8_t)(sync ? 0x01u : 0x00u);
    buf[k_frame.ofs_tail] = CHASSIS_FRAME_TAIL;
}

/* 组帧并发送给一个地址（不做逻辑方向 → 方向位的换算，调用方已算好） */
static void chassis_send_frame(uint8_t addr, uint8_t dir, uint16_t accel_rpm_s,
                               uint16_t speed_x10, uint8_t sync)
{
    uint8_t buf[CHASSIS_FRAME_MAX_LEN];

    chassis_frame_build(addr, dir, accel_rpm_s, speed_x10, sync, buf);
    Serial_SendArray(buf, k_frame.len);
    Delay_ms(CHASSIS_FRAME_GAP_MS);
}

/* 单轮极性：和 motor.c 的 motor_dir_bit() 同一套规则，按"地址号"取反，
 * 所以换电机后只需要改 motor.h 的 MOTOR_DIR_INV_1..4。 */
static uint8_t chassis_wheel_invert(uint8_t addr)
{
    switch (addr) {
        case 0x01u: return (uint8_t)(MOTOR_DIR_INV_1 ? 1u : 0u);
        case 0x02u: return (uint8_t)(MOTOR_DIR_INV_2 ? 1u : 0u);
        case 0x03u: return (uint8_t)(MOTOR_DIR_INV_3 ? 1u : 0u);
        case 0x04u: return (uint8_t)(MOTOR_DIR_INV_4 ? 1u : 0u);
        default:    return 0u;
    }
}

/* 逻辑方向 → 实际方向位（套单轮极性） */
static uint8_t chassis_dir_bit(uint8_t addr, uint8_t dir)
{
    return (uint8_t)(dir ^ chassis_wheel_invert(addr));
}

/* ===========================================================================
 * 二、使能 / 停车 / 底层接口
 * ======================================================================== */

void Chassis_Enable(uint8_t enable)
{
    uint8_t cmd[6];
    uint8_t i;

    for (i = 0u; i < CHASSIS_WHEEL_COUNT; i++) {
        cmd[0] = k_wheel_addr[i];
        cmd[1] = CHASSIS_FUNC_ENABLE;
        cmd[2] = 0xABu;
        cmd[3] = (uint8_t)(enable ? 0x01u : 0x00u);
        cmd[4] = 0x00u;
        cmd[5] = CHASSIS_FRAME_TAIL;
        Serial_SendArray(cmd, 6u);
        Delay_ms(CHASSIS_FRAME_GAP_MS);
    }
}

void Chassis_Stop(void)
{
    uint8_t i;

    for (i = 0u; i < CHASSIS_WHEEL_COUNT; i++) {
        /* 速度为 0 时方向位无意义，统一用 0 再套极性。
         * 开了同步的时候四轮一起减速，停车不会歪。 */
        chassis_send_frame(k_wheel_addr[i],
                           chassis_dir_bit(k_wheel_addr[i], 0u),
                           g_accel_rpm_s, 0u,
                           (uint8_t)(CHASSIS_USE_SYNC ? 1u : 0u));
    }
    if (CHASSIS_USE_SYNC) Chassis_SendSync();
}

void Chassis_SendSync(void)
{
    Serial_SendByte(0x00u);
    Serial_SendByte(0xFFu);
    Serial_SendByte(0x66u);
    Serial_SendByte(0x6Bu);
    Delay_ms(CHASSIS_SYNC_GAP_MS);
}

void Chassis_SendSpeed(uint8_t addr, uint8_t dir, uint16_t accel_rpm_s,
                       uint16_t speed_x10, uint8_t sync)
{
    /* 逃生口：原样发送，不套方向表/极性，方便对着手册逐字节验证 */
    chassis_send_frame(addr, dir, accel_rpm_s, speed_x10, sync);
}

void Chassis_Init(void)
{
    g_accel_rpm_s = CHASSIS_ACCEL_DEFAULT;
    g_abort_fn    = 0;
    Chassis_Enable(1u);      /* 上电默认失能，不动时先确认这一帧发出去了 */
}

void Chassis_SetAccel(uint16_t accel_rpm_s)
{
    g_accel_rpm_s = accel_rpm_s ? accel_rpm_s : CHASSIS_ACCEL_DEFAULT;
}

void Chassis_SetAbortCheck(ChassisAbortFn fn)
{
    g_abort_fn = fn;
}

uint8_t Chassis_SetWheelSpeed(const int32_t speed_x10[CHASSIS_WHEEL_COUNT])
{
    uint8_t i;

    if (speed_x10 == 0) return 0u;

    for (i = 0u; i < CHASSIS_WHEEL_COUNT; i++) {
        int32_t  v     = speed_x10[i];
        uint8_t  dir   = (uint8_t)((v < 0) ? 1u : 0u);
        uint32_t mag   = (v < 0) ? ((uint32_t)(-(v + 1)) + 1u) : (uint32_t)v;
        uint16_t speed = (mag > CHASSIS_SPEED_MAX) ? (uint16_t)CHASSIS_SPEED_MAX
                                                   : (uint16_t)mag;

        /* 连续控制用立即执行，不挂同步，改速度时四个轮子各自更新 */
        chassis_send_frame(k_wheel_addr[i],
                           chassis_dir_bit(k_wheel_addr[i], dir),
                           g_accel_rpm_s, speed, 0u);
    }

    return 1u;
}

/* ===========================================================================
 * 三、定时运动
 * ======================================================================== */

/* 距离/角度 → 电机需要转的圈数 */
static float chassis_revolutions(uint8_t is_rotate, uint32_t amount)
{
    if (is_rotate) return (float)amount / CHASSIS_DEG_PER_REV;
    return (float)amount / CHASSIS_MM_PER_REV;
}

/* 只用四则运算的平方根（牛顿迭代）。
 * 工程用的是 Keil microlib，不想为一次开方引入 math.h 依赖，所以自己算；
 * 只有"距离太短、还没加到目标速度"的三角形曲线才会走到这里。 */
static float chassis_sqrtf(float x)
{
    float   r;
    uint8_t i;

    if (x <= 0.0f) return 0.0f;

    r = x;                                  /* 初值 */
    for (i = 0u; i < 12u; i++) {            /* 二次收敛，12 次足够 */
        r = 0.5f * (r + x / r);
    }
    return r;
}

/* 估算一次动作的总时间（ms），按梯形/三角形速度曲线：
 *   梯形：匀速段时间 t_flat = 圈数 * 60 / 转速；加减速各占 t_ramp = 转速 / 加速度，
 *         加减速段少走的距离正好等于 v * t_ramp，所以总时间 = t_flat + t_ramp。
 *   三角形：距离太短，还没加到目标速度就要减速，
 *         总时间 = 2 * sqrt(60 * 圈数 / 加速度)。
 * 最后乘 CHASSIS_TIME_GAIN_PCT 并加 CHASSIS_TIME_OFFSET_MS 做现场修正。 */
static uint32_t chassis_estimate_ms(uint8_t is_rotate, uint32_t amount,
                                    uint16_t speed_x10, uint16_t accel_rpm_s)
{
    float revs, rpm, accel, t_flat, t_ramp, t;

    if (amount == 0u || speed_x10 == 0u || accel_rpm_s == 0u) return 0u;

    if (speed_x10 > CHASSIS_SPEED_MAX) speed_x10 = CHASSIS_SPEED_MAX;
    revs  = chassis_revolutions(is_rotate, amount);
    rpm   = (float)speed_x10 / 10.0f;
    accel = (float)chassis_limit_accel(accel_rpm_s);

    t_flat = revs * 60.0f / rpm;
    t_ramp = rpm / accel;

    if (t_flat >= t_ramp) {
        t = t_flat + t_ramp;
    } else {
        t = 2.0f * chassis_sqrtf(60.0f * revs / accel);
    }

    t = t * ((float)CHASSIS_TIME_GAIN_PCT / 100.0f)
          + ((float)CHASSIS_TIME_OFFSET_MS / 1000.0f);
    if (t < 0.0f) t = 0.0f;

    return (uint32_t)(t * 1000.0f + 0.5f);
}

/* 停车后要等多久：至少 CHASSIS_STOP_SETTLE_MS，且要覆盖按加速度减速的时间
 * （速度越高、加速度越小，减速越久；否则下一条命令会在还没停稳时下发）。 */
static uint32_t chassis_stop_ms(uint16_t speed_x10, uint16_t accel_rpm_s)
{
    uint32_t t;

    if (speed_x10 == 0u || accel_rpm_s == 0u) return CHASSIS_STOP_SETTLE_MS;

    /* 减速时间 = (speed_x10/10) / accel 秒 = speed_x10 * 100 / accel 毫秒 */
    t = (uint32_t)((float)speed_x10 * 100.0f / (float)accel_rpm_s + 0.5f) + 50u;
    return (t > CHASSIS_STOP_SETTLE_MS) ? t : CHASSIS_STOP_SETTLE_MS;
}

/* 分片等待：每片 10 ms，顺便查中止回调，避免定时运动按 PA8 停不下来 */
static uint8_t chassis_wait_ms(uint32_t ms)
{
    uint32_t done = 0u;

    while (done < ms) {
        uint32_t step = ms - done;

        if (g_abort_fn != 0 && g_abort_fn() != 0u) return 0u;
        if (step > CHASSIS_WAIT_SLICE_MS) step = CHASSIS_WAIT_SLICE_MS;
        Delay_ms(step);
        done += step;
    }

    return 1u;
}

/* 动作量的安全限幅 */
static uint32_t chassis_clamp_amount(uint8_t is_rotate, uint32_t amount)
{
    if (is_rotate) return (amount > CHASSIS_MAX_DEG) ? CHASSIS_MAX_DEG : amount;
    return (amount > CHASSIS_MAX_MM) ? CHASSIS_MAX_MM : amount;
}

/* 默认速度：平移和旋转各一个 */
static uint16_t chassis_default_speed(uint8_t is_rotate)
{
    return is_rotate ? (uint16_t)CHASSIS_SPEED_ROTATE
                     : (uint16_t)CHASSIS_SPEED_TRANSLATE;
}

uint32_t Chassis_EstimateMs(ChassisAction action, uint32_t amount, uint16_t speed_x10)
{
    const chassis_action_t *a;

    if (action >= CHASSIS_ACTION_COUNT) return 0u;
    a = &k_actions[action];

    amount = chassis_clamp_amount(a->is_rotate, amount);
    if (speed_x10 == 0u) speed_x10 = chassis_default_speed(a->is_rotate);

    return chassis_estimate_ms(a->is_rotate, amount, speed_x10, g_accel_rpm_s);
}

/* 下发一个动作：四个轮子先挂起（sync=1）再由同步释放帧统一起步；
 * 驱动器不支持 F6 同步时把 CHASSIS_USE_SYNC 改成 0，逐轮立即执行。 */
static void chassis_start(const chassis_action_t *a, uint16_t speed_x10,
                          uint16_t accel_rpm_s)
{
    uint8_t i;

    for (i = 0u; i < CHASSIS_WHEEL_COUNT; i++) {
        uint8_t dir = (uint8_t)(a->dir[i] ^ a->flip);
        chassis_send_frame(k_wheel_addr[i],
                           chassis_dir_bit(k_wheel_addr[i], dir),
                           accel_rpm_s, speed_x10,
                           (uint8_t)(CHASSIS_USE_SYNC ? 1u : 0u));
    }
    if (CHASSIS_USE_SYNC) Chassis_SendSync();
}

/* 已启动的动作跑 ms 毫秒后停车并等停稳；被中止返回 0 */
static uint8_t chassis_execute(const chassis_action_t *a, uint32_t ms,
                               uint16_t speed_x10, uint16_t accel_rpm_s)
{
    chassis_start(a, speed_x10, accel_rpm_s);

    if (!chassis_wait_ms(ms)) {       /* 被中止：停车后返回失败 */
        Chassis_Stop();
        return 0u;
    }

    Chassis_Stop();                   /* 0 速度 = 按加速度减速停车 */
    (void)chassis_wait_ms(chassis_stop_ms(speed_x10, accel_rpm_s));

    return 1u;
}

uint8_t Chassis_Run(ChassisAction action, uint32_t amount,
                    uint16_t speed_x10, uint16_t accel_rpm_s)
{
    const chassis_action_t *a;
    uint32_t ms;

    if (action >= CHASSIS_ACTION_COUNT) return 0u;

    a      = &k_actions[action];
    amount = chassis_clamp_amount(a->is_rotate, amount);
    if (amount == 0u) return 1u;

    if (speed_x10 == 0u)    speed_x10    = chassis_default_speed(a->is_rotate);
    if (accel_rpm_s == 0u)  accel_rpm_s  = g_accel_rpm_s;
    accel_rpm_s = chassis_limit_accel(accel_rpm_s);
    g_accel_rpm_s = accel_rpm_s;      /* 停车也用同一个减速度，曲线才对称 */

    ms = chassis_estimate_ms(a->is_rotate, amount, speed_x10, accel_rpm_s);

    return chassis_execute(a, ms, speed_x10, accel_rpm_s);
}

uint8_t Chassis_RunMs(ChassisAction action, uint32_t ms, uint16_t speed_x10)
{
    const chassis_action_t *a;

    if (action >= CHASSIS_ACTION_COUNT) return 0u;
    if (ms == 0u) return 1u;

    a = &k_actions[action];
    if (speed_x10 == 0u) speed_x10 = chassis_default_speed(a->is_rotate);
    if (speed_x10 > CHASSIS_SPEED_MAX) speed_x10 = (uint16_t)CHASSIS_SPEED_MAX;
    if (ms > CHASSIS_MAX_RUN_MS) ms = CHASSIS_MAX_RUN_MS;

    g_accel_rpm_s = chassis_limit_accel(g_accel_rpm_s);

    return chassis_execute(a, ms, speed_x10, g_accel_rpm_s);
}

/* 带符号动作的统一实现：value>0 走 pos 动作，value<0 走 neg 动作 */
static uint8_t chassis_signed_run(ChassisAction pos, ChassisAction neg,
                                  int32_t value, uint16_t speed_x10)
{
    uint32_t amount;

    if (value == 0) return 1u;

    if (value > 0) {
        amount = (uint32_t)value;
        return Chassis_Run(pos, amount, speed_x10, 0u);
    }

    /* 两补码取绝对值：INT32_MIN 也不会溢出 */
    amount = (uint32_t)(-(value + 1)) + 1u;
    return Chassis_Run(neg, amount, speed_x10, 0u);
}

uint8_t Chassis_Move(int32_t mm)
{
    return chassis_signed_run(CHASSIS_ACTION_FORWARD, CHASSIS_ACTION_BACKWARD, mm, 0u);
}

uint8_t Chassis_MoveAt(int32_t mm, uint16_t speed_x10)
{
    return chassis_signed_run(CHASSIS_ACTION_FORWARD, CHASSIS_ACTION_BACKWARD, mm, speed_x10);
}

uint8_t Chassis_Strafe(int32_t mm)
{
    return chassis_signed_run(CHASSIS_ACTION_LEFT, CHASSIS_ACTION_RIGHT, mm, 0u);
}

uint8_t Chassis_StrafeAt(int32_t mm, uint16_t speed_x10)
{
    return chassis_signed_run(CHASSIS_ACTION_LEFT, CHASSIS_ACTION_RIGHT, mm, speed_x10);
}

uint8_t Chassis_Rotate(int32_t deg)
{
    return chassis_signed_run(CHASSIS_ACTION_CCW, CHASSIS_ACTION_CW, deg, 0u);
}

uint8_t Chassis_RotateAt(int32_t deg, uint16_t speed_x10)
{
    return chassis_signed_run(CHASSIS_ACTION_CCW, CHASSIS_ACTION_CW, deg, speed_x10);
}
