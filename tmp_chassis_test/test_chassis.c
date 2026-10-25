/* ============================================================================
 * test_chassis.c —— chassis.c 的 PC 端回归测试（不连硬件）
 * ----------------------------------------------------------------------------
 * 做法：用 tmp_chassis_test/stub 里的 Serial.h / Delay.h 顶掉单片机的串口和
 *       延时，把 chassis.c 发出去的字节全部录下来，逐帧检查：
 *         - 帧格式（8/9 字节两种都测）、功能码、地址、方向位、加速度、速度、
 *           同步位、校验码；
 *         - 方向表 / 单轮极性 / 整轴取反有没有套对；
 *         - 定时运动的阻塞时间是否等于估算时间 + 帧间隔 + 停稳等待；
 *         - 运动学估算（mm/s、°/s、时间）是否合理。
 *
 * 编译（在工程根目录）：
 *   gcc -std=c99 -Wall -Wextra -I tmp_chassis_test/stub -I app \
 *       tmp_chassis_test/test_chassis.c app/chassis.c -lm -o tmp_chassis_test/test_chassis.exe
 * 运行：
 *   tmp_chassis_test/test_chassis.exe
 *
 * 想测另一种帧格式 / 打开同步，加 -D 覆盖：
 *   -DCHASSIS_FRAME_LAYOUT=CHASSIS_FRAME_LAYOUT_9B
 *   -DCHASSIS_USE_SYNC=1
 * ========================================================================== */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "chassis.h"
#include "chassis_config.h"
#include "motor.h"

/* ---------------------------------------------------------------------------
 * 当前帧格式的字段位置（和 chassis.c 的布局表必须一致，这里故意写死成
 * "独立的一份"，这样 chassis.c 改错能测出来）
 * ------------------------------------------------------------------------- */
#if CHASSIS_FRAME_LAYOUT == CHASSIS_FRAME_LAYOUT_9B
#define FRAME_LEN   9u
#define ACC_OFS     3u
#define ACC_LEN     2u
#define SPD_OFS     5u
#define SPD_LEN     2u
#define SYNC_OFS    7u
#else
#define FRAME_LEN   8u
#define ACC_OFS     5u
#define ACC_LEN     1u
#define SPD_OFS     3u
#define SPD_LEN     2u
#define SYNC_OFS    6u
#endif

#define TX_MAX      8192u
#define WHEEL_COUNT 4u

static uint8_t  g_tx[TX_MAX];
static uint32_t g_tx_len;
static uint32_t g_delay_ms;
static int      g_fail;

/* ---- 串口 / 延时测试桩 ---- */
void Serial_SendByte(uint8_t byte)
{
    if (g_tx_len < TX_MAX) g_tx[g_tx_len++] = byte;
}

void Serial_SendArray(uint8_t *array, uint16_t length)
{
    while (length--) Serial_SendByte(*array++);
}

void Delay_ms(uint32_t ms)
{
    g_delay_ms += ms;
}

/* ---- 小工具 ---- */
static const uint8_t k_wheel_addr[WHEEL_COUNT] = {
    MOTOR_ADDR_FL, MOTOR_ADDR_FR, MOTOR_ADDR_RL, MOTOR_ADDR_RR
};

static const uint8_t k_fwd[WHEEL_COUNT]  = CHASSIS_DIR_FORWARD;
static const uint8_t k_back[WHEEL_COUNT] = CHASSIS_DIR_BACKWARD;
static const uint8_t k_left[WHEEL_COUNT] = CHASSIS_DIR_LEFT;
static const uint8_t k_right[WHEEL_COUNT] = CHASSIS_DIR_RIGHT;
static const uint8_t k_ccw[WHEEL_COUNT]  = CHASSIS_DIR_CCW;
static const uint8_t k_cw[WHEEL_COUNT]   = CHASSIS_DIR_CW;

static void reset_capture(void)
{
    g_tx_len   = 0u;
    g_delay_ms = 0u;
}

static void check(int condition, const char *what)
{
    if (condition) {
        printf("  ok   %s\n", what);
    } else {
        printf("  FAIL %s\n", what);
        g_fail = 1;
    }
}

/* 浮点估算差 1 ms 属于正常舍入，比较时给 ±1 ms 容差 */
static void check_ms(uint32_t got, uint32_t expect, const char *what)
{
    int32_t diff = (int32_t)got - (int32_t)expect;
    char msg[160];

    if (diff < 0) diff = -diff;
    snprintf(msg, sizeof(msg), "%s（got %u ms, expect %u ms）",
             what, (unsigned)got, (unsigned)expect);
    check(diff <= 1, msg);
}

static uint8_t wheel_invert(uint8_t addr)
{
    switch (addr) {
        case 0x01u: return (uint8_t)(MOTOR_DIR_INV_1 ? 1u : 0u);
        case 0x02u: return (uint8_t)(MOTOR_DIR_INV_2 ? 1u : 0u);
        case 0x03u: return (uint8_t)(MOTOR_DIR_INV_3 ? 1u : 0u);
        case 0x04u: return (uint8_t)(MOTOR_DIR_INV_4 ? 1u : 0u);
        default:    return 0u;
    }
}

static uint8_t expected_dir(uint8_t base, uint8_t flip, uint8_t addr)
{
    return (uint8_t)(base ^ flip ^ wheel_invert(addr));
}

static uint16_t read_be(const uint8_t *p, uint8_t bytes)
{
    if (bytes >= 2u) return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
    return (uint16_t)p[0];
}

static void print_hex(const uint8_t *p, uint32_t len)
{
    uint32_t i;
    for (i = 0u; i < len; i++) printf("%02X ", p[i]);
    printf("\n");
}

/* 检查从字节偏移 offset 开始的一帧：地址、功能码、方向、加速度、速度、同步、校验 */
static void check_frame_at(uint32_t offset, uint8_t addr, uint8_t dir,
                           uint16_t accel, uint16_t speed, uint8_t sync,
                           const char *tag)
{
    const uint8_t *f = &g_tx[offset];
    char msg[128];
    uint16_t got_accel = read_be(&f[ACC_OFS], ACC_LEN);
    uint16_t got_speed = read_be(&f[SPD_OFS], SPD_LEN);
    uint16_t exp_accel = (ACC_LEN == 1u && accel > 0xFFu) ? 0xFFu : accel;
    uint16_t exp_speed = (speed > CHASSIS_SPEED_MAX) ? (uint16_t)CHASSIS_SPEED_MAX : speed;

    snprintf(msg, sizeof(msg),
             "%s: [%02X][F6][%02X][acc=%u][spd=%u][sync=%u][6B]",
             tag, addr, dir, (unsigned)exp_accel, (unsigned)exp_speed,
             (unsigned)sync);

    check(f[0] == addr && f[1] == CHASSIS_FUNC_SPEED && f[2] == dir &&
          got_accel == exp_accel && got_speed == exp_speed &&
          f[SYNC_OFS] == sync && f[FRAME_LEN - 1u] == CHASSIS_FRAME_TAIL,
          msg);
}

/* 按帧下标检查（第 index 帧） */
static void check_frame(uint32_t index, uint8_t addr, uint8_t dir,
                        uint16_t accel, uint16_t speed, uint8_t sync,
                        const char *tag)
{
    check_frame_at(index * FRAME_LEN, addr, dir, accel, speed, sync, tag);
}

/* 估算时间的独立实现（用 double 再算一遍，拿来和 chassis.c 对比） */
static uint32_t expect_ms(int is_rotate, uint32_t amount,
                          uint16_t speed_x10, uint16_t accel_rpm_s)
{
    double revs, rpm, accel, t_flat, t_ramp, t;

    if (amount == 0u || speed_x10 == 0u) return 0u;
    if (accel_rpm_s == 0u) accel_rpm_s = CHASSIS_ACCEL_DEFAULT;
    if (ACC_LEN == 1u && accel_rpm_s > 0xFFu) accel_rpm_s = 0xFFu;

    revs = is_rotate
         ? (double)amount / ((double)CHASSIS_PULSES_PER_REV * 360.0 / (double)MOTOR_PULSE_PER_360DEG)
         : (double)amount / ((double)CHASSIS_PULSES_PER_REV * 1000.0 / (double)MOTOR_PULSE_PER_100CM);
    rpm   = (double)speed_x10 / 10.0;
    accel = (double)accel_rpm_s;

    t_flat = revs * 60.0 / rpm;
    t_ramp = rpm / accel;
    if (t_flat >= t_ramp) t = t_flat + t_ramp;
    else                   t = 2.0 * sqrt(60.0 * revs / accel);

    t = t * ((double)CHASSIS_TIME_GAIN_PCT / 100.0)
          + ((double)CHASSIS_TIME_OFFSET_MS / 1000.0);
    return (uint32_t)(t * 1000.0 + 0.5);
}

/* 停车后等待时间（和 chassis.c 的 chassis_stop_ms 对应） */
static uint32_t expect_stop_ms(uint16_t speed_x10, uint16_t accel_rpm_s)
{
    uint32_t t = (uint32_t)((double)speed_x10 * 100.0 / (double)accel_rpm_s + 0.5) + 50u;
    return (t > CHASSIS_STOP_SETTLE_MS) ? t : CHASSIS_STOP_SETTLE_MS;
}

/* ---------------------------------------------------------------------------
 * 用例 1：示例帧逐字节
 * ------------------------------------------------------------------------- */
static void test_sample_frame(void)
{
#if CHASSIS_FRAME_LAYOUT == CHASSIS_FRAME_LAYOUT_9B
    static const uint8_t expect_9b[] = {0x01, 0xF6, 0x00, 0x00, 0x32, 0x00, 0x32, 0x01, 0x6B};
#else
    static const uint8_t expect_8b[] = {0x01, 0xF6, 0x00, 0x00, 0x32, 0x32, 0x01, 0x6B};
#endif

    printf("\n[1] 示例帧：地址 01，方向 0，加速度 50，速度 50(0.1RPM=5.0RPM)，同步 1\n");

    reset_capture();
    Chassis_SendSpeed(0x01u, 0x00u, 50u, 50u, 0x01u);
    printf("  TX: ");
    print_hex(g_tx, g_tx_len);

#if CHASSIS_FRAME_LAYOUT == CHASSIS_FRAME_LAYOUT_9B
    check(g_tx_len == sizeof(expect_9b) &&
          memcmp(g_tx, expect_9b, sizeof(expect_9b)) == 0,
          "9 字节帧 = 01 F6 00 00 32 00 32 01 6B");
#else
    check(g_tx_len == sizeof(expect_8b) &&
          memcmp(g_tx, expect_8b, sizeof(expect_8b)) == 0,
          "8 字节帧 = 01 F6 00 00 32 32 01 6B（与你给的示例一致）");
#endif
}

/* ---------------------------------------------------------------------------
 * 用例 2：前进 1000 mm
 * ------------------------------------------------------------------------- */
static void test_move_forward(void)
{
    uint32_t est, expect_total, i;
    uint32_t sync_bytes = (CHASSIS_USE_SYNC ? 4u : 0u);
    uint8_t  ok;

    printf("\n[2] Chassis_Move(1000)（前进，默认速度）\n");

    est = Chassis_EstimateMs(CHASSIS_ACTION_FORWARD, 1000u, 0u);
    check_ms(est, expect_ms(0, 1000u, CHASSIS_SPEED_TRANSLATE, CHASSIS_ACCEL_DEFAULT),
             "EstimateMs 与独立计算一致");

    reset_capture();
    ok = Chassis_Move(1000);

    printf("  TX %u 字节，Delay 累计 %u ms，Estimate %u ms\n",
           (unsigned)g_tx_len, (unsigned)g_delay_ms, (unsigned)est);

    check(ok == 1u, "返回值 1");
    check(g_tx_len == 4u * FRAME_LEN + sync_bytes + 4u * FRAME_LEN + sync_bytes,
          "共 4 帧启动 + 同步帧 + 4 帧停车 + 同步帧");

    for (i = 0u; i < 4u; i++) {
        char tag[32];
        snprintf(tag, sizeof(tag), "启动轮 %u", (unsigned)i);
        check_frame(i,
                    k_wheel_addr[i],
                    expected_dir(k_fwd[i], MOTOR_FLIP_FWD, k_wheel_addr[i]),
                    CHASSIS_ACCEL_DEFAULT, CHASSIS_SPEED_TRANSLATE,
                    (uint8_t)(CHASSIS_USE_SYNC ? 1u : 0u), tag);
    }

    if (CHASSIS_USE_SYNC) {
        check(g_tx[4u * FRAME_LEN + 0u] == 0x00u &&
              g_tx[4u * FRAME_LEN + 1u] == 0xFFu &&
              g_tx[4u * FRAME_LEN + 2u] == 0x66u &&
              g_tx[4u * FRAME_LEN + 3u] == 0x6Bu,
              "同步释放帧 00 FF 66 6B");
    }

    for (i = 0u; i < 4u; i++) {
        char tag[32];
        snprintf(tag, sizeof(tag), "停车轮 %u", (unsigned)i);
        /* 同步释放帧是 4 个裸字节，所以停车帧按字节偏移找，不能按帧下标找 */
        check_frame_at(4u * FRAME_LEN + sync_bytes + i * FRAME_LEN,
                       k_wheel_addr[i],
                       expected_dir(0u, 0u, k_wheel_addr[i]),
                       CHASSIS_ACCEL_DEFAULT, 0u,
                       (uint8_t)(CHASSIS_USE_SYNC ? 1u : 0u), tag);
    }

    if (CHASSIS_USE_SYNC) {
        uint32_t stop_sync = 4u * FRAME_LEN + sync_bytes + 4u * FRAME_LEN;
        check(g_tx[stop_sync + 0u] == 0x00u && g_tx[stop_sync + 1u] == 0xFFu &&
              g_tx[stop_sync + 2u] == 0x66u && g_tx[stop_sync + 3u] == 0x6Bu,
              "停车同步释放帧 00 FF 66 6B");
    }

    expect_total = est + 8u * CHASSIS_FRAME_GAP_MS +
                  (CHASSIS_USE_SYNC ? 2u * CHASSIS_SYNC_GAP_MS : 0u) +
                  expect_stop_ms(CHASSIS_SPEED_TRANSLATE, CHASSIS_ACCEL_DEFAULT);
    check_ms(g_delay_ms, expect_total, "阻塞时间 = 运动时间 + 帧间隔 + 停稳等待");

    /* 后退 100 mm：套 BACKWARD 表 */
    reset_capture();
    ok = Chassis_Move(-100);
    check(ok == 1u, "后退 100 返回 1");
    for (i = 0u; i < 4u; i++) {
        char tag[32];
        snprintf(tag, sizeof(tag), "后退轮 %u", (unsigned)i);
        check_frame(i,
                    k_wheel_addr[i],
                    expected_dir(k_back[i], MOTOR_FLIP_FWD, k_wheel_addr[i]),
                    CHASSIS_ACCEL_DEFAULT, CHASSIS_SPEED_TRANSLATE,
                    (uint8_t)(CHASSIS_USE_SYNC ? 1u : 0u), tag);
    }
}

/* ---------------------------------------------------------------------------
 * 用例 3：右平移 250 mm（走 BACKWARD 之外的另一张方向表，顺便验证符号）
 * ------------------------------------------------------------------------- */
static void test_strafe_right(void)
{
    uint32_t i;
    uint8_t ok;

    printf("\n[3] Chassis_Strafe(-250)（右平移）\n");
    reset_capture();
    ok = Chassis_Move(0);                  /* 0 距离：不应发任何帧 */
    check(ok == 1u && g_tx_len == 0u, "距离 0 直接返回，不发帧");

    reset_capture();
    ok = Chassis_Strafe(-250);

    check(ok == 1u, "返回值 1");
    check(g_tx_len == 8u * FRAME_LEN + (CHASSIS_USE_SYNC ? 8u : 0u),
          "4 帧启动 + 4 帧停车（开同步时各加一个 4 字节同步帧）");

    for (i = 0u; i < 4u; i++) {
        char tag[32];
        snprintf(tag, sizeof(tag), "右移轮 %u", (unsigned)i);
        check_frame(i,
                    k_wheel_addr[i],
                    expected_dir(k_right[i], MOTOR_FLIP_LAT, k_wheel_addr[i]),
                    CHASSIS_ACCEL_DEFAULT, CHASSIS_SPEED_TRANSLATE,
                    (uint8_t)(CHASSIS_USE_SYNC ? 1u : 0u), tag);
    }

    check_ms(Chassis_EstimateMs(CHASSIS_ACTION_RIGHT, 250u, 0u),
             expect_ms(0, 250u, CHASSIS_SPEED_TRANSLATE, CHASSIS_ACCEL_DEFAULT),
             "EstimateMs(右移 250) 与独立计算一致");

    /* 左移 100 mm：套 LEFT 表 */
    reset_capture();
    ok = Chassis_Strafe(100);
    check(ok == 1u, "左移返回 1");
    for (i = 0u; i < 4u; i++) {
        char tag[32];
        snprintf(tag, sizeof(tag), "左移轮 %u", (unsigned)i);
        check_frame(i,
                    k_wheel_addr[i],
                    expected_dir(k_left[i], MOTOR_FLIP_LAT, k_wheel_addr[i]),
                    CHASSIS_ACCEL_DEFAULT, CHASSIS_SPEED_TRANSLATE,
                    (uint8_t)(CHASSIS_USE_SYNC ? 1u : 0u), tag);
    }
}

/* ---------------------------------------------------------------------------
 * 用例 4：旋转 90°（逆时针 / 顺时针各一次，验证旋转方向表）
 * ------------------------------------------------------------------------- */
static void test_rotate(void)
{
    uint32_t i;
    uint8_t ok;

    printf("\n[4] Chassis_Rotate(+90) / Rotate(-90)\n");

    reset_capture();
    ok = Chassis_Rotate(90);
    check(ok == 1u, "逆时针返回 1");
    for (i = 0u; i < 4u; i++) {
        char tag[32];
        snprintf(tag, sizeof(tag), "逆时针轮 %u", (unsigned)i);
        check_frame(i,
                    k_wheel_addr[i],
                    expected_dir(k_ccw[i], MOTOR_YAW_SWAP, k_wheel_addr[i]),
                    CHASSIS_ACCEL_DEFAULT, CHASSIS_SPEED_ROTATE,
                    (uint8_t)(CHASSIS_USE_SYNC ? 1u : 0u), tag);
    }
    check_ms(Chassis_EstimateMs(CHASSIS_ACTION_CCW, 90u, 0u),
             expect_ms(1, 90u, CHASSIS_SPEED_ROTATE, CHASSIS_ACCEL_DEFAULT),
             "EstimateMs(逆时针 90°) 与独立计算一致");

    reset_capture();
    ok = Chassis_Rotate(-90);
    check(ok == 1u, "顺时针返回 1");
    for (i = 0u; i < 4u; i++) {
        char tag[32];
        snprintf(tag, sizeof(tag), "顺时针轮 %u", (unsigned)i);
        check_frame(i,
                    k_wheel_addr[i],
                    expected_dir(k_cw[i], MOTOR_YAW_SWAP, k_wheel_addr[i]),
                    CHASSIS_ACCEL_DEFAULT, CHASSIS_SPEED_ROTATE,
                    (uint8_t)(CHASSIS_USE_SYNC ? 1u : 0u), tag);
    }
}

/* ---------------------------------------------------------------------------
 * 用例 5：速度覆盖 + 单轮速度直发（含负速度 / 超限限幅）
 * ------------------------------------------------------------------------- */
static void test_speed_override(void)
{
    static const int32_t wheel_speed[4] = { 300, -300, 1234, -40000 };
    uint32_t i;
    uint8_t ok;

    printf("\n[5] Chassis_MoveAt(500, 150) 与 Chassis_SetWheelSpeed\n");

    reset_capture();
    ok = Chassis_MoveAt(500, 150);
    check(ok == 1u && g_tx_len == 8u * FRAME_LEN + (CHASSIS_USE_SYNC ? 8u : 0u),
          "MoveAt(500,150) 帧数正常");
    check_frame(0u, k_wheel_addr[0],
                expected_dir(k_fwd[0], MOTOR_FLIP_FWD, k_wheel_addr[0]),
                CHASSIS_ACCEL_DEFAULT, 150u,
                (uint8_t)(CHASSIS_USE_SYNC ? 1u : 0u), "MoveAt 速度覆盖");

    /* 高速时减速更久，停稳等待要跟着变大（700 = 70.0 RPM） */
    {
        uint32_t est = Chassis_EstimateMs(CHASSIS_ACTION_FORWARD, 500u, 700u);
        uint32_t total;

        reset_capture();
        ok = Chassis_MoveAt(500, 700);
        total = est + 8u * CHASSIS_FRAME_GAP_MS +
                (CHASSIS_USE_SYNC ? 2u * CHASSIS_SYNC_GAP_MS : 0u) +
                expect_stop_ms(700u, CHASSIS_ACCEL_DEFAULT);
        check(ok == 1u, "MoveAt(500,700) 返回 1");
        check_ms(g_delay_ms, total, "高速停车等待 = 减速时间（> 默认 300 ms）");
    }

    reset_capture();
    check(Chassis_SetWheelSpeed(wheel_speed) == 1u, "SetWheelSpeed 返回 1");
    for (i = 0u; i < 4u; i++) {
        int32_t v = wheel_speed[i];
        uint8_t dir = (uint8_t)((v < 0) ? 1u : 0u);
        uint32_t mag = (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
        uint16_t exp = (mag > CHASSIS_SPEED_MAX) ? (uint16_t)CHASSIS_SPEED_MAX : (uint16_t)mag;
        char tag[32];
        snprintf(tag, sizeof(tag), "直发轮 %u speed=%d", (unsigned)i, (int)v);
        check_frame(i,
                    k_wheel_addr[i],
                    expected_dir(dir, 0u, k_wheel_addr[i]),
                    CHASSIS_ACCEL_DEFAULT, exp, 0u, tag);
    }
}

/* ---------------------------------------------------------------------------
 * 用例 6：按时间跑（上位机 "前进 1 s" 用的接口）
 * ------------------------------------------------------------------------- */
static void test_run_ms(void)
{
    uint32_t sync_bytes = (CHASSIS_USE_SYNC ? 4u : 0u);
    uint32_t i;
    uint8_t  ok;

    printf("\n[6] Chassis_RunMs(FORWARD, 1000, 0) —— 按时间跑 1 秒\n");

    reset_capture();
    ok = Chassis_RunMs(CHASSIS_ACTION_FORWARD, 1000u, 0u);

    check(ok == 1u, "返回 1");
    check(g_tx_len == 4u * FRAME_LEN + sync_bytes + 4u * FRAME_LEN + sync_bytes,
          "4 帧启动 + 4 帧停车");
    for (i = 0u; i < 4u; i++) {
        char tag[32];
        snprintf(tag, sizeof(tag), "定时轮 %u", (unsigned)i);
        check_frame(i,
                    k_wheel_addr[i],
                    expected_dir(k_fwd[i], MOTOR_FLIP_FWD, k_wheel_addr[i]),
                    CHASSIS_ACCEL_DEFAULT, CHASSIS_SPEED_TRANSLATE,
                    (uint8_t)(CHASSIS_USE_SYNC ? 1u : 0u), tag);
    }
    check_ms(g_delay_ms,
             1000u + 8u * CHASSIS_FRAME_GAP_MS +
             (CHASSIS_USE_SYNC ? 2u * CHASSIS_SYNC_GAP_MS : 0u) +
             expect_stop_ms(CHASSIS_SPEED_TRANSLATE, CHASSIS_ACCEL_DEFAULT),
             "阻塞时间 = 1000 ms + 帧间隔 + 停车等待");

    reset_capture();
    check(Chassis_RunMs(CHASSIS_ACTION_FORWARD, 0u, 0u) == 1u && g_tx_len == 0u,
          "ms=0 直接返回，不发帧");

    check(Chassis_RunMs(CHASSIS_ACTION_COUNT, 1000u, 0u) == 0u,
          "非法动作返回 0");
}

/* ---------------------------------------------------------------------------
 * 用例 7：把估算表打印出来，方便换算现场参数
 * ------------------------------------------------------------------------- */
static void test_estimate_table(void)
{
    static const struct {
        ChassisAction action;
        uint32_t      amount;
        uint16_t      speed;
        const char   *name;
    } cases[] = {
        { CHASSIS_ACTION_FORWARD,  1000u, 0u,   "前进 1000mm 默认速度" },
        { CHASSIS_ACTION_FORWARD,   100u, 0u,   "前进 100mm 默认速度"  },
        { CHASSIS_ACTION_FORWARD,    50u, 0u,   "前进 50mm 默认速度"   },
        { CHASSIS_ACTION_FORWARD,    10u, 0u,   "前进 10mm 默认速度"   },
        { CHASSIS_ACTION_LEFT,      250u, 0u,   "左移 250mm 默认速度"  },
        { CHASSIS_ACTION_CCW,        90u, 0u,   "逆时针 90° 默认速度"  },
        { CHASSIS_ACTION_CCW,        90u, 700u, "逆时针 90° 70.0RPM"  },
        { CHASSIS_ACTION_FORWARD,   500u, 150u, "前进 500mm 15.0RPM"  }
    };
    uint32_t i;

    printf("\n[7] 时间估算表（1 圈 = %.3f mm = %.3f°，默认平移 %.1f mm/s，旋转 %.2f °/s）\n",
           (double)CHASSIS_MM_PER_REV, (double)CHASSIS_DEG_PER_REV,
           (double)(CHASSIS_SPEED_TRANSLATE / 10.0f) / 60.0f * (double)CHASSIS_MM_PER_REV,
           (double)(CHASSIS_SPEED_ROTATE / 10.0f) / 60.0f * (double)CHASSIS_DEG_PER_REV);

    for (i = 0u; i < sizeof(cases) / sizeof(cases[0]); i++) {
        uint32_t got = Chassis_EstimateMs(cases[i].action, cases[i].amount, cases[i].speed);
        uint32_t exp = expect_ms(cases[i].action == CHASSIS_ACTION_CCW ||
                                 cases[i].action == CHASSIS_ACTION_CW,
                                 cases[i].amount,
                                 cases[i].speed ? cases[i].speed
                                                : ((cases[i].action == CHASSIS_ACTION_CCW ||
                                                    cases[i].action == CHASSIS_ACTION_CW)
                                                   ? CHASSIS_SPEED_ROTATE
                                                   : CHASSIS_SPEED_TRANSLATE),
                                 CHASSIS_ACCEL_DEFAULT);
        char msg[128];
        snprintf(msg, sizeof(msg), "%-22s %6u ms", cases[i].name, (unsigned)got);
        printf("  %s\n", msg);
        check_ms(got, exp, "  与独立计算一致");
    }
}

int main(void)
{
    printf("=========================================================\n");
    printf(" chassis.c 回归测试  帧格式=%s  同步=%s  细分=%u  标定=%u/%u\n",
           (CHASSIS_FRAME_LAYOUT == CHASSIS_FRAME_LAYOUT_9B) ? "9B" : "8B",
           CHASSIS_USE_SYNC ? "on" : "off",
           (unsigned)CHASSIS_PULSES_PER_REV,
           (unsigned)CHASSIS_PULSE_PER_1000MM,
           (unsigned)CHASSIS_PULSE_PER_360DEG);
    printf("=========================================================\n");

    reset_capture();
    Chassis_Init();
    printf("\n[0] Chassis_Init(): 4 帧使能共 %u 字节\n", (unsigned)g_tx_len);
    check(g_tx_len == 4u * 6u, "使能帧 F3 AB 01 00 6B x4");

    test_sample_frame();
    test_move_forward();
    test_strafe_right();
    test_rotate();
    test_speed_override();
    test_run_ms();
    test_estimate_table();

    printf("\n=========================================================\n");
    if (g_fail) {
        printf(" 结果：FAIL\n");
        return 1;
    }
    printf(" 结果：ALL PASS\n");
    return 0;
}
