#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Gyro.h"
#include "Key.h"
#include "LED.h"
#include "Servo.h"
#include "motor.h"
#include "path_planner.h"
#include "path_config.h"

static u8 dir = PATH_INITIAL_DIR;
static u16 judge_trans_display_count = 0;

static u8 judge(void);
static u8 judge_wait(u16 max_rounds);

/* 等待四个驱动电机全部到位。
 * 0x3A 查询的回包格式为 [地址][0x3A][状态][0x6B]，其中状态字节 (Serial_RxPacket[2])
 * 为 0x03 时表示该电机已到达。每查一个地址，先清标志再发查询，然后阻塞等回包，
 * 确认回包地址与本电机一致后再判断到位，避免把别的电机的回包误判成自己的。
 * max_rounds 是轮询轮数上限（一轮约 220 ms）：正式流程给 60000（约几小时，基本不会触发），
 * 自检模式给几十轮，某个地址的驱动器不在线时能很快报错而不是卡住。 */
static u8 judge_wait(u16 max_rounds)
{
    u8 judge1 = 0, judge2 = 0, judge3 = 0, judge4 = 0;
    u16 t, guard = 0;

    while(1){
        if(judge1 == 0){
            Serial_RxFlag = 0;
            Serial_RxPacket[0] = Serial_RxPacket[1] = Serial_RxPacket[2] = Serial_RxPacket[3] = 0;
            Serial_SendByte(0x01); Serial_SendByte(0x3A); Serial_SendByte(0x6B);
            for(t = 0; t < 50; t++){ if(Serial_GetRxFlag()) break; Delay_ms(1); }
            if(Serial_RxPacket[0] == 0x01 && Serial_RxPacket[1] == 0x3A && Serial_RxPacket[2] == 0x03)
                judge1 = 1;
        }
        Delay_ms(5);
        if(judge2 == 0){
            Serial_RxFlag = 0;
            Serial_RxPacket[0] = Serial_RxPacket[1] = Serial_RxPacket[2] = Serial_RxPacket[3] = 0;
            Serial_SendByte(0x02); Serial_SendByte(0x3A); Serial_SendByte(0x6B);
            for(t = 0; t < 50; t++){ if(Serial_GetRxFlag()) break; Delay_ms(1); }
            if(Serial_RxPacket[0] == 0x02 && Serial_RxPacket[1] == 0x3A && Serial_RxPacket[2] == 0x03)
                judge2 = 1;
        }
        Delay_ms(5);
        if(judge3 == 0){
            Serial_RxFlag = 0;
            Serial_RxPacket[0] = Serial_RxPacket[1] = Serial_RxPacket[2] = Serial_RxPacket[3] = 0;
            Serial_SendByte(0x03); Serial_SendByte(0x3A); Serial_SendByte(0x6B);
            for(t = 0; t < 50; t++){ if(Serial_GetRxFlag()) break; Delay_ms(1); }
            if(Serial_RxPacket[0] == 0x03 && Serial_RxPacket[1] == 0x3A && Serial_RxPacket[2] == 0x03)
                judge3 = 1;
        }
        Delay_ms(5);
        if(judge4 == 0){
            Serial_RxFlag = 0;
            Serial_RxPacket[0] = Serial_RxPacket[1] = Serial_RxPacket[2] = Serial_RxPacket[3] = 0;
            Serial_SendByte(0x04); Serial_SendByte(0x3A); Serial_SendByte(0x6B);
            for(t = 0; t < 50; t++){ if(Serial_GetRxFlag()) break; Delay_ms(1); }
            if(Serial_RxPacket[0] == 0x04 && Serial_RxPacket[1] == 0x3A && Serial_RxPacket[2] == 0x03)
                judge4 = 1;
        }
        Delay_ms(5);

        if(judge1 == 1 && judge2 == 1 && judge3 == 1 && judge4 == 1){
            return 1;
        }

        OLED_ShowNum(2, 1, judge_trans_display_count++, 5);

        if(++guard > max_rounds){   /* 安全兜底：电机不应答/地址不存在时退出 */
            return 0;
        }
    }
}

/* 正式流程用的默认等待：上限 60000 轮，基本只在电机死机时触发。 */
static u8 judge(void)
{
    return judge_wait(60000u);
}

static u8 translation(u16 x0, u16 y0, u16 x1, u16 y1)
{
    u16 distance;

    if(x1 == x0 && y1 > y0){
        distance = y1 - y0;
        if(dir == 2){ shun_yaw_(90); Delay_ms(1000); if(!judge()) return 0; dir = 1; }
        if(dir == 4){ ni_yaw_(90);   Delay_ms(1000); if(!judge()) return 0; dir = 1; }
        if(dir == 3) houtui_(distance);
        else         qianjin_(distance);
    }else if(x1 == x0 && y1 < y0){
        distance = y0 - y1;
        if(dir == 2){ ni_yaw_(90);   Delay_ms(1000); if(!judge()) return 0; dir = 3; }
        if(dir == 4){ shun_yaw_(90); Delay_ms(1000); if(!judge()) return 0; dir = 3; }
        if(dir == 3) qianjin_(distance);
        else         houtui_(distance);
    }else if(x1 < x0 && y1 == y0){
        distance = x0 - x1;
        if(dir == 1){ ni_yaw_(90);   Delay_ms(1000); if(!judge()) return 0; dir = 2; }
        else if(dir == 3){ shun_yaw_(90); Delay_ms(1000); if(!judge()) return 0; dir = 2; }
        if(dir == 2) qianjin_(distance);
        else         houtui_(distance);
    }else if(x1 > x0 && y1 == y0){
        distance = x1 - x0;
        if(dir == 1){ shun_yaw_(90); Delay_ms(1000); if(!judge()) return 0; dir = 4; }
        if(dir == 3){ ni_yaw_(90);   Delay_ms(1000); if(!judge()) return 0; dir = 4; }
        if(dir == 4) qianjin_(distance);
        else         houtui_(distance);
    }else{
        return 0;
    }

    return judge();
}

/* 把角度差限制到 [-180, 180) 度，输入/输出单位均为 0.01 度。 */
static int32_t gyro_wrap_180(int32_t angle)
{
    while(angle >= 18000L) angle -= 36000L;
    while(angle < -18000L) angle += 36000L;
    return angle;
}

/* 等待一帧新的有效角度，避免用旧数据重复计算 PD。 */
static u8 gyro_wait_new_sample(uint32_t *last_frame_count, int32_t *yaw)
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

/* ============================================================================
 * 陀螺仪闭环转向（全图路线和"闭环旋转测试"共用同一份实现）
 * ----------------------------------------------------------------------------
 * target_delta 单位 0.01°，正 = 逆时针（左转）、负 = 顺时针（右转）。
 *   ① 先按标定值把整个角度走完（不拆成小步，速度不受影响）；
 *   ② 等车体停稳再测一次实际转角，估"指令角度 → 实际转角"的增益（限幅 0.5~2.0）；
 *   ③ 之后按「误差 / 增益」补差，且必须换算成【度】再发；估不出增益就退回 Kp=0.5；
 *   ④ 误差 ≤ 0.1° 且连续 3 帧稳定算到位；最多补 5 次，用尽后残留 ≤1° 也算到位。
 *
 * 已经踩过的两个坑：
 *   - 补差量必须把 0.01° 换算成"度"再交给 ni_yaw_/shun_yaw_；
 *     直接发会被限幅成 30°，表现就是每次都转过头（超调）。
 *   - 电机刚停时车体还在晃，必须等角度稳定再测，否则会反复过冲、来回振荡。
 * ========================================================================== */
#define TURN_TOL_CENTI       10L    /* 到位容差 0.1°（单位 0.01°） */
#define TURN_MAX_CORR         5u    /* 补差次数上限 */
#define TURN_MAX_STEP_DEG    30L    /* 单次补差最大角度 */
#define TURN_SETTLE_MS      100u    /* 等停稳的采样间隔 */
#define TURN_SETTLE_TOL      50L    /* 相邻两次采样差 < 0.5° 认为停稳 */
#define TURN_SETTLE_TRIES     5u    /* 最多等 5 轮（0.5 s） */
#define MOVE_TIMEOUT_ROUNDS 500u    /* 等电机到位的轮数上限（约 11~110 s） */

static u8 gyro_ok;                  /* 收到过有效角度帧 */

/* 最近一次闭环转向的过程量（测试模式直接显示，路线模式也用来看） */
static volatile int32_t turn_dbg_target;     /* 目标，0.01° */
static volatile int32_t turn_dbg_meas;       /* 实测相对转角，0.01° */
static volatile int32_t turn_dbg_err;        /* 最后误差，0.01° */
static volatile int32_t turn_dbg_gain;       /* 估计增益 ×100，0 = 没估出来 */
static volatile u8      turn_dbg_corr;       /* 补差次数 */
static volatile u8      turn_dbg_result;     /* 1 到位 0 失败 */

/* 等四个电机到位（有界，避免驱动器掉线时整条路线卡死） */
static u8 wait_motors(void)
{
    return judge_wait(MOVE_TIMEOUT_ROUNDS);
}

/* PA8 是否按下（带消抖） */
static u8 pa8_hit(void)
{
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) != Bit_RESET) return 0u;
    Delay_ms(20);
    return (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET) ? 1u : 0u;
}

/* 等车体停稳：间隔 100 ms 的两次角度差 < 0.5° 就算停稳，最多等 0.5 s。
 * 电机的 0x3A"到位"只代表脉冲发完，车体还会继续晃，不等稳就测会反复过冲。 */
static u8 gyro_turn_settle(int32_t *yaw)
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

/* 闭环转 target_delta（0.01°），返回 1 到位 / 0 失败。
 * 过程量写进 turn_dbg_*，测试模式会把它显示出来。 */
static u8 gyro_turn_by(int32_t target_delta)
{
    int32_t  start_yaw, yaw, rel, err, step, gain = 0L;
    uint32_t deg_whole;
    u8       corr = 0u, stable = 0u, first_measure = 1u;

    turn_dbg_target = target_delta;
    turn_dbg_meas   = 0L;
    turn_dbg_err    = target_delta;
    turn_dbg_gain   = 0L;
    turn_dbg_corr   = 0u;
    turn_dbg_result = 0u;

    /* 没有陀螺仪数据：退化成开环，保证还能动（测试模式会显示 NO GYRO） */
    if(!Gyro_GetYawCentiDegrees(&start_yaw)){
        if(target_delta > 0L) ni_yaw_(90u);
        else                  shun_yaw_(90u);
        Delay_ms(1000);
        return wait_motors();
    }
    gyro_ok = 1u;

    /* ① 先走完整个角度 */
    deg_whole = (uint32_t)(((target_delta > 0L) ? target_delta : -target_delta) + 50L) / 100u;
    if(deg_whole < 1u) deg_whole = 1u;
    if(target_delta > 0L) ni_yaw_(deg_whole);       /* 逆时针 */
    else                  shun_yaw_(deg_whole);     /* 顺时针 */
    Delay_ms(50);
    if(!wait_motors()) return 0u;

    while(1){
        /* ② 停稳再测 */
        if(!gyro_turn_settle(&yaw)) return 0u;

        rel = gyro_wrap_180(yaw - start_yaw);
        err = gyro_wrap_180(target_delta - rel);

        turn_dbg_meas = rel;
        turn_dbg_err  = err;

        /* 第一次量完估一次增益（0.01 倍，限幅 0.5~2.0） */
        if(first_measure){
            int32_t rel_abs;

            first_measure = 0u;
            rel_abs = (rel > 0L) ? rel : -rel;
            if(rel_abs > 0L && deg_whole > 0u){
                gain = rel_abs * 100L / (int32_t)deg_whole;
                if(gain < 50L)  gain = 50L;
                if(gain > 200L) gain = 200L;
            }
            turn_dbg_gain = gain;
        }

        /* 第 2 行右侧显示实时角度（judge 只用 1~5 列） */
        OLED_ShowString(2, 7, "Y");
        OLED_ShowSignedNum(2, 8, yaw / 100L, 3);

        if(err <= TURN_TOL_CENTI && err >= -TURN_TOL_CENTI){
            if(++stable >= 3u){ turn_dbg_result = 1u; return 1u; }
            continue;
        }
        stable = 0u;

        if(corr >= TURN_MAX_CORR){
            turn_dbg_result = (err <= 100L && err >= -100L) ? 1u : 0u;
            return turn_dbg_result;
        }

        /* ③ 补差量：约定 step(度) = 误差(0.01°) / 增益(0.01 倍)，
         *    两者单位都是 0.01，直接相除就是"度"。没增益退回 Kp=0.5。 */
        if(gain > 0L) step = err / gain;
        else          step = (err + 100L) / 200L;
        if(step < 0L) step = -step;
        if(step < 1L) step = 1L;
        if(step > TURN_MAX_STEP_DEG) step = TURN_MAX_STEP_DEG;

        if(err > 0L) ni_yaw_((uint32_t)step);       /* 还差逆时针 */
        else         shun_yaw_((uint32_t)step);     /* 转过头了，顺时针补回 */

        Delay_ms(50);
        if(!wait_motors()) return 0u;

        corr++;
        turn_dbg_corr = corr;
    }
}

/* ===== 下面是旧版实现，保留作对照；实际使用的是上面的 gyro_turn_by ===== */
/* 陀螺仪比例转向：先执行目标增量，再按剩余误差的一半逐次修正。
 * target_delta 使用相对起始角度，单位 0.01 度；所有修正均有次数和角度上限。 */
static u8 gyro_turn_pos_test(int32_t target_delta)
{
    const int32_t TOLERANCE = 10L;        /* 到位误差 ±0.1 度 */
    const uint8_t MAX_CORRECTIONS = 5u;
    const int32_t MAX_CORRECTION_DEG = 30L;
    int32_t start_yaw, yaw, relative_yaw, error, deg, abs_error;
    uint32_t last_frame_count;
    uint8_t correction_count = 0u;
    uint8_t stable_count = 0u;
    uint8_t first_feedback = 1u;

    if(!Gyro_GetYawCentiDegrees(&start_yaw)) return 0;
    last_frame_count = Gyro_FrameCount;

    /* 先按目标方向执行一次完整位置命令。 */
    deg = (target_delta >= 0L) ? target_delta : -target_delta;
    deg = (deg + 50L) / 100L;
    if(deg < 1L) return 0;
    if(target_delta >= 0L)
        shun_yaw_((uint32_t)deg);
    else
        ni_yaw_((uint32_t)deg);

    Delay_ms(50);
    if(!judge()) return 0;

    while(1){
        if(!gyro_wait_new_sample(&last_frame_count, &yaw)) return 0;

        relative_yaw = gyro_wrap_180(yaw - start_yaw);
        error = gyro_wrap_180(target_delta - relative_yaw);
        OLED_ShowString(3, 1, "DY:");
        OLED_ShowSignedNum(3, 4, relative_yaw / 100L, 3);

        if(error <= TOLERANCE && error >= -TOLERANCE){
            if(++stable_count >= 3u) return 1;
            continue;
        }
        stable_count = 0u;

        /* 首次反馈若明显朝反方向变化，判为方向配置错误并停止修正。 */
        if(first_feedback){
            first_feedback = 0u;
            if((target_delta > 0L && relative_yaw < -500L) ||
               (target_delta < 0L && relative_yaw > 500L))
                return 0;
        }

        if(correction_count >= MAX_CORRECTIONS)
            return 0;   /* 已做完全部修正，当前测量仍未进入容差 */

        abs_error = (error >= 0L) ? error : -error;
        deg = (abs_error + 100L) / 200L;    /* Kp=0.5，四舍五入到整度 */
        if(deg < 1L) deg = 1L;
        if(deg > MAX_CORRECTION_DEG) deg = MAX_CORRECTION_DEG;

        if(error > 0L)
            shun_yaw_((uint32_t)deg);
        else
            ni_yaw_((uint32_t)deg);

        Delay_ms(50);
        if(!judge()) return 0;
        correction_count++;
    }

}

/* 从 (cx,cy) 规划到 (tx,ty) 并逐段执行，每段阻塞等电机到位。
 * 成功返回 1；无路径或某段超时返回 0。 */
static u8 go_to(u16 *cx, u16 *cy, u16 tx, u16 ty)
{
    u16 xpath[MAX_KEY_POINTS], ypath[MAX_KEY_POINTS];
    u8 i;

    if(*cx == tx && *cy == ty) return 1;          /* 已在目标点，无需移动 */

    path_plan(*cx, *cy, tx, ty, MAX_KEY_POINTS, xpath, ypath);
    if(key_point_num < 2) return 0;               /* 规划失败（无路径 / 参数非法） */

    for(i = 1; i < key_point_num; i++){
        if(!translation(xpath[i - 1], ypath[i - 1], xpath[i], ypath[i])) return 0;
    }

    *cx = tx;
    *cy = ty;
    return 1;
}

/* ============================================================================
 * 调试模式选择（改 DEBUG_MODE 一个数就能切换，几套代码都留在文件里）
 *
 *   DEBUG_MODE_TURN    4 = 闭环旋转单独测试（当前默认，按键触发）
 *   DEBUG_MODE_ROUTE   3 = 全图路线 A→E→B→C→D→B→C→D→A
 *   DEBUG_MODE_MOTOR   2 = 电机方向 / 地址自检（换电机后先用它）
 *   DEBUG_MODE_CIRCLE  1 = K230 圆心跟踪
 *   DEBUG_MODE_SERVO   0 = 舵机角度按键标定
 * ========================================================================== */
#define DEBUG_MODE_SERVO    0
#define DEBUG_MODE_CIRCLE   1
#define DEBUG_MODE_MOTOR    2
#define DEBUG_MODE_ROUTE    3
#define DEBUG_MODE_TURN     4

#define DEBUG_MODE          DEBUG_MODE_ROUTE

#if DEBUG_MODE == DEBUG_MODE_ROUTE

/* ============================================================================
 * 全图路线：case 1..8 状态机 + 陀螺仪闭环转向（当前默认）
 * ----------------------------------------------------------------------------
 * 场地图：A=起点1(2250,2250)  E=堆码(2250,1200)  B=原料(1200,2250)
 *         C=粗加工(1200,150)  D=暂存(150,1200)
 * 路线 A → E → B → C → D → B → C → D → A。
 *
 * 状态就是"第几段"：case 1 = 从起点走到第一个目标点，到了变 case 2，依此类推；
 * 8 段跑完 → case 99（PATH DONE）。OLED 第 1 行显示当前 CASE 号。
 *
 *   while(1){
 *     switch(step){
 *       case 1: step = leg_execute(0) ? 2 : 出错;  break;   // 起点 → E
 *       case 2: ...
 *       case 8: step = leg_execute(7) ? 99 : 出错; break;   // D → 起点
 *       case 99: PATH DONE;  case 98: PATH ERROR;  case 97: ABORT BY PA8;
 *     }
 *   }
 *
 * 一条腿内部：path_plan() 拆成直角小段 → 每小段先 leg_face() 对准朝向
 * （需要 ±90° 时走陀螺仪闭环 gyro_turn_by）→ 再 qianjin_/houtui_ 走完 → wait_motors()。
 *
 * 显示：1 行 CASE n/8，2 行 1~5 列是 judge 计数、右侧是实时角度，
 *       3 行 TO 目标，4 行 AT 当前位置。任何阶段按 PA8 中止。
 * ========================================================================== */
#define ROUTE_LEG_COUNT  8u

/* 状态量（放文件级，调试器里能直接看） */
static u16 fsm_x, fsm_y;          /* 当前场地坐标 */
static u8  seg_backward;          /* 1 = 本小段倒着走 */
static u8  abort_flag;            /* 1 = 按过 PA8 */

/* 在指定行画 "TO 2250,1200" / "AT 150,1200" */
static void route_show_xy(uint8_t line, char *label, u16 x, u16 y)
{
    OLED_ShowString(line, 1, label);
    OLED_ShowNum(line, 4, x, 4);
    OLED_ShowString(line, 8, ",");
    OLED_ShowNum(line, 9, y, 4);
}

/* 朝向差：0=同向 1=左转90°(逆时针) 2=相反 3=右转90°(顺时针)。朝向编号 1北 2西 3南 4东 */
static u8 heading_turn(u8 from, u8 to)
{
    return (u8)((to + 4u - from) % 4u);
}

/* 轴对齐小段的目标朝向 */
static u8 segment_dir(u16 x0, u16 y0, u16 x1, u16 y1)
{
    if(x1 == x0) return (u8)((y1 > y0) ? 1u : 3u);
    return (u8)((x1 > x0) ? 4u : 2u);
}

/* 轴对齐小段的长度（mm） */
static u16 segment_len(u16 x0, u16 y0, u16 x1, u16 y1)
{
    if(x1 == x0) return (u16)((y1 > y0) ? (y1 - y0) : (y0 - y1));
    return (u16)((x1 > x0) ? (x1 - x0) : (x0 - x1));
}

/* 把车头对准 req 朝向：同向直行、相反倒车、±90° 走陀螺仪闭环转 */
static u8 leg_face(u8 req)
{
    u8 t = heading_turn(dir, req);

    if(t == 0u){ seg_backward = 0u; return 1u; }
    if(t == 2u){ seg_backward = 1u; return 1u; }   /* 180°：不转，倒着走 */

    if(!gyro_turn_by((t == 1u) ? 9000L : -9000L)) return 0u;

    dir = req;
    seg_backward = 0u;
    return 1u;
}

/* 走到场地坐标 (tx, ty)：规划 → 逐小段「对准朝向(闭环) → 直行/倒车」，成功返回 1。
 * 目标点直接写在 route_run 的 case 里（如 case 1 就是去 (2250,1200)），
 * 函数一进来先把 "TO x,y" 画到第 3 行。 */
static u8 leg_goto(u16 tx, u16 ty)
{
    u16 kpx[MAX_KEY_POINTS], kpy[MAX_KEY_POINTS];
    u8  n, i;
    u16 d;

    route_show_xy(3, "TO ", tx, ty);        /* 立刻显示本段目标 */

    path_plan(fsm_x, fsm_y, tx, ty, MAX_KEY_POINTS, kpx, kpy);
    n = key_point_num;
    if(n < 2u) return 0u;                   /* 无路径 / 起点终点非法 */

    for(i = 1u; i < n; i++){
        if(!leg_face(segment_dir(kpx[i - 1u], kpy[i - 1u], kpx[i], kpy[i]))) return 0u;

        d = segment_len(kpx[i - 1u], kpy[i - 1u], kpx[i], kpy[i]);
        if(seg_backward) houtui_(d);
        else             qianjin_(d);
        if(!wait_motors()) return 0u;

        if(pa8_hit()){ abort_flag = 1u; return 0u; }
    }

    fsm_x = tx;
    fsm_y = ty;
    return 1u;
}

/* 主循环：状态就是"第几段"（case 1..8），状态号显示在 OLED 第 1 行 */
static void route_run(void)
{
    u8 step = 1u;
    u8 n;
    int32_t yaw = 0;
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_8;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    path_planner_init();

    fsm_x      = PATH_START_X;
    fsm_y      = PATH_START_Y;
    abort_flag = 0u;
    gyro_ok    = 0u;

    OLED_Clear();
    route_show_xy(4, "AT ", fsm_x, fsm_y);

    /* 3 秒倒计时，留时间把车摆正 */
    for(n = 3u; n > 0u; n--){
        OLED_ShowString(1, 1, "START IN ");
        OLED_ShowNum(1, 10, n, 1);
        Delay_ms(1000);
    }

    while(1){
        /* ---- 顶栏：当前状态（case 号）；TO 由 leg_goto() 进来时自己画 ---- */
        if(step <= ROUTE_LEG_COUNT){
            OLED_ShowString(1, 1, "CASE ");
            OLED_ShowNum(1, 6, (uint32_t)step, 1);
            OLED_ShowString(1, 7, "/8   ");
        }
        route_show_xy(4, "AT ", fsm_x, fsm_y);

        /* 第 2 行右侧：没有陀螺仪时提示（judge 只用 1~5 列） */
        if(!Gyro_GetYawCentiDegrees(&yaw) && !gyro_ok){
            OLED_ShowString(2, 7, "NO GYRO");
        }

        /* PA8 中止：任何阶段按下都停 */
        if(step <= ROUTE_LEG_COUNT && pa8_hit()) step = 97u;

        /* 每个 case = 走到一个目标点，到了就把 step 换成下一个 */
        switch(step){
        case 1: step = leg_goto(2250u, 1200u) ? 2u : (abort_flag ? 97u : 98u); break;  /* → E 堆码 */
        case 2: step = leg_goto(1200u, 2250u) ? 3u : (abort_flag ? 97u : 98u); break;  /* → B 原料 */
        case 3: step = leg_goto(1200u,  150u) ? 4u : (abort_flag ? 97u : 98u); break;  /* → C 粗加工 */
        case 4: step = leg_goto( 150u, 1200u) ? 5u : (abort_flag ? 97u : 98u); break;  /* → D 暂存 */
        case 5: step = leg_goto(1200u, 2250u) ? 6u : (abort_flag ? 97u : 98u); break;  /* → B */
        case 6: step = leg_goto(1200u,  150u) ? 7u : (abort_flag ? 97u : 98u); break;  /* → C */
        case 7: step = leg_goto( 150u, 1200u) ? 8u : (abort_flag ? 97u : 98u); break;  /* → D */
        case 8: step = leg_goto(2250u, 2250u) ? 99u : (abort_flag ? 97u : 98u); break; /* → 回到起点 A */

        case 99:
            /* 结束画面留最后一次转向的增益/补差次数，方便赛后复盘标定 */
            OLED_ShowString(1, 1, "PATH DONE");
            OLED_ShowString(3, 1, "G");
            OLED_ShowNum(3, 2, (uint32_t)turn_dbg_gain, 3);
            OLED_ShowString(3, 6, " C");
            OLED_ShowNum(3, 8, turn_dbg_corr, 1);
            Delay_ms(200);
            break;

        case 98:
            OLED_ShowString(1, 1, "PATH ERROR");
            Delay_ms(200);
            break;

        case 97:
        default:
            OLED_ShowString(1, 1, "ABORT BY PA8");
            Delay_ms(200);
            break;
        }
    }
}
#elif DEBUG_MODE == DEBUG_MODE_TURN

/* ============================================================================
 * 闭环旋转单独测试（不跑路线）
 * ----------------------------------------------------------------------------
 * 直接调用路线里同一个 gyro_turn_by()，只为把闭环转向单独验一遍。
 *
 *   PA12 = 逆时针（左转）转当前档位
 *   PA11 = 顺时针（右转）转当前档位
 *   PA8  = 切换档位 15° → 45° → 90° → 15° …
 *
 * 每次转完把过程量留在屏幕上：
 *   1 行  TGT 目标角度   C 补差次数   G 增益×100（0 = 没估出来）
 *   2 行  左半是 judge 查询计数，右侧是陀螺仪实时角度 / NO GYRO
 *   3 行  MEA 实测相对转角
 *   4 行  ERR 最终误差 + OK / FAIL
 * ========================================================================== */
static const u16 turn_test_deg[3] = {15u, 45u, 90u};

/* 把 0.01° 显示成 ±ddd.cc（占 7 列） */
static void turn_show_2dp(uint8_t line, uint8_t col, int32_t centi)
{
    int32_t v = (centi < 0L) ? -centi : centi;

    OLED_ShowChar(line, col, (centi < 0L) ? '-' : '+');
    OLED_ShowNum(line, (uint8_t)(col + 1u), (uint32_t)(v / 100L), 3);
    OLED_ShowChar(line, (uint8_t)(col + 4u), '.');
    OLED_ShowNum(line, (uint8_t)(col + 5u), (uint32_t)(v % 100L), 2);
}

/* 把最近一次转向的过程量画出来 */
static void turn_test_show(void)
{
    if(turn_dbg_target >= 0L){
        OLED_ShowString(1, 1, "TGT+");
        OLED_ShowNum(1, 5, (uint32_t)(turn_dbg_target / 100L), 3);
    }else{
        OLED_ShowString(1, 1, "TGT-");
        OLED_ShowNum(1, 5, (uint32_t)(-turn_dbg_target / 100L), 3);
    }
    OLED_ShowString(1, 9, "C");
    OLED_ShowNum(1, 10, turn_dbg_corr, 1);
    OLED_ShowString(1, 11, " G");
    OLED_ShowNum(1, 13, (uint32_t)turn_dbg_gain, 3);

    OLED_ShowString(3, 1, "MEA");
    turn_show_2dp(3, 5, turn_dbg_meas);

    OLED_ShowString(4, 1, "ERR");
    turn_show_2dp(4, 5, turn_dbg_err);
    OLED_ShowString(4, 12, turn_dbg_result ? " OK " : "FAIL");
}

static void turn_test_clear(void)
{
    turn_dbg_meas   = 0L;
    turn_dbg_err    = 0L;
    turn_dbg_gain   = 0L;
    turn_dbg_corr   = 0u;
    turn_dbg_result = 0u;
}

static void turn_test_run(void)
{
    u8 idx = 2u;                 /* 0=15° 1=45° 2=90°，默认 90° */
    u8 key;
    int32_t target, yaw_now;
    GPIO_InitTypeDef gpio;

    /* PA8：换档位 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_8;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    OLED_Clear();
    OLED_ShowString(1, 1, "TURN TEST");
    OLED_ShowString(3, 1, "A12=CCW A11=CW");
    OLED_ShowString(4, 1, "PA8>NEXT ANGLE");

    turn_dbg_target = (int32_t)turn_test_deg[idx] * 100L;
    turn_test_clear();
    Delay_ms(600);
    turn_test_show();

    while(1){
        /* 第 2 行右侧：陀螺仪状态（judge 只用 1~5 列） */
        if(Gyro_GetYawCentiDegrees(&yaw_now)){
            gyro_ok = 1u;
            OLED_ShowString(2, 7, "Y");
            OLED_ShowSignedNum(2, 8, yaw_now / 100L, 3);
        }else if(!gyro_ok){
            OLED_ShowString(2, 7, "NO GYRO");
        }

        /* PA8：换档位 */
        if(pa8_hit()){
            idx = (u8)((idx + 1u) % 3u);
            turn_dbg_target = (int32_t)turn_test_deg[idx] * 100L;
            turn_test_clear();
            turn_test_show();
        }

        key = Key_GetNum();
        if(key != 4 && key != 5){
            Delay_ms(10);
            continue;
        }

        target = (int32_t)turn_test_deg[idx] * 100L;   /* 0.01° */
        if(key == 5) target = -target;                 /* PA11 = 顺时针 */

        OLED_ShowString(4, 1, "TURNING...     ");
        (void)gyro_turn_by(target);                    /* 过程量写进 turn_dbg_* */
        turn_test_show();
    }
}
#elif DEBUG_MODE == DEBUG_MODE_MOTOR

/* ============================================================================
 * 电机方向 / 地址自检
 * ----------------------------------------------------------------------------
 * 换了电机以后驱动器地址和轮子的对应关系可能是乱的，先用这个模式把"地址"和
 * "轮子"对上，再回来改 motor.h 里的 MOTOR_ADDR_FL/FR/RL/RR。
 *
 * 这块板子上只有 3 个按键可用（PB1/PB11/PB10 不接）：
 *   PA12 = 本页第 1 个动作    PA11 = 本页第 2 个动作    PA8 = 翻页
 *
 * 共 8 页，每页固定两个动作，A12 是"正向/正转"，A11 是"反向/反转"：
 *   第 1 页  前进 / 后退
 *   第 2 页  左平移 / 右平移
 *   第 3 页  逆时针 15° / 顺时针 15°（先用小角度验证方向）
 *   第 4 页  逆时针 90° / 顺时针 90°
 *   第 5 页  地址 1：正转点动 / 反转点动
 *   第 6 页  地址 2：同上
 *   第 7 页  地址 3：同上
 *   第 8 页  地址 4：同上
 *
 * 判定方法：
 *   - 第 5~8 页一次只有一个轮子动，动的是哪个位置 → 那个地址就是那个轮子，
 *     同时正/反两次也能看出它的转向极性对不对；
 *   - 第 1 页前进时四个轮子应当"左右两侧反向、对角同向"地配合；哪个轮子
 *     转反了，就是该轮极性不对（换线或改方向位）；
 *   - 第 3/4 页旋转走位置模式，四个轮子都会收到命令（F6 速度模式才只用 FL/FR）；
 *     转向方向不对时，重点查 FR、RR 的镜像方向位有没有和实际安装对上。
 * ========================================================================== */
#define TEST_STEP_MM       40u    /* 整车平移一次走多远 */
#define TEST_TURN_DEG      90u    /* 大角度旋转 */
#define TEST_TURN_SMALL    15u    /* 小角度旋转，先小步验证方向 */
#define TEST_JOG_PULSES    390u   /* 单电机点动脉冲，约 30 mm（13008 脉冲/1000 mm） */
#define MOTOR_TEST_PAGES   8u     /* 自检页数 */

/* 画某一页的静态提示（每行最多 16 列）；第 4 行固定提示 PA8 翻页。 */
static void motor_check_draw(uint8_t page)
{
    OLED_Clear();

    switch(page){
        case 1:
            OLED_ShowString(1, 1, "P1/8 FWD/BACK");
            OLED_ShowString(2, 1, "A12 = FWD");
            OLED_ShowString(3, 1, "A11 = BACK");
            break;
        case 2:
            OLED_ShowString(1, 1, "P2/8 LEFT/RIGHT");
            OLED_ShowString(2, 1, "A12 = LEFT");
            OLED_ShowString(3, 1, "A11 = RIGHT");
            break;
        case 3:
            OLED_ShowString(1, 1, "P3/8 ROT 15DEG");
            OLED_ShowString(2, 1, "A12 = CCW 15");
            OLED_ShowString(3, 1, "A11 = CW 15");
            break;
        case 4:
            OLED_ShowString(1, 1, "P4/8 ROT 90DEG");
            OLED_ShowString(2, 1, "A12 = CCW 90");
            OLED_ShowString(3, 1, "A11 = CW 90");
            break;
        default:
            /* 第 5~8 页：地址 = 页号 - 4，逐页点动一个地址 */
            OLED_ShowString(1, 1, "P");
            OLED_ShowNum(1, 2, page, 1);
            OLED_ShowString(1, 3, "/8 ADDR ");
            OLED_ShowNum(1, 11, (uint32_t)(page - 4u), 1);
            OLED_ShowString(2, 1, "A12 = JOG +");
            OLED_ShowString(3, 1, "A11 = JOG -");
            break;
    }

    OLED_ShowString(4, 1, "PA8>NEXT PAGE");
}

/* 按当前页 + 按键执行一个动作（A12=key4 走正向，A11=key5 走反向）。 */
static void motor_check_run(uint8_t page, uint8_t key)
{
    if(page == 1){
        if(key == 4)      qianjin_(TEST_STEP_MM);
        else if(key == 5) houtui_(TEST_STEP_MM);
    }else if(page == 2){
        if(key == 4)      zuo_(TEST_STEP_MM);
        else if(key == 5) you_(TEST_STEP_MM);
    }else if(page == 3){
        if(key == 4)      ni_yaw_(TEST_TURN_SMALL);      /* 逆时针 */
        else if(key == 5) shun_yaw_(TEST_TURN_SMALL);    /* 顺时针 */
    }else if(page == 4){
        if(key == 4)      ni_yaw_(TEST_TURN_DEG);
        else if(key == 5) shun_yaw_(TEST_TURN_DEG);
    }else{
        uint8_t addr = (uint8_t)(page - 4u);             /* 5~8 页 -> 地址 1~4 */

        if(key == 4)      motor_jog_addr(addr, 0u, TEST_JOG_PULSES);
        else if(key == 5) motor_jog_addr(addr, 1u, TEST_JOG_PULSES);
    }
}

/* 电机自检主循环：不返回。 */
static void motor_check_test(void)
{
    uint8_t page = 1;
    uint8_t key;
    GPIO_InitTypeDef gpio;

    /* PA8 当翻页键：上拉输入，按下接地 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_8;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    OLED_ShowString(1, 1, "MOTOR TEST");
    motor_check_draw(page);

    while(1){
        /* PA8：翻页（带消抖和松手等待） */
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET){
            Delay_ms(20);
            if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET){
                page = (uint8_t)((page % MOTOR_TEST_PAGES) + 1u);
                motor_check_draw(page);
                while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET) Delay_ms(10);
                Delay_ms(20);
            }
        }

        /* 只用 PA12(key 4) 和 PA11(key 5)；PB1/PB11/PB10 的键码 1/2/3 一律忽略 */
        key = Key_GetNum();
        if(key != 4 && key != 5){
            Delay_ms(10);
            continue;
        }

        motor_check_run(page, key);

        /* 发完动作必须等四个电机到位，否则连按会叠命令。
         * 这里用有界等待（约 9 s）：换电机时某个地址不在线能直接报 NO ANSWER。 */
        OLED_ShowString(4, 1, judge_wait(40u) ? "OK            " : "NO ANSWER     ");

        Delay_ms(500);
        motor_check_draw(page);      /* 恢复本页提示 */
    }
}

#elif DEBUG_MODE == DEBUG_MODE_CIRCLE

/* ============================================================================
 * K230 圆心跟踪调试
 * ----------------------------------------------------------------------------
 * K230 经 USART2 发 ASCII "x,y\r\n"：x/y 是圆心在图像里的像素坐标（左上角原点）。
 * 目标：让圆心停在画面中心 (320,240)。
 *
 * 轴向映射（按当前相机安装方向，换相机/改布线后只需改这三行判断）：
 *   y > 240（圆心偏下）→ 向右平移 you_()       y < 240 → 向左平移 zuo_()
 *   x > 320（圆心偏右）→ 后退     houtui_()     x < 320 → 前进     qianjin_()
 *
 * 比例：60 px ≈ 35 mm（3.5 cm）；单步限幅 40 mm；死区 ±8 px（约 5 mm），
 * 死区内不动作，避免在原地反复微调抖动。
 *
 * judge 的三条约束（把视觉和底盘闭环起来最容易踩坑的地方）：
 *   1) 每发完一帧运动命令必须 judge() 等四个电机全部到位，再读下一帧图像；
 *      否则旧的位移还没走完就叠加新命令，车会丢步、画面来回抖；
 *   2) 没有新帧（K230 断线/卡住）时绝不重复下发指令，只等待；
 *   3) 两轴分开做：先修 x（前后）并 judge，再修 y（左右）并 judge，不叠加。
 * judge() 收的是 USART1 上电机的 0x3A 应答，K230 走 USART2 中断，互不抢串口。
 * ========================================================================== */
#define K230_TARGET_X        320u
#define K230_TARGET_Y        240u
#define K230_DEADBAND_PX     8
#define K230_MM_PER_60PX     35u
#define K230_MAX_STEP_MM     40u

/* 像素偏差 → 毫米：按 60 px = 35 mm 线性换算，四舍五入后夹到 1~40 mm。
 * 传负数也按绝对值算，避免调用方漏取绝对值时"静默不动"。 */
static uint32_t k230_px_to_mm(int32_t px)
{
    uint32_t p, mm;

    if(px < 0) p = (uint32_t)(-(px + 1)) + 1u;   /* 两补码取绝对值，INT32_MIN 也安全 */
    else       p = (uint32_t)px;
    if(p == 0u) return 0u;

    mm = (p * K230_MM_PER_60PX + 30u) / 60u;
    if(mm < 1u) mm = 1u;
    if(mm > K230_MAX_STEP_MM) mm = K230_MAX_STEP_MM;
    return mm;
}

/* OLED 第 3 行显示当前圆心像素坐标，固定 3 位宽，不会留残影。 */
static void k230_show_xy(uint16_t x, uint16_t y)
{
    OLED_ShowString(3, 1, "X=");
    OLED_ShowNum(3, 3, x, 3);
    OLED_ShowString(3, 6, " Y=");
    OLED_ShowNum(3, 9, y, 3);
}

/* 单轴走一步：err > 0 走 move_pos，err < 0 走 move_neg。
 * OLED 第 4 行显示方向 + 本次步长（6 字符标签 + 3 位毫米数 + "mm"）。
 * 返回 0 表示 judge() 超时（电机没回 0x3A 状态 0x03）。 */
static u8 k230_axis_step(int32_t err,
                         void (*move_pos)(uint32_t),
                         void (*move_neg)(uint32_t),
                         char *pos_label,
                         char *neg_label)
{
    uint32_t mm = k230_px_to_mm(err);

    if(mm == 0u) return 1u;

    if(err > 0) move_pos(mm);
    else        move_neg(mm);

    OLED_ShowString(4, 1, (err > 0) ? pos_label : neg_label);
    OLED_ShowNum(4, 7, mm, 3);
    OLED_ShowString(4, 10, "mm ");

    return judge();     /* 等四个电机都到位，才允许下发下一帧命令 */
}

/* 按一帧坐标修正一次：先前后（x），再左右（y）。返回 0 表示 judge 失败。 */
static u8 k230_apply(uint16_t cx, uint16_t cy)
{
    int32_t dx = (int32_t)cx - (int32_t)K230_TARGET_X;
    int32_t dy = (int32_t)cy - (int32_t)K230_TARGET_Y;
    u8 moved = 0;

    if(dx > K230_DEADBAND_PX || dx < -K230_DEADBAND_PX){
        if(!k230_axis_step(dx, houtui_, qianjin_, "BACK  ", "AHEAD ")) return 0;
        moved = 1;
    }

    if(dy > K230_DEADBAND_PX || dy < -K230_DEADBAND_PX){
        if(!k230_axis_step(dy, you_, zuo_, "RIGHT ", "LEFT  ")) return 0;
        moved = 1;
    }

    if(!moved) OLED_ShowString(4, 1, "IN RANGE      ");   /* 已在死区内，不动 */

    return 1;
}

/* 圆心跟踪主循环：只在收到新的有效坐标时才动作，不返回。 */
static void circle_track_test(void)
{
    uint16_t cx = 0, cy = 0;
    uint32_t frame_count = 0;

    OLED_Clear();
    OLED_ShowString(1, 1, "K230 TRACK");
    OLED_ShowString(2, 7, "JCNT");        /* 2 行 1~5 列留给 judge 的查询计数 */
    OLED_ShowString(4, 1, "WAIT K230     ");

    while(1){
        /* 还没收到过任何有效帧：只等，不发运动指令。 */
        if(!K230_GetCenter(&cx, &cy)){
            Delay_ms(10);
            continue;
        }

        /* 没有新帧（K230 断线或卡住）：保持等待，绝不拿旧坐标重复下发。 */
        if(K230_FrameCount == frame_count){
            Delay_ms(10);
            continue;
        }
        frame_count = K230_FrameCount;

        k230_show_xy(cx, cy);

        if(!k230_apply(cx, cy)){
            OLED_ShowString(1, 1, "JUDGE FAIL");
            OLED_ShowString(4, 1, "MOTOR NO 0x3A ");
            while(1) Delay_ms(100);       /* 电机应答异常，停在这里等现场处理 */
        }
    }
}

#else

/* 舵机角度按键标定：PA12 每按一次 +1°，PA11 每按一次 -1°，限幅 0~100°。 */
static void servo_angle_test(void)
{
    uint8_t key;
    int servo_angle = SERVO_ANGLE_OPEN;   /* 上电张开（38°），加紧为 44° */

    Servo_SetAngle((uint16_t)servo_angle);
    OLED_ShowString(1, 1, "ANGLE:");
    OLED_ShowNum(1, 7, (uint32_t)servo_angle, 3);
    OLED_ShowString(2, 1, "PA12:+1 PA11:-1");

    while(1){
        key = Key_GetNum();
        if(key == 4){                                       /* PA12：+1° */
            if(servo_angle < (int)SERVO_ANGLE_MAX) servo_angle++;
        }else if(key == 5){                                 /* PA11：-1° */
            if(servo_angle > 0) servo_angle--;
        }else{
            Delay_ms(10);
            continue;
        }

        Servo_SetAngle((uint16_t)servo_angle);
        OLED_ShowNum(1, 7, (uint32_t)servo_angle, 3);
    }
}

#endif /* DEBUG_MODE */

int main(void)
{
    u16 cur_x = PATH_START_X;
    u16 cur_y = PATH_START_Y;
    u8 step = 0;
	
	s32 yaw_centi_degrees = 0, last_yaw_centi_degrees = 0;
    LED_Init();
    Servo_Init();             /* PC8(TIM3_CH3) 舵机 PWM，上电默认张开 38° */
    OLED_Init();
    Key_Init();
    Serial_Init();
    Gyro_Init();
    USART3_Init(115200);
    USART2_Init(115200);
	
	
	uint32_t oled_frame_us;

	/* 硬件 I2C1 效率测试：整屏 1024 字节刷新耗时，结果显示在 OLED 3、4 行，
	 * 同时从 USART3 打印一份，方便终端查看。测试会清屏，需放在其它绘图之前。 */
	oled_frame_us = OLED_ShowI2CStats();
	USART3_Printf("OLED HW I2C: frame=%dus, %dKB/s, util=%d%%\r\n",
	              (int)oled_frame_us,
	              (int)(1000000u / (oled_frame_us ? oled_frame_us : 1u)),
	              (int)(2304000u / (oled_frame_us ? oled_frame_us : 1u)));

	/* 电机上电默认失能（F3 使能帧之前所有运动指令都会被忽略），
	 * 任何会驱动底盘的模式都必须先使能，否则车一动不动。 */
	motor_enable_all();

	/* 调试模式派发：各模式函数内部都是死循环，正常情况下不会返回。
	 * 切换模式改文件上方的 DEBUG_MODE 即可。 */
#if DEBUG_MODE == DEBUG_MODE_TURN
	turn_test_run();            /* 闭环旋转单独测试：PA12 逆时针 / PA11 顺时针 / PA8 换档 */
#elif DEBUG_MODE == DEBUG_MODE_ROUTE
	route_run();                /* 全图路线：A→E→B→C→D→B→C→D→A */
#elif DEBUG_MODE == DEBUG_MODE_MOTOR
	motor_check_test();         /* 电机方向 / 地址自检（换电机后先用它） */
#elif DEBUG_MODE == DEBUG_MODE_CIRCLE
	circle_track_test();        /* K230 圆心跟踪：让圆心停在 (320,240) */
#else
	servo_angle_test();         /* 舵机角度按键标定 */
#endif


	/* 通用发送自检：发送字符串/字符/整数/小数/角度，串口终端核对输出。
	 * 需要恢复实时陀螺仪角度输出时，注释掉本循环、打开下方被注释的循环即可。 */
//	while(1){
//		USART3_SelfTest();
//		Delay_ms(1000);
//	}

	/* 陀螺仪角度经 USART3(PB10) 输出给下位机，格式 "xxx.xx\r\n"。
	 * 下位机以 \r 或 \n 作为一帧结束，直接按字符串显示即可。 */

//	while(1){
//		if (Gyro_GetYawCentiDegrees(&yaw_centi_degrees))
//		{
//			USART3_SendAngleCentiDegrees(yaw_centi_degrees);
//			OLED_ShowSignedNum(2, 1, yaw_centi_degrees / 100, 8);
//		}
//		Delay_ms(200);
//	}

	s32 yaw_error, rel;
	while(1){
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET){
			Delay_ms(20);
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET){
				/* 逆时针 90°：先记起始角。你的陀螺仪逆时针是角度增大(0→+180→-180→0)，
				 * 所以目标 = 起始角 + 90°，单位 0.01°。 */
				if(Gyro_GetYawCentiDegrees(&last_yaw_centi_degrees)){
					for(int i=0;i<5;i++){
						if(!Gyro_GetYawCentiDegrees(&yaw_centi_degrees)) break;
						/* 相对转角和误差都包到 [-180°,180°)，跨 ±180° 才不跳变 */
						rel = gyro_wrap_180(yaw_centi_degrees - last_yaw_centi_degrees);
						yaw_error = gyro_wrap_180(9000 - rel);

						if(yaw_error >= -5 && yaw_error <= 5) break;  /* 误差≤0.05°到位 */

						if(yaw_error > 0)
							ni_yaw_((uint32_t)(( yaw_error + 50) / 100));//+50是四舍五入
						else
							shun_yaw_((uint32_t)((-yaw_error + 50) / 100));  /* 转过了，顺时针补回 */

						if(!judge()) break;
						Delay_ms(50);
					}
				}
				/* 松手防抖，避免按住时连续触发 */
				while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET) Delay_ms(10);
				Delay_ms(20);
			}
		}
		USART3_Printf("angle=%d\r\n",(int)yaw_centi_degrees);
		Delay_ms(20);
	}


	shun_yaw_(90);
	while(1){
		Delay_ms(10);
		if(judge()){
			break;
		}
	}
	ni_yaw_(90);
	while(1){
		Delay_ms(10);
		if(judge()){
			break;
		}
	}
    /* 临时角度输出测试：陀螺仪以 100 Hz 接收，USART3 以 5 Hz 输出最新角度。 */
//    while(1)
//    {
//        if (Gyro_GetYawCentiDegrees(&yaw_centi_degrees))
//            USART3_SendAngleCentiDegrees(yaw_centi_degrees);
//			OLED_ShowSignedNum(2,1,yaw_centi_degrees/100,8);
//        Delay_ms(200);
//    }

    /* PA8 低电平按键，作为临时 90 度闭环测试触发。 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    }
    Delay_ms(200);

    path_planner_init();
    motor_enable_all();

    /* 临时比例转向测试：PA8 按下闭环转向 +90 度，松开后可再次测试。
     * 测试循环放在正式路线循环之前，因此本版本不会启动自动路线。 */
    OLED_ShowString(1, 1, "PA8 TURN TEST");
    while(1)
    {
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET){
            Delay_ms(20);
            if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET){
                OLED_ShowString(2, 1, "TURNING       ");
                if(gyro_turn_pos_test(9000L))
                    OLED_ShowString(2, 1, "TURN OK       ");
                else
                    OLED_ShowString(2, 1, "TURN ERROR    ");

                while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET)
                    Delay_ms(10);
                Delay_ms(20);
            }
        }
        Delay_ms(10);
    }

//	go_to(&cur_x, &cur_y, 2250u, 1200u);
//	while(1){
//		
//	}
    /* 路线：每个 case 开头指定「下一个地点」，随后 go_to 完成规划 + 发送电机 + 等到位，
     * 全部到位后才进入下一个 case（不共享 path_ready/path_index 之类的中间状态）。 */
    while(1)
    {
        if (Gyro_GetYawCentiDegrees(&yaw_centi_degrees))
            OLED_ShowSignedNum(4, 5, yaw_centi_degrees, 6);

        switch(step)
        {
            case 0: if(!go_to(&cur_x, &cur_y, 2250u, 1200u)) step = 99; else step = 1;   break; /* -> (2250,1200) */
            case 1: if(!go_to(&cur_x, &cur_y, 1200u, 2250u)) step = 99; else step = 2;   break; /* -> (1200,2250) */
            case 2: if(!go_to(&cur_x, &cur_y, 1200u, 150u))  step = 99; else step = 3;   break; /* -> (1200,150)  */
            case 3: if(!go_to(&cur_x, &cur_y, 150u,  1200u)) step = 99; else step = 4;   break; /* -> (150,1200)  */
            case 4: if(!go_to(&cur_x, &cur_y, 1200u, 2250u)) step = 99; else step = 5;   break; /* -> (1200,2250) */
            case 5: if(!go_to(&cur_x, &cur_y, 1200u, 150u))  step = 99; else step = 6;   break; /* -> (1200,150)  */
            case 6: if(!go_to(&cur_x, &cur_y, 150u,  1200u)) step = 99; else step = 7;   break; /* -> (150,1200)  */
            case 7: if(!go_to(&cur_x, &cur_y, PATH_TARGET_X, PATH_TARGET_Y)) step = 99; else step = 100;   break; /* -> (1200,2250) */
            case 8: if(!go_to(&cur_x, &cur_y, 1200u, 150u))  step = 99; else step = 9;   break; /* -> (1200,150)  */
            case 9: if(!go_to(&cur_x, &cur_y, PATH_TARGET_X, PATH_TARGET_Y)) step = 99; else step = 100; break; /* -> 终点(2250,2250) */

            case 100:
                OLED_ShowString(1, 1, "PATH DONE");
                while(1){}
            case 99:
                OLED_ShowString(1, 1, "PATH ERROR");
                while(1){}
        }
    }
}
