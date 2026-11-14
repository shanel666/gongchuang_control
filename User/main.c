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
#include "chassis.h"
#include "chassis_config.h"
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
 * 陀螺仪闭环转向（绝对角度；全图路线和"闭环旋转测试"共用同一份实现）
 * ----------------------------------------------------------------------------
 * 角度一律用"绝对角度"：开机时记一次零点 yaw_zero，之后 北=0°、西=+90°、
 * 南=180°、东=-90°（逆时针为正，和陀螺仪一致）。每次转向都是"转到某个绝对角度"，
 * 所以多段路线之间不会累积偏差。
 *
 * 修正方式：按当前误差直接算脉冲，一步到位，不再按比例一点点逼近：
 *     脉冲 = |误差| / 36000 * PULSE_PER_360DEG
 *   例：误差 10.00° → 17208/360×10 = 478 脉冲；误差 36.00° → 1721 脉冲。
 *   每次发完都等电机到位、等车体停稳，再读一次全局角度重算误差，
 *   最多补 TURN_MAX_CORR 次（误差按 1-标定误差 的比例衰减，标定差一半也能收敛）。
 *
 * 已经踩过的坑：
 *   - 补差必须换算成"度/脉冲"再发；把 0.01° 的数当度发，会每次被限幅成 30°（超调）。
 *   - 电机刚停时车体还在晃，必须等角度稳定再测，否则会反复过冲。
 * ========================================================================== */
#define TURN_TOL_CENTI       10L    /* 到位容差 0.1°（单位 0.01°） */
#define TURN_MAX_CORR        15u    /* 补差次数上限（误差按 1-k 衰减，多给几次能兜住标定差） */
#define TURN_SETTLE_MS      100u    /* 等停稳的采样间隔 */
#define TURN_SETTLE_TOL      50L    /* 相邻两次采样差 < 0.5° 认为停稳 */
#define TURN_SETTLE_TRIES     5u    /* 最多等 5 轮（0.5 s） */
#define MOVE_TIMEOUT_ROUNDS 500u    /* 等电机到位的轮数上限（约 11~110 s） */

static u8 gyro_ok;                  /* 收到过有效角度帧 */

/* 开机零点：所有绝对角度都相对它 */
static int32_t yaw_zero;
static u8      yaw_zero_ok;

/* 最近一次闭环转向的过程量（测试模式直接显示，路线模式用来复盘） */
static volatile int32_t turn_dbg_target;     /* 目标绝对角度，0.01° */
static volatile int32_t turn_dbg_meas;       /* 当前绝对角度，0.01° */
static volatile int32_t turn_dbg_err;        /* 最后误差，0.01° */
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


/* 记开机零点：之后所有朝向都用相对它的绝对角度。最多等 1 s 等陀螺仪出数。 */
static u8 yaw_zero_capture(void)
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

/* 把 0.01° 显示成 ±ddd.cc（占 7 列），两个模式共用 */
static void turn_show_2dp(uint8_t line, uint8_t col, int32_t centi)
{
    int32_t v = (centi < 0L) ? -centi : centi;

    OLED_ShowChar(line, col, (centi < 0L) ? '-' : '+');
    OLED_ShowNum(line, (uint8_t)(col + 1u), (uint32_t)(v / 100L), 3);
    OLED_ShowChar(line, (uint8_t)(col + 4u), '.');
    OLED_ShowNum(line, (uint8_t)(col + 5u), (uint32_t)(v % 100L), 2);
}

/* 转到绝对角度 target_abs（0.01°，相对开机零点），返回 1 到位 / 0 失败。
 * 每次补差都按"当前误差 → 脉冲"一次到位，过程量写进 turn_dbg_*。 */
static u8 gyro_turn_to_abs(int32_t target_abs)
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
 *   DEBUG_MODE_CHASSIS 5 = 速度模式底盘自检（chassis.c，F6 + 定时停车）
 *   DEBUG_MODE_WIRELESS 6 = 无线串口底盘测试（USART3 + 上位机 tools/chassis_host.py）
 * ========================================================================== */
#define DEBUG_MODE_SERVO    0
#define DEBUG_MODE_CIRCLE   1
#define DEBUG_MODE_MOTOR    2
#define DEBUG_MODE_ROUTE    3
#define DEBUG_MODE_TURN     4
#define DEBUG_MODE_CHASSIS  5
#define DEBUG_MODE_WIRELESS 6

#define DEBUG_MODE          DEBUG_MODE_WIRELESS

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

/* 朝向编号(1北 2西 3南 4东) → 相对开机零点的绝对角度（0.01°，逆时针为正） */
static int32_t heading_abs(u8 d)
{
    switch(d){
        case 2u:  return  9000L;    /* 西 = +90° */
        case 3u:  return 18000L;    /* 南 = 180° */
        case 4u:  return -9000L;    /* 东 = -90° */
        default:  return     0L;    /* 北 = 0° */
    }
}
/* 把车头对准 req 朝向：同向直行、相反倒车、±90° 走陀螺仪闭环转 */
/* 把车头对准 req 朝向：同向直行、相反倒车、±90° 转到该朝向的绝对角度 */
static u8 leg_face(u8 req)
{
    u8 t = heading_turn(dir, req);

    if(t == 0u){ seg_backward = 0u; return 1u; }
    if(t == 2u){ seg_backward = 1u; return 1u; }   /* 180°：不转，倒着走 */

    if(!yaw_zero_ok){
        /* 没有陀螺仪：退化为开环 90°，保证路线还能跑完 */
        if(t == 1u) ni_yaw_(90u);
        else        shun_yaw_(90u);
        Delay_ms(1000);
        if(!wait_motors()) return 0u;
        dir = req;
        seg_backward = 0u;
        return 1u;
    }

    /* ±90°：转到"该朝向对应的绝对角度"，闭环 */
    if(!gyro_turn_to_abs(gyro_wrap_180(yaw_zero + heading_abs(req)))) return 0u;

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

    /* 记开机零点：把摆好的车头方向当成"北"，之后所有转向都用绝对角度 */
    if(!yaw_zero_capture()){
        OLED_ShowString(1, 1, "NO GYRO");
        Delay_ms(800);
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
            /* 结束画面留最后一次转向的补差次数和残留误差，方便复盘 */
            OLED_ShowString(1, 1, "PATH DONE");
            OLED_ShowString(3, 1, "C");
            OLED_ShowNum(3, 2, turn_dbg_corr, 1);
            OLED_ShowString(3, 4, "E");
            turn_show_2dp(3, 5, turn_dbg_err);
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
 * 闭环旋转单独测试（不跑路线，用绝对角度）
 * ----------------------------------------------------------------------------
 * 直接调用路线里同一个 gyro_turn_to_abs()，把闭环转向单独验一遍。
 *
 *   PA12 = 目标角度 +90°（逆时针一档）后转过去
 *   PA11 = 目标角度 -90°（顺时针一档）后转过去
 *   PA8  = 不改目标，原地重新对准当前目标（检验绝对闭环：用手扳歪再按一下就回正）
 *
 * 目标角度是"绝对角度"：开机时把当前车头方向记为 0°，之后 北=0 西=+90 南=180 东=-90。
 *
 * 每次转完把过程量留在屏幕上：
 *   1 行  ABS 绝对目标角  C 补差次数
 *   2 行  左半是 judge 查询计数，右侧是当前绝对角度 / NO GYRO
 *   3 行  MEA 当前绝对角度
 *   4 行  ERR 最终误差 + OK / FAIL
 * ========================================================================== */

/* 把最近一次转向的过程量画出来 */
static void turn_test_show(int32_t target)
{
    if(target >= 0L){
        OLED_ShowString(1, 1, "ABS+");
        OLED_ShowNum(1, 5, (uint32_t)(target / 100L), 3);
    }else{
        OLED_ShowString(1, 1, "ABS-");
        OLED_ShowNum(1, 5, (uint32_t)(-target / 100L), 3);
    }
    OLED_ShowString(1, 9, " C");
    OLED_ShowNum(1, 11, turn_dbg_corr, 2);

    OLED_ShowString(3, 1, "MEA");
    turn_show_2dp(3, 5, turn_dbg_meas);

    OLED_ShowString(4, 1, "ERR");
    turn_show_2dp(4, 5, turn_dbg_err);
    OLED_ShowString(4, 12, turn_dbg_result ? " OK " : "FAIL");
}

static void turn_test_run(void)
{
    u8 key;
    int32_t target = 0L;          /* 绝对目标角，0.01°；开机即为 0（对准零点） */
    int32_t yaw_now;
    GPIO_InitTypeDef gpio;

    /* PA8：重新对准 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_8;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    OLED_Clear();
    OLED_ShowString(1, 1, "TURN TEST");
    OLED_ShowString(3, 1, "A12=+90 A11=-90");
    OLED_ShowString(4, 1, "PA8=REALIGN");

    /* 记开机零点：之后所有角度都是相对它的绝对角度 */
    if(!yaw_zero_capture()){
        OLED_ShowString(1, 1, "NO GYRO");
    }
    Delay_ms(600);
    turn_test_show(target);

    while(1){
        /* 第 2 行右侧：当前绝对角度（judge 只用 1~5 列） */
        if(Gyro_GetYawCentiDegrees(&yaw_now)){
            gyro_ok = 1u;
            if(yaw_zero_ok){
                OLED_ShowString(2, 7, "Y");
                OLED_ShowSignedNum(2, 8, gyro_wrap_180(yaw_now - yaw_zero) / 100L, 3);
            }
        }else if(!gyro_ok){
            OLED_ShowString(2, 7, "NO GYRO");
        }

        /* PA8：不改目标，重新对准当前目标（绝对闭环验证） */
        if(pa8_hit()){
            OLED_ShowString(4, 1, "REALIGN...     ");
            (void)gyro_turn_to_abs(target);
            turn_test_show(target);
        }

        key = Key_GetNum();
        if(key != 4 && key != 5){
            Delay_ms(10);
            continue;
        }

        /* PA12 加一档 +90°，PA11 减一档 -90°，然后转到这个绝对角度 */
        if(key == 4) target = gyro_wrap_180(target + 9000L);
        else         target = gyro_wrap_180(target - 9000L);

        OLED_ShowString(4, 1, "TURNING...     ");
        (void)gyro_turn_to_abs(target);            /* 过程量写进 turn_dbg_* */
        turn_test_show(target);
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

#elif DEBUG_MODE == DEBUG_MODE_CHASSIS

/* ============================================================================
 * 速度模式底盘自检（F6 速度模式 + 定时停车）
 * ----------------------------------------------------------------------------
 * 这个模式只测 chassis.c 的新底盘逻辑，不碰 motor.c 的 FD 位置模式。
 * 按键和电机自检页一致（这块板子只有 3 个键可用）：
 *   PA12 = 本页第 1 个动作   PA11 = 本页第 2 个动作   PA8 = 翻页
 * 运动中按 PA8 = 立即停车（走 chassis 的中止回调）。
 *
 *   第 1 页  前进 / 后退 100 mm
 *   第 2 页  左平移 / 右平移 100 mm
 *   第 3 页  逆时针 / 顺时针 90°
 *   第 4 页  前进 / 后退 1000 mm（拿卷尺量：走多了调小
 *            chassis_config.h 的 CHASSIS_TIME_GAIN_PCT，走少了调大）
 *   第 5 页  前进 500 mm：慢速 / 快速（验证 Chassis_MoveAt 的速度覆盖）
 * ========================================================================== */
#define CHASSIS_TEST_PAGES   5u

/* 运动中按下 PA8 就停车 */
static u8 chassis_test_abort(void)
{
    return (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET) ? 1u : 0u;
}

/* 每页的静态提示（每行最多 16 列）；第 4 行固定显示 PA8 翻页 */
static void chassis_check_draw(u8 page)
{
    OLED_Clear();

    switch(page){
        case 1:
            OLED_ShowString(1, 1, "P1/5 FWD 100MM");
            OLED_ShowString(2, 1, "A12 = FORWARD");
            OLED_ShowString(3, 1, "A11 = BACK");
            break;
        case 2:
            OLED_ShowString(1, 1, "P2/5 STRAFE");
            OLED_ShowString(2, 1, "A12 = LEFT 100");
            OLED_ShowString(3, 1, "A11 = RIGHT 100");
            break;
        case 3:
            OLED_ShowString(1, 1, "P3/5 ROTATE 90");
            OLED_ShowString(2, 1, "A12 = CCW 90");
            OLED_ShowString(3, 1, "A11 = CW 90");
            break;
        case 4:
            OLED_ShowString(1, 1, "P4/5 CAL 1000MM");
            OLED_ShowString(2, 1, "A12 = FORWARD");
            OLED_ShowString(3, 1, "A11 = BACK");
            break;
        default:
            OLED_ShowString(1, 1, "P5/5 SPEED TEST");
            OLED_ShowString(2, 1, "A12 = SLOW 500");
            OLED_ShowString(3, 1, "A11 = FAST 500");
            break;
    }

    OLED_ShowString(4, 1, "PA8>NEXT PAGE");
}

/* 按当前页 + 按键执行一个动作（A12=key4 走正向，A11=key5 走反向） */
static void chassis_check_run(u8 page, u8 key)
{
    u8 ok = 0;

    if(page == 1)       ok = (key == 4) ? Chassis_Move(100)   : Chassis_Move(-100);
    else if(page == 2)  ok = (key == 4) ? Chassis_Strafe(100) : Chassis_Strafe(-100);
    else if(page == 3)  ok = (key == 4) ? Chassis_Rotate(90)  : Chassis_Rotate(-90);
    else if(page == 4)  ok = (key == 4) ? Chassis_Move(1000)  : Chassis_Move(-1000);
    else                ok = (key == 4) ? Chassis_MoveAt(500, 150)
                                        : Chassis_MoveAt(500, 700);

    OLED_ShowString(4, 1, ok ? "OK            " : "ABORT         ");
    Delay_ms(500);
    chassis_check_draw(page);
}

/* 底盘自检主循环：不返回 */
static void chassis_check_test(void)
{
    u8 page = 1u;
    u8 key;
    GPIO_InitTypeDef gpio;

    /* PA8 当翻页键：上拉输入，按下接地 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_8;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    Chassis_Init();                                   /* 使能 + 默认加速度 */
    Chassis_SetAbortCheck(chassis_test_abort);

    OLED_ShowString(1, 1, "CHASSIS TEST");
    chassis_check_draw(page);

    while(1){
        /* PA8：翻页（带消抖和松手等待） */
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET){
            Delay_ms(20);
            if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET){
                page = (u8)((page % CHASSIS_TEST_PAGES) + 1u);
                chassis_check_draw(page);
                while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET) Delay_ms(10);
                Delay_ms(20);
            }
        }

        /* 只用 PA12(key 4) 和 PA11(key 5)，其余键码忽略 */
        key = Key_GetNum();
        if(key != 4 && key != 5){
            Delay_ms(10);
            continue;
        }

        OLED_ShowString(4, 1, "RUN...        ");
        chassis_check_run(page, key);
    }
}

#elif DEBUG_MODE == DEBUG_MODE_WIRELESS

/* ============================================================================
 * 无线串口底盘测试（USART3 / PB10=TX、PB11=RX，115200 8N1）
 * ----------------------------------------------------------------------------
 * 无线串口模块透传，电脑端用 tools/chassis_host.py 或任意串口助手发一行命令：
 *
 *   F 1000   前进 1000 ms          B 1000   后退 1000 ms
 *   L 1000   左平移 1000 ms        R 1000   右平移 1000 ms
 *   Q 1000   逆时针 1000 ms        E 1000   顺时针 1000 ms
 *   S        立即停车              D        内置测试：
 *                                           前进 1s → 逆时针 1s → 后退 1s → 顺时针 1s
 *   ?        查状态（回 YAW）      P        ping
 *   V 300    设默认速度（0.1RPM）  H        帮助
 *
 * 数字可以不写（默认 1000 ms），命令不区分大小写，F1000 / F 1000 都行。
 * 回包都是 ASCII 一行：READY / ACK <命令> <ms> / DONE <命令> / ABORT <命令>
 *                     / YAW <0.01 度> / PONG / SPEED <0.1RPM> / ERR ...
 *
 * 本地按键：PA8 = 空闲时跑内置测试，运动中 = 立即停车。
 * 注意：USART3 的 PB11 和按键 2 复用，本模式不调用 Key_GetNum()。
 * 上电默认不自动跑（安全），想上电就跑一次把 WIRELESS_AUTO_DEMO 改成 1。
 * ========================================================================== */
#define WIRELESS_DEFAULT_MS     1000u
#define WIRELESS_MIN_MS          100u
#define WIRELESS_MAX_MS        10000u
#define WIRELESS_AUTO_DEMO         0

static u16 g_wireless_speed_x10 = 0u;    /* 0 = 用 chassis_config.h 的默认速度 */
static volatile u8 g_wireless_abort;     /* 运动中按 PA8 置 1 */

/* 运动中按 PA8：置中止标志，Chassis_RunMs() 会提前停车 */
static u8 wireless_abort(void)
{
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET){
        g_wireless_abort = 1u;
        return 1u;
    }
    return 0u;
}

/* 等 PA8 松开并清中止标志，避免松手后被当成"开始测试" */
static void wireless_wait_pa8_release(void)
{
    while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET) Delay_ms(10);
    Delay_ms(20);
    g_wireless_abort = 0u;
}

/* 空闲时 PA8 短按：消抖 + 等松手，算一次 */
static u8 wireless_pa8_hit(void)
{
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) != Bit_RESET) return 0u;
    Delay_ms(20);
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) != Bit_RESET) return 0u;
    wireless_wait_pa8_release();
    return 1u;
}

static void wireless_show_idle(void)
{
    OLED_ShowString(1, 1, "WIRELESS");
    OLED_ShowString(2, 1, "WAIT CMD");
    OLED_ShowString(3, 1, "USART3 115200");
    OLED_ShowString(4, 1, "PA8=DEMO");
}

/* 回一行当前陀螺仪角度（0.01 度，没数据回 NA） */
static void wireless_send_yaw(void)
{
    int32_t yaw;

    if(Gyro_GetYawCentiDegrees(&yaw))
        USART3_Printf("YAW %d\r\n", (int)yaw);
    else
        USART3_SendString("YAW NA\r\n");
}

/* 解析命令后面的数字：允许 "F1000" 或 "F 1000"，返回 1 表示有数字 */
static u8 wireless_parse_num(const char *s, uint32_t *out)
{
    uint32_t v = 0u;
    u8 digits = 0u;

    while(*s == ' ' || *s == '\t') s++;
    while(*s >= '0' && *s <= '9'){
        if(v <= 100000u) v = v * 10u + (uint32_t)(*s - '0');
        s++;
        digits = 1u;
    }

    *out = v;
    return digits;
}

/* 命令没带数字就用默认 1000 ms，并限幅到 100~10000 ms */
static uint32_t wireless_ms(uint32_t arg, u8 has_arg)
{
    uint32_t ms = has_arg ? arg : WIRELESS_DEFAULT_MS;

    if(ms < WIRELESS_MIN_MS) ms = WIRELESS_MIN_MS;
    if(ms > WIRELESS_MAX_MS) ms = WIRELESS_MAX_MS;
    return ms;
}

/* 执行一个定时动作并回 ACK/DONE/ABORT，最后回一次 YAW */
static void wireless_do_action(ChassisAction action, char cmd, uint32_t ms)
{
    u8 ok;

    USART3_Printf("ACK %c %u\r\n", cmd, (unsigned)ms);

    OLED_ShowString(3, 1, "RUN ");
    OLED_ShowChar(3, 5, cmd);
    OLED_ShowNum(3, 7, ms, 5);

    ok = Chassis_RunMs(action, ms, g_wireless_speed_x10);

    if(!ok){
        wireless_wait_pa8_release();
        USART3_Printf("ABORT %c\r\n", cmd);
        OLED_ShowString(2, 1, "ABORT   ");
    }else{
        USART3_Printf("DONE %c %u\r\n", cmd, (unsigned)ms);
        OLED_ShowString(2, 1, "DONE    ");
    }

    wireless_send_yaw();
}

/* 内置测试：前进 1s → 逆时针 1s → 后退 1s → 顺时针 1s */
static void wireless_demo(void)
{
    USART3_SendString("ACK D\r\n");
    OLED_ShowString(2, 1, "DEMO...");

    if(!Chassis_RunMs(CHASSIS_ACTION_FORWARD,  1000u, g_wireless_speed_x10)) goto aborted;
    if(!Chassis_RunMs(CHASSIS_ACTION_CCW,      1000u, g_wireless_speed_x10)) goto aborted;
    if(!Chassis_RunMs(CHASSIS_ACTION_BACKWARD, 1000u, g_wireless_speed_x10)) goto aborted;
    if(!Chassis_RunMs(CHASSIS_ACTION_CW,       1000u, g_wireless_speed_x10)) goto aborted;

    USART3_SendString("DONE D\r\n");
    OLED_ShowString(2, 1, "DEMO DONE");
    wireless_send_yaw();
    return;

aborted:
    wireless_wait_pa8_release();
    USART3_SendString("ABORT D\r\n");
    OLED_ShowString(2, 1, "ABORT   ");
}

/* 处理一行命令（不区分大小写） */
static void wireless_handle(char *line)
{
    char     cmd;
    uint32_t arg = 0u;
    u8       has_arg;
    u8       i = 0u;

    while(line[i] == ' ' || line[i] == '\t') i++;
    if(line[i] == '\0') return;

    cmd = line[i];
    if(cmd >= 'a' && cmd <= 'z') cmd = (char)(cmd - 'a' + 'A');
    has_arg = wireless_parse_num(&line[i + 1u], &arg);

    switch(cmd){
        case 'F': wireless_do_action(CHASSIS_ACTION_FORWARD,  cmd, wireless_ms(arg, has_arg)); break;
        case 'B': wireless_do_action(CHASSIS_ACTION_BACKWARD, cmd, wireless_ms(arg, has_arg)); break;
        case 'L': wireless_do_action(CHASSIS_ACTION_LEFT,     cmd, wireless_ms(arg, has_arg)); break;
        case 'R': wireless_do_action(CHASSIS_ACTION_RIGHT,    cmd, wireless_ms(arg, has_arg)); break;
        case 'Q': wireless_do_action(CHASSIS_ACTION_CCW,      cmd, wireless_ms(arg, has_arg)); break;
        case 'E': wireless_do_action(CHASSIS_ACTION_CW,       cmd, wireless_ms(arg, has_arg)); break;

        case 'S':
            Chassis_Stop();
            USART3_SendString("STOP\r\n");
            OLED_ShowString(2, 1, "STOP    ");
            break;

        case 'D':
            wireless_demo();
            break;

        case '?':
            wireless_send_yaw();
            break;

        case 'P':
            USART3_SendString("PONG\r\n");
            break;

        case 'V':
            if(!has_arg){
                USART3_SendString("ERR ARG\r\n");
                break;
            }
            if(arg > CHASSIS_SPEED_MAX) arg = CHASSIS_SPEED_MAX;
            g_wireless_speed_x10 = (u16)arg;
            USART3_Printf("SPEED %u\r\n", (unsigned)arg);
            break;

        case 'H':
            USART3_SendString("CMDS F B L R Q E <ms> | S | D | ? | P | V <0.1RPM>\r\n");
            break;

        default:
            USART3_SendString("ERR CMD\r\n");
            break;
    }
}

/* 无线测试主循环：不返回 */
static void wireless_test_run(void)
{
    char line[64];
    GPIO_InitTypeDef gpio;

    /* PA8：本地开始测试 / 运动中停车 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_8;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    Chassis_Init();                       /* 使能 4 个电机 */
    Chassis_SetAbortCheck(wireless_abort);

    OLED_Clear();
    wireless_show_idle();

    USART3_SendString("\r\nREADY CHASSIS WIRELESS 115200\r\n");
    USART3_SendString("CMDS F B L R Q E <ms> | S | D | ? | P | V <0.1RPM>\r\n");

#if WIRELESS_AUTO_DEMO
    Delay_ms(2000);
    wireless_demo();
#endif

    while(1){
        if(USART3_ReadLine(line, (uint8_t)sizeof(line))){
            wireless_handle(line);
            wireless_show_idle();
            continue;
        }

        if(!g_wireless_abort && wireless_pa8_hit()){
            wireless_demo();
            wireless_show_idle();
            continue;
        }

        Delay_ms(5);
    }
}

#else

/* 舵机角度按键标定：PA12 每按一次 +1°，PA11 每按一次 -1°，限幅 0~270°。
 * 只动旋转舵机（PC9）；三路一起调请用 grab_run_once()。 */
static void servo_angle_test(void)
{
    uint8_t key;
    int servo_angle = (int)SERVO_ROT_INIT_DEG;   /* 上电 240° */

    Servo_SetAngle(SERVO_ROT, (uint16_t)servo_angle);
    OLED_ShowString(1, 1, "ANGLE:");
    OLED_ShowNum(1, 7, (uint32_t)servo_angle, 3);
    OLED_ShowString(2, 1, "PA12:+1 PA11:-1");

    while(1){
        key = Key_GetNum();
        if(key == 4){                                       /* PA12：+1° */
            if(servo_angle < (int)SERVO_ANGLE_MAX) servo_angle++;
        }else if(key == 5){                                 /* PA11：-1° */
            if(servo_angle > (int)SERVO_ANGLE_MIN) servo_angle--;
        }else{
            Delay_ms(10);
            continue;
        }

        Servo_SetAngle(SERVO_ROT, (uint16_t)servo_angle);
        OLED_ShowNum(1, 7, (uint32_t)servo_angle, 3);
    }
}

#endif /* DEBUG_MODE */

/* ============================================================================
 * 物料电机（地址 5）· 分度转动测试
 * ----------------------------------------------------------------------------
 * PA12：转 MATERIAL_DEG_PER_STEP 度 → 等驱动器到位 → 立即停止 → OLED 显示脉冲数
 * PA11：运动过程中随时立即停止（转动中随时拍停）
 *
 * 标定结果：3200 脉冲 = 物料轴一整圈（已实测确认，按 PA12 转一圈正好）。
 * 所以任意角度按下面公式换算：脉冲 = 3200 × 角度 / 360，60° → 533 脉冲。
 * 想改成别的角度，只动 MATERIAL_DEG_PER_STEP（≤ 999 保证 OLED 不串行）：
 *   90° → 800 脉冲，180° → 1600，360° → 3200（= 一整圈）。
 * 注意 533 是四舍五入后的整数，连按 6 次共 3198 脉冲，比整圈少 2 个（约 0.2°）。
 *
 * OLED：第 1 行角度+地址，第 2 行本次脉冲，第 3 行上电以来累计脉冲，第 4 行状态。
 * ========================================================================== */
#define MATERIAL_MOTOR_ADDR      0x05u   /* 物料电机驱动器地址 */
#define MATERIAL_MOTOR_DIR       0x00u   /* 0x00 正转 / 0x01 反转（转反了就改这里） */
#define MATERIAL_MOTOR_SPEED     100u    /* 速度 RPM：0~3000，越大越快 */
#define MATERIAL_MOTOR_ACCEL     200u    /* 加速度档 0~255（0 = 不用曲线加减速） */
#define MATERIAL_PULSES_PER_REV  3200u   /* 实测值：物料轴转一整圈的脉冲数 */
#define MATERIAL_DEG_PER_STEP    60u     /* ★ 每次按 PA12 转多少度 */
#define MATERIAL_DEG_FULL        360u    /* 一整圈的角度，只用来做换算 */

/* 标定界面第 4 行的状态 */
#define MAT_CAL_IDLE       0u   /* 等按键 */
#define MAT_CAL_MOVING     1u   /* 正在转 */
#define MAT_CAL_DONE       2u   /* 驱动器回到位、已停 */
#define MAT_CAL_NO_ANSWER  3u   /* 0x3A 没回包：驱动器不在线或关了应答 */
#define MAT_CAL_ABORT      4u   /* PA11 中途叫停 */

#define MAT_CAL_ROUND_MS   20u  /* 一次"查询 + 等待"大约 20 ms，用来算超时轮数 */

static uint32_t material_total_pulses = 0;   /* 上电以来累计发出的脉冲数 */

/* 角度 → 脉冲：3200 脉冲 = 360°，四舍五入（60° → 533，360° → 3200）。 */
static uint32_t material_deg_to_pulses(uint32_t deg)
{
    return (MATERIAL_PULSES_PER_REV * deg + MATERIAL_DEG_FULL / 2u) / MATERIAL_DEG_FULL;
}

/* 给物料电机发一帧位置命令：相对位置、立即执行（单电机不需要同步释放帧）。 */
static void material_motor_move(uint32_t pulses, uint8_t dir)
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

/* 立即停止：[地址][0xFE][0x98][同步][0x6B]（Emm42 的"立即停止"命令）。 */
static void material_motor_stop(void)
{
    Serial_SendByte(MATERIAL_MOTOR_ADDR);
    Serial_SendByte(0xFE);
    Serial_SendByte(0x98);
    Serial_SendByte(0x00);
    Serial_SendByte(0x6B);
    Delay_ms(5);
}

/* 这一下要走多久 → 超时轮数：脉冲 ÷ (3200 脉冲/圈 × 速度RPM) 换算成毫秒，
 * 再加 2 s 余量。3200 是驱动器细分决定的"电机一圈"，和上面按角度换算的脉冲无关，
 * 所以角度/脉冲填大了，这里的超时也会跟着变长。 */
static uint16_t material_calib_rounds(uint32_t pulses)
{
    uint32_t ms, rounds;

    ms = (pulses / 4u) * 75u / MATERIAL_MOTOR_SPEED + 2000u;   /* 60000/3200 = 18.75 = 75/4 */
    rounds = ms / MAT_CAL_ROUND_MS;

    if(rounds < 100u)  rounds = 100u;    /* 至少 2 s，别刚发完就判超时 */
    if(rounds > 3000u) rounds = 3000u;   /* 最多 60 s */

    return (uint16_t)rounds;
}

/* 轮询 0x3A 到位状态：回包 [地址][0x3A][状态][0x6B]，状态 bit1 = 已到位。
 * 返回 MAT_CAL_DONE / MAT_CAL_NO_ANSWER / MAT_CAL_ABORT。 */
static uint8_t material_motor_wait_arrive(uint16_t max_rounds)
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

static void material_calib_clear_line(uint8_t line)
{
    OLED_ShowString(line, 1, "                ");   /* 16 个空格，擦掉旧数字的尾巴 */
}

static uint8_t oled_digits(uint32_t v)
{
    uint8_t n = 1u;

    while(v >= 10u){ v /= 10u; n++; }
    return n;
}

/* 画标定界面：第 1 行角度，第 2 行本次脉冲，第 3 行累计脉冲，第 4 行状态。
 * 角度最多 3 位（≤999°）才能和 "DEG ADDR5" 拼在 16 列内。 */
static void material_calib_draw(uint8_t state)
{
    uint8_t  n;
    uint32_t step = material_deg_to_pulses(MATERIAL_DEG_PER_STEP);

    n = oled_digits(MATERIAL_DEG_PER_STEP);
    OLED_ShowString(1, 1, "MAT ");
    OLED_ShowNum(1, 5, MATERIAL_DEG_PER_STEP, n);
    OLED_ShowString(1, (uint8_t)(5u + n), "DEG ADDR5");

    /* 标签 5 列 + 数字最多 10 列 + "P" = 16 列，正好不出屏 */
    material_calib_clear_line(2);
    OLED_ShowString(2, 1, "STEP:");
    OLED_ShowNum(2, 6, step, oled_digits(step));
    OLED_ShowString(2, (uint8_t)(6u + oled_digits(step)), "P");

    material_calib_clear_line(3);
    OLED_ShowString(3, 1, "TOTAL:");
    OLED_ShowNum(3, 7, material_total_pulses, oled_digits(material_total_pulses));

    switch(state){
        case MAT_CAL_MOVING:    OLED_ShowString(4, 1, "MOVING...       "); break;
        case MAT_CAL_DONE:      OLED_ShowString(4, 1, "OK STOPPED      "); break;
        case MAT_CAL_NO_ANSWER: OLED_ShowString(4, 1, "NO 3A ANSWER    "); break;
        case MAT_CAL_ABORT:     OLED_ShowString(4, 1, "STOP BY PA11    "); break;
        default:                OLED_ShowString(4, 1, "PA12: TURN STEP "); break;
    }
}

/* 分度转动主循环：不返回。 */
static void material_calib_test(void)
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

/* ============================================================================
 * 270° 舵机（PC9 = TIM3_CH4）按键标定
 * ----------------------------------------------------------------------------
 * PA12：角度 +SERVO_STEP_DEG      PA11：角度 -SERVO_STEP_DEG
 * 越界时（已在 270° 还按 +，或已在 0° 还按 -）什么都不做：
 *   不发新的 CCR、也不重画 OLED —— 屏幕上的角度/脉宽保持上一次的数值。
 * 脉宽映射见 Servo.h：0° → 500 us，270° → 2500 us，20 ms 周期（50 Hz）。
 * OLED 第 2 行是目标角度，第 3 行是实际写进 CCR 的脉宽（和硬件输出一致）。
 * ========================================================================== */
#define SERVO_STEP_DEG   10u

/* 画界面：angle 是当前目标角度，脉宽直接取 Servo_GetPulse(SERVO_ROT)，避免两处各算一遍。 */
static void servo270_draw(uint16_t angle)
{
    uint16_t pulse = Servo_GetPulse(SERVO_ROT);
    uint8_t  n;

    OLED_ShowString(1, 1, "SERVO PC9  270D");

    /* 第 2 行角度：先擦整行再画，数字位数变化不会留尾巴 */
    OLED_ShowString(2, 1, "                ");
    OLED_ShowString(2, 1, "ANG:");
    n = oled_digits(angle);
    OLED_ShowNum(2, 5, angle, n);
    OLED_ShowString(2, (uint8_t)(5u + n), "D");

    /* 第 3 行脉宽 */
    OLED_ShowString(3, 1, "                ");
    OLED_ShowString(3, 1, "PULSE:");
    n = oled_digits(pulse);
    OLED_ShowNum(3, 7, pulse, n);
    OLED_ShowString(3, (uint8_t)(7u + n), "us");

    OLED_ShowString(4, 1, "A12:+10 A11:-10 ");
}

/* 270° 舵机按键标定主循环：不返回。 */
static void servo270_test(void)
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

/* ============================================================================
 * 夹取物料调试动作（升降舵机 PC7 / 夹爪舵机 PC8 / 旋转舵机 PC9）
 * ----------------------------------------------------------------------------
 * 上电安全位：旋转 240°、升降 90°、夹爪 65°（Servo_Init 直接输出，见 Servo.h）。
 * PA8 按一次 = 跑完整套动作；跑完停在最后一步，不自动回初始位。
 *   1) 升降 90° → 70°    贴近地面
 *   2) 夹爪 65° → 110°   夹起物料
 *   3) 升降 70° → 170°   抬高物料
 *   4) 旋转 240° → 80°   转到车上
 *   5) 夹爪 110° → 60°   放开物料
 * PA11：动作途中按下 = 立即中止，后续步骤不再执行，停在当前姿势。
 * 每步之后等 GRAB_SETTLE_MS 让舵机转到位；等待期间 PA11 每 1 ms 查一次，照样能打断。
 * 地址 5（物料盘电机）和地址 6（升降步进电机）本动作完全不碰。
 * ========================================================================== */
#define GRAB_SETTLE_MS   800u

typedef struct {
    ServoId  id;          /* 哪个舵机 */
    uint16_t deg;         /* 目标角度 */
    char     label[8];    /* OLED / 串口显示用，最多 7 个字符 */
} GrabStep;

static const GrabStep k_grab_steps[] = {
    { SERVO_LIFT,  70u, "LIFT70"  },
    { SERVO_GRIP, 110u, "GRIP110" },
    { SERVO_LIFT, 170u, "LIFT170" },
    { SERVO_ROT,   80u, "ROT80"   },
    { SERVO_GRIP,  60u, "GRIP60"  }
};

#define GRAB_STEP_COUNT   ((uint8_t)(sizeof(k_grab_steps) / sizeof(k_grab_steps[0])))

/* 界面状态（第 4 行） */
#define GRAB_READY   0u
#define GRAB_BUSY    1u
#define GRAB_DONE    2u
#define GRAB_ABORT   3u

/* PA8 当启动键：上拉输入，按下接地。Key_GetNum() 只扫 PB1/PB11/PB10/PA12/PA11，
 * 所以 PA8 在这里单独初始化、单独读。 */
static void grab_key_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_8;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
}

/* PA8 按了一下：20 ms 消抖 + 等松手，有按下返回 1 */
static uint8_t grab_start_hit(void)
{
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) != Bit_RESET) return 0;

    Delay_ms(20);
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) != Bit_RESET) return 0;

    while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET) Delay_ms(10);
    Delay_ms(20);
    return 1;
}

/* PA11 中止键：按下后等松手再返回 1，避免一直按着被重复判定 */
static uint8_t grab_abort_hit(void)
{
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) != Bit_RESET) return 0;

    while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == Bit_RESET) Delay_ms(5);
    return 1;
}

/* 等 ms 毫秒；期间 PA11 按下立刻返回 0（中止本次动作） */
static uint8_t grab_wait(uint16_t ms)
{
    uint16_t t;

    for(t = 0; t < ms; t++){
        if(grab_abort_hit()) return 0;
        Delay_ms(1);
    }
    return 1;
}

/* 画界面：step = 已完成/正在做的步数（0 = 还没开始），state 决定第 4 行。
 * 第 2、3 行显示的是三个舵机的实际角度（含微调），不是目标角度。 */
static void grab_show(uint8_t step, uint8_t state)
{
    /* 第 1 行：GRAB 2/5 GRIP110 */
    OLED_ShowString(1, 1, "                ");
    OLED_ShowString(1, 1, "GRAB ");
    OLED_ShowNum(1, 6, step, 1);
    OLED_ShowString(1, 7, "/5");
    if(step > 0u){
        OLED_ShowString(1, 10, (char *)k_grab_steps[step - 1u].label);
    }

    /* 第 2 行：ROT:240 LFT:170 （角度最多 3 位，列位置固定，不会串行） */
    OLED_ShowString(2, 1, "                ");
    OLED_ShowString(2, 1, "ROT:");
    OLED_ShowNum(2, 5, Servo_GetAngle(SERVO_ROT), oled_digits(Servo_GetAngle(SERVO_ROT)));
    OLED_ShowString(2, 9, "LFT:");
    OLED_ShowNum(2, 13, Servo_GetAngle(SERVO_LIFT), oled_digits(Servo_GetAngle(SERVO_LIFT)));

    /* 第 3 行：GRP:65 */
    OLED_ShowString(3, 1, "                ");
    OLED_ShowString(3, 1, "GRP:");
    OLED_ShowNum(3, 5, Servo_GetAngle(SERVO_GRIP), oled_digits(Servo_GetAngle(SERVO_GRIP)));

    switch(state){
        case GRAB_BUSY:  OLED_ShowString(4, 1, "BUSY...         "); break;
        case GRAB_DONE:  OLED_ShowString(4, 1, "DONE            "); break;
        case GRAB_ABORT: OLED_ShowString(4, 1, "ABORT BY PA11   "); break;
        default:         OLED_ShowString(4, 1, "PA8:START       "); break;
    }
}

/* 夹取动作主循环：不返回。 */
static void grab_run_once(void)
{
    uint8_t i, step, ok;

    /* 上电安全位（Servo_Init 里已经输出过，这里再确认一次，重进本函数时也一致） */
    Servo_SetAngle(SERVO_ROT,  SERVO_ROT_INIT_DEG);
    Servo_SetAngle(SERVO_LIFT, SERVO_LIFT_INIT_DEG);
    Servo_SetAngle(SERVO_GRIP, SERVO_GRIP_INIT_DEG);

    grab_key_init();

    OLED_Clear();
    grab_show(0u, GRAB_READY);
    USART3_SendString("\r\nGRAB READY: PA8=START PA11=ABORT\r\n");

    while(1){
        if(!grab_start_hit()){
            Delay_ms(5);
            continue;
        }

        USART3_SendString("[RUN] START\r\n");
        step = 0u;
        ok   = 1u;

        for(i = 0; i < GRAB_STEP_COUNT; i++){
            step = (uint8_t)(i + 1u);

            Servo_SetAngle(k_grab_steps[i].id, k_grab_steps[i].deg);
            grab_show(step, GRAB_BUSY);
            USART3_Printf("[S%u] %s -> %u deg\r\n",
                          (unsigned)step, k_grab_steps[i].label,
                          (unsigned)k_grab_steps[i].deg);

            if(!grab_wait(GRAB_SETTLE_MS)){
                ok = 0u;                 /* PA11 中止：停在当前姿势 */
                break;
            }
        }

        grab_show(step, ok ? GRAB_DONE : GRAB_ABORT);
        USART3_SendString(ok ? "[RUN] DONE\r\n" : "[RUN] ABORT BY PA11\r\n");

        /* 收尾：等 PA8 松手。否则"一直按住 PA8"会在动作结束后立刻再跑一遍。 */
        while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET) Delay_ms(10);
    }
}


int main(void)
{
    u16 cur_x = PATH_START_X;
    u16 cur_y = PATH_START_Y;
    u8 step = 0;
	
	s32 yaw_centi_degrees = 0, last_yaw_centi_degrees = 0;
    LED_Init();
    Servo_Init();             /* PC7/PC8/PC9 三路舵机 PWM，上电输出安全角度 240/90/65 */
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

	/* 夹取物料调试动作：PA8 启动整套动作，PA11 中途中止，OLED 显示当前步骤和三个舵机角度。
	 * 这是临时测试，排在调试模式派发之前，所以下面的 DEBUG_MODE 分支暂时都走不到；
	 * 想回去测物料电机 / 单个舵机就换成 material_calib_test() / servo270_test()。 */
	grab_run_once();

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
#elif DEBUG_MODE == DEBUG_MODE_CHASSIS
	chassis_check_test();       /* 速度模式底盘自检：前进/平移/旋转/标定 */
#elif DEBUG_MODE == DEBUG_MODE_WIRELESS
	wireless_test_run();        /* 无线串口底盘测试：USART3 + tools/chassis_host.py */
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
