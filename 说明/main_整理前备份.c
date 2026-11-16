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
 *   DEBUG_MODE_VISION  7 = 视觉链路测试（当前默认：发 circle/linear，OLED 显示 K230 回的数据）
 *   DEBUG_MODE_TURN    4 = 闭环旋转单独测试（按键触发）
 *   DEBUG_MODE_ROUTE   3 = 全图路线 A→E→B→C→D→B→C→D→A
 *   DEBUG_MODE_MOTOR   2 = 电机方向 / 地址自检（换电机后先用它）
 *   DEBUG_MODE_CIRCLE  1 = K230 圆心跟踪（旧的纯数字 "x,y" 格式）
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
#define DEBUG_MODE_VISION   7

#define DEBUG_MODE          DEBUG_MODE_VISION

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
 * 夹取物料调试动作（3 路舵机 + 升降步进电机）
 * ----------------------------------------------------------------------------
 * 部件：
 *   升降舵机 PC7  0~270°，初始 90°
 *   夹爪舵机 PC8  0~270°，初始 65°
 *   旋转舵机 PC9  0~270°，初始 240°
 *   升降电机 地址 6：13 字节位置帧，CCW(0x01) = 升高；**绝对位置模式**，
 *                    "上电位置 = 0"，所以升高 = 绝对 1000 脉冲，回原位 = 绝对 0
 *
 * 正向 6 步（PA8 触发）：
 *   1) 升降舵机 90 → 70      贴近地面
 *   2) 夹爪舵机 65 → 110     夹起物料
 *   3) 升降电机 → 1000 脉冲  把物料抬起（CCW）
 *   4) 升降舵机 70 → 170     抬高
 *   5) 旋转舵机 240 → 80     转到车上
 *   6) 夹爪舵机 110 → 60     放开物料   ← 这一步不记入撤销栈
 *
 * 栈式复位（正向跑完自动执行）：每执行一个动作就把"反向动作"压栈，
 * 复位时从栈顶一个个弹出，所以顺序正好是正向的倒序：
 *   R1 旋转 80 → 240    （退第 5 步）
 *   R2 升降 170 → 70    （退第 4 步）
 *   R3 升降电机 → 0     （退第 3 步，CW 下降）
 *   R4 夹爪 60 → 65     （退第 2 步）
 *   R5 升降 70 → 90     （退第 1 步）
 * 第 6 步"放开物料"故意不压栈：复位时不把空夹爪再合回去，夹爪的复位由
 * 第 2 步记录的反向动作（回 65°）完成。复位结束四个部件都回初始位，可再夹一次。
 *
 * PA11：正向、复位两个阶段都能中止；中止 = **停在原地**（不自动找姿态、不自动复位），
 *       现场手动摆回安全位。中止发生在升降电机运动途中时，会补发一帧"立即停止"。
 * 地址 5（物料盘电机）本动作完全不碰。
 * ========================================================================== */
#define GRAB_SETTLE_MS        800u    /* 舵机每步等待 */
#define GRAB_MOTOR_SETTLE_MS  1000u   /* 升降电机每步等待（30 RPM 走 1000 脉冲约 0.63 s） */

#define LIFT_MOTOR_ADDR       0x06u   /* 升降步进电机地址 */
#define LIFT_MOTOR_SPEED      40u     /* 速度 RPM。60 太快，电流冲击大（会把 K230 拉重启），
                                       * 降到 40；9000 脉冲约 4.2 s 走完 */
#define LIFT_MOTOR_ACCEL      80u     /* 加速度档（原来 150；越小斜坡越缓，起步电流越小） */
#define LIFT_MOTOR_HOME       0u      /* 上电位置 = 绝对 0 */
#define LIFT_MOTOR_UP         2000u   /* 升高 2000 脉冲 */

#define GRAB_UNDO_MAX         8u      /* 撤销栈深度，够 6 步用 */

typedef struct {
    uint8_t  is_servo;    /* 1 = 舵机, 0 = 升降步进电机 */
    ServoId  id;          /* is_servo=1：哪一路舵机 */
    uint16_t deg;         /* is_servo=1：目标角度 */
    uint8_t  dir;         /* is_servo=0：0x01 = CCW 升 / 0x00 = CW 降 */
    uint32_t pulses;      /* is_servo=0：绝对位置脉冲（上电位置 = 0） */
    uint8_t  undo;        /* 1 = 执行时把反向动作压栈 */
    char     label[8];    /* 正向步骤显示用，最多 7 个字符 */
} GrabAction;

/* 升降舵机已取消，动作表里只剩夹爪舵机、旋转舵机和升降步进电机（地址 6）。
 * 角度是现场实测标定的：
 *   旋转：30°  = 夹物料位置（初始）    200° = 放物料位置（满行程）
 *   夹爪：85°  = 松开                   115° = 夹紧
 *
 * 注意：满行程 30 → 200 实测转得太多，现在只转一半（85°），
 *       即放到 SERVO_ROT_PLACE_DEG = 115°。想改回满行程把下面这个宏改成 200 即可。
 *
 *   1) 夹爪 85 → 115      夹起物料
 *   2) 升降电机 → 2000    把物料抬起（绝对位置、CCW）
 *   3) 旋转 30 → 115      转半个行程（原来到 200 太多）
 *   4) 夹爪 115 → 85      放开物料（不压栈）
 * 复位（压栈弹栈）：旋转→30 → 升降电机→0 → 夹爪→85，倒序退回。 */
#define SERVO_ROT_PLACE_DEG   115u   /* 转到放料/落料位置的角度：满行程 200，现在用一半 */

static const GrabAction k_grab_steps[] = {
    { 1u, SERVO_GRIP, 115u, 0u,    0u,            1u, "GRIP115" },
    { 0u, SERVO_ROT,    0u, 0x01u, LIFT_MOTOR_UP, 1u, "MOT2000"  },
    { 1u, SERVO_ROT, SERVO_ROT_PLACE_DEG, 0u, 0u, 1u, "ROT"     },
    { 1u, SERVO_GRIP,  85u, 0u,    0u,            0u, "GRIP85"  }   /* 放开物料：不压栈 */
};

#define GRAB_STEP_COUNT   ((uint8_t)(sizeof(k_grab_steps) / sizeof(k_grab_steps[0])))

/* 界面状态（第 4 行） */
#define GRAB_READY   0u
#define GRAB_BUSY    1u
#define GRAB_DONE    2u
#define GRAB_ABORT   3u

/* 撤销栈 + 升降电机当前被命令到的绝对位置（只用于显示） */
static GrabAction g_undo[GRAB_UNDO_MAX];
static uint8_t    g_undo_n = 0u;
static uint32_t   g_lift_target = LIFT_MOTOR_HOME;

/* 升降步进电机（地址 6）：绝对位置模式、立即执行 */
static void lift_motor_move(uint32_t abs_pulses, uint8_t dir)
{
    uint8_t cmd[13] = {
        LIFT_MOTOR_ADDR,
        0xFD,                                            /* 位置模式 */
        dir,                                             /* 0x01 = CCW 升 */
        (uint8_t)(LIFT_MOTOR_SPEED >> 8),
        (uint8_t)(LIFT_MOTOR_SPEED & 0xFFu),
        (uint8_t)LIFT_MOTOR_ACCEL,
        0x00, 0x00, 0x00, 0x00,                          /* 脉冲数，下面填 */
        0x01,                                            /* 绝对位置：目标 = 上电位置起的脉冲数 */
        0x00,                                            /* 立即执行 */
        0x6B
    };

    cmd[6] = (uint8_t)(abs_pulses >> 24);
    cmd[7] = (uint8_t)(abs_pulses >> 16);
    cmd[8] = (uint8_t)(abs_pulses >> 8);
    cmd[9] = (uint8_t)abs_pulses;

    Serial_SendArray(cmd, 13);
    Delay_ms(5);

    g_lift_target = abs_pulses;
}

/* 立即停止（0xFE 0x98）：中止时用，别让它继续往目标位置走 */
static void lift_motor_stop(void)
{
    Serial_SendByte(LIFT_MOTOR_ADDR);
    Serial_SendByte(0xFE);
    Serial_SendByte(0x98);
    Serial_SendByte(0x00);
    Serial_SendByte(0x6B);
    Delay_ms(5);
}

/* —— 按键极性 ——
 * 本板按键：一脚接 3.3V、一脚接单片机引脚 → **按下时引脚变高**，
 * 所以按键脚统一配下拉输入（IPD），读到高电平 = 按下。
 * 如果以后换成"按下接 GND"的按键，把下面改成 0 并把 IPD 换成 IPU 即可。 */
#define GRAB_KEY_ACTIVE_HIGH   1

#if GRAB_KEY_ACTIVE_HIGH
  #define GRAB_KEY_MODE        GPIO_Mode_IPD
#else
  #define GRAB_KEY_MODE        GPIO_Mode_IPU
#endif

/* 读一个按键：1 = 按下 */
static uint8_t grab_key_down(GPIO_TypeDef *port, uint16_t pin)
{
#if GRAB_KEY_ACTIVE_HIGH
    return (GPIO_ReadInputDataBit(port, pin) == Bit_SET) ? 1u : 0u;
#else
    return (GPIO_ReadInputDataBit(port, pin) == Bit_RESET) ? 1u : 0u;
#endif
}

/* 按键初始化：PB1（单步）、PA7（连跑）、PA5（中止），全部下拉输入
 * （按键资源：PB1 / PA7 / PA5；按下时引脚变高）。
 * 不再使用 PA8/PA11/PA12：PA12 是 USB D+，板上有 1.5k 上拉，
 * 会被一直读成"按下"，从而卡在"等松手"里，导致按 PB1 完全没反应。 */
static void grab_key_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    gpio.GPIO_Mode  = GRAB_KEY_MODE;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    gpio.GPIO_Pin   = GPIO_Pin_7 | GPIO_Pin_5;      /* PA7 = 连跑、PA5 = 中止 */
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin   = GPIO_Pin_1;                   /* PB1 = 单步 */
    GPIO_Init(GPIOB, &gpio);
}

/* 等松手，最多等 2 s（400 × 5 ms）。加超时是为了防止某个按键脚被外部一直拉高，
 * 让程序永远卡在"等松手"里，表现成"按键全都没反应"。返回 1 = 真的松手了。 */
#define GRAB_RELEASE_TIMEOUT   400u

static uint8_t grab_wait_release(GPIO_TypeDef *port, uint16_t pin)
{
    uint16_t t;

    for(t = 0; t < GRAB_RELEASE_TIMEOUT; t++){
        if(!grab_key_down(port, pin)) return 1u;
        Delay_ms(5);
    }
    return 0u;
}

/* 连跑键 PA7：20 ms 消抖 + 等松手，有按下返回 1 */
static uint8_t grab_start_hit(void)
{
    if(!grab_key_down(GPIOA, GPIO_Pin_7)) return 0;

    Delay_ms(20);
    if(!grab_key_down(GPIOA, GPIO_Pin_7)) return 0;

    grab_wait_release(GPIOA, GPIO_Pin_7);      /* 等松手（带超时，不会卡死） */
    Delay_ms(20);
    return 1;
}

/* 单步键 PB1：20 ms 消抖 + 等松手，有按下返回 1 */
static uint8_t grab_step_hit(void)
{
    if(!grab_key_down(GPIOB, GPIO_Pin_1)) return 0u;

    Delay_ms(20);

    if(!grab_key_down(GPIOB, GPIO_Pin_1)) return 0u;          /* 抖动 */

    grab_wait_release(GPIOB, GPIO_Pin_1);                    /* 等松手（带超时） */

    Delay_ms(20);
    return 1u;
}

/* 每做完一个动作闪一下 PC13：确认"按键被识别、动作执行了" */
static void pc13_step_blink(void)
{
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);   /* 亮（低电平点亮） */
    Delay_ms(60);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);     /* 灭 */
}

/* 中止键 PA5：按下后等松手再返回 1，避免一直按着被重复判定 */
static uint8_t grab_abort_hit(void)
{
    if(!grab_key_down(GPIOA, GPIO_Pin_5)) return 0;

    grab_wait_release(GPIOA, GPIO_Pin_5);
    return 1;
}

/* 等 ms 毫秒；期间 PA5 按下立刻返回 0（中止本次动作） */
static uint8_t grab_wait(uint16_t ms)
{
    uint16_t t;

    for(t = 0; t < ms; t++){
        if(grab_abort_hit()) return 0;
        Delay_ms(1);
    }
    return 1;
}

/* 执行一个动作并等它到位；PA11 按下返回 0（中止，动作停在原地） */
static uint8_t grab_exec(const GrabAction *a)
{
    if(a->is_servo){
        Servo_SetAngle(a->id, a->deg);
        return grab_wait(GRAB_SETTLE_MS);
    }

    lift_motor_move(a->pulses, a->dir);
    if(!grab_wait(GRAB_MOTOR_SETTLE_MS)){
        lift_motor_stop();            /* 中止：先让它停下，别继续走完 */
        return 0;
    }
    return 1;
}

/* 把一个动作的反向动作压栈：
 *   舵机 → (同一路, 执行前的实际角度)
 *   电机 → 回 LIFT_MOTOR_HOME（上电位置 0），方向 CW 下降 */
static void grab_record_undo(const GrabAction *a)
{
    GrabAction inv;

    if(!a->undo || g_undo_n >= GRAB_UNDO_MAX) return;

    inv.is_servo = a->is_servo;
    inv.id       = a->id;
    inv.deg      = 0u;
    inv.dir      = 0u;
    inv.pulses   = 0u;
    inv.undo     = 0u;
    inv.label[0] = '\0';

    if(a->is_servo){
        inv.deg = Servo_GetAngle(a->id);              /* 执行前的角度 */
    }else{
        inv.pulses = LIFT_MOTOR_HOME;
        inv.dir    = 0x00u;                          /* 回 0 = 下降 = CW */
    }

    g_undo[g_undo_n++] = inv;
}

/* 第 4 行状态 */
static void grab_show_state(uint8_t state)
{
    switch(state){
        case GRAB_BUSY:  OLED_ShowString(4, 1, "BUSY...         "); break;
        case GRAB_DONE:  OLED_ShowString(4, 1, "DONE HOME       "); break;
        case GRAB_ABORT: OLED_ShowString(4, 1, "ABORT BY PA5    "); break;
        default:         OLED_ShowString(4, 1, "PB1:STEP PA7:RUN"); break;
    }
}

/* 第 2、3 行：两个舵机的实际角度 + 升降电机当前目标位置 */
static void grab_show_angles(void)
{
    uint16_t rot  = Servo_GetAngle(SERVO_ROT);
    uint16_t grip = Servo_GetAngle(SERVO_GRIP);

    /* 第 2 行：ROT:30（当前旋转角度） */
    OLED_ShowString(2, 1, "                ");
    OLED_ShowString(2, 1, "ROT:");
    OLED_ShowNum(2, 5, rot, oled_digits(rot));

    /* 第 3 行：GRP:85 M:2000（M = 升降电机目标脉冲） */
    OLED_ShowString(3, 1, "                ");
    OLED_ShowString(3, 1, "GRP:");
    OLED_ShowNum(3, 5, grip, oled_digits(grip));
    OLED_ShowString(3, 9, "M:");
    OLED_ShowNum(3, 11, g_lift_target, oled_digits(g_lift_target));
}

/* 待机 / 完成画面 */
static void grab_show_idle(uint8_t state)
{
    OLED_ShowString(1, 1, "                ");
    OLED_ShowString(1, 1, "GRAB  ");
    OLED_ShowString(1, 7, (state == GRAB_DONE) ? "DONE " : "READY");
    grab_show_angles();
    grab_show_state(state);
}

/* 正向第 step 步的画面：GRAB 3/4 ROT200 */
static void grab_show_forward(uint8_t step, uint8_t state)
{
    OLED_ShowString(1, 1, "                ");
    OLED_ShowString(1, 1, "GRAB ");
    OLED_ShowNum(1, 6, step, 1);
    OLED_ShowString(1, 7, "/");
    OLED_ShowNum(1, 8, GRAB_STEP_COUNT, 1);
    if(step > 0u){
        OLED_ShowString(1, 10, (char *)k_grab_steps[step - 1u].label);
    }

    grab_show_angles();
    grab_show_state(state);
}

/* 复位第 n 步的画面：BACK 2/5 */
static void grab_show_back(uint8_t n, uint8_t total, uint8_t state)
{
    OLED_ShowString(1, 1, "                ");
    OLED_ShowString(1, 1, "BACK ");
    OLED_ShowNum(1, 6, n, 1);
    OLED_ShowString(1, 7, "/");
    OLED_ShowNum(1, 8, total, 1);

    grab_show_angles();
    grab_show_state(state);
}

/* ============================================================================
 * 单步 / 连跑状态（供 PB1 单步、PA8 连跑共用）
 * ----------------------------------------------------------------------------
 * g_fwd_done   正向已经执行了几步（0~GRAB_STEP_COUNT）
 * g_undo_total 进入复位阶段时撤销栈里有多少个动作
 * g_undo_done  复位已经退了几步
 * ========================================================================== */
#define GRAB_PH_IDLE  0u
#define GRAB_PH_FWD   1u
#define GRAB_PH_UNDO  2u
#define GRAB_PH_DONE  3u

static uint8_t g_grab_phase  = GRAB_PH_IDLE;
static uint8_t g_fwd_done    = 0u;
static uint8_t g_undo_total  = 0u;
static uint8_t g_undo_done   = 0u;

/* 从待机重来：清空撤销栈和所有计数 */
static void grab_reset_state(void)
{
    g_undo_n     = 0u;
    g_fwd_done   = 0u;
    g_undo_total = 0u;
    g_undo_done  = 0u;
    g_grab_phase = GRAB_PH_IDLE;
}

/* 执行"一个"动作（正向或复位都算），供 PB1 单步用；返回 0 = 被 PA11 中止。
 * PB1 每按一次就前进一个动作：正向 4 步走完自动进入复位阶段，再一步步弹栈。 */
static uint8_t grab_step_once(void)
{
    GrabAction a;

    if(g_grab_phase == GRAB_PH_IDLE) g_grab_phase = GRAB_PH_FWD;

    /* 正向走完 -> 切到复位阶段 */
    if(g_grab_phase == GRAB_PH_FWD && g_fwd_done >= GRAB_STEP_COUNT){
        g_grab_phase = GRAB_PH_UNDO;
        g_undo_total = g_undo_n;
    }

    if(g_grab_phase == GRAB_PH_FWD){
        grab_record_undo(&k_grab_steps[g_fwd_done]);
        grab_show_forward((uint8_t)(g_fwd_done + 1u), GRAB_BUSY);

        if(k_grab_steps[g_fwd_done].is_servo){
            USART3_Printf("[S%u] %s -> %u deg\r\n", (unsigned)(g_fwd_done + 1u),
                          k_grab_steps[g_fwd_done].label,
                          (unsigned)k_grab_steps[g_fwd_done].deg);
        }else{
            USART3_Printf("[S%u] %s -> %u pulse (abs, CCW)\r\n", (unsigned)(g_fwd_done + 1u),
                          k_grab_steps[g_fwd_done].label,
                          (unsigned)k_grab_steps[g_fwd_done].pulses);
        }

        if(!grab_exec(&k_grab_steps[g_fwd_done])) return 0;
        g_fwd_done++;

        if(g_fwd_done >= GRAB_STEP_COUNT){
            g_grab_phase = GRAB_PH_UNDO;
            g_undo_total = g_undo_n;
        }
        return 1;
    }

    if(g_grab_phase == GRAB_PH_UNDO){
        if(g_undo_n == 0u){
            g_grab_phase = GRAB_PH_DONE;
            return 1;
        }

        a = g_undo[--g_undo_n];                 /* 后进先出：最后做的先退 */
        g_undo_done++;

        grab_show_back(g_undo_done, g_undo_total, GRAB_BUSY);
        USART3_Printf("[R%u/%u] %s -> %u\r\n", (unsigned)g_undo_done, (unsigned)g_undo_total,
                      a.is_servo ? "SERVO" : "LIFT-MOTOR",
                      (unsigned)(a.is_servo ? a.deg : a.pulses));

        if(!grab_exec(&a)) return 0;

        if(g_undo_n == 0u) g_grab_phase = GRAB_PH_DONE;
        return 1;
    }

    return 1;                                   /* 已经做完了 */
}

/* 上电自检动作：让两个舵机各走一次可见的往返，用来区分"引脚没输出"还是"舵机/供电问题"。
 *   旋转（PA1）：30° → 115° → 30°
 *   夹爪（PA0）：85° → 115° → 85°
 * 不想每次上电都动，把 GRAB_BOOT_SELFTEST 改成 0。 */
#define GRAB_BOOT_SELFTEST   0

static void grab_boot_selftest(void)
{
#if GRAB_BOOT_SELFTEST
    OLED_ShowString(1, 1, "SELFTEST ROT    ");
    Servo_SetAngle(SERVO_ROT, SERVO_ROT_PLACE_DEG);
    Delay_ms(800);
    Servo_SetAngle(SERVO_ROT, SERVO_ROT_INIT_DEG);
    Delay_ms(800);

    OLED_ShowString(1, 1, "SELFTEST GRIP   ");
    Servo_SetAngle(SERVO_GRIP, 115u);
    Delay_ms(800);
    Servo_SetAngle(SERVO_GRIP, SERVO_GRIP_INIT_DEG);
    Delay_ms(800);
    OLED_ShowString(1, 1, "                ");
#endif
}

/* 夹取动作主循环：不返回。
 *   PB1 ：单步——每按一次只做一个动作（正向 4 步做完后自动进入复位，再一步步弹栈）
 *   PA7 ：连跑——正向 4 步 + 栈式复位，然后回到待机
 *   PA5 ：中止——停在原地
 * 每做完一个动作 PC13 会闪一下，OLED 上也能看到当前步骤。
 */
static void grab_run_once(void)
{
    uint8_t ok;

    /* 上电安全位（Servo_Init 里已经输出过，这里再确认一次，重进本函数时也一致） */
    Servo_SetAngle(SERVO_ROT,  SERVO_ROT_INIT_DEG);
    Servo_SetAngle(SERVO_GRIP, SERVO_GRIP_INIT_DEG);

    grab_key_init();

    OLED_Clear();
    grab_boot_selftest();          /* 旋转 240→80→240、夹爪 65→110→65 */
    grab_show_idle(GRAB_READY);
    USART3_SendString("\r\nGRAB READY: PB1=STEP PA7=RUN PA5=ABORT\r\n");

    while(1){
        /* ---------- PA7：连跑整套 ---------- */
        if(grab_start_hit()){
            grab_reset_state();
            motor_enable_addr(LIFT_MOTOR_ADDR);     /* 地址 6 上电默认失能，跑之前先使能 */
            Delay_ms(20);

            USART3_SendString("[RUN] START (auto)\r\n");

            ok = 1u;
            while(g_grab_phase != GRAB_PH_DONE){
                if(!grab_step_once()){ ok = 0u; break; }
                pc13_step_blink();                  /* 每步闪一下 LED */
            }

            if(ok){
                grab_show_idle(GRAB_DONE);
                USART3_SendString("[RUN] DONE (back home, ready)\r\n");
            }else{
                grab_show_angles();
                grab_show_state(GRAB_ABORT);
                USART3_SendString("[RUN] ABORT BY PA5 - stopped in place\r\n");
            }

            /* 收尾：等 PA7 松手，避免按住不放连着再跑一遍 */
            while(grab_key_down(GPIOA, GPIO_Pin_7)) Delay_ms(10);
            continue;
        }

        /* ---------- PB1：单步 ---------- */
        if(grab_step_hit()){
            if(g_grab_phase == GRAB_PH_DONE) grab_reset_state();   /* 上一轮做完了：重新开始 */

            motor_enable_addr(LIFT_MOTOR_ADDR);     /* 每次单步前都保证地址 6 已使能 */
            Delay_ms(10);

            USART3_Printf("[STEP] phase=%u fwd=%u undo=%u\r\n",
                          (unsigned)g_grab_phase, (unsigned)g_fwd_done, (unsigned)g_undo_done);

            if(grab_step_once()){
                pc13_step_blink();                  /* 按键被识别 + 动作执行：闪一下 */
                if(g_grab_phase == GRAB_PH_DONE){
                    grab_show_idle(GRAB_DONE);
                    USART3_SendString("[STEP] DONE (back home)\r\n");
                }
            }else{
                grab_show_angles();
                grab_show_state(GRAB_ABORT);
                USART3_SendString("[STEP] ABORT BY PA5\r\n");
            }

            continue;
        }

        Delay_ms(5);
    }
}

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
static void servo_tune_test(void)
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
            OLED_ShowNum(2, 9, ang[sel], oled_digits(ang[sel]));

            OLED_ShowString(3, 1, "PUL:            ");
            OLED_ShowString(3, 1, "PUL:");
            OLED_ShowNum(3, 5, pulse, oled_digits(pulse));
            OLED_ShowString(3, 9, "us");

            /* 到边界时给个提示，方便知道"再按也不会动" */
            if(ang[sel] >= SERVO_ANGLE_MAX)      OLED_ShowString(3, 12, "MAX");
            else if(ang[sel] == 0u)              OLED_ShowString(3, 12, "MIN");
            else                                 OLED_ShowString(3, 12, "   ");
        }

        Delay_ms(5);
    }
}

/* ============================================================================
 * 小车移动测试（当前使用的测试）
 * ----------------------------------------------------------------------------
 *   PA5 = 切换模式（循环）：
 *           模式 1：前进 / 后退
 *           模式 2：左平移 / 右平移
 *           模式 3：顺时针 / 逆时针旋转
 *   PB1 = 正向（前进 / 左移 / 顺时针）
 *   PA7 = 反向（后退 / 右移 / 逆时针）
 *
 * 每按一次走一小段（可在下面两个宏里改）：
 *   直线/平移 CAR_STEP_MM 毫米，旋转 CAR_STEP_DEG 度。
 * 运动本身是非阻塞的：命令下发后电机自己走完，程序立刻回到按键扫描。
 *
 * OLED：
 *   第 1 行  CAR TEST  M1/3
 *   第 2 行  MODE: FWD/BWD
 *   第 3 行  STEP:200mm 45dg
 *   第 4 行  >FWD  （刚执行的方向）
 * 每次按键 PC13 闪一下。
 * ========================================================================== */
#define CAR_STEP_MM      200u    /* 每次前进/后退/平移多少毫米 */
#define CAR_STEP_DEG      45u    /* 每次旋转多少度 */

#define CAR_MODE_FWD     0u      /* 前进/后退 */
#define CAR_MODE_LAT     1u      /* 左右平移 */
#define CAR_MODE_YAW     2u      /* 顺/逆时针 */
#define CAR_MODE_COUNT   3u

static void car_show_mode(uint8_t mode)
{
    /* 第 1 行：CAR TEST  M1/3 */
    OLED_ShowString(1, 1, "CAR TEST  M     ");
    OLED_ShowNum(1, 12, (uint32_t)(mode + 1u), 1);
    OLED_ShowString(1, 13, "/3");

    /* 第 2 行：这个模式的两个方向 */
    OLED_ShowString(2, 1, "MODE:           ");
    if(mode == CAR_MODE_FWD)      OLED_ShowString(2, 7, "FWD/BWD ");
    else if(mode == CAR_MODE_LAT) OLED_ShowString(2, 7, "LEFT/RGT");
    else                          OLED_ShowString(2, 7, "CW/CCW  ");
}

/* 执行一次移动；fwd = 1 正向，0 反向。返回显示用的短名字 */
static const char *car_action(uint8_t mode, uint8_t fwd)
{
    if(mode == CAR_MODE_FWD){
        if(fwd){ qianjin_(CAR_STEP_MM); return "FWD "; }
        houtui_(CAR_STEP_MM);
        return "BWD ";
    }

    if(mode == CAR_MODE_LAT){
        if(fwd){ zuo_(CAR_STEP_MM); return "LEFT "; }
        you_(CAR_STEP_MM);
        return "RIGHT";
    }

    if(fwd){ shun_yaw_(CAR_STEP_DEG); return "CW  "; }
    ni_yaw_(CAR_STEP_DEG);
    return "CCW ";
}

static void car_move_test(void)
{
    uint8_t     mode = CAR_MODE_FWD;
    const char *name;

    grab_key_init();                 /* PB1 / PA7 / PA5：下拉输入，按下为高 */

    OLED_Clear();
    OLED_ShowString(3, 1, "STEP:200mm 45dg ");
    OLED_ShowString(4, 1, "PA5:MODE        ");
    car_show_mode(mode);

    while(1){
        /* ---------- PA5：切换模式 ---------- */
        if(grab_key_down(GPIOA, GPIO_Pin_5)){
            Delay_ms(20);
            if(grab_key_down(GPIOA, GPIO_Pin_5)){
                grab_wait_release(GPIOA, GPIO_Pin_5);
                Delay_ms(20);
                mode = (uint8_t)((mode + 1u) % CAR_MODE_COUNT);
                car_show_mode(mode);
                pc13_step_blink();
                continue;
            }
        }

        /* ---------- PB1：正向 ---------- */
        if(grab_key_down(GPIOB, GPIO_Pin_1)){
            Delay_ms(20);
            if(grab_key_down(GPIOB, GPIO_Pin_1)){
                grab_wait_release(GPIOB, GPIO_Pin_1);
                Delay_ms(20);
                name = car_action(mode, 1u);
                OLED_ShowString(4, 1, "                ");
                OLED_ShowString(4, 1, ">");
                OLED_ShowString(4, 2, (char *)name);
                pc13_step_blink();
                continue;
            }
        }

        /* ---------- PA7：反向 ---------- */
        if(grab_key_down(GPIOA, GPIO_Pin_7)){
            Delay_ms(20);
            if(grab_key_down(GPIOA, GPIO_Pin_7)){
                grab_wait_release(GPIOA, GPIO_Pin_7);
                Delay_ms(20);
                name = car_action(mode, 0u);
                OLED_ShowString(4, 1, "                ");
                OLED_ShowString(4, 1, ">");
                OLED_ShowString(4, 2, (char *)name);
                pc13_step_blink();
                continue;
            }
        }

        Delay_ms(5);
    }
}

/* ============================================================================
 * USART2 引脚自检（测试 PA2 = TX、PA3 = RX 是不是好的）
 * --------------------------------------------------------------------
 --------
 * 做三件事：
 *   1) 电气自检：把 PA2/PA3 依次配成上拉、下拉再读回。
 *      正常引脚会跟随（上拉读到 1、下拉读到 0）；一直读 0 = 短地/损坏。
 *   2) 自环测试：从 PA2 发 6 个字节，再在 PA3 上等回环。
 *      ★ 需要把 PA2 和 PA3 用一根杜邦线短接（不需要别的设备）。
 *   3) 在线测试：之后每 0.5 s 往 PA2 发一行 "PA2TEST"，
 *      同时把 PA3 收到的字节计数并显示最后一个字节。
 *      - USB-TTL 的 RX 接 PA2 → 串口助手应看到 PA2TEST
 *      - USB-TTL 的 TX 接 PA3 → OLED 的 RX 计数会增加、显示最后收到的字节
 *
 * OLED：
 *   1 行  USART2 PIN TEST
 *   2 行  PA2:OK  PA3:OK     （电气跟随）
 *   3 行  LOOP:OK / LOOP:FAIL（自环，未短接就是 FAIL，正常）
 *   4 行  TX:12 RX:3 5A      （已发几行 / 收到几字节 / 最后一字节）
 * LED：自环通过 PC13 闪 2 下，失败闪 5 下。
 * ========================================================================== */
#define PIN_TMO        40000u    /* 等一个回环字节的轮询上限（约几毫秒） */
#define PIN_LIVE_MS    500u      /* 在线测试：每 500 ms 发一行 */

/* 把某个脚先上拉读、再下拉读，判断它能不能跟随（= 电气基本正常） */
static uint8_t pin_follows_pull(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef g;
    uint8_t up, down;

    g.GPIO_Pin   = pin;
    g.GPIO_Speed = GPIO_Speed_50MHz;

    g.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(port, &g);
    Delay_ms(2);
    up = (GPIO_ReadInputDataBit(port, pin) == Bit_SET) ? 1u : 0u;

    g.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(port, &g);
    Delay_ms(2);
    down = (GPIO_ReadInputDataBit(port, pin) == Bit_SET) ? 1u : 0u;

    return (up == 1u && down == 0u) ? 1u : 0u;
}

/* PA2/PA3 恢复成串口引脚：PA2 = AF_PP(TX)，PA3 = IPU(RX) */
static void usart2_pins_restore(void)
{
    GPIO_InitTypeDef g;

    g.GPIO_Pin   = GPIO_Pin_2;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &g);

    g.GPIO_Pin   = GPIO_Pin_3;
    g.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &g);
}

/* 自环：PA2 发 → PA3 收，逐字节比对；需要 PA2-PA3 短接 */
static uint8_t usart2_loopback(void)
{
    static const uint8_t pat[] = { 0x55u, 0xAAu, 0x00u, 0xFFu, 0x41u, 0x42u };
    uint8_t  i, rx;
    uint32_t t;

    /* 先清掉可能残留的接收字节 */
    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET)
    {
        (void)USART_ReceiveData(USART2);
    }

    for (i = 0; i < (uint8_t)sizeof(pat); i++)
    {
        USART_SendData(USART2, pat[i]);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) { }

        t = 0;
        while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET)
        {
            if (++t > PIN_TMO) return 0u;              /* 没等到回环 */
        }

        rx = (uint8_t)USART_ReceiveData(USART2);
        if (rx != pat[i]) return 0u;                   /* 收到但对不上 */
    }

    return 1u;
}

static void usart2_pin_test(void)
{
    uint8_t  pa2_ok, pa3_ok, loop_ok;
    uint32_t tx_lines = 0u;
    uint32_t rx_bytes = 0u;
    uint8_t  last_rx  = 0u;
    uint16_t wait;
    char     hex[] = "00";

    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);    /* 测试期间自己轮询，不让中断抢数据 */

    OLED_Clear();
    OLED_ShowString(1, 1, "USART2 PIN TEST ");

    /* 1) 电气自检 */
    pa2_ok = pin_follows_pull(GPIOA, GPIO_Pin_2);
    pa3_ok = pin_follows_pull(GPIOA, GPIO_Pin_3);
    usart2_pins_restore();
    Delay_ms(5);

    OLED_ShowString(2, 1, "PA2:    PA3:    ");
    OLED_ShowString(2, 5, pa2_ok ? "OK " : "BAD");
    OLED_ShowString(2, 13, pa3_ok ? "OK " : "BAD");

    /* 2) 自环测试（PA2 和 PA3 短接时才可能通过） */
    loop_ok = usart2_loopback();
    OLED_ShowString(3, 1, loop_ok ? "LOOP:OK         " : "LOOP:FAIL       ");

    /* 串口也打一份，方便有调试线时看 */
    USART3_SendString("\r\n[USART2 PIN TEST]\r\n");
    USART3_Printf("  PA2 pull-up/down follow : %s\r\n", pa2_ok ? "OK" : "BAD");
    USART3_Printf("  PA3 pull-up/down follow : %s\r\n", pa3_ok ? "OK" : "BAD");
    USART3_Printf("  PA2->PA3 loopback       : %s\r\n", loop_ok ? "OK" : "FAIL");

    /* LED 提示：通过 2 下，失败 5 下 */
    {
        uint8_t n = loop_ok ? 2u : 5u, i;
        for (i = 0; i < n; i++)
        {
            GPIO_ResetBits(GPIOC, GPIO_Pin_13);
            Delay_ms(120);
            GPIO_SetBits(GPIOC, GPIO_Pin_13);
            Delay_ms(120);
        }
    }

    /* 3) 在线测试：持续发 + 收 */
    OLED_ShowString(1, 1, "USART2 LIVE TEST");

    while(1){
        USART2_SendString("PA2TEST\r\n");
        tx_lines++;

        /* 500 ms 内不断轮询接收 */
        for(wait = 0; wait < (PIN_LIVE_MS / 5u); wait++){
            while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET){
                last_rx = (uint8_t)USART_ReceiveData(USART2);
                rx_bytes++;
            }
            Delay_ms(5);
        }

        /* 第 4 行：TX 行数 / RX 字节数 / 最后一字节（十六进制） */
        hex[0] = "0123456789ABCDEF"[(last_rx >> 4) & 0x0Fu];
        hex[1] = "0123456789ABCDEF"[last_rx & 0x0Fu];

        OLED_ShowString(4, 1, "TX:     RX:     ");
        OLED_ShowNum(4, 4, tx_lines, oled_digits(tx_lines));
        OLED_ShowNum(4, 11, rx_bytes, oled_digits(rx_bytes));
        OLED_ShowString(4, 16, hex);      /* 最后一字节 */
    }
}


/* ============================================================================
 * 系统时钟自检
 * ----------------------------------------------------------------------------
 * 为什么需要它：Delay_us() 是按 HCLK 换算 SysTick 的。如果板上 8 MHz 的 HSE 晶振
 * 没起振，芯片会退回 HSI 8 MHz，而 SystemCoreClock 变量仍然写着 72 MHz，
 * 于是所有延时都慢 9 倍（500 ms 变成 4.5 s，LED 闪得特别慢就是它）。
 * 这里直接读 RCC 寄存器算出真实的 SYSCLK、写回 SystemCoreClock，
 * 再在 OLED / 串口上报告，方便区分"时钟不对"和"程序没跑/别的问题"。
 * ========================================================================== */
#define CLK_72MHZ   72000000u

/* 读 RCC 寄存器算真实 SYSCLK（不信 SystemCoreClock 变量）：
 *   SWS = 00 → HSI 8 MHz；01 → HSE 8 MHz；10 → PLL（HSI/2 或 HSE 再乘倍频） */
static uint32_t clk_sysclk_hz(void)
{
    uint32_t cfgr = RCC->CFGR;
    uint32_t mul, src;

    switch(cfgr & RCC_CFGR_SWS){
        case RCC_CFGR_SWS_HSI:
            return HSI_VALUE;

        case RCC_CFGR_SWS_HSE:
            return HSE_VALUE;

        case RCC_CFGR_SWS_PLL:
            mul = (((cfgr & RCC_CFGR_PLLMULL) >> 18) + 2u);
            if(mul > 16u) mul = 16u;                   /* 1111 也按 x16 算 */

            if((cfgr & RCC_CFGR_PLLSRC) != 0u){        /* PLL 源 = HSE */
                src = ((cfgr & RCC_CFGR_PLLXTPRE) != 0u) ? (HSE_VALUE / 2u) : HSE_VALUE;
            }else{                                     /* PLL 源 = HSI/2 */
                src = HSI_VALUE / 2u;
            }
            return src * mul;

        default:
            return 0u;
    }
}

/* 用真实频率修正 SystemCoreClock：Delay_us() 按它换算，改完延时才是准的 */
static void clk_fix(void)
{
    uint32_t hz = clk_sysclk_hz();

    if(hz != 0u) SystemCoreClock = hz;
}

/* 把时钟情况报告到串口（只打串口，不画 OLED：
 * OLED 没接/接错时绘图要等 I2C 超时，会把上电流程拖慢，这里不冒这个险） */
static void clk_report(void)
{
    uint32_t hz   = SystemCoreClock;
    uint32_t cfgr = RCC->CFGR;
    const char *src = "???";

    switch(cfgr & RCC_CFGR_SWS){
        case RCC_CFGR_SWS_HSI: src = "HSI"; break;
        case RCC_CFGR_SWS_HSE: src = "HSE"; break;
        case RCC_CFGR_SWS_PLL: src = "PLL"; break;
        default: break;
    }

    USART3_Printf("\r\n[CLK] SYSCLK=%u Hz (%u MHz) src=%s HSE_RDY=%u PLL_ON=%u\r\n",
                  (unsigned)hz, (unsigned)(hz / 1000000u), src,
                  (unsigned)((RCC->CR & RCC_CR_HSERDY) ? 1u : 0u),
                  (unsigned)((RCC->CR & RCC_CR_PLLON) ? 1u : 0u));
    USART3_Printf("[OLED] %s\r\n", OLED_IsReady() ? "OK" : "NOT FOUND - 绘图全部跳过");
}

/* ============================================================================
 * 按钮自检：PC1 / PC2 / PC3 → PC13 / PC14 / PC15
 * ----------------------------------------------------------------------------
 * 对应关系（一个键管一个灯，按一次翻转一次）：
 *     PC1  →  PC13
 *     PC2  →  PC14
 *     PC3  →  PC15
 * 输入：PC1/PC2/PC3 上拉输入，按下接地（Key_GetNum() 不扫 PC，
 *       这里单独初始化、直接读输入寄存器）。**PC0 没焊好，本版不用。**
 * 输出：PC13/PC14/PC15 由 pc_out_high_init() 配成推挽输出，上电全高（灯灭）。
 * 一次"按下"= 20 ms 消抖 + 等松手；同时按多个键时只处理编号最小的那个。
 * OLED：第 1 行标题；第 2 行三个灯的电平；第 3 行最后按的键 + 累计次数；
 *       第 4 行三路按键的实时电平（未按全 1，按下哪一路哪一位变 0）。
 * 注 意：PC13/PC14/PC15 是低驱动能力引脚（≤3 mA、≤2 MHz、≤30 pF），只适合点灯/测量；
 *        PC14/PC15 是 LSE 晶振脚，本工程没有开 LSE，所以可以当 GPIO 用。
 * ========================================================================== */
#define PC_KEY_COUNT    3u
#define PC_KEY_PINS     (GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3)   /* PC0 不焊，排除在外 */
#define PC13_OUT_PIN    GPIO_Pin_13
#define PC14_OUT_PIN    GPIO_Pin_14
#define PC15_OUT_PIN    GPIO_Pin_15
#define PC_KEY_NONE     0xFFu

/* 键 → 灯 的映射表：下标 0 = PC1，1 = PC2，2 = PC3 */
static const uint16_t k_pc_key_pin[PC_KEY_COUNT] = { GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3 };
static const uint16_t k_pc_led_pin[PC_KEY_COUNT] = { GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15 };

/* 【硬件自检】PC13/PC14/PC15 三脚直接输出高电平，放在 main() 最前面调用。
 * 作用：只要程序跑起来，这三个脚立刻是高电平，可以用万用表/示波器直接量到，
 *       用来区分"程序没跑"和"程序跑了但按键没接对"。
 * 注 意：PC13/PC14/PC15 是低驱动能力引脚（≤3 mA、≤2 MHz、≤30 pF），
 *        按数据手册同一时刻只建议用一个当输出，这里为了自检三个一起置高；
 *        PC14/PC15 是 LSE 晶振脚，本工程没有开 LSE，所以可以当 GPIO 用。 */
static void pc_out_high_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;                  /* 这三脚最高 2 MHz */
    GPIO_Init(GPIOC, &gpio);

    GPIO_SetBits(GPIOC, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
}

/* PC1~PC3 配成上拉输入（PC13/14/15 的推挽输出由 pc_out_high_init() 配，这里不重复配） */
static void pc_key_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    gpio.GPIO_Pin   = PC_KEY_PINS;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);
}

/* 某个输出脚当前电平：1 = 高，0 = 低 */
static uint8_t pc_out_level(uint16_t pin)
{
    return (GPIO_ReadOutputDataBit(GPIOC, pin) == Bit_RESET) ? 0u : 1u;
}

/* 翻转任意 GPIO 输出脚：先读输出寄存器再写，保证和当前电平相反 */
static void gpio_pin_toggle(GPIO_TypeDef *port, uint16_t pin)
{
    if(GPIO_ReadOutputDataBit(port, pin) == Bit_RESET){
        GPIO_SetBits(port, pin);      /* 低 → 高 */
    }else{
        GPIO_ResetBits(port, pin);    /* 高 → 低 */
    }
}

/* 第 4 行：三路按键的实时电平（未按全 1，按下哪一路哪一位变 0）。
 * 内部缓存，电平没变就不重画，避免 OLED 一直被刷。 */
static void pc_key_show_input(void)
{
    static uint8_t shown = 0xFFu;                 /* 0xFF = 还没画过 */
    uint16_t v = (uint16_t)(GPIO_ReadInputData(GPIOC) & PC_KEY_PINS);
    uint8_t  bits = 0u, i;

    for(i = 0; i < PC_KEY_COUNT; i++){
        if((v & k_pc_key_pin[i]) != 0u) bits |= (uint8_t)(1u << i);
    }

    if(bits == shown) return;
    shown = bits;

    OLED_ShowString(4, 1, "P1=");
    OLED_ShowNum(4, 4, (uint32_t)((bits >> 0) & 1u), 1);
    OLED_ShowString(4, 6, "P2=");
    OLED_ShowNum(4, 9, (uint32_t)((bits >> 1) & 1u), 1);
    OLED_ShowString(4, 11, "P3=");
    OLED_ShowNum(4, 14, (uint32_t)((bits >> 2) & 1u), 1);
}

/* 当前按着的键号：0 = PC1，1 = PC2，2 = PC3；没按返回 PC_KEY_NONE。
 * 上拉输入 → 对应位为 0 就是按下；同时按下多个时返回编号最小的那个。 */
static uint8_t pc_key_down(void)
{
    uint16_t v = (uint16_t)(GPIO_ReadInputData(GPIOC) & PC_KEY_PINS);
    uint8_t  i;

    for(i = 0; i < PC_KEY_COUNT; i++){
        if((v & k_pc_key_pin[i]) == 0u) return i;
    }

    return PC_KEY_NONE;
}

/* 画界面：三个灯的电平、最后按的键、累计次数（只在翻转后调用，不闪屏） */
static void pc_key_draw(uint32_t count, uint8_t last_key)
{
    OLED_ShowString(1, 1, "PC1-3 -> PC13-15");

    /* 第 2 行：13:1 14:1 15:1 */
    OLED_ShowString(2, 1, "                ");
    OLED_ShowString(2, 1, "13:");
    OLED_ShowNum(2, 4, pc_out_level(PC13_OUT_PIN), 1);
    OLED_ShowString(2, 7, "14:");
    OLED_ShowNum(2, 10, pc_out_level(PC14_OUT_PIN), 1);
    OLED_ShowString(2, 13, "15:");
    OLED_ShowNum(2, 16, pc_out_level(PC15_OUT_PIN), 1);

    /* 第 3 行：K:2  CNT:5（K 显示的是 PC 编号 1~3） */
    OLED_ShowString(3, 1, "                ");
    OLED_ShowString(3, 1, "K:");
    if(last_key != PC_KEY_NONE){
        OLED_ShowNum(3, 3, (uint32_t)(last_key + 1u), 1);
    }
    OLED_ShowString(3, 6, "CNT:");
    OLED_ShowNum(3, 10, count, oled_digits(count));
}

/* 按钮测试主循环：不返回。
 *   PC1 按一次 → PC13 翻转一次
 *   PC2 按一次 → PC14 翻转一次
 *   PC3 按一次 → PC15 翻转一次 */
static void pc_key_toggle_test(void)
{
    uint32_t count = 0u;
    uint8_t  key;

    pc_key_init();                /* PC1~PC3 上拉输入；PC13/14/15 已由 pc_out_high_init() 配好 */

    OLED_Clear();
    pc_key_draw(count, PC_KEY_NONE);
    pc_key_show_input();
    USART3_Printf("\r\nBUTTON TEST: PC1->PC13  PC2->PC14  PC3->PC15 (PC0 不用)\r\n");

    while(1){
        key = pc_key_down();
        pc_key_show_input();      /* 第 4 行跟一下输入电平，按下就能看到哪一位变 0 */

        if(key == PC_KEY_NONE){
            Delay_ms(5);
            continue;
        }

        /* 消抖：20 ms 后还按着才算一次 */
        Delay_ms(20);
        key = pc_key_down();
        if(key == PC_KEY_NONE) continue;

        /* 等松手，避免按住不放连着翻 */
        while(pc_key_down() != PC_KEY_NONE) Delay_ms(10);
        Delay_ms(20);

        gpio_pin_toggle(GPIOC, k_pc_led_pin[key]);   /* PC1→PC13 / PC2→PC14 / PC3→PC15 */
        count++;

        pc_key_show_input();      /* 松手后第 4 行恢复 */
        pc_key_draw(count, key);
        USART3_Printf("[PC%u] -> PC%u = %u (cnt %u)\r\n",
                      (unsigned)(key + 1u), (unsigned)(13u + key),
                      (unsigned)pc_out_level(k_pc_led_pin[key]),
                      (unsigned)count);
    }
}

/* ============================================================================
 * PC13 方波 + 串口自检
 * ----------------------------------------------------------------------------
 * PC13 配成推挽输出，每 PC13_HALF_MS 毫秒翻转一次（默认 200 ms = 2.5 Hz）。
 * 同时每 1 秒从 USART3 打一行心跳（PB10 = TX，PB11 = RX，115200 8N1），
 * 收到什么就原样回显，用来验证收、发两个方向都通。
 *
 * 循环里**不碰 OLED**：OLED 不在线时每次绘图都要等 I2C 超时，会把周期拖到几秒。
 * PC13 是低驱动能力脚（≤3 mA、≤2 MHz），只适合点灯/测量。
 * ========================================================================== */
#define PC13_TEST_PIN   GPIO_Pin_13
#define PC13_HALF_MS    200u     /* 每个电平持续多少毫秒（200 = 2.5 Hz，500 = 1 Hz） */

/* PC13 当前电平：1 = 高（灯灭），0 = 低（灯亮） */
static uint8_t pc13_bit(void)
{
    return (GPIO_ReadOutputDataBit(GPIOC, PC13_TEST_PIN) == Bit_RESET) ? 0u : 1u;
}

/* PC13 方波 + 串口自检主循环：不返回。 */
static void pc13_uart_test(void)
{
    GPIO_InitTypeDef gpio;
    char     line[32];
    uint32_t edges = 0u;         /* 翻转次数 */
    uint32_t secs  = 0u;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    gpio.GPIO_Pin   = PC13_TEST_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;        /* 这三脚最高 2 MHz */
    GPIO_Init(GPIOC, &gpio);

    GPIO_ResetBits(GPIOC, PC13_TEST_PIN);     /* 从亮开始（灯低电平点亮） */

    /* ---------- 串口自检：开机横幅 ---------- */
    USART3_SendString("\r\n\r\n");
    USART3_SendString("==============================\r\n");
    USART3_SendString("     UART3 TEST 115200 8N1\r\n");
    USART3_SendString("  PB10=TX -> USB-TTL 的 RX\r\n");
    USART3_SendString("  PB11=RX <- USB-TTL 的 TX\r\n");
    USART3_SendString("  GND 必须共地\r\n");
    USART3_SendString("==============================\r\n");
    USART3_Printf("SYSCLK = %u Hz (%u MHz)\r\n",
                  (unsigned)SystemCoreClock, (unsigned)(SystemCoreClock / 1000000u));
    USART3_Printf("OLED   = %s\r\n", OLED_IsReady() ? "OK" : "NOT FOUND");
    USART3_Printf("PC13   = %u ms / level\r\n", (unsigned)PC13_HALF_MS);
    USART3_SendString("ASCII: 0123456789 ABCDEFGHIJKLMNOPQRSTUVWXYZ abcxyz\r\n");
    USART3_SendString("每 1 s 一行 UART3 OK；收到字符回显 RX:\r\n");

    while(1){
        Delay_ms(PC13_HALF_MS);

        gpio_pin_toggle(GPIOC, PC13_TEST_PIN);   /* 方波 */
        edges++;

        /* 5 次翻转 = 1 s：打一行心跳，确认串口一直在发 */
        if((edges % 5u) == 0u){
            secs++;
            USART3_Printf("UART3 OK  #%u  SYSCLK=%u  PC13=%u\r\n",
                          (unsigned)secs, (unsigned)SystemCoreClock,
                          (unsigned)pc13_bit());
        }

        /* 收到一行就回显（验证 RX 方向；PB11 同时是按键 2，接了无线模块就别按） */
        if(USART3_ReadLine(line, (uint8_t)sizeof(line))){
            USART3_Printf("RX: %s\r\n", line);
        }
    }
}


/* ============================================================================
 * 视觉闭环（主控 <-> K230）—— v2：会真的动电机，注意安全
 * ----------------------------------------------------------------------------
 * PB1：切换模式（IDLE → CIRCLE → LINEAR → CIRCLE …）
 * PA7：点动自检（前进→后退→左移→右移→顺时针→逆时针 轮流各一次）
 * PA5：停止（含电机急停）
 *
 * 【两种模式都先举高】
 *   进 CIRCLE 或 LINEAR 之前，都把升降电机（地址 6）抬到 VISION_LIFT_UP = 9000 脉冲，
 *   方向 0x01（CCW = 升）——机械臂低了摄像头看不见圆/环。
 *   上电时升降在最低位限位，所以 9000 = 从最低点抬起来。
 *   按 60 RPM 约 2.8 s。已经在目标高度就跳过，不再等一遍。
 *
 * 【CIRCLE 模式】把圆心拉到"夹爪位置"
 *   K230 回 C<x>,<y>,<r>，主控算 dx = x - VISION_TARGET_X、dy = y - VISION_TARGET_Y，
 *   然后平移补偿（2026-11-17 现场实测的方向关系）：
 *       小车前进 -> 圆心 x 变大   =>  dx>0 走**后退**，dx<0 走前进
 *       小车右移 -> 圆心 y 变大   =>  dy>0 走**左移**，dy<0 走右移
 *   |dx|、|dy| 都 ≤ VISION_DEADBAND_PX 就算对准，不动。
 *   像素→毫米沿用老标定：60 px = 35 mm，再打 VISION_STEP_GAIN_PCT 折扣，
 *   单步绝不超过 VISION_MAX_STEP_MM = 10 mm。
 *
 * 【为什么要慢 + 打折】位置模式是"发脉冲 → 硬起硬停"，每步一次冲击，会"抽一下"、
 *   把 K230 晃掉线、电源抖。所以平移/旋转都走速度模式（F6）：
 *   VISION_SPEED_X10 = 16 (1.6 RPM ≈ 6.6 mm/s，是原来 8 RPM 的 1/5)、
 *   VISION_ACCEL_RPM_S = 120，软起软停；步长再打 VISION_STEP_GAIN_PCT = 50% 折扣。
 *   识别不准时"补一半"比"补满"稳得多：补满必然冲过头，来回摆。
 *   想退回位置模式把 VISION_USE_SPEED_MODE 改 0。
 *
 * 【LINEAR 模式】看三环 → 修车身偏航
 *   发 "linear"，K230 回 L<k>,<psi>；psi 单位 0.01°：
 *   3) psi 单位 0.01°：正 = 顺时针（motor_rotate_centideg dir=0x00），
 *      负 = 逆时针（dir=0x01）。|psi| ≤ VISION_PSI_DEADBAND 就算对准。
 *      单次最多转 VISION_PSI_MAX_STEP，防止某一帧误检把车甩飞。
 *   4) 连着两次"越修越大"就判定符号反了 / 目标丢了：停下来报警，不再动。
 *
 * 【画面中心 320,240 与"夹爪偏置"】
 *   K230 采集是 640×480，图像中心恒为 (320,240)（不是 0,0）。
 *   但摄像头不在夹爪正上方，所以程序不把圆往 320,240 拉，而是往
 *   **VISION_TARGET_X / VISION_TARGET_Y**（夹爪投影到画面的像素位置）拉。
 *   这两个宏就在下面，标定方法写在它们前面的注释里 —— 要调偏置只改这两个数。
 *   ★ 若把 K230 分辨率改成 QVGA(320×240)，中心变 (160,120)，目标点也要跟着改。
 *
 * OLED：
 *   1 行  模式 + 状态（OK / WAIT / TMO）
 *   2 行  CIRCLE: X:318 Y:242      LINEAR: K+0.04 P+1.92
 *   3 行  N:<序号> NG:<次数>
 *   4 行  最后一次动作，例如 [BACK 12mm] / [CW 3.50deg] / [LIFT 10000]
 *
 * 安全：所有电机动作都带超时（judge_wait 有轮数上限），PA5 随时急停；
 *       举高过程（0→10000 约 7 s）内也每 10 ms 检查一次 PA5。
 * ========================================================================== */
#define VISION_IDLE        0u
#define VISION_CIRCLE      1u
#define VISION_LINEAR      2u
#define VISION_QUIET_MAX   250u     /* 每个循环约 4 ms，250 × 4 ms ≈ 1 s */

/* ============================================================================
 * ★★★ 摄像头 / 夹爪偏置：你要改的就是下面这两个数 ★★★
 * ----------------------------------------------------------------------------
 * 摄像头不在夹爪正上方，所以"圆心在画面正中"并不等于"夹爪在圆心正上方"。
 * 做法很简单：把目标点从画面中心挪到"夹爪在画面里的位置"。
 *
 *   VISION_TARGET_X / VISION_TARGET_Y = 夹爪投影到画面上的像素坐标
 *
 * 【怎么标定（最省事的办法）】
 *   1) 手推小车，让**夹爪正好对准圆心**（人眼对）；
 *   2) 这时看 K230 那边报的圆心坐标（IDE 打印或 OLED 第 2 行的 X、Y）；
 *   3) 把这两个数直接抄进 VISION_TARGET_X / VISION_TARGET_Y，重新烧录。
 *   之后程序会把圆心往这个点上拉，而不是往画面正中拉。
 *
 * 【一定要用毫米算的话】
 *   像素差 = 毫米 * 60 / 35（沿用"60 px = 35 mm"的老标定，但这是在某个相机
 *   高度下标出来的，高度一变比例就变，所以还是上面那个"抄坐标"的办法最准）。
 *   例：夹爪在摄像头右边 20 mm，标定出来约为 34 px，就把 X 改成 320 + 34。
 *
 * ★ 符号自己不用推：以 K230 报的坐标为准，正负它自然就对了。
 * ========================================================================== */
#define VISION_TARGET_X    470      /* ← 夹爪在画面里的 X（现场实测：470） */
#define VISION_TARGET_Y    170      /* ← 夹爪在画面里的 Y（现场实测：170） */

/* 画面中心（只用来显示"偏离多少"做参考，以及算默认目标点） */
#define VISION_CX          320
#define VISION_CY          240

/* ---- CIRCLE：把圆心拉到目标点 ---- */
/* 死区放宽到 15 px：识别本身有抖动，死区太小会一直来回修（看起来像走过头）。 */
#define VISION_DEADBAND_PX   15     /* ±15 px 以内不动（≈9 mm） */
#define VISION_MM_PER_60PX   35u    /* 老标定：60 px = 35 mm */
#define VISION_MAX_STEP_MM   6u     /* ★ 单步最多 6 mm：一点点挪，但别太碎 */
/* ★ 步长增益：只补"理论距离"的这个百分比。
 *   识别不准 + 相机慢（3 Hz）时，一次补满必然冲过头，所以打个折：
 *   这次补一半，剩下的下次再补 —— 收敛慢一点，但不会冲过去来回摆。
 *   走得过冲就调小（40）；半天不到位就调大（70）。 */
#define VISION_STEP_GAIN_PCT 50u
/* ★ 每走一步之后停多久（ms）：
 *   1) 等车体停稳、等 K230 拍下一帧清晰的；
 *   2) 让电源电压恢复 —— 连续动作的电流冲击叠起来最容易把 K230 拉到重启。
 *   这个停顿**不阻塞主循环**：照样收帧、照样刷屏，只是暂时不发新命令。 */
#define VISION_STEP_GAP_MS   150u
/* 梯形加减速本身要花的时间余量（估"这一步还要多久"时加上）。
 * ★ 必须和 VISION_POS_ACCEL 一起改：加速度越小，斜坡越长。
 *   accel=20 时 120 ms 够用；accel=8 要 400 ms 左右，否则会"上一步还在爬坡就发下一条"。 */
#define VISION_RAMP_MS       400u
/* 主循环一拍的周期（ms）：Delay_ms(4) + 按键扫描 + 偶尔刷屏。
 * 用来把"这一步要多少 ms"换算成"要等多少拍"。 */
#define VISION_LOOP_MS       4u

/* 方向映射（现场实测，翻了两次才对，记在这里别再改回去）：
 *   dx>0  ->  **前进**   （第一次按"前进让 x 变大"推成后退，实测反了；
 *                          2026-11-17 二次实测：dx>0 要走前进）
 *   dy>0  ->  **右移**   （同理，实测右移才对）
 * 注意这两个宏和 motor.c 里"前进/后退"的字面含义无关：
 *   如果哪天发现"日志说 BACK、车却往前走"，那说明 motor.c 的方向表反了
 *   （改 MOTOR_FLIP_FWD），不是这两个宏的事。
 *   **判断方法**：看串口那一行 `-> BACK 3mm`，对照车实际往哪走。 */
#define VISION_DX_PLUS_DO_BACK   0  /* 0: dx>0 前进 / dx<0 后退（实测值） */
#define VISION_DY_PLUS_DO_RIGHT  1  /* 1: dy>0 右移 / dy<0 左移（实测值） */
/* 旋转方向：psi>0 我们约定是"顺时针"，如果实测越修越大（发散），把它改 1 翻过来。
 * 另外还有一道保险：连续两轮 |psi| 变大就 DIVERGE STOP 停车，不会一直转。 */
#define VISION_YAW_INVERT        0  /* 0 = psi>0 顺时针；1 = psi>0 逆时针 */

/* ============================================================================
 * 运动方式：位置模式（FD）+ 梯形加减速  ← 当前方案
 * ----------------------------------------------------------------------------
 * 速度模式（F6）在这台车上晃得厉害，而且它的加减速由驱动器内部管，
 * 不好精确控制斜坡；位置模式是"发固定脉冲 + 指定速度和加速度档"，
 * 加减速由驱动器按梯形曲线走，速度/加速度都能压到很低，动作最干净。
 *
 * 三个量一起压低才有效（缺一个都会抽）：
 *   VISION_POS_SPEED_RPM  速度 8 RPM（默认 70）
 *   VISION_POS_ACCEL      加速度档 20（默认 200，越小斜坡越缓 = 梯形越平）
 *   VISION_MAX_STEP_MM    每次最多走 4 mm
 * 想再慢：speed 降到 5、accel 降到 10；但太小可能带不动（听到声音不走）。
 * ========================================================================== */
#define VISION_USE_SPEED_MODE   0     /* 0 = 位置模式（当前）；1 = F6 速度模式 */
#define VISION_POS_SPEED_RPM    8u    /* 位置模式速度，RPM */
#define VISION_POS_ACCEL        8u    /* 位置模式加速度档 0~255。20 → 8：
                                       * 斜坡更缓、起步电流更小，代价是"到位"更慢 */

/* 下面三个只在 VISION_USE_SPEED_MODE = 1 时才用（F6 速度模式那套参数） */
#define VISION_SPEED_X10        16u   /* 平移速度，0.1RPM：16 = 1.6 RPM */
#define VISION_ROT_SPEED_X10    20u   /* 旋转速度，0.1RPM：20 = 2.0 RPM */
#define VISION_ACCEL_RPM_S      120u  /* 加速度 RPM/s */

/* ---- 举高（两个模式都用）---- */
/* 升降电机的目标绝对位置（脉冲）。上电时升降在最低位限位，所以这就是"抬起来多少"。
 * 9000 脉冲按 60 RPM 约 2.8 s 走完。 */
#define VISION_LIFT_UP            9000u
/* 每 1000 脉冲大约要等多久：60 RPM ≈ 0.32 s，取 0.35 留余量。
 * ★ 这个值和 LIFT_MOTOR_SPEED 是绑在一起的，改那个就得改这个，否则等待和实际对不上。
 * 等待时间按"当前位置→目标位置"的差值算，已经在高度上就不等。 */
#define VISION_LIFT_MS_PER_1000P  550u
#define VISION_LIFT_WAIT_MAX      6000u  /* 单次最多等 6 s */
#define VISION_PSI_DEADBAND  60     /* 0.01°：|psi| ≤ 0.6° 算对准 */
#define VISION_PSI_MAX_STEP  2000   /* 单次最多 20° */
#define VISION_ALIGN_ROUNDS  8u     /* 最多调几轮 */
#define VISION_JUDGE_ROUNDS  60u    /* judge_wait 轮数上限（一轮约 220 ms） */

/* ---- K230 掉线自恢复 ----
 * K230 被电源拉重启后，会忘掉当前模式、停止回数据，主控这边只看到 TMO。
 * 静默超过 1 s 就自动重发一次模式命令，让它自己接上（不用人按按键）。
 * 每 VISION_RESEND_GAP 拍重发一次，最多 VISION_RESEND_MAX 次。 */
#define VISION_RESEND_MAX    5u
#define VISION_RESEND_GAP    500u   /* 拍数，约 2 s */

/* 整行写 16 个字符：先铺空格再写字，避免新旧数字宽度不同留下残影。
 *
 * ★ 为什么要缓存：
 *   OLED_ShowChar 是"上半 8 字节一次 I2C 事务，下半 8 字节又一次事务"，
 *   一旦事务多了就偶发失败 —— 表现就是"字只显示上半截"（下半那次没写进去）。
 *   静态行（比如最后那行按键提示）本来就不用每帧重画，缓存后 I2C 流量能省掉九成。 */
static char    vision_last[5][17];
static uint8_t vision_last_valid = 0u;

/* 点动标定结果：没有串口时就靠这个在屏上看"坐标变了多少" */
static int16_t  v_dbx = 0;              /* 上一次点动后 X 的变化（像素） */
static int16_t  v_dby = 0;              /* 上一次点动后 Y 的变化 */
static uint8_t  v_dx_ok = 0u;           /* 1 = 这两个数是真测到的（视觉有数据） */
static uint16_t v_calib_ticks = 0u;     /* 标定画面还要显示多少拍（0 = 不显示） */

/* 清屏 / 强制整屏重画之前调用：让缓存失效，否则会以为"已经画过了"而不写 */
static void vision_row_invalidate(void)
{
    vision_last_valid = 0u;
}

static void vision_row(uint8_t line, const char *text)
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

/* 把 0.01 单位的整数变成 "+1.92" / "-0.04" */
static void vision_fmt_2dp(char *out, int32_t v)
{
    int32_t a = (v < 0L) ? -v : v;

    sprintf(out, "%c%d.%02d", (v < 0L) ? '-' : '+',
            (int)(a / 100L), (int)(a % 100L));
}

/* 把整数变成 "+96" / "-12"（Keil 的 sprintf 对 %+d 支持不稳，自己拼） */
static void vision_fmt_signed(char *out, int32_t v)
{
    int32_t a = (v < 0L) ? -v : v;

    sprintf(out, "%c%d", (v < 0L) ? '-' : '+', (int)a);
}

/* ---------------- 动作层：这里才开始真的动电机 ---------------- */

/* 像素偏差 → 这一步实际走多少毫米。
 *   1) 按 60px = 35mm 算理论距离
 *   2) 乘 VISION_STEP_GAIN_PCT 打折（识别不准 + 相机 3 Hz，一次补满必然冲过头）
 *   3) 夹到 1 ~ VISION_MAX_STEP_MM（单步绝对不会超过这个数）
 * px=0 返回 0（不动）。
 *
 * 注意顺序：先打折、再夹上限。反过来的话"上限 10mm × 50%"就只剩 5mm 了，
 * VISION_MAX_STEP_MM 这个数就失去意义。 */
static uint32_t vision_px_to_step(int32_t px)
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

/* 急停：4 个底盘电机 + 升降电机，全发 [addr] FE 98 00 6B。
 * 速度模式下再补一个 Chassis_Stop()（按加速度斜坡停车），双保险。 */
static void vision_stop_all(void)
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

/* ---------------- 三个"动一下"的封装：速度模式 / 位置模式在这里切换 ---------------- */

/* 前后平移：mm > 0 前进，mm < 0 后退 */
static void vision_move_fwd(int32_t mm)
{
    if(mm == 0L) return;
#if VISION_USE_SPEED_MODE
    (void)Chassis_MoveAt(mm, (uint16_t)VISION_SPEED_X10);
#else
    if(mm > 0L) qianjin_((uint32_t)mm);
    else        houtui_((uint32_t)(-mm));
#endif
}

/* 左右平移：mm > 0 左移，mm < 0 右移（chassis.h 的约定） */
static void vision_move_lat(int32_t mm)
{
    if(mm == 0L) return;
#if VISION_USE_SPEED_MODE
    (void)Chassis_StrafeAt(mm, (uint16_t)VISION_SPEED_X10);
#else
    if(mm > 0L) zuo_((uint32_t)mm);
    else        you_((uint32_t)(-mm));
#endif
}

/* 旋转：centi > 0 顺时针，centi < 0 逆时针（单位 0.01°）。
 * 注意 chassis.h 的约定是"度，>0 逆时针"，和这里正好相反，所以取负。
 * VISION_YAW_INVERT=1 时整体翻转（实测方向反了就用它，不用改别的地方）。 */
static void vision_rotate(int32_t centi)
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

/* 一步大概要多久（ms）：脉冲数 ÷ 每秒脉冲 + 梯形斜坡余量 + 停顿。
 * 用来把"这一步要多少 ms"换算成主循环要等多少拍（见 VISION_LOOP_MS）。
 * 只是个估算，但足够避免"前一步还没走完就叠下一条命令"。 */
static uint16_t vision_step_ms(int32_t mm)
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

/* CIRCLE 一步：**不阻塞**。发完命令立刻返回，主循环接着收帧/刷屏。
 *
 * 返回 = 这一步大概需要多少 ms（0 = 已在死区，没动）。
 * 调用方用它决定"多少拍之内不发新命令"，而不是傻等。
 *
 * 一次只动一个轴（电流最小、最不抽），优先修偏差大的那个。
 * 这样两个轴会交替收敛，比"永远先修 x 再修 y"快。 */
static uint16_t vision_circle_issue(int32_t cx, int32_t cy, char *act)
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


/* 旋转一步大概要多久（ms）：脉冲数 ÷ 每秒脉冲 + 梯形斜坡 + 停顿。
 * 脉冲/度 = MOTOR_PULSE_PER_360DEG / 360 = 17208/360 ≈ 47.8。
 * 注意 p_centi 是 0.01°，所以除 36000。 */
static uint16_t vision_rot_ms(uint32_t p_centi)
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

/* LINEAR 一步：**不阻塞**，返回这一步大概要多久（0 = 已在死区）。
 *
 * ★ 这里不再等 0x3A 回包。原因：0x3A 只说明"驱动器回没回应"，和
 *   "命令有没有发出去"是两回事 —— 用它当失败条件就会出现
 *   "电机明明在动，OLED 却一直显示 MOTOR TIMEOUT"。
 *   想确认驱动器在不在，用 PA7 点动（那条专门查 0x3A）。 */
static uint16_t vision_yaw_issue(int16_t psi_centi, char *act)
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

/* PA7 点动自检：每按一次做一种动作，轮流覆盖 前进/后退/左移/右移/顺时针/逆时针。
 * 用途：接视觉之前先确认"车到底动不动、每个方向对不对"。
 * 幅度都很小（50 mm / 15°），但轮子悬空时看不出方向，所以最好架空或垫起来试。
 * 返回 1 正常 / 0 电机没回报到位。 */
#define VISION_JOG_MM    50u
#define VISION_JOG_DEG   15u
#define VISION_JOG_COUNT 6u

static uint8_t vision_jog(uint8_t idx, char *act)
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

/* 举高机械臂到 target 脉冲（绝对位置），方向 0x01 = CCW = 升。
 * ★ 已经在目标高度就**直接返回**，不重发也不白等 —— CIRCLE ⇄ LINEAR 来回切
 *   的时候很常见（两个模式都要举到同一个高度）。
 * 等待时间按"当前位置 → 目标位置"的差值估算，每 10 ms 查一次 PA5，随时能中止。
 * 返回 1 = 到位（或本来就在），0 = 被 PA5 中止。 */
static uint8_t vision_lift_to(uint32_t target, char *act)
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

static void vision_link_test(void)
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


/* 上电用 PC13 闪几下报告 OLED 探测结果（OLED 不亮、串口也关了，就靠这个判断）：
 *   1 下 = 面板有应答（I2C 通了，只是显示内容没刷出来 → 看对比度/地址）
 *   3 下 = 面板没应答（接线 / 上拉 / 供电 / 模块坏）
 */
static void boot_oled_indicator(uint8_t oled_ok)
{
    uint8_t n = oled_ok ? 1u : 3u;
    uint8_t i;

    for(i = 0; i < n; i++){
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);   /* 亮（低电平点亮） */
        Delay_ms(150);
        GPIO_SetBits(GPIOC, GPIO_Pin_13);     /* 灭 */
        Delay_ms(150);
    }

    Delay_ms(500);
}


int main(void)
{
    u16 cur_x = PATH_START_X;
    u16 cur_y = PATH_START_Y;
    u8 step = 0;
	
	s32 yaw_centi_degrees = 0, last_yaw_centi_degrees = 0;

    /* 【第 0 步】先把 SystemCoreClock 修正成芯片的真实频率（读 RCC 寄存器推算）。
     * HSE 晶振没起振时芯片跑 HSI 8 MHz，而默认变量还写着 72 MHz，
     * 不修正的话后面所有 Delay_ms 都会慢 9 倍。 */
    clk_fix();

    /* 【第 1 步】PC13/PC14/PC15 直接输出高电平：只要程序跑起来，这三个脚立刻是高 */
    pc_out_high_init();

    LED_Init();
    Servo_Init();             /* PA1(旋转)/PA0(夹爪) 两路舵机 PWM，上电输出 30°/85°（PA2 弃用） */
    OLED_Init();              /* 硬件 I2C1：PB6=SCL / PB7=SDA；探测不到就不画 */
    boot_oled_indicator(OLED_IsReady());   /* 闪 1 下 = OLED 应答，3 下 = 没应答 */
    Key_Init();               /* 按键 PB1 / PA7 / PA5（下拉输入，按下为高） */
    Serial_Init();            /* USART1：电机总线（也是串口烧录口） */
    Gyro_Init();
    USART3_Init(115200);      /* 调试串口：PB10=TX / PB11=RX，115200 8N1 */
    USART2_Init(115200);      /* 陀螺仪 / K230：只收（PA3），PA2 已弃用 */
	
	
	/* 电机上电默认失能（F3 使能帧之前所有运动指令都会被忽略），
	 * 任何会驱动底盘的模式都必须先使能，否则车一动不动。 */
	motor_enable_all();

	/* 时钟情况 + OLED 是否在线，都只打串口：
	 * SYSCLK 显示 HSI/8 MHz、HSE_RDY=0 就是晶振没起振（延时慢 9 倍的原因）。 */
	clk_report();

	/* ===== USART2 引脚自检（PA2=TX / PA3=RX 是不是好的）=====
	 *  1) 电气：PA2/PA3 上拉/下拉是否跟随 → OLED 第 2 行
	 *  2) 自环：把 PA2 和 PA3 用杜邦线短接，通过则第 3 行 LOOP:OK，PC13 闪 2 下
	 *  3) 在线：每 0.5 s 往 PA2 发一行 "PA2TEST"，同时统计 PA3 收到的字节
	 * 注意：它是**死循环**，一旦调用就进不了下面的调试模式。
	 * 已经实测确认 PA2/PA3 都正常，所以默认注释掉；要复测再打开这一行。 */
	// usart2_pin_test();

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
#elif DEBUG_MODE == DEBUG_MODE_VISION
	vision_link_test();         /* 视觉链路：发 circle/linear，OLED 显示 K230 回的数据 */
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
