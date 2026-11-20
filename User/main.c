#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <stdio.h>
#include <string.h>
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
#include "lift_motor.h"
#include "motion.h"
#include "vision.h"


/* 陀螺仪闭环转向的宏和过程量已搬到 Hardware/Gyro.h 与 Gyro.c */


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
#define DEBUG_MODE_UART_TEST 9   /* ★ 串口发送方向测试：只往选定的串口发，不看收 */
#define DEBUG_MODE_GRAB_ALIGN 10 /* ★ PC2 触发：读 K230 坐标 → 对准 → 夹一次 → 归位 */
#define DEBUG_MODE_SERVO_TEST 11 /* ★ 舵机单独测试：PC9/PC8 + PA0 同时出脉宽，PC2/PC3 加减角度 */
#define DEBUG_MODE_CAR_PUT 12    /* ★ 车上取料→放地上：半自动（PC3 启动/视觉时 PC3 确认、PC2 单步、PA5 中止） */
#define DEBUG_MODE_LIFT_TEST 13  /* ★ 升降电机高度调试：PB1/PA7 升降、PC2 换步长、PC3 回零 */
#define DEBUG_MODE_LIFT_RAW 14   /* ★ 升降原始帧判定测试：5×20 累计判定相对/绝对（最保守） */
#define DEBUG_MODE_SERVO3 15     /* ★ 三舵机标定：PC0 换对象、PC2/PC3 正反方向 */
#define DEBUG_MODE_ARM_ALL 16    /* ★ 整臂联动标定：升降步进 + 三舵机，一个界面全调 */

/* 默认模式。平时只改这一行。
 * 也可以从编译器覆盖（不改文件就能切模式，方便把所有模式一次性都编一遍）：
 *   gcc -DDEBUG_MODE=3
 *   Keil Options → C/C++ → Define 里填 DEBUG_MODE=3 */
#ifndef DEBUG_MODE
#define DEBUG_MODE          DEBUG_MODE_CAR_PUT
#endif

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

        if(Key_PA8Hit()){ abort_flag = 1u; return 0u; }
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
        if(step <= ROUTE_LEG_COUNT && Key_PA8Hit()) step = 97u;

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
            OLED_ShowCentiDeg(3, 5, turn_dbg_err);
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
    OLED_ShowCentiDeg(3, 5, turn_dbg_meas);

    OLED_ShowString(4, 1, "ERR");
    OLED_ShowCentiDeg(4, 5, turn_dbg_err);
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
        if(Key_PA8Hit()){
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
 * 串口「只发」测试（DEBUG_MODE_UART_TEST = 9）
 * ----------------------------------------------------------------------------
 * 目的：单独验证 **F103 的某一串口能不能把数据发出去**，不看接收。
 *       接 K230、接 USB-TTL、接串口助手都行，只要对端能打出 "K230TEST #0001"
 *       就说明这条链路（引脚 + 波特率 + 交叉接法）是通的。
 *
 * ★ 先选端口：改下面的 UART_TEST_PORT
 *     1 = USART1  PA9 (TX)      ← 注意 PA9/PA10 是步进电机总线兼烧写口，别乱接
 *     2 = USART2  PA2 (TX)      ← 陀螺仪 JY901 那一路（乱发会打断陀螺仪帧）
 *     3 = USART3  PB10 (TX)     ← 无线调试模块（Serial.c 里 SERIAL_USART3_ENABLE = 1）
 *     4 = UART4   PC10 (TX)     ← ★ K230 那一路（PC10/PC11 是 UART4 固定引脚）
 *     5 = USART2 + USART3 两路同时发（不确定该接哪根时用这个，接哪根都能看到）
 *
 * ⚠️ 关于 PC10/PC11（本板是 STM32F103RCT6 / LQFP64，PC0~PC15 齐全，可用）：
 *   PC10/PC11 是 **UART4** 的固定引脚（独立外设，中断号 52），**不做任何重映射**。
 *   前提：启动文件 startup_stm32f10x_hd.s + 宏 STM32F10X_HD，否则发不出去。
 *   若哪天换回 48 脚的 C8T6（只有 PC13/14/15），端口 4 会"程序照发、引脚无输出"，
 *   那时改用端口 3（PB10）。
 *
 * 接线（永远交叉）：F103 的 TX → 对方 RX，对方 TX → F103 的 RX，GND 共地。
 *   端口 4：PC10(TX) → 对方 RX，PC11(RX) ← 对方 TX
 *
 * 界面：
 *   第 1 行  UART SEND TEST
 *   第 2 行  PORT:UART4 PC10    （当前选的端口，一眼看出烧的是哪版）
 *   第 3 行  SENT:000123        （已发多少行，数字一直在涨 = 主循环活着）
 *   第 4 行  8N1 TXE:01         （TXE = 发送寄存器空，1 = 串口真在往外出数据）
 * PC13 每发一行闪一次，肉眼就能确认 MCU 在跑。
 * ========================================================================== */
#define UART_TEST_PORT       3        /* ★ 1/2/3/4/5，见上面说明 */
#define UART_TEST_INTERVAL   500u     /* 每多少 ms 发一行 */
#define UART_TEST_BAUD       115200u

#if   UART_TEST_PORT == 1
  #define UART_TEST_PORT_NAME "PORT:USART1 PA9 "
#elif UART_TEST_PORT == 2
  #define UART_TEST_PORT_NAME "PORT:USART2 PA2 "
#elif UART_TEST_PORT == 3
  #define UART_TEST_PORT_NAME "PORT:USART3 PB10"
#elif UART_TEST_PORT == 4
  #define UART_TEST_PORT_NAME "PORT:UART4 PC10"
#elif UART_TEST_PORT == 5
  #define UART_TEST_PORT_NAME "PORT:U2+U3 BOTH "
#else
  #error "UART_TEST_PORT must be 1, 2, 3, 4 or 5"
#endif

/* 按选定的端口发一个字节 */
static void uart_test_putc(uint8_t b)
{
#if   UART_TEST_PORT == 1
    Serial_SendByte(b);
#elif UART_TEST_PORT == 2
    USART2_SendByte(b);
#elif UART_TEST_PORT == 3
    USART3_SendByte(b);
#elif UART_TEST_PORT == 4
    /* UART4 直接发（K230 那条线），不经过任何调试口重定向 */
    USART_SendData(UART4, b);
    while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
#else   /* 5 = 两路同时发：PA2 和 PB10 都有数据，接哪根都能看到 */
    USART2_SendByte(b);
    USART3_SendByte(b);
#endif
}

/* 发一整行 "K230TEST #00001\r\n" */
static void uart_test_send_line(uint16_t n)
{
    const char *p = "K230TEST #";
    char buf[6];
    uint8_t i = 5u;
    uint16_t v = n;

    while(*p != '\0') uart_test_putc((uint8_t)*p++);

    /* 5 位十进制，前导零 */
    buf[5] = '\0';
    do{
        buf[--i] = (char)('0' + (v % 10u));
        v /= 10u;
    }while(v != 0u && i > 0u);
    while(i > 0u) buf[--i] = '0';

    p = buf;
    while(*p != '\0') uart_test_putc((uint8_t)*p++);

    uart_test_putc('\r');
    uart_test_putc('\n');
}

/* 串口只发测试主循环：不返回 */
static void uart_test_run(void)
{
    uint16_t sent = 0u;
    uint8_t  tb   = 0u;

    OLED_Clear();
    OLED_ShowString(1, 1, "UART SEND TEST  ");
    OLED_ShowString(2, 1, UART_TEST_PORT_NAME);
    OLED_ShowString(3, 1, "SENT:00000      ");
    OLED_ShowString(4, 1, "115200 8N1      ");

    USART3_Printf("\r\n[UART TEST] %s, %u 8N1\r\n",
                  UART_TEST_PORT_NAME, (unsigned)UART_TEST_BAUD);

    while(1){
        uart_test_send_line((uint16_t)(sent + 1u));
        sent++;

        PC13_Turn();                    /* PC13 每发一行闪一次 */

        /* TXE（发送数据寄存器空）说明串口真的在往外吐数据；
         * 如果 TXE 一直是 0，说明串口根本没使能 / 波特率算错。 */
#if UART_TEST_PORT == 1
        tb = (USART_GetFlagStatus(USART1, USART_FLAG_TXE) != RESET) ? 1u : 0u;
#elif UART_TEST_PORT == 2
        tb = (USART_GetFlagStatus(USART2, USART_FLAG_TXE) != RESET) ? 1u : 0u;
#else
        tb = (USART_GetFlagStatus(USART3, USART_FLAG_TXE) != RESET) ? 1u : 0u;
#endif

        OLED_ShowNum(3, 6, (uint32_t)sent, 5);
        OLED_ShowString(4, 1, "8N1 TXE:        ");
        OLED_ShowNum(4, 9, (uint32_t)tb, 2);

        Delay_ms(UART_TEST_INTERVAL);
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

/* 升降电机的地址/速度/加速度/限位等宏在 app/lift_motor.h（LIFT_MOTOR_*） */

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
 *   旋转：30°  = 夹物料位置（初始，对台面/转盘）
 *         163° = 放物料位置（对车上料仓）★ 2026-11 重新实测
 *   夹爪：85°  = 松开                   115° = 夹紧
 *
 * ★ 关于 163°：换到 PC8/PC9 之后机械臂重新装过，原来那个 115° 已经不对了。
 *   用 DEBUG_MODE=11（舵机精调）从 30° 一路往上加，找到"升降抬起后夹爪正对车上料仓"
 *   的那个点，串口打出来是 deg=163（pulse=1293us）。
 *   同一个物理位置，新的"从车上夹下来"流程（DEBUG_MODE=12）也用它。
 *
 *   1) 夹爪 85 → 115      夹起物料
 *   2) 升降电机 → 2000    把物料抬起（绝对位置、CCW）
 *   3) 旋转 30 → 163      转到车上料仓
 *   4) 夹爪 115 → 85      放开物料（不压栈）
 * 复位（压栈弹栈）：旋转→30 → 升降电机→0 → 夹爪→85，倒序退回。 */
#define SERVO_ROT_PLACE_DEG   163u   /* 转到车上料仓的角度（实测；旧值 115 已作废） */

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
/* g_lift_target 已经搬到 app/lift_motor.c，声明在 lift_motor.h */

/* 按键极性和 grab_key_* 系列函数已搬到 Hardware/Key.c，声明在 Key.h */

/* 等松手，最多等 2 s（400 × 5 ms）。加超时是为了防止某个按键脚被外部一直拉高，
 * 让程序永远卡在"等松手"里，表现成"按键全都没反应"。返回 1 = 真的松手了。 */

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
    OLED_ShowNum(2, 5, rot, OLED_Digits(rot));

    /* 第 3 行：GRP:85 M:2000（M = 升降电机目标脉冲） */
    OLED_ShowString(3, 1, "                ");
    OLED_ShowString(3, 1, "GRP:");
    OLED_ShowNum(3, 5, grip, OLED_Digits(grip));
    OLED_ShowString(3, 9, "M:");
    OLED_ShowNum(3, 11, g_lift_target, OLED_Digits(g_lift_target));
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
        OLED_ShowNum(4, 4, tx_lines, OLED_Digits(tx_lines));
        OLED_ShowNum(4, 11, rx_bytes, OLED_Digits(rx_bytes));
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
    OLED_ShowNum(3, 10, count, OLED_Digits(count));
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


/* 视觉闭环的宏和说明已搬到 app/vision.h */


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


/* ============================================================================
 * 对准 → 夹一次 → 归位（DEBUG_MODE_GRAB_ALIGN = 10）
 * ----------------------------------------------------------------------------
 * 硬件（本板 STM32F103RCT6 / LQFP64）：
 *     K230     PA2(USART2_TX) / PA3(USART2_RX)，115200 8N1
 *     调试     PB10(USART3_TX) / PB11(USART3_RX) → 无线串口模块
 *     旋转舵机 PC9 (TIM3_CH4)      夹爪舵机 PC8 (TIM3_CH3)
 *     触发键   PC2（上拉输入，按下接地 = 读到低电平）
 *     中止键   PA5（沿用夹取流程的按键，按下为高）
 *     状态灯   PC13
 *
 * 按一下 PC2 走一遍：
 *     ① 举高机械臂到 VISION_LIFT_UP（摄像头得先看得见地面目标）
 *     ② 给 K230 发 "circle"
 *     ③ 闭环对准：把圆心拉到夹爪投影点 (VISION_TARGET_X/Y)，
 *        连续 GA_ALIGN_STABLE 帧进死区才算"定好位置"
 *     ④ 跑一次夹取动作（夹紧 → 抬起 → 转到车上 → 松开）
 *     ⑤ 自动走撤销栈归位（旋转回 30°、升降回 0、夹爪回 85°）
 *     ⑥ 回待机，等下一次 PC2
 *
 * 安全与兜底：
 *   - 对准最多 GA_ALIGN_MAX_MS，超时停手报 TIMEOUT
 *   - PA5 任何时刻按下：四轮 + 升降急停，本轮回待机
 *   - K230 静默 GA_RESEND_MS 自动重发 "circle"（最多 GA_RESEND_MAX 次），
 *     防止它被电源拉重启后自己忘了当前模式、主控一直干等
 *
 * OLED：
 *     1 行  GRAB+ALIGN <状态>      2 行  C:470 Y:170
 *     3 行  N:12 BAD:0             4 行  [ALIGN 4mm] / [FWD 2/4] / [HOME]
 * ========================================================================== */
#define GA_ALIGN_MAX_MS   25000u   /* 对准阶段总超时 */
#define GA_ALIGN_STABLE   3u       /* 连续几帧在死区才算对准 */
#define GA_RESEND_MS      2000u    /* K230 静默多久重发一次 circle */
#define GA_RESEND_MAX     5u
#define GA_LOOP_MS        10u      /* 主循环一拍 */
#define GA_SHOW_MS        200u     /* 刷屏间隔 */

#define GA_IDLE    0u
#define GA_ALIGN   1u
#define GA_GRAB    2u
#define GA_DONE    3u
#define GA_FAIL    4u

/* ★ PC2 按键极性：
 *   1 = 上拉输入、按下接地（读到低电平 = 按下）—— 和本工程 PC1/PC2/PC3
 *       按键自检里"上拉输入，按下接地"的接法一致，默认用这个。
 *   0 = 按下接 3.3V（读到高电平 = 按下）。按了没反应就把这个改成 0。 */
#define GA_PC2_ACTIVE_LOW   1

/* PC2 上拉输入：按下接地 */
static void ga_pc2_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_2;
#if GA_PC2_ACTIVE_LOW
    gpio.GPIO_Mode  = GPIO_Mode_IPU;      /* 上拉，按下接地 */
#else
    gpio.GPIO_Mode  = GPIO_Mode_IPD;      /* 下拉，按下接 3.3V */
#endif
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);
}

static uint8_t ga_pc2_raw(void)
{
#if GA_PC2_ACTIVE_LOW
    return (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == Bit_RESET) ? 1u : 0u;
#else
    return (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == Bit_SET)   ? 1u : 0u;
#endif
}

/* 消抖 20 ms + 等松手（最多 2 s）：按一下只返回一次 1 */
static uint8_t ga_pc2_hit(void)
{
    uint16_t n;

    if(!ga_pc2_raw()) return 0u;

    for(n = 0u; n < 20u; n++){
        Delay_ms(1);
        if(!ga_pc2_raw()) return 0u;          /* 抖动，当没按 */
    }
    for(n = 0u; n < 400u; n++){               /* 等松手，2 s 超时防卡死 */
        if(!ga_pc2_raw()) break;
        Delay_ms(5);
    }
    Delay_ms(20);
    return 1u;
}

/* 第 1 行状态词 */
static char *ga_state_name(uint8_t st)
{
    switch(st){
        case GA_ALIGN: return (char *)"ALIGN ";
        case GA_GRAB:  return (char *)"GRAB  ";
        case GA_DONE:  return (char *)"DONE  ";
        case GA_FAIL:  return (char *)"FAIL  ";
        default:       return (char *)"READY ";
    }
}

static void ga_show(uint8_t st, char *act)
{
    OLED_ShowString(1, 1, "GRAB+ALIGN      ");
    OLED_ShowString(1, 11, ga_state_name(st));      /* 6 字符，占 11~16 列，不能写到 17 */

    OLED_ShowString(2, 1, "C:470 Y:170");
    OLED_ShowString(2, 1, "C:");
    OLED_ShowSignedNum(2, 3, (int32_t)K230_Cx, 4);
    OLED_ShowString(2, 8, "Y:");
    OLED_ShowSignedNum(2, 10, (int32_t)K230_Cy, 4);
    OLED_ShowString(2, 14, "   ");

    OLED_ShowString(3, 1, "                ");
    OLED_ShowString(3, 1, "N:");
    OLED_ShowNum(3, 3, (uint32_t)K230_CircleSeq, 5);
    OLED_ShowString(3, 9, "BAD:");
    OLED_ShowNum(3, 13, (uint32_t)(K230_BadLine % 100u), 2);

    OLED_ShowString(4, 1, "                ");
    OLED_ShowString(4, 1, act);
}

/* 对准 + 夹取主循环：不返回 */
static void grab_align_run(void)
{
    uint8_t  st       = GA_IDLE;
    uint32_t cseq     = 0u;
    uint32_t t_ms     = 0u;
    uint32_t t_show   = 0u;
    uint32_t silent   = 0u;
    uint8_t  stable   = 0u;
    uint8_t  retry    = 0u;
    uint16_t wait_ms  = 0u;
    char     act[17];

    ga_pc2_init();
    grab_key_init();                       /* PB1/PA7/PA5：这里只用 PA5 中止 */

    /* 位置模式降到很慢 + 软斜坡（和 vision 一致，别删） */
    motor_set_profile((uint16_t)VISION_POS_SPEED_RPM, (uint16_t)VISION_POS_ACCEL);

    OLED_Clear();
    sprintf(act, "PC2 = GRAB");
    ga_show(GA_IDLE, act);

    USART3_SendString("\r\n[GRAB+ALIGN] PC2=start  PA5=stop\r\n");
    USART3_Printf("[GRAB+ALIGN] 目标点(夹爪)=(%d,%d)  死区=±%d px  单步≤%u mm\r\n",
                  VISION_TARGET_X, VISION_TARGET_Y,
                  VISION_DEADBAND_PX, (unsigned)VISION_MAX_STEP_MM);
    USART3_Printf("[GRAB+ALIGN] 舵机: PC9=旋转 CH4 / PC8=夹爪 CH3\r\n");
    USART3_Printf("[GRAB+ALIGN] K230: USART2 (PA2/PA3) 115200\r\n");
    USART3_Printf("[GRAB+ALIGN] 调试: USART3 (PB10/PB11) 无线串口\r\n");

    while(1)
    {
        /* ---------- PA5：任何时候都能急停 ---------- */
        if(grab_abort_hit()){
            vision_stop_all();
            lift_motor_stop();
            sprintf(act, "STOP BY PA5");
            USART3_SendString("[GRAB+ALIGN] PA5 -> 全部急停，回待机\r\n");
            st = GA_IDLE;
            ga_show(st, act);
            grab_wait_release(GPIOA, GPIO_Pin_5);
            Delay_ms(200);
            continue;
        }

        switch(st)
        {
        /* ================= 待机：等 PC2 ================= */
        case GA_IDLE:
            if(ga_pc2_hit()){
                motor_enable_addr(LIFT_MOTOR_ADDR);   /* 地址 6 上电默认失能 */

                sprintf(act, "LIFTING...");
                ga_show(GA_IDLE, act);

                /* ① 举高（阻塞，内部会查 PA5） */
                if(!vision_lift_to(VISION_LIFT_UP, act)){
                    sprintf(act, "LIFT ABORT");
                    st = GA_FAIL;
                    ga_show(st, act);
                    break;
                }

                /* ② 发 circle，准备对准 */
                cseq    = K230_CircleSeq;
                stable  = 0u;
                retry   = 0u;
                t_ms    = 0u;
                t_show  = 0u;
                silent  = 0u;
                wait_ms = 0u;
                K230_SendLine(K230_CMD_CIRCLE);
                USART3_SendString("[GRAB+ALIGN] TX circle，开始对准\r\n");

                sprintf(act, "WAIT K230...");
                st = GA_ALIGN;
                ga_show(st, act);
            }
            break;

        /* ================= 对准：把圆心拉到夹爪下 ================= */
        case GA_ALIGN:
        {
            uint16_t need;

            t_ms += GA_LOOP_MS;

            /* 上一步还在走：先等它走完，别叠命令 */
            if(wait_ms > 0u){
                wait_ms = (wait_ms > GA_LOOP_MS)
                          ? (uint16_t)(wait_ms - GA_LOOP_MS) : 0u;
                break;
            }

            if(K230_CircleSeq != cseq){
                cseq    = K230_CircleSeq;
                silent  = 0u;
                retry   = 0u;
                stable  = 0u;                 /* 新一帧重新计稳定 */

                need = vision_circle_issue((int32_t)K230_Cx, (int32_t)K230_Cy, act);

                if(need == 0u){
                    /* 已经在死区里：连续 N 帧都这样才算定好位置 */
                    if(stable < 255u) stable++;
                    wait_ms = 0u;
                }else{
                    wait_ms = need;
                }

                if(stable >= GA_ALIGN_STABLE){
                    USART3_Printf("[GRAB+ALIGN] 对准完成 (%d,%d)，开始夹取\r\n",
                                  (int)K230_Cx, (int)K230_Cy);
                    K230_SendLine(K230_CMD_STOP);

                    /* ★ 关键：对准时机械臂是抬高的（VISION_LIFT_UP），
                     *   而夹取动作表 k_grab_steps 是按"上电在最低位 0"设计的
                     *   （第 1 步就是合爪）。所以必须先降回 LIFT_MOTOR_HOME，
                     *   否则会在半空中闭爪，什么都没夹到。 */
                    sprintf(act, "LOWERING...");
                    ga_show(st, act);
                    if(!vision_lift_to(LIFT_MOTOR_HOME, act)){
                        sprintf(act, "LIFT ABORT");
                        st = GA_FAIL;
                        ga_show(st, act);
                        break;
                    }

                    grab_reset_state();
                    sprintf(act, "ALIGNED");
                    st = GA_GRAB;
                    ga_show(st, act);
                    break;
                }
            }

            /* K230 静默重发 */
            silent += GA_LOOP_MS;
            if(silent >= GA_RESEND_MS){
                silent = 0u;
                if(retry < GA_RESEND_MAX){
                    retry++;
                    K230_SendLine(K230_CMD_CIRCLE);
                    USART3_Printf("[GRAB+ALIGN] K230 静默，重发 circle（第 %u 次）\r\n",
                                  (unsigned)retry);
                }
            }

            /* 超时 */
            if(t_ms >= GA_ALIGN_MAX_MS){
                vision_stop_all();
                sprintf(act, "ALIGN TMO");
                USART3_Printf("[GRAB+ALIGN] 对准超时 %u ms，放弃\r\n",
                              (unsigned)GA_ALIGN_MAX_MS);
                K230_SendLine(K230_CMD_STOP);
                st = GA_FAIL;
                ga_show(st, act);
                break;
            }

            /* 刷屏 */
            if((t_ms - t_show) >= GA_SHOW_MS){
                t_show = t_ms;
                ga_show(st, act);
            }
            break;
        }

        /* ================= 夹取：正向 4 步 + 撤销栈归位 ================= */
        case GA_GRAB:
            if(g_grab_phase == GRAB_PH_DONE){
                sprintf(act, "HOME");
                USART3_SendString("[GRAB+ALIGN] 夹取完成，已归位\r\n");
                st = GA_DONE;
                ga_show(st, act);
                break;
            }

            if(!grab_step_once()){            /* 返回 0 = 被 PA5 中止 */
                vision_stop_all();
                lift_motor_stop();
                sprintf(act, "ABORT");
                st = GA_FAIL;
                ga_show(st, act);
                break;
            }

            pc13_step_blink();

            if(g_grab_phase == GRAB_PH_FWD){
                sprintf(act, "FWD %u/%u",
                        (unsigned)g_fwd_done, (unsigned)GRAB_STEP_COUNT);
            }else{
                sprintf(act, "HOME %u/%u",
                        (unsigned)g_undo_done, (unsigned)g_undo_total);
            }
            ga_show(st, act);
            break;

        /* ================= 完成 / 失败：都回待机 ================= */
        case GA_DONE:
        case GA_FAIL:
            Delay_ms(600);
            sprintf(act, "PC2 = GRAB");
            st = GA_IDLE;
            ga_show(st, act);
            break;

        default:
            st = GA_IDLE;
            break;
        }

        Delay_ms(GA_LOOP_MS);
    }
}

/* ============================================================================
 * 舵机单独测试 / 旋转角度精调（DEBUG_MODE_SERVO_TEST = 11）
 * ----------------------------------------------------------------------------
 * 用途：**精细调旋转舵机的角度**，并把角度信息实时打到**无线串口**
 *       （USART3 = PB10/PB11，115200 8N1）。
 *
 * 每改 1° 就在无线串口打一行，同时每秒打一次"当前角度"心跳，
 * 方便边拧边记：看到机械臂指到想要的位置时，把那一行的 deg 抄下来告诉我即可。
 *
 *      [SERVO] SET deg=30 pulse=722us duty=3.6% C4=722 step=1
 *      [SERVO] NOW deg=30 pulse=722us duty=3.6% C4=722 step=1
 *
 * 输出脚（两路同时输出同一脉宽，冗余备份）：
 *
 *      PC9 (TIM3_CH4)  ← Servo.c 配的，正常工程用的那根
 *      PA0 (TIM2_CH1)  ← 备用脚，独立定时器（不接也没关系）
 *
 *   【判据｜万用表】黑表笔 GND，红表笔量 PC9，DC 档读到的是占空比 × 3.3 V：
 *         0°   → 500 us  → 2.5%  → 约 0.08 V
 *         30°  → 722 us  → 3.6%  → 约 0.12 V   ← 上电初始位置
 *         135° → 1500 us → 7.5%  → 约 0.25 V
 *         270° → 2500 us → 12.5% → 约 0.41 V
 *
 *   【判据｜屏上数字】第 3 行是直接读回定时器寄存器 CCR：
 *       C4 = TIM3->CCR4（PC9 用的）、C2 = TIM2->CCR1（PA0 用的）
 *       跟着角度变 → 外设确实被写进去了，问题在更外面（引脚/线/舵机）
 *
 * 按键：
 *     PC2   角度 +step（按住连加）
 *     PC3   角度 -step（按住连减）
 *     PA5   回初始位置（旋转舵机 = 30°，即 SERVO_ROT_INIT_DEG）
 *     PA7   切换步长：1° → 5° → 10° → 45° → 1°（循环）
 *
 * OLED：
 *     1 行  SERVO PC9 ST:1     （ST: 当前步长）
 *     2 行  DEG 030 P 0722     （逻辑角度 / 实际脉宽 us）
 *     3 行  C4 0722 C2 0722    （TIM3->CCR4 / TIM2->CCR1 寄存器读回）
 *     4 行  PC2+ PC3- PA5=R    （PA7 换步长，屏上没位置了）
 *
 * ★ 要测夹爪舵机（PC8）就把下面 ST_SERVO_ID 改成 1。
 *   注意夹爪初始位是 85°，别拿它去撞机械限位。
 * ========================================================================== */
#define DEBUG_MODE_SERVO_TEST 11

#define ST_SERVO_ID        0       /* ★ 0 = 旋转舵机 PC9(TIM3_CH4)   1 = 夹爪舵机 PC8(TIM3_CH3) */
#define ST_HOLD_MS         150u    /* 按住时每多少 ms 走一步 */
#define ST_LOOP_MS         10u     /* 主循环一拍 */
#define ST_SHOW_MS         200u    /* 刷屏间隔（OLED 慢，别每步都刷） */
#define ST_NOW_MS          1000u   /* 无线串口"当前角度"心跳间隔 */

/* PA7 循环切换的步长表（下标 0 是上电默认） */
static const uint16_t k_st_steps[] = { 1u, 5u, 10u, 45u };
#define ST_STEP_N   ((uint8_t)(sizeof(k_st_steps) / sizeof(k_st_steps[0])))

#if   ST_SERVO_ID == 0
  #define ST_PIN_NAME  "PC9"
  #define ST_CH_NAME   "C4"
  #define ST_INIT_DEG  ((uint16_t)SERVO_ROT_INIT_DEG)
#elif ST_SERVO_ID == 1
  #define ST_PIN_NAME  "PC8"
  #define ST_CH_NAME   "C3"
  #define ST_INIT_DEG  ((uint16_t)SERVO_GRIP_INIT_DEG)
#else
  #error "ST_SERVO_ID must be 0 (PC9/ROT) or 1 (PC8/GRIP)"
#endif

static void st_pc_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_3;   /* 本板可用的三个 PC 按键 */
#if GA_PC2_ACTIVE_LOW
    gpio.GPIO_Mode = GPIO_Mode_IPU;         /* 上拉，按下接地 */
#else
    gpio.GPIO_Mode = GPIO_Mode_IPD;
#endif
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);
}

/* 按键是否正按着（不做消抖，这里用"按住连发"的方式，抖动看不出来） */
static uint8_t st_pc_held(uint16_t pin)
{
#if GA_PC2_ACTIVE_LOW
    return (GPIO_ReadInputDataBit(GPIOC, pin) == Bit_RESET) ? 1u : 0u;
#else
    return (GPIO_ReadInputDataBit(GPIOC, pin) == Bit_SET)   ? 1u : 0u;
#endif
}

/* TIM2_CH1 -> PA0：备用输出脚，和 PC9 输出同一个脉宽（1 us 计数 / 20 ms 周期） */
static void st_tim2_pa0_init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef oc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_0;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    tb.TIM_Prescaler         = (uint16_t)(72u - 1u);
    tb.TIM_Period            = (uint16_t)(20000u - 1u);
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tb);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_Pulse       = 722u;
    TIM_OC1Init(TIM2, &oc);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

/* 把当前角度同时输出到 PC9/PC8（经 Servo.c）和 PA0（TIM2），返回实际脉宽 */
static uint16_t st_apply(uint16_t deg)
{
    uint16_t pulse;

    Servo_SetAngle((ServoId)ST_SERVO_ID, deg);
    pulse = Servo_GetPulse((ServoId)ST_SERVO_ID);
    TIM_SetCompare1(TIM2, (uint32_t)pulse);      /* PA0 走同一个脉宽 */
    return pulse;
}

/* 打一行无线串口日志。tag：SET = 刚改过，NOW = 每秒心跳 */
static void st_log(const char *tag, uint16_t deg, uint16_t pulse,
                   uint16_t ccr, uint16_t step)
{
    uint32_t duty = ((uint32_t)pulse * 1000u) / 20000u;   /* 千分比（×10 的百分数） */

    USART3_Printf("[SERVO] %s deg=%u pulse=%uus duty=%u.%u%% %s=%u step=%u\r\n",
                  tag,
                  (unsigned)deg, (unsigned)pulse,
                  (unsigned)(duty / 10u), (unsigned)(duty % 10u),
                  ST_CH_NAME, (unsigned)ccr, (unsigned)step);
}

static void servo_test_run(void)
{
    uint16_t deg;
    uint16_t pulse;
    uint16_t ccr3;
    uint16_t hold_gap = 0u;
    uint16_t show_gap = 0u;
    uint16_t now_gap  = 0u;
    uint16_t step;
    uint8_t  step_idx = 0u;
    uint8_t  dirty    = 1u;
    uint8_t  pa7_prev = 0u;
    uint8_t  pa5_prev = 0u;
    uint32_t inv;

    st_pc_init();
    grab_key_init();          /* PA7 / PA5 */
    st_tim2_pa0_init();

    inv = (uint32_t)(ST_SERVO_ID == 0 ? SERVO_INVERT_ROT : SERVO_INVERT_GRIP);
    step = k_st_steps[step_idx];

    /* ① 上电先摆到初始位置 */
    deg = ST_INIT_DEG;
    pulse = st_apply(deg);

    OLED_Clear();
    OLED_ShowString(1, 1, "SERVO " ST_PIN_NAME " ST:");
    OLED_ShowNum(1, 15, (uint32_t)step, 2);
    OLED_ShowString(2, 1, "DEG ");
    OLED_ShowNum(2, 5, (uint32_t)deg, 3);
    OLED_ShowString(2, 9, "P ");
    OLED_ShowNum(2, 11, (uint32_t)pulse, 4);
    OLED_ShowString(3, 1, ST_CH_NAME " ");
    OLED_ShowNum(3, 4, (uint32_t)TIM3->CCR4, 4);
    OLED_ShowString(3, 8, " C2 ");
    OLED_ShowNum(3, 12, (uint32_t)TIM2->CCR1, 4);
    OLED_ShowString(4, 1, "PC2+ PC3- PA5=R ");

    USART3_SendString("\r\n[SERVO] ===== 旋转舵机角度精调 =====\r\n");
    USART3_Printf("[SERVO] 输出脚 %s（TIM3）+ PA0（TIM2）同时输出同一脉宽\r\n",
                  ST_PIN_NAME);
    USART3_Printf("[SERVO] 方向取反=%u（0=正常 1=反）  初始 %u 度  脉宽 %u us\r\n",
                  (unsigned)inv, (unsigned)deg, (unsigned)pulse);
    USART3_SendString("[SERVO] PC2=+step  PC3=-step  PA5=回初始  PA7=换步长(1/5/10/45)\r\n");
    USART3_SendString("[SERVO] 每改一次立刻打 SET 行，另外每秒打一行 NOW\r\n");
    USART3_SendString("[SERVO] 机械臂指到想要的位置时，把那行的 deg 记下来\r\n");

    /* 定时器状态一次性打出来（排"定时器没跑"用） */
    USART3_Printf("[SERVO] TIM3 CEN=%u PSC=%u ARR=%u  MAPR=0x%08X (T3REMAP=%u)\r\n",
                  (unsigned)(TIM3->CR1 & 0x0001u),
                  (unsigned)(TIM3->PSC),
                  (unsigned)(TIM3->ARR),
                  (unsigned)AFIO->MAPR,
                  (unsigned)((AFIO->MAPR >> 10) & 0x0003u));
    USART3_Printf("[SERVO] TIM2 CEN=%u PSC=%u ARR=%u\r\n",
                  (unsigned)(TIM2->CR1 & 0x0001u),
                  (unsigned)(TIM2->PSC),
                  (unsigned)(TIM2->ARR));

    st_log("SET", deg, pulse, (uint16_t)TIM3->CCR4, step);

    /* 初始那一行已经打过了：清掉 dirty、把心跳计时重新拉满，避免上电重复打 */
    dirty   = 0u;
    now_gap = ST_NOW_MS;

    while(1)
    {
        uint8_t pa7 = grab_key_down(GPIOA, GPIO_Pin_7) ? 1u : 0u;
        uint8_t pa5 = grab_key_down(GPIOA, GPIO_Pin_5) ? 1u : 0u;

        if(hold_gap > 0u) hold_gap = (hold_gap > ST_LOOP_MS) ? (uint16_t)(hold_gap - ST_LOOP_MS) : 0u;
        if(show_gap > 0u) show_gap = (show_gap > ST_LOOP_MS) ? (uint16_t)(show_gap - ST_LOOP_MS) : 0u;
        if(now_gap  > 0u) now_gap  = (now_gap  > ST_LOOP_MS) ? (uint16_t)(now_gap  - ST_LOOP_MS) : 0u;

        /* PA7：切换步长 1° → 5° → 10° → 45° → 1° */
        if(pa7 && !pa7_prev){
            step_idx = (uint8_t)((step_idx + 1u) % ST_STEP_N);
            step = k_st_steps[step_idx];
            USART3_Printf("[SERVO] 步长 -> %u 度\r\n", (unsigned)step);
        }

        /* PA5：回初始位置 */
        if(pa5 && !pa5_prev){
            deg = ST_INIT_DEG;
            dirty = 1u;
            USART3_Printf("[SERVO] 回初始 %u 度\r\n", (unsigned)deg);
        }

        pa7_prev = pa7;
        pa5_prev = pa5;

        /* PC2 / PC3：加减角度（按住连发） */
        if(hold_gap == 0u && st_pc_held(GPIO_Pin_2)){
            deg = (deg <= (uint16_t)(SERVO_ANGLE_MAX - step))
                  ? (uint16_t)(deg + step) : (uint16_t)SERVO_ANGLE_MAX;
            hold_gap = ST_HOLD_MS;
            dirty = 1u;
        }else if(hold_gap == 0u && st_pc_held(GPIO_Pin_3)){
            deg = (deg >= step) ? (uint16_t)(deg - step) : 0u;
            hold_gap = ST_HOLD_MS;
            dirty = 1u;
        }

        /* ② 角度一变：立刻写 CCR + 立刻打一行无线串口 */
        if(dirty){
            dirty = 0u;

            pulse = st_apply(deg);

#if ST_SERVO_ID == 0
            ccr3 = (uint16_t)TIM3->CCR4;
#else
            ccr3 = (uint16_t)TIM3->CCR3;
#endif
            st_log("SET", deg, pulse, ccr3, step);

            now_gap  = ST_NOW_MS;      /* 刚打过，心跳重新计时 */
            show_gap = 0u;             /* 立刻刷屏 */
        }

        /* ③ 心跳：每秒打一次"当前角度"，方便随时读数 */
        if(now_gap == 0u){
            now_gap = ST_NOW_MS;
#if ST_SERVO_ID == 0
            ccr3 = (uint16_t)TIM3->CCR4;
#else
            ccr3 = (uint16_t)TIM3->CCR3;
#endif
            st_log("NOW", deg, pulse, ccr3, step);
        }

        /* ④ 刷屏 */
        if(show_gap == 0u){
            show_gap = ST_SHOW_MS;

            OLED_ClearLine(1);
            OLED_ShowString(1, 1, "SERVO " ST_PIN_NAME " ST:");
            OLED_ShowNum(1, 15, (uint32_t)step, 2);

            OLED_ShowString(2, 1, "DEG ");
            OLED_ShowNum(2, 5, (uint32_t)deg, 3);
            OLED_ShowString(2, 9, "P ");
            OLED_ShowNum(2, 11, (uint32_t)pulse, 4);

            OLED_ShowString(3, 1, ST_CH_NAME " ");
            OLED_ShowNum(3, 4, (uint32_t)ccr3, 4);
            OLED_ShowString(3, 8, " C2 ");
            OLED_ShowNum(3, 12, (uint32_t)TIM2->CCR1, 4);
        }

        Delay_ms(ST_LOOP_MS);
    }
}

/* ============================================================================
 * 「从车上把物料夹下来，放到地上」—— 半自动流程（DEBUG_MODE_CAR_PUT = 12）
 * ----------------------------------------------------------------------------
 * 方向正好和原来那个 k_grab_steps[] 相反：
 *     老的：台面/转盘夹起来 → 放到车上
 *     这个：**车上夹起来 → 放到地上**
 *
 * ★ 现在的主操作只有两个按键（PC3 两次）：
 *     按 PC3 第 1 次 → ①② 自动跑：抬到 7600 → 发 circle → 底盘自动修正
 *     按 PC3 第 2 次 → ③~⑪ 自动跑：转 163° → 抬臂 → 降 1000 → 夹 →
 *                     抬回 7600 → 转回 30° → 回 90° → 降 0 → 松开（放地上）
 *     PA5 = 中止；PC2 / 上位机 N = 一次只做一步（调试用）
 *
 * 正向 11 个子动作：
 *     1) 升降步进 0 → 7600      先把机械臂抬起来（相机才看得到地面）
 *     2) ★ K230 圆心精准定位     对准**地面上的圆环**，动底盘把车摆正
 *                                （此时旋转舵机还在 30°、升降舵机还在 90°，都没动）
 *                                这一步**停下来等你按 PC3**，其余全自动
 *     3) 旋转舵机 30° → 163°    转到车上料仓方向
 *     4) 升降舵机 90° → 130°    抬臂姿态
 *     5) 升降步进 7600 → 1000   降到车上物料的高度
 *     6) 夹爪舵机 85° → 110°    夹住物料
 *     6.5) 升降步进 1000 → 7600 夹住后先抬起来（CP_UP_BEFORE_BACK=0 时没这步）
 *     7) 旋转舵机 163° → 30°    转回放料侧
 *     8) 升降舵机 130° → 90°    回初始
 *     9) 升降步进 7600 → 0      降到地上（带 CP_DESCEND_TRIM 补偿，会少降一点）
 *    10) 夹爪舵机 110° → 85°    松开 = 放下（这一步不压栈）
 *
 * ★ 第 2 步"圆心定位"为什么在最前面：
 *    一开始机械臂只抬了步进（7600），**旋转舵机还没转**、升降舵机还在 90°，
 *    相机正对着**地面上的圆环**。这时候先把车摆正（底盘平移/微调），
 *    后面再转到车上、降下去夹，落点才准。
 *    它是**独立动作**，可以单独点（上位机 V 命令 / 点流程第 2 行），
 *    也可以按顺序走。它动的是底盘，所以**不压撤销栈**（底盘位置没法恢复）。
 *
 * ★ 为什么"先抬步进再旋转"：
 *   旋转（偏航）一定要在高位做，低位转过去容易撞车身/料仓。
 *   步进是先粗定位（垂直），舵机负责细节姿态（升降舵机=俯仰、旋转舵机=偏航）。
 *
 * ★ 六个标定值（改下面 CP_* 宏即可，都是现场实测的绝对量）：
 *     步进   HIGH=7600   GRAB=1000   HOME=0
 *     升降舵机  UP=130°   HOME=90°
 *     夹爪     HOLD=110°  HOME=85°
 *     旋转舵机  CAR=163°  HOME=30°
 *
 * ★ 旋转舵机的两个角度都是**固定常量**，流程里不做动态记忆。
 *   先做视觉对圆心把"车"摆好，舵机永远转同一个角度，落点才稳定。
 *
 * 然后进入"恢复"阶段，**严格按栈的后进先出**一个一个退（每按一次退 1 个）：
 *   第 10 步不压栈、第 4 步（视觉）也不压栈，所以栈里是 8 个动作。
 *   退的顺序：
 *     退 8) 步进 0 → 1000     抬回抓料高度
 *     退 7) 升降舵机 90 → 130  抬起
 *     退 6) 旋转 30 → 163      转回车上
 *     退 5) 夹爪 110 → 85      张开
 *     退 4) 步进 1000 → 7600   抬到高位
 *     退 3) 升降舵机 130 → 90  回初始
 *     退 2) 旋转 163 → 30      回放料侧
 *     退 1) 步进 7600 → 0      降回最低位
 *   最后四个部件都回到上电姿态：旋转 30° / 夹爪 85° / 升降舵机 90° / 步进 0。
 *   （底盘停在视觉对准后的位置，不会自动退回发车点）
 *
 * 撤销栈的做法：每执行一个正向动作，就把"回到执行前状态"的反向动作压栈。
 *   舵机 → 记录执行前的角度
 *   步进 → 记录执行前的**绝对位置**
 *   视觉 → 不压栈（底盘位置没法"恢复"）
 *
 * 按键（本板只有 PC0/PC2/PC3 好用）：
 *     PC3   单步：按一次执行 1 个子动作（★ 主键）
 *     PC2   单步：和 PC3 等价（备用）
 *     PB1   连跑：正向全做完 + 自动弹栈恢复
 *     PC0   复位到第 1 步（不动硬件）
 *     PA5   中止：停在原地
 *
 * OLED：
 *     1 行  CARPUT FWD 1/10
 *     2 行  ROT163              （刚做的子动作）
 *     3 行  G085 R030 U130      （夹爪 / 旋转 / 升降舵机）
 *     4 行  L1000 PC2=STEP      （步进绝对位置 + 键提示）
 * ========================================================================== */
/* DEBUG_MODE_CAR_PUT 的定义在文件顶部（和其它模式号放一起） */

/* ================= 现场标定（2026-11 实测，用户口述流程） ================= */
/* 升降步进电机（地址 6）：**绝对位置**
 *   现象判据：同一目标值连发多次只有第一次会动 → 绝对定位，确认无疑 */
#define CP_MOT_HIGH        7600u   /* 抬起让位高度 */
#define CP_MOT_GRAB        1000u   /* 降到车上夹料的高度 */
#define CP_MOT_HOME        0u      /* 回零（最低位） */

/* 升降舵机（PC7） */
#define CP_LIFT_UP_DEG     130u    /* 抬臂姿态 */
#define CP_LIFT_HOME       90u     /* 回初始 */

/* 夹爪舵机（PC8） */
#define CP_GRIP_HOLD       110u    /* 夹紧 */
#define CP_GRIP_HOME       85u     /* 松开 / 回初始 */

/* 旋转舵机（PC9） */
#define CP_ROT_HOME        30u     /* 回初始（对台面/放料侧） */
#define CP_ROT_CAR         SERVO_ROT_PLACE_DEG   /* 163：转到车上料仓方向（已实测） */
/* ======================================================================== */

#define CP_SERVO_SETTLE_MS GRAB_SETTLE_MS           /* 舵机每步等多久（800） */
#define CP_LIFT_MS_PER_1000P 600u                   /* 升降每 1000 脉冲约走多久（ms），与 LIFT_MOTOR_SPEED=40 匹配 */
#define CP_LIFT_EXTRA_MS   400u                     /* 每步额外留的余量 */
#define CP_UNDO_MAX        16u                      /* 正向步骤数 + 余量（加了 UP7600B 之后是 11 步） */

/* ★ 夹住之后要不要先把步进抬回让位高度，再转回 30°（往车斗那一侧转的时候，
 *   停在 1000 的高度容易蹭到车斗）。现场口述流程是"拿到料 → 先升起来 → 再降下去放地上"，
 *   所以默认 1：
 *     1 = 夹住后先 UP7600B 抬回 7600，再 ROT30 / LFT90 / DN0 / 松开（11 步）
 *     0 = 不抬，夹住后直接转回（原来的 10 步）
 *   发现机械上不需要多这一次抬升（时间多 6~7 秒），把它改成 0 就行。 */
#define CP_UP_BEFORE_BACK  1u

/* ★ 每按一次 PC3 只做**一个动作**，而且**发完命令立刻返回、不等待到位**。
 *   现场原话："我自己眼睛会判断有没有到位" —— 代码不再插任何延时，
 *   上一动作还在走就可以直接按下一次。
 *     1 = 不等（★当前）
 *     0 = 老行为：等估算时间走完再返回（模式 12 里基本用不到了） */
#define CP_STEP_NO_WAIT    1u

/* ★ 视觉那一步结束（按 PC3 确认对准）时，要不要**顺手把下一个动作也做掉**。
 *     1 = 做（★当前）：按一下就转到 163°，不会出现"按了没反应、还得再按一次"
 *     0 = 不做：那一按只负责停下底盘，下一个动作等下一次按键 */
#define CP_VISION_EXIT_NEXT 1u

/* ★ 升降下降要不要"两段式软着陆"（先 SPEED_DOWN 到 目标+300，再 8 RPM 爬行收尾）。
 *   2026-11-21 现场：下降会触发驱动器堵转保护。爬行段 8 RPM 太慢是重点嫌疑
 *   （头文件里本来就写着"太小可能带不动"）。排查期先关掉，一段降到底：
 *     0 = 一段（排查用，现在的值）
 *     1 = 两段软着陆（原来那套）
 *   关掉之后如果**不再堵转**，就说明是爬行段的问题，把 LIFT_MOTOR_SPEED_CREEP 调到 15 再打开。 */
#define CP_LIFT_SOFT_LANDING  0u

/* ★ 下降"少走一点"补偿（脉冲）：正数 = 每次下降都停在**更高**的位置。
 *
 * ★ 2026-11-21 归零了 —— 因为"降过头"的**根因已经修掉**：
 *     根因不是我原来以为的"重力丢步"，而是**帧的语义错了**：
 *     驱动器把脉冲字段当"要走多少"，我们一直发"走到哪个坐标"，
 *     于是上升看起来正常、下降变成"按目标坐标当行程走一大段"。
 *     现在 lift_motor_move() 统一换算成差值 + 相对帧（LIFT_USE_REL_FRAME=1），
 *     走多少就是多少，不再需要预先把目标抬高。
 *
 *   这个数现在是纯微调：
 *       发现下降比预期低（压太死）→ D200 / D300
 *       发现降不到底（悬空）      → D0（现在就是 0）
 *   D<n> 现场改，立即生效，不用重烧。 */
#define CP_DESCEND_TRIM    0u

/* ★ 机械底防呆：驱动器报回的位置 ACT <= 这个值时，**拒绝再执行下降命令**。
 *   注意：这条要靠 0x36 读位置才生效，而现场 `ACT=-9999`（读不到），
 *   所以它目前**兜不住**；真正防呆靠的是"走多少"的相对帧 + 你眼睛盯着。 */
#define CP_FLOOR_GUARD     60u

#define CP_PH_IDLE  0u
#define CP_PH_FWD   1u
#define CP_PH_UNDO  2u
#define CP_PH_DONE  3u

#define CP_KIND_SERVO  0u
#define CP_KIND_LIFT   1u
#define CP_KIND_VISION 2u     /* K230 圆心精准定位（动底盘，不动机械臂） */

/* ---- 视觉对准参数 ---- */
/* ★ 现场流程："我觉得对准了，就再按一次 PC3" —— 那收工/超时就**不能**由代码自己决定。
 *   CP_VISION_MAX_MS = 0 → 不超时，一直等你按 PC3（PA5 随时中止）
 *   CP_VISION_SETTLE = 0 → 不自动收工，即使连续多帧在死区里也继续等你 */
#define CP_VISION_MAX_MS       0u       /* 0 = 不超时；>0 = 等这么多 ms 后强行退出 */
/* ★ 收不到数据时要不要提前退出：
 *     0 = 不提前退（默认；K230 没接也能一直等，靠 PC3 确认 / PA5 中止）
 *     1 = CP_VISION_NO_DATA_MS 内一帧都没有就立刻退出（赶时间/正式比赛时用） */
#define CP_VISION_NO_DATA_BAIL  0u
#define CP_VISION_NO_DATA_MS    3000u
#define CP_VISION_STABLE       3u       /* （保留，内部已改用 SETTLE） */
#define CP_VISION_SETTLE       0u       /* 0 = 不自动收工（只认 PC3）；>0 = 连续这么多帧稳在死区就收工 */
#define CP_VISION_RESEND_MS    2000u    /* K230 静默多久重发一次 circle */
#define CP_VISION_RETRY_MAX    5u
/* 对准超时/失败时：1 = 打个大红警告继续往下走；0 = 让这一步失败停住。
 * 现场调试阶段建议 1（K230 没接也能走完流程看清机械动作），
 * 正式比赛改成 0（没对准就别去夹，免得夹空）。 */
#define CP_VISION_SOFT_FAIL    1u

typedef struct {
    uint8_t  kind;      /* CP_KIND_SERVO / CP_KIND_LIFT */
    uint8_t  id;        /* kind=SERVO：哪一路舵机 */
    uint16_t deg;       /* kind=SERVO：目标角度 */
    uint32_t pulses;    /* kind=LIFT：目标绝对位置 */
    uint8_t  undo;      /* 1 = 执行时把反向动作压栈 */
    char     label[8];  /* 屏上显示，最多 7 字符 */
} CarPutStep;

/* 正向 11 步：抬 → 识别圆 → 转到车 → 抬臂 → 降下去夹 → **再抬起来** → 转回 → 降下去放 → 松开
 * （CP_UP_BEFORE_BACK = 0 时没有第 6.5 步，就是 10 步）
 * ★ 升降帧里没有方向字段：方向在 cp_apply() 里按"当前 → 目标"自动算，
 *   这样以后改了高度不会因为方向写反而往反方向撞。 */
static const CarPutStep k_cp_steps[] = {
    /* ① 升降步进 → 7600（先把机械臂抬起来，相机才看得到地面） */
    { CP_KIND_LIFT,  0u,          0u,                       CP_MOT_HIGH,    1u, "UP7600"  },
    /* ② ★ K230 圆心精准定位：对准**地面上的圆环**把车摆正
     *    此时旋转舵机还在初始 30°（没转到车上），升降舵机也在 90°，
     *    相机正对地面圆环。动底盘，不压栈。 */
    { CP_KIND_VISION, 0u,         0u,                       0u,             0u, "CIRCLE"  },
    /* ③ 旋转舵机 → 163°（转到车上料仓方向） */
    { CP_KIND_SERVO, SERVO_ROT,   (uint16_t)CP_ROT_CAR,     0u,             1u, "ROT163"  },
    /* ④ 升降舵机 → 130°（抬臂姿态） */
    { CP_KIND_SERVO, SERVO_LIFT,  (uint16_t)CP_LIFT_UP_DEG, 0u,             1u, "LFT130"  },
    /* ⑤ 升降步进 → 1000（降到车上料的位置） */
    { CP_KIND_LIFT,  0u,          0u,                       CP_MOT_GRAB,    1u, "DN1000"  },
    /* ⑥ 夹爪舵机 → 110°（夹住物料） */
    { CP_KIND_SERVO, SERVO_GRIP,  (uint16_t)CP_GRIP_HOLD,   0u,             1u, "GRP110"  },
#if CP_UP_BEFORE_BACK
    /* ⑥.5 ★ 夹住之后先把步进抬回让位高度，再往回转 —— 避免在 1000 的高度
     *     转回 30° 时蹭到车斗。CP_UP_BEFORE_BACK = 0 时这一步不存在。 */
    { CP_KIND_LIFT,  0u,          0u,                       CP_MOT_HIGH,    1u, "UP7600B" },
#endif
    /* ⑦ 旋转舵机 → 回初始 30° */
    { CP_KIND_SERVO, SERVO_ROT,   (uint16_t)CP_ROT_HOME,    0u,             1u, "ROT30"   },
    /* ⑧ 升降舵机 → 回初始 90° */
    { CP_KIND_SERVO, SERVO_LIFT,  (uint16_t)CP_LIFT_HOME,   0u,             1u, "LFT90"   },
    /* ⑨ 升降步进 → 回零 0 */
    { CP_KIND_LIFT,  0u,          0u,                       CP_MOT_HOME,    1u, "DN0"     },
    /* ⑩ 夹爪舵机 → 回初始 85°（松开＝放下，不压栈） */
    { CP_KIND_SERVO, SERVO_GRIP,  (uint16_t)CP_GRIP_HOME,   0u,             0u, "GRP85"   }
};

#define CP_STEP_COUNT   ((uint8_t)(sizeof(k_cp_steps) / sizeof(k_cp_steps[0])))

static CarPutStep g_cp_undo[CP_UNDO_MAX];
static uint8_t    g_cp_undo_n     = 0u;
static uint8_t    g_cp_phase      = CP_PH_IDLE;
static uint8_t    g_cp_fwd        = 0u;
static uint8_t    g_cp_undo_total = 0u;
static uint8_t    g_cp_undo_done  = 0u;

/* ★ 下降补偿的当前值（脉冲）。上电 = CP_DESCEND_TRIM，串口 D<n> 可现场改。 */
static uint32_t   g_cp_descend_trim = CP_DESCEND_TRIM;

/* 把"表里写的目标绝对位置"换算成"这次真正要发的位置"。
 *   只在**下降**（目标比当前低）时把目标抬高 trim 个脉冲 —— 就是"降少一点"；
 *   上升完全不受影响，所以 7600 让位高度、1000 抓料高度都不变。
 *   注意：必须用修过的值去和 g_lift_target 比"是否已经在位"。 */
static uint32_t cp_lift_aim(uint32_t target)
{
    if(target >= g_lift_target) return target;              /* 上升 / 原地：不动 */

    if((target + g_cp_descend_trim) > (uint32_t)LIFT_MOTOR_MAX)
        return (uint32_t)LIFT_MOTOR_MAX;                    /* 软上限保护 */

    return target + g_cp_descend_trim;
}

/* ============================================================================
 * ★ 带堵转检测的"等电机走完"
 * ----------------------------------------------------------------------------
 * 现场问题：下降时驱动器一旦堵转，电机还在死顶，代码却只是傻等估算时间
 * （最长 9 秒），等完还照样往下走下一步 —— 机械就是在这一段被顶坏的。
 *
 * 做法：把一次等待切成 500 ms 一片，每片读一次驱动器实时位置（0x36）。
 *   位置连续 3 片（1.5 s）几乎不动 → 判定堵转 → 立刻急停 → 返回 0。
 *   最后 1.5 s 不判（减速收尾时本来就几乎不动，免得误报）。
 *   读不到位置（驱动器不回 0x36）时**不判**，退化成普通等待，不会误停。
 *
 * 返回 1 = 正常等完；0 = PA5 中止 或 堵转急停。
 * ========================================================================== */
static uint8_t cp_lift_wait_guarded(uint16_t ms)
{
    uint16_t elapsed = 0u;
    int32_t  last  = 0;
    int32_t  now   = 0;
    uint8_t  have  = 0u;
    uint8_t  still = 0u;

    while(elapsed < ms)
    {
        uint16_t slice = (uint16_t)(((uint16_t)(ms - elapsed) > 500u) ? 500u
                                                                    : (uint16_t)(ms - elapsed));
        if(!grab_wait(slice)) return 0u;                  /* PA5 中止 */
        elapsed = (uint16_t)(elapsed + slice);

        if((uint16_t)(elapsed + 1500u) >= ms) break;      /* 收尾段不判 */

        if(lift_motor_read_pos(&now)){
            if(have){
                if(((now > last) ? (now - last) : (last - now)) < 30) still++;
                else                                                  still = 0u;
            }
            last = now;
            have = 1u;
        }

        if(still >= 3u){
            lift_motor_stop();
            USART3_SendString("[CARPUT] ⚠ 位置 1.5 s 没变化 —— 判定【堵转】，已急停\r\n");
            USART3_SendString("[CARPUT]   查：是不是降到机械底了 / 零点设错 / 电流档太低\r\n");
            return 0u;
        }
    }

    if(elapsed < ms){
        if(!grab_wait((uint16_t)(ms - elapsed))) return 0u;
    }
    return 1u;
}

/* 找到"视觉对准"那一步的下标；没有就返回 CP_STEP_COUNT（当不存在处理） */
static uint8_t cp_vision_index(void)
{
    uint8_t i;
    for(i = 0u; i < CP_STEP_COUNT; i++){
        if(k_cp_steps[i].kind == CP_KIND_VISION) return i;
    }
    return CP_STEP_COUNT;
}

/* 复位内部状态（不动硬件） */
static void cp_reset_state(void)
{
    g_cp_undo_n     = 0u;
    g_cp_phase      = CP_PH_IDLE;
    g_cp_fwd        = 0u;
    g_cp_undo_total = 0u;
    g_cp_undo_done  = 0u;
}

/* ============================================================================
 * K230 圆心定位（动底盘，不动机械臂）—— ★ 连续修正模式 ★
 * ----------------------------------------------------------------------------
 * 流程：
 *   ① 主控给 K230 发读取圆心的命令 "circle"
 *   ② K230 持续回 C<x>,<y>,<r>（约 3 Hz）
 *   ③ 主控**一直读、一直修**：每来一帧就比一次偏差，超出死区就发一步底盘修正
 *   ④ 连续 CP_VISION_SETTLE 帧都稳在死区里才收工 —— 不是"几帧就走"，
 *      识别抖一帧就重新计数，抖完继续修
 *
 * 返回 1 = 对准成功（或软失败放行），0 = 中止（PA5）或硬失败。
 *
 * ★ 为什么它是一步独立的动作：
 *   底盘修正和机械臂动作是两回事，混在一起出错很难查。
 *   做成独立步骤之后，可以单独点它（上位机 V 命令或点流程第 2 行），
 *   也可以按顺序走。
 * ★ 它**不压撤销栈**：底盘位置没法"恢复"，所以不进栈。
 * ========================================================================== */
static uint8_t cp_vision_align(void)
{
    uint32_t cseq;
    uint32_t t_ms   = 0u;
    uint32_t silent = 0u;
    uint16_t no_data = 0u;     /* 从发命令到现在，一帧都没收到多久了 */
    uint16_t wait_ms = 0u;
    uint8_t  steady = 0u;      /* 连续稳住多少帧 */
    uint8_t  retry  = 0u;
    uint16_t frames = 0u;
    uint16_t moves  = 0u;      /* 实际发了几步底盘修正 */
    uint32_t max_ms = CP_VISION_MAX_MS;   /* 0 = 不超时（用变量，别让编译器对常量比较报警） */
    int16_t  prev_cx = 0;      /* 上一帧圆心，用来打印 d=(Δx,Δy)：一眼看出车往哪边动了 */
    int16_t  prev_cy = 0;
    uint8_t  has_prev = 0u;
    char     act[17];

    USART3_SendString("[CARPUT] --- K230 圆心定位（连续修正）---\r\n");

    /* 先清掉旧序号，避免用过期帧 */
    cseq = K230_CircleSeq;
    K230_SendLine(K230_CMD_CIRCLE);
    USART3_Printf("[CARPUT] TX circle  目标点=(%d,%d)  死区=±%d px\r\n",
                  VISION_TARGET_X, VISION_TARGET_Y, VISION_DEADBAND_PX);
    USART3_SendString("[CARPUT] 底盘会自动一步步修正；**对准了就再按一次 PC3**\r\n");
#if CP_VISION_SETTLE > 0
    USART3_Printf("[CARPUT] 另外：连续 %u 帧进死区也会自动收工\r\n",
                  (unsigned)CP_VISION_SETTLE);
#else
    USART3_SendString("[CARPUT] 不做自动收工、不超时 —— 只有 PC3 能结束这一步\r\n");
#endif
#if CP_VISION_MAX_MS > 0
    USART3_Printf("[CARPUT] 最长等 %u ms\r\n", (unsigned)CP_VISION_MAX_MS);
#endif
    USART3_SendString("[CARPUT] PA5 = 中止整段\r\n");

    while((max_ms == 0u) || (t_ms < max_ms))
    {
        /* ---------- PA5：中止整段流程 ---------- */
        if(grab_abort_hit()){
            vision_stop_all();
            K230_SendLine(K230_CMD_STOP);
            USART3_SendString("[CARPUT] 视觉对准被 PA5 中止\r\n");
            return 0u;
        }

        /* ---------- PC3：对准确认（停底盘，继续往下走） ---------- */
        if(st_pc_held(GPIO_Pin_3)){
            Delay_ms(20);
            if(st_pc_held(GPIO_Pin_3)){
                vision_stop_all();
                K230_SendLine(K230_CMD_STOP);
                USART3_Printf("[CARPUT] ✅ PC3 确认对准，底盘停住（%u 帧 / %u 步修正）\r\n",
                              (unsigned)frames, (unsigned)moves);
                while(st_pc_held(GPIO_Pin_3)) Delay_ms(10);
                return 1u;
            }
        }

        /* ---------- 上一步还在走：等它走完再算下一帧 ---------- */
        if(wait_ms > 0u){
            uint16_t slice = (wait_ms > 20u) ? 20u : wait_ms;
            if(!grab_wait(slice)){
                vision_stop_all();
                K230_SendLine(K230_CMD_STOP);
                return 0u;
            }
            wait_ms = (wait_ms > 20u) ? (uint16_t)(wait_ms - 20u) : 0u;
            t_ms += 20u;
            continue;
        }

        /* ---------- K230 一直发，我们一直读 ---------- */
        if(K230_CircleSeq != cseq)
        {
            uint16_t need;

            cseq    = K230_CircleSeq;
            silent  = 0u;
            retry   = 0u;
            no_data = 0u;
            frames++;

            need = vision_circle_issue((int32_t)K230_Cx, (int32_t)K230_Cy, act);

            if(need == 0u){
                if(steady < 255u) steady++;
#if CP_VISION_SETTLE > 0
                USART3_Printf("[CARPUT]   帧%u C(%d,%d) 稳住 %u/%u\r\n",
                              (unsigned)frames, (int)K230_Cx, (int)K230_Cy,
                              (unsigned)steady, (unsigned)CP_VISION_SETTLE);
#else
                /* 不自动收工：只报"在当前死区里"，收工由 PC3 决定 */
                USART3_Printf("[CARPUT]   帧%u C(%d,%d) d=(%d,%d) 已在死区（连续 %u 帧）\r\n",
                              (unsigned)frames, (int)K230_Cx, (int)K230_Cy,
                              (int)(has_prev ? ((int16_t)K230_Cx - prev_cx) : 0),
                              (int)(has_prev ? ((int16_t)K230_Cy - prev_cy) : 0),
                              (unsigned)steady);
#endif

#if CP_VISION_SETTLE > 0
                if(steady >= CP_VISION_SETTLE){
                    vision_stop_all();
                    K230_SendLine(K230_CMD_STOP);
                    USART3_Printf("[CARPUT] ✅ 对准完成（自动收工）C(%d,%d)  %u 帧 / %u 步修正\r\n",
                                  (int)K230_Cx, (int)K230_Cy,
                                  (unsigned)frames, (unsigned)moves);
                    return 1u;
                }
#endif
            }else{
                steady = 0u;
                moves++;
                wait_ms = need;
                if(has_prev){
                    USART3_Printf("[CARPUT]   帧%u C(%d,%d) d=(%d,%d) -> %s (%u ms)  第%u步\r\n",
                                  (unsigned)frames, (int)K230_Cx, (int)K230_Cy,
                                  (int)((int16_t)K230_Cx - prev_cx),
                                  (int)((int16_t)K230_Cy - prev_cy),
                                  act, (unsigned)need, (unsigned)moves);
                }else{
                    USART3_Printf("[CARPUT]   帧%u C(%d,%d) -> %s (%u ms)  第%u步\r\n",
                                  (unsigned)frames, (int)K230_Cx, (int)K230_Cy,
                                  act, (unsigned)need, (unsigned)moves);
                }
            }

            prev_cx  = (int16_t)K230_Cx;
            prev_cy  = (int16_t)K230_Cy;
            has_prev = 1u;
        }
        else
        {
            /* ---------- ★ 一帧都没收到：3 秒就退，别干等 ---------- */
            no_data += 10u;

            if((no_data % 1000u) < 10u){
                USART3_Printf("[CARPUT]   等 K230 回帧… 已等 %u ms（再按 PC3 = 对准确认 / PA5 中止）\r\n",
                              (unsigned)no_data);
            }

#if CP_VISION_NO_DATA_BAIL
            if(no_data >= CP_VISION_NO_DATA_MS){
                vision_stop_all();
                K230_SendLine(K230_CMD_STOP);
                USART3_Printf("[CARPUT] ❌ %u ms 内 K230 一帧都没回 —— 它没进识别圆的状态\r\n",
                              (unsigned)CP_VISION_NO_DATA_MS);
                USART3_SendString("[CARPUT]   依次查：\r\n");
                USART3_SendString("[CARPUT]    1) K230 上有没有在跑 vision/圆线串口.py\r\n");
                USART3_SendString("[CARPUT]    2) K230 的 T->PC11、R<-PC10、GND 共地（V 不接）\r\n");
                USART3_SendString("[CARPUT]    3) K230 是不是还在开机/被电源拉重启\r\n");
                USART3_SendString("[CARPUT]    4) 这段线是不是被陀螺仪占了\r\n");
#if CP_VISION_SOFT_FAIL
                USART3_SendString("[CARPUT]   ★ 软失败：直接跳过这一步，继续下一步\r\n");
                return 1u;
#else
                USART3_SendString("[CARPUT]   ★ 硬失败：这一步停下（修好再按 PC3）\r\n");
                return 0u;
#endif
            }
#endif
        }

        /* ---------- K230 静默重发 ----------
         * ★ 前 CP_VISION_RETRY_MAX 次用短间隔（2 s），之后改成 10 s 一次**一直发**：
         *   现场经常是先按 PC3、K230 那边脚本还没跑起来（或者被电源拉重启了），
         *   如果只重发 5 次就彻底放弃，K230 后来起来了也永远收不到 circle。 */
        silent += 10u;
        {
            uint32_t gap = (retry < CP_VISION_RETRY_MAX) ? (uint32_t)CP_VISION_RESEND_MS
                                                         : 10000u;
            if(silent >= gap){
                silent = 0u;
                if(retry < 255u) retry++;
                K230_SendLine(K230_CMD_CIRCLE);
                USART3_Printf("[CARPUT]   K230 静默 %u ms，重发 circle（第 %u 次）\r\n",
                              (unsigned)gap, (unsigned)retry);
            }
        }

        Delay_ms(10);
        t_ms += 10u;
    }

    K230_SendLine(K230_CMD_STOP);
    vision_stop_all();

    USART3_Printf("[CARPUT] ❌ 视觉对准超时 %u ms（%u 帧 / %u 步修正，最后稳住 %u 帧）\r\n",
                  (unsigned)max_ms, (unsigned)frames,
                  (unsigned)moves, (unsigned)steady);
    USART3_SendString("[CARPUT]   查：K230 是否在跑 圆线串口.py、K230T->PC11 / K230R<-PC10 / GND、有没有掉电重启\r\n");

#if CP_VISION_SOFT_FAIL
    USART3_SendString("[CARPUT]   ★ 软失败：继续往下走（会偏），要严格就设 CP_VISION_SOFT_FAIL=0\r\n");
    return 1u;
#else
    USART3_SendString("[CARPUT]   ★ 硬失败：这一步停下，不会去夹（修好再按单步重试）\r\n");
    return 0u;
#endif
}

/* 执行一个动作并等到位；PA5 中止返回 0（停在原地） */
static uint8_t cp_apply(const CarPutStep *s)
{
    uint32_t aim;
#if !CP_STEP_NO_WAIT
    uint16_t ms;
#endif

    if(s->kind == CP_KIND_SERVO){
        Servo_SetAngle((ServoId)s->id, s->deg);
#if CP_STEP_NO_WAIT
        return 1u;                       /* ★ 不等舵机到位，人眼看着按下一键 */
#else
        return grab_wait(CP_SERVO_SETTLE_MS);
#endif
    }

    if(s->kind == CP_KIND_VISION){
        return cp_vision_align();          /* 动底盘，不动机械臂 */
    }

    /* 升降：移动前先用驱动器实际位置校准跟踪值 */
    lift_motor_sync();

    /* ★ 下降补偿：表里写的是"理论目标"，真正发出去的是 aim（下降时抬高 trim） */
    aim = cp_lift_aim(s->pulses);

    if(aim == g_lift_target){
        return 1u;                       /* 已经在这个高度，什么都不用做 */
    }

    /* ★ 每次升降都打一行，并带上驱动器**自己报的位置**（ACT）。
     *   "降过头"只有两种可能，这一行就能分辨：
     *     ACT 也跑到了目标附近 → 驱动器确实走过去了（丢步/没按绝对位置停）
     *     ACT 还停在目标上     → 驱动器以为自己在目标，实际机械更低（丢步更多）
     *   同时用它做防呆：ACT 已经在机械底附近就拒绝继续下降，避免越降越深。 */
    {
        int32_t act_now = 0;
        uint8_t have_act = lift_motor_read_pos(&act_now);
        uint8_t going_down = (aim < g_lift_target) ? 1u : 0u;

        USART3_Printf("[CARPUT]   升降 %u -> %u  %s %u RPM  ACT=%d\r\n",
                      (unsigned)g_lift_target, (unsigned)aim,
                      going_down ? "下降" : "上升",
                      (unsigned)(going_down ? LIFT_MOTOR_SPEED_DOWN : LIFT_MOTOR_SPEED),
                      have_act ? (int)act_now : -9999);

        if(going_down && have_act && (act_now <= (int32_t)CP_FLOOR_GUARD)){
            USART3_Printf("[CARPUT] ⚠ ACT=%d 已在机械底附近（<=%u），拒绝继续下降\r\n",
                          (int)act_now, (unsigned)CP_FLOOR_GUARD);
            USART3_SendString("[CARPUT]   要重新对零：模式16 里用 ↓ 降到低位再按 W 设零点\r\n");
            return 1u;                   /* 不动，也不往下走 */
        }
    }

#if CP_STEP_NO_WAIT
    /* ★ 只把命令发出去就返回：上一条还在走也可以直接按下一条，
     *   到没到位由人眼判断（现场要求）。速度/加速度仍按下面这套默认值。 */
    lift_motor_move(aim, (aim > g_lift_target) ? 0x01u : 0x00u);
    return 1u;
#else

#if CP_LIFT_SOFT_LANDING
    {
        uint32_t mid;

        /* 长距离下降走两段：先下降速度到 目标+SOFT_ZONE，再爬行速度收最后一段。 */
        if(lift_motor_soft_split(g_lift_target, aim, &mid))
        {
            ms = lift_motor_ms_for(g_lift_target, mid, (uint8_t)LIFT_MOTOR_SPEED_DOWN);
            USART3_Printf("[CARPUT]     ① 先到 %u（%u RPM，等 %u ms）\r\n",
                          (unsigned)mid, (unsigned)LIFT_MOTOR_SPEED_DOWN, (unsigned)ms);
            lift_motor_move_at(mid, (uint8_t)LIFT_MOTOR_SPEED_DOWN);
            if(!cp_lift_wait_guarded(ms)){ lift_motor_stop(); return 0u; }

            Delay_ms(150);

            ms = lift_motor_ms_for(mid, aim, (uint8_t)LIFT_MOTOR_SPEED_CREEP);
            USART3_Printf("[CARPUT]     ② 爬行到 %u（%u RPM，等 %u ms）\r\n",
                          (unsigned)aim, (unsigned)LIFT_MOTOR_SPEED_CREEP, (unsigned)ms);
            lift_motor_move_at(aim, (uint8_t)LIFT_MOTOR_SPEED_CREEP);
            if(!cp_lift_wait_guarded(ms)){ lift_motor_stop(); return 0u; }

            Delay_ms(200);
            return 1u;
        }
    }
#endif

    /* 一段走到底（CP_LIFT_SOFT_LANDING=0 时走这里，排查堵转用） */
    ms = lift_motor_move_ms(g_lift_target, aim);
    USART3_Printf("[CARPUT]     一段到底（等 %u ms）\r\n", (unsigned)ms);
    lift_motor_move(aim, (aim > g_lift_target) ? 0x01u : 0x00u);
    if(!cp_lift_wait_guarded(ms)){
        lift_motor_stop();               /* 中止：补一帧急停，别让它继续走完 */
        return 0u;
    }

    Delay_ms(200);                       /* 等停稳再让下一步动舵机 */
    return 1u;
#endif  /* CP_STEP_NO_WAIT */
}

/* 把"回到执行前状态"的反向动作压栈
 * prev_lift 必须是**执行这个动作之前**读到的 g_lift_target */
static void cp_push_undo(const CarPutStep *s, uint32_t prev_lift)
{
    CarPutStep inv;

    if(!s->undo) return;
    if(g_cp_undo_n >= CP_UNDO_MAX) return;

    inv.kind      = s->kind;
    inv.id        = s->id;
    inv.deg       = 0u;
    inv.pulses    = 0u;
    inv.undo      = 0u;
    inv.label[0]  = '\0';

    if(s->kind == CP_KIND_SERVO){
        inv.deg = Servo_GetAngle((ServoId)s->id);      /* 执行前的角度 */
    }else{
        inv.pulses = prev_lift;                        /* ★ 执行前的绝对位置 */
    }

    g_cp_undo[g_cp_undo_n++] = inv;
}

/* 执行一个子动作（正向或撤销）；返回 0 = 被 PA5 中止。
 * out_label 回填这次做的是什么，给屏上用。 */
static uint8_t cp_step_once(char *out_label)
{
    CarPutStep a;
    uint32_t   prev_lift;

    if(g_cp_phase == CP_PH_IDLE) g_cp_phase = CP_PH_FWD;

    /* 正向已经走完：切到撤销阶段 */
    if((g_cp_phase == CP_PH_FWD) && (g_cp_fwd >= CP_STEP_COUNT)){
        g_cp_phase      = CP_PH_UNDO;
        g_cp_undo_total = g_cp_undo_n;
    }

    if(g_cp_phase == CP_PH_FWD){
        prev_lift = g_lift_target;                     /* ★ 先记下执行前的位置 */
        cp_push_undo(&k_cp_steps[g_cp_fwd], prev_lift);

        strcpy(out_label, k_cp_steps[g_cp_fwd].label);

        if(!cp_apply(&k_cp_steps[g_cp_fwd])) return 0u;

        g_cp_fwd++;

        if(g_cp_fwd >= CP_STEP_COUNT){
            g_cp_phase      = CP_PH_UNDO;
            g_cp_undo_total = g_cp_undo_n;
        }
        return 1u;
    }

    if(g_cp_phase == CP_PH_UNDO){
        if(g_cp_undo_n == 0u){
            g_cp_phase = CP_PH_DONE;
            strcpy(out_label, "DONE");
            return 1u;
        }

        a = g_cp_undo[--g_cp_undo_n];                  /* 后进先出 */
        g_cp_undo_done++;

        if(a.kind == CP_KIND_SERVO){
            sprintf(out_label, "U-%u", (unsigned)a.deg);
        }else{
            sprintf(out_label, "U-L%u", (unsigned)a.pulses);
        }

        if(!cp_apply(&a)) return 0u;

        if(g_cp_undo_n == 0u) g_cp_phase = CP_PH_DONE;
        return 1u;
    }

    strcpy(out_label, "DONE");
    return 1u;
}

/* 画界面 */
static void cp_show(char *label)
{
    OLED_ClearLine(1);
    if(g_cp_phase == CP_PH_UNDO || g_cp_phase == CP_PH_DONE){
        OLED_ShowString(1, 1, "CARPUT UNDO ");
        OLED_ShowNum(1, 13, (uint32_t)g_cp_undo_done, 1);
        OLED_ShowString(1, 14, "/");
        OLED_ShowNum(1, 15, (uint32_t)g_cp_undo_total, 1);
    }else{
        OLED_ShowString(1, 1, "CARPUT FWD  ");
        OLED_ShowNum(1, 13, (uint32_t)(g_cp_fwd + 1u), 1);
        OLED_ShowString(1, 14, "/");
        OLED_ShowNum(1, 15, (uint32_t)CP_STEP_COUNT, 1);
    }

    OLED_ClearLine(2);
    OLED_ShowString(2, 1, label);

    /* 3 行：三路舵机角度 */
    OLED_ShowString(3, 1, "G");
    OLED_ShowNum(3, 2, (uint32_t)Servo_GetAngle(SERVO_GRIP), 3);
    OLED_ShowString(3, 5, " R");
    OLED_ShowNum(3, 7, (uint32_t)Servo_GetAngle(SERVO_ROT), 3);
    OLED_ShowString(3, 10, " U");
    OLED_ShowNum(3, 12, (uint32_t)Servo_GetAngle(SERVO_LIFT), 3);

    /* 4 行：升降步进的绝对位置 + 键提示 */
    OLED_ClearLine(4);
    OLED_ShowString(4, 1, "L");
    OLED_ShowNum(4, 2, (uint32_t)g_lift_target, 4);
    OLED_ShowString(4, 7, " PC2=STEP");
}

/* 下降探针：从当前位置往下走一小段，读回实际位置，判定"下降停不停得住"。
 * 每次只走 200 脉冲，绝对安全，专门用来区分是代码问题还是驱动器/机械问题。 */
#define CP_PROBE_STEP   200u
#define CP_PROBE_TOL    60u

static void cp_descent_probe(void)
{
    uint32_t cur;
    uint32_t target;
    uint16_t ms;
    int32_t  a0 = 0, a1 = 0;
    uint8_t  ok0, ok1;
    int32_t  err;

    lift_motor_sync();
    cur = g_lift_target;

    if(cur < (uint32_t)(CP_PROBE_STEP + 200u)){
        USART3_Printf("[CARPUT] 探针拒绝：当前 %u 太低，先抬起来再测（>%u）\r\n",
                      (unsigned)cur, (unsigned)(CP_PROBE_STEP + 200u));
        return;
    }

    target = cur - CP_PROBE_STEP;

    ok0 = lift_motor_read_pos(&a0);

    USART3_SendString("[CARPUT] ===== 下降探针 =====\r\n");
    USART3_Printf("[CARPUT] 跟踪位置=%u  驱动器ACT=%d %s\r\n",
                  (unsigned)cur, (int)a0, ok0 ? "OK" : "(读不到)");
    USART3_Printf("[CARPUT] 目标 = %u（往下 %u 脉冲，下降速度 %u RPM）\r\n",
                  (unsigned)target, (unsigned)CP_PROBE_STEP,
                  (unsigned)LIFT_MOTOR_SPEED_DOWN);

    ms = lift_motor_move_ms(cur, target);
    lift_motor_move(target, 0x00u);

    if(!grab_wait(ms)){
        lift_motor_stop();
        USART3_SendString("[CARPUT] 探针被 PA5 中止\r\n");
        return;
    }

    Delay_ms(300);                         /* 等驱动器彻底停稳再读 */
    ok1 = lift_motor_read_pos(&a1);

    if(!ok0 || !ok1){
        USART3_SendString("[CARPUT] 读不到位置，无法判定。先修 0x36 通信\r\n");
        return;
    }

    err = a1 - (int32_t)target;

    USART3_Printf("[CARPUT] 走完 ACT=%d  期望=%d  误差=%d\r\n",
                  (int)a1, (int)target, (int)err);

    if(err < -(int32_t)CP_PROBE_TOL){
        USART3_Printf("[CARPUT] ❌ 冲过头 %d 脉冲（走比要的多）\r\n", (int)(-err));
        USART3_SendString("[CARPUT]   → 驱动器没在按绝对位置停，或者下降时被重力带过了头\r\n");
        USART3_SendString("[CARPUT]   → 对策：先把速度降一档再试（模式13 里发 V15），或加大 D 补偿\r\n");
    }else if(err > (int32_t)CP_PROBE_TOL){
        USART3_Printf("[CARPUT] ❌ 没走到位，差 %d 脉冲（走比要的少）\r\n", (int)err);
        USART3_SendString("[CARPUT]   → 等待时间不够（命令被下一条打断），或驱动器丢步\r\n");
    }else{
        USART3_SendString("[CARPUT] ✅ 下降停在目标上，绝对定位正常\r\n");
        USART3_SendString("[CARPUT]   → 说明这套组帧和驱动都没问题，之前跑飞要看具体是哪一步\r\n");
    }
}

/* 前向声明：十进制解析器定义在后面的「升降高度调试」段里，这里先用 */
static uint8_t lt_parse_u32(const char *s, uint32_t *out);

/* 前向声明：升降单独测试（模式 13）的主循环，定义在文件后面。
 * 模式 12 里发 L 可以直接跳进去 —— 不用改 DEBUG_MODE 重烧。 */
static void lift_test_run(void);

/* 打一行完整状态（含 K230 链路计数器，用来判断视觉到底通不通） */
static void cp_status(void)
{
    int32_t act = 0;
    uint8_t ok  = lift_motor_read_pos(&act);

    USART3_Printf("[CARPUT] STATUS phase=%u fwd=%u/%u undo=%u/%u MOT=%u ROT=%u GRIP=%u LIFT=%u\r\n",
                  (unsigned)g_cp_phase,
                  (unsigned)g_cp_fwd, (unsigned)CP_STEP_COUNT,
                  (unsigned)g_cp_undo_done, (unsigned)g_cp_undo_total,
                  (unsigned)g_lift_target,
                  (unsigned)Servo_GetAngle(SERVO_ROT),
                  (unsigned)Servo_GetAngle(SERVO_GRIP),
                  (unsigned)Servo_GetAngle(SERVO_LIFT));

    /* ★ 升降的"我们以为的位置" vs "驱动器自己报的位置"。
     *   两个数一偏离，就说明下降时丢步了（驱动器数的是它发出去的脉冲数，
     *   机械被重力带着多走了它并不知道），这就是"降过头"的根源。
     *   d = ACT - MOT，负得越多 = 机械比我们以为的更低。 */
    USART3_Printf("[CARPUT] 升降 MOT=%u ACT=%d d=%d  %s\r\n",
                  (unsigned)g_lift_target,
                  ok ? (int)act : -9999,
                  ok ? (int)(act - (int32_t)g_lift_target) : 0,
                  ok ? "OK" : "NO-ANSWER(0x36 没回)");

    /* ★ K230 链路健康度：C_seq 一直不动 = 一帧都没收到（命令没到 / 没在跑脚本）
     *   BAD 一直涨 = 收到字节了但格式不对（多半波特率/接线反了） */
    USART3_Printf("[CARPUT] K230 C_seq=%u NG=%u BAD=%u  last C(%d,%d,%d)\r\n",
                  (unsigned)K230_CircleSeq,
                  (unsigned)K230_NgSeq,
                  (unsigned)K230_BadLine,
                  (int)K230_Cx, (int)K230_Cy, (int)K230_R);
}

/* 执行一步 + 打日志；返回 1 = 正常 */
static uint8_t cp_do_step(char *label)
{
    uint8_t ok;

    if(g_cp_phase == CP_PH_DONE) cp_reset_state();   /* 上一轮做完了，重新开始 */

    motor_enable_addr(LIFT_MOTOR_ADDR);              /* 地址 6 上电默认失能 */
    Delay_ms(10);

    ok = cp_step_once(label);

    if(ok){
        pc13_step_blink();
        USART3_Printf("[CARPUT]   -> %s   MOT=%u ROT=%u GRIP=%u LIFT=%u  (fwd=%u undo=%u/%u)\r\n",
                      label,
                      (unsigned)g_lift_target,
                      (unsigned)Servo_GetAngle(SERVO_ROT),
                      (unsigned)Servo_GetAngle(SERVO_GRIP),
                      (unsigned)Servo_GetAngle(SERVO_LIFT),
                      (unsigned)g_cp_fwd,
                      (unsigned)g_cp_undo_done, (unsigned)g_cp_undo_total);
        if(g_cp_phase == CP_PH_DONE){
            USART3_SendString("[CARPUT] 已按栈恢复完，回到初始姿态\r\n");
        }
    }else{
        USART3_SendString("[CARPUT] 这一步被中止/失败，停在原地\r\n");
    }

    cp_show(label);
    return ok;
}

/* ============================================================================
 * ★ 一次按键 = 一个动作（2026-11-21 现场要求）
 * ----------------------------------------------------------------------------
 * 现场流程（用户口述）：
 *   按下 PC3     → 步进升到 7600（发完命令就走人，不等到位）
 *   再按 PC3     → 开始识别圆：发 circle，K230 持续回坐标，底盘自动修正
 *                  这一步是**循环**，会一直站在这里等你
 *   再按 PC3     → 底盘停住 + 顺手转到 163°（一次按键把"确认"和"下一步"都做掉）
 *   再按 PC3     → 升降舵机 130°
 *   再按 PC3     → 步进降到 1000
 *   再按 PC3     → 夹爪 110°
 *   …… 依此类推，每一按只做 k_cp_steps[] 里的下一条。
 *
 * ★ 不再有任何"等它走完"的延时（CP_STEP_NO_WAIT=1）：
 *   命令发出去就返回，上一条还在走也能直接按下一条 —— 到位没到位由人眼看。
 *   唯一的例外是视觉那一步，它本身就是个循环，必须站着等你按 PC3。
 *
 * PA5 任何时候 = 中止（会补急停帧）。
 * ========================================================================== */
static uint8_t cp_auto_sequence(char *label)
{
    uint8_t ok  = 1u;
    uint8_t vis = cp_vision_index();

    if(g_cp_phase == CP_PH_DONE) cp_reset_state();         /* 上一轮做完了，重新开始 */
    if(g_cp_phase == CP_PH_UNDO) g_cp_phase = CP_PH_FWD;   /* 从"回退"切回"前进" */

    if(g_cp_fwd >= CP_STEP_COUNT){
        USART3_SendString("[CARPUT] 全部动作已经做完了。要重来：先按 PC0 复位\r\n");
        cp_show(label);
        return 1u;
    }

    USART3_Printf("[CARPUT] 动作 %u/%u：%s\r\n",
                  (unsigned)(g_cp_fwd + 1u), (unsigned)CP_STEP_COUNT,
                  k_cp_steps[g_cp_fwd].label);

    if(g_cp_fwd == vis){
        USART3_SendString("[CARPUT]  ⏳ 这一步会一直修正底盘，**对准了再按一次 PC3**（PA5 中止）\r\n");
    }else if(g_cp_fwd == 0u){
        USART3_SendString("[CARPUT]  抬升命令已发（不等它走完）。等臂升起来再按 PC3 开始识别\r\n");
    }

    if(!cp_do_step(label)) return 0u;

#if CP_VISION_EXIT_NEXT
    /* 视觉那一步结束（PC3 确认对准）→ 同一按顺手把下一个动作也做掉 */
    if((vis < CP_STEP_COUNT) && (g_cp_fwd == (uint8_t)(vis + 1u))
       && (g_cp_fwd < CP_STEP_COUNT)){
        USART3_Printf("[CARPUT] 顺带下一个：%s\r\n", k_cp_steps[g_cp_fwd].label);
        if(!cp_do_step(label)) ok = 0u;
    }
#endif

    if(!ok){
        USART3_SendString("[CARPUT] ❌ 被中止/失败，停在原地（再按 PC3 从这一步继续）\r\n");
    }else if(g_cp_fwd >= CP_STEP_COUNT){
        g_cp_phase = CP_PH_DONE;       /* 全部做完了：停住，不自动弹栈 */
        USART3_SendString("[CARPUT] ✅ 全部动作发完了，就停在这个姿态\r\n");
        USART3_SendString("[CARPUT]    再跑一轮：直接按 PC3 就从头开始；回退用 B\r\n");
    }else{
        USART3_Printf("[CARPUT] 下一条是 %u/%u：%s（按 PC3 做它）\r\n",
                      (unsigned)(g_cp_fwd + 1u), (unsigned)CP_STEP_COUNT,
                      k_cp_steps[g_cp_fwd].label);
    }

    cp_show(label);
    return ok;
}

/* ============================================================================
 * ★ 关于「连跑 A」：已经停用
 * ----------------------------------------------------------------------------
 * 现在每个动作都是"发完命令立刻返回、不等到位"（CP_STEP_NO_WAIT=1）。
 * 如果还保留连跑，一条 A 命令会把 11 个动作在几毫秒内全发出去 ——
 * 机械臂还在原地，就要它同时"转 163°+降 1000+夹紧"，非常危险。
 * 所以 'A' 现在只提示一句，不再执行。要连跑请在 CP_STEP_NO_WAIT=0 时再用。
 * ========================================================================== */

/* 弹栈一步（逆序回退） */
static uint8_t cp_step_back(char *label)
{
    CarPutStep inv;

    if(g_cp_undo_n == 0u){
        USART3_SendString("[CARPUT] 栈是空的，没得回退\r\n");
        return 0u;
    }

    inv = g_cp_undo[--g_cp_undo_n];

    if((g_cp_phase == CP_PH_FWD) && (g_cp_fwd > 0u)) g_cp_fwd--;

    if(inv.kind == CP_KIND_SERVO){
        sprintf(label, "U-%u", (unsigned)inv.deg);
    }else{
        sprintf(label, "U-L%u", (unsigned)inv.pulses);
    }

    USART3_Printf("[CARPUT] BACK -> %s\r\n", label);

    motor_enable_addr(LIFT_MOTOR_ADDR);
    Delay_ms(10);

    if(!cp_apply(&inv)) return 0u;

    cp_show(label);
    return 1u;
}

/* 十六进制打印一个字节（USART3_Printf 不支持 %x，自己拼） */
static void cp_put_hex8(uint8_t v)
{
    static const char hx[] = "0123456789ABCDEF";

    USART3_SendByte((uint8_t)hx[(v >> 4) & 0x0Fu]);
    USART3_SendByte((uint8_t)hx[v & 0x0Fu]);
}

/* 上位机命令（无线串口 USART3 收，一行一条） */
static void cp_host_cmd(char *line, char *label)
{
    char c;
    uint16_t i;

    for(i = 0; line[i] != '\0'; i++){ }               /* 去行尾空白 */
    while((i > 0u) && ((line[i - 1u] == ' ') || (line[i - 1u] == '\r'))) i--;
    line[i] = '\0';
    if(line[0] == '\0') return;

    c = (char)((line[0] >= 'a' && line[0] <= 'z') ? (line[0] - 'a' + 'A') : line[0]);
    USART3_Printf("[CARPUT] HOST> %s\r\n", line);

    switch(c)
    {
        case 'N': cp_do_step(label);   break;         /* next 单步 */
        case 'A':                                      /* all 连跑：现在已停用 */
            USART3_SendString("[CARPUT] 连跑 A 已停用（每按一次只做一个动作，用 N 或 PC3）\r\n");
            break;
        case 'B': cp_step_back(label); break;         /* back 回退一步 */
        case 'R':                                      /* reset 内部状态 */
            cp_reset_state();
            strcpy(label, "READY");
            cp_show(label);
            USART3_SendString("[CARPUT] 已复位到第 1 步\r\n");
            break;
        case 'S': cp_status();         break;
        case 'D':                                     /* D<n>：下降补偿（脉冲） */
        {
            uint32_t arg;
            if(line[1] == '\0'){
                USART3_Printf("[CARPUT] 下降补偿 = %u 脉冲（越大停得越高；D0 = 关闭）\r\n",
                              (unsigned)g_cp_descend_trim);
                break;
            }
            if(!lt_parse_u32(&line[1], &arg) || (arg > 1000u)){
                USART3_SendString("[CARPUT] 用法 D<0~1000>：每次下降少走多少脉冲\r\n");
                break;
            }
            g_cp_descend_trim = arg;
            USART3_Printf("[CARPUT] 下降补偿 = %u 脉冲（下一条下降命令立即生效）\r\n",
                          (unsigned)arg);
            break;
        }
        case 'T': cp_descent_probe();  break;         /* 下降探针 */
        case 'V':                                     /* 单独跑一次视觉对准 */
        {
            char vlbl[10];
            USART3_SendString("[CARPUT] 单独执行 K230 圆心定位（不动机械臂）\r\n");
            if(cp_vision_align()){
                strcpy(vlbl, "CIRCLE OK");
            }else{
                strcpy(vlbl, "CIRCLE NG");
            }
            cp_show(vlbl);
            break;
        }
        case 'K':                                     /* 直接执行第 n 个动作 */
        {
            uint32_t arg;
            if(!lt_parse_u32(&line[1], &arg) || (arg < 1u) || (arg > (uint32_t)CP_STEP_COUNT)){
                USART3_Printf("[CARPUT] 用法 K<1~%u>：直接执行第 n 个动作\r\n",
                              (unsigned)CP_STEP_COUNT);
                break;
            }
            if(g_cp_phase == CP_PH_UNDO){
                USART3_SendString("[CARPUT] 正在恢复阶段，先按 R 复位再单点动作\r\n");
                break;
            }

            {
                uint8_t  idx = (uint8_t)(arg - 1u);
                uint32_t prev_lift = g_lift_target;

                motor_enable_addr(LIFT_MOTOR_ADDR);
                Delay_ms(10);

                USART3_Printf("[CARPUT] 单点动作 %u/%u：%s\r\n",
                              (unsigned)arg, (unsigned)CP_STEP_COUNT,
                              k_cp_steps[idx].label);

                cp_push_undo(&k_cp_steps[idx], prev_lift);
                strcpy(label, k_cp_steps[idx].label);

                if(cp_apply(&k_cp_steps[idx])){
                    /* 把"下一步"指针移到这个动作之后，后续 N 就从下一条继续 */
                    g_cp_fwd = (uint8_t)arg;
                    if(g_cp_phase == CP_PH_IDLE) g_cp_phase = CP_PH_FWD;
                    pc13_step_blink();
                    USART3_Printf("[CARPUT]   -> %s   MOT=%u ROT=%u GRIP=%u LIFT=%u\r\n",
                                  label,
                                  (unsigned)g_lift_target,
                                  (unsigned)Servo_GetAngle(SERVO_ROT),
                                  (unsigned)Servo_GetAngle(SERVO_GRIP),
                                  (unsigned)Servo_GetAngle(SERVO_LIFT));
                    USART3_SendString("[CARPUT]   注意：单点跳步会让回退栈不完整，"
                                      "要完整恢复请先 R 复位再从头走\r\n");
                }else{
                    USART3_SendString("[CARPUT]   这一步失败/被中止\r\n");
                }
                cp_show(label);
            }
            break;
        }
        case 'X':
            vision_stop_all();
            lift_motor_stop();
            USART3_SendString("[CARPUT] HOST -> 急停\r\n");
            break;

        case 'P':                                     /* 0x36 原始回包测试 */
        {
            uint8_t  k;
            uint16_t t;

            USART3_SendString("[CARPUT] 发 0x36 读位置，抓原始回包（最多等 300 ms）…\r\n");
            Serial_RxFlag = 0;
            Serial_SendByte(LIFT_MOTOR_ADDR);
            Serial_SendByte(0x36);
            Serial_SendByte(0x6B);

            for(t = 0u; t < 300u; t++){
                if(Serial_RxFlag != 0) break;
                Delay_ms(1);
            }

            USART3_Printf("[CARPUT] RxFlag=%u  等了 %u ms  raw =",
                          (unsigned)Serial_RxFlag, (unsigned)t);
            for(k = 0u; k < 10u; k++){
                USART3_SendByte(' ');
                cp_put_hex8(Serial_RxPacket[k]);
            }
            USART3_SendString("\r\n");
            USART3_SendString("[CARPUT]   正常应该是: 06 36 00 xx xx xx xx 6B\r\n");
            USART3_SendString("[CARPUT]   全 00 / RxFlag=0  → 驱动器根本没用 0x36 应答\r\n");
            break;
        }

        case 'L':
            /* ★ 不重烧就进"升降电机单独测试"：只动地址 6，四个轮子和 K230 都不参与，
             *   正好用来隔离"下降堵转是不是别的负载/供电拖出来的"。 */
            USART3_SendString("\r\n[CARPUT] --- 进入【升降步进单独测试】(模式 13) ---\r\n");
            USART3_SendString("[CARPUT]   只动地址 6 的升降电机，四个轮子 / K230 都不参与\r\n");
            USART3_SendString("[CARPUT]   U/D 按步进  G 走绝对位置  Z 回零  S 查状态  X 急停\r\n");
            USART3_SendString("[CARPUT]   V<rpm> 改速度  A<accel> 改加速度档  W 分段下降到 0\r\n");
            USART3_SendString("[CARPUT]   ★ 这个测试不返回：要回取放流程请按 PC0 复位或重新上电\r\n");
            lift_test_run();
            break;
        default:
            USART3_SendString("[CARPUT] 命令: N 单步 B 回退 R 复位 S 状态 T 探针 D<n> 补偿 K<n> 单点 X 急停 L 升降测试 P 读位置原始包\r\n");
            USART3_Printf("[CARPUT] 流程: 正向 %u 步，之后按栈逆序恢复\r\n",
                          (unsigned)CP_STEP_COUNT);
            break;
    }
}

/* 栈式单步调试主循环：不返回 */
static void car_put_run(void)
{
    char    label[10];
    char    cmdline[24];
    uint16_t pc2_gap = 0u;
    uint16_t pc3_gap = 0u;

    grab_key_init();            /* PA5 */
    st_pc_init();               /* PC0 / PC2 / PC3 */

    OLED_Clear();
    strcpy(label, "READY");
    cp_show(label);
    OLED_ShowString(4, 1, "PC3=NEXT PC0=RST");

    USART3_SendString("\r\n[CARPUT] 从车上取料 → 放地上（★ 每按一次 PC3 = 一个动作，不等到位）\r\n");
    USART3_Printf("[CARPUT] 正向 %u 个动作\r\n", (unsigned)CP_STEP_COUNT);
    USART3_Printf("[CARPUT] 升降步进(绝对): 高位 %u / 抓料 %u / 零位 %u   下降补偿 %u 脉冲\r\n",
                  (unsigned)CP_MOT_HIGH, (unsigned)CP_MOT_GRAB, (unsigned)CP_MOT_HOME,
                  (unsigned)g_cp_descend_trim);
    USART3_Printf("[CARPUT] 升降舵机: 抬臂 %u° / 初始 %u°\r\n",
                  (unsigned)CP_LIFT_UP_DEG, (unsigned)CP_LIFT_HOME);
    USART3_Printf("[CARPUT] 夹爪: 夹紧 %u° / 松开 %u°   旋转初始 %u°\r\n",
                  (unsigned)CP_GRIP_HOLD, (unsigned)CP_GRIP_HOME, (unsigned)CP_ROT_HOME);
    USART3_SendString("[CARPUT] 升降步进按【绝对位置】发（byte10=0x01，脉冲字段=目标位置）\r\n");
    USART3_SendString("[CARPUT] 板载: PC3=做下一个动作(视觉时=对准完成) PC2=同 PC0=复位 PA5=中止\r\n");
    USART3_SendString("[CARPUT] 上位机: N 单步/D<n> 补偿/T 探针/K<n> 单点/B 回退/R 复位/S 状态/X 急停/L 升降单独测试\r\n");
    USART3_SendString("[CARPUT] 流程: PC3抬升 -> PC3开始识别+底盘修正 -> PC3确认并转163° -> 依次 舵机/步进/夹爪 …\r\n");

    while(1)
    {
        /* ---------- 上位机命令 ---------- */
        if(USART3_ReadLine(cmdline, sizeof(cmdline))){
            cp_host_cmd(cmdline, label);
            pc2_gap = 60u;      /* 命令期间屏蔽按键，避免手误叠加 */
            pc3_gap = 60u;
            continue;
        }

        /* ---------- PC3：★ 半自动（抬升 → 识别圆 → 你按 PC3 → 夹取放料） ---------- */
        if((pc3_gap == 0u) && st_pc_held(GPIO_Pin_3)){
            uint16_t hold;
            pc3_gap = 400u;
            USART3_SendString("[CARPUT] --- PC3 下一个动作 ---\r\n");
            /* 先等松手再开跑，免得"按住"被当成连按 */
            for(hold = 0u; st_pc_held(GPIO_Pin_3) && (hold < 200u); hold++) Delay_ms(10);
            cp_auto_sequence(label);
            pc3_gap = 400u;          /* 半自动回来后屏蔽一段，防抖 */
            continue;
        }

        /* ---------- PC2：单步（备用，一次只做一个动作；调试用） ---------- */
        if((pc2_gap == 0u) && st_pc_held(GPIO_Pin_2)){
            uint16_t hold;
            pc2_gap = 400u;
            USART3_SendString("[CARPUT] --- PC2 单步（一次一个动作） ---\r\n");
            cp_do_step(label);
            for(hold = 0u; st_pc_held(GPIO_Pin_2) && (hold < 200u); hold++) Delay_ms(10);
            continue;
        }

        /* ---------- PB1：连跑（和 PC3 一样是半自动，视觉处等 PC3） ---------- */
        if(grab_step_hit()){
            USART3_SendString("[CARPUT] --- PB1 下一个动作 ---\r\n");
            cp_auto_sequence(label);
            continue;
        }

        /* ---------- PC0：复位内部状态（不动硬件） ---------- */
        if(st_pc_held(GPIO_Pin_0)){
            cp_reset_state();
            strcpy(label, "READY");
            cp_show(label);
            USART3_SendString("[CARPUT] PC0 -> 复位到第 1 步（硬件没动）\r\n");
            while(st_pc_held(GPIO_Pin_0)) Delay_ms(10);
            continue;
        }

        /* ---------- PA5：中止 ---------- */
        if(grab_abort_hit()){
            vision_stop_all();
            lift_motor_stop();
            USART3_SendString("[CARPUT] PA5 -> 急停\r\n");
            strcpy(label, "STOP");
            cp_show(label);
            grab_wait_release(GPIOA, GPIO_Pin_5);
            Delay_ms(200);
            continue;
        }

        if(pc2_gap > 0u) pc2_gap = (pc2_gap > 10u) ? (uint16_t)(pc2_gap - 10u) : 0u;
        if(pc3_gap > 0u) pc3_gap = (pc3_gap > 10u) ? (uint16_t)(pc3_gap - 10u) : 0u;

        Delay_ms(10);
    }
}

/* ============================================================================
 * 升降步进电机高度调试（DEBUG_MODE_LIFT_TEST = 13）
 * ----------------------------------------------------------------------------
 * 只动升降电机（总线地址 6），角度/底盘/夹爪全都不碰。
 * 所有信息打到**无线串口**（USART3 = PB10/PB11，115200 8N1）。
 *
 * 按键：
 *     PB1   上升 step 脉冲
 *     PA7   下降 step 脉冲
 *     PC2   切换步长：10 / 50 / 100 / 500 / 1000 脉冲
 *     PC3   回零（命令绝对位置 0）
 *     PA5   急停
 *
 * 为什么要有这个模式、以及它跟"降多了"的关系：
 *   升降电机用的是"绝对位置"帧（byte[10] = 0x01），目标是从上电位置算起的脉冲数。
 *   但以前 USART1 的接收中断只认地址 0x01~0x05、并且写死 4 字节帧，
 *   所以地址 6 的回包**整帧被丢掉**、0x36 那 8 字节也拼不起来 ——
 *   我们从来不知道驱动器实际在哪，只能"以为"它在哪。
 *   现在改成按功能码决定帧长（0x3A=4 字节 / 0x36=8 字节）并认地址 6，
 *   所以这个模式能同时打出：
 *       CMD = 我们以为的位置（g_lift_target，我们自己维护的）
 *       ACT = 驱动器自己报的实际位置（0x36 读回来）
 *   两个数一旦对不上，就说明绝对定位没按预期生效。
 *
 * 串口输出示例：
 *     [LIFT] RAW 06 36 00 00 00 03 E8 6B  -> OK act=1000 cmd=1000 d=0
 *     [LIFT] UP  1000 -> 1100  (expect 460 ms)
 *     [LIFT] RAW 06 36 00 00 00 04 4C 6B  -> OK act=1100 cmd=1100 d=0
 *   如果 ACT 变成负数（符号位 = 0x01），就是你说的"降到上电零点以下"。
 *
 * 安全：
 *   - 目标位置夹在 [0, LIFT_MOTOR_MAX]，命令侧永远不会发负数
 *   - 抬起高度按行程算等待时间，不会半路叠命令
 *   - PA5 随时急停（会补一帧 0xFE 0x98）
 * ========================================================================== */
#define DEBUG_MODE_LIFT_TEST 13

/* PC2 循环切换的步长表（下标 0 是上电默认） */
static const uint16_t k_lt_steps[] = { 10u, 50u, 100u, 500u, 1000u };
#define LT_STEP_N   ((uint8_t)(sizeof(k_lt_steps) / sizeof(k_lt_steps[0])))

#define LT_LOOP_MS          10u
#define LT_NOW_MS           1000u    /* 无线串口心跳间隔 */
#define LT_MS_PER_1000P     600u     /* 升降每 1000 脉冲大约走多久（ms） */
#define LT_EXTRA_MS         400u     /* 每步额外余量 */

/* 当前实际位置（从驱动器读回来）；lt_have_act = 0 表示这次没读到 */
static int32_t  lt_act      = 0;
static uint8_t  lt_have_act = 0u;

/* 位置校验容差：命令位置和驱动器实际位置差超过这个就判"位置不符"并锁死。
 * 600 脉冲 ≈ 6 mm，正常跟踪误差远小于它。 */
#define LT_VERIFY_TOL   600u

/* 故障锁：一旦发现"发了 4000 却跑到底"这种位置不符，就禁止后续动作，
 * 防止越点越远。按 Z（回零）/ 发 C 清除。 */
static uint8_t  lt_fault = 0u;

/* 动到 target：发绝对位置帧 → 按行程等 → 读回实际位置 → 校验 → 打串口 */
static uint8_t lt_move_to(uint32_t target, const char *tag)
{
    uint32_t cur = g_lift_target;
    uint32_t delta;
    uint16_t ms;
    int32_t  err;

    if(lt_fault){
        USART3_SendString("[LIFT] FAULT 锁着：位置不符还没清。按 Z 回零，或发 C 清除\r\n");
        return 0u;
    }

    lift_motor_sync();                 /* 先用驱动器实际位置校准跟踪值 */

    if(target > LIFT_MOTOR_MAX) target = LIFT_MOTOR_MAX;

    if(target == cur){
        USART3_Printf("[LIFT] %s  already at %u\r\n", tag, (unsigned)cur);
        lt_have_act = lift_motor_report();
        return 1u;
    }

    delta = (target > cur) ? (target - cur) : (cur - target);
    ms = lift_motor_move_ms(cur, target);
    (void)delta;

    USART3_Printf("[LIFT] %s  %u -> %u  (expect %u ms)\r\n",
                  tag, (unsigned)cur, (unsigned)target, (unsigned)ms);

    lift_motor_move(target, (target > cur) ? 0x01u : 0x00u);

    if(!grab_wait(ms)){
        lift_motor_stop();
        USART3_SendString("[LIFT] ABORT by PA5\r\n");
        return 0u;
    }

    lt_have_act = lift_motor_report();

    /* ★ 位置校验：这是"下降到 4000 却一路到底"的拦截阀。
     * 只要驱动器没停在目标上，立刻急停并锁住，绝不让下一次点击继续往外走。 */
    if(lt_have_act){
        err = lt_act - (int32_t)target;
        if(err < 0) err = -err;

        if(err > (int32_t)LT_VERIFY_TOL){
            lift_motor_stop();
            USART3_Printf("[LIFT] !!! 位置不符：目标 %u，实际 %d，差 %d 脉冲\r\n",
                          (unsigned)target, (int)lt_act, (int)err);
            USART3_SendString("[LIFT] !!! 已急停并锁定后续动作。\r\n");
            USART3_SendString("[LIFT] !!! 说明驱动器没在按绝对位置执行 —— 这就是『下降一路到底』的原因\r\n");
            USART3_SendString("[LIFT] !!! 处理：按 Z 回零 / 发 C 清故障 / 发 R1000 试相对模式\r\n");
            lt_fault = 1u;
            return 0u;
        }
    }

    return 1u;
}

static void lt_show(uint16_t step)
{
    OLED_ClearLine(1);
    OLED_ShowString(1, 1, "LIFT ST:");
    OLED_ShowNum(1, 9, (uint32_t)step, 4);

    OLED_ShowString(2, 1, "CMD:");
    OLED_ShowNum(2, 5, (uint32_t)g_lift_target, 5);

    OLED_ShowString(3, 1, "ACT:");
    if(lt_have_act){
        OLED_ShowSignedNum(3, 5, lt_act, 6);
    }else{
        OLED_ShowString(3, 5, "  ----");
    }
}

/* ============================================================================
 * 上位机命令行（无线串口 USART3 收，一行一条，\r\n 结尾）
 * ----------------------------------------------------------------------------
 *    U<step>    上升 step 脉冲              例 U100
 *    D<step>    下降 step 脉冲              例 D100
 *    G<pos>     走到绝对位置 pos            例 G1500
 *    Z          回零（绝对 0）
 *    S          查询状态 → 回一行 STATUS
 *    X          急停（0xFE 0x98）
 *    M<max>     设置软上限（只改 RAM，断电丢）  例 M8000
 *    H          打帮助
 *
 * 每条命令都会先回一行 [LIFT] HOST> ... 做回声，再走动作 + 回 RAW/OK 行。
 * 上位机 tools/lift_host.py 就是按这个协议点的按钮。
 * ========================================================================== */
static uint16_t lt_step    = 100u;   /* 当前步长 */
static uint8_t  lt_step_idx = 2u;    /* 当前步长下标 */

/* 把字符串前面的十进制数取出来；成功返回 1 */
static uint8_t lt_parse_u32(const char *s, uint32_t *out)
{
    uint32_t v = 0u;
    uint8_t  n = 0u;

    while((*s == ' ') || (*s == '\t')) s++;
    while((*s >= '0') && (*s <= '9')){
        v = v * 10u + (uint32_t)(*s - '0');
        s++;
        n++;
        if(n > 7u) return 0u;        /* 太大，当非法 */
    }
    if(n == 0u) return 0u;

    *out = v;
    return 1u;
}

/* 处理一条上位机命令 */
static void lt_host_cmd(char *line)
{
    uint32_t arg;
    char     c;
    uint16_t i;

    /* 去掉行尾空白 */
    for(i = 0; line[i] != '\0'; i++){ }
    while((i > 0u) && ((line[i - 1u] == ' ') || (line[i - 1u] == '\r'))) i--;
    line[i] = '\0';

    if(line[0] == '\0') return;

    c = (char)((line[0] >= 'a' && line[0] <= 'z') ? (line[0] - 'a' + 'A') : line[0]);
    USART3_Printf("[LIFT] HOST> %s\r\n", line);

    switch(c)
    {
        case 'U':
            if(!lt_parse_u32(&line[1], &arg)) arg = lt_step;
            if(arg == 0u) arg = lt_step;
            if(arg > 5000u) arg = 5000u;
            lt_move_to(g_lift_target + arg, "UP");
            break;

        case 'D':
            if(!lt_parse_u32(&line[1], &arg)) arg = lt_step;
            if(arg == 0u) arg = lt_step;
            if(arg > 5000u) arg = 5000u;
            if(g_lift_target > arg) lt_move_to(g_lift_target - arg, "DOWN");
            else                    lt_move_to(LIFT_MOTOR_HOME, "DOWN->0");
            break;

        case 'G':
            if(!lt_parse_u32(&line[1], &arg)){
                USART3_SendString("[LIFT] G 后面要跟目标脉冲数，例 G1500\r\n");
                break;
            }
            lt_move_to(arg, "GO");
            break;

        case 'Z':
            lt_fault = 0u;                 /* 回零同时清故障 */
            lt_move_to(LIFT_MOTOR_HOME, "ZERO");
            break;

        case 'C':
            lt_fault = 0u;
            USART3_SendString("[LIFT] 故障已清除\r\n");
            break;

        case 'R':
            /* 相对移动：用来判断驱动器到底认不认"绝对位置"。
             * 发 R1000 上升，如果实际也走 1000，并且和 G 命令的行为不同，
             * 就能确认驱动器在跑相对模式。 */
            {
                const char *p = &line[1];
                uint8_t  rel_up = 1u;
                uint16_t rel_ms;

                if(*p == '-'){ rel_up = 0u; p++; }
                else if(*p == '+'){ p++; }

                if(!lt_parse_u32(p, &arg) || (arg == 0u)){
                    USART3_SendString("[LIFT] 用法：R1000（相对上升 1000）/ R-1000（相对下降）\r\n");
                    break;
                }
                if(arg > 5000u) arg = 5000u;

                rel_ms = (uint16_t)((arg * LT_MS_PER_1000P) / 1000u + LT_EXTRA_MS);

                USART3_Printf("[LIFT] REL %s %u 脉冲（相对帧 byte10=0x00）\r\n",
                              rel_up ? "UP" : "DOWN", (unsigned)arg);
                lift_motor_move_rel(arg, rel_up ? 0x01u : 0x00u);
                (void)grab_wait(rel_ms);
                lt_have_act = lift_motor_report();
                USART3_SendString("[LIFT] 相对移动后 g_lift_target 不再可信，请按 Z 回零重建参考\r\n");
            }
            break;

        case 'S':
            lt_have_act = lift_motor_read_pos(&lt_act);
            USART3_Printf("[LIFT] STATUS cmd=%u act=%d d=%d step=%u max=%u spd=%u acc=%u %s\r\n",
                          (unsigned)g_lift_target,
                          (int)lt_act,
                          (int)(lt_act - (int32_t)g_lift_target),
                          (unsigned)lt_step,
                          (unsigned)LIFT_MOTOR_MAX,
                          (unsigned)lift_motor_get_speed_ovr(),
                          (unsigned)lift_motor_get_accel(),
                          lt_have_act ? "OK" : "NO-ANSWER");
            break;

        case 'X':
            lift_motor_stop();
            USART3_SendString("[LIFT] HOST -> 急停\r\n");
            break;

        case 'M':
            if(!lt_parse_u32(&line[1], &arg)){
                USART3_Printf("[LIFT] 软上限 = %u（没改，M 后面要跟数字）\r\n",
                              (unsigned)LIFT_MOTOR_MAX);
                break;
            }
            /* 只改 RAM，不改头文件里的宏；断电恢复原值 */
            USART3_Printf("[LIFT] 软上限请求 %u，编译期宏 %u（当前实现以宏为准，断电也不变）\r\n",
                          (unsigned)arg, (unsigned)LIFT_MOTOR_MAX);
            break;

        case 'V':
            if(!lt_parse_u32(&line[1], &arg)){
                USART3_Printf("[LIFT] 速度覆盖 = %u RPM（0 = 不覆盖；例 V12）\r\n",
                              (unsigned)lift_motor_get_speed_ovr());
                break;
            }
            if(arg > 60u) arg = 60u;
            lift_motor_set_speed_ovr((uint8_t)arg);
            USART3_Printf("[LIFT] 速度覆盖 = %u RPM（0 = 恢复默认：上升 %u / 下降 %u）\r\n",
                          (unsigned)lift_motor_get_speed_ovr(),
                          (unsigned)LIFT_MOTOR_SPEED, (unsigned)LIFT_MOTOR_SPEED_DOWN);
            break;

        case 'A':
            if(!lt_parse_u32(&line[1], &arg)){
                USART3_Printf("[LIFT] 加速度档 = %u（0 = 恢复默认 %u；例 A40）\r\n",
                              (unsigned)lift_motor_get_accel(), (unsigned)LIFT_MOTOR_ACCEL);
                break;
            }
            if(arg > 255u) arg = 255u;
            lift_motor_set_accel((uint8_t)arg);
            USART3_Printf("[LIFT] 加速度档 = %u（越小斜坡越平、起步电流越小）\r\n",
                          (unsigned)lift_motor_get_accel());
            break;

        case 'W':
            /* ★ 把**当前位置**设成驱动器零点（Emm42 的 0x93 0x88）。
             *   用法：先用 ↓ 一点点降到你要当"0"的地方，再发 W。
             *   之后 G<数字> 才是"离这里多少脉冲"。 */
            lift_motor_set_zero();
            USART3_SendString("[LIFT] 已把当前位置设为驱动器零点（0x93 0x88）\r\n");
            USART3_SendString("[LIFT]   现在 cmd=0；之后 G<pos> 都从这点算\r\n");
            lt_fault = 0u;                 /* 设完零点顺手清故障锁 */
            break;

        case 'P':
        {
            /* ★ 抓 0x36 的原始回包：判断"驱动器到底答不答"、答几个字节、格式对不对 */
            uint8_t  k;
            uint16_t t;

            USART3_SendString("[LIFT] 发 0x36，抓原始回包（最多等 300 ms）…\r\n");
            Serial_RxFlag = 0;
            Serial_SendByte(LIFT_MOTOR_ADDR);
            Serial_SendByte(0x36);
            Serial_SendByte(0x6B);

            for(t = 0u; t < 300u; t++){
                if(Serial_RxFlag != 0) break;
                Delay_ms(1);
            }

            USART3_Printf("[LIFT] RxFlag=%u 等了%ums raw =",
                          (unsigned)Serial_RxFlag, (unsigned)t);
            for(k = 0u; k < 10u; k++){
                USART3_SendByte(' ');
                cp_put_hex8(Serial_RxPacket[k]);
            }
            USART3_SendString("\r\n");
            USART3_SendString("[LIFT]   正常应是 06 36 00 xx xx xx xx 6B；全 00 或 RxFlag=0 = 驱动器不答\r\n");
            break;
        }

        case 'E':
        {
            /* ★ 分段下降到 0：每段 500 脉冲，段间停 300 ms，每段都读回位置。
             *   堵转时**最后打印的那一段**就是堵转高度：
             *     - 每次都在同一个高度卡住 → 机械问题（导轨/皮带/限位）
             *     - 随便哪段都可能卡       → 速度/电流/供电问题 */
            uint8_t n = 0u;

            USART3_SendString("[LIFT] 分段下降到 0：每段 500 脉冲，段间停 300 ms\r\n");
            while((g_lift_target > 0u) && (n < 40u)){
                uint32_t next = (g_lift_target > 500u) ? (g_lift_target - 500u) : 0u;
                if(!lt_move_to(next, "SEG-DN")) break;
                if(lt_fault) break;
                n++;
                Delay_ms(300);
            }
            USART3_Printf("[LIFT] 分段下降结束：cmd=%u，共走了 %u 段\r\n",
                          (unsigned)g_lift_target, (unsigned)n);
            break;
        }

        case 'H':
        default:
            USART3_SendString("[LIFT] 移动: U<步> D<步> G<绝对位置> Z(回零) E(分段下降到0)\r\n");
            USART3_SendString("[LIFT] 零点: W(把当前位置设为0)  查询: S 状态  P 0x36原始包\r\n");
            USART3_SendString("[LIFT] 参数: V<rpm> 速度  A<accel> 加速度档  其他: X 急停 C 清故障 R 相对\r\n");
            break;
    }

    lt_have_act = lift_motor_read_pos(&lt_act);
    lt_show(lt_step);
}

/* 升降高度调试主循环：不返回 */
static void lift_test_run(void)
{
    uint16_t now_gap;
    uint16_t pc2_gap = 0u;
    uint16_t pc3_gap = 0u;
    uint8_t  pa7_prev = 0u;
    uint8_t  pa5_prev = 0u;
    char     cmdline[24];

    grab_key_init();               /* PA7 / PA5 */
    st_pc_init();                  /* PC0 / PC2 / PC3 */
    motor_enable_addr(LIFT_MOTOR_ADDR);

    lt_step = k_lt_steps[lt_step_idx];

    OLED_Clear();
    OLED_ShowString(4, 1, "PC2+ PC3- PC0=Z");

    USART3_SendString("\r\n[LIFT] ===== 升降电机高度调试 =====\r\n");
    USART3_Printf("[LIFT] 地址 %u  上升 %u / 下降 %u RPM  加速度 %u  软上限 %u 脉冲\r\n",
                  (unsigned)LIFT_MOTOR_ADDR,
                  (unsigned)LIFT_MOTOR_SPEED, (unsigned)LIFT_MOTOR_SPEED_DOWN,
                  (unsigned)LIFT_MOTOR_ACCEL, (unsigned)LIFT_MOTOR_MAX);
    USART3_SendString("[LIFT] 板载按键: PC2=上升 PC3=下降 PC0=回零 PB1=换步长 PA7=读位置 PA5=急停\r\n");
    USART3_SendString("[LIFT] 上位机: U<步> D<步> G<位置> Z 回零 W 设零点 P 原始包 E 分段下降到 0\r\n");
    USART3_SendString("[LIFT]         S 状态 X 急停 C 清故障 V<rpm> 速度 A<accel> 加速度档 H 帮助\r\n");
    USART3_SendString("[LIFT] 每次移动后都会校验 ACT 和命令位置，差超过 600 脉冲就急停并锁定\r\n");

    /* 上电先读一次驱动器实际位置：
     *   读不到（NO-ANSWER）→ 地址 6 不应答 / 线没接 / 驱动器关了应答
     *   读到但不是 0    → 驱动器在最后一次断电时的位置，说明零点会随断电漂移 */
    lt_have_act = lift_motor_read_pos(&lt_act);
    if(lt_have_act){
        USART3_Printf("[LIFT] 上电实际位置 = %d 脉冲（驱动器零点参考）\r\n", (int)lt_act);
    }else{
        USART3_SendString("[LIFT] 上电读位置失败：地址 6 没回 0x36\r\n");
    }
    USART3_SendString("[LIFT] 上电先把 CMD 对齐到实际位置，避免一上来就跳一大段\r\n");
    if(lt_have_act && (lt_act > 0)){
        g_lift_target = (uint32_t)lt_act;      /* 以驱动器实际位置为起点 */
    }else{
        g_lift_target = LIFT_MOTOR_HOME;
    }

    now_gap = LT_NOW_MS;
    lt_show(lt_step);

    while(1)
    {
        uint8_t pa7 = grab_key_down(GPIOA, GPIO_Pin_7) ? 1u : 0u;
        uint8_t pa5 = grab_key_down(GPIOA, GPIO_Pin_5) ? 1u : 0u;

        if(now_gap > 0u) now_gap = (now_gap > LT_LOOP_MS) ? (uint16_t)(now_gap - LT_LOOP_MS) : 0u;
        if(pc2_gap  > 0u) pc2_gap = (pc2_gap > LT_LOOP_MS) ? (uint16_t)(pc2_gap - LT_LOOP_MS) : 0u;
        if(pc3_gap  > 0u) pc3_gap = (pc3_gap > LT_LOOP_MS) ? (uint16_t)(pc3_gap - LT_LOOP_MS) : 0u;

        /* ---------- 上位机命令（无线串口收一行） ---------- */
        if(USART3_ReadLine(cmdline, sizeof(cmdline))){
            lt_host_cmd(cmdline);
            continue;
        }

        /* ---------- PA5：急停 ---------- */
        if(pa5 && !pa5_prev){
            vision_stop_all();
            lift_motor_stop();
            USART3_SendString("[LIFT] PA5 -> 急停\r\n");
            lt_have_act = lift_motor_read_pos(&lt_act);
            lt_show(lt_step);
        }
        pa5_prev = pa5;

        /* ---------- PC2：上升（★ 一次按下只走一步，等松手才允许下一步） ---------- */
        if((pc2_gap == 0u) && st_pc_held(GPIO_Pin_2)){
            uint16_t hold;
            pc2_gap = 400u;                /* 防抖 + 防连击 */
            USART3_Printf("[LIFT] --- PC2 上升 step=%u ---\r\n", (unsigned)lt_step);
            lt_move_to(g_lift_target + lt_step, "UP");
            lt_have_act = lift_motor_read_pos(&lt_act);
            lt_show(lt_step);
            for(hold = 0u; st_pc_held(GPIO_Pin_2) && (hold < 200u); hold++) Delay_ms(10);
            now_gap = LT_NOW_MS;
            continue;
        }

        /* ---------- PC3：下降（同样一次一步） ---------- */
        if((pc3_gap == 0u) && st_pc_held(GPIO_Pin_3)){
            uint16_t hold;
            pc3_gap = 400u;
            USART3_Printf("[LIFT] --- PC3 下降 step=%u ---\r\n", (unsigned)lt_step);
            if(g_lift_target > lt_step){
                lt_move_to(g_lift_target - lt_step, "DOWN");
            }else{
                lt_move_to(LIFT_MOTOR_HOME, "DOWN->0");
            }
            lt_have_act = lift_motor_read_pos(&lt_act);
            lt_show(lt_step);
            for(hold = 0u; st_pc_held(GPIO_Pin_3) && (hold < 200u); hold++) Delay_ms(10);
            now_gap = LT_NOW_MS;
            continue;
        }

        /* ---------- PC0：回零 ---------- */
        if(st_pc_held(GPIO_Pin_0)){
            USART3_SendString("[LIFT] --- PC0 回零（命令绝对 0）---\r\n");
            lt_move_to(LIFT_MOTOR_HOME, "ZERO");
            lt_have_act = lift_motor_read_pos(&lt_act);
            lt_show(lt_step);
            while(st_pc_held(GPIO_Pin_0)) Delay_ms(10);
            continue;
        }

        /* ---------- PB1：换步长 ---------- */
        if(grab_step_hit()){
            lt_step_idx = (uint8_t)((lt_step_idx + 1u) % LT_STEP_N);
            lt_step = k_lt_steps[lt_step_idx];
            USART3_Printf("[LIFT] 步长 -> %u 脉冲\r\n", (unsigned)lt_step);
            lt_show(lt_step);
            continue;
        }

        /* ---------- PA7：读一次实际位置 ---------- */
        if(pa7 && !pa7_prev){
            lt_have_act = lift_motor_read_pos(&lt_act);
            USART3_Printf("[LIFT] 读位置 cmd=%u act=%d d=%d %s\r\n",
                          (unsigned)g_lift_target, (int)lt_act,
                          (int)(lt_act - (int32_t)g_lift_target),
                          lt_have_act ? "OK" : "NO-ANSWER");
            lt_show(lt_step);
        }
        pa7_prev = pa7;

        /* ---------- 心跳：每秒打一次 CMD / ACT ---------- */
        if(now_gap == 0u){
            now_gap = LT_NOW_MS;
            lt_have_act = lift_motor_read_pos(&lt_act);
            USART3_Printf("[LIFT] NOW cmd=%u act=%d d=%d step=%u %s\r\n",
                          (unsigned)g_lift_target,
                          (int)lt_act,
                          (int)(lt_act - (int32_t)g_lift_target),
                          (unsigned)lt_step,
                          lt_have_act ? "OK" : "NO-ANSWER");
            lt_show(lt_step);
        }

        Delay_ms(LT_LOOP_MS);
    }
}
/* ============================================================================
 * 升降电机「原始帧」判定测试（DEBUG_MODE_LIFT_RAW = 14）—— 最保守的单独测试
 * ----------------------------------------------------------------------------
 * 目的：一次性、无歧义地判定驱动器到底按【相对脉冲】还是【绝对位置】执行。
 *       之前的现象（上升看着对、下降一路到底）就是这两种语义搞混导致的，
 *       再猜没有意义，这个模式让它自己给结论。
 *
 * 【为什么这么测：靠"累计不累计"】
 *   每次都发一模一样的帧（脉冲字段 = 20，方向向上），连发 5 次：
 *       相对模式 → 实际累计走 +100（20,40,60,80,100）
 *       绝对模式 → 每次都去"位置 20"，原地不动，累计变化 ≈ 0
 *   然后连发 5 次向下的 20：
 *       相对模式 → 回到起点（-100）
 *       绝对模式 → 还是不动
 *   两种情况下总位移都 ≤ 100 脉冲，**绝对安全**，不会撞限位。
 *
 * 【为什么必须从低位开始】
 *   万一驱动器是"绝对"语义，而机械臂正停在 7000，那么发"位置 20"会直接从
 *   7000 掉到 20 —— 很危险。所以自动测试会先读一次 ACT，
 *   位置 > 1000 时**拒绝执行**，提示先把机械臂放低（或重新上电）。
 *
 * 这个模式**不经过** lift_motor_move()，直接拼原始帧：
 *      帧里 pulses = 参数原样，byte[10] = 0x01（和底盘轮子同一个值）
 *   所以它测的是驱动器本身，不掺任何我们的换算逻辑。
 *
 * 按键 / 上位机命令：
 *     PC2 或 U<step>    发一次"向上 step"原始帧（默认 20）
 *     PC3 或 D<step>    发一次"向下 step"原始帧
 *     PC0 或 T          跑自动判定（5 上 + 5 下，打印结论）
 *         或 S          读一次位置
 *         或 R          切 byte[10]（0x01 / 0x00）做对照
 *     PA5 或 X          急停
 * ========================================================================== */
#define DEBUG_MODE_LIFT_RAW 14

#define LR_STEP_DEF   20u     /* 单次脉冲数（小到不可能撞限位） */
#define LR_READ_MS    700u    /* 每次移动后等多久再读位置（20 脉冲很快） */
#define LR_SAFE_MAX   1000u   /* 起点位置高于这个就拒绝自动测试 */

static uint16_t lr_step  = LR_STEP_DEF;
static uint8_t  lr_rel   = 0u;    /* 0 = byte[10] 填 0x01（和底盘一样）；1 = 填 0x00 */

/* 直接拼一条原始位置模式帧，脉冲字段 = pulses（不做任何换算） */
static void lr_send_raw(uint32_t pulses, uint8_t up)
{
    uint8_t cmd[13] = {
        LIFT_MOTOR_ADDR,
        0xFD,                          /* 位置模式 */
        0x00,                          /* 方向，下面填 */
        (uint8_t)(LIFT_MOTOR_SPEED >> 8),
        (uint8_t)(LIFT_MOTOR_SPEED & 0xFFu),
        (uint8_t)LIFT_MOTOR_ACCEL,
        0x00, 0x00, 0x00, 0x00,
        0x00,                          /* byte[10]，下面填 */
        0x00,                          /* 立即执行 */
        0x6B
    };

    cmd[2]  = up ? 0x01u : 0x00u;
    cmd[10] = lr_rel ? 0x00u : 0x01u;

    cmd[6] = (uint8_t)(pulses >> 24);
    cmd[7] = (uint8_t)(pulses >> 16);
    cmd[8] = (uint8_t)(pulses >> 8);
    cmd[9] = (uint8_t)pulses;

    Serial_SendArray(cmd, 13);
    Delay_ms(5);
}

/* 读位置；读不到给 0 并返回 0 */
static uint8_t lr_read(int32_t *out)
{
    return lift_motor_read_pos(out);
}

/* 发一次 + 读回，打印"这次实际变化了多少" */
static void lr_once(uint8_t up, const char *tag)
{
    int32_t before = 0, after = 0;
    uint8_t okb, oka;

    okb = lr_read(&before);

    USART3_Printf("[LTEST] %s %u 脉冲（byte10=0x%02X, dir=%s）\r\n",
                  tag, (unsigned)lr_step,
                  (unsigned)(lr_rel ? 0x00u : 0x01u),
                  up ? "UP" : "DOWN");

    lr_send_raw(lr_step, up);
    Delay_ms(LR_READ_MS);

    oka = lr_read(&after);

    if(okb && oka){
        USART3_Printf("[LTEST]   ACT %d -> %d   本次实际变化 = %d\r\n",
                      (int)before, (int)after, (int)(after - before));
    }else{
        USART3_SendString("[LTEST]   ACT 读不到（0x36 没回应），这次只能靠眼睛看\r\n");
    }
}

/* 自动判定：5 上 + 5 下，每步 20 脉冲，打印结论 */
static void lr_auto_test(void)
{
    int32_t a0 = 0, a1 = 0, a2 = 0;
    uint8_t ok0, ok1, ok2;
    int32_t up_sum, dn_sum;
    uint8_t i;

    USART3_SendString("\r\n[LTEST] ===== 自动判定开始 =====");
    USART3_SendString("\r\n");

    ok0 = lr_read(&a0);
    if(!ok0){
        USART3_SendString("[LTEST] 读不到驱动器位置（0x36 没回应），无法自动判定\r\n");
        USART3_SendString("[LTEST] 请先确认地址 6 的通信，或改用 U/D 手动一步步看\r\n");
        return;
    }

    USART3_Printf("[LTEST] 起点 ACT = %d 脉冲\r\n", (int)a0);

    if(a0 > (int32_t)LR_SAFE_MAX){
        USART3_Printf("[LTEST] ★拒绝执行：当前位置 %d 太靠上。\r\n", (int)a0);
        USART3_SendString("[LTEST] 万一驱动器是『绝对』语义，发『位置 20』会直接掉到底。\r\n");
        USART3_SendString("[LTEST] 请先把机械臂放低（重新上电让它停在最低位），再按 PC0/T\r\n");
        return;
    }

    USART3_Printf("[LTEST] 连发 5 次向上 %u 脉冲 ...\r\n", (unsigned)lr_step);
    for(i = 0u; i < 5u; i++){
        lr_send_raw(lr_step, 1u);
        Delay_ms(LR_READ_MS);
    }
    ok1 = lr_read(&a1);
    up_sum = ok1 ? (a1 - a0) : 0;
    USART3_Printf("[LTEST] 5 次之后 ACT = %d   累计变化 = %d\r\n", (int)a1, (int)up_sum);

    USART3_Printf("[LTEST] 连发 5 次向下 %u 脉冲 ...\r\n", (unsigned)lr_step);
    for(i = 0u; i < 5u; i++){
        lr_send_raw(lr_step, 0u);
        Delay_ms(LR_READ_MS);
    }
    ok2 = lr_read(&a2);
    dn_sum = ok2 ? (a2 - a1) : 0;
    USART3_Printf("[LTEST] 5 次之后 ACT = %d   累计变化 = %d\r\n", (int)a2, (int)dn_sum);

    USART3_SendString("\r\n[LTEST] ---------- 结论 ----------\r\n");

    /* 判定阈值：相对模式 5×20 = 100，允许 ±40 误差（丢步/回差） */
    if((up_sum > 60) && (up_sum < 140) && (dn_sum < -60) && (dn_sum > -140)){
        USART3_SendString("[LTEST] ★ 驱动器按【相对脉冲】执行 ★\r\n");
        USART3_SendString("[LTEST]   证据：5×20 累计上 +100、下 -100，会累加。\r\n");
        USART3_SendString("[LTEST]   → 帧里的脉冲必须是『差值』。lift_motor.c 已按差值发，\r\n");
        USART3_SendString("[LTEST]     若下降仍不对，问题在别处（方向位/速度/机械）。\r\n");
    }else if((up_sum > -40) && (up_sum < 60) && (dn_sum > -60) && (dn_sum < 40)){
        USART3_SendString("[LTEST] ★ 驱动器按【绝对位置】执行 ★\r\n");
        USART3_SendString("[LTEST]   证据：每次都停在『位置 20』，不累加。\r\n");
        USART3_SendString("[LTEST]   → 那就必须填绝对目标，我的『差值』改法反而错了，\r\n");
        USART3_SendString("[LTEST]     把 lift_motor_move() 里的 delta 换回 abs_pulses 即可。\r\n");
    }else{
        USART3_SendString("[LTEST] 行为异常（既不像纯相对也不像纯绝对）。\r\n");
        USART3_SendString("[LTEST] 请把上面两个『累计变化』数值发我，按实际数字判断。\r\n");
        USART3_SendString("[LTEST] 可能原因：方向位被忽略但符号取自别处 / 丢步 / 驱动器限流。\r\n");
    }
}

/* 原始帧判定测试主循环 */
static void lift_raw_run(void)
{
    char     cmdline[24];
    uint16_t pc2_gap = 0u;
    uint16_t pc3_gap = 0u;
    int32_t  act = 0;
    uint8_t  have;

    grab_key_init();               /* PA5 */
    st_pc_init();                  /* PC0 / PC2 / PC3 */
    motor_enable_addr(LIFT_MOTOR_ADDR);

    OLED_Clear();
    OLED_ShowString(1, 1, "LIFT RAW TEST");
    OLED_ShowString(2, 1, "PC2=UP PC3=DN");
    OLED_ShowString(3, 1, "PC0=AUTO TEST");

    USART3_SendString("\r\n[LTEST] ===== 升降原始帧判定测试 =====\r\n");
    USART3_SendString("[LTEST] 每次只走 20 脉冲，总位移 ≤100，绝对安全\r\n");
    USART3_SendString("[LTEST] PC2=向上一次  PC3=向下一次  PC0=自动判定  PA5=急停\r\n");
    USART3_SendString("[LTEST] 上位机: U / D / T(自动) / S(读位置) / R(切byte10) / X\r\n");

    have = lr_read(&act);
    if(have){
        USART3_Printf("[LTEST] 上电 ACT = %d 脉冲\r\n", (int)act);
        OLED_ShowString(4, 1, "ACT:");
        OLED_ShowSignedNum(4, 5, act, 5);
    }else{
        USART3_SendString("[LTEST] 上电读不到位置（0x36 没回应）\r\n");
        OLED_ShowString(4, 1, "ACT: ----");
    }

    while(1)
    {
        if(pc2_gap > 0u) pc2_gap = (pc2_gap > 10u) ? (uint16_t)(pc2_gap - 10u) : 0u;
        if(pc3_gap > 0u) pc3_gap = (pc3_gap > 10u) ? (uint16_t)(pc3_gap - 10u) : 0u;

        /* ---------- 上位机命令 ---------- */
        if(USART3_ReadLine(cmdline, sizeof(cmdline))){
            char c = cmdline[0];
            uint32_t arg;
            uint16_t i;

            for(i = 0; cmdline[i] != '\0'; i++){ }
            while((i > 0u) && ((cmdline[i - 1u] == ' ') || (cmdline[i - 1u] == '\r'))) i--;
            cmdline[i] = '\0';

            if(cmdline[0] != '\0'){
                if((c >= 'a') && (c <= 'z')) c = (char)(c - 'a' + 'A');
                USART3_Printf("[LTEST] HOST> %s\r\n", cmdline);

                switch(c)
                {
                    case 'U': lr_once(1u, "UP");   break;
                    case 'D': lr_once(0u, "DOWN"); break;
                    case 'T': lr_auto_test();      break;
                    case 'R':
                        lr_rel = lr_rel ? 0u : 1u;
                        USART3_Printf("[LTEST] byte[10] 切成 0x%02X（%s）\r\n",
                                      (unsigned)(lr_rel ? 0x00u : 0x01u),
                                      lr_rel ? "相对" : "绝对");
                        break;
                    case 'N':
                        if(lt_parse_u32(&cmdline[1], &arg) && (arg > 0u) && (arg <= 500u)){
                            lr_step = (uint16_t)arg;
                            USART3_Printf("[LTEST] 单次脉冲 -> %u\r\n", (unsigned)lr_step);
                        }else{
                            USART3_Printf("[LTEST] 当前单次 %u 脉冲（N 后面跟 1~500）\r\n",
                                          (unsigned)lr_step);
                        }
                        break;
                    case 'S':
                        have = lr_read(&act);
                        USART3_Printf("[LTEST] 位置 = %d %s\r\n",
                                      (int)act, have ? "OK" : "(读不到)");
                        break;
                    case 'X':
                        lift_motor_stop();
                        USART3_SendString("[LTEST] 急停\r\n");
                        break;
                    default:
                        USART3_SendString("[LTEST] U / D / T / S / R / N<step> / X\r\n");
                        break;
                }

                have = lr_read(&act);
                OLED_ShowString(4, 1, "ACT:");
                if(have) OLED_ShowSignedNum(4, 5, act, 5);
                else     OLED_ShowString(4, 5, " ----");
            }
            continue;
        }

        /* ---------- PC2：向上一次 ---------- */
        if((pc2_gap == 0u) && st_pc_held(GPIO_Pin_2)){
            uint16_t hold;
            pc2_gap = 400u;
            lr_once(1u, "UP");
            have = lr_read(&act);
            OLED_ShowString(4, 1, "ACT:");
            if(have) OLED_ShowSignedNum(4, 5, act, 5);
            for(hold = 0u; st_pc_held(GPIO_Pin_2) && (hold < 200u); hold++) Delay_ms(10);
            continue;
        }

        /* ---------- PC3：向下一次 ---------- */
        if((pc3_gap == 0u) && st_pc_held(GPIO_Pin_3)){
            uint16_t hold;
            pc3_gap = 400u;
            lr_once(0u, "DOWN");
            have = lr_read(&act);
            OLED_ShowString(4, 1, "ACT:");
            if(have) OLED_ShowSignedNum(4, 5, act, 5);
            for(hold = 0u; st_pc_held(GPIO_Pin_3) && (hold < 200u); hold++) Delay_ms(10);
            continue;
        }

        /* ---------- PC0：自动判定 ---------- */
        if(st_pc_held(GPIO_Pin_0)){
            USART3_SendString("[LTEST] PC0 -> 自动判定\r\n");
            lr_auto_test();
            have = lr_read(&act);
            OLED_ShowString(4, 1, "ACT:");
            if(have) OLED_ShowSignedNum(4, 5, act, 5);
            while(st_pc_held(GPIO_Pin_0)) Delay_ms(10);
            continue;
        }

        /* ---------- PA5：急停 ---------- */
        if(grab_abort_hit()){
            vision_stop_all();
            lift_motor_stop();
            USART3_SendString("[LTEST] PA5 -> 急停\r\n");
            grab_wait_release(GPIOA, GPIO_Pin_5);
            Delay_ms(200);
            continue;
        }

        Delay_ms(10);
    }
}

/* ============================================================================
 * 三舵机标定（DEBUG_MODE_SERVO3 = 15）
 * ----------------------------------------------------------------------------
 * 现在一共三路舵机（都是 TIM3 完全重映射到 PC 口）：
 *
 *      SERVO_ROT  = PC9 / TIM3_CH4   旋转舵机   上电 30°
 *      SERVO_GRIP = PC8 / TIM3_CH3   夹爪舵机   上电 85°
 *      SERVO_LIFT = PC7 / TIM3_CH2   升降舵机   上电 90°
 *
 * 板载按键（一次按下只走一步，等松手）：
 *      PC0   切换当前要调的舵机：旋转 → 夹爪 → 升降 → 旋转 …
 *      PC2   当前舵机 角度 +step
 *      PC3   当前舵机 角度 −step
 *      PB1   切换步长 1 / 5 / 10 / 45°
 *      PA5   当前舵机 回上电初始角度
 *
 * 上位机命令（无线串口 USART3，115200 8N1）：
 *      S1 / S2 / S3   选舵机（1=旋转 PC9，2=夹爪 PC8，3=升降 PC7）
 *      +N / -N        当前舵机 加/减 N 度（省略 N 用当前步长）
 *      A<N>           当前舵机 走到绝对角度 N
 *      N<N>           设置步长
 *      Z              当前舵机 回初始角度
 *      Q              查询三路角度
 *      H              帮助
 *
 * 串口回：每次动作一行
 *      [SERVO3] LIFT 90 -> 95   pulse=1204us
 *      [SERVO3] STATUS sel=LIFT step=5  ROT=30 GRIP=85 LIFT=95
 *
 * 对应上位机：tools/servo_host.py（双击 tools/run_servo_host.bat）
 * ========================================================================== */
#define DEBUG_MODE_SERVO3 15

static const uint16_t k_sv3_steps[] = { 1u, 5u, 10u, 45u };
#define SV3_STEP_N  ((uint8_t)(sizeof(k_sv3_steps) / sizeof(k_sv3_steps[0])))

static uint8_t  sv3_sel  = (uint8_t)SERVO_ROT;   /* 当前调哪一路 */
static uint8_t  sv3_sidx = 1u;                   /* 步长下标（默认 5°） */
static uint16_t sv3_step = 5u;
static uint16_t sv3_ang[SERVO_COUNT];            /* 三路的逻辑角度 */

/* 舵机名（显示用，都是 ASCII） */
static char *sv3_name(uint8_t id)
{
    switch(id){
        case SERVO_ROT:  return (char *)"ROT ";
        case SERVO_GRIP: return (char *)"GRIP";
        case SERVO_LIFT: return (char *)"LIFT";
        default:         return (char *)"????";
    }
}

static char *sv3_pin(uint8_t id)
{
    switch(id){
        case SERVO_ROT:  return (char *)"PC9";
        case SERVO_GRIP: return (char *)"PC8";
        case SERVO_LIFT: return (char *)"PC7";
        default:         return (char *)"???";
    }
}

static uint16_t sv3_init_deg(uint8_t id)
{
    switch(id){
        case SERVO_ROT:  return (uint16_t)SERVO_ROT_INIT_DEG;
        case SERVO_GRIP: return (uint16_t)SERVO_GRIP_INIT_DEG;
        case SERVO_LIFT: return (uint16_t)SERVO_LIFT_INIT_DEG;
        default:         return 0u;
    }
}

/* 设某一一路角度并打一行串口 */
static void sv3_set(uint8_t id, uint16_t deg, const char *tag)
{
    uint16_t before;

    if(id >= (uint8_t)SERVO_COUNT) return;
    if(deg > SERVO_ANGLE_MAX) deg = (uint16_t)SERVO_ANGLE_MAX;

    before = sv3_ang[id];

    Servo_SetAngle((ServoId)id, deg);
    sv3_ang[id] = deg;

    USART3_Printf("[SERVO3] %s %s %u -> %u   pulse=%uus\r\n",
                  tag, sv3_name(id),
                  (unsigned)before, (unsigned)deg,
                  (unsigned)Servo_GetPulse((ServoId)id));
}

static void sv3_status(void)
{
    USART3_Printf("[SERVO3] STATUS sel=%s step=%u  ROT=%u GRIP=%u LIFT=%u\r\n",
                  sv3_name(sv3_sel), (unsigned)sv3_step,
                  (unsigned)sv3_ang[SERVO_ROT],
                  (unsigned)sv3_ang[SERVO_GRIP],
                  (unsigned)sv3_ang[SERVO_LIFT]);
}

static void sv3_show(void)
{
    OLED_ClearLine(1);
    OLED_ShowString(1, 1, "SERVO SEL:");
    OLED_ShowString(1, 11, sv3_name(sv3_sel));
    OLED_ShowString(1, 15, "  ");

    OLED_ShowString(2, 1, "ANG ");
    OLED_ShowNum(2, 5, (uint32_t)sv3_ang[sv3_sel], 3);
    OLED_ShowString(2, 9, " STEP ");
    OLED_ShowNum(2, 15, (uint32_t)sv3_step, 1);

    OLED_ShowString(3, 1, "R");
    OLED_ShowNum(3, 2, (uint32_t)sv3_ang[SERVO_ROT], 3);
    OLED_ShowString(3, 5, " G");
    OLED_ShowNum(3, 7, (uint32_t)sv3_ang[SERVO_GRIP], 3);
    OLED_ShowString(3, 10, " L");
    OLED_ShowNum(3, 12, (uint32_t)sv3_ang[SERVO_LIFT], 3);
}

/* 上位机命令 */
static void sv3_host_cmd(char *line)
{
    char c;
    uint32_t arg;
    uint16_t i;

    for(i = 0; line[i] != '\0'; i++){ }
    while((i > 0u) && ((line[i - 1u] == ' ') || (line[i - 1u] == '\r'))) i--;
    line[i] = '\0';
    if(line[0] == '\0') return;

    c = line[0];
    if((c >= 'a') && (c <= 'z')) c = (char)(c - 'a' + 'A');
    USART3_Printf("[SERVO3] HOST> %s\r\n", line);

    switch(c)
    {
        case 'S':                                   /* 选舵机 */
            if(lt_parse_u32(&line[1], &arg) && (arg >= 1u) && (arg <= (uint32_t)SERVO_COUNT)){
                sv3_sel = (uint8_t)(arg - 1u);
                USART3_Printf("[SERVO3] 选中 %s（%s），当前 %u°\r\n",
                              sv3_name(sv3_sel), sv3_pin(sv3_sel),
                              (unsigned)sv3_ang[sv3_sel]);
            }else{
                USART3_SendString("[SERVO3] 用法 S1(旋转) S2(夹爪) S3(升降)\r\n");
            }
            break;

        case '+':
        case '-': {
            uint16_t step = sv3_step;
            uint16_t deg;

            if(lt_parse_u32(&line[1], &arg) && (arg > 0u) && (arg <= 270u)) step = (uint16_t)arg;

            deg = sv3_ang[sv3_sel];
            if(c == '+'){
                deg = ((uint16_t)(SERVO_ANGLE_MAX - deg) < step)
                      ? (uint16_t)SERVO_ANGLE_MAX : (uint16_t)(deg + step);
            }else{
                deg = (deg < step) ? 0u : (uint16_t)(deg - step);
            }
            sv3_set(sv3_sel, deg, (c == '+') ? "UP  " : "DOWN");
            break;
        }

        case 'A':                                   /* 绝对角度 */
            if(!lt_parse_u32(&line[1], &arg)){
                USART3_SendString("[SERVO3] 用法 A135（当前舵机走到 135°）\r\n");
                break;
            }
            sv3_set(sv3_sel, (uint16_t)((arg > SERVO_ANGLE_MAX) ? SERVO_ANGLE_MAX : arg), "ABS ");
            break;

        case 'N':                                   /* 设步长 */
            if(lt_parse_u32(&line[1], &arg) && (arg > 0u) && (arg <= 90u)){
                sv3_step = (uint16_t)arg;
                USART3_Printf("[SERVO3] 步长 -> %u°\r\n", (unsigned)sv3_step);
            }else{
                USART3_Printf("[SERVO3] 当前步长 %u°（N 后面跟 1~90）\r\n",
                              (unsigned)sv3_step);
            }
            break;

        case 'Z':                                   /* 回初始角度 */
            sv3_set(sv3_sel, sv3_init_deg(sv3_sel), "HOME");
            break;

        case 'Q':
            sv3_status();
            break;

        case 'H':
        default:
            USART3_SendString("[SERVO3] S1/S2/S3 选  +N/-N 加减  A<N> 绝对  N<N> 步长  Z 回初始  Q 查询\r\n");
            break;
    }

    sv3_show();
}

/* 三舵机标定主循环：不返回 */
static void servo3_run(void)
{
    char     cmdline[24];
    uint16_t pc2_gap = 0u;
    uint16_t pc3_gap = 0u;
    uint8_t  i;

    st_pc_init();                  /* PC0 / PC2 / PC3 */
    grab_key_init();               /* PA5 */

    /* 三路都摆到上电初始角度 */
    for(i = 0u; i < (uint8_t)SERVO_COUNT; i++){
        sv3_ang[i] = sv3_init_deg(i);
        Servo_SetAngle((ServoId)i, sv3_ang[i]);
        Delay_ms(300);
    }
    sv3_step = k_sv3_steps[sv3_sidx];

    OLED_Clear();
    OLED_ShowString(4, 1, "PC0 SW PC2+ PC3-");
    sv3_show();

    USART3_SendString("\r\n[SERVO3] ===== 三舵机标定 =====\r\n");
    USART3_Printf("[SERVO3] 旋转 PC9=%u°  夹爪 PC8=%u°  升降 PC7=%u°\r\n",
                  (unsigned)SERVO_ROT_INIT_DEG,
                  (unsigned)SERVO_GRIP_INIT_DEG,
                  (unsigned)SERVO_LIFT_INIT_DEG);
    USART3_SendString("[SERVO3] 板载: PC0=换舵机 PC2=+ PC3=- PB1=换步长 PA5=回初始\r\n");
    USART3_SendString("[SERVO3] 上位机: S1/S2/S3 选  +N/-N  A<N>  N<N>  Z  Q  H\r\n");
    sv3_status();

    while(1)
    {
        if(pc2_gap > 0u) pc2_gap = (pc2_gap > 10u) ? (uint16_t)(pc2_gap - 10u) : 0u;
        if(pc3_gap > 0u) pc3_gap = (pc3_gap > 10u) ? (uint16_t)(pc3_gap - 10u) : 0u;

        /* ---------- 上位机命令 ---------- */
        if(USART3_ReadLine(cmdline, sizeof(cmdline))){
            sv3_host_cmd(cmdline);
            pc2_gap = 60u;
            pc3_gap = 60u;
            continue;
        }

        /* ---------- PC0：切换要调的舵机 ---------- */
        if(st_pc_held(GPIO_Pin_0)){
            sv3_sel = (uint8_t)((sv3_sel + 1u) % (uint8_t)SERVO_COUNT);
            USART3_Printf("[SERVO3] 换舵机 -> %s（%s）当前 %u°\r\n",
                          sv3_name(sv3_sel), sv3_pin(sv3_sel),
                          (unsigned)sv3_ang[sv3_sel]);
            sv3_show();
            while(st_pc_held(GPIO_Pin_0)) Delay_ms(10);
            continue;
        }

        /* ---------- PC2：角度 + ---------- */
        if((pc2_gap == 0u) && st_pc_held(GPIO_Pin_2)){
            uint16_t hold;
            uint16_t deg = sv3_ang[sv3_sel];
            pc2_gap = 200u;
            deg = ((uint16_t)(SERVO_ANGLE_MAX - deg) < sv3_step)
                  ? (uint16_t)SERVO_ANGLE_MAX : (uint16_t)(deg + sv3_step);
            sv3_set(sv3_sel, deg, "PC2+");
            sv3_show();
            for(hold = 0u; st_pc_held(GPIO_Pin_2) && (hold < 100u); hold++) Delay_ms(10);
            continue;
        }

        /* ---------- PC3：角度 − ---------- */
        if((pc3_gap == 0u) && st_pc_held(GPIO_Pin_3)){
            uint16_t hold;
            uint16_t deg = sv3_ang[sv3_sel];
            pc3_gap = 200u;
            deg = (deg < sv3_step) ? 0u : (uint16_t)(deg - sv3_step);
            sv3_set(sv3_sel, deg, "PC3-");
            sv3_show();
            for(hold = 0u; st_pc_held(GPIO_Pin_3) && (hold < 100u); hold++) Delay_ms(10);
            continue;
        }

        /* ---------- PB1：换步长 ---------- */
        if(grab_step_hit()){
            sv3_sidx = (uint8_t)((sv3_sidx + 1u) % SV3_STEP_N);
            sv3_step = k_sv3_steps[sv3_sidx];
            USART3_Printf("[SERVO3] 步长 -> %u°\r\n", (unsigned)sv3_step);
            sv3_show();
            continue;
        }

        /* ---------- PA5：当前舵机回初始角度 ---------- */
        if(grab_abort_hit()){
            sv3_set(sv3_sel, sv3_init_deg(sv3_sel), "HOME");
            sv3_show();
            grab_wait_release(GPIOA, GPIO_Pin_5);
            Delay_ms(200);
            continue;
        }

        Delay_ms(10);
    }
}

/* ============================================================================
 * 整臂联动标定（DEBUG_MODE_ARM_ALL = 16）—— 升降步进 + 三个舵机，一个界面全调
 * ----------------------------------------------------------------------------
 * 四个可调对象：
 *     1  升降步进电机  总线地址 6，脉冲 0~8000（上电 0 = 最低位）
 *     2  旋转舵机      PC9 / TIM3_CH4  上电 30°
 *     3  夹爪舵机      PC8 / TIM3_CH3  上电 85°
 *     4  升降舵机      PC7 / TIM3_CH2  上电 90°
 *
 * 板载按键（一次按下走一步，等松手）：
 *     PC0   切换对象：升降步进 → 旋转 → 夹爪 → 升降舵机 → …
 *     PC2   当前对象 正方向（步进上升 / 舵机角度 +）
 *     PC3   当前对象 反方向
 *     PB1   换步长（步进 10/50/100/500/1000；舵机 1/5/10/45）
 *     PA5   当前对象 回初始（步进回 0；舵机回上电角度）
 *
 * 上位机命令（无线串口 USART3，115200 8N1）：
 *     U<n> / D<n>     升降步进 上升/下降 n 脉冲
 *     G<n>            升降步进 走到绝对位置 n
 *     L               升降步进 回零（绝对 0）
 *     S<1..3>         选舵机（1旋转 2夹爪 3升降）
 *     +<n> / -<n>     选中舵机 角度 加/减 n 度
 *     A<n>            选中舵机 走到绝对角度 n
 *     R               选中舵机 回上电角度
 *     P<s>,<r>,<g>,<l>  一次摆到完整姿态：步进 s 脉冲 + 旋转 r° + 夹爪 g° + 升降舵机 l°
 *     N<n>            设舵机步长
 *     Q               查询状态
 *     X               升降急停
 *     H               帮助
 *
 * 串口回：
 *     [ARM] STATUS MOT=200 ACT=200 SEL=LIFT STEP=10 ROT=30 GRIP=85 LIFT=90
 *     [ARM] UP   L 0 -> 100 (expect 460 ms)
 *     [ARM]   ACT=100  d=0
 *     [ARM] GRIP 85 -> 90   pulse=1167us
 *
 * 对应上位机：tools/arm_calib_host.py（双击 tools/run_arm_calib.bat）
 * ========================================================================== */
#define DEBUG_MODE_ARM_ALL 16

/* 四个对象的下标 */
#define AA_OBJ_MOTOR   0u
#define AA_OBJ_ROT     1u
#define AA_OBJ_GRIP    2u
#define AA_OBJ_LIFT    3u
#define AA_OBJ_COUNT   4u

static const uint16_t k_aa_motor_steps[] = { 10u, 50u, 100u, 500u, 1000u };
#define AA_MOTOR_STEP_N ((uint8_t)(sizeof(k_aa_motor_steps) / sizeof(k_aa_motor_steps[0])))

static uint8_t  aa_obj     = AA_OBJ_MOTOR;   /* 当前调哪个对象 */
static uint8_t  aa_msidx   = 2u;             /* 步进步长下标（默认 100） */
static uint16_t aa_mstep   = 100u;           /* 步进当前步长 */
static uint16_t aa_sstep   = 5u;             /* 舵机当前步长 */
static uint8_t  aa_ssidx   = 1u;             /* 舵机步长下标（默认 5°） */

/* 对象名（ASCII，给屏和串口用） */
static char *aa_obj_name(uint8_t obj)
{
    switch(obj){
        case AA_OBJ_MOTOR: return (char *)"MOTOR";
        case AA_OBJ_ROT:   return (char *)"ROT";
        case AA_OBJ_GRIP:  return (char *)"GRIP";
        case AA_OBJ_LIFT:  return (char *)"LIFT";
        default:           return (char *)"????";
    }
}

/* 对象 → ServoId（非舵机返回 0xFF） */
static uint8_t aa_obj_servo(uint8_t obj)
{
    switch(obj){
        case AA_OBJ_ROT:  return (uint8_t)SERVO_ROT;
        case AA_OBJ_GRIP: return (uint8_t)SERVO_GRIP;
        case AA_OBJ_LIFT: return (uint8_t)SERVO_LIFT;
        default:          return 0xFFu;
    }
}

/* 升降步进走到目标位置（绝对位置帧）
 * ★ 长距离下降走两段：先下降速度到 目标+SOFT_ZONE，再爬行速度收最后一段。
 *   底部有减速余量，不容易硬顶限位。走完读 ACT 校验并报告。 */
static void aa_motor_to(uint32_t target, const char *tag)
{
    uint32_t cur;
    uint16_t ms;
    int32_t  act = 0;
    uint8_t  ok;

    lift_motor_sync();                     /* 先用驱动器实际位置校准 */
    cur = g_lift_target;

    if(target > LIFT_MOTOR_MAX) target = LIFT_MOTOR_MAX;

    if(target == cur){
        USART3_Printf("[ARM] %s 已经在 %u\r\n", tag, (unsigned)cur);
        return;
    }

    USART3_Printf("[ARM] %s  MOT %u -> %u\r\n", tag, (unsigned)cur, (unsigned)target);

    /* ★ 长距离下降先提醒一句：这一下去多少脉冲、按什么速度走，
     *   万一零点设错，看到这行就能立刻按 X 急停。 */
    if(target < cur){
        USART3_Printf("[ARM]   ↓ 要下降 %u 脉冲（约 %u ms，%u RPM）——不对就按 X\r\n",
                      (unsigned)(cur - target),
                      (unsigned)lift_motor_ms_for(cur, target, (uint8_t)LIFT_MOTOR_SPEED_DOWN),
                      (unsigned)LIFT_MOTOR_SPEED_DOWN);
    }

#if CP_LIFT_SOFT_LANDING
    {
        uint32_t mid;

        if(lift_motor_soft_split(cur, target, &mid))
        {
            ms = lift_motor_ms_for(cur, mid, (uint8_t)LIFT_MOTOR_SPEED_DOWN);
            USART3_Printf("[ARM]   段1 %u -> %u (%uRPM 等%ums)\r\n",
                          (unsigned)cur, (unsigned)mid,
                          (unsigned)LIFT_MOTOR_SPEED_DOWN, (unsigned)ms);
            lift_motor_move_at(mid, (uint8_t)LIFT_MOTOR_SPEED_DOWN);
            if(!cp_lift_wait_guarded(ms)){ lift_motor_stop(); USART3_SendString("[ARM] 中止\r\n"); return; }
            Delay_ms(150);

            ms = lift_motor_ms_for(mid, target, (uint8_t)LIFT_MOTOR_SPEED_CREEP);
            USART3_Printf("[ARM]   段2 %u -> %u (%uRPM 等%ums 爬行)\r\n",
                          (unsigned)mid, (unsigned)target,
                          (unsigned)LIFT_MOTOR_SPEED_CREEP, (unsigned)ms);
            lift_motor_move_at(target, (uint8_t)LIFT_MOTOR_SPEED_CREEP);
            if(!cp_lift_wait_guarded(ms)){ lift_motor_stop(); USART3_SendString("[ARM] 中止\r\n"); return; }
        }
        else
        {
            ms = lift_motor_move_ms(cur, target);
            USART3_Printf("[ARM]   一段 %u -> %u (等%ums)\r\n",
                          (unsigned)cur, (unsigned)target, (unsigned)ms);
            lift_motor_move(target, 0u);
            if(!cp_lift_wait_guarded(ms)){ lift_motor_stop(); USART3_SendString("[ARM] 中止\r\n"); return; }
        }
    }
#else
    /* 一段走到底（CP_LIFT_SOFT_LANDING = 0） */
    ms = lift_motor_move_ms(cur, target);
    USART3_Printf("[ARM]   一段 %u -> %u (等%ums)\r\n",
                  (unsigned)cur, (unsigned)target, (unsigned)ms);
    lift_motor_move(target, 0u);
    if(!cp_lift_wait_guarded(ms)){ lift_motor_stop(); USART3_SendString("[ARM] 中止\r\n"); return; }
#endif

    Delay_ms(250);                         /* 等彻底停稳再读 */
    ok = lift_motor_read_pos(&act);
    if(ok){
        USART3_Printf("[ARM]   ACT=%d  d=%d\r\n", (int)act, (int)(act - (int32_t)target));
        if(act < 0){
            USART3_SendString("[ARM]   !! ACT 负数：驱动器认为自己在零点下面（降过头了）\r\n");
            USART3_SendString("[ARM]   !! 对策：LIFT_MOTOR_SPEED_DOWN 再降低，或按 W 重设零点\r\n");
        }
    }else{
        USART3_SendString("[ARM]   ACT 读不到\r\n");
    }
}

static void aa_status(void)
{
    int32_t act = 0;
    uint8_t ok = lift_motor_read_pos(&act);

    USART3_Printf("[ARM] STATUS MOT=%u ACT=%d SEL=%s STEP=%u ROT=%u GRIP=%u LIFT=%u\r\n",
                  (unsigned)g_lift_target,
                  ok ? (int)act : -1,
                  aa_obj_name(aa_obj),
                  (unsigned)((aa_obj == AA_OBJ_MOTOR) ? aa_mstep : aa_sstep),
                  (unsigned)sv3_ang[SERVO_ROT],
                  (unsigned)sv3_ang[SERVO_GRIP],
                  (unsigned)sv3_ang[SERVO_LIFT]);
}

static void aa_show(void)
{
    OLED_ClearLine(1);
    OLED_ShowString(1, 1, "ARM L:");
    OLED_ShowNum(1, 7, (uint32_t)g_lift_target, 4);
    OLED_ShowString(1, 12, "      ");

    OLED_ClearLine(2);
    OLED_ShowString(2, 1, "SEL:");
    OLED_ShowString(2, 5, aa_obj_name(aa_obj));
    OLED_ShowString(2, 10, " ST");
    OLED_ShowNum(2, 13, (uint32_t)((aa_obj == AA_OBJ_MOTOR) ? aa_mstep : aa_sstep), 3);
    OLED_ShowString(2, 16, " ");

    OLED_ShowString(3, 1, "R");
    OLED_ShowNum(3, 2, (uint32_t)sv3_ang[SERVO_ROT], 3);
    OLED_ShowString(3, 5, " G");
    OLED_ShowNum(3, 7, (uint32_t)sv3_ang[SERVO_GRIP], 3);
    OLED_ShowString(3, 10, " L");
    OLED_ShowNum(3, 12, (uint32_t)sv3_ang[SERVO_LIFT], 3);
}

/* 当前对象 正/反 走一步 */
static void aa_step(int8_t dir)
{
    uint8_t sid;

    if(aa_obj == AA_OBJ_MOTOR){
        uint32_t cur = g_lift_target;
        if(dir > 0){
            uint32_t t = cur + aa_mstep;
            if(t > LIFT_MOTOR_MAX) t = LIFT_MOTOR_MAX;
            aa_motor_to(t, "UP  ");
        }else{
            uint32_t t = (cur > aa_mstep) ? (cur - aa_mstep) : 0u;
            aa_motor_to(t, "DOWN");
        }
        return;
    }

    sid = aa_obj_servo(aa_obj);
    if(sid == 0xFFu) return;

    {
        uint16_t deg = sv3_ang[sid];
        if(dir > 0){
            deg = ((uint16_t)(SERVO_ANGLE_MAX - deg) < aa_sstep)
                  ? (uint16_t)SERVO_ANGLE_MAX : (uint16_t)(deg + aa_sstep);
        }else{
            deg = (deg < aa_sstep) ? 0u : (uint16_t)(deg - aa_sstep);
        }
        sv3_set(sid, deg, (dir > 0) ? "PC2+" : "PC3-");
    }
}

/* 当前对象 回初始 */
static void aa_home(void)
{
    if(aa_obj == AA_OBJ_MOTOR){
        aa_motor_to(LIFT_MOTOR_HOME, "ZERO");
    }else{
        uint8_t sid = aa_obj_servo(aa_obj);
        if(sid != 0xFFu) sv3_set(sid, sv3_init_deg(sid), "HOME");
    }
}

/* 上位机命令 */
static void aa_host_cmd(char *line)
{
    char c;
    uint32_t arg;
    uint16_t i;

    for(i = 0; line[i] != '\0'; i++){ }
    while((i > 0u) && ((line[i - 1u] == ' ') || (line[i - 1u] == '\r'))) i--;
    line[i] = '\0';
    if(line[0] == '\0') return;

    c = line[0];
    if((c >= 'a') && (c <= 'z')) c = (char)(c - 'a' + 'A');
    USART3_Printf("[ARM] HOST> %s\r\n", line);

    switch(c)
    {
        case 'U':
            arg = aa_mstep;
            (void)lt_parse_u32(&line[1], &arg);
            if(arg == 0u) arg = aa_mstep;
            if(arg > 5000u) arg = 5000u;
            {
                uint32_t t = g_lift_target + arg;
                if(t > LIFT_MOTOR_MAX) t = LIFT_MOTOR_MAX;
                aa_motor_to(t, "UP  ");
            }
            break;

        case 'D':
            arg = aa_mstep;
            (void)lt_parse_u32(&line[1], &arg);
            if(arg == 0u) arg = aa_mstep;
            if(arg > 5000u) arg = 5000u;
            aa_motor_to((g_lift_target > arg) ? (g_lift_target - arg) : 0u, "DOWN");
            break;

        case 'G':
            if(!lt_parse_u32(&line[1], &arg)){
                USART3_SendString("[ARM] 用法 G4000（升降步进走到 4000）\r\n");
                break;
            }
            aa_motor_to(arg, "GO  ");
            break;

        case 'L':
            aa_motor_to(LIFT_MOTOR_HOME, "ZERO");
            break;

        case 'S':
            if(lt_parse_u32(&line[1], &arg) && (arg >= 1u) && (arg <= 3u)){
                sv3_sel = (uint8_t)(arg - 1u);
                USART3_Printf("[ARM] 选中舵机 %s（%s），当前 %u°\r\n",
                              sv3_name(sv3_sel), sv3_pin(sv3_sel),
                              (unsigned)sv3_ang[sv3_sel]);
            }else{
                USART3_SendString("[ARM] 用法 S1(旋转) S2(夹爪) S3(升降舵机)\r\n");
            }
            break;

        case '+':
        case '-': {
            uint16_t st = aa_sstep;
            uint16_t deg;
            if(lt_parse_u32(&line[1], &arg) && (arg > 0u) && (arg <= 270u)) st = (uint16_t)arg;
            deg = sv3_ang[sv3_sel];
            if(c == '+'){
                deg = ((uint16_t)(SERVO_ANGLE_MAX - deg) < st)
                      ? (uint16_t)SERVO_ANGLE_MAX : (uint16_t)(deg + st);
            }else{
                deg = (deg < st) ? 0u : (uint16_t)(deg - st);
            }
            sv3_set(sv3_sel, deg, (c == '+') ? "UP  " : "DOWN");
            break;
        }

        case 'A': {
            uint8_t sid = aa_obj_servo(aa_obj);
            if(sid == 0xFFu) sid = sv3_sel;
            if(!lt_parse_u32(&line[1], &arg)){
                USART3_SendString("[ARM] 用法 A135（当前舵机走到 135°）\r\n");
                break;
            }
            sv3_set(sid, (uint16_t)((arg > SERVO_ANGLE_MAX) ? SERVO_ANGLE_MAX : arg), "ABS ");
            break;
        }

        case 'R': {
            uint8_t sid = aa_obj_servo(aa_obj);
            if(sid == 0xFFu) sid = sv3_sel;
            sv3_set(sid, sv3_init_deg(sid), "HOME");
            break;
        }

        case 'P': {
            /* P<s>,<r>,<g>,<l> */
            uint32_t v[4];
            uint8_t  n = 0u;
            const char *p = &line[1];

            while((n < 4u) && (*p != '\0')){
                uint32_t x = 0u;
                if(!lt_parse_u32(p, &x)) break;
                v[n++] = x;
                while((*p >= '0') && (*p <= '9')) p++;
                while((*p == ',') || (*p == ' ')) p++;
            }

            if(n != 4u){
                USART3_SendString("[ARM] 用法 P4000,30,85,90（步进,旋转,夹爪,升降舵机）\r\n");
                break;
            }

            USART3_Printf("[ARM] 摆姿态 MOT=%u ROT=%u GRIP=%u LIFT=%u\r\n",
                          (unsigned)v[0], (unsigned)v[1], (unsigned)v[2], (unsigned)v[3]);

            aa_motor_to(v[0], "POSE");
            sv3_set((uint8_t)SERVO_ROT,  (uint16_t)((v[1] > SERVO_ANGLE_MAX) ? SERVO_ANGLE_MAX : v[1]), "POSE");
            Delay_ms(300);
            sv3_set((uint8_t)SERVO_GRIP, (uint16_t)((v[2] > SERVO_ANGLE_MAX) ? SERVO_ANGLE_MAX : v[2]), "POSE");
            Delay_ms(300);
            sv3_set((uint8_t)SERVO_LIFT, (uint16_t)((v[3] > SERVO_ANGLE_MAX) ? SERVO_ANGLE_MAX : v[3]), "POSE");
            break;
        }

        case 'N':
            if(lt_parse_u32(&line[1], &arg) && (arg > 0u) && (arg <= 90u)){
                aa_sstep = (uint16_t)arg;
                USART3_Printf("[ARM] 舵机步长 -> %u°\r\n", (unsigned)aa_sstep);
            }else{
                USART3_Printf("[ARM] 舵机步长 %u°，步进步长 %u\r\n",
                              (unsigned)aa_sstep, (unsigned)aa_mstep);
            }
            break;

        case 'Q':
            aa_status();
            break;

        case 'X':
            lift_motor_stop();
            USART3_SendString("[ARM] 升降急停\r\n");
            break;

        case 'W':
            /* 把当前位置设为驱动器零点。机械臂先手动降到最低位（或停在限位上）再按，
             * 之后"0"就等于机械底，abs 位置不会再整体偏移。 */
            lift_motor_set_zero();
            USART3_SendString("[ARM] 已把当前位置设为驱动器零点（0x93 0x88）\r\n");
            USART3_SendString("[ARM] 现在 0 = 机械当前位，接下来 G<pos> 都从这里算\r\n");
            aa_status();
            break;

        case 'H':
        default:
            USART3_SendString("[ARM] U<n> D<n> G<n> L | S1/S2/S3 +</-> A<n> R | P<s>,<r>,<g>,<l> | N Q X W\r\n");
            USART3_SendString("[ARM]   W = 把当前位置设为驱动器零点（机械臂在最低位时按）\r\n");
            break;
    }

    aa_show();
}

/* 整臂标定主循环：不返回 */
static void arm_all_run(void)
{
    char     cmdline[32];
    uint16_t pc2_gap = 0u;
    uint16_t pc3_gap = 0u;
    uint8_t  i;

    st_pc_init();                  /* PC0 / PC2 / PC3 */
    grab_key_init();               /* PB1 / PA5 */
    motor_enable_addr(LIFT_MOTOR_ADDR);

    /* 三路舵机先摆到上电初始角度 */
    for(i = 0u; i < (uint8_t)SERVO_COUNT; i++){
        sv3_ang[i] = sv3_init_deg(i);
        Servo_SetAngle((ServoId)i, sv3_ang[i]);
        Delay_ms(300);
    }
    aa_mstep = k_aa_motor_steps[aa_msidx];
    aa_sstep = k_sv3_steps[aa_ssidx];

    OLED_Clear();
    OLED_ShowString(4, 1, "PC0 SW PC2+ PC3-");
    aa_show();

    USART3_SendString("\r\n[ARM] ===== 整臂联动标定（升降步进 + 三舵机）=====\r\n");
    USART3_Printf("[ARM] 升降步进 addr=%u  上限 %u 脉冲\r\n",
                  (unsigned)LIFT_MOTOR_ADDR, (unsigned)LIFT_MOTOR_MAX);
    USART3_Printf("[ARM] 舵机 PC9=旋转 %u°  PC8=夹爪 %u°  PC7=升降 %u°\r\n",
                  (unsigned)SERVO_ROT_INIT_DEG,
                  (unsigned)SERVO_GRIP_INIT_DEG,
                  (unsigned)SERVO_LIFT_INIT_DEG);
    USART3_SendString("[ARM] 板载: PC0=换对象 PC2=+ PC3=- PB1=换步长 PA5=回初始\r\n");
    USART3_SendString("[ARM] 上位机: U D G L | S1 S2 S3 +/- A R | P<s>,<r>,<g>,<l> | N Q X W\r\n");

    /* ★ 上电检查：驱动器自报的位置如果不是 0，说明它的"0"不是机械最低位
     *   （Emm42 会记住上次断电时的位置，或者上次是停在半路/被顶住时设的零点）。
     *   这时候直接发 G<小数字>，是从**驱动器自己的坐标**往下走那么远，
     *   很容易一路顶到机械底 —— 现场那次"输个小脉冲却降到底堵转"就是这么来的。 */
    {
        int32_t act0 = 0;
        if(lift_motor_read_pos(&act0)){
            if((act0 > 200) || (act0 < -200)){
                USART3_Printf("[ARM] ⚠ 驱动器位置 = %d（不是 0）\r\n", (int)act0);
                USART3_SendString("[ARM] ⚠ 现在 G<数字> 是相对**驱动器坐标**的，不是相对机械底！\r\n");
                USART3_SendString("[ARM] ⚠ 正确做法：先用 ↓（D 命令）一点点降到机械低位，\r\n");
                USART3_SendString("[ARM] ⚠ 再发 W 把那里设成 0。之后 G 的数字才有意义。\r\n");
            }else{
                USART3_SendString("[ARM] 位置≈0：G 的数字可以直接当【离机械底多少脉冲】用\r\n");
            }
        }else{
            USART3_SendString("[ARM] ⚠ 读不到驱动器位置（0x36 没回包）——绝对位置不可靠，先用 ↓/↑ 手动试\r\n");
        }
    }

    aa_status();

    while(1)
    {
        if(pc2_gap > 0u) pc2_gap = (pc2_gap > 10u) ? (uint16_t)(pc2_gap - 10u) : 0u;
        if(pc3_gap > 0u) pc3_gap = (pc3_gap > 10u) ? (uint16_t)(pc3_gap - 10u) : 0u;

        /* ---------- 上位机 ---------- */
        if(USART3_ReadLine(cmdline, sizeof(cmdline))){
            aa_host_cmd(cmdline);
            pc2_gap = 60u;
            pc3_gap = 60u;
            continue;
        }

        /* ---------- PC0：换对象 ---------- */
        if(st_pc_held(GPIO_Pin_0)){
            aa_obj = (uint8_t)((aa_obj + 1u) % AA_OBJ_COUNT);
            USART3_Printf("[ARM] 换对象 -> %s\r\n", aa_obj_name(aa_obj));
            aa_show();
            while(st_pc_held(GPIO_Pin_0)) Delay_ms(10);
            continue;
        }

        /* ---------- PC2：正方向 ---------- */
        if((pc2_gap == 0u) && st_pc_held(GPIO_Pin_2)){
            uint16_t hold;
            pc2_gap = 200u;
            aa_step(1);
            aa_show();
            for(hold = 0u; st_pc_held(GPIO_Pin_2) && (hold < 100u); hold++) Delay_ms(10);
            continue;
        }

        /* ---------- PC3：反方向 ---------- */
        if((pc3_gap == 0u) && st_pc_held(GPIO_Pin_3)){
            uint16_t hold;
            pc3_gap = 200u;
            aa_step(-1);
            aa_show();
            for(hold = 0u; st_pc_held(GPIO_Pin_3) && (hold < 100u); hold++) Delay_ms(10);
            continue;
        }

        /* ---------- PB1：换步长 ---------- */
        if(grab_step_hit()){
            if(aa_obj == AA_OBJ_MOTOR){
                aa_msidx = (uint8_t)((aa_msidx + 1u) % AA_MOTOR_STEP_N);
                aa_mstep = k_aa_motor_steps[aa_msidx];
                USART3_Printf("[ARM] 步进步长 -> %u 脉冲\r\n", (unsigned)aa_mstep);
            }else{
                aa_ssidx = (uint8_t)((aa_ssidx + 1u) % SV3_STEP_N);
                aa_sstep = k_sv3_steps[aa_ssidx];
                USART3_Printf("[ARM] 舵机步长 -> %u°\r\n", (unsigned)aa_sstep);
            }
            aa_show();
            continue;
        }

        /* ---------- PA5：回初始 ---------- */
        if(grab_abort_hit()){
            aa_home();
            aa_show();
            grab_wait_release(GPIOA, GPIO_Pin_5);
            Delay_ms(200);
            continue;
        }

        Delay_ms(10);
    }
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
    Servo_Init();             /* PC9(TIM3_CH4)=旋转 / PC8(TIM3_CH3)=夹爪，上电输出 30°/85° */
    OLED_Init();              /* 硬件 I2C1：PB6=SCL / PB7=SDA；探测不到就不画 */
    boot_oled_indicator(OLED_IsReady());   /* 闪 1 下 = OLED 应答，3 下 = 没应答 */
    Key_Init();               /* 按键 PB1 / PA7 / PA5（下拉输入，按下为高） */
    Serial_Init();            /* USART1：电机总线（也是串口烧录口） */
    Gyro_Init();
    USART3_Init(115200);      /* 无线调试：PB10=TX / PB11=RX → 无线串口模块（上位机） */
    USART2_Init(115200);      /* 陀螺仪 JY901：PA2=TX / PA3=RX */
    UART4_Init(115200);       /* ★ K230：PC10=TX / PC11=RX（UART4，需 STM32F10X_HD） */
	
	
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
#elif DEBUG_MODE == DEBUG_MODE_UART_TEST
	uart_test_run();            /* 串口只发测试：验证某一串口能不能发出去（选端口见 UART_TEST_PORT） */
#elif DEBUG_MODE == DEBUG_MODE_GRAB_ALIGN
	grab_align_run();           /* ★ PC2：读 K230 坐标 → 对准 → 夹一次 → 归位 */
#elif DEBUG_MODE == DEBUG_MODE_SERVO_TEST
	servo_test_run();           /* ★ 舵机单独测试：PC2/PC3 加减角度，PA5 回初始，PA7 自动扫描 */
#elif DEBUG_MODE == DEBUG_MODE_CAR_PUT
	car_put_run();              /* ★ 车上取料→放地上：PC3 一按一个动作，PA5 中止 */
#elif DEBUG_MODE == DEBUG_MODE_LIFT_TEST
	lift_test_run();            /* ★ 升降电机高度调试：PB1/PA7 升降，PC2 步长，PC3 回零 */
#elif DEBUG_MODE == DEBUG_MODE_LIFT_RAW
	lift_raw_run();             /* ★ 升降原始帧判定：5×20 累计判定相对/绝对 */
#elif DEBUG_MODE == DEBUG_MODE_SERVO3
	servo3_run();               /* ★ 三舵机标定：PC0 换对象，PC2/PC3 正反方向 */
#elif DEBUG_MODE == DEBUG_MODE_ARM_ALL
	arm_all_run();              /* ★ 整臂联动标定：升降步进 + 三舵机一个界面 */
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
