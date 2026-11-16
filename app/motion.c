#include "stm32f10x.h"
#include "motion.h"
#include "motor.h"
#include "Gyro.h"
#include "Delay.h"
#include "Serial.h"
#include "OLED.h"
#include "path_planner.h"
#include "path_config.h"

/* ============================================================================
 * 运动控制 —— 从 main.c 整块搬过来，逻辑未改
 * ========================================================================== */

/* 车头当前朝向（1=北 2=西 3=南 4=东）。原来在 main.c，跟着 translation 搬过来。 */
u8 dir = PATH_INITIAL_DIR;

/* ===== moved from main.c (logic unchanged): translation ===== */
u8 translation(u16 x0, u16 y0, u16 x1, u16 y1)
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

/* ===== moved from main.c (logic unchanged): wait_motors ===== */
/* 等四个电机到位（有界，避免驱动器掉线时整条路线卡死） */
u8 wait_motors(void)
{
    return judge_wait(MOVE_TIMEOUT_ROUNDS);
}

/* ===== moved from main.c (logic unchanged): gyro_turn_pos_test ===== */
/* ===== 下面是旧版实现，保留作对照；实际使用的是上面的 gyro_turn_by ===== */
/* 陀螺仪比例转向：先执行目标增量，再按剩余误差的一半逐次修正。
 * target_delta 使用相对起始角度，单位 0.01 度；所有修正均有次数和角度上限。 */
u8 gyro_turn_pos_test(int32_t target_delta)
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

/* ===== moved from main.c (logic unchanged): go_to ===== */
/* 从 (cx,cy) 规划到 (tx,ty) 并逐段执行，每段阻塞等电机到位。
 * 成功返回 1；无路径或某段超时返回 0。 */
u8 go_to(u16 *cx, u16 *cy, u16 tx, u16 ty)
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
