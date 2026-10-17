#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Gyro.h"
#include "Key.h"
#include "LED.h"
#include "motor.h"
#include "path_planner.h"
#include "path_config.h"

static u8 dir = PATH_INITIAL_DIR;
static u16 judge_trans_display_count = 0;

static u8 judge(void);

/* 等待四个驱动电机全部到位。
 * 0x3A 查询的回包格式为 [地址][0x3A][状态][0x6B]，其中状态字节 (Serial_RxPacket[2])
 * 为 0x03 时表示该电机已到达。每查一个地址，先清标志再发查询，然后阻塞等回包，
 * 确认回包地址与本电机一致后再判断到位，避免把别的电机的回包误判成自己的。 */
static u8 judge(void)
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

        if(++guard > 60000u){   /* 安全兜底，仅在电机死机时触发 */
            return 0;
        }
    }
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

// #define __MIKELIN_DEBUG

#ifndef __MIKELIN_DEBUG
/* 主程序区，原本的功能 */
int main(void)
{
    u16 cur_x = PATH_START_X;
    u16 cur_y = PATH_START_Y;
    u8 step = 0;
	
	s32 yaw_centi_degrees = 0, last_yaw_centi_degrees = 0;
    LED_Init();
    OLED_Init();
    Key_Init();
    Serial_Init();
    Gyro_Init();
    USART3_Init(115200);
    USART2_Init(115200);
    path_planner_init();
    motor_enable_all();

	/* 通用发送自检：发送字符串/字符/整数/小数/角度，串口终端核对输出。
	 * 需要恢复实时陀螺仪角度输出时，注释掉本循环、打开下方被注释的循环即可。 */
//	while(1){
//		USART3_SelfTest();
//		Delay_ms(1000);
//	}

	/* 陀螺仪角度经 USART3(PB10) 输出给上位机，格式 "xxx.xx\r\n"。
	 * 下位机以 \r 或 \n 作为一帧结束，直接按字符串显示即可。 */

//	while(1){
//		if (Gyro_GetYawCentiDegrees(&yaw_centi_degrees))
//		{
//			USART3_SendAngleCentiDegrees(yaw_centi_degrees);
//			OLED_ShowSignedNum(2, 1, yaw_centi_degrees / 100, 8);
//		}
//		Delay_ms(200);
//	}

	// s32 yaw_error, rel;
	// while(1){
	// 	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET){
	// 		Delay_ms(20);
	// 		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET){
	// 			/* 逆时针 90°：先记起始角。你的陀螺仪逆时针是角度增大(0→+180→-180→0)，
	// 			 * 所以目标 = 起始角 + 90°，单位 0.01°。 */
	// 			if(Gyro_GetYawCentiDegrees(&last_yaw_centi_degrees)){
	// 				for(int i=0;i<5;i++){
	// 					if(!Gyro_GetYawCentiDegrees(&yaw_centi_degrees)) break;
	// 					/* 相对转角和误差都包到 [-180°,180°)，跨 ±180° 才不跳变 */
	// 					rel = gyro_wrap_180(yaw_centi_degrees - last_yaw_centi_degrees);
	// 					yaw_error = gyro_wrap_180(9000 - rel);

	// 					if(yaw_error >= -100 && yaw_error <= 100) break;  /* 误差≤1°到位 */

	// 					if(yaw_error > 0)
	// 						ni_yaw_((uint32_t)(( yaw_error + 50) / 100));    /* 还没转够，继续逆时针 */
	// 					else
	// 						shun_yaw_((uint32_t)((-yaw_error + 50) / 100));  /* 转过了，顺时针补回 */

	// 					if(!judge()) break;
	// 					Delay_ms(50);
	// 				}
	// 			}
	// 			/* 松手防抖，避免按住时连续触发 */
	// 			while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET) Delay_ms(10);
	// 			Delay_ms(20);
	// 		}
	// 	}
	// 	USART3_Printf("angle=%d\r\n",(int)yaw_centi_degrees);
	// 	Delay_ms(20);
	// }

    /* 临时角度输出测试：陀螺仪以 100 Hz 接收，USART3 以 5 Hz 输出最新角度。 */
//    while(1)
//    {
//        if (Gyro_GetYawCentiDegrees(&yaw_centi_degrees))
//            USART3_SendAngleCentiDegrees(yaw_centi_degrees);
// 			OLED_ShowSignedNum(2,1,yaw_centi_degrees/100,8);
//        Delay_ms(200);
//    }

    // /* PA8 低电平按键，作为临时 90 度闭环测试触发。 */
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // {
    //     GPIO_InitTypeDef GPIO_InitStructure;
    //     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    //     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    //     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //     GPIO_Init(GPIOA, &GPIO_InitStructure);
    // }
    // Delay_ms(200);

    

    // /* 临时比例转向测试：PA8 按下闭环转向 +90 度，松开后可再次测试。
    //  * 测试循环放在正式路线循环之前，因此本版本不会启动自动路线。 */
    // OLED_ShowString(1, 1, "PA8 TURN TEST");
    // while(1)
    // {
    //     if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET){
    //         Delay_ms(20);
    //         if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET){
    //             OLED_ShowString(2, 1, "TURNING       ");
    //             if(gyro_turn_pos_test(9000L))
    //                 OLED_ShowString(2, 1, "TURN OK       ");
    //             else
    //                 OLED_ShowString(2, 1, "TURN ERROR    ");

    //             while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET)
    //                 Delay_ms(10);
    //             Delay_ms(20);
    //         }
    //     }
    //     Delay_ms(10);
    // }

    /* 路线：每个 case 开头指定「下一个地点」，随后 go_to 完成规划 + 发送电机 + 等到位，
     * 全部到位后才进入下一个 case（不共享 path_ready/path_index 之类的中间状态）。 */
    while(1)
    {
        OLED_ShowNum(1, 1, step, 4);

        /* 显示最近一次校验通过的 Z 轴角度，单位 0.01 度。传感器未接入时
         * 保留空白，解析仍在 USART2 中断中持续进行。 */
        OLED_ShowString(4, 1, "YAW:");
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

#else
/* 测试区，学习代码用，并且用来做一些测试 */
int main(void)
{
    u16 cur_x = PATH_START_X;
    u16 cur_y = PATH_START_Y;
    u8 step = 0;

    OLED_Init();
    Serial_Init();
    Gyro_Init();
    USART3_Init(115200);
    USART2_Init(115200);
    path_planner_init();
    motor_enable_all();

    gyro_turn_pos_test(9000L);


    while(1) {
        
    }

}

#endif

