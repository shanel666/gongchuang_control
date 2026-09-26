#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Key.h"
#include "LED.h"
#include "motor.h"
#include "path_planner.h"
#include "path_config.h"

static u8 dir = PATH_INITIAL_DIR;
static u8 judge_display_count = 0;

#define ABS(x) ((x) < 0 ? -(x) : (x))

void judge(void)
{
    u8 done1 = 0, done2 = 0, done3 = 0, done4 = 0;
    u16 t, guard = 0;

    Serial_RxFlag = 0;
    Serial_RxPacket[0] = Serial_RxPacket[1] = Serial_RxPacket[2] = Serial_RxPacket[3] = 0;

    while(1){
        OLED_ShowNum(1,1,done1,1);
        OLED_ShowNum(2,1,done2,1);
        OLED_ShowNum(3,1,done3,1);
        OLED_ShowNum(4,1,done4,1);

        if(done1 == 0){
            Serial_RxFlag = 0;
            Serial_RxPacket[0]=Serial_RxPacket[1]=Serial_RxPacket[2]=Serial_RxPacket[3]=0;
            Serial_SendByte(0x01); Serial_SendByte(0x3A); Serial_SendByte(0x6B);
            t = 0;
            while(t < 50){ if(Serial_GetRxFlag()) break; Delay_ms(1); t++; }
            if(Serial_RxPacket[0]==0x01 && Serial_RxPacket[1]==0x3A && Serial_RxPacket[2]==0x03)
                done1 = 1;
            Serial_RxFlag = 0;
        }
        Delay_ms(5);

        /* done2 / done3 / done4 用同样模板，把地址分别换成 0x02 / 0x03 / 0x04 */

        if(done1 && done2 && done3 && done4){
            Serial_RxFlag = 0;
            return;
        }

        if(++guard > 400) break;   /* 强退保护 */
        OLED_ShowNum(1, 3, judge_display_count++, 5);
    }
}

static void translation(u16 x0, u16 y0, u16 x1, u16 y1)
{
    if(x1 == x0 && y1 > y0){
        if(dir == 2){ shun_yaw_(90); Delay_ms(1000); judge(); dir = 1; }
        if(dir == 4){ ni_yaw_(90);   Delay_ms(1000); judge(); dir = 1; }
        if(dir == 3) houtui_(ABS(y1 - y0));
        else         qianjin_(ABS(y1 - y0));
    }else if(x1 == x0 && y1 < y0){
        if(dir == 2){ ni_yaw_(90);   Delay_ms(1000); judge(); dir = 3; }
        if(dir == 4){ shun_yaw_(90); Delay_ms(1000); judge(); dir = 3; }
        if(dir == 3) qianjin_(ABS(y1 - y0));
        else         houtui_(ABS(y1 - y0));
    }else if(x1 < x0 && y1 == y0){
        if(dir == 1){ ni_yaw_(90); Delay_ms(1000); judge(); dir = 2; }
        else if(dir == 3){ shun_yaw_(90); Delay_ms(1000); judge(); dir = 2; }
        if(dir == 2) qianjin_(ABS(x1 - x0));
        else         houtui_(ABS(x1 - x0));
    }else if(x1 > x0 && y1 == y0){
        if(dir == 1){ shun_yaw_(90); Delay_ms(1000); judge(); dir = 4; }
        if(dir == 3){ ni_yaw_(90);   Delay_ms(1000); judge(); dir = 4; }
        if(dir == 4) qianjin_(ABS(x1 - x0));
        else         houtui_(ABS(x1 - x0));
    }

    judge();
}
void judge_yaw(void)
{
    u8 done1 = 0, done2 = 0;
    u16 t, guard = 0;

    Serial_RxFlag = 0;
    Serial_RxPacket[0] = Serial_RxPacket[1] = Serial_RxPacket[2] = Serial_RxPacket[3] = 0;

    while(!(done1 && done2)){
        OLED_ShowNum(1,1,done1,1);
        OLED_ShowNum(2,1,done2,1);
        OLED_ShowHexNum(3,1, Serial_RxPacket[0], 2);
        OLED_ShowHexNum(3,4, Serial_RxPacket[1], 2);
        OLED_ShowHexNum(3,7, Serial_RxPacket[2], 2);
        OLED_ShowHexNum(3,10,Serial_RxPacket[3], 2);

        if(done1 == 0){
            Serial_RxFlag = 0;
            Serial_RxPacket[0]=Serial_RxPacket[1]=Serial_RxPacket[2]=Serial_RxPacket[3]=0;
            Serial_SendByte(0x01); Serial_SendByte(0x3A); Serial_SendByte(0x6B);
            t = 0;
            while(t < 50){ if(Serial_GetRxFlag()) break; Delay_ms(1); t++; }
            if(Serial_RxPacket[0]==0x01 && Serial_RxPacket[1]==0x3A && Serial_RxPacket[2]==0x03)
                done1 = 1;
            Serial_RxFlag = 0;
        }
        Delay_ms(5);

        if(done2 == 0){
            Serial_RxFlag = 0;
            Serial_RxPacket[0]=Serial_RxPacket[1]=Serial_RxPacket[2]=Serial_RxPacket[3]=0;
            Serial_SendByte(0x02); Serial_SendByte(0x3A); Serial_SendByte(0x6B);
            t = 0;
            while(t < 50){ if(Serial_GetRxFlag()) break; Delay_ms(1); t++; }
            if(Serial_RxPacket[0]==0x02 && Serial_RxPacket[1]==0x3A && Serial_RxPacket[2]==0x03)
                done2 = 1;
            Serial_RxFlag = 0;
        }
        Delay_ms(5);

        if(++guard > 400) break;   /* 大约 44s 强退，防止死循环 */
    }
}

/* —— 标定 —— */
#define PULSE_PER_100CM  12658u   /* 平移标定：12658 脉冲 = 100 cm（已标定） */
#define PULSE_PER_360DEG    16028u     /* 旋转标定：每度脉冲数 —— TODO 待标定！当前为占位值 360度 15800*/

/* 速度 / 加速度（可调） */
#define MOTOR_SPEED      50    /* 速度 0x07D0；上位机默认 200 已较快 */
#define MOTOR_ACCEL      200     /* 加速度 0x64 */
#define MOTOR_SPEED_H    ((uint8_t)(MOTOR_SPEED >> 8))   /* 速度高字节 */
#define MOTOR_SPEED_L    ((uint8_t)(MOTOR_SPEED & 0xFF)) /* 速度低字节 */

int main(void)
{
    u16 xpath[MAX_KEY_POINTS];
    u16 ypath[MAX_KEY_POINTS];
    u8 i;

    LED_Init();
    OLED_Init();
    Key_Init();
    Serial_Init();
    Delay_ms(1000);

	
//	uint32_t p = 600 * PULSE_PER_100CM / 1000u;   /* 脉冲数 */
//    uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

//    cmd[6] = (uint8_t)(p >> 24);
//    cmd[7] = (uint8_t)(p >> 16);
//    cmd[8] = (uint8_t)(p >> 8);
//    cmd[9] = (uint8_t)p;

//    cmd[0] = 0x01; cmd[2] = 0x00;   /* FL 正转 */
//    Serial_SendArray(cmd, 13); Delay_ms(5);
//    cmd[0] = 0x02; cmd[2] = 0x01;   /* FR 反转（镜像） */
//    Serial_SendArray(cmd, 13); Delay_ms(5);
//    cmd[0] = 0x03; cmd[2] = 0x00;   /* RL 正转 */
//    Serial_SendArray(cmd, 13); Delay_ms(5);
//    cmd[0] = 0x04; cmd[2] = 0x01;   /* RR 反转（镜像） */
//    Serial_SendArray(cmd, 13); Delay_ms(5);

//    /* 同步释放：直行 sync=0x01，此帧让 4 个电机同时起步 */
//    Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
//	Delay_ms(5);
//	judge();
//	while(1){}

//    uint32_t p = 90 *PULSE_PER_360DEG/360u;
//    uint8_t cmd[13] = {0x00, 0xFD, 0x00, MOTOR_SPEED_H, MOTOR_SPEED_L, (uint8_t)MOTOR_ACCEL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6B};

//    cmd[6] = (uint8_t)(p >> 24);
//    cmd[7] = (uint8_t)(p >> 16);
//    cmd[8] = (uint8_t)(p >> 8);
//    cmd[9] = (uint8_t)p;

//    cmd[0] = 0x01; cmd[2] = 0x00;   /* FL */
//    Serial_SendArray(cmd, 13); Delay_ms(5);
//    cmd[0] = 0x02; cmd[2] = 0x00;   /* FR */
//    Serial_SendArray(cmd, 13); Delay_ms(5);
////    cmd[0] = 0x03; cmd[2] = 0x00;   /* RL */
////    Serial_SendArray(cmd, 13); Delay_ms(5);
////    cmd[0] = 0x04; cmd[2] = 0x00;   /* RR */
////    Serial_SendArray(cmd, 13); Delay_ms(5);

//    Serial_SendByte(0x00); Serial_SendByte(0xFF); Serial_SendByte(0x66); Serial_SendByte(0x6B);
//	Delay_ms(5);
//	judge();
//	while(1){}
		
		
	
    shun_yaw_(90);
    judge_yaw();
	Delay_ms(500);
    houtui_(600);
    judge();
    Delay_ms(500);
    shun_yaw_(90);
    judge_yaw();
    Delay_ms(500);
    qianjin_(600);
    judge();
	while(1){}
		
		
    path_planner_init();
    path_plan(PATH_START_X, PATH_START_Y, PATH_TARGET_X, PATH_TARGET_Y,
              MAX_KEY_POINTS, xpath, ypath);

    if(key_point_num == 0){
        OLED_ShowString(1, 1, "PATH ERROR");
        while(1){}
    }

    motor_enable_all();

    for(i = 1; i < key_point_num; i++){
        translation(xpath[i - 1], ypath[i - 1], xpath[i], ypath[i]);
    }

    OLED_ShowString(1, 1, "PATH DONE");
    while(1){}
}
