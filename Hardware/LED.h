#ifndef __LED_H
#define __LED_H

/* 注意：PA1/PA2 已分配给舵机 PWM（TIM2_CH2/CH3），
 * 原来操作 PA1/PA2 的 LED1 / LED2 函数已删除，只保留 PC13。 */

void LED_Init(void);
void PC13_pulse(void);
void PC13_Turn(void);
	
#endif
