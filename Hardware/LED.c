#include "stm32f10x.h"                  // Device header
#include "Delay.h"
/**
  * 函    数：LED初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：只保留 PC13（板载 LED）。
  *           PA1/PA2 原来是 LED1/LED2，现在让给舵机 PWM（TIM2_CH2/CH3），
  *           所以这里不再配置它们，LED1/ LED2 相关函数也已删除。
  */
void LED_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE);		//开启GPIOA的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	/*设置GPIO初始化后的默认电平*/
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

/*
 * 注意：PA1/PA2 已经分配给了舵机 PWM（TIM2_CH2/CH3），
 * 原来的 LED1 / LED2 函数（操作 PA1/PA2）已删除，
 * 保留下面的 PC13 相关函数。
 */

void PC13_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13) == 0)		//获取输出寄存器的状态，如果当前引脚输出低电平
	{                                                  
		GPIO_SetBits(GPIOC, GPIO_Pin_13);               		//则设置PA2引脚为高电平
	}                                                  
	else                                               		//否则，即当前引脚输出高电平
	{                                                  
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);             		//则设置PA2引脚为低电平
	}
}

void PC13_pulse(void){
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
	Delay_ms(1000);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	Delay_ms(1000);
}

/* ===== moved from main.c (logic unchanged): pc13_step_blink ===== */
/* 每做完一个动作闪一下 PC13：确认"按键被识别、动作执行了" */
void pc13_step_blink(void)
{
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);   /* 亮（低电平点亮） */
    Delay_ms(60);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);     /* 灭 */
}
