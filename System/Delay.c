#include "stm32f10x.h"
#include "system_stm32f10x.h"

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015（受 SysTick 24 位限制）
  * @retval 无
  * @note   重装值按 SystemCoreClock（真实 HCLK）换算，不再写死 72。
  *         写死 72 的话，一旦 HSE 8 MHz 晶振没起振、芯片退回 HSI 8 MHz，
  *         所有延时都会慢 9 倍。
  */
void Delay_us(uint32_t xus)
{
	uint32_t reload = (SystemCoreClock / 1000000u) * xus;	//1 us 需要多少个 SysTick 计数

	if (reload > 0x00FFFFFFu) reload = 0x00FFFFFFu;			//SysTick 只有 24 位，超了就截断
	if (reload == 0u) reload = 1u;

	SysTick->LOAD = reload;					//设置定时器重装值
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
	SysTick->CTRL = 0x00000004;				//关闭定时器
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 
