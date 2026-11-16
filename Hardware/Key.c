#include "stm32f10x.h"                  // Device header
#include "Delay.h"

/*
 * 按键：本板按键资源是 PB1 / PA7 / PA5（三个按键），
 *       极性是"一脚接 3.3V、一脚接引脚"，即**按下时引脚变高**，
 *       所以配下拉输入(IPD)，读到高电平 = 按下。
 *
 * 注意：这里只配 PB1（键码 1）。PA7/PA5 在夹取动作里单独用（连跑/中止），
 *       由 main.c 的 grab_key_init() 配置。
 *       PA11/PA12 不再当按键用（PA12 是 USB D+，板上有 1.5k 上拉，会被一直读成高电平，
 *       一旦进"等松手"的死循环就卡住，所以彻底不用它）。
 */
#define KEY_ACTIVE_HIGH   1

#if KEY_ACTIVE_HIGH
  #define KEY_GPIO_MODE   GPIO_Mode_IPD
  #define KEY_PRESSED     1          /* 读到高 = 按下 */
#else
  #define KEY_GPIO_MODE   GPIO_Mode_IPU
  #define KEY_PRESSED     0          /* 读到低 = 按下 */
#endif

/* 读一个按键：1 = 按下 */
static uint8_t Key_IsDown(GPIO_TypeDef *Port, uint16_t Pin)
{
	return (GPIO_ReadInputDataBit(Port, Pin) == (BitAction)KEY_PRESSED) ? 1u : 0u;
}

/**
  * 函    数：按键初始化
  * 参    数：无
  * 返 回 值：无
  */
void Key_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

	/* PB1（按键 1） */
	GPIO_InitStructure.GPIO_Mode = KEY_GPIO_MODE;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* PA7 / PA5（另外两个按键，夹取动作里用） */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
  * 函    数：按键获取键码
  * 参    数：无
  * 返 回 值：1 = PB1 按下，0 = 没有按键
  * 注意事项：阻塞式，按住不放会卡住直到松手
  */
uint8_t Key_GetNum(void)
{
	uint8_t KeyNum = 0;

	if (Key_IsDown(GPIOB, GPIO_Pin_1))
	{
		Delay_ms(20);
		while (Key_IsDown(GPIOB, GPIO_Pin_1));
		Delay_ms(20);
		KeyNum = 1;
	}

	return KeyNum;
}
