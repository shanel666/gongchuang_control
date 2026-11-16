#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "stm32f10x_gpio.h"
#include "OLED.h"

/* —— PB1/PA7/PA5 的极性（从 main.c 搬过来，逻辑未改）——
 * 本板按键：一脚接 3.3V、一脚接单片机引脚 → **按下时引脚变高**，
 * 所以按键脚统一配下拉输入（IPD），读到高电平 = 按下。
 * 如果以后换成"按下接 GND"的按键，把下面改成 0 并把 IPD 换成 IPU 即可。 */
#define GRAB_KEY_ACTIVE_HIGH   1

#if GRAB_KEY_ACTIVE_HIGH
  #define GRAB_KEY_MODE        GPIO_Mode_IPD
#else
  #define GRAB_KEY_MODE        GPIO_Mode_IPU
#endif

/* 等松手的超时（ms）：按着不放最多等这么久，避免死循环 */
#define GRAB_RELEASE_TIMEOUT   400u

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

/* PA8 是否按下（带消抖）。从 main.c 原样搬过来，逻辑未改。
 *
 * ★ 极性和上面三个键**相反**：PA8 是"按下接 GND"，所以配上拉输入(IPU)、
 *   读到低电平才算按下。调用方自己负责把 PA8 配成 GPIO_Mode_IPU。 */
uint8_t Key_PA8Hit(void)
{
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) != Bit_RESET) return 0u;
    Delay_ms(20);
    return (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET) ? 1u : 0u;
}

/* ===== moved from main.c (logic unchanged): grab_key_down ===== */
/* 读一个按键：1 = 按下 */
uint8_t grab_key_down(GPIO_TypeDef *port, uint16_t pin)
{
#if GRAB_KEY_ACTIVE_HIGH
    return (GPIO_ReadInputDataBit(port, pin) == Bit_SET) ? 1u : 0u;
#else
    return (GPIO_ReadInputDataBit(port, pin) == Bit_RESET) ? 1u : 0u;
#endif
}

/* ===== moved from main.c (logic unchanged): grab_key_init ===== */
/* 按键初始化：PB1（单步）、PA7（连跑）、PA5（中止），全部下拉输入
 * （按键资源：PB1 / PA7 / PA5；按下时引脚变高）。
 * 不再使用 PA8/PA11/PA12：PA12 是 USB D+，板上有 1.5k 上拉，
 * 会被一直读成"按下"，从而卡在"等松手"里，导致按 PB1 完全没反应。 */
void grab_key_init(void)
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

/* ===== moved from main.c (logic unchanged): grab_wait_release ===== */
uint8_t grab_wait_release(GPIO_TypeDef *port, uint16_t pin)
{
    uint16_t t;

    for(t = 0; t < GRAB_RELEASE_TIMEOUT; t++){
        if(!grab_key_down(port, pin)) return 1u;
        Delay_ms(5);
    }
    return 0u;
}

/* ===== moved from main.c (logic unchanged): grab_start_hit ===== */
/* 连跑键 PA7：20 ms 消抖 + 等松手，有按下返回 1 */
uint8_t grab_start_hit(void)
{
    if(!grab_key_down(GPIOA, GPIO_Pin_7)) return 0;

    Delay_ms(20);
    if(!grab_key_down(GPIOA, GPIO_Pin_7)) return 0;

    grab_wait_release(GPIOA, GPIO_Pin_7);      /* 等松手（带超时，不会卡死） */
    Delay_ms(20);
    return 1;
}

/* ===== moved from main.c (logic unchanged): grab_step_hit ===== */
/* 单步键 PB1：20 ms 消抖 + 等松手，有按下返回 1 */
uint8_t grab_step_hit(void)
{
    if(!grab_key_down(GPIOB, GPIO_Pin_1)) return 0u;

    Delay_ms(20);

    if(!grab_key_down(GPIOB, GPIO_Pin_1)) return 0u;          /* 抖动 */

    grab_wait_release(GPIOB, GPIO_Pin_1);                    /* 等松手（带超时） */

    Delay_ms(20);
    return 1u;
}

/* ===== moved from main.c (logic unchanged): grab_abort_hit ===== */
/* 中止键 PA5：按下后等松手再返回 1，避免一直按着被重复判定 */
uint8_t grab_abort_hit(void)
{
    if(!grab_key_down(GPIOA, GPIO_Pin_5)) return 0;

    grab_wait_release(GPIOA, GPIO_Pin_5);
    return 1;
}

/* ===== moved from main.c (logic unchanged): grab_wait ===== */
/* 等 ms 毫秒；期间 PA5 按下立刻返回 0（中止本次动作） */
uint8_t grab_wait(uint16_t ms)
{
    uint16_t t;

    for(t = 0; t < ms; t++){
        if(grab_abort_hit()) return 0;
        Delay_ms(1);
    }
    return 1;
}
