#include "stm32f10x.h"                  // Device header
#include "Servo.h"

/*
 * PC8 的定时器复用关系：
 *   TIM3_CH1/CH2/CH3/CH4 默认在 PA6/PA7/PB0/PB1，
 *   完全重映射后为 PC6/PC7/PC8/PC9，所以 PC8 = TIM3_CH3。
 *
 * 时基：TIM3 时钟 72 MHz，预分频 72 -> 计数频率 1 MHz（1 个计数 = 1 us），
 *       ARR = 20000 - 1 -> 周期 20 ms（50 Hz），CCR 即高电平脉宽 us 数。
 */

#define SERVO_TIM           TIM3
#define SERVO_TIM_CLK       RCC_APB1Periph_TIM3
#define SERVO_GPIO_CLK      RCC_APB2Periph_GPIOC
#define SERVO_GPIO_PORT     GPIOC
#define SERVO_GPIO_PIN      GPIO_Pin_8

#define SERVO_PERIOD_US     20000u  /* 20 ms */
#define SERVO_TIM_PSC       (72u - 1u)

/* 上电初始脉宽 = 张开角度对应的脉宽，保证 PWM 一启动就是张开状态 */
static uint16_t servo_pulse_us = (uint16_t)SERVO_ANGLE_TO_PULSE(SERVO_ANGLE_OPEN);

/**
  * 函    数：舵机PWM初始化
  * 参    数：无
  * 返 回 值：无
  */
void Servo_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/*开启时钟：GPIO + 复用功能重映射 + TIM3*/
	RCC_APB2PeriphClockCmd(SERVO_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(SERVO_TIM_CLK, ENABLE);

	/*TIM3 完全重映射，CH3 输出到 PC8*/
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

	/*PC8 配置为复用推挽输出*/
	GPIO_InitStructure.GPIO_Pin = SERVO_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SERVO_GPIO_PORT, &GPIO_InitStructure);

	/*时基单元：1 us 计数，20 ms 周期*/
	TIM_TimeBaseInitStructure.TIM_Prescaler = SERVO_TIM_PSC;
	TIM_TimeBaseInitStructure.TIM_Period = SERVO_PERIOD_US - 1u;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(SERVO_TIM, &TIM_TimeBaseInitStructure);

	/*CH3 PWM 模式 1：CNT < CCR 时输出高电平*/
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = servo_pulse_us;   /* 1 个计数 = 1 us，CCR 即脉宽 */
	TIM_OC3Init(SERVO_TIM, &TIM_OCInitStructure);

	/*CCR 预装载，改脉宽时在下一个周期生效，避免出现畸形脉冲*/
	TIM_OC3PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(SERVO_TIM, ENABLE);

	TIM_Cmd(SERVO_TIM, ENABLE);
}

/**
  * 函    数：设置舵机脉宽
  * 参    数：pulse_us 高电平时间，500~2500，单位 us
  * 返 回 值：无
  */
void Servo_SetPulse(uint16_t pulse_us)
{
	if (pulse_us < SERVO_MIN_PULSE_US) pulse_us = SERVO_MIN_PULSE_US;
	if (pulse_us > SERVO_MAX_PULSE_US) pulse_us = SERVO_MAX_PULSE_US;

	servo_pulse_us = pulse_us;
	TIM_SetCompare3(SERVO_TIM, (uint32_t)pulse_us);
}

/**
  * 函    数：设置舵机角度
  * 参    数：angle_deg 角度，0~SERVO_ANGLE_MAX，单位 度
  * 返 回 值：无
  */
void Servo_SetAngle(uint16_t angle_deg)
{
	uint32_t pulse;

	if (angle_deg > SERVO_ANGLE_MAX) angle_deg = SERVO_ANGLE_MAX;

	/* 500 + 2000 * angle / ANGLE_MAX，四舍五入 */
	pulse = SERVO_MIN_PULSE_US +
	        ((uint32_t)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * angle_deg
	         + SERVO_ANGLE_MAX / 2u) / SERVO_ANGLE_MAX;

	Servo_SetPulse((uint16_t)pulse);
}

/**
  * 函    数：夹爪张开（38°）
  * 参    数：无
  * 返 回 值：无
  */
void Servo_Open(void)
{
	Servo_SetAngle(SERVO_ANGLE_OPEN);
}

/**
  * 函    数：夹爪加紧（44°）
  * 参    数：无
  * 返 回 值：无
  */
void Servo_Clamp(void)
{
	Servo_SetAngle(SERVO_ANGLE_CLAMP);
}

/**
  * 函    数：读取当前脉宽
  * 参    数：无
  * 返 回 值：当前高电平脉宽，单位 us
  */
uint16_t Servo_GetPulse(void)
{
	return servo_pulse_us;
}
