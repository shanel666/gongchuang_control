#include "stm32f10x.h"                  // Device header
#include "Servo.h"

/*
 * PC7 / PC8 / PC9 的定时器复用关系：
 *   TIM3_CH1/CH2/CH3/CH4 默认在 PA6/PA7/PB0/PB1，
 *   **完全重映射** 后为 PC6/PC7/PC8/PC9，所以：
 *     PC7 = TIM3_CH2（升降）
 *     PC8 = TIM3_CH3（夹爪）
 *     PC9 = TIM3_CH4（旋转）
 *
 * ★ 注意：这是"完全重映射"（GPIO_FullRemap_TIM3），不要和"部分重映射"混了：
 *     部分重映射 CH1=PB4 CH2=PB5，CH3/CH4 仍在 PB0/PB1（不是 PC8/PC9）；
 *     完全重映射 CH1=PC6 CH2=PC7 CH3=PC8 CH4=PC9  ← 我们要的是这个。
 *   顺带好处：完全重映射把 CH1/CH2 从 PA6/PA7 挪走，PA7 可以安心当按键。
 *
 * 时基：TIM3 时钟 72 MHz，预分频 72 -> 计数频率 1 MHz（1 个计数 = 1 us），
 *       ARR = 20000 - 1 -> 周期 20 ms（50 Hz），CCR 即高电平脉宽 us 数。
 *
 * 历史：换 C8T6 时（LQFP48 没有 PC7/PC8/PC9）曾临时改用 PA0/PA1 + TIM2；
 *       现在换回 RCT6，恢复 PC7/PC8/PC9 三路。
 */

#define SERVO_TIM           TIM3
#define SERVO_TIM_CLK       RCC_APB1Periph_TIM3
#define SERVO_GPIO_CLK      RCC_APB2Periph_GPIOC
#define SERVO_GPIO_PORT     GPIOC
#define SERVO_GPIO_PIN      (GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9)   /* PC7=升降、PC8=夹爪、PC9=旋转 */

#define SERVO_PERIOD_US     20000u  /* 20 ms */
#define SERVO_TIM_PSC       (72u - 1u)

/* 每路舵机：定时器通道、上电角度、角度微调（下标就是 ServoId）
 *   SERVO_ROT  = PC9 / TIM3_CH4
 *   SERVO_GRIP = PC8 / TIM3_CH3
 *   SERVO_LIFT = PC7 / TIM3_CH2 */
static const uint8_t  k_servo_ch[SERVO_COUNT]   = {4u, 3u, 2u};
static const uint16_t k_servo_init[SERVO_COUNT] = {
    SERVO_ROT_INIT_DEG, SERVO_GRIP_INIT_DEG, SERVO_LIFT_INIT_DEG
};
static const int16_t  k_servo_trim[SERVO_COUNT] = {
    SERVO_TRIM_ROT, SERVO_TRIM_GRIP, SERVO_TRIM_LIFT
};
/* 方向取反标志（见 Servo.h 里的说明） */
static const uint8_t  k_servo_invert[SERVO_COUNT] = {
    SERVO_INVERT_ROT, SERVO_INVERT_GRIP, SERVO_INVERT_LIFT
};

static uint16_t servo_angle[SERVO_COUNT];   /* 实际角度（含微调） */
static uint16_t servo_pulse[SERVO_COUNT];   /* 实际脉宽，单位 us */

/* 角度 -> 脉宽：500 + 2000 * deg / 270，四舍五入 */
static uint16_t servo_deg_to_pulse(uint16_t deg)
{
	uint32_t pulse;

	if (deg > SERVO_ANGLE_MAX) deg = SERVO_ANGLE_MAX;

	pulse = SERVO_MIN_PULSE_US +
	        ((uint32_t)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * (uint32_t)deg
	         + SERVO_ANGLE_MAX / 2u) / SERVO_ANGLE_MAX;

	if (pulse < SERVO_MIN_PULSE_US) pulse = SERVO_MIN_PULSE_US;
	if (pulse > SERVO_MAX_PULSE_US) pulse = SERVO_MAX_PULSE_US;

	return (uint16_t)pulse;
}

/* 逻辑角度 -> 实际脉宽：方向取反的那一路按 (270 - 角度) 算，
 * 这样流程里写的角度和 OLED 显示都不用改，只是舵机转向反过来。 */
static uint16_t servo_pulse_of(uint8_t id, uint16_t deg)
{
	uint16_t d = deg;

	if (k_servo_invert[id]) d = (uint16_t)(SERVO_ANGLE_MIN + SERVO_ANGLE_MAX - d);
	if (d > SERVO_ANGLE_MAX) d = SERVO_ANGLE_MAX;

	return servo_deg_to_pulse(d);
}

/* 通道初始化：PWM 模式 1、高电平有效、CCR 预装载 */
static void servo_ch_init(uint8_t ch, uint16_t pulse)
{
	TIM_OCInitTypeDef oc;

	TIM_OCStructInit(&oc);
	oc.TIM_OCMode      = TIM_OCMode_PWM1;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_OCPolarity  = TIM_OCPolarity_High;
	oc.TIM_Pulse       = pulse;

	switch (ch)
	{
		case 1u:
			TIM_OC1Init(SERVO_TIM, &oc);
			TIM_OC1PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);
			break;
		case 2u:
			TIM_OC2Init(SERVO_TIM, &oc);
			TIM_OC2PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);
			break;
		case 3u:
			TIM_OC3Init(SERVO_TIM, &oc);
			TIM_OC3PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);
			break;
		case 4u:
			TIM_OC4Init(SERVO_TIM, &oc);
			TIM_OC4PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);
			break;
		default:
			break;
	}
}

/* 写 CCR：1 个计数 = 1 us，所以 CCR 就是脉宽 */
static void servo_ch_set(uint8_t ch, uint16_t pulse)
{
	switch (ch)
	{
		case 1u: TIM_SetCompare1(SERVO_TIM, (uint32_t)pulse); break;
		case 2u: TIM_SetCompare2(SERVO_TIM, (uint32_t)pulse); break;
		case 3u: TIM_SetCompare3(SERVO_TIM, (uint32_t)pulse); break;
		case 4u: TIM_SetCompare4(SERVO_TIM, (uint32_t)pulse); break;
		default: break;
	}
}

/**
  * 函    数：两路舵机 PWM 初始化（PC8 夹爪 / PC9 旋转，TIM3 完全重映射）
  * 参    数：无
  * 返 回 值：无
  * 说    明：初始化完成立即输出安全角度（旋转 240°/夹爪 65°）
  */
void Servo_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	uint8_t i;
	int32_t d;

	/*开启时钟：GPIO + 复用功能重映射 + TIM3*/
	RCC_APB2PeriphClockCmd(SERVO_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(SERVO_TIM_CLK, ENABLE);

	/*TIM3 完全重映射：CH3/CH4 输出到 PC8/PC9*/
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

	/*PC8/PC9 配置为复用推挽输出*/
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

	/*两路一起初始化：先把安全角度算好，PWM 一使能就是安全姿态*/
	for (i = 0; i < (uint8_t)SERVO_COUNT; i++)
	{
		d = (int32_t)k_servo_init[i] + (int32_t)k_servo_trim[i];
		if (d < (int32_t)SERVO_ANGLE_MIN) d = (int32_t)SERVO_ANGLE_MIN;
		if (d > (int32_t)SERVO_ANGLE_MAX) d = (int32_t)SERVO_ANGLE_MAX;

		servo_angle[i] = (uint16_t)d;
		servo_pulse[i] = servo_pulse_of(i, servo_angle[i]);
		servo_ch_init(k_servo_ch[i], servo_pulse[i]);
	}

	/*ARR 预装载，改周期也在下一个周期生效*/
	TIM_ARRPreloadConfig(SERVO_TIM, ENABLE);

	TIM_Cmd(SERVO_TIM, ENABLE);
}

/**
  * 函    数：设置某一路舵机角度
  * 参    数：id  舵机编号（SERVO_ROT / SERVO_GRIP）
  *           deg 目标角度，0~270，超出自动限幅
  * 返 回 值：无
  */
void Servo_SetAngle(ServoId id, uint16_t deg)
{
	int32_t d;

	if ((uint8_t)id >= (uint8_t)SERVO_COUNT) return;

	d = (int32_t)deg + (int32_t)k_servo_trim[id];          /* 加现场微调 */
	if (d < (int32_t)SERVO_ANGLE_MIN) d = (int32_t)SERVO_ANGLE_MIN;
	if (d > (int32_t)SERVO_ANGLE_MAX) d = (int32_t)SERVO_ANGLE_MAX;

	servo_angle[id] = (uint16_t)d;
	servo_pulse[id] = servo_pulse_of((uint8_t)id, (uint16_t)d);

	servo_ch_set(k_servo_ch[id], servo_pulse[id]);
}

/**
  * 函    数：读取某一路舵机当前角度（含微调）
  * 参    数：id 舵机编号
  * 返 回 值：实际角度，单位 度；id 非法时返回 0
  */
uint16_t Servo_GetAngle(ServoId id)
{
	if ((uint8_t)id >= (uint8_t)SERVO_COUNT) return 0u;
	return servo_angle[id];
}

/**
  * 函    数：读取某一路舵机当前脉宽
  * 参    数：id 舵机编号
  * 返 回 值：实际写进 CCR 的脉宽，单位 us；id 非法时返回 0
  */
uint16_t Servo_GetPulse(ServoId id)
{
	if ((uint8_t)id >= (uint8_t)SERVO_COUNT) return 0u;
	return servo_pulse[id];
}
