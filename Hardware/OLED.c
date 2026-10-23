#include "stm32f10x.h"
#include "OLED_Font.h"
#include "OLED.h"
/*
 * 硬件 I2C1 驱动 SSD1306
 *   PB6 = I2C1_SCL（默认映射，无需重映射）
 *   PB7 = I2C1_SDA
 *   速率 400 kHz（快速模式），SSD1306 支持
 *
 * 相比原来的软件 I2C（逐位翻转 GPIO），这里：
 *   1. 由 I2C 外设产生时序，CPU 只等事件标志；
 *   2. 一条命令/一段显存数据只用一次 START/STOP，刷新时把整页 128 字节
 *      一次性发出去，而不是每个字节都 START/STOP。
 */

#define OLED_I2C            I2C1
#define OLED_I2C_CLK        RCC_APB1Periph_I2C1
#define OLED_SLAVE_ADDR     0x78		/* SSD1306 从机地址 0111 1000 */
#define OLED_I2C_SPEED      400000u		/* 400 kHz；不稳定时可降到 100000 */

#define OLED_SCL_PIN        GPIO_Pin_6
#define OLED_SDA_PIN        GPIO_Pin_7

/* 耗时测量用的定时器：TIM2 自由运行，1 个计数 = 1 us。
 * TIM3 已用于舵机 PWM，TIM2 在本工程中未被占用。
 * F103 的 TIM2 是 32 位计数器，ARR 拉满后约 71 分钟才回绕。 */
#define OLED_TIM            TIM2
#define OLED_TIM_CLK        RCC_APB1Periph_TIM2

/* 全屏空数据，用于清屏和效率测试 */
static const uint8_t oled_blank[128] = {0};

/**
  * @brief  TIM2 初始化：72MHz / 72 = 1MHz，自由运行递增计数
  */
static void OLED_Timer_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	RCC_APB1PeriphClockCmd(OLED_TIM_CLK, ENABLE);

	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;	/*1MHz -> 1 计数 = 1us*/
	TIM_TimeBaseInitStructure.TIM_Period = 0xFFFFu;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(OLED_TIM, &TIM_TimeBaseInitStructure);

	/*标准库的 TIM_Period 和头文件里的 ARR 都是 uint16_t，会截断；
	 * TIM2 硬件是 32 位，这里按 32 位直接写 ARR，回绕周期约 71 分钟*/
	*(volatile uint32_t *)&OLED_TIM->ARR = 0xFFFFFFFFu;

	TIM_SetCounter(OLED_TIM, 0);
	TIM_Cmd(OLED_TIM, ENABLE);
}

/**
  * @brief  读 TIM2 当前计数值（32 位，单位 us）
  *         头文件里 CNT 声明为 uint16_t，这里按 32 位读，避免截断
  */
static uint32_t OLED_Timer_GetUs(void)
{
	return *(volatile uint32_t *)&OLED_TIM->CNT;
}

/*引脚初始化：硬件 I2C1，PB6/PB7 复用开漏*/
void OLED_I2C_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	/*开启 GPIOB 与 I2C1 时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(OLED_I2C_CLK, ENABLE);

	/*PB6/PB7 配置为复用开漏输出（I2C 需要外部上拉）*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = OLED_SCL_PIN | OLED_SDA_PIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*I2C1 初始化：主机模式，7 位地址，400kHz*/
	I2C_DeInit(OLED_I2C);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = OLED_I2C_SPEED;
	I2C_Init(OLED_I2C, &I2C_InitStructure);
	I2C_Cmd(OLED_I2C, ENABLE);
}

/**
  * @brief  等待 I2C 事件，带超时
  * @retval 1=成功，0=超时
  */
static uint8_t OLED_I2C_WaitEvent(uint32_t event)
{
	uint16_t timeout = 20000u;

	while (I2C_CheckEvent(OLED_I2C, event) == ERROR)
	{
		if (--timeout == 0u) return 0;
	}
	return 1;
}

/**
  * @brief  总线恢复：从机把 SDA 拉死时，手动打 9 个时钟再重新初始化 I2C。
  *         重新初始化后 SSD1306 内部状态不变，后续绘图可继续。
  */
static void OLED_I2C_Recover(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	volatile uint32_t d;
	uint8_t i;

	I2C_Cmd(OLED_I2C, DISABLE);

	/*切回普通开漏 GPIO，手动产生时钟*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = OLED_SCL_PIN | OLED_SDA_PIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_SetBits(GPIOB, OLED_SDA_PIN);
	for (i = 0; i < 9u; i++)
	{
		GPIO_ResetBits(GPIOB, OLED_SCL_PIN);
		for (d = 0; d < 100u; d++) __NOP();
		GPIO_SetBits(GPIOB, OLED_SCL_PIN);
		for (d = 0; d < 100u; d++) __NOP();
	}

	/*手动 STOP：SCL 高电平期间 SDA 由低变高*/
	GPIO_ResetBits(GPIOB, OLED_SDA_PIN);
	for (d = 0; d < 100u; d++) __NOP();
	GPIO_SetBits(GPIOB, OLED_SCL_PIN);
	for (d = 0; d < 100u; d++) __NOP();
	GPIO_SetBits(GPIOB, OLED_SDA_PIN);

	OLED_I2C_Init();
}

/**
  * @brief  一次事务写入：START + 地址 + 控制字节 + 若干数据 + STOP
  * @param  control 0x00=写命令，0x40=写数据
  * @retval 1=成功，0=失败（已自动恢复总线）
  */
static uint8_t OLED_I2C_WriteBytes(uint8_t control, const uint8_t *data, uint16_t len)
{
	uint16_t i;

	I2C_GenerateSTART(OLED_I2C, ENABLE);
	if (!OLED_I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto error;

	I2C_Send7bitAddress(OLED_I2C, OLED_SLAVE_ADDR, I2C_Direction_Transmitter);
	if (!OLED_I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) goto error;

	I2C_SendData(OLED_I2C, control);
	if (!OLED_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING)) goto error;

	for (i = 0; i < len; i++)
	{
		I2C_SendData(OLED_I2C, data[i]);
		if (!OLED_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING)) goto error;
	}

	/*等最后一个字节真正发完（BTF），再发 STOP，避免截断*/
	if (!OLED_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto error;

	I2C_GenerateSTOP(OLED_I2C, ENABLE);
	return 1;

error:
	I2C_GenerateSTOP(OLED_I2C, ENABLE);
	OLED_I2C_Recover();
	return 0;
}

/**
  * @brief  OLED写命令（单条）
  */
void OLED_WriteCommand(uint8_t Command)
{
	OLED_I2C_WriteBytes(0x00, &Command, 1);
}

/**
  * @brief  OLED写多条命令（一次事务，用于设置光标）
  */
static void OLED_WriteCommands(const uint8_t *Commands, uint16_t Len)
{
	OLED_I2C_WriteBytes(0x00, Commands, Len);
}

/**
  * @brief  OLED写数据（单个字节）
  */
void OLED_WriteData(uint8_t Data)
{
	OLED_I2C_WriteBytes(0x40, &Data, 1);
}

/**
  * @brief  OLED连续写显存数据（一次事务），刷新效率的关键
  */
static void OLED_WriteDataBurst(const uint8_t *Data, uint16_t Len)
{
	OLED_I2C_WriteBytes(0x40, Data, Len);
}

/**
  * @brief  OLED设置光标位置
  * @param  Y 以左上角为原点，向下方向的坐标，范围：0~7
  * @param  X 以左上角为原点，向右方向的坐标，范围：0~127
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	uint8_t cmds[3];

	cmds[0] = 0xB0 | Y;						/*设置Y位置*/
	cmds[1] = 0x10 | ((X & 0xF0) >> 4);		/*设置X位置高4位*/
	cmds[2] = 0x00 | (X & 0x0F);			/*设置X位置低4位*/
	OLED_WriteCommands(cmds, 3);
}

/**
  * @brief  OLED清屏（每页 128 字节一次事务）
  */
void OLED_Clear(void)
{
	uint8_t j;
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0);
		OLED_WriteDataBurst(oled_blank, 128);
	}
}

/**
  * @brief  OLED显示一个字符
  * @param  Line 行位置，范围：1~4
  * @param  Column 列位置，范围：1~16
  * @param  Char 要显示的一个字符，范围：ASCII可见字符
  */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);			/*设置光标位置在上半部分*/
	OLED_WriteDataBurst(&OLED_F8x16[Char - ' '][0], 8);			/*一次发完上半部分 8 字节*/

	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);		/*设置光标位置在下半部分*/
	OLED_WriteDataBurst(&OLED_F8x16[Char - ' '][8], 8);			/*一次发完下半部分 8 字节*/
}

/**
  * @brief  OLED显示字符串
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}
}

/**
  * @brief  OLED次方函数
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

/**
  * @brief  OLED显示数字（十进制，正数）
  */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  OLED显示数字（十进制，带符号数）
  */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  OLED显示数字（十六进制，正数）
  */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			OLED_ShowChar(Line, Column + i, SingleNumber + '0');
		}
		else
		{
			OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
		}
	}
}

/**
  * @brief  OLED显示数字（二进制，正数）
  */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}
}

/**
  * @brief  OLED初始化
  */
void OLED_Init(void)
{
	uint32_t i, j;

	for (i = 0; i < 1000; i++)			//上电延时
	{
		for (j = 0; j < 1000; j++);
	}

	OLED_I2C_Init();			//硬件 I2C1 初始化
	OLED_Timer_Init();			//TIM2 1us 计时，效率测量用

	OLED_WriteCommand(0xAE);	//关闭显示

	OLED_WriteCommand(0xD5);	//设置显示时钟分频比/振荡器频率
	OLED_WriteCommand(0x80);

	OLED_WriteCommand(0xA8);	//设置多路复用率
	OLED_WriteCommand(0x3F);

	OLED_WriteCommand(0xD3);	//设置显示偏移
	OLED_WriteCommand(0x00);

	OLED_WriteCommand(0x40);	//设置显示开始行

	OLED_WriteCommand(0xA1);	//设置左右方向，0xA1正常 0xA0左右反置

	OLED_WriteCommand(0xC8);	//设置上下方向，0xC8正常 0xC0上下反置

	OLED_WriteCommand(0xDA);	//设置COM引脚硬件配置
	OLED_WriteCommand(0x12);

	OLED_WriteCommand(0x81);	//设置对比度控制
	OLED_WriteCommand(0xCF);

	OLED_WriteCommand(0xD9);	//设置预充电周期
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB);	//设置VCOMH取消选择级别
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4);	//设置整个显示打开/关闭

	OLED_WriteCommand(0xA6);	//设置正常/倒转显示

	OLED_WriteCommand(0x8D);	//设置充电泵
	OLED_WriteCommand(0x14);

	OLED_WriteCommand(0xAF);	//开启显示

	OLED_Clear();				//OLED清屏
}

/**
  * @brief  测量整屏 1024 字节刷新的耗时（内含清屏效果）
  * @retval 耗时，单位 us
  */
uint32_t OLED_MeasureFullFrame(void)
{
	uint32_t us_start, us_end;
	uint8_t page;

	us_start = OLED_Timer_GetUs();				/*1 计数 = 1us*/
	for (page = 0; page < 8; page++)
	{
		OLED_SetCursor(page, 0);				/*3 条命令一次事务*/
		OLED_WriteDataBurst(oled_blank, 128);	/*128 字节一次事务*/
	}
	us_end = OLED_Timer_GetUs();

	/*无符号相减，计数器回绕时结果仍正确*/
	return us_end - us_start;
}

/**
  * @brief  硬件 I2C 效率测试：测全屏刷新耗时，结果显示在 OLED 第 3、4 行
  * @retval 全屏耗时，单位 us
  *
  * 说明：
  *   KB/s = 1024 字节 / 耗时；
  *   占用率 = 实际传输位(1024字节×9位，含ACK) / 400kbit/s 总线在该时间内能传的位数。
  * 注意：测试过程会清空屏幕，请在其它绘图之前调用。
  */
uint32_t OLED_ShowI2CStats(void)
{
	uint32_t us, kbytes_s, util;

	us = OLED_MeasureFullFrame();
	if (us == 0u) us = 1u;

	kbytes_s = 1000000u / us;		/*1024B / us µs = 1e6/us KB/s*/
	util = 2304000u / us;			/*9216bit×100% / (0.4bit/µs×us) */
	if (util > 100u) util = 100u;

	OLED_ShowString(3, 1, "REFRESH");
	OLED_ShowNum(3, 9, us, 5);
	OLED_ShowString(3, 15, "us");

	OLED_ShowString(4, 1, "I2C");
	OLED_ShowNum(4, 5, kbytes_s, 3);
	OLED_ShowString(4, 8, "KB/s");
	OLED_ShowNum(4, 13, util, 3);
	OLED_ShowString(4, 16, "%");

	return us;
}
