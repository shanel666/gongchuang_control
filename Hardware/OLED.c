#include "stm32f10x.h"
#include "OLED_Font.h"
#include "OLED.h"
/*
 * 硬件 I2C1 驱动 SSD1306
 *
 * 引脚（芯片内部固定，不能自己指定）：
 *   PB6 = I2C1_SCL
 *   PB7 = I2C1_SDA
 * 也就是 **SCL 接 PB6、SDA 接 PB7**，默认映射，不需要重映射。
 *
 * 为什么用 I2C1 而不是 I2C2：
 *   PB10/PB11 是 I2C2 的脚，但这两个脚要还给 USART3（调试/无线串口）；
 *   I2C1 的默认脚就是 PB6/PB7，本工程里没有别的用途，正好给 OLED。
 *   （LQFP48 封装没有 PC10/PC11，所以 USART3 没法重映射出别的脚。）
 *
 * 速率 400 kHz；线接得长/不稳定时把 OLED_I2C_SPEED 降到 100000。
 */

#define OLED_I2C            I2C1
#define OLED_I2C_CLK        RCC_APB1Periph_I2C1
#define OLED_SLAVE_ADDR     0x78		/* SSD1306 从机地址 0111 1000 */
#define OLED_I2C_SPEED      100000u		/* 100 kHz。
										 * 原来是 400 kHz，但 vision 模式每 100 ms 整屏刷 4 行
										 * （约 10 KB/s）时，400 kHz 下偶发 I2C 事务超时，
										 * 表现是"每个字只显示上半截"——因为 OLED_ShowChar
										 * 是"上半 8 字节一次事务、下半 8 字节又一次事务"，
										 * 下半那次失败就只剩上半截。
										 * 100 kHz 只是慢一点，刷新率本来就不高，够用。 */

#define OLED_SCL_PIN        GPIO_Pin_6	/* PB6 = I2C1_SCL */
#define OLED_SDA_PIN        GPIO_Pin_7	/* PB7 = I2C1_SDA */

/* 注意：TIM2 已经分配给两路舵机 PWM（PA1/PA2），这里不碰任何定时器。 */

/* 全屏空数据，用于清屏和效率测试 */
static const uint8_t oled_blank[128] = {0};

/* 面板是否在线：OLED_Init() 里探测一次。
 * 不在线时所有绘图函数直接返回，避免每次 I2C 都等超时。 */
static uint8_t oled_ready = 0;

/* I2C 事务失败计数。写失败时 OLED_ShowChar 不会报错（它只恢复总线就继续），
 * 屏幕上的表现就是"缺笔划/只显示上半截"。把这个数打到调试串口就能判断
 * 到底是真的 I2C 出错，还是画的内容问题。 */
static volatile uint32_t oled_i2c_err = 0u;

uint32_t OLED_GetI2cErrors(void)
{
	return oled_i2c_err;
}

/**
  * @brief  面板是否在线（1 = 在线，可以画；0 = 没接/没应答，绘图全部跳过）
  */
uint8_t OLED_IsReady(void)
{
	return oled_ready;
}

/* 引脚初始化：PB10(SCL)/PB11(SDA) 复用开漏 */
void OLED_I2C_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	/* 开启 GPIOB 与 I2C1 时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(OLED_I2C_CLK, ENABLE);

	/* 复用开漏（I2C 需要上拉；模块一般自带 4.7k） */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = OLED_SCL_PIN | OLED_SDA_PIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* I2C1 初始化：主机模式，7 位地址，400kHz */
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

/* ---------------- 硬件 I2C1 ---------------- */

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
  * @brief  总线恢复：从机把 SDA 拉死时，手动打 9 个时钟再把 I2C 外设重新初始化
  */
static void OLED_I2C_Recover(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	volatile uint32_t d;
	uint8_t i;

	I2C_Cmd(OLED_I2C, DISABLE);

	/* 切回普通开漏 GPIO，手动产生时钟 */
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

	/* 手动 STOP：SCL 高电平期间 SDA 由低变高 */
	GPIO_ResetBits(GPIOB, OLED_SDA_PIN);
	for (d = 0; d < 100u; d++) __NOP();
	GPIO_SetBits(GPIOB, OLED_SCL_PIN);
	for (d = 0; d < 100u; d++) __NOP();
	GPIO_SetBits(GPIOB, OLED_SDA_PIN);

	OLED_I2C_Init();          /* 重新配回复用开漏 + 使能 I2C1 */
}

/**
  * @brief  一次事务写入：START + 地址 + 控制字节 + 若干数据 + STOP（硬件 I2C1）
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

	/* 等最后一个字节真正发完（BTF），再发 STOP，避免截断 */
	if (!OLED_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto error;

	I2C_GenerateSTOP(OLED_I2C, ENABLE);
	return 1;

error:
	oled_i2c_err++;                 /* 记一笔：屏幕缺笔划时就靠这个数判断 */
	I2C_GenerateSTOP(OLED_I2C, ENABLE);
	OLED_I2C_Recover();
	return 0;
}

/**
  * @brief  OLED写命令（单条）
  */
void OLED_WriteCommand(uint8_t Command)
{
	if (!oled_ready) return;
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
	if (!oled_ready) return;
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

	if (!oled_ready) return;

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

	if (!oled_ready) return;

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
	if (!oled_ready) return;

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
  * @brief  探测面板在不在：START + 地址，看有没有 ACK
  * @retval 1=在线，0=没接/没应答
  * @note   必须在 OLED_I2C_Init() 之后调用；失败会顺手恢复一次总线
  */
static uint8_t OLED_Probe(void)
{
	uint16_t timeout = 2000u;

	I2C_GenerateSTART(OLED_I2C, ENABLE);
	while (I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_MODE_SELECT) == ERROR)
	{
		if (--timeout == 0u) goto fail;
	}

	I2C_Send7bitAddress(OLED_I2C, OLED_SLAVE_ADDR, I2C_Direction_Transmitter);

	timeout = 2000u;
	while (I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR)
	{
		if (--timeout == 0u) goto fail;
	}

	I2C_GenerateSTOP(OLED_I2C, ENABLE);
	return 1u;

fail:
	I2C_GenerateSTOP(OLED_I2C, ENABLE);
	OLED_I2C_Recover();
	return 0u;
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

	OLED_I2C_Init();			//硬件 I2C1（PB6=SCL / PB7=SDA）初始化
	/* TIM2 已给舵机 PWM 用，这里不再初始化它 */

	/*先探测面板在不在：没接就直接返回，后面所有绘图都会立刻返回，
	 *不会再一个个等 I2C 超时（否则上电要卡十几秒）*/
	oled_ready = OLED_Probe();
	if (!oled_ready) return;

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
  * @brief  整屏刷新一次（内含清屏效果）
  * @retval 固定返回 0：TIM2 已分配给舵机 PWM，这里不再做耗时测量
  */
uint32_t OLED_MeasureFullFrame(void)
{
	uint8_t page;

	if (!oled_ready) return 1u;					/*面板不在线：直接给个非 0，避免除零*/

	for (page = 0; page < 8; page++)
	{
		OLED_SetCursor(page, 0);				/*3 条命令一次事务*/
		OLED_WriteDataBurst(oled_blank, 128);	/*128 字节一次事务*/
	}

	return 0u;									/*无计时（TIM2 已给舵机）*/
}

/**
  * @brief  硬件 I2C 效率测试（已停用：TIM2 给了舵机 PWM）
  * @retval 固定返回 0
  *
  * 说明：原来用 TIM2 自由计数测全屏刷新耗时，TIM2 现在被三路舵机占用，
  *       所以这里只清屏、不再显示 us / KB/s / 占用率。
  */
uint32_t OLED_ShowI2CStats(void)
{
	if (!oled_ready) return 0u;					/*面板不在线：不做效率测试，立刻返回*/

	OLED_MeasureFullFrame();					/*只清屏，不测时间*/

	return 0u;
}
