#include "stm32f10x.h"                  // Device header
#include "Gyro.h"
#include <stdio.h>
#include <stdarg.h>

uint8_t Serial_TxPacket[4];				//定义发送数据包数组，数据包格式：FF 01 02 03 04 FE
volatile uint8_t Serial_RxPacket[16];				//定义接收数据包数组
volatile uint8_t Serial_RxFlag;					//定义接收数据包标志位

/**
  * 函    数：串口初始化
  * 参    数：无
  * 返 回 值：无
  */
void Serial_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//开启USART1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//开启GPIOA的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA9引脚初始化为复用推挽输出
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA10引脚初始化为上拉输入
	
	/*USART初始化*/
	USART_InitTypeDef USART_InitStructure;					//定义结构体变量
	USART_InitStructure.USART_BaudRate = 115200;				//波特率
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制，不需要
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//模式，发送模式和接收模式均选择
	USART_InitStructure.USART_Parity = USART_Parity_No;		//奇偶校验，不需要
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	//停止位，选择1位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//字长，选择8位
	USART_Init(USART1, &USART_InitStructure);				//将结构体变量交给USART_Init，配置USART1
	
	/*中断输出配置*/
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);			//开启串口接收数据的中断
	
	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//配置NVIC为分组2
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;					//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		//选择配置NVIC的USART1线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		//指定NVIC线路的抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);							//将结构体变量交给NVIC_Init，配置NVIC外设
	
	/*USART使能*/
	USART_Cmd(USART1, ENABLE);								//使能USART1，串口开始运行
}

/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	//等待发送完成
	/*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}

/**
  * 函    数：串口发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		Serial_SendByte(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：串口发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)//遍历字符数组（字符串），遇到字符串结束标志位后停止
	{
		Serial_SendByte(String[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：次方函数（内部使用）
  * 返 回 值：返回值等于X的Y次方
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;	//设置结果初值为1
	while (Y --)			//执行Y次
	{
		Result *= X;		//将X累乘到结果
	}
	return Result;
}

/**
  * 函    数：串口发送数字
  * 参    数：Number 要发送的数字，范围：0~4294967295
  * 参    数：Length 要发送数字的长度，范围：0~10
  * 返 回 值：无
  */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)		//根据数字长度遍历数字的每一位
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');	//依次调用Serial_SendByte发送每位数字
	}
}

/**
  * 函    数：使用printf需要重定向的底层函数
  * 参    数：保持原始格式即可，无需变动
  * 返 回 值：保持原始格式即可，无需变动
  */
int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);			//将printf的底层重定向到自己的发送字节函数
	return ch;
}

/**
  * 函    数：自己封装的prinf函数
  * 参    数：format 格式化字符串
  * 参    数：... 可变的参数列表
  * 返 回 值：无
  */
void Serial_Printf(char *format, ...)
{
	char String[100];				//定义字符数组
	va_list arg;					//定义可变参数列表数据类型的变量arg
	va_start(arg, format);			//从format开始，接收参数列表到arg变量
	vsprintf(String, format, arg);	//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);					//结束变量arg
	Serial_SendString(String);		//串口发送字符数组（字符串）
}

/**
  * 函    数：串口发送数据包
  * 参    数：无
  * 返 回 值：无
  * 说    明：调用此函数后，Serial_TxPacket数组的内容将加上包头（FF）包尾（FE）后，作为数据包发送出去
  */
void Serial_SendPacket(void)
{
	Serial_SendByte(0xFF);
	Serial_SendArray(Serial_TxPacket, 4);
	Serial_SendByte(0xFE);
}

/**
  * 函    数：获取串口接收数据包标志位
  * 参    数：无
  * 返 回 值：串口接收数据包标志位，范围：0~1，接收到数据包后，标志位置1，读取后标志位自动清零
  */
uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)			//如果标志位为1
	{
		Serial_RxFlag = 0;
		return 1;					//则返回1，并自动清零标志位
	}
	return 0;						//如果标志位为0，则返回0
}

/**
  * 函    数：USART1中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  *           函数名为预留的指定名称，可以从启动文件复制
  *           请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  */
void USART1_IRQHandler(void)
{
    static uint8_t RxState = 0;
    static uint8_t pRxPacket = 0;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        uint8_t RxData = USART_ReceiveData(USART1);

        /* 状态机解析回包：帧头(地址 0x01~0x05) -> 3 字节数据 -> 包尾 0x6B
         * 回包格式：[地址][功能码][数据][校验 0x6B]
         * 0x3A 查询回包：[地址][0x3A][状态][0x6B]，其中状态 bit1 表示已到位
         * （底盘 1~4 用 0x03 = 使能+到位；物料电机地址 5 的一圈标定也靠这包判断到位） */
        if (RxState == 0)
        {
            if (RxData >= 0x01 && RxData <= 0x05)   /* 地址作为帧头 */
            {
                Serial_RxPacket[0] = RxData;
                pRxPacket = 1;
                RxState = 1;
            }
        }
        else if (RxState == 1)
        {
            Serial_RxPacket[pRxPacket++] = RxData;
            if (pRxPacket >= 4)                     /* 收够 4 字节 */
            {
                RxState = 0;
                if (Serial_RxPacket[3] == 0x6B)     /* 尾校验 */
                {
                    Serial_RxFlag = 1;
                }
            }
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
#define USART2_RX_BUF_SIZE  16
uint8_t Serial2_RxPacket[USART2_RX_BUF_SIZE];
volatile uint8_t Serial2_RxFlag = 0;
void USART2_Init(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 使能时钟：GPIOA 在 APB2，USART2 在 APB1 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* 注意：PA2 原来是 USART2_TX，中间让给过舵机；现在舵机在 PA0/PA1，
     * PA2 空出来了，这里恢复成"发送"——主控要靠它给 K230 发任务码。
     * ★ 你之前怀疑 PA2 这个脚损坏：发任务码如果 K230 收不到，先量 PA2 有没有波形，
     *   确认后可以把 K230 改接到 USART3(PB10/PB11) 上（见 SERIAL_USART3_ENABLE）。 */

    /* PA2 - USART2_TX 复用推挽输出 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA3 - USART2_RX 上拉输入（K230 的 TX / 陀螺仪的 TX 接这里） */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART2 参数配置 */
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;   /* 收 K230 数据 + 发任务码 */
    USART_Init(USART2, &USART_InitStructure);

    /* 开启 USART2 接收中断 */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    /* NVIC 配置 */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 使能 USART2 */
    USART_Cmd(USART2, ENABLE);
}

void USART2_SendByte(uint8_t byte)
{
    USART_SendData(USART2, byte);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}
/* ============================================================================
 * K230 文本帧解析（USART2 RX = PA3，ASCII，一行一帧，'\r' 或 '\n' 结尾）
 * ----------------------------------------------------------------------------
 * 【下行】主控 -> K230（USART2 TX = PA2，K230_SendLine() 发，自动补 \r\n）
 *      circle      开始找圆心，K230 持续回 C 帧
 *      linear      开始找直线，K230 持续回 L 帧
 *      stop        停止回发
 *
 * 【上行】K230 -> 主控，三种帧：
 *      C<x>,<y>,<r>      找圆心结果        例：C318,242,41
 *                        x,y = 圆心像素坐标（0~999），r = 半径像素
 *      L<k>,<psi>        直线结果          例：L4,192   L-12,-345
 *                        k   = 斜率 ×100（整数，4 就是 0.04）
 *                        psi = 需要的偏航修正角，单位 0.01°，
 *                              正 = 顺时针转（shun_yaw_ / dir=0x00）
 *                              负 = 逆时针转（ni_yaw_   / dir=0x01）
 *      NG                这次没找到（圆不够 / 斜率超门限）
 *
 *      另外保留旧格式 "<x>,<y>"（老的 find_circle 脚本还在用）。
 *
 * 规则：
 *   · 一行最多 15 字节，超长整行丢弃重新同步；
 *   · 白名单字符 = 数字 / 字母 / 逗号 / '+' / '-' / 空格，
 *     其它字节（比如陀螺仪的 0x55 帧头）一律丢掉整行；
 *   · 格式或范围不对整行丢弃，只让 K230_BadLine 自增，**不覆盖上一帧好数据**；
 *   · 每种帧有自己的序号（CircleSeq / LineSeq / NgSeq），主循环靠"序号变了"
 *     判断来了新数据，天然不怕旧帧。
 * ========================================================================== */
#define K230_LINE_MAX   16u

/* 旧格式（find_circle 老脚本）：圆心 */
volatile uint16_t K230_CircleX = 0;
volatile uint16_t K230_CircleY = 0;
volatile uint32_t K230_FrameCount = 0;

/* C 帧：找圆心结果 */
volatile int16_t  K230_Cx = 0;
volatile int16_t  K230_Cy = 0;
volatile int16_t  K230_R  = 0;
volatile uint32_t K230_CircleSeq = 0;

/* L 帧：直线结果 */
volatile int16_t  K230_K100 = 0;        /* 斜率 ×100 */
volatile int16_t  K230_PsiCenti = 0;    /* 偏航修正角，0.01° */
volatile uint32_t K230_LineSeq = 0;

/* NG 帧次数 / 丢弃的行数（调试用：只涨不跌说明格式对不上） */
volatile uint32_t K230_NgSeq = 0;
volatile uint32_t K230_BadLine = 0;

static char    k230_line[K230_LINE_MAX];
static uint8_t k230_len = 0;

/* ==== TEST-BLOCK-BEGIN：纯逻辑，主机上可以单独抽出来编译测试 ==== */

static const char *k230_p = 0;          /* 解析游标 */

static void k230_skip_space(void)
{
    while (*k230_p == ' ') k230_p++;
}

/* 读一个十进制整数（可带 +/-）。成功 1，失败 0。 */
static uint8_t k230_parse_int(int32_t *out)
{
    int32_t v = 0;
    uint8_t neg = 0u;
    uint8_t nd  = 0u;

    k230_skip_space();
    if (*k230_p == '-')      { neg = 1u; k230_p++; }
    else if (*k230_p == '+') { k230_p++; }

    while (*k230_p >= '0' && *k230_p <= '9')
    {
        v = v * 10 + (int32_t)(*k230_p - '0');
        k230_p++;
        if (++nd > 5u) return 0u;       /* 位数过多 */
    }
    if (nd == 0u) return 0u;            /* 一个数字都没有 */

    *out = neg ? -v : v;
    return 1u;
}

/* 期待一个分隔符（允许夹空格） */
static uint8_t k230_expect(char c)
{
    k230_skip_space();
    if (*k230_p != c) return 0u;
    k230_p++;
    return 1u;
}

static uint8_t k230_at_end(void)
{
    k230_skip_space();
    return (*k230_p == '\0') ? 1u : 0u;
}

/* 解析一整行（不含 \r\n）。返回 1 = 认出一帧有效数据；0 = 丢弃。
 * 这个函数只碰变量，不碰任何硬件寄存器，所以能在电脑上直接跑断言。 */
uint8_t K230_ParseLine(const char *line, uint8_t len)
{
    char    c;
    int32_t a, b, d;

    if ((line == 0) || (len == 0u)) return 0u;

    k230_p = line;
    k230_skip_space();

    c = *k230_p;
    if ((c >= 'a') && (c <= 'z')) c = (char)(c - 'a' + 'A');   /* 小写也认 */

    if (c == 'C')                                   /* C<x>,<y>,<r> */
    {
        k230_p++;
        if (!k230_parse_int(&a)) goto bad;
        if (!k230_expect(','))   goto bad;
        if (!k230_parse_int(&b)) goto bad;
        if (!k230_expect(','))   goto bad;
        if (!k230_parse_int(&d)) goto bad;
        if (!k230_at_end())      goto bad;
        if ((a < 0) || (a > 999) || (b < 0) || (b > 999) ||
            (d < 0) || (d > 999))
            goto bad;

        K230_Cx = (int16_t)a;
        K230_Cy = (int16_t)b;
        K230_R  = (int16_t)d;
        K230_CircleSeq++;
        return 1u;
    }

    if (c == 'L')                                   /* L<k>,<psi> */
    {
        k230_p++;
        if (!k230_parse_int(&a)) goto bad;
        if (!k230_expect(','))   goto bad;
        if (!k230_parse_int(&b)) goto bad;
        if (!k230_at_end())      goto bad;
        if ((a < -9999) || (a > 9999) ||
            (b < -18000) || (b > 18000))
            goto bad;

        K230_K100     = (int16_t)a;
        K230_PsiCenti = (int16_t)b;
        K230_LineSeq++;
        return 1u;
    }

    if (c == 'N')                                   /* NG */
    {
        k230_p++;
        c = *k230_p;
        if ((c >= 'a') && (c <= 'z')) c = (char)(c - 'a' + 'A');
        if (c == 'G')
        {
            k230_p++;
            if (k230_at_end())
            {
                K230_NgSeq++;
                return 1u;
            }
        }
        goto bad;
    }

    if ((c >= '0') && (c <= '9'))                   /* 旧格式 x,y */
    {
        if (!k230_parse_int(&a)) goto bad;
        if (!k230_expect(','))   goto bad;
        if (!k230_parse_int(&b)) goto bad;
        if (!k230_at_end())      goto bad;
        if ((a < 0) || (a > 999) || (b < 0) || (b > 999)) goto bad;

        K230_CircleX = (uint16_t)a;
        K230_CircleY = (uint16_t)b;
        K230_FrameCount++;
        return 1u;
    }

bad:
    K230_BadLine++;
    return 0u;
}

/* ==== TEST-BLOCK-END ==== */

/* 一行收完：交给解析器。失败只是丢这一行，已存的好数据不动。 */
static void K230_LineCommit(void)
{
    k230_line[k230_len] = '\0';
    (void)K230_ParseLine(k230_line, k230_len);
}

/* 逐字节喂行缓冲；只收合法文本字符，遇 \r/\n 结束一行。 */
static void K230_FeedByte(uint8_t ch)
{
    if ((ch == '\r') || (ch == '\n'))
    {
        if (k230_len != 0u) K230_LineCommit();
        k230_len = 0u;
    }
    else if (((ch >= '0') && (ch <= '9')) ||
             ((ch >= 'A') && (ch <= 'Z')) ||
             ((ch >= 'a') && (ch <= 'z')) ||
             (ch == ',') || (ch == '-') || (ch == '+') || (ch == ' '))
    {
        if (k230_len < (K230_LINE_MAX - 1u)) k230_line[k230_len++] = (char)ch;
        else k230_len = 0u;                             /* 超长，丢弃整行重新同步 */
    }
    else
    {
        k230_len = 0u;                                  /* 非文本字节，丢弃当前行 */
    }
}

uint8_t K230_GetCenter(uint16_t *x, uint16_t *y)
{
    if (K230_FrameCount == 0u) return 0u;

    if (x != 0) *x = K230_CircleX;
    if (y != 0) *y = K230_CircleY;
    return 1u;
}

/* USART2 模拟按键：收到 k:1 / k:2 / k:3 时置位 */
#include "motor.h"
void USART2_IRQHandler(void)
{
    static uint8_t state = 0;
    static uint8_t digit = 0;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
        uint8_t ch = USART_ReceiveData(USART2);

        /* USART2 同时接收电脑调试文本和 JY901 二进制帧；二进制数据交给
         * 陀螺仪解析器，不能像普通串口回显那样原样发回。 */
        Gyro_ProcessByte(ch);
        K230_FeedByte(ch);          /* K230 的 "x,y" 文本行，和上面的 k:数字 互不影响 */
        switch (state)
        {
            case 0:
                if (ch == 'k') state = 1;
                break;
            case 1:
                if (ch == ':') { state = 2; digit = 0; }
                else state = 0;
                break;
            case 2:
                if (ch >= '0' && ch <= '9')
                {
                    digit = digit * 10 + (ch - '0');
                }
                else if (ch == '\n' || ch == '\r')
                {
                    if (digit != 0) uart_key_num = digit;
                    state = 0;
                }
                else
                {
                    state = 0;   // 其他字符，放弃
                }
                break;
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}


void USART2_SendArray(uint8_t *array, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        USART2_SendByte(array[i]);
    }
}


void USART2_SendString(char *s)
{
    while (*s)
    {
        USART2_SendByte((uint8_t)*s++);
    }
}

void USART2_SendHex32(uint32_t val)
{
    const char hex[] = "0123456789ABCDEF";
    for (int i = 28; i >= 0; i -= 4)
    {
        USART2_SendByte(hex[(val >> i) & 0x0F]);
    }
}

void USART2_SendDec(uint32_t val)
{
    char buf[12];
    int i = 0;
    if (val == 0)
    {
        USART2_SendByte('0');
        return;
    }
    while (val > 0)
    {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    }
    while (i > 0) USART2_SendByte(buf[--i]);
}

void USART2_SendInt(int32_t v)
{
    if (v < 0)
    {
        USART2_SendByte('-');
        v = -v;
    }
    USART2_SendDec((uint32_t)v);
}

/* ============================================================================
 * 主控 -> K230：命令字（ASCII 文本行，走 USART2 TX = PA2，115200 8N1）
 * ----------------------------------------------------------------------------
 * 用文本而不是二进制，是为了 K230 那边用 Python 直接 readline() 解析，
 * 调试时也能在串口终端上一眼看出收到了什么。
 *
 * 现在只有三条（K230 那边收到什么就切什么模式）：
 *   circle   开始找圆心，K230 持续回 C<x>,<y>,<r>
 *   linear   开始找直线，K230 持续回 L<k>,<psi>
 *   stop     停止回发
 * 回帧格式和字段含义见本文件上方"K230 文本帧解析"那段注释。
 * ========================================================================== */

/* 发一整行自定义文本（结尾自动补 \r\n） */
void K230_SendLine(const char *s)
{
    while (*s != '\0') USART2_SendByte((uint8_t)*s++);
    USART2_SendByte('\r');
    USART2_SendByte('\n');
}

/* ---------------- USART3：无线串口（上位机 115200） ----------------
 * 默认映射 PB10(TX) / PB11(RX)。发送用于调试文本，接收按"一行一条命令"
 * 处理：\r 或 \n 结尾。中断里用双缓冲，主循环拿走 ready 缓冲区时中断写
 * 另一个缓冲区，避免边读边被覆盖。
 * 注意：PB10/PB11 和按键复用，接无线模块后 PB10/PB11 两个按键不要再用。 */
#define USART3_LINE_MAX   64u

volatile uint32_t USART3_RxCount = 0;              /* 完整行计数（1 = 有新行） */

static char             usart3_rx_buf[2][USART3_LINE_MAX];
static volatile uint8_t usart3_rx_active = 0;      /* 中断正在写哪个缓冲区 */
static volatile uint8_t usart3_rx_ready_idx = 0;   /* 已收完、等主循环取走的缓冲区 */
static volatile uint8_t usart3_rx_ready = 0;
static volatile uint8_t usart3_rx_len = 0;

/* USART3 开关（默认映射 TX=PB10 / RX=PB11；也可以完全重映射到 PC10/PC11）
 *
 *   0 = 关闭（当前）：PB10/PB11 接 OLED 的 SDA/SCL（见 OLED.c），
 *       USART3_Init() 什么都不做，USART3_SendXxx / ReadLine 全变空操作，
 *       主程序里原有的打印语句不用删，也不影响 OLED。
 *
 *   1 = 用默认引脚 PB10(TX) / PB11(RX) —— 此时 OLED 不能接在这两个脚上。
 *
 *   2 = USART3 完全重映射 → PC10(TX) / PC11(RX)，★推荐★
 *       OLED 继续用 PB10/PB11，互不冲突，等于白捡一路完整串口。
 *       PC10/PC11 在本工程里没有别的用途（PC13/14/15 是板载 LED）。
 *       接线：PC10 → USB-TTL 的 RX，PC11 ← USB-TTL 的 TX，GND 共地。 */
#define SERIAL_USART3_ENABLE   1

#if   SERIAL_USART3_ENABLE == 1
  #define U3_TX_PORT   GPIOB
  #define U3_TX_PIN    GPIO_Pin_10
  #define U3_RX_PORT   GPIOB
  #define U3_RX_PIN    GPIO_Pin_11
  #define U3_TX_CLK    RCC_APB2Periph_GPIOB
  #define U3_RX_CLK    RCC_APB2Periph_GPIOB
#elif SERIAL_USART3_ENABLE == 2
  #define U3_TX_PORT   GPIOC
  #define U3_TX_PIN    GPIO_Pin_10
  #define U3_RX_PORT   GPIOC
  #define U3_RX_PIN    GPIO_Pin_11
  #define U3_TX_CLK    RCC_APB2Periph_GPIOC
  #define U3_RX_CLK    RCC_APB2Periph_GPIOC
#endif

#if SERIAL_USART3_ENABLE != 0

void USART3_Init(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 使能 GPIO 时钟（模式 2 还要 AFIO 时钟做重映射） */
    RCC_APB2PeriphClockCmd(U3_TX_CLK | U3_RX_CLK
#if SERIAL_USART3_ENABLE == 2
                           | RCC_APB2Periph_AFIO
#endif
                           , ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

#if SERIAL_USART3_ENABLE == 2
    /* USART3 完全重映射：TX -> PC10，RX -> PC11 */
    GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);
#endif

    /* TX：复用推挽 */
    GPIO_InitStructure.GPIO_Pin = U3_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(U3_TX_PORT, &GPIO_InitStructure);

    /* RX：上拉输入，空闲为高（对端模块的 TX 接这里） */
    GPIO_InitStructure.GPIO_Pin = U3_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(U3_RX_PORT, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    /* 接收中断：一行一条命令（无线串口） */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART3, ENABLE);
}

#else   /* SERIAL_USART3_ENABLE == 0：PB10/PB11 让给 OLED，串口全部变空操作 */

void USART3_Init(uint32_t bound)
{
    (void)bound;      /* 不配置 PB10/PB11，把这两个脚完整留给 OLED 的 I2C */
}

#endif  /* SERIAL_USART3_ENABLE */

/* 取走一行命令（不含 \r\n），有整行返回 1；没有返回 0。
 * 复制期间关 RXNE 中断，避免中断正好把同一缓冲区换回来写。 */
uint8_t USART3_ReadLine(char *line, uint8_t size)
{
    uint8_t i = 0;
    uint8_t idx;

    if (size == 0u) return 0u;

    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);

    if (!usart3_rx_ready)
    {
        USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
        return 0u;
    }

    idx = usart3_rx_ready_idx;
    while (usart3_rx_buf[idx][i] != '\0' && (uint8_t)(i + 1u) < size)
    {
        line[i] = usart3_rx_buf[idx][i];
        i++;
    }
    line[i] = '\0';
    usart3_rx_ready = 0u;

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    return 1u;
}

/* USART3 接收中断：按 \r 或 \n 分帧。缓冲区满了就丢整行重新同步。 */
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
    {
        uint8_t ch = USART_ReceiveData(USART3);

        if (ch == '\r' || ch == '\n')
        {
            if (usart3_rx_len != 0u)
            {
                usart3_rx_buf[usart3_rx_active][usart3_rx_len] = '\0';
                usart3_rx_ready_idx = usart3_rx_active;
                usart3_rx_ready = 1u;
                usart3_rx_active ^= 1u;          /* 换另一个缓冲区继续收 */
                usart3_rx_len = 0u;
                USART3_RxCount++;
            }
        }
        else if (usart3_rx_len < (USART3_LINE_MAX - 1u))
        {
            usart3_rx_buf[usart3_rx_active][usart3_rx_len++] = (char)ch;
        }
        else
        {
            usart3_rx_len = 0u;                  /* 超长，丢弃整行 */
        }

        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}

void USART3_SendByte(uint8_t byte)
{
#if SERIAL_USART3_ENABLE
    USART_SendData(USART3, byte);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
#else
    /* 串口已关闭（PB10/PB11 给了 OLED）：直接返回，避免等一个永远不会置位的 TXE */
    (void)byte;
#endif
}

static void USART3_SendUnsigned(uint32_t value)
{
    char digits[10];
    uint8_t count = 0;

    if (value == 0u)
    {
        USART3_SendByte('0');
        return;
    }

    while (value != 0u)
    {
        digits[count++] = (char)('0' + (value % 10u));
        value /= 10u;
    }
    while (count != 0u)
        USART3_SendByte((uint8_t)digits[--count]);
}

void USART3_SendString(const char *s)
{
    while (*s)
        USART3_SendByte((uint8_t)*s++);
}

void USART3_SendInt(int32_t v)
{
    uint32_t u;
    if (v < 0)
    {
        USART3_SendByte('-');
        u = (uint32_t)(~v) + 1u;        /* 两补码取绝对值，INT32_MIN 也安全 */
    }
    else
    {
        u = (uint32_t)v;
    }
    USART3_SendUnsigned(u);
}

void USART3_SendFloat(float v, uint8_t decimals)
{
    uint32_t scale = 1u;
    uint32_t fpart;
    int32_t ipart;
    uint8_t i;

    if (decimals > 6u) decimals = 6u;
    for (i = 0; i < decimals; i++) scale *= 10u;

    if (v < 0.0f)
    {
        USART3_SendByte('-');
        v = -v;
    }

    ipart = (int32_t)v;
    fpart = (uint32_t)((v - (float)ipart) * (float)scale + 0.5f);
    if (fpart >= scale)                 /* 四舍五入进位 */
    {
        fpart = 0u;
        ipart++;
    }

    USART3_SendInt(ipart);
    if (decimals != 0u)
    {
        USART3_SendByte('.');
        for (i = 0; i < decimals; i++)
        {
            scale /= 10u;
            USART3_SendByte((uint8_t)('0' + (fpart / scale) % 10u));
        }
    }
}

/* 通用格式化发送：整数 / 小数 / 字符 / 字符串都能直接转换后经 USART3 发出。
 * 支持 %d %i %u %c %s %f（可带 %.Nf 精度，默认 2 位）以及 %%。 */
void USART3_Printf(const char *format, ...)
{
    va_list ap;
    const char *p;
    uint8_t prec;

    va_start(ap, format);
    for (p = format; *p != '\0'; p++)
    {
        if (*p != '%')
        {
            USART3_SendByte((uint8_t)*p);
            continue;
        }

        p++;
        prec = 2u;
        if (*p == '.')
        {
            p++;
            prec = 0u;
            while (*p >= '0' && *p <= '9')
            {
                prec = (uint8_t)(prec * 10u + (*p - '0'));
                p++;
            }
            if (prec > 6u) prec = 6u;
        }

        switch (*p)
        {
            case 'd':
            case 'i': USART3_SendInt((int32_t)va_arg(ap, int)); break;
            case 'u': USART3_SendUnsigned((uint32_t)va_arg(ap, unsigned int)); break;
            case 'c': USART3_SendByte((uint8_t)va_arg(ap, int)); break;
            case 's': USART3_SendString(va_arg(ap, const char *)); break;
            case 'f': USART3_SendFloat((float)va_arg(ap, double), prec); break;
            case '%': USART3_SendByte('%'); break;
            default:
                USART3_SendByte('%');
                USART3_SendByte((uint8_t)*p);
                break;
        }
    }
    va_end(ap);
}

/* 角度专用发送：0.01 度整数 -> "xxx.xx\r\n"，底层复用 USART3_SendUnsigned。 */
void USART3_SendAngleCentiDegrees(int32_t centi_degrees)
{
    uint32_t magnitude;

    if (centi_degrees < 0)
    {
        USART3_SendByte('-');
        magnitude = (uint32_t)(-(centi_degrees + 1)) + 1u;
    }
    else
    {
        magnitude = (uint32_t)centi_degrees;
    }

    USART3_SendUnsigned(magnitude / 100u);
    USART3_SendByte('.');
    USART3_SendByte((uint8_t)('0' + ((magnitude / 10u) % 10u)));
    USART3_SendByte((uint8_t)('0' + (magnitude % 10u)));
    USART3_SendByte('\r');
    USART3_SendByte('\n');
}
#include "Delay.h"
/* 自检：按顺序发送字符串 / 字符 / 整数 / 小数 / 角度，供串口终端核对。 */
void USART3_SelfTest(void)
{
    USART3_Printf("--- USART3 test ---\r\n");
	Delay_ms(500);
    USART3_Printf("str   = %s\r\n", "Hello");
	Delay_ms(500);
    USART3_Printf("char  = %c\r\n", 'A');
	Delay_ms(500);
    USART3_Printf("int   = %d\r\n", -12345);
	Delay_ms(500);
    USART3_Printf("uint  = %u\r\n", 123456);
	Delay_ms(500);
    USART3_Printf("float = %.2f\r\n", 12.34);
	Delay_ms(500);
    USART3_Printf("float = %.3f\r\n", -0.125);
	Delay_ms(500);
    USART3_Printf("pct   = 100%%\r\n");
	Delay_ms(500);
    USART3_Printf("angle = ");
	Delay_ms(500);
    USART3_SendAngleCentiDegrees(12345);
	Delay_ms(500);
    USART3_SendAngleCentiDegrees(-5);
    USART3_Printf("done\r\n");
}
