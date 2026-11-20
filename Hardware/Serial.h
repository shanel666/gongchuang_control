#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

/* USART1_IRQHandler updates the receive buffer and flag asynchronously. */
extern uint8_t Serial_TxPacket[];
extern volatile uint8_t Serial_RxPacket[];
extern volatile uint8_t Serial_RxFlag;

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

void Serial_SendPacket(void);
uint8_t Serial_GetRxFlag(void);

void USART2_SendByte(uint8_t byte);
void USART2_Init(uint32_t bound);
/* Serial2.h */
extern volatile uint8_t uart_key_num;
void USART2_SendString(char *s);
void USART2_SendHex32(uint32_t val);
void USART2_SendDec(uint32_t val);
void USART2_SendInt(int32_t v);

/* USART3：陀螺仪角度文本输出 / 无线串口，使用默认映射 PB10(TX)/PB11(RX)。 */
void USART3_Init(uint32_t bound);
void USART3_SendByte(uint8_t byte);

/* USART3 接收（无线串口）：中断里按 \r 或 \n 收一行，最多 63 字节。
 * USART3_ReadLine() 取走一行（不含换行符），没有整行时返回 0。
 * 注意 PB11 同时是按键 2，接无线模块后不要再调用 Key_GetNum()。 */
extern volatile uint32_t USART3_RxCount;
uint8_t USART3_ReadLine(char *line, uint8_t size);

/* 通用发送：整数 / 小数 / 字符 / 字符串都能转换后经 USART3 发出。 */
void USART3_SendString(const char *s);
void USART3_SendInt(int32_t v);
void USART3_SendFloat(float v, uint8_t decimals);
void USART3_Printf(const char *format, ...);   /* 支持 %d %i %u %c %s %f(可带 %.Nf 精度) % */

/* 角度专用发送（0.01 度整数 -> "xxx.xx\r\n"），底层复用上述通用发送。 */
void USART3_SendAngleCentiDegrees(int32_t centi_degrees);

/* 自检：按顺序发送字符串/字符/整数/小数/角度，串口终端可核对输出。 */
void USART3_SelfTest(void);

/* ============================================================================
 * K230 视觉链路
 * ----------------------------------------------------------------------------
 * ★★★ 串口分配表（2026-11 现场最终版：四个外设各管一件事）★★★
 *
 *   串口     引脚            接什么                        谁来配
 *   -------- ---------------- ----------------------------- -------------------
 *   USART1   PA9 / PA10       六个步进驱动器（二进制总线）   固定，别动
 *   USART2   PA2 / PA3       JY901 陀螺仪（二进制帧）       固定，别动
 *   USART3   PB10 / PB11     ★ 无线调试 + 上位机命令 ★     SERIAL_USART3_ENABLE = 1
 *   UART4    PC10 / PC11     ★ K230（视觉）★               K230_PORT_UART4 / UART4_Init
 *
 *   ★★★ PC10/PC11 是 **UART4 自己的引脚**，不是 USART3 的重映射 ★★★
 *     UART4 是独立外设（挂在 APB1，中断号 52），和 USART3 没有任何关系，所以
 *     "无线模块接 PB10/PB11" 和 "K230 接 PC10/PC11" **可以同时成立**。
 *
 *     旧代码用 "USART3 部分重映射" 去抢 PC10/PC11，等于让 USART3 这个外设
 *     只能服务 K230，于是：① PB10/PB11 上的无线模块没了串口；② 调试只能挤到
 *     PA2/PA3 —— 而那里是陀螺仪。三处全错，现在统一改成 UART4。
 *
 *   ⚠️ PC10/PC11 不要做任何 GPIO_PinRemapConfig：UART4 是固定引脚。
 *   ⚠️ 必须用 startup_stm32f10x_hd.s + 宏 STM32F10X_HD，
 *      md 版启动文件里**没有** UART4_IRQHandler 的向量表项（IRQ 52），
 *      否则"编译通过、中断永远不进"，白折腾一轮。
 *
 * 【下行】命令字，一行一条，K230_SendLine() 发（自动补 \r\n）：
 *      "circle"  开始找圆心 → K230 持续回 C 帧
 *      "linear"  开始找直线 → K230 持续回 L 帧
 *      "stop"    停止回发
 *
 * 【上行】K230 回三种帧（解析在 Serial.c 的 K230_ParseLine，中断里跑）：
 *      C<x>,<y>,<r>   圆心像素坐标 + 半径      例 C318,242,41
 *      L<k>,<psi>     斜率×100 + 偏航 0.01°    例 L4,192  /  L-12,-345
 *      NG             这次没找到
 *
 * 每类帧有自己的序号，主循环用"序号变了"判断来了新帧；
 * 格式错的行只让 K230_BadLine 自增，不会覆盖上一帧好数据。
 * ========================================================================== */
#define K230_PORT_USART2   2
#define K230_PORT_USART3   3
#define K230_PORT_UART4    4                      /* ★ 新增：UART4 = PC10/PC11 */
#define K230_PORT          K230_PORT_UART4        /* ★ K230 接 PC10(TX)/PC11(RX) → UART4 */

/* UART4：PC10=TX / PC11=RX → K230（固定引脚，无需重映射；需要 STM32F10X_HD） */
void UART4_Init(uint32_t bound);

#define K230_CMD_CIRCLE   "circle"
#define K230_CMD_LINEAR   "linear"
#define K230_CMD_STOP     "stop"

/* C 帧：找圆心结果（seq 变了就是新帧） */
extern volatile int16_t  K230_Cx;
extern volatile int16_t  K230_Cy;
extern volatile int16_t  K230_R;
extern volatile uint32_t K230_CircleSeq;

/* L 帧：直线结果。k = 斜率×100；psi 单位 0.01°，正 = 顺时针转 */
extern volatile int16_t  K230_K100;
extern volatile int16_t  K230_PsiCenti;
extern volatile uint32_t K230_LineSeq;

/* NG 帧次数 / 被丢掉的行数 */
extern volatile uint32_t K230_NgSeq;
extern volatile uint32_t K230_BadLine;

/* 纯解析函数：认出一帧有效数据返回 1。不碰硬件，主机上可以直接测。 */
uint8_t K230_ParseLine(const char *line, uint8_t len);

void K230_SendLine(const char *s);   /* 发一整行文本（自动补 \r\n） */

/* ---------------- 旧的纯数字格式 "x,y"（find_circle 老脚本还在用） ---------------- */
extern volatile uint16_t K230_CircleX;
extern volatile uint16_t K230_CircleY;
extern volatile uint32_t K230_FrameCount;
uint8_t K230_GetCenter(uint16_t *x, uint16_t *y);

#endif
