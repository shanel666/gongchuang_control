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

/* USART3：陀螺仪角度文本输出，使用默认映射 PB10(TX)/PB11(RX)。 */
void USART3_Init(uint32_t bound);
void USART3_SendByte(uint8_t byte);

/* 通用发送：整数 / 小数 / 字符 / 字符串都能转换后经 USART3 发出。 */
void USART3_SendString(const char *s);
void USART3_SendInt(int32_t v);
void USART3_SendFloat(float v, uint8_t decimals);
void USART3_Printf(const char *format, ...);   /* 支持 %d %i %u %c %s %f(可带 %.Nf 精度) % */

/* 角度专用发送（0.01 度整数 -> "xxx.xx\r\n"），底层复用上述通用发送。 */
void USART3_SendAngleCentiDegrees(int32_t centi_degrees);

/* 自检：按顺序发送字符串/字符/整数/小数/角度，串口终端可核对输出。 */
void USART3_SelfTest(void);

/* ---------------- K230 圆心数据（USART2，ASCII "x,y\r\n"） ----------------
 * 中断里按行解析；只有格式和范围都正确的行才更新下面两个值和帧计数，
 * 所以 K230 断线时这里不会留下"过期但看着还有值"的假数据。
 * K230_GetCenter() 返回 1 表示至少收到过一帧有效数据。 */
extern volatile uint16_t K230_CircleX;
extern volatile uint16_t K230_CircleY;
extern volatile uint32_t K230_FrameCount;
uint8_t K230_GetCenter(uint16_t *x, uint16_t *y);

#endif
