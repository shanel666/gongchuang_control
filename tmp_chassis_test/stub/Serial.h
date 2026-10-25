#ifndef STUB_SERIAL_H
#define STUB_SERIAL_H

#include <stdint.h>

/* PC 端测试桩：只保留 chassis.c 用到的发送函数 */
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);

#endif
