#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

/* 硬件 I2C1：PB6=SCL，PB7=SDA，默认 400kHz */
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/* 硬件 I2C 效率测试：清屏 + 整屏刷新计时，返回耗时 us */
uint32_t OLED_MeasureFullFrame(void);

/* 测量全屏刷新并在第 3、4 行显示 us / KB/s / 总线占用率，返回耗时 us */
uint32_t OLED_ShowI2CStats(void);

#endif
